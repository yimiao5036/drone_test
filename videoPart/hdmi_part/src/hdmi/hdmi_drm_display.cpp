/**
 * @file hdmi_drm_display.cpp
 * @brief HDMI 直显的 DRM/KMS + GBM + EGL/GLES 实现（香橙派实机路径）
 *
 * 输出链路：
 *   NV12 帧 ──► GLES3 双纹理（R8 Y / RG8 UV）──► YUV→RGB 着色器 ──► GBM 缓冲
 *          ──► drmModeAddFB2 ──► drmModePageFlip ──► VOP2 ──► HDMI 输出
 *
 * 关键设计取舍：
 * - **不编码**：标注帧已是 NV12，GPU 只做 YUV→RGB 与缩放到 GBM 扫描缓冲，
 *   省掉 H.264 编码/解码与 RTSP，端到端延迟更低。
 * - **仅用 GLES3**：NV12 的 Y/UV 平面行 stride 可能大于有效宽（FrameHandle 的
 *   hor_stride），需用 GL_UNPACK_ROW_LENGTH 让驱动按 stride 跨行读取。GLES2 的
 *   GL_UNPACK_ROW_LENGTH 在部分驱动上不生效，会花屏，故不提供 ES2 回退，
 *   而是明确失败并打印诊断。RK3588 的 Mali-G610 原生支持 GLES 3.2。
 * - **时序优先**：Open 完成 modeset 并点黑场，HDMI 信号在 Start 返回时即已锁定，
 *   HDMI-in 图传发射机不会反复重锁。
 * - **绝不阻塞**：DRM fd 设为非阻塞，翻转事件用 poll(timeout=0) 收割；上一次翻转
 *   未完成时本次 Submit 直接返回 kBusy 丢帧，与 VideoSender"拥塞只丢图传帧"一致。
 * - **framebuffer 一帧滞后回收**：翻转确认后回收"上一个"已上屏的 framebuffer，
 *   任何时刻至多 2~3 个 fb 存活；Close 阶段不回收当前 fb，使画面（或黑场）保持到
 *   进程退出，避免图传发射机瞬间失锁。
 * - **模式选择三级回退**：EDID 精确匹配（宽高一致 + 刷新率 ±2Hz）→ EDID 同分辨率
 *   任意刷新率（strict_mode 关闭时，锁定优先于刷新率）→ 按 CEA-861 消隐参数合成
 *   目标刷新率变体。三级都会打印实际采用的模式，便于上板排查。
 *
 * 依赖：libdrm、libgbm、libEGL、libGLESv2（见同目录 CMakeLists.txt）。
 * 权限：需要 root 或 CAP_SYS_ADMIN，且无其他 DRM master（无桌面环境）。
 */
#ifndef DRONE_HAVE_HDMI_KMS

#include "video_transmission/hdmi/hdmi_display_backend.h"

#include <memory>
#include <string>

namespace drone::video_transmission {

// 未编译 DRM 支持（开发机缺 libdrm/libgbm/EGL）时的占位实现：
// 保持符号存在，使调用方无需条件编译；CreateHdmiDisplay() 走占位显示器。

std::unique_ptr<IHdmiDisplay> CreateHdmiDrmDisplay() { return nullptr; }

std::string DescribeHdmiDrmResources(const std::string& device,
                                     std::uint32_t /*target_width*/,
                                     std::uint32_t /*target_height*/,
                                     int /*target_refresh_hz*/) {
    return "本构建未启用 DRM/KMS 支持（缺少 libdrm/libgbm/libEGL），无法枚举 " +
           device + "。请在香橙派上安装开发包后重新构建。";
}

}  // namespace drone::video_transmission

#else  // DRONE_HAVE_HDMI_KMS

#include "video_transmission/hdmi/hdmi_display_backend.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

// EGL 原生句柄按 GBM 使用（无 X11/Wayland 平台），须在 EGL 头文件前定义。
#ifndef EGL_NO_X11
#define EGL_NO_X11
#endif
#ifndef MESA_EGL_NO_X11_HEADERS
#define MESA_EGL_NO_X11_HEADERS
#endif

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <xf86drm.h>
#include <xf86drmMode.h>

#include <gbm.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>

// 兼容未定义这两个扩展常量的 EGL 头文件（取值来自 EGL 扩展注册表）
#ifndef EGL_PLATFORM_GBM_KHR
#define EGL_PLATFORM_GBM_KHR 0x31D7
#endif
#ifndef EGL_OPENGL_ES3_BIT_KHR
#define EGL_OPENGL_ES3_BIT_KHR 0x0040
#endif

#include <spdlog/spdlog.h>

namespace drone::video_transmission {

namespace {

using Clock = std::chrono::steady_clock;

/// 两个时间点之间的毫秒数。
double ElapsedMs(Clock::time_point begin, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

/// DRM 资源 RAII 包装，避免各失败分支漏掉 Free。
using DrmResPtr = std::unique_ptr<drmModeRes, decltype(&drmModeFreeResources)>;
using DrmConnectorPtr =
    std::unique_ptr<drmModeConnector, decltype(&drmModeFreeConnector)>;
using DrmEncoderPtr = std::unique_ptr<drmModeEncoder, decltype(&drmModeFreeEncoder)>;
using DrmCrtcPtr = std::unique_ptr<drmModeCrtc, decltype(&drmModeFreeCrtc)>;

/// 连接器类型名。不用 drmModeGetConnectorTypeName 以兼容较旧的 libdrm。
const char* ConnectorTypeName(std::uint32_t type) {
    switch (type) {
        case DRM_MODE_CONNECTOR_VGA:
            return "VGA";
        case DRM_MODE_CONNECTOR_DVII:
            return "DVI-I";
        case DRM_MODE_CONNECTOR_DVID:
            return "DVI-D";
        case DRM_MODE_CONNECTOR_DVIA:
            return "DVI-A";
        case DRM_MODE_CONNECTOR_Composite:
            return "Composite";
        case DRM_MODE_CONNECTOR_LVDS:
            return "LVDS";
        case DRM_MODE_CONNECTOR_Component:
            return "Component";
        case DRM_MODE_CONNECTOR_9PinDIN:
            return "DIN";
        case DRM_MODE_CONNECTOR_DisplayPort:
            return "DP";
        case DRM_MODE_CONNECTOR_HDMIA:
            return "HDMI-A";
        case DRM_MODE_CONNECTOR_HDMIB:
            return "HDMI-B";
        case DRM_MODE_CONNECTOR_TV:
            return "TV";
        case DRM_MODE_CONNECTOR_eDP:
            return "eDP";
        case DRM_MODE_CONNECTOR_VIRTUAL:
            return "Virtual";
        case DRM_MODE_CONNECTOR_DSI:
            return "DSI";
        case DRM_MODE_CONNECTOR_DPI:
            return "DPI";
        default:
            return "Unknown";
    }
}

/// 连接器名称，形如 "HDMI-A-1"。
std::string ConnectorName(const drmModeConnector* connector) {
    return std::string(ConnectorTypeName(connector->connector_type)) + "-" +
           std::to_string(connector->connector_type_id);
}

/// 为常见 errno 补充可操作的中文提示，便于上板定位。
std::string DescribeDrmError(int errnum) {
    std::string text = std::strerror(errnum);
    switch (errnum) {
        case EACCES:
        case EPERM:
            text += "（需要 root 或 CAP_SYS_ADMIN，并确保没有其他 DRM master/桌面占用）";
            break;
        case EINVAL:
            text += "（模式或参数不被驱动接受，检查分辨率/刷新率是否被 EDID 支持）";
            break;
        case ENOSPC:
            text += "（显示带宽不足，尝试降低分辨率或刷新率）";
            break;
        case ENODEV:
            text += "（设备在运行中被移除）";
            break;
        default:
            break;
    }
    return text;
}

/// 按 CEA-861 消隐参数合成目标刷新率的模式。
/// 以 720p60 时序为基准（clock 74.25MHz、htotal 1650、vtotal 750），
/// 按刷新率比例缩放像素时钟即可得到 30Hz / 15Hz 变体，消隐区保持不变。
/// 仅在 EDID 未提供目标分辨率时使用——图传发射机可能不锁定，需实测确认。
bool SynthesizeMode(std::uint32_t width, std::uint32_t height, int refresh_hz,
                    drmModeModeInfo* out) {
    if (width != 1280 || height != 720) {
        return false;  // 目前只覆盖项目实际使用的 720p 档
    }
    constexpr int kPixelClock60Khz = 74250;
    const int divisor = refresh_hz >= 60 ? 1 : (refresh_hz >= 30 ? 2 : 4);
    drmModeModeInfo mode{};
    mode.clock = kPixelClock60Khz / divisor;
    mode.hdisplay = 1280;
    mode.hsync_start = 1390;
    mode.hsync_end = 1430;
    mode.htotal = 1650;
    mode.vdisplay = 720;
    mode.vsync_start = 725;
    mode.vsync_end = 730;
    mode.vtotal = 750;
    const double refresh =
        static_cast<double>(mode.clock) * 1000.0 /
        (static_cast<double>(mode.htotal) * static_cast<double>(mode.vtotal));
    mode.vrefresh = static_cast<int>(std::lround(refresh));
    mode.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC;
    mode.type = DRM_MODE_TYPE_DRIVER;
    std::snprintf(mode.name, sizeof(mode.name), "1280x720@%d(合成)", mode.vrefresh);
    *out = mode;
    return true;
}

/// 输出模式选择结果，用于日志区分实际采用的回退级别。
enum class ModePickSource {
    kEdidExact = 0,   ///< EDID 精确匹配（宽高一致 + 刷新率 ±2Hz）
    kEdidFallback,    ///< EDID 同分辨率但刷新率不同（strict_mode 关闭）
    kSynthesized,     ///< 按 CEA-861 合成
};

/// 三级模式选择。失败返回 false（EDID 无该分辨率且无法合成）。
bool PickOutputMode(drmModeConnector* connector, const HdmiDisplayConfig& config,
                    drmModeModeInfo* out, ModePickSource* source) {
    int best_index = -1;
    int best_delta = 0;
    for (int i = 0; i < connector->count_modes; ++i) {
        const drmModeModeInfo& mode = connector->modes[i];
        if (mode.hdisplay != config.width || mode.vdisplay != config.height) {
            continue;
        }
        const int delta = std::abs(mode.vrefresh - config.refresh_hz);
        if (best_index < 0 || delta < best_delta) {
            best_index = i;
            best_delta = delta;
        }
    }
    if (best_index >= 0 && best_delta <= 2) {
        *out = connector->modes[best_index];
        *source = ModePickSource::kEdidExact;
        return true;
    }
    if (best_index >= 0 && !config.strict_mode) {
        *out = connector->modes[best_index];
        *source = ModePickSource::kEdidFallback;
        return true;
    }
    if (SynthesizeMode(config.width, config.height, config.refresh_hz, out)) {
        *source = ModePickSource::kSynthesized;
        return true;
    }
    return false;
}

/// 顶点着色器：单个覆盖全屏的大三角形，UV 在 Y 方向翻转以匹配图像行序
/// （GL 纹理原点在左下，视频帧第 0 行在顶部）。
constexpr const char* kVertexShader = R"(#version 300 es
layout(location = 0) in vec2 a_pos;
out vec2 v_uv;
void main() {
    v_uv = vec2(a_pos.x * 0.5 + 0.5, 0.5 - a_pos.y * 0.5);
    gl_Position = vec4(a_pos, 0.0, 1.0);
}
)";

/// 片元着色器：Y/UV 双纹理 → 去偏置与缩放 → 3x3 色彩矩阵 → RGB。
constexpr const char* kFragmentShader = R"(#version 300 es
precision mediump float;
in vec2 v_uv;
uniform sampler2D u_tex_y;
uniform sampler2D u_tex_uv;
uniform mat3 u_color_matrix;
uniform vec3 u_yuv_offset;
uniform vec3 u_yuv_scale;
out vec4 frag_color;
void main() {
    float y = texture(u_tex_y, v_uv).r;
    vec2 uv = texture(u_tex_uv, v_uv).rg;
    vec3 yuv = (vec3(y, uv.x, uv.y) - u_yuv_offset) * u_yuv_scale;
    frag_color = vec4(clamp(u_color_matrix * yuv, 0.0, 1.0), 1.0);
}
)";

/// 覆盖全屏的大三角形（NDC 顶点）。
constexpr float kFullscreenTriangle[6] = {-1.0f, -1.0f, 3.0f, -1.0f, -1.0f, 3.0f};

/// YUV→RGB 矩阵（列主序，直接喂给 glUniformMatrix3fv），输入为已归一化的 YUV。
/// 有限范围的去偏置与缩放由 u_yuv_offset/u_yuv_scale 完成，矩阵本身只做矩阵变换。
constexpr float kColorMatrixBt601[9] = {
    1.0f,      1.0f,      1.0f,        // 第 1 列
    0.0f,      -0.344136f, 1.772f,     // 第 2 列
    1.402f,    -0.714136f, 0.0f,       // 第 3 列
};
constexpr float kColorMatrixBt709[9] = {
    1.0f,   1.0f,      1.0f,       // 第 1 列
    0.0f,   -0.187324f, 1.8556f,   // 第 2 列
    1.5748f, -0.468124f, 0.0f,     // 第 3 列
};

/// HDMI 直显的 DRM/KMS + GBM + EGL/GLES 实现。
///
/// 所有方法只在 VideoSender 的消费线程被调用，内部不额外加锁。
class HdmiDrmDisplay final : public IHdmiDisplay {
public:
    HdmiDrmDisplay() = default;
    ~HdmiDrmDisplay() override { Close(); }

    HdmiDrmDisplay(const HdmiDrmDisplay&) = delete;
    HdmiDrmDisplay& operator=(const HdmiDrmDisplay&) = delete;

    // ---- 生命周期 ----

    bool Open(const HdmiDisplayConfig& config) override {
        if (open_) {
            return true;  // 幂等
        }
        if (active_instance_ != nullptr && active_instance_ != this) {
            SPDLOG_WARN(
                "已存在另一个活动的 HDMI 输出实例；翻转事件按实例指针派发，"
                "同一进程内不要同时启用两路 HDMI 输出");
        }
        config_ = config;
        // 任一步失败都由 Close 统一回滚，避免 DRM master、GBM 或 EGL 资源残留。
        if (!OpenDrm() || !SetupGbm() || !SetupEgl() || !SetupGl() ||
            !ModesetBlack()) {
            Close();
            return false;
        }
        open_ = true;
        active_instance_ = this;
        SPDLOG_INFO(
            "HDMI 显示设备就绪: 设备={} 连接器={} CRTC={} 模式={} 色彩空间={} 范围={} "
            "EGL={} GL={} 渲染器={}",
            config_.drm_device, ConnectorNameOf(connector_id_), crtc_id_, mode_name_,
            config_.color_space == HdmiColorSpace::kBt601 ? "BT.601" : "BT.709",
            config_.full_range ? "全范围" : "有限范围", egl_version_, gl_version_,
            gl_renderer_);
        return true;
    }

    void Close() override {
        if (drm_fd_ < 0 && gbm_device_ == nullptr &&
            egl_display_ == EGL_NO_DISPLAY) {
            return;  // 已关闭
        }
        // 仅在会话完整建立时做画面收尾；部分失败的回滚不做任何显示操作。
        if (open_) {
            WaitForPendingFlip(static_cast<std::uint32_t>(config_.flip_timeout_ms));
            if (config_.blank_on_stop) {
                // 点黑场并保持 CRTC 使能：图传发射机停在稳定黑场而非冻结画面。
                if (!PresentBlackAndWait()) {
                    SPDLOG_WARN("HDMI 停止时点黑场失败，画面可能停在最后一帧");
                }
            } else if (saved_crtc_ != nullptr) {
                const int ret = drmModeSetCrtc(
                    drm_fd_, saved_crtc_->crtc_id, saved_crtc_->buffer_id,
                    saved_crtc_->x, saved_crtc_->y, &saved_connector_id_, 1,
                    &saved_crtc_->mode);
                if (ret != 0) {
                    SPDLOG_WARN("HDMI 恢复原 CRTC 失败: {}", DescribeDrmError(errno));
                }
            }
            open_ = false;
        }
        // 刻意不回收当前 framebuffer：让黑场/最后一帧保持到进程退出，
        // 避免图传发射机在收尾瞬间失锁。fd 关闭时内核会统一释放。
        TeardownGl();
        TeardownEgl();
        TeardownGbm();
        TeardownDrm();
        mode_name_.clear();
        SPDLOG_INFO("HDMI 显示设备关闭: 累计上屏 {} 帧, 丢弃 {} 帧, 非法帧 {} 次",
                    presented_count_, skipped_count_, invalid_count_);
    }

    bool IsOpen() const override { return open_; }

    // ---- 帧输入 ----

    HdmiSubmitResult Submit(const std::uint8_t* data, std::uint32_t width,
                            std::uint32_t height,
                            std::uint32_t hor_stride) override {
        if (!open_) {
            return HdmiSubmitResult::kFailed;
        }
        // 后端已先行校验，这里兜底，避免把非法参数带进 GL。
        if (data == nullptr || width == 0 || height == 0 || hor_stride < width ||
            width != config_.width || height != config_.height) {
            ++invalid_count_;
            return HdmiSubmitResult::kFailed;
        }

        // 1) 收割可能已完成的翻转；仍挂起则本帧直接丢弃，绝不等待垂直同步。
        const auto reap_begin = Clock::now();
        ReapFlipEvents();
        const double reap_ms = ElapsedMs(reap_begin, Clock::now());
        if (flip_pending_) {
            const auto waited_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    Clock::now() - flip_submit_time_)
                    .count();
            if (waited_ms > config_.flip_timeout_ms) {
                // 翻转长时间不完成：判定设备异常，复位状态让后续帧可以重试，
                // 避免一次异常把整条输出链永久卡死。
                SPDLOG_ERROR("HDMI 页面翻转超时 {}ms（阈值 {}ms），复位翻转状态",
                             waited_ms, config_.flip_timeout_ms);
                RecyclePrevious();
                flip_pending_ = false;
                return HdmiSubmitResult::kFailed;
            }
            ++skipped_count_;
            return HdmiSubmitResult::kBusy;
        }

        // 2) 上传 NV12 并绘制到 GBM 后端缓冲。
        const auto prepare_begin = Clock::now();
        UploadNv12(data, width, height, hor_stride);
        DrawFullscreen();
        if (eglSwapBuffers(egl_display_, egl_surface_) != EGL_TRUE) {
            SPDLOG_ERROR("HDMI eglSwapBuffers 失败: 0x{:x}", eglGetError());
            return HdmiSubmitResult::kFailed;
        }
        prepare_latency_.Add(ElapsedMs(prepare_begin, Clock::now()));

        // 3) 提交页面翻转。
        const auto flip_begin = Clock::now();
        const bool submitted = SubmitPageFlip();
        flip_latency_.Add(reap_ms + ElapsedMs(flip_begin, Clock::now()));
        if (!submitted) {
            return HdmiSubmitResult::kFailed;
        }
        ++presented_count_;
        ++frame_index_;
        if (config_.log_every_n_frames > 0 &&
            frame_index_ % static_cast<std::uint64_t>(config_.log_every_n_frames) == 0) {
            SPDLOG_INFO("HDMI 上屏进度: {} 帧, 丢弃 {} 帧", frame_index_,
                        skipped_count_);
        }
        return HdmiSubmitResult::kSubmitted;
    }

    // ---- 状态查询 ----

    common::LatencySummary FlipLatency() const override {
        return flip_latency_.Snapshot();
    }

    common::LatencySummary PrepareLatency() const override {
        return prepare_latency_.Snapshot();
    }

    std::uint64_t SkippedFrameCount() const override { return skipped_count_; }

    std::string ActiveModeName() const override { return mode_name_; }

private:
    // ---- DRM 打开与模式选择 ----

    bool OpenDrm() {
        drm_fd_ = ::open(config_.drm_device.c_str(), O_RDWR | O_CLOEXEC);
        if (drm_fd_ < 0) {
            SPDLOG_ERROR("HDMI 打开 DRM 设备失败: {} ({})", config_.drm_device,
                         DescribeDrmError(errno));
            drm_fd_ = -1;
            return false;
        }
        // 非阻塞：翻转事件靠 poll(timeout=0) 收割，任何路径都不能阻塞消费线程。
        const int flags = ::fcntl(drm_fd_, F_GETFL, 0);
        if (flags >= 0) {
            ::fcntl(drm_fd_, F_SETFL, flags | O_NONBLOCK);
        }
        // open 时若无其他 master 即已持有 master，此处显式获取仅在必要时生效。
        // 失败不视为致命：真正的判据是后续 drmModeSetCrtc 是否被拒。
        if (drmSetMaster(drm_fd_) != 0) {
            SPDLOG_WARN("HDMI 显式获取 DRM master 失败: {}（若无桌面环境通常可忽略，"
                        "modeset 失败时会给出明确提示）",
                        DescribeDrmError(errno));
        }

        DrmResPtr resources(drmModeGetResources(drm_fd_), drmModeFreeResources);
        if (resources == nullptr) {
            SPDLOG_ERROR("HDMI 读取 DRM 资源失败: {}", DescribeDrmError(errno));
            return false;
        }

        DrmConnectorPtr connector = PickConnector(resources.get());
        if (connector == nullptr) {
            return false;
        }
        connector_id_ = connector->connector_id;

        if (!PickCrtc(resources.get(), connector.get())) {
            return false;
        }
        // 先记录进入前的 CRTC 状态，供 blank_on_stop=false 时恢复。
        saved_connector_id_ = connector_id_;
        saved_crtc_.reset(drmModeGetCrtc(drm_fd_, crtc_id_));

        ModePickSource source = ModePickSource::kEdidExact;
        if (!PickOutputMode(connector.get(), config_, &mode_, &source)) {
            SPDLOG_ERROR(
                "HDMI 无可用输出模式: 目标 {}x{}@{}Hz，连接器 {} 仅支持 {} 种模式"
                "（用 `hdmi_probe modes` 查看完整列表；非 720p 目标不支持合成）",
                config_.width, config_.height, config_.refresh_hz,
                ConnectorName(connector.get()), connector->count_modes);
            return false;
        }
        mode_name_ = mode_.name;
        switch (source) {
            case ModePickSource::kEdidExact:
                SPDLOG_INFO("HDMI 输出模式(EDID 精确匹配): {} {}x{}@{}Hz",
                            mode_name_, mode_.hdisplay, mode_.vdisplay,
                            mode_.vrefresh);
                break;
            case ModePickSource::kEdidFallback:
                SPDLOG_WARN(
                    "HDMI EDID 未提供 {}Hz 模式，回退到 {}（{}Hz）：锁定优先于刷新率",
                    config_.refresh_hz, mode_name_, mode_.vrefresh);
                break;
            case ModePickSource::kSynthesized:
                SPDLOG_WARN(
                    "HDMI EDID 未提供 {}x{} 模式，改用合成时序 {}：图传发射机可能不锁定，"
                    "需实测确认，必要时改用 EDID 支持的档位",
                    config_.width, config_.height, mode_name_);
                break;
        }
        return true;
    }

    /// 选择输出连接器：指定名称时按名称匹配，否则取第一个已连接且带模式的。
    DrmConnectorPtr PickConnector(drmModeRes* resources) {
        DrmConnectorPtr fallback(nullptr, drmModeFreeConnector);
        for (int i = 0; i < resources->count_connectors; ++i) {
            DrmConnectorPtr connector(
                drmModeGetConnector(drm_fd_, resources->connectors[i]),
                drmModeFreeConnector);
            if (connector == nullptr) {
                continue;
            }
            if (connector->connection != DRM_MODE_CONNECTED ||
                connector->count_modes <= 0) {
                continue;
            }
            if (!config_.connector_name.empty() &&
                ConnectorName(connector.get()) != config_.connector_name) {
                continue;
            }
            if (!config_.connector_name.empty()) {
                return connector;  // 命中指定名称
            }
            if (fallback == nullptr) {
                fallback = std::move(connector);  // 自动模式取第一个
            }
        }
        if (fallback == nullptr) {
            SPDLOG_ERROR(
                "HDMI 找不到可用连接器: 指定={} 设备={}（确认图传/显示器已上电并输出 EDID）",
                config_.connector_name.empty() ? "自动" : config_.connector_name,
                config_.drm_device);
        }
        return fallback;
    }

    /// 选择 CRTC：优先沿用连接器当前绑定的 CRTC，否则取第一个空闲 CRTC。
    bool PickCrtc(drmModeRes* resources, drmModeConnector* connector) {
        if (connector->encoder_id != 0) {
            DrmEncoderPtr encoder(
                drmModeGetEncoder(drm_fd_, connector->encoder_id),
                drmModeFreeEncoder);
            if (encoder != nullptr && encoder->crtc_id != 0) {
                crtc_id_ = encoder->crtc_id;
                return true;
            }
        }
        for (int i = 0; i < resources->count_crtcs; ++i) {
            const std::uint32_t candidate = resources->crtcs[i];
            if (!CrtcInUse(resources, candidate)) {
                crtc_id_ = candidate;
                SPDLOG_INFO("HDMI 连接器未绑定 CRTC，选用空闲 CRTC {}", crtc_id_);
                return true;
            }
        }
        SPDLOG_ERROR("HDMI 没有可用 CRTC（{} 个 CRTC 全部被占用）",
                     resources->count_crtcs);
        return false;
    }

    /// 判断 CRTC 是否已被任一连接器占用。
    bool CrtcInUse(drmModeRes* resources, std::uint32_t crtc_id) {
        for (int i = 0; i < resources->count_connectors; ++i) {
            DrmConnectorPtr connector(
                drmModeGetConnector(drm_fd_, resources->connectors[i]),
                drmModeFreeConnector);
            if (connector == nullptr || connector->encoder_id == 0) {
                continue;
            }
            DrmEncoderPtr encoder(
                drmModeGetEncoder(drm_fd_, connector->encoder_id),
                drmModeFreeEncoder);
            if (encoder == nullptr) {
                continue;
            }
            if (encoder->crtc_id == crtc_id) {
                return true;
            }
        }
        return false;
    }

    // ---- GBM / EGL / GL 初始化 ----

    bool SetupGbm() {
        gbm_device_ = gbm_create_device(drm_fd_);
        if (gbm_device_ == nullptr) {
            SPDLOG_ERROR("HDMI 创建 GBM 设备失败（检查 libgbm 与 DRM 驱动匹配）");
            return false;
        }
        // 扫描输出缓冲：需要 SCANOUT + RENDERING，EGL 才能在其上渲染并被 CRTC 扫描。
        gbm_surface_ = gbm_surface_create(
            gbm_device_, config_.width, config_.height, GBM_FORMAT_XRGB8888,
            GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);
        if (gbm_surface_ == nullptr) {
            SPDLOG_ERROR("HDMI 创建 GBM 表面失败: {}x{}（驱动可能不支持该尺寸作为扫描缓冲）",
                         config_.width, config_.height);
            return false;
        }
        return true;
    }

    bool SetupEgl() {
        // eglGetPlatformDisplay 到 EGL 1.5 才进核心，1.4 需走 EXT 扩展，动态取址。
        using GetPlatformDisplayFn = EGLDisplay (*)(EGLenum, void*, const EGLint*);
        const auto get_platform_display = reinterpret_cast<GetPlatformDisplayFn>(
            eglGetProcAddress("eglGetPlatformDisplayEXT"));
        if (get_platform_display != nullptr) {
            egl_display_ = get_platform_display(EGL_PLATFORM_GBM_KHR, gbm_device_,
                                                nullptr);
        } else {
            egl_display_ = eglGetDisplay(
                reinterpret_cast<EGLNativeDisplayType>(gbm_device_));
        }
        if (egl_display_ == EGL_NO_DISPLAY) {
            SPDLOG_ERROR("HDMI 获取 EGLDisplay 失败: 0x{:x}", eglGetError());
            return false;
        }

        EGLint major = 0;
        EGLint minor = 0;
        if (eglInitialize(egl_display_, &major, &minor) != EGL_TRUE) {
            SPDLOG_ERROR(
                "HDMI EGL 初始化失败: 0x{:x}（检查 libEGL 是否支持 GBM 平台/GPU 驱动）",
                eglGetError());
            egl_display_ = EGL_NO_DISPLAY;
            return false;
        }
        egl_version_ = std::to_string(major) + "." + std::to_string(minor);

        const EGLint config_attribs[] = {
            EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR,
            EGL_RED_SIZE,        8,
            EGL_GREEN_SIZE,      8,
            EGL_BLUE_SIZE,       8,
            EGL_ALPHA_SIZE,      0,
            EGL_DEPTH_SIZE,      0,
            EGL_STENCIL_SIZE,    0,
            EGL_NONE};
        EGLint config_count = 0;
        if (eglChooseConfig(egl_display_, config_attribs, &egl_config_, 1,
                            &config_count) != EGL_TRUE ||
            config_count < 1) {
            SPDLOG_ERROR(
                "HDMI 无满足 GLES3 的 EGL 配置: 0x{:x}（本实现要求 GLES3，"
                "因为需要 GL_UNPACK_ROW_LENGTH 处理带 stride 的 NV12）",
                eglGetError());
            return false;
        }

        if (eglBindAPI(EGL_OPENGL_ES_API) != EGL_TRUE) {
            SPDLOG_ERROR("HDMI eglBindAPI(OPENGL_ES) 失败: 0x{:x}", eglGetError());
            return false;
        }
        const EGLint context_attribs[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
        egl_context_ = eglCreateContext(egl_display_, egl_config_, EGL_NO_CONTEXT,
                                        context_attribs);
        if (egl_context_ == EGL_NO_CONTEXT) {
            SPDLOG_ERROR("HDMI 创建 GLES3 上下文失败: 0x{:x}", eglGetError());
            return false;
        }
        egl_surface_ = eglCreateWindowSurface(
            egl_display_, egl_config_,
            reinterpret_cast<EGLNativeWindowType>(gbm_surface_), nullptr);
        if (egl_surface_ == EGL_NO_SURFACE) {
            SPDLOG_ERROR("HDMI 创建 EGL 窗口表面失败: 0x{:x}", eglGetError());
            return false;
        }
        if (eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_) !=
            EGL_TRUE) {
            SPDLOG_ERROR("HDMI eglMakeCurrent 失败: 0x{:x}", eglGetError());
            return false;
        }
        return true;
    }

    bool SetupGl() {
        const GLubyte* version = glGetString(GL_VERSION);
        const GLubyte* renderer = glGetString(GL_RENDERER);
        if (version == nullptr) {
            SPDLOG_ERROR("HDMI 读取 GL 版本失败（当前上下文不可用）");
            return false;
        }
        gl_version_ = reinterpret_cast<const char*>(version);
        gl_renderer_ =
            renderer != nullptr ? reinterpret_cast<const char*>(renderer) : "未知";

        const GLuint vertex_shader = CompileShader(GL_VERTEX_SHADER, kVertexShader);
        const GLuint fragment_shader =
            CompileShader(GL_FRAGMENT_SHADER, kFragmentShader);
        if (vertex_shader == 0 || fragment_shader == 0) {
            if (vertex_shader != 0) {
                glDeleteShader(vertex_shader);
            }
            if (fragment_shader != 0) {
                glDeleteShader(fragment_shader);
            }
            return false;
        }
        program_ = glCreateProgram();
        glAttachShader(program_, vertex_shader);
        glAttachShader(program_, fragment_shader);
        glLinkProgram(program_);
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        GLint linked = GL_FALSE;
        glGetProgramiv(program_, GL_LINK_STATUS, &linked);
        if (linked != GL_TRUE) {
            GLchar info[512] = {0};
            glGetProgramInfoLog(program_, sizeof(info), nullptr, info);
            SPDLOG_ERROR("HDMI 着色器链接失败: {}", info);
            return false;
        }
        loc_tex_y_ = glGetUniformLocation(program_, "u_tex_y");
        loc_tex_uv_ = glGetUniformLocation(program_, "u_tex_uv");
        loc_matrix_ = glGetUniformLocation(program_, "u_color_matrix");
        loc_offset_ = glGetUniformLocation(program_, "u_yuv_offset");
        loc_scale_ = glGetUniformLocation(program_, "u_yuv_scale");

        // 全屏三角形顶点缓冲 + VAO（ES3 下不依赖默认 VAO，兼容性更好）。
        glGenVertexArrays(1, &vao_);
        glBindVertexArray(vao_);
        glGenBuffers(1, &vbo_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(kFullscreenTriangle),
                     kFullscreenTriangle, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glBindVertexArray(0);

        // NV12 双纹理：Y 为 R8，UV 交错为 RG8（半分辨率）。
        glGenTextures(1, &tex_y_);
        glBindTexture(GL_TEXTURE_2D, tex_y_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, static_cast<GLsizei>(config_.width),
                     static_cast<GLsizei>(config_.height), 0, GL_RED,
                     GL_UNSIGNED_BYTE, nullptr);

        glGenTextures(1, &tex_uv_);
        glBindTexture(GL_TEXTURE_2D, tex_uv_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8,
                     static_cast<GLsizei>(config_.width / 2),
                     static_cast<GLsizei>(config_.height / 2), 0, GL_RG,
                     GL_UNSIGNED_BYTE, nullptr);
        glBindTexture(GL_TEXTURE_2D, 0);

        // 色彩矩阵与范围参数只与配置有关，初始化时设置一次。
        const float* matrix = config_.color_space == HdmiColorSpace::kBt601
                                  ? kColorMatrixBt601
                                  : kColorMatrixBt709;
        std::memcpy(color_matrix_, matrix, sizeof(color_matrix_));
        if (config_.full_range) {
            yuv_offset_[0] = 0.0f;
            yuv_scale_[0] = 1.0f;
        } else {
            yuv_offset_[0] = 16.0f / 255.0f;   // 有限范围亮度下限
            yuv_scale_[0] = 255.0f / 219.0f;   // 亮度 219 级扩展到 0~1
        }
        yuv_offset_[1] = 0.5f;
        yuv_offset_[2] = 0.5f;
        yuv_scale_[1] = 1.0f;
        yuv_scale_[2] = 1.0f;
        if (!config_.full_range) {
            yuv_scale_[1] = 255.0f / 224.0f;  // 色度 224 级扩展到 0~1
            yuv_scale_[2] = 255.0f / 224.0f;
        }
        return true;
    }

    GLuint CompileShader(GLenum type, const char* source) {
        const GLuint shader = glCreateShader(type);
        if (shader == 0) {
            SPDLOG_ERROR("HDMI 创建着色器失败（类型 0x{:x}）", type);
            return 0;
        }
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);
        GLint compiled = GL_FALSE;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
        if (compiled != GL_TRUE) {
            GLchar info[512] = {0};
            glGetShaderInfoLog(shader, sizeof(info), nullptr, info);
            SPDLOG_ERROR("HDMI 着色器编译失败（类型 0x{:x}）: {}", type, info);
            glDeleteShader(shader);
            return 0;
        }
        return shader;
    }

    // ---- 上屏 ----

    /// 渲染黑场并完成首次 modeset，使 HDMI 信号在 Open 返回时即已锁定。
    bool ModesetBlack() {
        if (!eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_)) {
            SPDLOG_ERROR("HDMI eglMakeCurrent 失败: 0x{:x}", eglGetError());
            return false;
        }
        glViewport(0, 0, static_cast<GLsizei>(config_.width),
                   static_cast<GLsizei>(config_.height));
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        if (eglSwapBuffers(egl_display_, egl_surface_) != EGL_TRUE) {
            SPDLOG_ERROR("HDMI 黑场 eglSwapBuffers 失败: 0x{:x}", eglGetError());
            return false;
        }
        gbm_bo* bo = gbm_surface_lock_front_buffer(gbm_surface_);
        if (bo == nullptr) {
            SPDLOG_ERROR("HDMI 获取 GBM 前端缓冲失败");
            return false;
        }
        std::uint32_t fb_id = 0;
        if (!AddFramebuffer(bo, &fb_id)) {
            gbm_surface_release_buffer(gbm_surface_, bo);
            return false;
        }
        const int ret = drmModeSetCrtc(drm_fd_, crtc_id_, fb_id, 0, 0,
                                       &connector_id_, 1, &mode_);
        if (ret != 0) {
            SPDLOG_ERROR("HDMI 设置 CRTC/输出模式失败: {}", DescribeDrmError(errno));
            drmModeRmFB(drm_fd_, fb_id);
            gbm_surface_release_buffer(gbm_surface_, bo);
            return false;
        }
        displayed_bo_ = bo;
        displayed_fb_id_ = fb_id;
        return true;
    }

    /// 上传 NV12 到 Y/UV 双纹理。
    /// 布局与 video_encoder.cpp 的拷贝保持一致：Y 平面在前，UV 交错平面在
    /// offset = hor_stride * height 处，每行有效字节数为 width（UV 为 width/2 对）。
    void UploadNv12(const std::uint8_t* data, std::uint32_t width,
                    std::uint32_t height, std::uint32_t hor_stride) {
        const std::uint8_t* y_plane = data;
        const std::uint8_t* uv_plane =
            data + static_cast<std::size_t>(hor_stride) * height;

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, tex_y_);
        // GLES3 的 GL_UNPACK_ROW_LENGTH 以源格式纹素计：R8 时 1 纹素 = 1 像素，
        // 因此直接填 hor_stride，驱动会按 stride 跨行取样并只取 width 列。
        glPixelStorei(GL_UNPACK_ROW_LENGTH, static_cast<GLint>(hor_stride));
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, static_cast<GLsizei>(width),
                        static_cast<GLsizei>(height), GL_RED, GL_UNSIGNED_BYTE,
                        y_plane);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, tex_uv_);
        // UV 为 RG8，每纹素 2 字节：纹素数 = stride / 2。
        glPixelStorei(GL_UNPACK_ROW_LENGTH,
                      static_cast<GLint>(hor_stride / 2));
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, static_cast<GLsizei>(width / 2),
                        static_cast<GLsizei>(height / 2), GL_RG, GL_UNSIGNED_BYTE,
                        uv_plane);

        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glActiveTexture(GL_TEXTURE0);
    }

    void DrawFullscreen() {
        glViewport(0, 0, static_cast<GLsizei>(config_.width),
                   static_cast<GLsizei>(config_.height));
        glUseProgram(program_);
        glUniform1i(loc_tex_y_, 0);
        glUniform1i(loc_tex_uv_, 1);
        glUniformMatrix3fv(loc_matrix_, 1, GL_FALSE, color_matrix_);
        glUniform3fv(loc_offset_, 1, yuv_offset_);
        glUniform3fv(loc_scale_, 1, yuv_scale_);
        glBindVertexArray(vao_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
        glUseProgram(0);
    }

    bool AddFramebuffer(gbm_bo* bo, std::uint32_t* out_fb_id) {
        const std::uint32_t handles[4] = {gbm_bo_get_handle(bo).u32, 0, 0, 0};
        const std::uint32_t pitches[4] = {gbm_bo_get_stride(bo), 0, 0, 0};
        const std::uint32_t offsets[4] = {0, 0, 0, 0};
        const int ret = drmModeAddFB2(
            drm_fd_, gbm_bo_get_width(bo), gbm_bo_get_height(bo),
            gbm_bo_get_format(bo), handles, pitches, offsets, out_fb_id, 0);
        if (ret != 0) {
            SPDLOG_ERROR("HDMI 创建 framebuffer 失败: {}", DescribeDrmError(errno));
            return false;
        }
        return true;
    }

    /// 取前端缓冲并提交页面翻转。失败时不改变已建立的显示状态。
    bool SubmitPageFlip() {
        gbm_bo* bo = gbm_surface_lock_front_buffer(gbm_surface_);
        if (bo == nullptr) {
            SPDLOG_ERROR("HDMI 获取 GBM 前端缓冲失败");
            return false;
        }
        std::uint32_t fb_id = 0;
        if (!AddFramebuffer(bo, &fb_id)) {
            gbm_surface_release_buffer(gbm_surface_, bo);
            return false;
        }
        // 保留 bo 直到翻转确认，避免内核仍在扫描时缓冲被 GBM 复用。
        pending_bo_ = bo;
        pending_fb_id_ = fb_id;
        const int ret = drmModePageFlip(drm_fd_, crtc_id_, fb_id,
                                        DRM_MODE_PAGE_FLIP_EVENT, nullptr);
        if (ret != 0) {
            SPDLOG_ERROR("HDMI 提交页面翻转失败: {}", DescribeDrmError(errno));
            drmModeRmFB(drm_fd_, fb_id);
            gbm_surface_release_buffer(gbm_surface_, bo);
            pending_bo_ = nullptr;
            pending_fb_id_ = 0;
            return false;
        }
        flip_pending_ = true;
        flip_submit_time_ = Clock::now();
        return true;
    }

    /// 渲染黑场并提交翻转，然后等待完成。用于 Close 收尾：让图传发射机停在
    /// 稳定黑场而不是冻结的最后一帧。
    bool PresentBlackAndWait() {
        glViewport(0, 0, static_cast<GLsizei>(config_.width),
                   static_cast<GLsizei>(config_.height));
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        if (eglSwapBuffers(egl_display_, egl_surface_) != EGL_TRUE) {
            return false;
        }
        if (!SubmitPageFlip()) {
            return false;
        }
        WaitForPendingFlip(static_cast<std::uint32_t>(config_.flip_timeout_ms));
        return true;
    }

    /// 非阻塞收割 DRM 事件：仅在确有可读事件时处理，绝不等待垂直同步。
    void ReapFlipEvents() {
        if (drm_fd_ < 0) {
            return;
        }
        struct pollfd descriptor {};
        descriptor.fd = drm_fd_;
        descriptor.events = POLLIN;
        if (::poll(&descriptor, 1, 0) <= 0) {
            return;
        }
        drmEventContext context {};
        context.version = DRM_EVENT_CONTEXT_VERSION;
        context.page_flip_handler = &HdmiDrmDisplay::PageFlipTrampoline;
        drmHandleEvent(drm_fd_, &context);
    }

    /// 页面翻转完成处理：确认本次上屏，并回收上一个已上屏的 framebuffer。
    void OnFlipComplete() {
        RecyclePrevious();
        displayed_bo_ = pending_bo_;
        displayed_fb_id_ = pending_fb_id_;
        pending_bo_ = nullptr;
        pending_fb_id_ = 0;
        flip_pending_ = false;
    }

    /// 翻转事件跳板。
    ///
    /// 刻意不取用 drmModePageFlip 传入的 user_data：该字段依赖内核对
    /// drm_mode_page_flip.user_data 的回传支持，在新旧内核/libdrm 的组合上行为不一致；
    /// 未回传时 drmHandleEvent 会把空指针直接交给回调，静态转换后解引用即崩溃。
    /// 本模块同一时刻只允许一个 HDMI 输出实例，故改用静态实例指针定位，
    /// 从机制上避开这一版本差异。
    static void PageFlipTrampoline(int /*fd*/, unsigned int /*sequence*/,
                                   unsigned int /*tv_sec*/,
                                   unsigned int /*tv_usec*/,
                                   void* /*user_data*/) {
        if (active_instance_ != nullptr) {
            active_instance_->OnFlipComplete();
        }
    }

    /// 回收"上一个"已上屏的缓冲。调用时机必须是新翻转已确认之后，
    /// 此时内核已不再扫描旧缓冲，删除 framebuffer 与归还 GBM 缓冲都是安全的。
    /// 采用一帧滞后的方式，任何时刻至多 2~3 个 framebuffer 存活。
    void RecyclePrevious() {
        if (displayed_fb_id_ != 0 && drm_fd_ >= 0) {
            drmModeRmFB(drm_fd_, displayed_fb_id_);
        }
        if (displayed_bo_ != nullptr && gbm_surface_ != nullptr) {
            gbm_surface_release_buffer(gbm_surface_, displayed_bo_);
        }
        displayed_bo_ = nullptr;
        displayed_fb_id_ = 0;
    }

    /// 有界等待未完成的翻转。只能在收尾路径使用，运行期绝不调用。
    void WaitForPendingFlip(std::uint32_t timeout_ms) {
        if (!flip_pending_ || drm_fd_ < 0) {
            return;
        }
        const auto deadline =
            Clock::now() + std::chrono::milliseconds(timeout_ms);
        while (flip_pending_) {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - Clock::now());
            if (remaining.count() <= 0) {
                break;
            }
            struct pollfd descriptor {};
            descriptor.fd = drm_fd_;
            descriptor.events = POLLIN;
            if (::poll(&descriptor, 1, static_cast<int>(remaining.count())) <= 0) {
                break;
            }
            drmEventContext context {};
            context.version = DRM_EVENT_CONTEXT_VERSION;
            context.page_flip_handler = &HdmiDrmDisplay::PageFlipTrampoline;
            drmHandleEvent(drm_fd_, &context);
        }
        if (flip_pending_) {
            SPDLOG_WARN("HDMI 等待页面翻转完成超时({}ms)，强制继续收尾", timeout_ms);
        }
    }

    // ---- 资源释放 ----

    void TeardownGl() {
        if (egl_display_ == EGL_NO_DISPLAY) {
            return;
        }
        if (tex_y_ != 0) {
            glDeleteTextures(1, &tex_y_);
            tex_y_ = 0;
        }
        if (tex_uv_ != 0) {
            glDeleteTextures(1, &tex_uv_);
            tex_uv_ = 0;
        }
        if (vbo_ != 0) {
            glDeleteBuffers(1, &vbo_);
            vbo_ = 0;
        }
        if (vao_ != 0) {
            glDeleteVertexArrays(1, &vao_);
            vao_ = 0;
        }
        if (program_ != 0) {
            glDeleteProgram(program_);
            program_ = 0;
        }
        eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE,
                       EGL_NO_CONTEXT);
    }

    void TeardownEgl() {
        if (egl_display_ == EGL_NO_DISPLAY) {
            return;
        }
        if (egl_surface_ != EGL_NO_SURFACE) {
            eglDestroySurface(egl_display_, egl_surface_);
            egl_surface_ = EGL_NO_SURFACE;
        }
        if (egl_context_ != EGL_NO_CONTEXT) {
            eglDestroyContext(egl_display_, egl_context_);
            egl_context_ = EGL_NO_CONTEXT;
        }
        eglTerminate(egl_display_);
        egl_display_ = EGL_NO_DISPLAY;
        egl_config_ = nullptr;
    }

    void TeardownGbm() {
        if (gbm_surface_ != nullptr) {
            gbm_surface_destroy(gbm_surface_);
            gbm_surface_ = nullptr;
        }
        if (gbm_device_ != nullptr) {
            gbm_device_destroy(gbm_device_);
            gbm_device_ = nullptr;
        }
        // 未归还的 GBM 缓冲随 surface 销毁一并释放，无需单独处理。
        displayed_bo_ = nullptr;
        pending_bo_ = nullptr;
    }

    void TeardownDrm() {
        if (saved_crtc_ != nullptr) {
            saved_crtc_.reset();
        }
        if (drm_fd_ >= 0) {
            drmDropMaster(drm_fd_);
            ::close(drm_fd_);
            drm_fd_ = -1;
        }
        connector_id_ = 0;
        crtc_id_ = 0;
        saved_connector_id_ = 0;
        displayed_fb_id_ = 0;
        pending_fb_id_ = 0;
        flip_pending_ = false;
        frame_index_ = 0;
        if (active_instance_ == this) {
            active_instance_ = nullptr;
        }
    }

    /// 用连接器 ID 反查名称，仅用于日志。
    std::string ConnectorNameOf(std::uint32_t connector_id) const {
        if (drm_fd_ < 0 || connector_id == 0) {
            return "未知";
        }
        DrmConnectorPtr connector(drmModeGetConnector(drm_fd_, connector_id),
                                  drmModeFreeConnector);
        return connector != nullptr ? ConnectorName(connector.get())
                                    : std::to_string(connector_id);
    }

    // ---- 状态 ----
    HdmiDisplayConfig config_;
    bool open_ = false;
    std::string mode_name_;
    std::string egl_version_;
    std::string gl_version_;
    std::string gl_renderer_;

    // DRM
    int drm_fd_ = -1;
    std::uint32_t connector_id_ = 0;
    std::uint32_t crtc_id_ = 0;
    std::uint32_t saved_connector_id_ = 0;
    drmModeModeInfo mode_{};
    DrmCrtcPtr saved_crtc_{nullptr, drmModeFreeCrtc};

    // GBM
    gbm_device* gbm_device_ = nullptr;
    gbm_surface* gbm_surface_ = nullptr;

    // EGL / GLES
    EGLDisplay egl_display_ = EGL_NO_DISPLAY;
    EGLContext egl_context_ = EGL_NO_CONTEXT;
    EGLSurface egl_surface_ = EGL_NO_SURFACE;
    EGLConfig egl_config_ = nullptr;
    GLuint program_ = 0;
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLuint tex_y_ = 0;
    GLuint tex_uv_ = 0;
    GLint loc_tex_y_ = -1;
    GLint loc_tex_uv_ = -1;
    GLint loc_matrix_ = -1;
    GLint loc_offset_ = -1;
    GLint loc_scale_ = -1;
    float color_matrix_[9] = {0.0f};
    float yuv_offset_[3] = {0.0f, 0.5f, 0.5f};
    float yuv_scale_[3] = {1.0f, 1.0f, 1.0f};

    // 页面翻转状态
    gbm_bo* displayed_bo_ = nullptr;   ///< 正在扫描输出的缓冲（未归还 GBM）
    gbm_bo* pending_bo_ = nullptr;     ///< 已提交翻转、尚未确认的缓冲
    std::uint32_t displayed_fb_id_ = 0;
    std::uint32_t pending_fb_id_ = 0;
    bool flip_pending_ = false;
    Clock::time_point flip_submit_time_{};

    // 统计
    common::LatencyStatistics flip_latency_;
    common::LatencyStatistics prepare_latency_;
    std::uint64_t presented_count_ = 0;
    std::uint64_t skipped_count_ = 0;
    std::uint64_t invalid_count_ = 0;
    std::uint64_t frame_index_ = 0;

    /// 当前活动的显示实例（翻转事件跳板用）。进程内至多一个 HDMI 输出。
    static HdmiDrmDisplay* active_instance_;
};

HdmiDrmDisplay* HdmiDrmDisplay::active_instance_ = nullptr;

}  // namespace

std::unique_ptr<IHdmiDisplay> CreateHdmiDrmDisplay() {
    return std::make_unique<HdmiDrmDisplay>();
}

std::string DescribeHdmiDrmResources(const std::string& device,
                                     std::uint32_t target_width,
                                     std::uint32_t target_height,
                                     int target_refresh_hz) {
    const int fd = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        return "打开 " + device + " 失败: " + DescribeDrmError(errno) + "\n";
    }
    std::string report = "DRM 设备: " + device + "\n";

    DrmResPtr resources(drmModeGetResources(fd), drmModeFreeResources);
    if (resources == nullptr) {
        report += std::string("读取 DRM 资源失败: ") + DescribeDrmError(errno) + "\n";
        ::close(fd);
        return report;
    }
    report += "CRTC 数量: " + std::to_string(resources->count_crtcs) +
              ", 连接器数量: " + std::to_string(resources->count_connectors) + "\n";

    for (int i = 0; i < resources->count_connectors; ++i) {
        DrmConnectorPtr connector(drmModeGetConnector(fd, resources->connectors[i]),
                                  drmModeFreeConnector);
        if (connector == nullptr) {
            continue;
        }
        const char* connection_text = "未知";
        switch (connector->connection) {
            case DRM_MODE_CONNECTED:
                connection_text = "已连接";
                break;
            case DRM_MODE_DISCONNECTED:
                connection_text = "未连接";
                break;
            default:
                break;
        }
        report += "\n连接器 " + ConnectorName(connector.get()) + " [" +
                  connection_text + "]" + " 模式数=" +
                  std::to_string(connector->count_modes);
        if (connector->encoder_id != 0) {
            DrmEncoderPtr encoder(drmModeGetEncoder(fd, connector->encoder_id),
                                  drmModeFreeEncoder);
            report += ", 当前CRTC=" +
                      std::to_string(encoder != nullptr ? encoder->crtc_id : 0);
        }
        report += "\n";
        for (int m = 0; m < connector->count_modes; ++m) {
            const drmModeModeInfo& mode = connector->modes[m];
            char line[160];
            std::snprintf(line, sizeof(line),
                          "  [%2d] %-28s %4dx%-4d @%3dHz clock=%ukHz%s\n", m,
                          mode.name, mode.hdisplay, mode.vdisplay, mode.vrefresh,
                          static_cast<unsigned>(mode.clock),
                          (mode.type & DRM_MODE_TYPE_PREFERRED) ? "  <- 优选" : "");
            report += line;
        }

        // 目标模式匹配结论：给出会被实际采用的模式及回退级别。
        if (target_width != 0 && target_height != 0) {
            HdmiDisplayConfig probe;
            probe.width = target_width;
            probe.height = target_height;
            probe.refresh_hz = target_refresh_hz > 0 ? target_refresh_hz : 30;
            drmModeModeInfo picked{};
            ModePickSource source = ModePickSource::kEdidExact;
            if (PickOutputMode(connector.get(), probe, &picked, &source)) {
                const char* source_text = "EDID 精确匹配";
                if (source == ModePickSource::kEdidFallback) {
                    source_text = "EDID 同分辨率回退(刷新率不一致)";
                } else if (source == ModePickSource::kSynthesized) {
                    source_text = "合成时序(EDID 无该分辨率，图传可能不锁定)";
                }
                report += std::string("  -> 目标 ") + std::to_string(target_width) +
                          "x" + std::to_string(target_height) + "@" +
                          std::to_string(probe.refresh_hz) + "Hz 将采用: " +
                          picked.name + "（" + source_text + "）\n";
            } else {
                report += std::string("  -> 目标 ") + std::to_string(target_width) +
                          "x" + std::to_string(target_height) + "@" +
                          std::to_string(probe.refresh_hz) +
                          "Hz 无可用模式（非 720p 不支持合成）\n";
            }
        }
    }

    for (int i = 0; i < resources->count_crtcs; ++i) {
        DrmCrtcPtr crtc(drmModeGetCrtc(fd, resources->crtcs[i]), drmModeFreeCrtc);
        if (crtc == nullptr) {
            continue;
        }
        char line[160];
        std::snprintf(line, sizeof(line), "CRTC %u: %s framebuffer=%u 模式=%s\n",
                      crtc->crtc_id, crtc->mode_valid ? "已启用" : "已关闭",
                      crtc->buffer_id,
                      crtc->mode_valid ? crtc->mode.name : "-");
        report += line;
    }
    ::close(fd);
    return report;
}

}  // namespace drone::video_transmission

#endif  // DRONE_HAVE_HDMI_KMS
