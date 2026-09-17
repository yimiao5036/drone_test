/**
 * @file hdmi_display_backend.cpp
 * @brief HDMI 直显后端实现（实现 IVideoEncoderBackend）与显示设备编译期分派
 *
 * 数据流：标注帧 NV12 FrameHandle ──► 帧校验 / 尺寸核对 ──► IHdmiDisplay::Submit
 *         ──► DRM 页面翻转 ──► HDMI 输出。
 *
 * 与 FFmpeg 编码后端（src/video_transmission/video_encoder.cpp）的对应关系：
 * - 同样从 VideoSender 的单线程被调用，内部不额外加锁；
 * - 同样把"拥塞/失败"限制在输出链内：翻转未完成只丢本帧并返回 false，
 *   VideoSender 会将其计入 DroppedFrameCount，不阻塞、不反压上游；
 * - 关键路径按项目日志纪律接入 spdlog（错误节流：第 1 次 + 每满 100 次）。
 *
 * 本文件刻意不引用任何图形库：libdrm/libgbm/EGL 只出现在
 * hdmi_drm_display.cpp 内，因此本文件与占位实现可在任意开发机编译与单测。
 */
#include "video_transmission/hdmi/hdmi_display_backend.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <spdlog/spdlog.h>

namespace drone::video_transmission {

namespace {

/// 异常日志节流：第 1 次与每满 100 次才打印，避免高频异常刷屏。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

/// 色彩空间与范围的可读名称，仅用于日志。
const char* ColorSpaceName(HdmiColorSpace space) {
    return space == HdmiColorSpace::kBt601 ? "BT.601" : "BT.709";
}

}  // namespace

/// HDMI 后端实现细节（PIMPL）：显示设备持有者与统计。
struct HdmiDisplayBackend::Impl {
    HdmiDisplayConfig config;
    std::unique_ptr<IHdmiDisplay> display;
    std::atomic<std::uint64_t> sent_count{0};
    std::atomic<std::uint64_t> error_count{0};
    std::atomic<bool> running{false};

    // 构造阶段只校验静态配置，不打开设备；真正的 DRM/EGL 资源在 Open 中创建。
    explicit Impl(HdmiDisplayConfig cfg, std::unique_ptr<IHdmiDisplay> disp)
        : config(std::move(cfg)), display(std::move(disp)) {
        if (config.width == 0 || config.height == 0) {
            throw std::invalid_argument("HDMI 输出分辨率必须大于 0");
        }
        if (config.refresh_hz <= 0) {
            throw std::invalid_argument("HDMI 输出刷新率必须大于 0");
        }
        if (config.flip_timeout_ms <= 0) {
            throw std::invalid_argument("HDMI 翻转等待超时必须大于 0");
        }
        // 占位实现不需要设备节点，仅在走 DRM 实现时校验
        if (display == nullptr) {
            if (config.drm_device.empty()) {
                throw std::invalid_argument("HDMI DRM 设备节点不能为空");
            }
            display = CreateHdmiDisplay();
        }
        if (display == nullptr) {
            throw std::runtime_error("HDMI 显示设备创建失败");
        }
    }

    ~Impl() { Stop(); }

    // 启动顺序：打开设备 + modeset 出黑场 → 置运行标志。
    // 失败时统一关闭设备，避免 DRM master 或 EGL 上下文残留。
    bool Start() {
        if (running.load()) {
            return true;  // 幂等
        }
        try {
            if (!display->Open(config)) {
                ++error_count;
                SPDLOG_ERROR("HDMI 直显设备打开失败: {} {}x{}@{}Hz（检查设备节点/权限/图传是否上电）",
                             config.drm_device, config.width, config.height,
                             config.refresh_hz);
                return false;
            }
            running = true;
            SPDLOG_INFO("HDMI 直显后端就绪: {} {}x{}@{}Hz, 色彩空间={}, 范围={}, 模式={}",
                        config.drm_device, config.width, config.height,
                        config.refresh_hz, ColorSpaceName(config.color_space),
                        config.full_range ? "全范围" : "有限范围",
                        display->ActiveModeName());
            return true;
        } catch (const std::exception& e) {
            ++error_count;
            SPDLOG_ERROR("HDMI 直显后端启动异常: {}", e.what());
            display->Close();
            return false;
        }
    }

    // 停止顺序：清运行标志 → 关闭设备（冲刷未完成翻转、按需点黑场、释放资源）。
    void Stop() {
        if (!running.load()) {
            return;
        }
        running = false;
        if (display != nullptr) {
            display->Close();
        }
        SPDLOG_INFO("HDMI 直显后端关闭: 累计上屏 {} 帧, 丢弃 {} 帧, 错误 {} 次",
                    sent_count.load(), display != nullptr
                                          ? display->SkippedFrameCount()
                                          : 0,
                    error_count.load());
    }

    /// 校验并提交一帧 NV12 上屏。
    /// 与 FFmpeg 后端一致：格式/尺寸/句柄异常计入错误并丢帧；设备繁忙只丢帧不计错误。
    bool EncodeFrame(const video::FrameHandle& frame) {
        if (!running.load()) {
            ++error_count;
            return false;
        }
        const video::VideoFrameInfo& info = frame.Info();
        if (!frame.Valid() || info.format != video::PixelFormat::kYuv420SpNv12) {
            ++error_count;
            if (ShouldLogThrottled(error_count)) {
                SPDLOG_ERROR("HDMI 输入帧非法(格式/句柄无效)，累计 {}",
                             error_count.load());
            }
            return false;
        }
        // 输出模式在 Open 时已固定，输入尺寸不匹配无法靠缩放兜底（避免静默拉伸）。
        if (info.width != config.width || info.height != config.height) {
            ++error_count;
            if (ShouldLogThrottled(error_count)) {
                SPDLOG_ERROR("HDMI 输入尺寸不匹配: 输出={}x{} 输入={}x{}，累计 {}",
                             config.width, config.height, info.width, info.height,
                             error_count.load());
            }
            return false;
        }
        const std::byte* src = frame.Data();
        if (src == nullptr) {
            ++error_count;
            return false;
        }

        const HdmiSubmitResult result = display->Submit(
            reinterpret_cast<const std::uint8_t*>(src), info.width, info.height,
            info.hor_stride);
        if (result == HdmiSubmitResult::kSubmitted) {
            ++sent_count;
            return true;
        }
        if (result == HdmiSubmitResult::kBusy) {
            // 上一次翻转未完成：属预期降级，只丢本帧，不计错误、不打日志（高频）。
            // VideoSender 会把它计入 DroppedFrameCount。
            return false;
        }
        ++error_count;
        if (ShouldLogThrottled(error_count)) {
            SPDLOG_ERROR("HDMI 提交上屏失败(设备错误或翻转卡死)，累计 {}",
                         error_count.load());
        }
        return false;
    }
};

// ---------------------------------------------------------------------------
// 编译期分派：有 libdrm/libgbm/EGL 时用真实 DRM 实现，否则回退占位实现
// ---------------------------------------------------------------------------

std::unique_ptr<IHdmiDisplay> CreateHdmiDisplay() {
#ifdef DRONE_HAVE_HDMI_KMS
    return CreateHdmiDrmDisplay();
#else
    return CreateHdmiDisplayStub();
#endif
}

// ---------------------------------------------------------------------------
// HdmiDisplayBackend：IVideoEncoderBackend 接口转发
// ---------------------------------------------------------------------------

HdmiDisplayBackend::HdmiDisplayBackend(HdmiDisplayConfig config)
    : impl_(std::make_unique<Impl>(std::move(config), nullptr)) {
    SPDLOG_INFO("HDMI 直显后端创建: {} {}x{}@{}Hz, 连接器={}",
                impl_->config.drm_device, impl_->config.width, impl_->config.height,
                impl_->config.refresh_hz,
                impl_->config.connector_name.empty() ? "自动"
                                                     : impl_->config.connector_name);
}

HdmiDisplayBackend::HdmiDisplayBackend(HdmiDisplayConfig config,
                                       std::unique_ptr<IHdmiDisplay> display)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(display))) {
    SPDLOG_INFO("HDMI 直显后端创建(注入显示设备): {}x{}@{}Hz",
                impl_->config.width, impl_->config.height,
                impl_->config.refresh_hz);
}

HdmiDisplayBackend::~HdmiDisplayBackend() {
    if (impl_ != nullptr) {
        impl_->Stop();
    }
    SPDLOG_INFO("HDMI 直显后端销毁");
}

bool HdmiDisplayBackend::Start() { return impl_->Start(); }

void HdmiDisplayBackend::Stop() { impl_->Stop(); }

bool HdmiDisplayBackend::IsRunning() const { return impl_->running.load(); }

bool HdmiDisplayBackend::EncodeFrame(const video::FrameHandle& frame) {
    return impl_->EncodeFrame(frame);
}

std::uint64_t HdmiDisplayBackend::SentFrameCount() const {
    return impl_->sent_count.load();
}

std::uint64_t HdmiDisplayBackend::ErrorCount() const {
    return impl_->error_count.load();
}

common::LatencySummary HdmiDisplayBackend::FramePrepareLatency() const {
    return impl_->display != nullptr ? impl_->display->PrepareLatency()
                                     : common::LatencySummary{};
}

common::LatencySummary HdmiDisplayBackend::PacketWriteLatency() const {
    return impl_->display != nullptr ? impl_->display->FlipLatency()
                                     : common::LatencySummary{};
}

std::uint64_t HdmiDisplayBackend::SkippedFrameCount() const {
    return impl_->display != nullptr ? impl_->display->SkippedFrameCount() : 0;
}

std::string HdmiDisplayBackend::ActiveModeName() const {
    return impl_->display != nullptr ? impl_->display->ActiveModeName() : std::string{};
}

// ---------------------------------------------------------------------------
// 装配辅助：注入工厂与配置桥接
// ---------------------------------------------------------------------------

std::function<std::unique_ptr<IVideoEncoderBackend>()> HdmiMakeBackendFactory(
    HdmiDisplayConfig config) {
    // 配置按值捕获，每次调用生成一个新后端；VideoSender 只会调用一次。
    // 构造阶段抛出的配置错误在此收敛为 nullptr，交由 VideoSender::Start 的
    // "创建编码后端失败"分支处理，避免未捕获异常直接终止进程。
    return [config]() -> std::unique_ptr<IVideoEncoderBackend> {
        try {
            return std::make_unique<HdmiDisplayBackend>(config);
        } catch (const std::exception& e) {
            SPDLOG_ERROR("HDMI 直显后端创建失败: {}", e.what());
            return nullptr;
        }
    };
}

HdmiDisplayConfig HdmiConfigFromEncoderConfig(const EncoderBackendConfig& encode) {
    HdmiDisplayConfig config;
    // 只桥接对 HDMI 输出有意义的字段：输出尺寸与帧率。
    if (encode.width > 0 && encode.height > 0) {
        config.width = encode.width;
        config.height = encode.height;
    }
    if (encode.fps > 0) {
        // 输出刷新率取源帧率向上取整到 30/60 常用档，避免低于源帧率造成丢帧。
        config.refresh_hz = encode.fps <= 30 ? 30 : 60;
    }
    return config;
}

}  // namespace drone::video_transmission
