/**
 * @file hdmi_display_backend.h
 * @brief HDMI 直显后端（IVideoEncoderBackend 的第二个实现：DRM/KMS + GBM + EGL/GLES）
 *
 * 属于 drone/video_transmission 模块。`video_encoder.h` 把"编码 + 图传输出"
 * 抽象为可替换边界 IVideoEncoderBackend，FFmpeg 编码后端是其默认实现；本文件
 * 提供针对"HDMI 直出"场景的第二个实现，两者通过 VideoSenderConfig::backend_factory
 * 在装配层二选一，原有文件不做任何改动。
 *
 * 适用场景：
 * - 香橙派 HDMI 口直连 HDMI-in 数字图传发射机（本实现的主要目标），或直连监视屏；
 * - 运行环境无桌面（X11/Wayland），进程独占 DRM master。
 *
 * 设计目标：
 * - **不编码**：标注帧本身已是 NV12，直接由 GPU 做 YUV→RGB 后提交 DRM 页面翻转，
 *   省掉 H.264 编码与 RTSP 推流环节，链路更短、端到端延迟更低。
 * - **时序优先**：Start() 立即完成 modeset 并点出黑场后持续输出，使 HDMI-in 图传
 *   发射机上电即锁定信号，避免反复重锁。
 * - **绝不阻塞**：上一次页面翻转未完成时直接丢弃本帧，不等待、不反压上游，
 *   与 VideoSender"拥塞只丢图传帧"的既有约定一致。
 *
 * 语义映射（接口名沿用编码后端，避免改动 VideoSender 与冻结接口）：
 * - EncodeFrame         → 提交一帧 NV12 上屏；被丢弃时返回 false，VideoSender
 *                         会将其计入 DroppedFrameCount；
 * - SentFrameCount      → 累计成功上屏帧数；
 * - FramePrepareLatency → NV12 纹理上传与绘制准备耗时；
 * - PacketWriteLatency  → 单次页面翻转提交耗时（含收割上一次翻转事件）。
 *
 * 权限要求：打开 /dev/dri/card0 并成为 DRM master 需要 root 或 CAP_SYS_ADMIN。
 *
 * 开发机说明：CMake 未找到 libdrm/libgbm/EGL 时不定义 DRONE_HAVE_HDMI_KMS，
 * CreateHdmiDisplay() 会返回占位实现（hdmi_display_stub.cpp）：生命周期完整、
 * 可跑通 VideoSender 全链路，但不产生真实画面。实机必须使用 DRM 实现。
 */
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "common/latency_statistics.h"
#include "video/video_frame.h"
#include "video_transmission/video_encoder.h"

namespace drone::video_transmission {

/// HDMI 输出的 YUV→RGB 转换矩阵。
/// 源为 720p H.264，普遍按 BT.709 编码；若实测偏色/发灰，改这里或 `full_range`。
enum class HdmiColorSpace : std::uint8_t {
    kBt601 = 0,  ///< BT.601（标清习惯）
    kBt709 = 1,  ///< BT.709（高清习惯，默认）
};

/// 单帧上屏提交结果。
enum class HdmiSubmitResult : std::uint8_t {
    kSubmitted = 0,  ///< 已提交页面翻转，等待垂直同步完成
    kBusy = 1,       ///< 上一次翻转未完成，本帧丢弃（预期降级，不计错误）
    kFailed = 2,     ///< 提交失败（设备错误或翻转卡死超时）
};

/// HDMI 直显配置。
struct HdmiDisplayConfig {
    std::uint32_t width = 1280;  ///< 输出模式宽（像素）
    std::uint32_t height = 720;  ///< 输出模式高（像素）
    int refresh_hz = 30;         ///< 输出模式刷新率（Hz）；源为 25fps，30Hz 足以保持时序
    /// 严格模式：true 时只接受与 width/height/refresh_hz 完全匹配（或合成）的模式；
    /// false 时允许回退到同分辨率的其他刷新率（EDID 未报 30Hz 时避免图传失锁）。
    bool strict_mode = false;
    std::string drm_device = "/dev/dri/card0";  ///< DRM 设备节点
    /// connector 名称（如 "HDMI-A-1"）；为空时自动选择第一个已连接的 connector。
    std::string connector_name;
    HdmiColorSpace color_space = HdmiColorSpace::kBt709;  ///< YUV→RGB 矩阵
    /// false = 有限范围（16-235，与 H.264 源一致）；true = 全范围（0-255）。
    bool full_range = false;
    /// 页面翻转等待超时：Close 时等待未完成翻转的上限；运行期超过该值视为设备卡死。
    int flip_timeout_ms = 100;
    /// 停止时是否点一帧黑场（保持 CRTC 使能）。true 避免图传发射机停在冻结画面上；
    /// false 则恢复进入前的 CRTC 状态。
    bool blank_on_stop = true;
    /// >0 时每 N 帧打一次 INFO 日志，仅用于上板调试；正式运行保持 0（逐帧热路径不打日志）。
    int log_every_n_frames = 0;
};

/// HDMI 显示设备抽象：把 libdrm/libgbm/EGL 全部隔离在实现文件内，使后端主体、
/// 占位实现与单元测试不依赖任何图形库。
///
/// 线程安全：实现只在 VideoSender 的消费线程内被调用，内部不额外加锁。
class IHdmiDisplay {
public:
    virtual ~IHdmiDisplay() = default;

    // ---- 生命周期 ----
    /// 打开设备、设置输出模式并点出初始黑场。失败返回 false（设备缺失、无权限、
    /// 无可用模式、EGL 初始化失败等）。
    virtual bool Open(const HdmiDisplayConfig& config) = 0;
    /// 冲刷未完成翻转、按配置点黑场或恢复原 CRTC，并释放全部资源。幂等。
    virtual void Close() = 0;
    /// 是否已建立输出会话。
    virtual bool IsOpen() const = 0;

    // ---- 帧输入 ----
    /// 提交一帧 NV12 上屏。
    /// @param data       NV12 缓冲首地址（Y 平面起始，UV 平面紧随其后）
    /// @param width      有效像素宽
    /// @param height     有效像素高
    /// @param hor_stride 源行 stride（像素），可能大于 width，需按 stride 逐行读取
    /// @return 提交结果；kBusy 表示上一次翻转未完成，本帧已被丢弃。
    virtual HdmiSubmitResult Submit(const std::uint8_t* data, std::uint32_t width,
                                    std::uint32_t height,
                                    std::uint32_t hor_stride) = 0;

    // ---- 状态查询 ----
    /// 单次页面翻转提交耗时（毫秒），含事件收割。
    virtual common::LatencySummary FlipLatency() const = 0;
    /// NV12 纹理上传与绘制准备耗时（毫秒）。
    virtual common::LatencySummary PrepareLatency() const = 0;
    /// 因上一次翻转未完成而丢弃的帧数。
    virtual std::uint64_t SkippedFrameCount() const = 0;
    /// 实际生效的输出模式描述（如 "1280x720@30"），用于日志与带外自检。
    virtual std::string ActiveModeName() const = 0;
};

/// 创建默认 HDMI 显示设备。
/// 编译期定义 DRONE_HAVE_HDMI_KMS 时返回 DRM/KMS 实现，否则返回占位实现。
std::unique_ptr<IHdmiDisplay> CreateHdmiDisplay();

/// 创建 DRM/KMS 显示设备（仅在定义 DRONE_HAVE_HDMI_KMS 时可用）。
/// 测试可显式调用以绕开编译期分派。
std::unique_ptr<IHdmiDisplay> CreateHdmiDrmDisplay();

/// 创建占位显示设备：不接触任何图形设备，仅模拟时序与计数。
/// 用途：无 /dev/dri 的开发机上编译并跑通 VideoSender 全链路，验证丢帧与统计。
std::unique_ptr<IHdmiDisplay> CreateHdmiDisplayStub();

/// 枚举 DRM 资源并返回可读报告（连接器 / CRTC / 模式列表）。
/// 用于上板自检 `hdmi_probe modes`：**只读**，不获取 master、不修改显示状态。
/// @param device      DRM 设备节点，如 "/dev/dri/card0"
/// @param target_width  目标模式宽；非 0 时额外给出模式匹配结论
/// @param target_height 目标模式高
/// @param target_refresh_hz 目标刷新率；<=0 表示不限定刷新率
/// @return 多行文本报告；未定义 DRONE_HAVE_HDMI_KMS 时返回说明字符串。
std::string DescribeHdmiDrmResources(const std::string& device,
                                     std::uint32_t target_width = 0,
                                     std::uint32_t target_height = 0,
                                     int target_refresh_hz = 0);

/// HDMI 直显后端：实现 IVideoEncoderBackend，供 VideoSender 的 backend_factory 注入。
///
/// 生命周期：Start（打开设备 + modeset）→ 多帧 EncodeFrame → Stop。
/// 日志约定：创建/销毁、设备启停、模式选择为主线 INFO；错误按项目日志纪律节流
/// （第 1 次 + 每满 100 次）；逐帧热路径不打日志。
class HdmiDisplayBackend final : public IVideoEncoderBackend {
public:
    explicit HdmiDisplayBackend(HdmiDisplayConfig config);
    /// 注入式构造：display 为空时回退到 CreateHdmiDisplay()。
    /// 测试通过注入 Mock 验证丢帧/计数逻辑而不依赖真实 DRM 设备。
    HdmiDisplayBackend(HdmiDisplayConfig config,
                       std::unique_ptr<IHdmiDisplay> display);
    ~HdmiDisplayBackend() override;

    HdmiDisplayBackend(const HdmiDisplayBackend&) = delete;
    HdmiDisplayBackend& operator=(const HdmiDisplayBackend&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    bool EncodeFrame(const video::FrameHandle& frame) override;

    std::uint64_t SentFrameCount() const override;
    std::uint64_t ErrorCount() const override;
    common::LatencySummary FramePrepareLatency() const override;
    common::LatencySummary PacketWriteLatency() const override;

    /// 累计因上一次翻转未完成而丢弃的帧数（接口外的补充统计，供探针/测试读取）。
    std::uint64_t SkippedFrameCount() const;
    /// 实际生效的输出模式描述。
    std::string ActiveModeName() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// 构造可直接赋给 VideoSenderConfig::backend_factory 的注入函数。
///
/// 用法（装配层二选一，不改动 VideoSender）：
/// \code
/// if (使用 HDMI 输出) {
///     video_sender_config.backend_factory =
///         video_transmission::HdmiMakeBackendFactory(hdmi_config);
/// }
/// \endcode
std::function<std::unique_ptr<IVideoEncoderBackend>()> HdmiMakeBackendFactory(
    HdmiDisplayConfig config);

/// 从既有 EncoderBackendConfig 派生 HDMI 配置（只复用 width/height/fps）。
/// url/codec/bitrate/gop/transport/output_format 对 HDMI 输出无意义，直接忽略，
/// 便于装配层在"RTSP 推流 / HDMI 直出"之间切换时复用同一份 video 配置。
HdmiDisplayConfig HdmiConfigFromEncoderConfig(const EncoderBackendConfig& encode);

}  // namespace drone::video_transmission
