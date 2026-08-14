/**
 * @file video_sender.h
 * @brief 图传发送部件接口（IVideoSender）与真实实现（VideoSender）
 *
 * 属于 drone/video_transmission 模块。职责：订阅标注后的视频帧
 * （video::FrameHandle，即 kAnnotatedFrame，由 FrameCompositor 产出），
 * 编码并推送给图传。图传拥塞只允许丢图传帧，不得反压摄像头或感知链路。
 *
 * 说明：
 * - 本接口为纯虚抽象，接口签名在 docs/数据接口文档.md §4.3 冻结，
 *   业务逻辑阶段只替换 Stub 实现不改签名。
 * - VideoSender 为真实实现：订阅标注帧 → 经 IVideoEncoderBackend 编码
 *   （rkmpp 硬编优先 / 软编回退）→ RTSP 推流图传。
 * - VideoSenderStub 保留供骨架冒烟测试与装配联调。
 *
 * 数据流：kAnnotatedFrame ──► IVideoSender ──► 图传 RJ45
 * 可替换边界：编码 + 图传协议适配器（IVideoEncoderBackend）。
 */
#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include "common/topic.h"
#include "video/video_frame.h"
#include "video_transmission/video_encoder.h"

namespace drone::video_transmission {

/// 图传发送配置。
struct VideoSenderConfig {
    EncoderBackendConfig encode;  ///< 编码 + 图传推流后端配置
    std::size_t input_queue = 2;  ///< 标注帧订阅队列容量（丢最旧）

    /// 编码后端注入工厂（测试用）：为空时用默认 FFmpeg 后端。
    /// 测试注入 Mock 以验证发送线程/计数逻辑而不依赖真实 RTSP 目标。
    std::function<std::unique_ptr<IVideoEncoderBackend>()> backend_factory;
};

/// 图传发送部件抽象接口。
class IVideoSender {
public:
    virtual ~IVideoSender() = default;

    // ---- 生命周期 ----
    /// 启动发送（建立图传链路、启动消费线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止发送并断开图传链路；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;

    // ---- 输入 ----
    /// 绑定标注帧输入主题（common::Topic<video::FrameHandle>）。
    virtual void SetInput(common::Topic<video::FrameHandle>& input) = 0;
    // ---- 状态查询 ----
    /// 累计成功发送帧数。
    virtual uint64_t SentFrameCount() const = 0;
    /// 累计丢弃帧数（图传拥塞）。
    virtual uint64_t DroppedFrameCount() const = 0;
    /// 累计错误次数。
    virtual uint64_t ErrorCount() const = 0;
};

/// 图传发送器（实现 IVideoSender）。
///
/// 职责：订阅 FrameCompositor 产出的标注帧（kAnnotatedFrame），经
/// IVideoEncoderBackend 编码并 RTSP 推流给图传。图传拥塞/失败只丢图传帧，
/// 不反压上游（订阅队列 kDropOldest + 引擎前端直接 `continue`）。内部使用
/// FFmpeg（avcodec/avformat/avutil），实现细节通过 PIMPL 隔离，接口头文件
/// 不依赖 FFmpeg。
///
/// 日志约定：创建/销毁、后端启动/关闭、编码/推流错误为主线关键路径日志
/// （错误节流，第 1 次 + 每满 100 次），逐帧热路径不打日志。
class VideoSender final : public IVideoSender {
public:
    explicit VideoSender(VideoSenderConfig config);
    ~VideoSender() override;

    VideoSender(const VideoSender&) = delete;
    VideoSender& operator=(const VideoSender&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInput(common::Topic<video::FrameHandle>& input) override;

    uint64_t SentFrameCount() const override;
    uint64_t DroppedFrameCount() const override;
    uint64_t ErrorCount() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class VideoSenderStub final : public IVideoSender {
public:
    VideoSenderStub();
    ~VideoSenderStub() override;

    VideoSenderStub(const VideoSenderStub&) = delete;
    VideoSenderStub& operator=(const VideoSenderStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInput(common::Topic<video::FrameHandle>& input) override;

    uint64_t SentFrameCount() const override;
    uint64_t DroppedFrameCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t sent_count_ = 0;
    uint64_t dropped_count_ = 0;
    uint64_t error_count_ = 0;
};

}  // namespace drone::video_transmission
