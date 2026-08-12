/**
 * @file video_sender.h
 * @brief 图传发送部件接口（IVideoSender）
 *
 * 属于 drone/video_transmission 模块。职责：订阅标注后的视频帧
 * （video::FrameHandle），编码并推送给图传。图传拥塞只允许丢图传帧，
 * 不得反压摄像头或感知链路。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，编码后端与图传协议在实现期接入。
 * - VideoSenderStub 为骨架占位实现：生命周期可运行，业务方法
 *   记录"未实现"节流日志并返回默认值。
 *
 * 数据流：common::Topic<FrameHandle>（标注帧）──► IVideoSender ──► 图传 RJ45
 * 可替换边界：图传协议适配器。
 */
#pragma once

#include <cstdint>

#include "common/topic.h"
#include "video/video_frame.h"

namespace drone::video_transmission {

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
