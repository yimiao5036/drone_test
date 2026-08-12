/**
 * @file video_decoder.h
 * @brief 视频解码部件接口（IVideoDecoder）
 *
 * 属于 drone/video 模块。职责：订阅 H.265 码流块，解码为图像帧
 * （当前链路为 NV12，后续按需扩展），解码帧以 video::FrameHandle
 * 形式发布，多订阅者零拷贝共享，底层缓冲最后引用释放时归还内存池。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，硬件（MPP）或软件解码后端在实现期接入。
 * - VideoDecoderStub 为骨架占位实现：生命周期可运行，业务方法
 *   记录"未实现"节流日志并返回默认值。
 *
 * 数据流：common::Topic<EncodedFrame> ──► IVideoDecoder ──► common::Topic<FrameHandle>
 * 可替换边界：软件/硬件解码后端。
 */
#pragma once

#include <cstdint>

#include "common/topic.h"
#include "common/types.h"
#include "video/video_frame.h"

namespace drone::video {

/// 视频解码部件抽象接口。
class IVideoDecoder {
public:
    virtual ~IVideoDecoder() = default;

    // ---- 生命周期 ----
    /// 启动解码（创建解码器、启动消费线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止解码并释放解码器；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;

    // ---- 输入 ----
    /// 绑定码流输入主题（ICameraReceiver::StreamOutput()）。
    virtual void SetInput(common::Topic<common::EncodedFrame>& input) = 0;

    // ---- 输出 ----
    /// 解码帧输出主题：video::FrameHandle（像素格式见 VideoFrameInfo）。
    virtual common::Topic<FrameHandle>& FrameOutput() = 0;

    // ---- 状态查询 ----
    /// 累计成功解码帧数。
    virtual uint64_t DecodedFrameCount() const = 0;
    /// 累计丢弃帧数（解码失败、超龄等）。
    virtual uint64_t DroppedFrameCount() const = 0;
    /// 累计错误次数。
    virtual uint64_t ErrorCount() const = 0;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class VideoDecoderStub final : public IVideoDecoder {
public:
    VideoDecoderStub();
    ~VideoDecoderStub() override;

    VideoDecoderStub(const VideoDecoderStub&) = delete;
    VideoDecoderStub& operator=(const VideoDecoderStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInput(common::Topic<common::EncodedFrame>& input) override;

    common::Topic<FrameHandle>& FrameOutput() override;

    uint64_t DecodedFrameCount() const override;
    uint64_t DroppedFrameCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t decoded_count_ = 0;
    uint64_t dropped_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<FrameHandle> frame_output_;
};

}  // namespace drone::video
