/**
 * @file camera_receiver.h
 * @brief 摄像头接收部件接口（ICameraReceiver）
 *
 * 属于 drone/video 模块。职责：TCP 建连、接收 H.265 码流、断线重连，
 * 产出 H.265 码流块并发布到输出主题。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，具体协议适配（摄像头厂商 TCP 协议）在实现期完成。
 * - CameraReceiverStub 为骨架占位实现：生命周期方法可运行，
 *   业务方法记录"未实现"节流日志并返回默认值，用于早期联调与装配验证。
 *
 * 数据流：摄像头 RJ45/TCP ──► ICameraReceiver ──► common::Topic<EncodedFrame> ──► IVideoDecoder
 * 可替换边界：视频协议适配器。
 */
#pragma once

#include <cstdint>

#include "common/topic.h"
#include "common/types.h"

namespace drone::video {

/// 摄像头接收部件抽象接口。
class ICameraReceiver {
public:
    virtual ~ICameraReceiver() = default;

    // ---- 生命周期 ----
    /// 启动接收（建连、开始接收码流）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止接收并断开连接；幂等，可重复调用。
    virtual void Stop() = 0;
    /// 是否已启动（Start 成功且未 Stop）。
    virtual bool IsRunning() const = 0;

    // ---- 状态查询 ----
    /// 当前是否已建立连接。
    virtual bool IsConnected() const = 0;
    /// 累计建连次数（断线重连计数）。
    virtual uint64_t ConnectCount() const = 0;
    /// 累计接收字节数。
    virtual uint64_t ReceivedBytes() const = 0;
    /// 累计错误次数（断线、协议错误等）。
    virtual uint64_t ErrorCount() const = 0;

    // ---- 输出 ----
    /// 码流输出主题：H.265 码流块（common::EncodedFrame）。
    /// 消费者（IVideoDecoder）通过 Subscribe 绑定。
    virtual common::Topic<common::EncodedFrame>& StreamOutput() = 0;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class CameraReceiverStub final : public ICameraReceiver {
public:
    CameraReceiverStub();
    ~CameraReceiverStub() override;

    CameraReceiverStub(const CameraReceiverStub&) = delete;
    CameraReceiverStub& operator=(const CameraReceiverStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    bool IsConnected() const override;
    uint64_t ConnectCount() const override;
    uint64_t ReceivedBytes() const override;
    uint64_t ErrorCount() const override;

    common::Topic<common::EncodedFrame>& StreamOutput() override;

private:
    bool running_ = false;
    uint64_t connect_count_ = 0;
    uint64_t received_bytes_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::EncodedFrame> stream_output_;
};

}  // namespace drone::video
