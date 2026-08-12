/**
 * @file px4_link.h
 * @brief PX4 通信部件接口（IPx4Link）
 *
 * 属于 drone/communication 模块。职责：与 PX4 飞控 MAVLink 双向通信——
 * 接收遥测并维护飞行状态快照（FlightStateSnapshot），发送 PX4 设定值
 * 与 MAVLink 命令并关联 ACK。只有本链路线程可以写飞控串口。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，MAVLink 解析、ACK 关联与 PX4 版本适配在实现期接入。
 * - Px4LinkStub 为骨架占位实现：生命周期可运行，业务方法记录
 *   "未实现"节流日志并返回默认值。
 *
 * 数据流：
 *   common::Topic<Px4Setpoint>（飞行控制器）──► IPx4Link ──► PX4 串口
 *   PX4 串口 ──► IPx4Link ──► common::Topic<FlightStateSnapshot>
 * 可替换边界：PX4 版本适配层。
 */
#pragma once

#include <cstdint>

#include "common/topic.h"
#include "common/types.h"

namespace drone::communication {

/// PX4 通信部件抽象接口。
class IPx4Link {
public:
    virtual ~IPx4Link() = default;

    // ---- 生命周期 ----
    /// 启动通信（打开串口、启动收发线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止通信并关闭串口；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;
    /// 当前是否收到 PX4 心跳（连接状态）。
    virtual bool IsConnected() const = 0;

    // ---- 输入 ----
    /// 绑定 PX4 设定值输入主题（IFlightController::SetpointOutput()）。
    virtual void SetInput(common::Topic<common::Px4Setpoint>& setpoint) = 0;

    // ---- 输出 ----
    /// 飞行状态快照输出主题：common::FlightStateSnapshot。
    virtual common::Topic<common::FlightStateSnapshot>& StateOutput() = 0;

    // ---- 命令 ----
    /// 发送 MAVLink 命令（解锁/起飞/返航等）。
    /// @param mavlink_command MAV_CMD 常量值（实现期用 mavlink 库常量，骨架期传数值）。
    /// @param params 命令参数 param1~param7。
    /// @return 是否成功入队发送。
    virtual bool SendCommand(uint16_t mavlink_command, float param1, float param2,
                             float param3, float param4, float param5,
                             float param6, float param7) = 0;

    // ---- 状态查询 ----
    /// 累计发送设定值帧数。
    virtual uint64_t SetpointSendCount() const = 0;
    /// 累计接收消息数。
    virtual uint64_t ReceiveCount() const = 0;
    /// ACK 匹配成功次数。
    virtual uint64_t AckMatchCount() const = 0;
    /// ACK 超时次数。
    virtual uint64_t AckTimeoutCount() const = 0;
    /// 累计错误次数。
    virtual uint64_t ErrorCount() const = 0;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class Px4LinkStub final : public IPx4Link {
public:
    Px4LinkStub();
    ~Px4LinkStub() override;

    Px4LinkStub(const Px4LinkStub&) = delete;
    Px4LinkStub& operator=(const Px4LinkStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;
    bool IsConnected() const override;

    void SetInput(common::Topic<common::Px4Setpoint>& setpoint) override;

    common::Topic<common::FlightStateSnapshot>& StateOutput() override;

    bool SendCommand(uint16_t mavlink_command, float param1, float param2,
                     float param3, float param4, float param5,
                     float param6, float param7) override;

    uint64_t SetpointSendCount() const override;
    uint64_t ReceiveCount() const override;
    uint64_t AckMatchCount() const override;
    uint64_t AckTimeoutCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t setpoint_send_count_ = 0;
    uint64_t receive_count_ = 0;
    uint64_t ack_match_count_ = 0;
    uint64_t ack_timeout_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::FlightStateSnapshot> state_output_;
};

}  // namespace drone::communication
