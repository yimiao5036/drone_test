/**
 * @file px4_link.h
 * @brief PX4 通信部件接口（IPx4Link）
 *
 * 属于 drone/communication 模块。职责：与 PX4 飞控 MAVLink 双向通信——
 * 接收遥测并维护飞行状态快照（FlightStateSnapshot），发送 PX4 设定值
 * 与 MAVLink 命令并关联 ACK。只有本链路线程可以写飞控串口。
 *
 * 当前实现阶段：
 * - Px4Link 已实现独占串口线程、MAVLink 1/2 心跳、连接超时、版本与飞行遥测快照；
 * - 可靠命令队列、ACK 和安全遥测请求已实现；设定值与串口自动重连后续接入；
 * - Px4LinkStub 继续保留用于骨架冒烟测试。
 *
 * 数据流：
 *   common::Topic<Px4Setpoint>（飞行控制器）──► IPx4Link ──► PX4 串口
 *   PX4 串口 ──► IPx4Link ──► common::Topic<FlightStateSnapshot>
 * 可替换边界：PX4 版本适配层。
 */
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "common/topic.h"
#include "common/types.h"
#include "communication/communication_transport.h"

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

/// MAV_CMD_SET_MESSAGE_INTERVAL 请求项。
struct MavlinkMessageIntervalRequest {
    uint32_t message_id = 0;
    int32_t interval_us = 0;
};

/// PX4 MAVLink 链路配置。设备名、身份和周期均由 JSON 注入，禁止在实现中写死。
struct Px4LinkConfig {
    std::string transport = "serial";  ///< serial 或 udp
    SerialPortConfig serial;
    UdpTransportConfig udp;
    std::string firmware_version = "1.17.0";  ///< 当前适配目标，用于日志与版本核验
    uint8_t onboard_system_id = 1;
    uint8_t onboard_component_id = 191;  ///< MAV_COMP_ID_ONBOARD_COMPUTER
    uint8_t target_system_id = 1;
    uint8_t target_component_id = 1;     ///< MAV_COMP_ID_AUTOPILOT1
    uint8_t mavlink_version = 2;         ///< 1 或 2
    std::chrono::milliseconds heartbeat_send_interval{1000};
    std::chrono::milliseconds heartbeat_timeout{3000};
    std::chrono::milliseconds telemetry_timeout{2000};
    std::chrono::milliseconds state_publish_interval{100};
    std::chrono::milliseconds reconnect_interval{1000};
    std::chrono::milliseconds command_ack_timeout{1000};
    std::chrono::milliseconds setpoint_send_interval{50};
    std::chrono::milliseconds setpoint_timeout{500};
    std::size_t setpoint_queue_capacity = 4;
    std::size_t command_queue_capacity = 16;
    std::vector<uint32_t> one_shot_message_requests;
    std::vector<MavlinkMessageIntervalRequest> message_interval_requests;

    void Validate() const;
};

/// PX4 真实通信实现：独占串口线程，心跳、版本、飞行遥测与新鲜度快照。
class Px4Link final : public IPx4Link {
public:
    explicit Px4Link(Px4LinkConfig config);
    ~Px4Link() override;

    Px4Link(const Px4Link&) = delete;
    Px4Link& operator=(const Px4Link&) = delete;

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

    /// 指定 MAVLink message ID 的累计接收数（用于硬件 smoke 诊断）。
    uint64_t MessageReceiveCount(uint32_t message_id) const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
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
