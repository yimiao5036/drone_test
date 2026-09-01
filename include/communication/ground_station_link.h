#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "common/topic.h"
#include "common/types.h"
#include "communication/serial_port.h"

namespace drone::communication {

/// 地面站数传链路配置。串口、身份和发送周期全部由 JSON 注入。
struct GroundStationLinkConfig {
    SerialPortConfig serial;
    uint8_t onboard_system_id = 1;
    uint8_t onboard_component_id = 191;  ///< MAV_COMP_ID_ONBOARD_COMPUTER
    uint8_t mavlink_version = 2;
    std::chrono::milliseconds heartbeat_send_interval{1000};
    std::chrono::milliseconds heartbeat_timeout{3000};
    std::chrono::milliseconds attitude_send_interval{100};
    std::chrono::milliseconds local_position_send_interval{200};
    std::chrono::milliseconds global_position_send_interval{200};
    std::chrono::milliseconds gps_send_interval{500};
    std::chrono::milliseconds extended_state_send_interval{500};
    std::chrono::milliseconds system_status_send_interval{1000};
    std::chrono::milliseconds battery_send_interval{1000};
    std::chrono::milliseconds home_send_interval{5000};
    std::size_t flight_state_queue_capacity = 2;

    void Validate() const;
};

/// 地面站通信部件抽象接口。
class IGroundStationLink {
public:
    virtual ~IGroundStationLink() = default;

    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual bool IsRunning() const = 0;
    /// 最近是否收到 MAV_TYPE_GCS 心跳。
    virtual bool IsConnected() const = 0;

    /// 目标协议当前预留；协议确定前真实实现不会发布伪造目标。
    virtual common::Topic<common::GroundStationTarget>& TargetOutput() = 0;

    /// 绑定 PX4 飞行状态快照。必须在 Start 前调用。
    virtual void SetFlightStateInput(
        common::Topic<common::FlightStateSnapshot>& flight_state) = 0;
    /// 任务状态回传接口预留，当前真实链路暂不编码自定义任务协议。
    virtual void SetMissionStatusInput(
        common::Topic<common::MissionStatus>& mission_status) = 0;
    /// 健康状态回传接口预留，当前真实链路暂不编码自定义健康协议。
    virtual void SetHealthInput(common::Topic<common::HealthStatus>& health) = 0;

    virtual uint64_t SendCount() const = 0;
    virtual uint64_t ReceiveCount() const = 0;
    virtual uint64_t ErrorCount() const = 0;
};

/// 真实地面站链路：订阅 FlightStateSnapshot，按限频策略编码标准 MAVLink 2 遥测。
class GroundStationLink final : public IGroundStationLink {
public:
    explicit GroundStationLink(GroundStationLinkConfig config);
    ~GroundStationLink() override;

    GroundStationLink(const GroundStationLink&) = delete;
    GroundStationLink& operator=(const GroundStationLink&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;
    bool IsConnected() const override;

    common::Topic<common::GroundStationTarget>& TargetOutput() override;
    void SetFlightStateInput(
        common::Topic<common::FlightStateSnapshot>& flight_state) override;
    void SetMissionStatusInput(
        common::Topic<common::MissionStatus>& mission_status) override;
    void SetHealthInput(common::Topic<common::HealthStatus>& health) override;

    uint64_t SendCount() const override;
    uint64_t ReceiveCount() const override;
    uint64_t ErrorCount() const override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

/// 骨架占位实现，保留给模块冒烟测试。
class GroundStationLinkStub final : public IGroundStationLink {
public:
    GroundStationLinkStub();
    ~GroundStationLinkStub() override;

    GroundStationLinkStub(const GroundStationLinkStub&) = delete;
    GroundStationLinkStub& operator=(const GroundStationLinkStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;
    bool IsConnected() const override;

    common::Topic<common::GroundStationTarget>& TargetOutput() override;
    void SetFlightStateInput(
        common::Topic<common::FlightStateSnapshot>& flight_state) override;
    void SetMissionStatusInput(
        common::Topic<common::MissionStatus>& mission_status) override;
    void SetHealthInput(common::Topic<common::HealthStatus>& health) override;

    uint64_t SendCount() const override;
    uint64_t ReceiveCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t send_count_ = 0;
    uint64_t receive_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::GroundStationTarget> target_output_;
};

}  // namespace drone::communication
