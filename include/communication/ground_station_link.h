#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "common/topic.h"
#include "common/types.h"
#include "communication/serial_port.h"

namespace drone::communication {

constexpr uint8_t kNetCaptureAircraftComponentId = 25;
constexpr uint8_t kRocketAircraftComponentId = 26;
constexpr uint8_t kGroundStationSystemId = 255;
constexpr uint8_t kGroundStationComponentId = 190;
constexpr uint16_t kTrackTargetUpdateMessageType = 65010;
constexpr uint16_t kTrackTargetAckMessageType = 65011;
constexpr uint16_t kHealthStatusMessageType = 65012;
constexpr uint16_t kMissionStatusMessageType = 65013;
/// 任务分配（65014）与分配回执（65015），见
/// docs/superpowers/specs/2026-10-22-多机任务分配与自动起飞-design.md §5。
constexpr uint16_t kMissionAssignMessageType = 65014;
constexpr uint16_t kMissionAssignAckMessageType = 65015;
/// 捕网直控发射（65016，地面站 → 指定飞机）与发射回执（65017）。
/// 只在 actuator.gcs_fire_enabled=true 时被 GcsNetFire 消费。
constexpr uint16_t kNetFireMessageType = 65016;
constexpr uint16_t kNetFireAckMessageType = 65017;
constexpr uint8_t kNetFireProtocolVersion = 1;
constexpr uint8_t kNetFireFlagConfirmed = 1U << 0U;  ///< bit0=确认发射
constexpr uint8_t kNetFireKnownFlagsMask = kNetFireFlagConfirmed;
/// 围栏区域下发（65018，地面站 → 指定飞机）与围栏状态回传（65019，飞机 → 地面站）。
/// 区域是"一群飞机共享的快照"：地面站按 zone_id/version 下发，机载缓存并自治执行，
/// 起飞前用 65014 携带的 zone_id/version 做一致性预检（2026-10-29）。
/// 65013/65014 的版本号**不变**（仍为 2/1）：围栏字段写在两帧的保留区
/// （65013 offsets 28/30/50、65014 offsets 15/17），老地面站忽略即可，Web 与
/// Python 工具无需同步升级；未填这些字段的老发送端按"未声明区域"处理。
constexpr uint16_t kFenceZoneMessageType = 65018;
constexpr uint16_t kFenceStatusMessageType = 65019;
/// 围栏区域回读（机载 → 地面站，2026-10-29）：与 65018 完全同构，
/// 把机载当前生效的区域原样回传，供地面站比对"本地定义 vs 机载实况"。
constexpr uint16_t kFenceZoneReportMessageType = 65022;
constexpr uint8_t kFenceZoneProtocolVersion = 1;
constexpr uint8_t kFenceMaxVertices = 8;  ///< 单帧上限：顶点区 21~53（marker 在 54）
/// 地面站任务指令（65020，地面站 → 指定飞机）与回执（65021，飞机 → 地面站）。
/// 与 65014 的分工：65014 是"分配任务"，65020 是对已起飞任务的即时人工指令
/// （当前只有主动返航 kReturnHome）；中止后的任务分配以 65015 aborted
/// （reason=kCancelledByGcs）同步，因此两条回执各管一件事（2026-10-29）。
constexpr uint16_t kMissionCommandMessageType = 65020;
constexpr uint16_t kMissionCommandAckMessageType = 65021;
constexpr uint8_t kMissionCommandProtocolVersion = 1;
constexpr uint8_t kMissionAssignmentProtocolVersion = 1;
constexpr uint8_t kTrackTargetProtocolVersion = 1;
/// 65012 与 65013 的协议版本独立：65013 v2 才携带 assignment/availability，
/// 65012 保持 v1；两者不再共用版本常量。
constexpr uint8_t kHealthStatusProtocolVersion = 1;
constexpr uint8_t kMissionStatusProtocolVersion = 2;
constexpr uint8_t kMissionAssignFlagAuthorized = 1U << 0U;  ///< bit0=授权自动起飞
constexpr uint8_t kMissionAssignFlagCancel = 1U << 1U;      ///< bit1=取消当前分配
constexpr uint8_t kMissionAssignKnownFlagsMask =
    kMissionAssignFlagAuthorized | kMissionAssignFlagCancel;
constexpr uint8_t kTrackTargetCoordinateFrameWgs84 = 1;
constexpr uint16_t kTrackTargetKnownFlagsMask = 0x007F;

/// TRACK_TARGET_ACK接收结果。
enum class TrackTargetAckResult : uint8_t {
    kAccepted = 0,
    kRejectedInvalidField = 1,
    kRejectedStaleOrDuplicate = 2,
    kRejectedUnsupportedVersion = 3,
    kRejectedTimeSyncUnavailable = 4,
    kRejectedNotReady = 5,
    kRejectedInternalError = 6,
};

/// TRACK_TARGET_ACK详细原因码。
enum class TrackTargetAckReason : uint16_t {
    kOk = 0,
    kSourceIdInvalid = 1,
    kTargetAddressMismatch = 2,
    kProtocolVersionUnsupported = 3,
    kBootIdInvalidOrChanged = 4,
    kUpdateSequenceStale = 5,
    kTargetIdInvalid = 6,
    kLatitudeOrLongitudeInvalid = 7,
    kValidForInvalid = 8,
    kFlagsInvalid = 9,
    kAltitudeReferenceInvalid = 10,
    kHeadingInvalid = 11,
    kAccuracyInvalid = 12,
    kTimeSyncUnavailable = 13,
    kSourceTimeInFuture = 14,
    kTargetExpired = 15,
    kRemainingValidityTooShort = 16,
    kModuleNotReady = 17,
    kInternalError = 100,
};

/// 地面站与飞机单调时钟的同步状态。
enum class GroundStationTimeSyncState : uint8_t {
    kUnsynchronized = 0,
    kAcquiring,
    kSynchronized,
    kDegraded,
};

/// TIMESYNC运行状态快照。offset语义为“地面站时钟 - 飞机时钟”。
struct GroundStationTimeSyncStatus {
    GroundStationTimeSyncState state =
        GroundStationTimeSyncState::kUnsynchronized;
    int64_t offset_ns = 0;
    uint64_t round_trip_time_ns = 0;
    uint64_t jitter_ns = 0;
    uint64_t sample_age_ms = 0;
    std::size_t valid_sample_count = 0;
    uint64_t request_count = 0;
    uint64_t response_count = 0;
    uint64_t rejected_sample_count = 0;
    uint64_t request_timeout_count = 0;
};

/// 地面站数传链路配置。串口、身份和发送周期全部由 JSON 注入。
struct GroundStationLinkConfig {
    SerialPortConfig serial;
    uint8_t aircraft_system_id = 1;      ///< 地面站链路同类别飞机编号。
    uint8_t aircraft_component_id = kNetCaptureAircraftComponentId;  ///< 飞机功能类别。
    std::string aircraft_type = "net_capture";
    uint8_t aircraft_number = 1;
    std::string callsign = "捕网-01";
    uint8_t ground_system_id = kGroundStationSystemId;
    uint8_t ground_component_id = kGroundStationComponentId;
    uint8_t mavlink_version = 2;
    std::chrono::milliseconds heartbeat_send_interval{1000};
    std::chrono::milliseconds heartbeat_timeout{3000};
    bool enable_time_sync = true;
    std::chrono::milliseconds time_sync_acquire_interval{200};
    std::chrono::milliseconds time_sync_steady_interval{1000};
    std::chrono::milliseconds time_sync_timeout{5000};
    std::chrono::milliseconds time_sync_max_rtt{300};
    std::chrono::milliseconds time_sync_max_offset_jump{100};
    std::size_t time_sync_minimum_samples = 5;
    std::size_t time_sync_window_capacity = 10;
    std::chrono::milliseconds target_minimum_valid_for{100};
    std::chrono::milliseconds target_maximum_valid_for{5000};
    std::chrono::milliseconds target_minimum_remaining_valid{100};
    std::chrono::milliseconds target_maximum_transport_delay{1000};
    std::chrono::milliseconds target_future_tolerance{200};
    std::chrono::milliseconds attitude_send_interval{100};
    std::chrono::milliseconds local_position_send_interval{200};
    std::chrono::milliseconds global_position_send_interval{200};
    std::chrono::milliseconds gps_send_interval{500};
    std::chrono::milliseconds extended_state_send_interval{500};
    std::chrono::milliseconds system_status_send_interval{1000};
    std::chrono::milliseconds battery_send_interval{1000};
    std::chrono::milliseconds home_send_interval{5000};
    std::chrono::milliseconds health_status_send_interval{1000};
    std::chrono::milliseconds mission_status_send_interval{500};
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
    /// 获取TIMESYNC状态快照；当前只观测时间，不接收目标位置。
    virtual GroundStationTimeSyncStatus GetTimeSyncStatus() const = 0;

    /// 目标协议当前预留；协议确定前真实实现不会发布伪造目标。
    virtual common::Topic<common::GroundStationTarget>& TargetOutput() = 0;

    /// 65014 任务分配解析结果。地址双字段匹配本机后才发布；
    /// 版本/标记/拦截类型等语义校验由状态机负责。
    virtual common::Topic<common::MissionAssignment>& MissionAssignmentOutput() = 0;
    /// 绑定 65015 任务分配回执输入。收到一条回执即按协议编码发送给 255/190。
    virtual void SetMissionAssignmentAckInput(
        common::Topic<common::MissionAssignmentAck>& ack) = 0;

    /// 65016 捕网直控发射指令解析结果。地址双字段匹配本机后才发布；
    /// 版本/flags/落地态等语义校验由 GcsNetFire 负责，链路层不产生动作。
    virtual common::Topic<common::NetFireCommand>& NetFireCommandOutput() = 0;
    /// 绑定 65017 发射回执输入。收到一条回执即按协议编码发送给 255/190。
    virtual void SetNetFireAckInput(common::Topic<common::NetFireAck>& ack) = 0;

    /// 65018 围栏区域下发解析结果（2026-10-29）。地址双字段匹配本机后才发布；
    /// 多边形/高度带合法性由状态机校验（非法区域被忽略并记日志）。
    virtual common::Topic<common::FenceZone>& FenceZoneOutput() = 0;
    /// 绑定 65019 围栏状态回传输入。按 V2_EXTENSION message_type=65019 编码发送。
    virtual void SetFenceStatusInput(common::Topic<common::FenceStatus>& state) = 0;
    /// 绑定 65022 围栏区域回读输入（状态机输出的当前生效区域）。
    virtual void SetFenceZoneReportInput(common::Topic<common::FenceZone>& zone) = 0;

    /// 65020 任务指令（主动返航）解析结果（2026-10-29）。地址双字段匹配本机后
    /// 才发布；协议版本/指令类型/状态合法性由状态机校验，链路层不产生动作。
    virtual common::Topic<common::MissionCommand>& MissionCommandOutput() = 0;
    /// 绑定 65021 任务指令回执输入。每条回执按 V2_EXTENSION message_type=65021 发送。
    virtual void SetMissionCommandAckInput(
        common::Topic<common::MissionCommandAck>& ack) = 0;

    /// 绑定 PX4 飞行状态快照。必须在 Start 前调用。
    virtual void SetFlightStateInput(
        common::Topic<common::FlightStateSnapshot>& flight_state) = 0;
    /// 绑定任务状态回传；协议使用 V2_EXTENSION message_type=65013。
    virtual void SetMissionStatusInput(
        common::Topic<common::MissionStatus>& mission_status) = 0;
    /// 绑定健康状态回传；协议使用 V2_EXTENSION message_type=65012。
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
    GroundStationTimeSyncStatus GetTimeSyncStatus() const override;

    common::Topic<common::GroundStationTarget>& TargetOutput() override;
    common::Topic<common::MissionAssignment>& MissionAssignmentOutput() override;
    void SetMissionAssignmentAckInput(
        common::Topic<common::MissionAssignmentAck>& ack) override;
    common::Topic<common::NetFireCommand>& NetFireCommandOutput() override;
    void SetNetFireAckInput(common::Topic<common::NetFireAck>& ack) override;
    common::Topic<common::FenceZone>& FenceZoneOutput() override;
    void SetFenceStatusInput(common::Topic<common::FenceStatus>& state) override;
    void SetFenceZoneReportInput(common::Topic<common::FenceZone>& zone) override;
    common::Topic<common::MissionCommand>& MissionCommandOutput() override;
    void SetMissionCommandAckInput(
        common::Topic<common::MissionCommandAck>& ack) override;
    /// 标记 65016 是否存在消费者（GcsNetFire 是否装配）。只用于把“开关关闭导致指令
    /// 无人处理”变成一条可自证的节流 WARN，不改变任何发射语义；必须在 Start 前调用。
    void SetNetFireConsumerPresent(bool present);
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
    GroundStationTimeSyncStatus GetTimeSyncStatus() const override;

    common::Topic<common::GroundStationTarget>& TargetOutput() override;
    common::Topic<common::MissionAssignment>& MissionAssignmentOutput() override;
    void SetMissionAssignmentAckInput(
        common::Topic<common::MissionAssignmentAck>& ack) override;
    common::Topic<common::NetFireCommand>& NetFireCommandOutput() override;
    void SetNetFireAckInput(common::Topic<common::NetFireAck>& ack) override;
    common::Topic<common::FenceZone>& FenceZoneOutput() override;
    void SetFenceStatusInput(common::Topic<common::FenceStatus>& state) override;
    void SetFenceZoneReportInput(common::Topic<common::FenceZone>& zone) override;
    common::Topic<common::MissionCommand>& MissionCommandOutput() override;
    void SetMissionCommandAckInput(
        common::Topic<common::MissionCommandAck>& ack) override;
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
    common::Topic<common::MissionAssignment> mission_assignment_output_;
    common::Topic<common::NetFireCommand> net_fire_command_output_;
    common::Topic<common::FenceZone> fence_zone_output_;
    common::Topic<common::MissionCommand> mission_command_output_;
};

}  // namespace drone::communication
