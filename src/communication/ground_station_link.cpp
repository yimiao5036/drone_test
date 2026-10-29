#include "communication/ground_station_link.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include "communication/mavlink_handler.h"

namespace drone::communication {
namespace {

// ============================================================================
// GroundStationLink —— 地面站数传链路真实实现。
//
// 职责：
//   - 订阅 Px4Link 的 FlightStateSnapshot，按配置限频编码标准 MAVLink 2 遥测
//     （HEARTBEAT / ATTITUDE / LOCAL_POSITION_NED / GLOBAL_POSITION_INT /
//       GPS_RAW_INT / EXTENDED_SYS_STATE / SYS_STATUS / BATTERY_STATUS /
//       HOME_POSITION）下行给地面站；
//   - 识别来源 255/190 的 MAV_TYPE_GCS 心跳，维护地面站在线状态；
//   - 实现标准 MAVLink TIMESYNC 第一阶段，估算飞机相对地面站的 offset/RTT/jitter；
//   - 通过 V2_EXTENSION(65010/65011) 接收 TRACK_TARGET_UPDATE 并回 ACK，
//     合法目标发布 GroundStationTarget，错误地址/非法字段回拒绝 ACK；
//   - 通过 V2_EXTENSION(65012/65013) 下发 HEALTH_STATUS/MISSION_STATUS；
//   - 不向 PX4 发送任何控制命令，不转发地面站到飞控的指令。
// 边界：本模块只做遥测/时间同步/目标接收/状态回传，不执行任务或飞行控制决策。
// 线程模型：一条独占通信线程读写串口，通过阻塞读 + Topic 订阅队列获取飞行快照。
// ============================================================================

// 单调时钟毫秒：用于心跳超时、样本老化等粗粒度判断。
uint64_t MonotonicMs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

// 单调时钟纳秒：用于 TIMESYNC 请求/响应的精确时间戳。
int64_t MonotonicNs() {
    return static_cast<int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

// 求两个带符号数的绝对差值，规避负数差值可能带来的符号问题。
uint64_t AbsoluteDifference(int64_t left, int64_t right) {
    return left >= right ? static_cast<uint64_t>(left - right)
                         : static_cast<uint64_t>(right - left);
}

const char* TimeSyncStateName(GroundStationTimeSyncState state) {
    switch (state) {
        case GroundStationTimeSyncState::kUnsynchronized:
            return "UNSYNCHRONIZED";
        case GroundStationTimeSyncState::kAcquiring:
            return "ACQUIRING";
        case GroundStationTimeSyncState::kSynchronized:
            return "SYNCHRONIZED";
        case GroundStationTimeSyncState::kDegraded:
            return "DEGRADED";
    }
    return "UNKNOWN";
}

const char* TrackTargetAckResultName(TrackTargetAckResult result) {
    switch (result) {
        case TrackTargetAckResult::kAccepted:
            return "ACCEPTED";
        case TrackTargetAckResult::kRejectedInvalidField:
            return "REJECTED_INVALID_FIELD";
        case TrackTargetAckResult::kRejectedStaleOrDuplicate:
            return "REJECTED_STALE_OR_DUPLICATE";
        case TrackTargetAckResult::kRejectedUnsupportedVersion:
            return "REJECTED_UNSUPPORTED_VERSION";
        case TrackTargetAckResult::kRejectedTimeSyncUnavailable:
            return "REJECTED_TIME_SYNC_UNAVAILABLE";
        case TrackTargetAckResult::kRejectedNotReady:
            return "REJECTED_NOT_READY";
        case TrackTargetAckResult::kRejectedInternalError:
            return "REJECTED_INTERNAL_ERROR";
    }
    return "UNKNOWN";
}

const char* TrackTargetAckReasonName(TrackTargetAckReason reason) {
    switch (reason) {
        case TrackTargetAckReason::kOk:
            return "OK";
        case TrackTargetAckReason::kSourceIdInvalid:
            return "SOURCE_ID_INVALID";
        case TrackTargetAckReason::kTargetAddressMismatch:
            return "TARGET_ADDRESS_MISMATCH";
        case TrackTargetAckReason::kProtocolVersionUnsupported:
            return "PROTOCOL_VERSION_UNSUPPORTED";
        case TrackTargetAckReason::kBootIdInvalidOrChanged:
            return "BOOT_ID_INVALID_OR_CHANGED";
        case TrackTargetAckReason::kUpdateSequenceStale:
            return "UPDATE_SEQUENCE_STALE";
        case TrackTargetAckReason::kTargetIdInvalid:
            return "TARGET_ID_INVALID";
        case TrackTargetAckReason::kLatitudeOrLongitudeInvalid:
            return "LATITUDE_OR_LONGITUDE_INVALID";
        case TrackTargetAckReason::kValidForInvalid:
            return "VALID_FOR_INVALID";
        case TrackTargetAckReason::kFlagsInvalid:
            return "FLAGS_INVALID";
        case TrackTargetAckReason::kAltitudeReferenceInvalid:
            return "ALTITUDE_REFERENCE_INVALID";
        case TrackTargetAckReason::kHeadingInvalid:
            return "HEADING_INVALID";
        case TrackTargetAckReason::kAccuracyInvalid:
            return "ACCURACY_INVALID";
        case TrackTargetAckReason::kTimeSyncUnavailable:
            return "TIME_SYNC_UNAVAILABLE";
        case TrackTargetAckReason::kSourceTimeInFuture:
            return "SOURCE_TIME_IN_FUTURE";
        case TrackTargetAckReason::kTargetExpired:
            return "TARGET_EXPIRED";
        case TrackTargetAckReason::kRemainingValidityTooShort:
            return "REMAINING_VALIDITY_TOO_SHORT";
        case TrackTargetAckReason::kModuleNotReady:
            return "MODULE_NOT_READY";
        case TrackTargetAckReason::kInternalError:
            return "INTERNAL_ERROR";
    }
    return "UNKNOWN";
}

bool ShouldLogThrottled(uint64_t count) {
    return count == 1 || count % 100 == 0;
}

// 串口读错误重连：连续读失败达到阈值即判定设备丢失（单次 EIO/EINTR 不触发），
// 关闭端口后按固定间隔重开。GroundStationLinkConfig 没有重连字段（配置解析在
// drone_config 模块），这里用模块内常量，阈值语义与 px4_link 保持一致。
constexpr uint32_t kSerialReadErrorReconnectThreshold = 3;
constexpr std::chrono::milliseconds kSerialReconnectInterval{500};
// 重连等待期间工作线程的睡眠，避免串口关闭后忙轮询占满一个核。
constexpr std::chrono::milliseconds kSerialReconnectWorkerPause{100};
// 遥测发送失败后的短退避：失败不推进正常周期，只按该退避重试（避免瞬态失败
// 把一帧推迟整个周期，也避免写失败时空转）。
constexpr std::chrono::milliseconds kSendRetryBackoff{100};
// 时间同步状态的低频发布周期：读超时 20ms 时每轮发布会造成 ~50Hz 的
// shared_ptr + 互斥量开销，无状态变化时降到 1Hz。
constexpr std::chrono::milliseconds kTimeSyncStatusPublishInterval{1000};
/// 65010 TRACK_TARGET_UPDATE 的协议有效数据最小长度：offset 54 的 alt_reference
/// 允许被 MAVLink2 截尾（为 0），因此最小总长度是 5 + 54 = 59，而不是 60。
constexpr uint16_t kTrackTargetUpdateMinimumPayloadLength = 54;

// 65012/65013/65014/65015 均使用 MAVLink2 短帧：
// V2_EXTENSION 固定头 5 字节 + 协议有效数据 55 字节，payload_len=60；
// 协议 offset 54 固定写入 1，避免尾部零字节被 MAVLink2 截断。
constexpr uint16_t kV2ExtensionFixedHeaderLength = 5;
constexpr uint16_t kStatusShortFramePayloadLength = 55;
constexpr uint16_t kStatusShortFrameLength =
    kV2ExtensionFixedHeaderLength + kStatusShortFramePayloadLength;
constexpr uint8_t kFrameLengthMarker = 1;
// 65018 顶点相对参考点用 int16 厘米偏移（±327m，1cm 精度），每顶点 4 字节。
// 纬度 1 度 ≈ 111320m；经度按参考纬度做 cos 缩放（场地尺度误差可忽略）。
constexpr double kDegToRad = 0.017453292519943295;
/// 65016 指令在机载队列中的最长有效期：超过该时长仍未处理即按“指令过期”拒绝，
/// 避免串口积压后补发一次过期发射。
constexpr uint32_t kNetFireCommandValidForMs = 2000;
/// 65020 指令在机载队列中的最长有效期：人工返航指令允许 1s 的排队延迟，
/// 超时仍未处理即按过期丢弃，避免串口积压后补执行一次迟到的返航。
constexpr uint32_t kMissionCommandValidForMs = 1000;

// 把浮点物理量乘 scale 换算成 MAVLink 整数字段，跳过非有限值并夹逼到 [min,max]。
template <typename T>
T ClampRounded(float value, float scale, T minimum, T maximum) {
    if (!std::isfinite(value)) {
        return minimum;
    }
    const double scaled = std::round(static_cast<double>(value) * scale);
    return static_cast<T>(std::clamp(scaled, static_cast<double>(minimum),
                                     static_cast<double>(maximum)));
}

// double 版本，用于围栏回读的高度/顶点偏移换算：非有限值（NaN）返回 0，超量程
// 夹逼到 int16 边界，避免 static_cast<int16_t> 在越界时发生实现定义的回绕。
template <typename T>
T ClampRoundedDouble(double value, double scale, T minimum, T maximum) {
    if (!std::isfinite(value)) {
        return static_cast<T>(0);
    }
    const double scaled = std::round(value * scale);
    return static_cast<T>(std::clamp(scaled, static_cast<double>(minimum),
                                     static_cast<double>(maximum)));
}

// 航向弧度 → 0..35999 厘度；非有限值返回 UINT16_MAX（表示未知）。
uint16_t HeadingCentidegrees(float yaw_rad) {
    if (!std::isfinite(yaw_rad)) {
        return std::numeric_limits<uint16_t>::max();
    }
    constexpr double kRadiansToDegrees = 57.295779513082320876;
    double degrees = std::fmod(static_cast<double>(yaw_rad) * kRadiansToDegrees, 360.0);
    if (degrees < 0.0) {
        degrees += 360.0;
    }
    return static_cast<uint16_t>(std::lround(degrees * 100.0)) % 36000U;
}

// 小端读取 V2_EXTENSION payload 指定偏移的字段（整型直接 memcpy 到目标小端）。
template <typename T>
T ReadLe(const uint8_t* payload, std::size_t offset) {
    T value{};
    std::memcpy(&value, payload + offset, sizeof(T));
    return value;
}

// 小端写入 V2_EXTENSION payload 指定偏移的字段。
template <typename T>
void WriteLe(std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN>& payload,
             std::size_t offset, T value) {
    std::memcpy(payload.data() + offset, &value, sizeof(T));
}

}  // namespace

// 地面站配置校验：身份/类型匹配、来源固定 255/190、版本、各周期与门限为正数。
void GroundStationLinkConfig::Validate() const {
    serial.Validate();
    // aircraft_system_id 与 aircraft_number 都是同类型编号，必须一致且非广播(255)。
    if (aircraft_system_id == 0 || aircraft_system_id == 255) {
        throw std::invalid_argument("地面站链路 aircraft_system_id 必须在1~254之间");
    }
    if (aircraft_number == 0 || aircraft_number == 255) {
        throw std::invalid_argument("地面站链路 aircraft_number 必须在1~254之间");
    }
    if (aircraft_system_id != aircraft_number) {
        throw std::invalid_argument("地面站链路 aircraft_system_id 必须等于 aircraft_number");
    }
    if (callsign.empty()) {
        throw std::invalid_argument("地面站链路 callsign 不能为空");
    }
    if (aircraft_type == "net_capture") {
        if (aircraft_component_id != kNetCaptureAircraftComponentId) {
            throw std::invalid_argument("net_capture 必须使用 aircraft_component_id=25");
        }
    } else if (aircraft_type == "rocket") {
        if (aircraft_component_id != kRocketAircraftComponentId) {
            throw std::invalid_argument("rocket 必须使用 aircraft_component_id=26");
        }
    } else {
        throw std::invalid_argument("未知地面站飞机类型，当前仅支持 net_capture 或 rocket");
    }
    if (ground_system_id != kGroundStationSystemId ||
        ground_component_id != kGroundStationComponentId) {
        throw std::invalid_argument("地面站来源身份当前必须固定为255/190");
    }
    if (mavlink_version != 1 && mavlink_version != 2) {
        throw std::invalid_argument("地面站链路 MAVLink 版本必须是1或2");
    }
    if (enable_time_sync && mavlink_version != 2) {
        throw std::invalid_argument(
            "TIMESYNC多机目标字段需要地面站链路使用MAVLink 2");
    }
    if (heartbeat_send_interval.count() <= 0 || heartbeat_timeout.count() <= 0 ||
        time_sync_acquire_interval.count() <= 0 ||
        time_sync_steady_interval.count() <= 0 ||
        time_sync_timeout.count() <= 0 || time_sync_max_rtt.count() <= 0 ||
        time_sync_max_offset_jump.count() <= 0 ||
        attitude_send_interval.count() <= 0 ||
        local_position_send_interval.count() <= 0 ||
        global_position_send_interval.count() <= 0 || gps_send_interval.count() <= 0 ||
        extended_state_send_interval.count() <= 0 ||
        system_status_send_interval.count() <= 0 ||
        battery_send_interval.count() <= 0 || home_send_interval.count() <= 0 ||
        health_status_send_interval.count() <= 0 ||
        mission_status_send_interval.count() <= 0) {
        throw std::invalid_argument("地面站链路发送周期和心跳超时必须为正数");
    }
    if (flight_state_queue_capacity == 0) {
        throw std::invalid_argument("地面站飞行状态订阅队列容量必须大于0");
    }
    if (target_minimum_valid_for.count() <= 0 ||
        target_maximum_valid_for.count() <= 0 ||
        target_minimum_remaining_valid.count() <= 0 ||
        target_maximum_transport_delay.count() <= 0 ||
        target_future_tolerance.count() <= 0) {
        throw std::invalid_argument("地面站目标输入有效期和延迟门限必须为正数");
    }
    if (target_minimum_valid_for > target_maximum_valid_for) {
        throw std::invalid_argument("地面站目标最小有效期不得大于最大有效期");
    }
    if (target_minimum_remaining_valid > target_maximum_valid_for) {
        throw std::invalid_argument("地面站目标最小剩余有效期不得大于最大有效期");
    }
    if (time_sync_minimum_samples == 0 || time_sync_window_capacity == 0 ||
        time_sync_minimum_samples > time_sync_window_capacity) {
        throw std::invalid_argument(
            "TIMESYNC最小样本数必须大于0且不超过窗口容量");
    }
    if (time_sync_max_rtt >= time_sync_timeout) {
        throw std::invalid_argument("TIMESYNC最大RTT必须小于同步超时");
    }
}

// PIMPL：真实实现封装在 Impl，对外只暴露 GroundStationLink 的稳定接口。
// 内部持有串口、独占 MavlinkHandler（避免与 PX4 链路共享序号/半包）与运行状态。
class GroundStationLink::Impl final {
public:
    explicit Impl(GroundStationLinkConfig config)
        : config_(std::move(config)),
          serial_(config_.serial),
          mavlink_(config_.mavlink_version == 1 ? MavlinkVersion::kV1
                                                : MavlinkVersion::kV2) {
        config_.Validate();
        SPDLOG_INFO("地面站通信部件创建: serial={}@{} aircraft={}/{} type={} callsign={} gcs={}/{} mavlink={} timesync={}",
                    config_.serial.device, config_.serial.baud_rate,
                    config_.aircraft_system_id, config_.aircraft_component_id,
                    config_.aircraft_type, config_.callsign,
                    config_.ground_system_id, config_.ground_component_id,
                    config_.mavlink_version, config_.enable_time_sync);
    }

    ~Impl() {
        Stop();
        SPDLOG_INFO("地面站通信部件销毁: serial={}", config_.serial.device);
    }

    // 启动：幂等。要求已绑定 FlightStateSnapshot Topic，随后打开串口、复位
    // MAVLink 状态、订阅飞行快照，并启动工作线程。任一步失败则回滚并返回 false。
    bool Start() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (running_.load(std::memory_order_acquire)) {
            return true;
        }
        if (flight_state_topic_ == nullptr) {
            RecordError("地面站链路启动失败：未绑定 FlightStateSnapshot Topic");
            return false;
        }

        try {
            serial_.Open();
            serial_.Flush();  // 清空残留字节，避免把旧数据当新帧
            mavlink_.Reset();
            flight_state_subscription_ = flight_state_topic_->Subscribe(
                config_.flight_state_queue_capacity,
                common::Topic<common::FlightStateSnapshot>::OverflowPolicy::kDropOldest);
            if (mission_status_topic_ != nullptr) {
                mission_status_subscription_ = mission_status_topic_->Subscribe(
                    2, common::Topic<common::MissionStatus>::OverflowPolicy::kDropOldest);
            }
            if (health_topic_ != nullptr) {
                health_subscription_ = health_topic_->Subscribe(
                    2, common::Topic<common::HealthStatus>::OverflowPolicy::kDropOldest);
            }
            if (mission_assignment_ack_topic_ != nullptr) {
                mission_assignment_ack_subscription_ =
                    mission_assignment_ack_topic_->Subscribe(
                        8, common::Topic<common::MissionAssignmentAck>::OverflowPolicy::
                               kDropOldest);
            }
            if (net_fire_ack_topic_ != nullptr) {
                net_fire_ack_subscription_ = net_fire_ack_topic_->Subscribe(
                    8,
                    common::Topic<common::NetFireAck>::OverflowPolicy::kDropOldest);
            }
            if (fence_status_topic_ != nullptr) {
                fence_status_subscription_ = fence_status_topic_->Subscribe(
                    2, common::Topic<common::FenceStatus>::OverflowPolicy::kDropOldest);
            }
            if (fence_zone_report_topic_ != nullptr) {
                fence_zone_report_subscription_ = fence_zone_report_topic_->Subscribe(
                    4, common::Topic<common::FenceZone>::OverflowPolicy::kDropOldest);
            }
            if (mission_command_ack_topic_ != nullptr) {
                mission_command_ack_subscription_ =
                    mission_command_ack_topic_->Subscribe(
                        4, common::Topic<common::MissionCommandAck>::OverflowPolicy::
                               kDropOldest);
            }
            latest_state_.reset();
            latest_health_.reset();
            latest_mission_status_.reset();
            last_ground_station_boot_id_ = 0;
            last_target_update_seq_ = 0;
            last_gcs_heartbeat_ms_ = 0;
            // 串口重连状态必须随启动复位，否则上一次 Stop 时的待重连标志会
            // 让新线程一进来就跳过读循环。
            serial_reconnect_pending_ = false;
            serial_read_error_streak_ = 0;
            serial_reconnect_attempts_ = 0;
            next_serial_reconnect_ms_ = 0;
            ResetTimeSync(true);
            connected_.store(false, std::memory_order_release);
            running_.store(true, std::memory_order_release);
            worker_ = std::thread(&Impl::WorkerLoop, this);
        } catch (const std::exception& error) {
            running_.store(false, std::memory_order_release);
            connected_.store(false, std::memory_order_release);
            flight_state_subscription_.Reset();
            mission_assignment_ack_subscription_.Reset();
            net_fire_ack_subscription_.Reset();
            fence_status_subscription_.Reset();
            fence_zone_report_subscription_.Reset();
            mission_command_ack_subscription_.Reset();
            serial_.Close();
            RecordError((std::string("地面站通信启动失败: ") + error.what()).c_str());
            return false;
        }
        SPDLOG_INFO("地面站通信启动: serial={}@{}", config_.serial.device,
                    config_.serial.baud_rate);
        return true;
    }

    // 停止：置位停止标志并等待工作线程退出，再关闭串口、清订阅与同步状态。幂等。
    void Stop() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (!running_.exchange(false, std::memory_order_acq_rel)) {
            return;
        }
        if (worker_.joinable()) {
            worker_.join();
        }
        flight_state_subscription_.Reset();
        mission_status_subscription_.Reset();
        health_subscription_.Reset();
        mission_assignment_ack_subscription_.Reset();
        net_fire_ack_subscription_.Reset();
        fence_status_subscription_.Reset();
        fence_zone_report_subscription_.Reset();
        mission_command_ack_subscription_.Reset();
        latest_state_.reset();
        latest_health_.reset();
        latest_mission_status_.reset();
        serial_.Close();
        serial_reconnect_pending_ = false;
        serial_read_error_streak_ = 0;
        next_serial_reconnect_ms_ = 0;
        connected_.store(false, std::memory_order_release);
        ResetTimeSync(false);
        SPDLOG_INFO("地面站通信停止: serial={}", config_.serial.device);
    }

    bool IsRunning() const { return running_.load(std::memory_order_acquire); }
    bool IsConnected() const { return connected_.load(std::memory_order_acquire); }

    GroundStationTimeSyncStatus GetTimeSyncStatus() const {
        std::lock_guard<std::mutex> lock(time_sync_status_mutex_);
        return time_sync_status_;
    }

    common::Topic<common::GroundStationTarget>& TargetOutput() { return target_output_; }

    common::Topic<common::MissionAssignment>& MissionAssignmentOutput() {
        return mission_assignment_output_;
    }

    // 绑定 65015 回执输入。必须在 Start 前调用，运行中不允许改绑。
    void SetMissionAssignmentAckInput(common::Topic<common::MissionAssignmentAck>& topic) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (running_.load(std::memory_order_acquire)) {
            RecordError("地面站 MissionAssignmentAck Topic 必须在 Start 前绑定");
            return;
        }
        mission_assignment_ack_topic_ = &topic;
    }

    common::Topic<common::NetFireCommand>& NetFireCommandOutput() {
        return net_fire_command_output_;
    }

    // 绑定 65017 发射回执输入。必须在 Start 前调用，运行中不允许改绑。
    void SetNetFireAckInput(common::Topic<common::NetFireAck>& topic) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (running_.load(std::memory_order_acquire)) {
            RecordError("地面站 NetFireAck Topic 必须在 Start 前绑定");
            return;
        }
        net_fire_ack_topic_ = &topic;
    }

    // 记录 65016 是否有人消费。装配层在 GcsNetFire 未创建时置 false，
    // 让链路层能就“指令已发布但无人处理”给出可自证的 WARN。
    void SetNetFireConsumerPresent(bool present) {
        net_fire_consumer_present_ = present;
    }

    common::Topic<common::FenceZone>& FenceZoneOutput() {
        return fence_zone_output_;
    }

    // 绑定 65019 围栏状态回传输入。必须在 Start 前调用，运行中不允许改绑。
    void SetFenceStatusInput(common::Topic<common::FenceStatus>& topic) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (running_.load(std::memory_order_acquire)) {
            RecordError("地面站 FenceStatus Topic 必须在 Start 前绑定");
            return;
        }
        fence_status_topic_ = &topic;
    }

    // 绑定 65022 围栏区域回读输入。必须在 Start 前调用，运行中不允许改绑。
    void SetFenceZoneReportInput(common::Topic<common::FenceZone>& topic) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (running_.load(std::memory_order_acquire)) {
            RecordError("地面站 FenceZone Report Topic 必须在 Start 前绑定");
            return;
        }
        fence_zone_report_topic_ = &topic;
    }

    // 65020 任务指令解析结果输出（地址双字段匹配本机后才发布）。
    common::Topic<common::MissionCommand>& MissionCommandOutput() {
        return mission_command_output_;
    }

    // 绑定 65021 任务指令回执输入。必须在 Start 前调用，运行中不允许改绑。
    void SetMissionCommandAckInput(common::Topic<common::MissionCommandAck>& topic) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (running_.load(std::memory_order_acquire)) {
            RecordError("地面站 MissionCommandAck Topic 必须在 Start 前绑定");
            return;
        }
        mission_command_ack_topic_ = &topic;
    }

    // 绑定飞行状态输入。必须在 Start 前调用，运行中不允许改绑。
    void SetFlightStateInput(common::Topic<common::FlightStateSnapshot>& topic) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (running_.load(std::memory_order_acquire)) {
            RecordError("地面站 FlightStateSnapshot Topic 必须在 Start 前绑定");
            return;
        }
        flight_state_topic_ = &topic;
    }

    void SetMissionStatusInput(common::Topic<common::MissionStatus>& topic) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (running_.load(std::memory_order_acquire)) {
            RecordError("地面站 MissionStatus Topic 必须在 Start 前绑定");
            return;
        }
        mission_status_topic_ = &topic;
    }

    void SetHealthInput(common::Topic<common::HealthStatus>& topic) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (running_.load(std::memory_order_acquire)) {
            RecordError("地面站 HealthStatus Topic 必须在 Start 前绑定");
            return;
        }
        health_topic_ = &topic;
    }

    uint64_t SendCount() const { return send_count_.load(std::memory_order_relaxed); }
    uint64_t ReceiveCount() const {
        return receive_count_.load(std::memory_order_relaxed);
    }
    uint64_t ErrorCount() const {
        return error_count_.load(std::memory_order_relaxed);
    }

private:
    using FlightSubscription =
        common::Topic<common::FlightStateSnapshot>::Subscription;
    using HealthSubscription = common::Topic<common::HealthStatus>::Subscription;
    using MissionSubscription = common::Topic<common::MissionStatus>::Subscription;
    using AssignmentAckSubscription =
        common::Topic<common::MissionAssignmentAck>::Subscription;
    using NetFireAckSubscription = common::Topic<common::NetFireAck>::Subscription;
    using FenceStatusSubscription = common::Topic<common::FenceStatus>::Subscription;
    using FenceZoneReportSubscription = common::Topic<common::FenceZone>::Subscription;
    using MissionCommandAckSubscription =
        common::Topic<common::MissionCommandAck>::Subscription;

    struct Deadlines {
        std::chrono::steady_clock::time_point heartbeat;
        std::chrono::steady_clock::time_point time_sync;
        std::chrono::steady_clock::time_point health_status;
        std::chrono::steady_clock::time_point mission_status;
        std::chrono::steady_clock::time_point attitude;
        std::chrono::steady_clock::time_point local_position;
        std::chrono::steady_clock::time_point global_position;
        std::chrono::steady_clock::time_point gps;
        std::chrono::steady_clock::time_point extended_state;
        std::chrono::steady_clock::time_point system_status;
        std::chrono::steady_clock::time_point battery;
        std::chrono::steady_clock::time_point home;
    };

    // 进入串口重连：关闭端口、摘掉连接状态并准备按间隔重开。
    // 只在 WorkerLoop 调用。重连后必须彻底清掉半包缓冲与对端身份相关状态，
    // 否则旧的 boot_id/序号/半包数据会污染新链路（协议 §6）。
    void EnterSerialReconnect(const char* reason, uint64_t now_ms) {
        if (serial_reconnect_pending_) {
            return;
        }
        serial_reconnect_pending_ = true;
        serial_read_error_streak_ = 0;
        serial_reconnect_attempts_ = 0;
        next_serial_reconnect_ms_ =
            now_ms + static_cast<uint64_t>(kSerialReconnectInterval.count());
        connected_.store(false, std::memory_order_release);
        serial_.Close();
        mavlink_.Reset();
        latest_state_.reset();
        latest_health_.reset();
        latest_mission_status_.reset();
        last_ground_station_boot_id_ = 0;
        last_target_update_seq_ = 0;
        last_gcs_heartbeat_ms_ = 0;
        ResetTimeSync(false);
        const uint64_t count = ++serial_reconnect_count_;
        if (ShouldLogThrottled(count)) {
            SPDLOG_WARN("地面站串口链路中断，进入重连: reason={} device={} "
                        "重试间隔={}ms 累计 {}",
                        reason, config_.serial.device, kSerialReconnectInterval.count(),
                        count);
        }
    }

    // 按固定间隔尝试重开串口；成功后重置解析器并等待地面站心跳，失败继续退避。
    // 只在 WorkerLoop 调用。
    void TrySerialReconnect(uint64_t now_ms) {
        if (!serial_reconnect_pending_ || now_ms < next_serial_reconnect_ms_) {
            return;
        }
        const uint64_t attempts = ++serial_reconnect_attempts_;
        try {
            serial_.Open();
            serial_.Flush();
            mavlink_.Reset();
            serial_reconnect_pending_ = false;
            serial_read_error_streak_ = 0;
            serial_reconnect_attempts_ = 0;
            connected_.store(false, std::memory_order_release);
            SPDLOG_INFO("地面站串口重连成功: device={} 尝试={} 次，等待地面站心跳",
                        config_.serial.device, attempts);
        } catch (const std::exception& error) {
            next_serial_reconnect_ms_ =
                now_ms + static_cast<uint64_t>(kSerialReconnectInterval.count());
            const uint64_t count = ++serial_reconnect_failure_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("地面站串口重连失败: device={} 尝试={} 次 error={}"
                            "（{}ms 后重试）累计 {}",
                            config_.serial.device, attempts, error.what(),
                            kSerialReconnectInterval.count(), count);
            }
        }
    }

    // 工作线程主循环：阻塞读串口 → 增量解析 MAVLink → 消费飞行快照 →
    // 心跳/时间同步超时检查 → 各类遥测按各自截止时间发送。
    void WorkerLoop() {
        const auto now = std::chrono::steady_clock::now();
        Deadlines deadlines{now, now, now, now, now, now, now, now, now, now, now, now};
        std::array<uint8_t, 512> read_buffer{};

        while (running_.load(std::memory_order_acquire)) {
            // 串口待重连：端口已关闭，按固定间隔重试打开；期间只维护超时判断，
            // 不读不写，避免对无效 fd 反复系统调用。
            if (serial_reconnect_pending_) {
                const uint64_t reconnect_ms = MonotonicMs();
                TrySerialReconnect(reconnect_ms);
                CheckHeartbeatTimeout(reconnect_ms);
                CheckTimeSyncTimeout(reconnect_ms);
                std::this_thread::sleep_for(kSerialReconnectWorkerPause);
                continue;
            }
            // 一次读取尽可能多的串口字节并交给 MavlinkHandler 增量解析。
            const std::ptrdiff_t count = serial_.Read(read_buffer.data(), read_buffer.size());
            if (count > 0) {
                serial_read_error_streak_ = 0;
                mavlink_.Feed(read_buffer.data(), static_cast<std::size_t>(count),
                              [this](const mavlink_message_t& message) {
                                  HandleMessage(message);
                              });
            } else if (count < 0) {
                const uint64_t error_count =
                    error_count_.fetch_add(1, std::memory_order_relaxed) + 1;
                // 只计数 + sleep 会让链路永久失效：设备被拔出/波特率失配后
                // Read 会一直返回 -1。连续达到阈值即关闭端口进入重连状态机。
                if (++serial_read_error_streak_ >= kSerialReadErrorReconnectThreshold) {
                    if (ShouldLogThrottled(error_count)) {
                        SPDLOG_WARN("地面站串口连续读取错误: device={} 连续={}，累计 {}",
                                    config_.serial.device, serial_read_error_streak_,
                                    error_count);
                    }
                    EnterSerialReconnect("连续串口读取错误", MonotonicMs());
                    continue;
                }
                std::this_thread::sleep_for(config_.serial.read_timeout);
            } else {
                serial_read_error_streak_ = 0;
            }

            // 把各输入队列里最新的状态捞出，并判断是否有“事件”需要立即下行。
            DrainFlightState();
            DrainStatusInputs();
            DrainMissionAssignmentAcks();
            DrainNetFireAcks();
            DrainFenceStatuses();
            DrainFenceZoneReports();
            DrainMissionCommandAcks();
            const uint64_t current_ms = MonotonicMs();
            CheckHeartbeatTimeout(current_ms);
            CheckTimeSyncTimeout(current_ms);
            const auto current = std::chrono::steady_clock::now();
            if (current >= deadlines.heartbeat) {
                SendHeartbeat();
                deadlines.heartbeat = current + config_.heartbeat_send_interval;
            }
            if (config_.enable_time_sync &&
                connected_.load(std::memory_order_acquire) &&
                current >= deadlines.time_sync) {
                SendTimeSyncRequest();
                deadlines.time_sync = current + TimeSyncSendInterval();
            }
            if (latest_health_) {
                SendIfDue(current, deadlines.health_status,
                          config_.health_status_send_interval,
                          [&] { return SendHealthStatus(*latest_health_); });
            }
            if (latest_mission_status_) {
                SendIfDue(current, deadlines.mission_status,
                          config_.mission_status_send_interval,
                          [&] { return SendMissionStatus(*latest_mission_status_); });
            }
            if (!latest_state_) {
                continue;
            }
            const auto& state = *latest_state_;
            SendIfDue(current, deadlines.attitude, config_.attitude_send_interval,
                      [&] { return SendAttitude(state); });
            SendIfDue(current, deadlines.local_position,
                      config_.local_position_send_interval,
                      [&] { return SendLocalPosition(state); });
            SendIfDue(current, deadlines.global_position,
                      config_.global_position_send_interval,
                      [&] { return SendGlobalPosition(state); });
            SendIfDue(current, deadlines.gps, config_.gps_send_interval,
                      [&] { return SendGps(state); });
            SendIfDue(current, deadlines.extended_state,
                      config_.extended_state_send_interval,
                      [&] { return SendExtendedState(state); });
            SendIfDue(current, deadlines.system_status,
                      config_.system_status_send_interval,
                      [&] { return SendSystemStatus(state); });
            SendIfDue(current, deadlines.battery, config_.battery_send_interval,
                      [&] { return SendBatteryStatus(state); });
            SendIfDue(current, deadlines.home, config_.home_send_interval,
                      [&] { return SendHomePosition(state); });
        }
    }

    // 到期的遥测发送；每类维护各自 deadline，按 JSON 周期限频。
    // 发送失败（编码为空或写串口失败）不推进正常周期，只按短退避重试：
    // 一次瞬态失败不应该把该帧推迟一整个周期，失败本身由 EncodeAndWrite 计入
    // ErrorCount 并被上层可观测。
    template <typename Sender>
    void SendIfDue(const std::chrono::steady_clock::time_point& now,
                   std::chrono::steady_clock::time_point& deadline,
                   std::chrono::milliseconds interval, Sender&& sender) {
        if (now < deadline) {
            return;
        }
        if (!sender()) {
            deadline = now + std::min(interval, kSendRetryBackoff);
            return;
        }
        deadline = now + interval;
    }

    // 消费飞行快照队列；对比新旧状态，若关键字段变化则立即发送关键遥测，
    // 以减少地面站对该事件的显示延迟，不必等周期到点。
    void DrainFlightState() {
        while (auto message = flight_state_subscription_.TryTake()) {
            const common::FlightStateSnapshot& next = **message;
            const bool event = !latest_state_ ||
                               next.connected != latest_state_->connected ||
                               next.armed != latest_state_->armed ||
                               next.landed_state_valid != latest_state_->landed_state_valid ||
                               next.landed != latest_state_->landed ||
                               next.base_mode != latest_state_->base_mode ||
                               next.custom_mode != latest_state_->custom_mode ||
                               next.gps_state_valid != latest_state_->gps_state_valid ||
                               next.gps_fix != latest_state_->gps_fix ||
                               next.battery_valid != latest_state_->battery_valid ||
                               next.home_valid != latest_state_->home_valid;
            latest_state_ = next;
            if (event) {
                SendHeartbeat();
                (void)SendExtendedState(next);
                (void)SendSystemStatus(next);
                (void)SendBatteryStatus(next);
                (void)SendHomePosition(next);
            }
        }
    }

    // 消费健康和任务状态队列。只保留最新快照；关键字段变化时立即发送，
    // 普通周期发送由 WorkerLoop 的 deadline 控制。
    void DrainStatusInputs() {
        while (auto message = health_subscription_.TryTake()) {
            const common::HealthStatus& next = **message;
            const bool event = !latest_health_ ||
                               next.link_health_bits != latest_health_->link_health_bits ||
                               next.device_health_bits != latest_health_->device_health_bits ||
                               next.data_freshness_bits != latest_health_->data_freshness_bits ||
                               next.error_bits != latest_health_->error_bits ||
                               next.timeout_event_count != latest_health_->timeout_event_count ||
                               next.cpu_load_pct != latest_health_->cpu_load_pct;
            latest_health_ = next;
            if (event) {
                (void)SendHealthStatus(next);
            }
        }
        while (auto message = mission_status_subscription_.TryTake()) {
            const common::MissionStatus& next = **message;
            const bool event = !latest_mission_status_ ||
                               next.state != latest_mission_status_->state ||
                               next.control_source != latest_mission_status_->control_source ||
                               next.task_phase != latest_mission_status_->task_phase ||
                               next.active_warning_bits != latest_mission_status_->active_warning_bits ||
                               next.interception_authorized != latest_mission_status_->interception_authorized ||
                               next.power_status_bits != latest_mission_status_->power_status_bits ||
                               // v2 分配字段变化同样立即下行，调度器据此更新可用状态。
                               next.assignment_id != latest_mission_status_->assignment_id ||
                               next.target_id != latest_mission_status_->target_id ||
                               next.availability != latest_mission_status_->availability ||
                               next.interceptor_type != latest_mission_status_->interceptor_type ||
                               // v3 围栏字段变化同样立即下行（地面站据此更新图层与告警）。
                               next.zone_id != latest_mission_status_->zone_id ||
                               next.zone_version != latest_mission_status_->zone_version ||
                               next.fence_state != latest_mission_status_->fence_state;
            latest_mission_status_ = next;
            if (event) {
                (void)SendMissionStatus(next);
            }
        }
    }

    // 逐条消费 65015 回执输入。分配回执是事件消息，不能只保留最新一条。
    void DrainMissionAssignmentAcks() {
        while (auto message = mission_assignment_ack_subscription_.TryTake()) {
            (void)SendMissionAssignmentAck(**message);
        }
    }

    // 逐条消费 65017 发射回执输入。发射回执同样是事件消息，不能只留最新一条。
    void DrainNetFireAcks() {
        while (auto message = net_fire_ack_subscription_.TryTake()) {
            (void)SendNetFireAck(**message);
        }
    }

    // 消费 65019 围栏状态输入：只保留最新一条（状态是周期性快照，不是事件）。
    void DrainFenceStatuses() {
        while (auto message = fence_status_subscription_.TryTake()) {
            (void)SendFenceStatus(**message);
        }
    }

    // 消费 65022 区域回读输入：区域是持久快照，只发最新一份。
    void DrainFenceZoneReports() {
        while (auto message = fence_zone_report_subscription_.TryTake()) {
            (void)SendFenceZoneReport(**message);
        }
    }

    // 消费 65021 任务指令回执输入：每条都发（回执是一对一事件，不能合并）。
    void DrainMissionCommandAcks() {
        while (auto message = mission_command_ack_subscription_.TryTake()) {
            (void)SendMissionCommandAck(**message);
        }
    }

    // 分发已解析的 MAVLink 消息；目前只处理心跳、TIMESYNC 与 V2_EXTENSION 目标。
    void HandleMessage(const mavlink_message_t& message) {
        receive_count_.fetch_add(1, std::memory_order_relaxed);
        if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            HandleHeartbeat(message);
        } else if (message.msgid == MAVLINK_MSG_ID_TIMESYNC) {
            HandleTimeSync(message);
        } else if (message.msgid == MAVLINK_MSG_ID_V2_EXTENSION) {
            HandleV2Extension(message);
        }
    }

    // 只接受来源 255/190 且类型为 MAV_TYPE_GCS 的心跳；首次建立时记 INFO 并置在线。
    void HandleHeartbeat(const mavlink_message_t& message) {
        mavlink_heartbeat_t heartbeat{};
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        if (heartbeat.type != MAV_TYPE_GCS ||
            message.sysid != config_.ground_system_id ||
            message.compid != config_.ground_component_id) {
            return;
        }
        last_gcs_heartbeat_ms_ = MonotonicMs();
        if (!connected_.exchange(true, std::memory_order_acq_rel)) {
            SPDLOG_INFO("地面站心跳建立: system={} component={}", message.sysid,
                        message.compid);
        }
    }

    // 处理地面站 TIMESYNC：tc1==0 表示对端请求，我们定向/广播回复；
    // 否则是本机请求的响应，校验目标身份后进入估计。
    void HandleTimeSync(const mavlink_message_t& message) {
        if (!config_.enable_time_sync ||
            message.sysid != config_.ground_system_id ||
            message.compid != config_.ground_component_id) {
            return;
        }

        mavlink_timesync_t time_sync{};
        mavlink_msg_timesync_decode(&message, &time_sync);
        if (time_sync.tc1 == 0) {
            const bool broadcast = time_sync.target_system == 0 &&
                                   time_sync.target_component == 0;
            const bool addressed =
                time_sync.target_system == config_.aircraft_system_id &&
                time_sync.target_component == config_.aircraft_component_id;
            if (broadcast || addressed) {
                SendTimeSyncResponse(time_sync.ts1);
            }
            return;
        }

        if (time_sync.target_system != config_.aircraft_system_id ||
            time_sync.target_component != config_.aircraft_component_id) {
            ++time_sync_rejected_sample_count_;
            PublishTimeSyncStatus(MonotonicNs());
            return;
        }
        HandleTimeSyncResponse(time_sync);
    }

    // TRACK_TARGET_UPDATE 的有效负载镜像（小端，当前用前 55 字节）。
    struct TrackTargetUpdatePayload {
        uint64_t source_time_ms = 0;
        uint32_t ground_station_boot_id = 0;
        uint32_t update_seq = 0;
        uint32_t target_id = 0;
        uint32_t valid_for_ms = 0;
        int32_t latitude_1e7 = 0;
        int32_t longitude_1e7 = 0;
        int32_t altitude_mm = 0;
        int16_t velocity_north_cms = 0;
        int16_t velocity_east_cms = 0;
        int16_t velocity_down_cms = 0;
        uint16_t heading_cd = 0;
        uint16_t horizontal_accuracy_cm = 0;
        uint16_t vertical_accuracy_cm = 0;
        uint16_t flags = 0;
        uint8_t target_system = 0;
        uint8_t target_component = 0;
        uint8_t coordinate_frame = 0;
        uint8_t protocol_version = 0;
        uint8_t alt_reference = 0;
    };

    // 处理 V2_EXTENSION：只收地面站来源，按 message_type 分发目标更新或任务分配。
    void HandleV2Extension(const mavlink_message_t& message) {
        if (message.sysid != config_.ground_system_id ||
            message.compid != config_.ground_component_id) {
            return;
        }
        mavlink_v2_extension_t extension{};
        mavlink_msg_v2_extension_decode(&message, &extension);
        const uint64_t count = ++target_v2_extension_count_;
        if (ShouldLogThrottled(count)) {
            SPDLOG_INFO("收到地面站V2_EXTENSION: type={} target={}/{} count={}",
                        extension.message_type, extension.target_system,
                        extension.target_component, count);
        }
        if (extension.message_type == kMissionAssignMessageType) {
            HandleMissionAssignment(message, extension);
            return;
        }
        if (extension.message_type == kNetFireMessageType) {
            HandleNetFireCommand(message, extension);
            return;
        }
        if (extension.message_type == kFenceZoneMessageType) {
            HandleFenceZone(message, extension);
            return;
        }
        if (extension.message_type == kMissionCommandMessageType) {
            HandleMissionCommand(message, extension);
            return;
        }
        if (extension.message_type != kTrackTargetUpdateMessageType) {
            return;
        }
        // 65010 的协议有效数据到 offset 54（alt_reference），MAVLink2 允许截掉
        // 尾部零字节，因此不能直接沿用 60 字节的短帧门槛；把实际 payload 长度
        // 交给处理器按最小长度 54 校验（A-05）。
        const uint16_t payload_length =
            message.len > kV2ExtensionFixedHeaderLength
                ? static_cast<uint16_t>(message.len - kV2ExtensionFixedHeaderLength)
                : 0U;
        HandleTrackTargetUpdate(extension, payload_length);
    }

    // 65014 任务分配 payload 的链路层镜像（小端，协议有效 55 字节）。
    struct MissionAssignPayload {
        uint8_t protocol_version = 0;
        uint32_t assignment_id = 0;
        uint32_t target_id = 0;
        uint8_t interceptor_type = 0;
        uint8_t flags = 0;
        uint32_t valid_for_ms = 0;
        uint8_t target_system = 0;
        uint8_t target_component = 0;
        uint8_t frame_length_marker = 0;
        // 65014 v3（2026-10-29）：保留区 15~18 携带围栏区域标识，用于起飞前
        // 一致性预检（与服务端下发区域/机载缓存不一致时回 zone_mismatch）。
        uint16_t zone_id = 0;
        uint16_t zone_version = 0;
    };

    // 处理 MISSION_ASSIGN(65014)：来源已在 HandleV2Extension 过滤；这里只做帧长与
    // 地址双字段校验，地址匹配后原样发布，版本/flags/类型/target_id 语义留给状态机。
    // 地址不匹配必须静默丢弃并禁止回 65015，避免广播分配触发任何飞机动作。
    void HandleMissionAssignment(const mavlink_message_t& message,
                                 const mavlink_v2_extension_t& extension) {
        const uint64_t receive_count = ++mission_assign_receive_count_;
        if (ShouldLogThrottled(receive_count)) {
            SPDLOG_INFO("收到任务分配: target={}/{} len={} count={}",
                        extension.target_system, extension.target_component,
                        message.len, receive_count);
        }
        // 短帧/截断帧不解析，避免把零填充的地址字段误判为本机地址。
        if (message.len < kStatusShortFrameLength) {
            const uint64_t invalid_count = ++mission_assign_invalid_count_;
            if (ShouldLogThrottled(invalid_count)) {
                SPDLOG_WARN("任务分配帧长不足: len={} required={} count={}",
                            message.len, kStatusShortFrameLength, invalid_count);
            }
            return;
        }

        const MissionAssignPayload payload = DecodeMissionAssignment(extension.payload);
        // 帧头与 payload 的双字段地址必须同时匹配本机。
        if (extension.target_system != config_.aircraft_system_id ||
            extension.target_component != config_.aircraft_component_id ||
            payload.target_system != config_.aircraft_system_id ||
            payload.target_component != config_.aircraft_component_id) {
            const uint64_t mismatch_count = ++mission_assign_address_mismatch_count_;
            if (ShouldLogThrottled(mismatch_count)) {
                SPDLOG_WARN(
                    "任务分配地址不匹配: header={}/{} payload={}/{} local={}/{} count={}",
                    extension.target_system, extension.target_component,
                    payload.target_system, payload.target_component,
                    config_.aircraft_system_id, config_.aircraft_component_id,
                    mismatch_count);
            }
            return;
        }
        // 短帧长度标记必须为 1；缺失说明对端未按 55 字节协议编码。
        if (payload.frame_length_marker != kFrameLengthMarker) {
            const uint64_t invalid_count = ++mission_assign_invalid_count_;
            if (ShouldLogThrottled(invalid_count)) {
                SPDLOG_WARN("任务分配帧长度标记非法: marker={} count={}",
                            payload.frame_length_marker, invalid_count);
            }
            return;
        }

        const uint64_t receive_time_ms = MonotonicMs();
        common::MissionAssignment assignment;
        assignment.header.sequence = receive_count;
        // 65014 不携带地面站生成时刻，source_time_ms 保持 0。
        assignment.header.receive_time_ms = receive_time_ms;
        assignment.header.valid_for_ms = payload.valid_for_ms;
        assignment.header.source_id = config_.ground_system_id;
        assignment.header.health = 1;
        // 任务分配不携带空间数据，frame_id 不指向任何坐标系。
        assignment.header.frame_id = 0;
        assignment.protocol_version = payload.protocol_version;
        assignment.assignment_id = payload.assignment_id;
        assignment.target_id = payload.target_id;
        assignment.interceptor_type = payload.interceptor_type;
        assignment.flags = payload.flags;
        assignment.valid_for_ms = payload.valid_for_ms;
        assignment.received_ms = receive_time_ms;
        // 65014 v3：围栏区域标识（v1/v2 发送方为 0，启用围栏时被状态机判不一致）。
        assignment.zone_id = payload.zone_id;
        assignment.zone_version = payload.zone_version;
        (void)mission_assignment_output_.Publish(
            std::make_shared<const common::MissionAssignment>(assignment));
        const uint64_t publish_count = ++mission_assign_publish_count_;
        if (ShouldLogThrottled(publish_count)) {
            SPDLOG_INFO(
                "任务分配已发布: version={} assignment_id={} target_id={} type={} flags={} valid_for_ms={} count={}",
                assignment.protocol_version, assignment.assignment_id,
                assignment.target_id, assignment.interceptor_type,
                assignment.flags, assignment.valid_for_ms, publish_count);
        }
    }

    // 逐字段小端解码 65014 payload；调用方已保证帧长不少于 55 字节。
    MissionAssignPayload DecodeMissionAssignment(const uint8_t* data) const {
        MissionAssignPayload payload;
        payload.protocol_version = ReadLe<uint8_t>(data, 0);
        payload.assignment_id = ReadLe<uint32_t>(data, 1);
        payload.target_id = ReadLe<uint32_t>(data, 5);
        payload.interceptor_type = ReadLe<uint8_t>(data, 9);
        payload.flags = ReadLe<uint8_t>(data, 10);
        payload.valid_for_ms = ReadLe<uint32_t>(data, 11);
        payload.target_system = ReadLe<uint8_t>(data, 50);
        payload.target_component = ReadLe<uint8_t>(data, 51);
        payload.frame_length_marker = ReadLe<uint8_t>(data, 54);
        // 65014 v3 围栏区域标识（保留区 15~18；v1/v2 发送方为零 → 与任意区域
        // 都不一致，启用围栏时由状态机回 zone_mismatch，属预期行为）。
        payload.zone_id = ReadLe<uint16_t>(data, 15);
        payload.zone_version = ReadLe<uint16_t>(data, 17);
        return payload;
    }

    // 65016 捕网直控发射 payload 的链路层镜像（小端，协议有效 55 字节）。
    struct NetFirePayload {
        uint8_t protocol_version = 0;
        uint32_t command_id = 0;
        uint8_t flags = 0;
        uint8_t target_system = 0;
        uint8_t target_component = 0;
        uint8_t frame_length_marker = 0;
    };

    // 处理 NET_FIRE(65016)：来源已在 HandleV2Extension 过滤；地址双字段必须同时
    // 匹配本机，不匹配静默丢弃——广播发射指令不得触发任何一架的动作。版本、
    // 确认位、落地态等语义校验全部留给 GcsNetFire，链路层只做长度与地址校验。
    void HandleNetFireCommand(const mavlink_message_t& message,
                              const mavlink_v2_extension_t& extension) {
        const uint64_t receive_count = ++net_fire_receive_count_;
        if (ShouldLogThrottled(receive_count)) {
            SPDLOG_INFO("收到捕网直控发射指令: target={}/{} flags={} count={}",
                        extension.target_system, extension.target_component,
                        ReadLe<uint8_t>(extension.payload, 5), receive_count);
        }
        if (message.len < kStatusShortFrameLength) {
            const uint64_t invalid_count = ++net_fire_invalid_count_;
            if (ShouldLogThrottled(invalid_count)) {
                SPDLOG_WARN("捕网发射指令帧长不足: len={} required={} count={}",
                            message.len, kStatusShortFrameLength, invalid_count);
            }
            return;
        }
        const NetFirePayload payload = DecodeNetFireCommand(extension.payload);
        if (extension.target_system != config_.aircraft_system_id ||
            extension.target_component != config_.aircraft_component_id ||
            payload.target_system != config_.aircraft_system_id ||
            payload.target_component != config_.aircraft_component_id) {
            const uint64_t mismatch_count = ++net_fire_address_mismatch_count_;
            if (ShouldLogThrottled(mismatch_count)) {
                SPDLOG_WARN(
                    "捕网发射指令地址不匹配: header={}/{} payload={}/{} local={}/{} count={}",
                    extension.target_system, extension.target_component,
                    payload.target_system, payload.target_component,
                    config_.aircraft_system_id, config_.aircraft_component_id,
                    mismatch_count);
            }
            return;
        }
        if (payload.frame_length_marker != kFrameLengthMarker) {
            const uint64_t invalid_count = ++net_fire_invalid_count_;
            if (ShouldLogThrottled(invalid_count)) {
                SPDLOG_WARN("捕网发射指令帧长度标记非法: marker={} count={}",
                            payload.frame_length_marker, invalid_count);
            }
            return;
        }

        const uint64_t receive_time_ms = MonotonicMs();
        common::NetFireCommand command;
        command.header.sequence = receive_count;
        // 65016 不携带地面站生成时刻，source_time_ms 保持 0。
        command.header.receive_time_ms = receive_time_ms;
        command.header.valid_for_ms = kNetFireCommandValidForMs;
        command.header.source_id = config_.ground_system_id;
        command.header.health = 1;
        // 发射指令不携带空间数据，frame_id 不指向任何坐标系。
        command.header.frame_id = 0;
        command.protocol_version = payload.protocol_version;
        command.command_id = payload.command_id;
        command.flags = payload.flags;
        command.received_ms = receive_time_ms;
        (void)net_fire_command_output_.Publish(
            std::make_shared<const common::NetFireCommand>(command));
        const uint64_t publish_count = ++net_fire_publish_count_;
        if (ShouldLogThrottled(publish_count)) {
            SPDLOG_INFO(
                "捕网发射指令已发布: version={} command_id={} flags={} count={}",
                command.protocol_version, command.command_id, command.flags,
                publish_count);
        }
        if (!net_fire_consumer_present_) {
            // 开关关闭时 65016 会正常解码并发布，但无人消费、也不会回 65017；
            // 这条 WARN 让“地面站发了却没反应”能直接从机载日志判断原因。
            const uint64_t unconsumed_count = ++net_fire_unconsumed_count_;
            if (ShouldLogThrottled(unconsumed_count)) {
                SPDLOG_WARN(
                    "捕网发射指令无人消费: command_id={} count={}"
                    "（actuator.gcs_fire_enabled=false 时预期如此：执行器不动作，"
                    "也不回 65017）",
                    command.command_id, unconsumed_count);
            }
        }
    }

    // 逐字段小端解码 65016 payload；调用方已保证帧长不少于 60 字节。
    NetFirePayload DecodeNetFireCommand(const uint8_t* data) const {
        NetFirePayload payload;
        payload.protocol_version = ReadLe<uint8_t>(data, 0);
        payload.command_id = ReadLe<uint32_t>(data, 1);
        payload.flags = ReadLe<uint8_t>(data, 5);
        payload.target_system = ReadLe<uint8_t>(data, 50);
        payload.target_component = ReadLe<uint8_t>(data, 51);
        payload.frame_length_marker = ReadLe<uint8_t>(data, 54);
        return payload;
    }


    // 65018 围栏区域 payload 的链路层镜像（小端，协议有效 55 字节）。
    // 顶点用相对参考点的 int16 厘米偏移编码（每顶点 4B），首顶点之外全部相对
    // 同一个参考点（参考点=首顶点，精度 1cm、范围 ±327m，足够现场场地）。
    struct FenceZonePayload {
        uint8_t protocol_version = 0;
        uint16_t zone_id = 0;
        uint16_t version = 0;
        uint8_t type = 0;
        uint8_t action = 0;
        uint8_t altitude_reference = 0;
        int16_t alt_min_dm = 0;   ///< 0.1m 分辨率
        int16_t alt_max_dm = 0;   ///< 0=未限制
        uint8_t vertex_count = 0;
        int32_t ref_lat_1e7 = 0;
        int32_t ref_lon_1e7 = 0;
        uint8_t frame_length_marker = 0;
    };

    // 处理 ZONE_DEFINE(65018)：来源已在 HandleV2Extension 过滤；地址双字段必须
    // 同时匹配本机（区域是共享快照，但广播帧不得让任意飞机改变自己的执行边界）。
    // 区域语义（顶点数/高度带/动作）留给状态机校验，链路层只做长度与地址校验。
    void HandleFenceZone(const mavlink_message_t& message,
                         const mavlink_v2_extension_t& extension) {
        const uint64_t receive_count = ++fence_receive_count_;
        if (ShouldLogThrottled(receive_count)) {
            SPDLOG_INFO("收到围栏区域: target={}/{} count={}",
                        extension.target_system, extension.target_component,
                        receive_count);
        }
        if (message.len < kStatusShortFrameLength) {
            const uint64_t invalid_count = ++fence_invalid_count_;
            if (ShouldLogThrottled(invalid_count)) {
                SPDLOG_WARN("围栏区域帧长不足: len={} required={} count={}",
                            message.len, kStatusShortFrameLength, invalid_count);
            }
            return;
        }
        const FenceZonePayload payload = DecodeFenceZone(extension.payload);
        if (extension.target_system != config_.aircraft_system_id ||
            extension.target_component != config_.aircraft_component_id) {
            const uint64_t mismatch_count = ++fence_address_mismatch_count_;
            if (ShouldLogThrottled(mismatch_count)) {
                SPDLOG_WARN(
                    "围栏区域地址不匹配: header={}/{} local={}/{} count={}",
                    extension.target_system, extension.target_component,
                    config_.aircraft_system_id, config_.aircraft_component_id,
                    mismatch_count);
            }
            return;
        }
        if (payload.frame_length_marker != kFrameLengthMarker) {
            const uint64_t invalid_count = ++fence_invalid_count_;
            if (ShouldLogThrottled(invalid_count)) {
                SPDLOG_WARN("围栏区域帧长度标记非法: marker={} count={}",
                            payload.frame_length_marker, invalid_count);
            }
            return;
        }
        // 字段级校验：动作码只允许 0/1/2（保持/返航/降落）。越界值必须显式拒收，
        // 不能静默映射成返航——地面站/干扰帧给出非法动作时不应改变执行边界。
        // 高度带必须满足 alt_min <= alt_max；alt_max=0 表示"未限制"，此时不比较。
        const bool altitude_band_invalid =
            payload.alt_max_dm != 0 && payload.alt_min_dm > payload.alt_max_dm;
        if (payload.vertex_count < 3U || payload.vertex_count > kFenceMaxVertices ||
            payload.type != 0U || payload.action > 2U || altitude_band_invalid) {
            const uint64_t invalid_count = ++fence_invalid_count_;
            if (ShouldLogThrottled(invalid_count)) {
                SPDLOG_WARN(
                    "围栏区域字段非法: type={} 顶点={}（支持 3~{}）action={} "
                    "高度带={}~{}dm count={}",
                    payload.type, payload.vertex_count, kFenceMaxVertices,
                    payload.action, payload.alt_min_dm, payload.alt_max_dm,
                    invalid_count);
            }
            return;
        }

        const uint64_t receive_time_ms = MonotonicMs();
        common::FenceZone zone;
        zone.header.sequence = receive_count;
        // 65018 不携带地面站生成时刻，source_time_ms 保持 0。
        zone.header.receive_time_ms = receive_time_ms;
        zone.header.valid_for_ms = 0;  // 区域是持久快照，不按时效失效
        zone.header.source_id = config_.ground_system_id;
        zone.header.health = 1;
        zone.header.frame_id = 0;
        zone.protocol_version = payload.protocol_version;
        zone.zone_id = payload.zone_id;
        zone.version = payload.version;
        zone.type = common::FenceZoneType::kPolygon;
        // 动作码已在上面校验为 0/1/2，这里用显式分支保留语义（不再有 3~255 的兜底）。
        switch (payload.action) {
            case 0U:
                zone.action = common::FenceAction::kHold;
                break;
            case 1U:
                zone.action = common::FenceAction::kReturn;
                break;
            case 2U:
                zone.action = common::FenceAction::kLand;
                break;
            default:
                // 不可达：非法动作在字段校验处已整帧拒收。
                return;
        }
        zone.altitude_reference = payload.altitude_reference;
        zone.alt_min_m = static_cast<double>(payload.alt_min_dm) * 0.1;
        zone.alt_max_m = static_cast<double>(payload.alt_max_dm) * 0.1;
        const double ref_lat = static_cast<double>(payload.ref_lat_1e7) * 1e-7;
        const double ref_lon = static_cast<double>(payload.ref_lon_1e7) * 1e-7;
        zone.vertices.reserve(payload.vertex_count);
        for (uint8_t index = 0; index < payload.vertex_count; ++index) {
            const int16_t north_cm =
                ReadLe<int16_t>(extension.payload, 21U + index * 4U);
            const int16_t east_cm =
                ReadLe<int16_t>(extension.payload, 23U + index * 4U);
            // 顶点 = 参考点 + 相对偏移（NED 近似：1cm 偏移对场地的经纬度换算误差
            // 远小于 GPS 精度，且参考点经纬度由地面站给出，不做反向投影）。
            common::FenceVertex vertex;
            vertex.latitude_deg = ref_lat + static_cast<double>(north_cm) * 1e-2 / 111320.0;
            vertex.longitude_deg =
                ref_lon +
                static_cast<double>(east_cm) * 1e-2 /
                    (111320.0 * std::max(0.1, std::cos(ref_lat * kDegToRad)));
            zone.vertices.push_back(vertex);
        }
        zone.ref_lat_1e7 = payload.ref_lat_1e7;
        zone.ref_lon_1e7 = payload.ref_lon_1e7;
        zone.received_ms = receive_time_ms;
        (void)fence_zone_output_.Publish(
            std::make_shared<const common::FenceZone>(zone));
        const uint64_t publish_count = ++fence_publish_count_;
        if (ShouldLogThrottled(publish_count)) {
            SPDLOG_INFO(
                "围栏区域已发布: version={} zone={}/{} 顶点={} 动作={} count={}",
                zone.protocol_version, zone.zone_id, zone.version,
                zone.vertices.size(), static_cast<int>(zone.action),
                publish_count);
        }
    }

    // 处理 65020 任务指令（地面站主动返航）：帧长/地址双字段/marker 校验后发布。
    // 协议版本、指令类型、状态合法性由状态机校验（链路层不产生动作）。
    void HandleMissionCommand(const mavlink_message_t& message,
                              const mavlink_v2_extension_t& extension) {
        const uint64_t receive_count = ++mission_command_receive_count_;
        if (ShouldLogThrottled(receive_count)) {
            SPDLOG_INFO("收到任务指令: target={}/{} count={}",
                        extension.target_system, extension.target_component,
                        receive_count);
        }
        if (message.len < kStatusShortFrameLength) {
            const uint64_t invalid_count = ++mission_command_invalid_count_;
            if (ShouldLogThrottled(invalid_count)) {
                SPDLOG_WARN("任务指令帧长不足: len={} required={} count={}",
                            message.len, kStatusShortFrameLength, invalid_count);
            }
            return;
        }
        if (extension.target_system != config_.aircraft_system_id ||
            extension.target_component != config_.aircraft_component_id) {
            const uint64_t mismatch_count = ++mission_command_address_mismatch_count_;
            if (ShouldLogThrottled(mismatch_count)) {
                SPDLOG_WARN("任务指令地址不匹配: header={}/{} local={}/{} count={}",
                            extension.target_system, extension.target_component,
                            config_.aircraft_system_id,
                            config_.aircraft_component_id, mismatch_count);
            }
            return;
        }
        const uint8_t marker = ReadLe<uint8_t>(extension.payload, 54);
        if (marker != kFrameLengthMarker) {
            const uint64_t invalid_count = ++mission_command_invalid_count_;
            if (ShouldLogThrottled(invalid_count)) {
                SPDLOG_WARN("任务指令帧长度标记非法: marker={} count={}",
                            marker, invalid_count);
            }
            return;
        }

        const uint64_t receive_time_ms = MonotonicMs();
        common::MissionCommand command;
        command.header.sequence = receive_count;
        command.header.receive_time_ms = receive_time_ms;
        command.header.valid_for_ms = kMissionCommandValidForMs;
        command.header.source_id = config_.ground_system_id;
        command.header.health = 1;
        command.header.frame_id = 0;
        command.protocol_version = ReadLe<uint8_t>(extension.payload, 0);
        command.command_id = ReadLe<uint32_t>(extension.payload, 1);
        const uint8_t type = ReadLe<uint8_t>(extension.payload, 5);
        // 未知类型保留原值（状态机按"不支持"拒绝并回执），不做静默丢弃。
        command.type = static_cast<common::MissionCommandType>(type);
        command.received_ms = receive_time_ms;
        (void)mission_command_output_.Publish(
            std::make_shared<const common::MissionCommand>(command));
        const uint64_t publish_count = ++mission_command_publish_count_;
        if (ShouldLogThrottled(publish_count)) {
            SPDLOG_INFO("任务指令已发布: version={} command_id={} type={} count={}",
                        command.protocol_version, command.command_id,
                        static_cast<int>(command.type), publish_count);
        }
    }

    // 逐字段小端解码 65018 payload；调用方已保证帧长不少于 60 字节。
    FenceZonePayload DecodeFenceZone(const uint8_t* data) const {
        FenceZonePayload payload;
        payload.protocol_version = ReadLe<uint8_t>(data, 0);
        payload.zone_id = ReadLe<uint16_t>(data, 1);
        payload.version = ReadLe<uint16_t>(data, 3);
        payload.type = ReadLe<uint8_t>(data, 5);
        payload.action = ReadLe<uint8_t>(data, 6);
        payload.altitude_reference = ReadLe<uint8_t>(data, 7);
        payload.alt_min_dm = ReadLe<int16_t>(data, 8);
        payload.alt_max_dm = ReadLe<int16_t>(data, 10);
        payload.vertex_count = ReadLe<uint8_t>(data, 12);
        payload.ref_lat_1e7 = ReadLe<int32_t>(data, 13);
        payload.ref_lon_1e7 = ReadLe<int32_t>(data, 17);
        payload.frame_length_marker = ReadLe<uint8_t>(data, 54);
        return payload;
    }


    // 处理 TRACK_TARGET_UPDATE：依次做帧长、地址匹配、协议版本、坐标、身份、时间同步、
    // 序号、字段范围与时效校验，全部通过后发布 GroundStationTarget 并回 ACCEPTED ACK；
    // 任一失败按语义回对应拒绝 ACK。
    // @param payload_length V2_EXTENSION payload 的协议有效长度（message.len - 5）。
    void HandleTrackTargetUpdate(const mavlink_v2_extension_t& extension,
                                 uint16_t payload_length) {
        const uint64_t receive_count = ++target_update_receive_count_;
        if (ShouldLogThrottled(receive_count)) {
            SPDLOG_INFO("收到目标UPDATE: target={}/{} len={} count={}",
                        extension.target_system, extension.target_component,
                        payload_length, receive_count);
        }
        // 帧长不足时字段可能已被 MAVLink2 截尾成 0，直接按非法字段拒收；
        // 此时连 boot_id/seq 都不可信，ACK 用零值告知地面站帧被截断。
        if (payload_length < kTrackTargetUpdateMinimumPayloadLength) {
            const uint64_t invalid_count = ++target_update_invalid_count_;
            if (ShouldLogThrottled(invalid_count)) {
                SPDLOG_WARN("目标UPDATE帧长不足: len={} required={} count={}",
                            payload_length, kTrackTargetUpdateMinimumPayloadLength,
                            invalid_count);
            }
            SendTrackTargetAck(0, 0, 0, 0, TrackTargetAckResult::kRejectedInvalidField,
                               TrackTargetAckReason::kInternalError);
            return;
        }
        if (extension.target_system != config_.aircraft_system_id ||
            extension.target_component != config_.aircraft_component_id) {
            const uint64_t mismatch_count = ++target_update_address_mismatch_count_;
            if (ShouldLogThrottled(mismatch_count)) {
                SPDLOG_WARN("目标UPDATE地址不匹配: target={}/{} local={}/{} count={}",
                            extension.target_system, extension.target_component,
                            config_.aircraft_system_id, config_.aircraft_component_id,
                            mismatch_count);
            }
            return;
        }

        const TrackTargetUpdatePayload update = DecodeTrackTargetUpdate(extension.payload);
        if (update.target_system != config_.aircraft_system_id ||
            update.target_component != config_.aircraft_component_id) {
            const uint64_t mismatch_count = ++target_update_address_mismatch_count_;
            if (ShouldLogThrottled(mismatch_count)) {
                SPDLOG_WARN("目标UPDATE payload地址不匹配: target={}/{} local={}/{} count={}",
                            update.target_system, update.target_component,
                            config_.aircraft_system_id, config_.aircraft_component_id,
                            mismatch_count);
            }
            return;
        }

        const uint64_t receive_time_ms = MonotonicMs();
        uint64_t measured_age_ms = 0;
        // reject 统一封装：把 boot/seq/target_id 与当前测得年龄回填到 ACK。
        const auto reject = [&](TrackTargetAckResult result,
                                TrackTargetAckReason reason) {
            SendTrackTargetAck(update.ground_station_boot_id, update.update_seq,
                               update.target_id, measured_age_ms, result, reason);
        };

        // 协议版本必须匹配；不匹配即拒收（不进入后续严重校验）。
        if (update.protocol_version != kTrackTargetProtocolVersion) {
            reject(TrackTargetAckResult::kRejectedUnsupportedVersion,
                   TrackTargetAckReason::kProtocolVersionUnsupported);
            return;
        }
        // 仅接受 WGS84 坐标框架。
        if (update.coordinate_frame != kTrackTargetCoordinateFrameWgs84) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kLatitudeOrLongitudeInvalid);
            return;
        }
        // 地面站启动 ID 必须非 0；地面站每次重启会更换，需重新同步。
        if (update.ground_station_boot_id == 0) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kBootIdInvalidOrChanged);
            return;
        }
        // 目标时效依赖时间同步；未同步时拒绝定位类目标。
        if (time_sync_state_ != GroundStationTimeSyncState::kSynchronized) {
            reject(TrackTargetAckResult::kRejectedTimeSyncUnavailable,
                   TrackTargetAckReason::kTimeSyncUnavailable);
            return;
        }
        // 地面站重启（boot_id 变化）：重置序号与时间同步，并要求重新建立同步。
        if (last_ground_station_boot_id_ != 0 &&
            update.ground_station_boot_id != last_ground_station_boot_id_) {
            last_ground_station_boot_id_ = update.ground_station_boot_id;
            last_target_update_seq_ = 0;
            ResetTimeSync(false);
            reject(TrackTargetAckResult::kRejectedTimeSyncUnavailable,
                   TrackTargetAckReason::kBootIdInvalidOrChanged);
            return;
        }
        // 更新序号必须严格递增，否则视为过期/重复。
        if (update.update_seq <= last_target_update_seq_) {
            reject(TrackTargetAckResult::kRejectedStaleOrDuplicate,
                   TrackTargetAckReason::kUpdateSequenceStale);
            return;
        }
        // 目标 ID 不能为 0；经纬度范围校验（1e7 定点，单位度）。
        if (update.target_id == 0) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kTargetIdInvalid);
            return;
        }
        if (update.latitude_1e7 < -900000000 || update.latitude_1e7 > 900000000 ||
            update.longitude_1e7 < -1800000000 ||
            update.longitude_1e7 > 1800000000) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kLatitudeOrLongitudeInvalid);
            return;
        }
        // 有效期必须在配置的上下限内。
        if (update.valid_for_ms <
                static_cast<uint32_t>(config_.target_minimum_valid_for.count()) ||
            update.valid_for_ms >
                static_cast<uint32_t>(config_.target_maximum_valid_for.count())) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kValidForInvalid);
            return;
        }
        // flags 只能在已知位掩码内；按位校验 alt/heading/accuracy 等附带字段。
        if ((update.flags & ~kTrackTargetKnownFlagsMask) != 0) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kFlagsInvalid);
            return;
        }
        if ((update.flags & 0x0001U) != 0 && update.alt_reference > 3) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kAltitudeReferenceInvalid);
            return;
        }
        if ((update.flags & 0x0010U) != 0 && update.heading_cd >= 36000U) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kHeadingInvalid);
            return;
        }
        if (((update.flags & 0x0020U) != 0 && update.horizontal_accuracy_cm == UINT16_MAX) ||
            ((update.flags & 0x0040U) != 0 && update.vertical_accuracy_cm == UINT16_MAX)) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kAccuracyInvalid);
            return;
        }

        // 依据时间同步 offset，把地面站的 source_time 换算成飞机时钟，判断是否超前。
        const int64_t offset_ms = time_sync_offset_ns_ / 1000000LL;
        const int64_t source_aircraft_ms =
            static_cast<int64_t>(update.source_time_ms) - offset_ms;
        const int64_t receive_ms = static_cast<int64_t>(receive_time_ms);
        if (source_aircraft_ms > receive_ms +
                                  config_.target_future_tolerance.count()) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kSourceTimeInFuture);
            return;
        }
        const int64_t age_ms = std::max<int64_t>(0, receive_ms - source_aircraft_ms);
        measured_age_ms = static_cast<uint64_t>(age_ms);
        // 传输年龄超上限或已过有效期 → 目标过期。
        if (measured_age_ms >
                static_cast<uint64_t>(config_.target_maximum_transport_delay.count()) ||
            measured_age_ms >= update.valid_for_ms) {
            reject(TrackTargetAckResult::kRejectedStaleOrDuplicate,
                   TrackTargetAckReason::kTargetExpired);
            return;
        }
        // 剩余有效期不足最小门限 → 拒收。
        const uint64_t remaining_valid_ms = update.valid_for_ms - measured_age_ms;
        if (remaining_valid_ms <
            static_cast<uint64_t>(config_.target_minimum_remaining_valid.count())) {
            reject(TrackTargetAckResult::kRejectedStaleOrDuplicate,
                   TrackTargetAckReason::kRemainingValidityTooShort);
            return;
        }

        // 校验全部通过，组装项目内部 GroundStationTarget 并发布到 TargetOutput。
        common::GroundStationTarget target;
        target.header.sequence = update.update_seq;
        target.header.source_time_ms = update.source_time_ms;
        target.header.receive_time_ms = receive_time_ms;
        target.header.valid_for_ms = remaining_valid_ms;
        target.header.source_id = config_.ground_system_id;
        target.header.health = 1;
        target.header.frame_id = 1;
        target.ground_station_boot_id = update.ground_station_boot_id;
        target.update_seq = update.update_seq;
        target.target_id = update.target_id;
        target.latitude_1e7 = update.latitude_1e7;
        target.longitude_1e7 = update.longitude_1e7;
        target.altitude_mm = update.altitude_mm;
        target.alt_reference = update.alt_reference;
        target.protocol_version = update.protocol_version;
        target.transport_age_ms = measured_age_ms;
        target.velocity_north_mps = update.velocity_north_cms / 100.0F;
        target.velocity_east_mps = update.velocity_east_cms / 100.0F;
        target.velocity_down_mps = update.velocity_down_cms / 100.0F;
        target.heading_deg = update.heading_cd / 100.0F;
        target.horizontal_accuracy_m = update.horizontal_accuracy_cm / 100.0F;
        target.vertical_accuracy_m = update.vertical_accuracy_cm / 100.0F;
        target.validity_flags = update.flags;
        (void)target_output_.Publish(
            std::make_shared<const common::GroundStationTarget>(target));
        const uint64_t publish_count = ++target_publish_count_;
        if (ShouldLogThrottled(publish_count)) {
            SPDLOG_INFO("目标UPDATE已发布: boot={} seq={} target_id={} age_ms={} remaining_ms={} count={}",
                        update.ground_station_boot_id, update.update_seq,
                        update.target_id, measured_age_ms, remaining_valid_ms,
                        publish_count);
        }
        last_ground_station_boot_id_ = update.ground_station_boot_id;
        last_target_update_seq_ = update.update_seq;
        SendTrackTargetAck(update.ground_station_boot_id, update.update_seq,
                           update.target_id, measured_age_ms,
                           TrackTargetAckResult::kAccepted,
                           TrackTargetAckReason::kOk);
    }

    // 逐字段小端解码 TRACK_TARGET_UPDATE payload（前 55 字节）。
    // 调用方必须先按 kTrackTargetUpdateMinimumPayloadLength 校验长度，因此这里
    // 不再返回 optional：字段解码本身不会失败，长度不足属帧级错误。
    TrackTargetUpdatePayload DecodeTrackTargetUpdate(const uint8_t* data) const {
        TrackTargetUpdatePayload payload;
        payload.source_time_ms = ReadLe<uint64_t>(data, 0);
        payload.ground_station_boot_id = ReadLe<uint32_t>(data, 8);
        payload.update_seq = ReadLe<uint32_t>(data, 12);
        payload.target_id = ReadLe<uint32_t>(data, 16);
        payload.valid_for_ms = ReadLe<uint32_t>(data, 20);
        payload.latitude_1e7 = ReadLe<int32_t>(data, 24);
        payload.longitude_1e7 = ReadLe<int32_t>(data, 28);
        payload.altitude_mm = ReadLe<int32_t>(data, 32);
        payload.velocity_north_cms = ReadLe<int16_t>(data, 36);
        payload.velocity_east_cms = ReadLe<int16_t>(data, 38);
        payload.velocity_down_cms = ReadLe<int16_t>(data, 40);
        payload.heading_cd = ReadLe<uint16_t>(data, 42);
        payload.horizontal_accuracy_cm = ReadLe<uint16_t>(data, 44);
        payload.vertical_accuracy_cm = ReadLe<uint16_t>(data, 46);
        payload.flags = ReadLe<uint16_t>(data, 48);
        payload.target_system = ReadLe<uint8_t>(data, 50);
        payload.target_component = ReadLe<uint8_t>(data, 51);
        payload.coordinate_frame = ReadLe<uint8_t>(data, 52);
        payload.protocol_version = ReadLe<uint8_t>(data, 53);
        payload.alt_reference = ReadLe<uint8_t>(data, 54);
        return payload;
    }

    // 组装并发送 TRACK_TARGET_ACK：回填 boot/seq/target_id/年龄/RTT 与结果/原因码。
    void SendTrackTargetAck(uint32_t boot_id, uint32_t update_seq,
                            uint32_t target_id, uint64_t measured_age_ms,
                            TrackTargetAckResult result,
                            TrackTargetAckReason reason) {
        std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
        WriteLe<uint32_t>(payload, 0, boot_id);
        WriteLe<uint32_t>(payload, 4, update_seq);
        WriteLe<uint32_t>(payload, 8, target_id);
        WriteLe<uint32_t>(payload, 12, static_cast<uint32_t>(std::min<uint64_t>(
                                          measured_age_ms, UINT32_MAX)));
        WriteLe<uint32_t>(payload, 16, static_cast<uint32_t>(
                                          time_sync_round_trip_time_ns_ / 1000000ULL));
        WriteLe<uint16_t>(payload, 20, static_cast<uint16_t>(reason));
        WriteLe<uint8_t>(payload, 22, static_cast<uint8_t>(result));
        WriteLe<uint8_t>(payload, 23, config_.aircraft_system_id);
        WriteLe<uint8_t>(payload, 24, config_.aircraft_component_id);
        WriteLe<uint8_t>(payload, 25, kTrackTargetProtocolVersion);
        const bool sent = EncodeAndWrite(
            [this, &payload](mavlink_status_t* status, mavlink_message_t* message) {
                return mavlink_msg_v2_extension_pack_status(
                    config_.aircraft_system_id, config_.aircraft_component_id, status,
                    message, 0, config_.ground_system_id, config_.ground_component_id,
                    kTrackTargetAckMessageType, payload.data());
            });
        const uint64_t count = ++target_ack_send_count_;
        if (ShouldLogThrottled(count)) {
            SPDLOG_INFO("目标ACK发送{}: result={} reason={} boot={} seq={} target_id={} age_ms={} rtt_ms={} count={}",
                        sent ? "成功" : "失败", TrackTargetAckResultName(result),
                        TrackTargetAckReasonName(reason), boot_id, update_seq,
                        target_id, measured_age_ms,
                        time_sync_round_trip_time_ns_ / 1000000ULL, count);
        }
    }

    static uint32_t CpuLoadX100(float cpu_load_pct) {
        if (!std::isfinite(cpu_load_pct)) {
            return 0;
        }
        const double scaled = std::round(static_cast<double>(cpu_load_pct) * 100.0);
        return static_cast<uint32_t>(std::clamp(scaled, 0.0, 10000.0));
    }

    static int32_t DistanceMillimeters(float distance_m) {
        if (!std::isfinite(distance_m)) {
            return std::numeric_limits<int32_t>::min();
        }
        const double scaled = std::round(static_cast<double>(distance_m) * 1000.0);
        return static_cast<int32_t>(std::clamp(
            scaled, static_cast<double>(std::numeric_limits<int32_t>::min() + 1),
            static_cast<double>(std::numeric_limits<int32_t>::max())));
    }

    bool SendStatusExtension(
        uint16_t message_type,
        const std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN>& payload) {
        if (config_.mavlink_version != 2) {
            // v1 无法承载 V2_EXTENSION：显式记录而不是静默丢帧，
            // 否则地面站看到的只是"遥测停止"而链路错误计数毫无线索。
            RecordError("地面站扩展帧需要 MAVLink 2，当前配置无法发送");
            return false;
        }
        return EncodeAndWrite([this, message_type, &payload](mavlink_status_t* status,
                                                               mavlink_message_t* message) {
            return mavlink_msg_v2_extension_pack_status(
                config_.aircraft_system_id, config_.aircraft_component_id, status,
                message, 0, config_.ground_system_id, config_.ground_component_id,
                message_type, payload.data());
        });
    }

    // 发送 65015 任务分配回执：以本机身份发往 255/190，不要求地面站再回 ACK。
    bool SendMissionAssignmentAck(const common::MissionAssignmentAck& ack) {
        std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
        WriteLe<uint8_t>(payload, 0, kMissionAssignmentProtocolVersion);
        WriteLe<uint32_t>(payload, 1, ack.assignment_id);
        WriteLe<uint32_t>(payload, 5, ack.target_id);
        WriteLe<uint8_t>(payload, 9, static_cast<uint8_t>(ack.result));
        WriteLe<uint8_t>(payload, 10, static_cast<uint8_t>(ack.reason));
        WriteLe<uint8_t>(payload, 11, static_cast<uint8_t>(ack.mission_state));
        WriteLe<uint8_t>(payload, 12, static_cast<uint8_t>(ack.availability));
        WriteLe<uint8_t>(payload, 26, config_.aircraft_system_id);
        WriteLe<uint8_t>(payload, 27, config_.aircraft_component_id);
        // 保留尾字节非零，确保 MAVLink V2_EXTENSION 短帧 payload_len 固定为60。
        WriteLe<uint8_t>(payload, 54, kFrameLengthMarker);
        const bool sent = SendStatusExtension(kMissionAssignAckMessageType, payload);
        const uint64_t count = ++mission_assign_ack_send_count_;
        if (ShouldLogThrottled(count)) {
            SPDLOG_INFO(
                "任务分配ACK发送{}: assignment_id={} target_id={} result={} reason={} state={} availability={} count={}",
                sent ? "成功" : "失败", ack.assignment_id, ack.target_id,
                static_cast<uint8_t>(ack.result), static_cast<uint8_t>(ack.reason),
                static_cast<uint8_t>(ack.mission_state),
                static_cast<uint8_t>(ack.availability), count);
        }
        return sent;
    }

    // 发送 65017 捕网发射回执：以本机身份发往 255/190，不要求地面站再回 ACK。
    bool SendNetFireAck(const common::NetFireAck& ack) {
        std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
        WriteLe<uint8_t>(payload, 0, kNetFireProtocolVersion);
        WriteLe<uint32_t>(payload, 1, ack.command_id);
        WriteLe<uint8_t>(payload, 5, static_cast<uint8_t>(ack.result));
        WriteLe<uint8_t>(payload, 6, static_cast<uint8_t>(ack.reason));
        WriteLe<uint8_t>(payload, 7, ack.actuator_state);
        WriteLe<uint8_t>(payload, 50, config_.aircraft_system_id);
        WriteLe<uint8_t>(payload, 51, config_.aircraft_component_id);
        // 保留尾字节非零，确保 MAVLink V2_EXTENSION 短帧 payload_len 固定为60。
        WriteLe<uint8_t>(payload, 54, kFrameLengthMarker);
        const bool sent = SendStatusExtension(kNetFireAckMessageType, payload);
        const uint64_t count = ++net_fire_ack_send_count_;
        if (ShouldLogThrottled(count)) {
            SPDLOG_INFO(
                "捕网发射ACK发送{}: command_id={} result={} reason={} actuator_state={} count={}",
                sent ? "成功" : "失败", ack.command_id,
                static_cast<uint8_t>(ack.result), static_cast<uint8_t>(ack.reason),
                ack.actuator_state, count);
        }
        return sent;
    }

    bool SendHealthStatus(const common::HealthStatus& health) {
        std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
        const bool cpu_load_valid = std::isfinite(health.cpu_load_pct);
        WriteLe<uint8_t>(payload, 0, kHealthStatusProtocolVersion);
        WriteLe<uint32_t>(payload, 1, static_cast<uint32_t>(health.header.sequence));
        WriteLe<uint32_t>(payload, 5, health.link_health_bits);
        WriteLe<uint32_t>(payload, 9, health.device_health_bits);
        WriteLe<uint32_t>(payload, 13, health.data_freshness_bits);
        WriteLe<uint32_t>(payload, 17, health.error_bits);
        WriteLe<uint32_t>(payload, 21, CpuLoadX100(health.cpu_load_pct));
        WriteLe<uint32_t>(payload, 25, health.timeout_event_count);
        WriteLe<uint8_t>(payload, 29, config_.aircraft_system_id);
        WriteLe<uint8_t>(payload, 30, config_.aircraft_component_id);
        WriteLe<uint8_t>(payload, 31, cpu_load_valid ? 0x01U : 0x00U);
        WriteLe<uint64_t>(payload, 32, health.header.receive_time_ms);
        // 保留尾字节非零，确保 MAVLink V2_EXTENSION 短帧 payload_len 固定为60。
        WriteLe<uint8_t>(payload, 54, 1);
        return SendStatusExtension(kHealthStatusMessageType, payload);
    }

    bool SendMissionStatus(const common::MissionStatus& mission) {
        std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
        WriteLe<uint8_t>(payload, 0, kMissionStatusProtocolVersion);
        WriteLe<uint32_t>(payload, 1, static_cast<uint32_t>(mission.header.sequence));
        WriteLe<uint8_t>(payload, 5, static_cast<uint8_t>(mission.state));
        WriteLe<uint8_t>(payload, 6, mission.control_source);
        WriteLe<uint8_t>(payload, 7, mission.task_phase);
        WriteLe<uint32_t>(payload, 8, mission.active_warning_bits);
        WriteLe<uint64_t>(payload, 12, mission.state_entered_ms);
        WriteLe<int32_t>(payload, 20, DistanceMillimeters(mission.front_distance_m));
        WriteLe<uint8_t>(payload, 24, mission.interception_authorized ? 1 : 0);
        WriteLe<uint8_t>(payload, 25, mission.power_status_bits);
        WriteLe<uint8_t>(payload, 26, config_.aircraft_system_id);
        WriteLe<uint8_t>(payload, 27, config_.aircraft_component_id);
        WriteLe<uint64_t>(payload, 32, mission.header.receive_time_ms);
        // 65013 v2 扩展字段复用原保留区，offset 0~39 与 v1 完全一致。
        WriteLe<uint32_t>(payload, 40, mission.assignment_id);
        WriteLe<uint32_t>(payload, 44, mission.target_id);
        WriteLe<uint8_t>(payload, 48, static_cast<uint8_t>(mission.availability));
        WriteLe<uint8_t>(payload, 49, mission.interceptor_type);
        // 65013 v3（2026-10-29）：围栏区域与状态。offset 28~31 / 50 均为原保留区，
        // 0~27 与 v1、40~49 与 v2 完全一致（v1/v2 对端读不到新字段即视为无区域）。
        WriteLe<uint16_t>(payload, 28, mission.zone_id);
        WriteLe<uint16_t>(payload, 30, mission.zone_version);
        WriteLe<uint8_t>(payload, 50, mission.fence_state);
        // 保留尾字节非零，确保 MAVLink V2_EXTENSION 短帧 payload_len 固定为60。
        WriteLe<uint8_t>(payload, 54, kFrameLengthMarker);
        return SendStatusExtension(kMissionStatusMessageType, payload);
    }

    // 发送 65022 围栏区域回读：与 65018 完全同构，把机载生效区域原样回传。
    // 地面站据此判断"本地定义是否等于机载实况"，并在重连后恢复图层显示。
    bool SendFenceZoneReport(const common::FenceZone& zone) {
        // 顶点数越界属上游数据缺陷（区域加载/回填错误），显式失败并节流告警，
        // 不能静默 return false —— 否则地面站永远收不到回读也查不到原因。
        if (zone.vertices.size() < 3U || zone.vertices.size() > kFenceMaxVertices) {
            const uint64_t invalid_count = ++fence_report_invalid_count_;
            if (ShouldLogThrottled(invalid_count)) {
                SPDLOG_WARN("围栏区域回读顶点数非法: 顶点={} 支持 3~{} zone={}/{} "
                            "count={}",
                            zone.vertices.size(), kFenceMaxVertices, zone.zone_id,
                            zone.version, invalid_count);
            }
            return false;
        }
        // 参考点优先用下发时保存的原值；默认区域（本机生成）无原值时取首顶点。
        int32_t ref_lat_1e7 = zone.ref_lat_1e7;
        int32_t ref_lon_1e7 = zone.ref_lon_1e7;
        if (ref_lat_1e7 == 0 && ref_lon_1e7 == 0) {
            ref_lat_1e7 =
                static_cast<int32_t>(std::llround(zone.vertices.front().latitude_deg * 1e7));
            ref_lon_1e7 =
                static_cast<int32_t>(std::llround(zone.vertices.front().longitude_deg * 1e7));
        }
        const double ref_lat = static_cast<double>(ref_lat_1e7) * 1e-7;
        const double ref_lon = static_cast<double>(ref_lon_1e7) * 1e-7;
        const double lon_scale =
            111320.0 * std::max(0.1, std::cos(ref_lat * kDegToRad));
        std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
        WriteLe<uint8_t>(payload, 0, zone.protocol_version);
        WriteLe<uint16_t>(payload, 1, zone.zone_id);
        WriteLe<uint16_t>(payload, 3, zone.version);
        WriteLe<uint8_t>(payload, 5, static_cast<uint8_t>(zone.type));
        WriteLe<uint8_t>(payload, 6, static_cast<uint8_t>(zone.action));
        WriteLe<uint8_t>(payload, 7, zone.altitude_reference);
        // 高度带与顶点偏移用 clamp 转换：超量程夹逼到 int16 边界，NaN 走
        // ClampRoundedDouble 的非有限分支（→0），避免实现定义回绕。
        constexpr int16_t kInt16Min = std::numeric_limits<int16_t>::min();
        constexpr int16_t kInt16Max = std::numeric_limits<int16_t>::max();
        WriteLe<int16_t>(payload, 8,
                         ClampRoundedDouble<int16_t>(zone.alt_min_m, 10.0, kInt16Min,
                                                     kInt16Max));
        WriteLe<int16_t>(payload, 10,
                         ClampRoundedDouble<int16_t>(zone.alt_max_m, 10.0, kInt16Min,
                                                     kInt16Max));
        WriteLe<uint8_t>(payload, 12, static_cast<uint8_t>(zone.vertices.size()));
        WriteLe<int32_t>(payload, 13, ref_lat_1e7);
        WriteLe<int32_t>(payload, 17, ref_lon_1e7);
        for (size_t index = 0; index < zone.vertices.size(); ++index) {
            const double north_m =
                (zone.vertices[index].latitude_deg - ref_lat) * 111320.0;
            const double east_m = (zone.vertices[index].longitude_deg - ref_lon) * lon_scale;
            WriteLe<int16_t>(payload, static_cast<size_t>(21U + index * 4U),
                             ClampRoundedDouble<int16_t>(north_m, 100.0, kInt16Min,
                                                         kInt16Max));
            WriteLe<int16_t>(payload, static_cast<size_t>(23U + index * 4U),
                             ClampRoundedDouble<int16_t>(east_m, 100.0, kInt16Min,
                                                         kInt16Max));
        }
        WriteLe<uint8_t>(payload, 54, kFrameLengthMarker);
        const bool sent = SendStatusExtension(kFenceZoneReportMessageType, payload);
        const uint64_t count = ++fence_report_send_count_;
        if (ShouldLogThrottled(count)) {
            SPDLOG_INFO("围栏区域回读发送{}: zone={}/{} 顶点={} 动作={} count={}",
                        sent ? "" : "失败", zone.zone_id, zone.version,
                        zone.vertices.size(), static_cast<int>(zone.action), count);
        }
        return sent;
    }

    // 发送 65019 围栏状态：以本机身份发往 255/190。状态是周期快照，地面站据此
    // 绘制区域与越界告警；区域本身由地面站下发，机载只回报判定结果。
    bool SendFenceStatus(const common::FenceStatus& status) {
        std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
        WriteLe<uint8_t>(payload, 0, kFenceZoneProtocolVersion);
        WriteLe<uint16_t>(payload, 1, status.zone_id);
        WriteLe<uint16_t>(payload, 3, status.version);
        WriteLe<uint8_t>(payload, 5, static_cast<uint8_t>(status.state));
        WriteLe<int32_t>(payload, 6, DistanceMillimeters(
                                        static_cast<float>(status.distance_to_boundary_m)));
        WriteLe<uint32_t>(payload, 10, status.violation_duration_ms);
        // 局部位置用 mm 编码（与前端距离字段同一换算，NaN→INT32_MIN 表示无效）。
        WriteLe<int32_t>(payload, 14,
                         DistanceMillimeters(static_cast<float>(status.local_north_m)));
        WriteLe<int32_t>(payload, 18,
                         DistanceMillimeters(static_cast<float>(status.local_east_m)));
        WriteLe<int32_t>(payload, 22,
                         DistanceMillimeters(static_cast<float>(status.local_down_m)));
        WriteLe<uint8_t>(payload, 26, status.action_pending ? 1 : 0);
        WriteLe<uint8_t>(payload, 50, config_.aircraft_system_id);
        WriteLe<uint8_t>(payload, 51, config_.aircraft_component_id);
        // 保留尾字节非零，确保 MAVLink V2_EXTENSION 短帧 payload_len 固定为60。
        WriteLe<uint8_t>(payload, 54, kFrameLengthMarker);
        const bool sent = SendStatusExtension(kFenceStatusMessageType, payload);
        const uint64_t count = ++fence_status_send_count_;
        // 1Hz 常态上报，只在“状态变化 / 首条 / 每 300 条”打日志：围栏状态是持续流，
        // 逐条 INFO 会在长时间飞行中把日志刷满。
        const bool state_changed = status.state != last_fence_state_ ||
                                   status.zone_id != last_fence_zone_id_ ||
                                   status.version != last_fence_version_;
        last_fence_state_ = status.state;
        last_fence_zone_id_ = status.zone_id;
        last_fence_version_ = status.version;
        if (state_changed || count == 1U || count % 300U == 0U) {
            SPDLOG_INFO(
                "围栏状态发送{}: zone={}/{} state={} 距边界={:.2f}m 越界={}ms count={}",
                sent ? "成功" : "失败", status.zone_id, status.version,
                static_cast<int>(status.state), status.distance_to_boundary_m,
                status.violation_duration_ms, count);
        }
        return sent;
    }

    // 发送 65021 任务指令回执：以本机身份发往 255/190，command_id 原样带回，
    // 地面站据此把"主动返航"按钮的结果落到具体指令上（2026-10-29）。
    bool SendMissionCommandAck(const common::MissionCommandAck& ack) {
        std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
        WriteLe<uint8_t>(payload, 0, kMissionCommandProtocolVersion);
        WriteLe<uint32_t>(payload, 1, ack.command_id);
        WriteLe<uint8_t>(payload, 5, static_cast<uint8_t>(ack.type));
        WriteLe<uint8_t>(payload, 6, static_cast<uint8_t>(ack.result));
        WriteLe<uint8_t>(payload, 7, static_cast<uint8_t>(ack.reason));
        WriteLe<uint8_t>(payload, 8, static_cast<uint8_t>(ack.mission_state));
        WriteLe<uint8_t>(payload, 50, config_.aircraft_system_id);
        WriteLe<uint8_t>(payload, 51, config_.aircraft_component_id);
        // 保留尾字节非零，确保 MAVLink V2_EXTENSION 短帧 payload_len 固定为60。
        WriteLe<uint8_t>(payload, 54, kFrameLengthMarker);
        const bool sent = SendStatusExtension(kMissionCommandAckMessageType, payload);
        const uint64_t count = ++mission_command_ack_send_count_;
        if (ShouldLogThrottled(count)) {
            SPDLOG_INFO(
                "任务指令回执发送{}: command_id={} type={} result={} reason={} state={} count={}",
                sent ? "成功" : "失败", ack.command_id,
                static_cast<int>(ack.type), static_cast<int>(ack.result),
                static_cast<int>(ack.reason), static_cast<int>(ack.mission_state),
                count);
        }
        return sent;
    }

    // 地面站心跳超时：超过配置时长则置离线并清空时间同步（同步依赖在线心跳）。
    void CheckHeartbeatTimeout(uint64_t now_ms) {
        if (!connected_.load(std::memory_order_acquire) || last_gcs_heartbeat_ms_ == 0 ||
            now_ms < last_gcs_heartbeat_ms_) {
            return;
        }
        if (now_ms - last_gcs_heartbeat_ms_ >
            static_cast<uint64_t>(config_.heartbeat_timeout.count())) {
            connected_.store(false, std::memory_order_release);
            ResetTimeSync(false);
            SPDLOG_WARN("地面站心跳超时: timeout_ms={}",
                        config_.heartbeat_timeout.count());
        }
    }

    // 同步后改用较慢的稳定期采样；未同步/降级则用较快获取期，加快收敛。
    std::chrono::milliseconds TimeSyncSendInterval() const {
        return time_sync_state_ == GroundStationTimeSyncState::kSynchronized
                   ? config_.time_sync_steady_interval
                   : config_.time_sync_acquire_interval;
    }

    // 清理超时未回的 TIMESYNC 请求：超过最大 RTT 即视为超时丢弃。
    void PrunePendingTimeSyncRequests(int64_t now_ns) {
        const int64_t max_rtt_ns =
            static_cast<int64_t>(config_.time_sync_max_rtt.count()) * 1000000LL;
        while (!pending_time_sync_requests_.empty()) {
            const int64_t sent_ns = pending_time_sync_requests_.front();
            if (now_ns >= sent_ns && now_ns - sent_ns > max_rtt_ns) {
                pending_time_sync_requests_.pop_front();
                ++time_sync_request_timeout_count_;
            } else {
                break;
            }
        }
    }

    // 主动发起定向 TIMESYNC 请求：tc1=0、ts1=当前飞机单调纳秒，入队用于后续匹配。
    void SendTimeSyncRequest() {
        const int64_t request_time_ns = MonotonicNs();
        PrunePendingTimeSyncRequests(request_time_ns);
        const bool sent = EncodeAndWrite(
            [this, request_time_ns](mavlink_status_t* status,
                                    mavlink_message_t* message) {
                return mavlink_msg_timesync_pack_status(
                    config_.aircraft_system_id, config_.aircraft_component_id,
                    status, message, 0, request_time_ns,
                    config_.ground_system_id, config_.ground_component_id);
            });
        if (!sent) {
            return;
        }

        pending_time_sync_requests_.push_back(request_time_ns);
        const std::size_t pending_capacity =
            std::max<std::size_t>(2, config_.time_sync_window_capacity * 2);
        if (pending_time_sync_requests_.size() > pending_capacity) {
            pending_time_sync_requests_.pop_front();
            ++time_sync_request_timeout_count_;
        }
        ++time_sync_request_count_;
        if (time_sync_state_ == GroundStationTimeSyncState::kUnsynchronized) {
            time_sync_state_ = GroundStationTimeSyncState::kAcquiring;
        }
        PublishTimeSyncStatus(request_time_ns);
    }

    // 响应地面站发起的 TIMESYNC：回填 tc1=接收时刻、ts1=对端请求时间。
    void SendTimeSyncResponse(int64_t requester_time_ns) {
        const int64_t response_time_ns = MonotonicNs();
        (void)EncodeAndWrite(
            [this, requester_time_ns, response_time_ns](mavlink_status_t* status,
                                                        mavlink_message_t* message) {
                return mavlink_msg_timesync_pack_status(
                    config_.aircraft_system_id, config_.aircraft_component_id,
                    status, message, response_time_ns, requester_time_ns,
                    config_.ground_system_id, config_.ground_component_id);
            });
    }

    // 处理本机请求的响应：按 ts1 匹配在途请求，计算 RTT 与 offset 并进入样本估计。
    void HandleTimeSyncResponse(const mavlink_timesync_t& response) {
        const int64_t receive_time_ns = MonotonicNs();
        PrunePendingTimeSyncRequests(receive_time_ns);
        const auto request = std::find(pending_time_sync_requests_.begin(),
                                       pending_time_sync_requests_.end(),
                                       response.ts1);
        if (request == pending_time_sync_requests_.end()) {
            ++time_sync_rejected_sample_count_;
            PublishTimeSyncStatus(receive_time_ns);
            return;
        }

        const int64_t request_time_ns = *request;
        pending_time_sync_requests_.erase(request);
        if (receive_time_ns <= request_time_ns) {
            ++time_sync_rejected_sample_count_;
            PublishTimeSyncStatus(receive_time_ns);
            return;
        }

        const uint64_t round_trip_time_ns =
            static_cast<uint64_t>(receive_time_ns - request_time_ns);
        const uint64_t max_rtt_ns =
            static_cast<uint64_t>(config_.time_sync_max_rtt.count()) * 1000000ULL;
        if (round_trip_time_ns > max_rtt_ns) {
            ++time_sync_rejected_sample_count_;
            PublishTimeSyncStatus(receive_time_ns);
            return;
        }

        const int64_t midpoint_ns =
            request_time_ns + (receive_time_ns - request_time_ns) / 2;
        const int64_t offset_ns = response.tc1 - midpoint_ns;
        ++time_sync_response_count_;
        ProcessTimeSyncSample({offset_ns, round_trip_time_ns, receive_time_ns});
    }

    // 单条时间同步样本：offset、RTT 与接收时刻（用于窗口老化）。
    struct TimeSyncSample {
        int64_t offset_ns = 0;
        uint64_t round_trip_time_ns = 0;
        int64_t receive_time_ns = 0;
    };

    // 进入样本估计：同步态下若 offset 突变超门限则降级并清空样本。
    void ProcessTimeSyncSample(const TimeSyncSample& sample) {
        const uint64_t max_jump_ns =
            static_cast<uint64_t>(config_.time_sync_max_offset_jump.count()) *
            1000000ULL;
        if (time_sync_state_ == GroundStationTimeSyncState::kSynchronized &&
            AbsoluteDifference(sample.offset_ns, time_sync_offset_ns_) > max_jump_ns) {
            const auto previous_state = time_sync_state_;
            ++time_sync_rejected_sample_count_;
            time_sync_samples_.clear();
            time_sync_state_ = GroundStationTimeSyncState::kDegraded;
            SPDLOG_WARN("地面站时间同步偏移突变: state={}->{} max_jump_ms={}",
                        TimeSyncStateName(previous_state),
                        TimeSyncStateName(time_sync_state_),
                        config_.time_sync_max_offset_jump.count());
        }

        time_sync_samples_.push_back(sample);
        if (time_sync_samples_.size() > config_.time_sync_window_capacity) {
            time_sync_samples_.erase(time_sync_samples_.begin());
        }
        last_time_sync_sample_ns_ = sample.receive_time_ns;
        RecomputeTimeSyncEstimate(sample.receive_time_ns);
    }

    // 重新估计：优先选 RTT 最低的样本，对 offset/RTT 取中位数，计算 jitter。
    // 达到最小样本数且 jitter 不超门限才进入 SYNCHRONIZED。
    void RecomputeTimeSyncEstimate(int64_t now_ns) {
        std::vector<TimeSyncSample> selected = time_sync_samples_;
        std::sort(selected.begin(), selected.end(),
                  [](const TimeSyncSample& left, const TimeSyncSample& right) {
                      return left.round_trip_time_ns < right.round_trip_time_ns;
                  });
        if (selected.size() > config_.time_sync_minimum_samples) {
            selected.resize(config_.time_sync_minimum_samples);
        }
        if (selected.empty()) {
            PublishTimeSyncStatus(now_ns);
            return;
        }

        std::vector<int64_t> offsets;
        std::vector<uint64_t> round_trip_times;
        offsets.reserve(selected.size());
        round_trip_times.reserve(selected.size());
        for (const auto& sample : selected) {
            offsets.push_back(sample.offset_ns);
            round_trip_times.push_back(sample.round_trip_time_ns);
        }
        std::sort(offsets.begin(), offsets.end());
        std::sort(round_trip_times.begin(), round_trip_times.end());
        const std::size_t middle = offsets.size() / 2;
        const int64_t estimated_offset_ns =
            offsets.size() % 2 == 0
                ? offsets[middle - 1] +
                      (offsets[middle] - offsets[middle - 1]) / 2
                : offsets[middle];
        const uint64_t estimated_rtt_ns =
            round_trip_times.size() % 2 == 0
                ? round_trip_times[middle - 1] +
                      (round_trip_times[middle] - round_trip_times[middle - 1]) / 2
                : round_trip_times[middle];
        uint64_t jitter_ns = 0;
        for (const int64_t offset : offsets) {
            jitter_ns = std::max(jitter_ns,
                                 AbsoluteDifference(offset, estimated_offset_ns));
        }

        time_sync_offset_ns_ = estimated_offset_ns;
        time_sync_round_trip_time_ns_ = estimated_rtt_ns;
        time_sync_jitter_ns_ = jitter_ns;
        const uint64_t max_jitter_ns =
            static_cast<uint64_t>(config_.time_sync_max_offset_jump.count()) *
            1000000ULL;
        bool state_changed = false;
        if (time_sync_samples_.size() >= config_.time_sync_minimum_samples &&
            jitter_ns <= max_jitter_ns) {
            const auto previous_state = time_sync_state_;
            time_sync_state_ = GroundStationTimeSyncState::kSynchronized;
            state_changed =
                previous_state != GroundStationTimeSyncState::kSynchronized;
            if (state_changed) {
                SPDLOG_INFO(
                    "地面站时间同步建立: offset_ms={:.3f} rtt_ms={:.3f} jitter_ms={:.3f} samples={}",
                    static_cast<double>(time_sync_offset_ns_) / 1000000.0,
                    static_cast<double>(time_sync_round_trip_time_ns_) / 1000000.0,
                    static_cast<double>(time_sync_jitter_ns_) / 1000000.0,
                    time_sync_samples_.size());
            }
        } else if (time_sync_state_ == GroundStationTimeSyncState::kSynchronized) {
            // 已同步后如果抖动恶化（例如链路拥塞导致 RTT 抖动放大），必须降级
            // 而不是继续用不可信的 offset（协议 §5：DEGRADED 覆盖 RTT/offset 跳变）。
            time_sync_state_ = GroundStationTimeSyncState::kDegraded;
            state_changed = true;
            const uint64_t degraded_count = ++time_sync_degraded_count_;
            if (ShouldLogThrottled(degraded_count)) {
                SPDLOG_WARN(
                    "地面站时间同步降级: jitter_ms={:.3f} 门限_ms={} rtt_ms={:.3f} "
                    "samples={} 累计 {}",
                    static_cast<double>(time_sync_jitter_ns_) / 1000000.0,
                    config_.time_sync_max_offset_jump.count(),
                    static_cast<double>(time_sync_round_trip_time_ns_) / 1000000.0,
                    time_sync_samples_.size(), degraded_count);
            }
        }
        // 状态跃迁立即发布；估计值的小幅变化按 1Hz 低频发布，
        // 避免每收到一条 TIMESYNC 响应都加锁刷新状态快照。
        if (state_changed) {
            PublishTimeSyncStatus(now_ns);
        } else {
            PublishTimeSyncStatusIfDue(now_ns);
        }
    }

    // 同步样本老化超时：超过 time_sync_timeout 即退回未同步。
    void CheckTimeSyncTimeout(uint64_t now_ms) {
        const int64_t now_ns = static_cast<int64_t>(now_ms) * 1000000LL;
        PrunePendingTimeSyncRequests(now_ns);
        if (last_time_sync_sample_ns_ == 0 || now_ns < last_time_sync_sample_ns_) {
            PublishTimeSyncStatusIfDue(now_ns);
            return;
        }
        const uint64_t sample_age_ms =
            static_cast<uint64_t>(now_ns - last_time_sync_sample_ns_) / 1000000ULL;
        if (sample_age_ms <=
            static_cast<uint64_t>(config_.time_sync_timeout.count())) {
            // 无状态变化：按 1Hz 低频发布，避免读超时 20ms 时每轮都加锁分配。
            PublishTimeSyncStatusIfDue(now_ns);
            return;
        }

        const auto previous_state = time_sync_state_;
        ResetTimeSync(false);
        if (previous_state == GroundStationTimeSyncState::kSynchronized ||
            previous_state == GroundStationTimeSyncState::kDegraded) {
            SPDLOG_WARN("地面站时间同步超时: previous={} timeout_ms={}",
                        TimeSyncStateName(previous_state),
                        config_.time_sync_timeout.count());
        }
    }

    // 重置时间同步：清状态、样本与在途请求；可选清空计数（重连/重建场景）。
    void ResetTimeSync(bool reset_counters) {
        time_sync_state_ = GroundStationTimeSyncState::kUnsynchronized;
        time_sync_samples_.clear();
        pending_time_sync_requests_.clear();
        time_sync_offset_ns_ = 0;
        time_sync_round_trip_time_ns_ = 0;
        time_sync_jitter_ns_ = 0;
        last_time_sync_sample_ns_ = 0;
        if (reset_counters) {
            time_sync_request_count_ = 0;
            time_sync_response_count_ = 0;
            time_sync_rejected_sample_count_ = 0;
            time_sync_request_timeout_count_ = 0;
        }
        PublishTimeSyncStatus(MonotonicNs());
    }

    // 低频发布：距上次发布未满 kTimeSyncStatusPublishInterval 时跳过，
    // 用于"无状态变化"的轮询路径（CheckTimeSyncTimeout）。状态跃迁仍走强制发布。
    void PublishTimeSyncStatusIfDue(int64_t now_ns) {
        if (last_time_sync_publish_ns_ != 0 && now_ns >= last_time_sync_publish_ns_ &&
            static_cast<uint64_t>(now_ns - last_time_sync_publish_ns_) <
                static_cast<uint64_t>(kTimeSyncStatusPublishInterval.count()) *
                    1000000ULL) {
            return;
        }
        PublishTimeSyncStatus(now_ns);
    }

    // 把当前时间同步快照发布到 status（带锁复制，供诊断线程查询）。
    void PublishTimeSyncStatus(int64_t now_ns) {
        last_time_sync_publish_ns_ = now_ns;
        GroundStationTimeSyncStatus status;
        status.state = time_sync_state_;
        status.offset_ns = time_sync_offset_ns_;
        status.round_trip_time_ns = time_sync_round_trip_time_ns_;
        status.jitter_ns = time_sync_jitter_ns_;
        if (last_time_sync_sample_ns_ != 0 && now_ns >= last_time_sync_sample_ns_) {
            status.sample_age_ms =
                static_cast<uint64_t>(now_ns - last_time_sync_sample_ns_) /
                1000000ULL;
        }
        status.valid_sample_count = time_sync_samples_.size();
        status.request_count = time_sync_request_count_;
        status.response_count = time_sync_response_count_;
        status.rejected_sample_count = time_sync_rejected_sample_count_;
        status.request_timeout_count = time_sync_request_timeout_count_;
        std::lock_guard<std::mutex> lock(time_sync_status_mutex_);
        time_sync_status_ = status;
    }

    // 统一发送：Encode 得到完整 MAVLink 帧字节，写入串口并累加发送计数。
    template <typename Packer>
    bool EncodeAndWrite(Packer&& packer) {
        const auto frame = mavlink_.Encode(std::forward<Packer>(packer));
        if (frame.empty() || !serial_.Write(frame.data(), frame.size())) {
            RecordError("地面站 MAVLink 帧发送失败");
            return false;
        }
        // 发送统计只在帧真正写进串口后累加：Encode 本身不再记账，
        // 否则"编码成功但写失败"会虚增 SentMessageCount（A-10）。
        mavlink_.RecordSentFrame(frame.size());
        send_count_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    // HEARTBEAT：回传 armed/mode/system_status；未连接时用 UNINIT。
    void SendHeartbeat() {
        const common::FlightStateSnapshot state =
            latest_state_.value_or(common::FlightStateSnapshot{});
        (void)EncodeAndWrite([this, &state](mavlink_status_t* status,
                                           mavlink_message_t* message) {
            return mavlink_msg_heartbeat_pack_status(
                config_.aircraft_system_id, config_.aircraft_component_id, status,
                message, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
                state.base_mode, state.custom_mode,
                state.connected ? state.system_status : MAV_STATE_UNINIT);
        });
    }

    bool SendAttitude(const common::FlightStateSnapshot& state) {
        if (!state.attitude_valid) {
            return false;
        }
        return EncodeAndWrite([this, &state](mavlink_status_t* status,
                                             mavlink_message_t* message) {
            return mavlink_msg_attitude_pack_status(
                config_.aircraft_system_id, config_.aircraft_component_id, status,
                message, static_cast<uint32_t>(MonotonicMs()), state.roll_rad,
                state.pitch_rad, state.yaw_rad, 0.f, 0.f, 0.f);
        });
    }

    bool SendLocalPosition(const common::FlightStateSnapshot& state) {
        if (!state.local_position_valid) {
            return false;
        }
        return EncodeAndWrite([this, &state](mavlink_status_t* status,
                                             mavlink_message_t* message) {
            return mavlink_msg_local_position_ned_pack_status(
                config_.aircraft_system_id, config_.aircraft_component_id, status,
                message, static_cast<uint32_t>(MonotonicMs()), state.local_x_m,
                state.local_y_m, state.local_z_m, state.vx_mps, state.vy_mps,
                state.vz_mps);
        });
    }

    // GLOBAL_POSITION_INT：relative_alt 仅在 Home 有效时由 MSL 高差计算，否则为 0。
    bool SendGlobalPosition(const common::FlightStateSnapshot& state) {
        if (!state.global_position_valid) {
            return false;
        }
        const int32_t relative_altitude =
            state.home_valid ? state.altitude_mm - state.home_altitude_mm : 0;
        const int16_t vx = ClampRounded<int16_t>(
            state.vx_mps, 100.f, std::numeric_limits<int16_t>::min(),
            std::numeric_limits<int16_t>::max());
        const int16_t vy = ClampRounded<int16_t>(
            state.vy_mps, 100.f, std::numeric_limits<int16_t>::min(),
            std::numeric_limits<int16_t>::max());
        const int16_t vz = ClampRounded<int16_t>(
            state.vz_mps, 100.f, std::numeric_limits<int16_t>::min(),
            std::numeric_limits<int16_t>::max());
        const uint16_t heading =
            state.attitude_valid ? HeadingCentidegrees(state.yaw_rad)
                                 : std::numeric_limits<uint16_t>::max();
        return EncodeAndWrite([this, &state, relative_altitude, vx, vy, vz, heading](
                                  mavlink_status_t* status,
                                  mavlink_message_t* message) {
            return mavlink_msg_global_position_int_pack_status(
                config_.aircraft_system_id, config_.aircraft_component_id, status,
                message, static_cast<uint32_t>(MonotonicMs()), state.latitude_1e7,
                state.longitude_1e7, state.altitude_mm, relative_altitude, vx, vy,
                vz, heading);
        });
    }

    // GPS_RAW_INT：仅在 GPS 状态有效时发送；无全局位置时经纬高填 0，靠 fix_type 表达。
    // 卫星数/eph/epv 透传真实值，未知时按 MAVLink 约定填 UINT8_MAX/UINT16_MAX。
    bool SendGps(const common::FlightStateSnapshot& state) {
        if (!state.gps_state_valid) {
            return false;
        }
        const int32_t latitude = state.global_position_valid ? state.latitude_1e7 : 0;
        const int32_t longitude = state.global_position_valid ? state.longitude_1e7 : 0;
        const int32_t altitude = state.global_position_valid ? state.altitude_mm : 0;
        const uint16_t eph_cm = state.gps_accuracy_valid
            ? static_cast<uint16_t>(state.gps_eph_m * 100.f)
            : std::numeric_limits<uint16_t>::max();
        const uint16_t epv_cm = state.gps_accuracy_valid
            ? static_cast<uint16_t>(state.gps_epv_m * 100.f)
            : std::numeric_limits<uint16_t>::max();
        const uint8_t satellites = state.gps_satellites_valid
            ? state.gps_satellites_visible
            : std::numeric_limits<uint8_t>::max();
        return EncodeAndWrite([this, &state, latitude, longitude, altitude, eph_cm,
                               epv_cm, satellites](
                                  mavlink_status_t* status,
                                  mavlink_message_t* message) {
            return mavlink_msg_gps_raw_int_pack_status(
                config_.aircraft_system_id, config_.aircraft_component_id, status,
                message, 0, state.gps_fix_type, latitude, longitude, altitude,
                eph_cm, epv_cm, UINT16_MAX, UINT16_MAX, satellites, 0,
                0, 0, 0, 0, 0);
        });
    }

    bool SendExtendedState(const common::FlightStateSnapshot& state) {
        const uint8_t landed_state =
            !state.landed_state_valid
                ? MAV_LANDED_STATE_UNDEFINED
                : (state.landed ? MAV_LANDED_STATE_ON_GROUND
                                : MAV_LANDED_STATE_IN_AIR);
        return EncodeAndWrite([this, landed_state](mavlink_status_t* status,
                                                   mavlink_message_t* message) {
            return mavlink_msg_extended_sys_state_pack_status(
                config_.aircraft_system_id, config_.aircraft_component_id, status,
                message, MAV_VTOL_STATE_UNDEFINED, landed_state);
        });
    }

    // SYS_STATUS：总压 mV、电流 cA、剩余 %；无效字段用 UINT16_MAX / -1 表示未知。
    bool SendSystemStatus(const common::FlightStateSnapshot& state) {
        const uint16_t voltage = state.battery_valid
                                     ? ClampRounded<uint16_t>(state.battery_voltage_v, 1000.f,
                                                               0, UINT16_MAX)
                                     : UINT16_MAX;
        const int16_t current = state.battery_current_valid
                                    ? ClampRounded<int16_t>(state.battery_current_a, 100.f,
                                                            INT16_MIN, INT16_MAX)
                                    : -1;
        const int8_t remaining = state.battery_remaining_valid
                                     ? ClampRounded<int8_t>(state.battery_remaining_pct, 1.f,
                                                            0, 100)
                                     : -1;
        return EncodeAndWrite([this, voltage, current, remaining](
                                  mavlink_status_t* status,
                                  mavlink_message_t* message) {
            return mavlink_msg_sys_status_pack_status(
                config_.aircraft_system_id, config_.aircraft_component_id, status,
                message, 0, 0, 0, 0, voltage, current, remaining, 0, 0, 0, 0, 0,
                0, 0, 0, 0);
        });
    }

    // BATTERY_STATUS：无单体电压，仅填电流/剩余；三项有效标志全部缺失时不发送。
    bool SendBatteryStatus(const common::FlightStateSnapshot& state) {
        if (!state.battery_valid && !state.battery_current_valid &&
            !state.battery_remaining_valid) {
            return false;
        }
        std::array<uint16_t, 10> voltages{};
        voltages.fill(UINT16_MAX);
        // FlightStateSnapshot只有电池总压，没有单体电压；不得把总压伪装成首节电芯。
        // 总压通过SYS_STATUS.voltage_battery发送，BATTERY_STATUS单体数组保持未知。
        std::array<uint16_t, 4> voltages_ext{};
        const int16_t current = state.battery_current_valid
                                    ? ClampRounded<int16_t>(state.battery_current_a, 100.f,
                                                            INT16_MIN, INT16_MAX)
                                    : -1;
        const int8_t remaining = state.battery_remaining_valid
                                     ? ClampRounded<int8_t>(state.battery_remaining_pct, 1.f,
                                                            0, 100)
                                     : -1;
        return EncodeAndWrite([this, &voltages, &voltages_ext, current, remaining](
                                  mavlink_status_t* status,
                                  mavlink_message_t* message) {
            return mavlink_msg_battery_status_pack_status(
                config_.aircraft_system_id, config_.aircraft_component_id, status,
                message, 0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_UNKNOWN,
                INT16_MAX, voltages.data(), current, -1, -1, remaining, 0,
                MAV_BATTERY_CHARGE_STATE_UNDEFINED, voltages_ext.data(), 0, 0);
        });
    }

    // HOME_POSITION：仅在 Home 有效时发送，四元数取单位元（无姿态语义）。
    bool SendHomePosition(const common::FlightStateSnapshot& state) {
        if (!state.home_valid) {
            return false;
        }
        const std::array<float, 4> quaternion{1.f, 0.f, 0.f, 0.f};
        return EncodeAndWrite([this, &state, &quaternion](mavlink_status_t* status,
                                                          mavlink_message_t* message) {
            return mavlink_msg_home_position_pack_status(
                config_.aircraft_system_id, config_.aircraft_component_id, status,
                message, state.home_lat_1e7, state.home_lon_1e7,
                state.home_altitude_mm, 0.f, 0.f, 0.f, quaternion.data(), 0.f, 0.f,
                0.f, 0);
        });
    }

    // 统一错误记账：按第 1 次与每满 100 次节流打印。
    void RecordError(const char* message) {
        const uint64_t count = error_count_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (ShouldLogThrottled(count)) {
            SPDLOG_ERROR("{}，累计 {}", message, count);
        }
    }

    GroundStationLinkConfig config_;
    SerialPort serial_;
    MavlinkHandler mavlink_;
    mutable std::mutex lifecycle_mutex_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::thread worker_;

    common::Topic<common::FlightStateSnapshot>* flight_state_topic_ = nullptr;
    common::Topic<common::MissionStatus>* mission_status_topic_ = nullptr;
    common::Topic<common::HealthStatus>* health_topic_ = nullptr;
    common::Topic<common::MissionAssignmentAck>* mission_assignment_ack_topic_ = nullptr;
    common::Topic<common::NetFireAck>* net_fire_ack_topic_ = nullptr;
    common::Topic<common::FenceStatus>* fence_status_topic_ = nullptr;
    common::Topic<common::MissionCommandAck>* mission_command_ack_topic_ = nullptr;
    common::Topic<common::FenceZone>* fence_zone_report_topic_ = nullptr;
    FlightSubscription flight_state_subscription_;
    HealthSubscription health_subscription_;
    MissionSubscription mission_status_subscription_;
    AssignmentAckSubscription mission_assignment_ack_subscription_;
    NetFireAckSubscription net_fire_ack_subscription_;
    FenceStatusSubscription fence_status_subscription_;
    FenceZoneReportSubscription fence_zone_report_subscription_;
    uint64_t fence_report_send_count_ = 0;
    uint64_t fence_report_invalid_count_ = 0;
    MissionCommandAckSubscription mission_command_ack_subscription_;
    std::optional<common::FlightStateSnapshot> latest_state_;
    std::optional<common::HealthStatus> latest_health_;
    std::optional<common::MissionStatus> latest_mission_status_;
    common::Topic<common::GroundStationTarget> target_output_;
    common::Topic<common::MissionAssignment> mission_assignment_output_;
    common::Topic<common::NetFireCommand> net_fire_command_output_;
    common::Topic<common::FenceZone> fence_zone_output_;
    common::Topic<common::MissionCommand> mission_command_output_;
    common::Topic<common::FenceZone> fence_zone_report_output_;
    uint32_t last_ground_station_boot_id_ = 0;
    uint32_t last_target_update_seq_ = 0;
    uint64_t target_v2_extension_count_ = 0;
    uint64_t target_update_receive_count_ = 0;
    uint64_t target_update_address_mismatch_count_ = 0;
    uint64_t target_update_invalid_count_ = 0;
    uint64_t target_publish_count_ = 0;
    uint64_t target_ack_send_count_ = 0;
    uint64_t mission_assign_receive_count_ = 0;
    uint64_t mission_assign_address_mismatch_count_ = 0;
    uint64_t mission_assign_invalid_count_ = 0;
    uint64_t mission_assign_publish_count_ = 0;
    uint64_t mission_assign_ack_send_count_ = 0;
    uint64_t net_fire_receive_count_ = 0;
    uint64_t net_fire_address_mismatch_count_ = 0;
    uint64_t net_fire_invalid_count_ = 0;
    uint64_t net_fire_publish_count_ = 0;
    uint64_t net_fire_ack_send_count_ = 0;
    uint64_t net_fire_unconsumed_count_ = 0;
    bool net_fire_consumer_present_ = false;
    uint64_t fence_receive_count_ = 0;
    uint64_t fence_address_mismatch_count_ = 0;
    uint64_t fence_invalid_count_ = 0;
    uint64_t fence_publish_count_ = 0;
    uint64_t fence_status_send_count_ = 0;
    common::FenceState last_fence_state_ = common::FenceState::kNoZone;
    uint16_t last_fence_zone_id_ = 0;
    uint16_t last_fence_version_ = 0;
    uint64_t mission_command_receive_count_ = 0;
    uint64_t mission_command_invalid_count_ = 0;
    uint64_t mission_command_address_mismatch_count_ = 0;
    uint64_t mission_command_publish_count_ = 0;
    uint64_t mission_command_ack_send_count_ = 0;

    uint64_t last_gcs_heartbeat_ms_ = 0;

    // 串口重连状态机（A-01）：读错误累计 → 阈值触发 → 关闭端口 → 定时重开。
    bool serial_reconnect_pending_ = false;
    uint32_t serial_read_error_streak_ = 0;
    uint64_t serial_reconnect_attempts_ = 0;
    uint64_t next_serial_reconnect_ms_ = 0;
    uint64_t serial_reconnect_count_ = 0;
    uint64_t serial_reconnect_failure_count_ = 0;

    GroundStationTimeSyncState time_sync_state_ =
        GroundStationTimeSyncState::kUnsynchronized;
    std::vector<TimeSyncSample> time_sync_samples_;
    std::deque<int64_t> pending_time_sync_requests_;
    int64_t time_sync_offset_ns_ = 0;
    uint64_t time_sync_round_trip_time_ns_ = 0;
    uint64_t time_sync_jitter_ns_ = 0;
    int64_t last_time_sync_sample_ns_ = 0;
    uint64_t time_sync_request_count_ = 0;
    uint64_t time_sync_response_count_ = 0;
    uint64_t time_sync_rejected_sample_count_ = 0;
    uint64_t time_sync_request_timeout_count_ = 0;
    uint64_t time_sync_degraded_count_ = 0;
    int64_t last_time_sync_publish_ns_ = 0;
    mutable std::mutex time_sync_status_mutex_;
    GroundStationTimeSyncStatus time_sync_status_;

    std::atomic<uint64_t> send_count_{0};
    std::atomic<uint64_t> receive_count_{0};
    std::atomic<uint64_t> error_count_{0};
};

GroundStationLink::GroundStationLink(GroundStationLinkConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}
GroundStationLink::~GroundStationLink() = default;
bool GroundStationLink::Start() { return impl_->Start(); }
void GroundStationLink::Stop() { impl_->Stop(); }
bool GroundStationLink::IsRunning() const { return impl_->IsRunning(); }
bool GroundStationLink::IsConnected() const { return impl_->IsConnected(); }
GroundStationTimeSyncStatus GroundStationLink::GetTimeSyncStatus() const {
    return impl_->GetTimeSyncStatus();
}
common::Topic<common::GroundStationTarget>& GroundStationLink::TargetOutput() {
    return impl_->TargetOutput();
}
common::Topic<common::MissionAssignment>&
GroundStationLink::MissionAssignmentOutput() {
    return impl_->MissionAssignmentOutput();
}
void GroundStationLink::SetMissionAssignmentAckInput(
    common::Topic<common::MissionAssignmentAck>& topic) {
    impl_->SetMissionAssignmentAckInput(topic);
}
common::Topic<common::NetFireCommand>& GroundStationLink::NetFireCommandOutput() {
    return impl_->NetFireCommandOutput();
}
void GroundStationLink::SetNetFireAckInput(
    common::Topic<common::NetFireAck>& topic) {
    impl_->SetNetFireAckInput(topic);
}
void GroundStationLink::SetNetFireConsumerPresent(bool present) {
    impl_->SetNetFireConsumerPresent(present);
}
common::Topic<common::FenceZone>& GroundStationLink::FenceZoneOutput() {
    return impl_->FenceZoneOutput();
}
void GroundStationLink::SetFenceStatusInput(
    common::Topic<common::FenceStatus>& topic) {
    impl_->SetFenceStatusInput(topic);
}
void GroundStationLink::SetFenceZoneReportInput(
    common::Topic<common::FenceZone>& topic) {
    impl_->SetFenceZoneReportInput(topic);
}
common::Topic<common::MissionCommand>& GroundStationLink::MissionCommandOutput() {
    return impl_->MissionCommandOutput();
}
void GroundStationLink::SetMissionCommandAckInput(
    common::Topic<common::MissionCommandAck>& topic) {
    impl_->SetMissionCommandAckInput(topic);
}
void GroundStationLink::SetFlightStateInput(
    common::Topic<common::FlightStateSnapshot>& topic) {
    impl_->SetFlightStateInput(topic);
}
void GroundStationLink::SetMissionStatusInput(
    common::Topic<common::MissionStatus>& topic) {
    impl_->SetMissionStatusInput(topic);
}
void GroundStationLink::SetHealthInput(common::Topic<common::HealthStatus>& topic) {
    impl_->SetHealthInput(topic);
}
uint64_t GroundStationLink::SendCount() const { return impl_->SendCount(); }
uint64_t GroundStationLink::ReceiveCount() const { return impl_->ReceiveCount(); }
uint64_t GroundStationLink::ErrorCount() const { return impl_->ErrorCount(); }

GroundStationLinkStub::GroundStationLinkStub() {
    SPDLOG_INFO("地面站通信部件骨架创建");
}
GroundStationLinkStub::~GroundStationLinkStub() {
    SPDLOG_INFO("地面站通信部件骨架销毁");
}
bool GroundStationLinkStub::Start() {
    running_ = true;
    SPDLOG_INFO("地面站通信部件骨架启动");
    return true;
}
void GroundStationLinkStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("地面站通信部件骨架停止");
}
bool GroundStationLinkStub::IsRunning() const { return running_; }
bool GroundStationLinkStub::IsConnected() const { return false; }
GroundStationTimeSyncStatus GroundStationLinkStub::GetTimeSyncStatus() const {
    return {};
}
common::Topic<common::GroundStationTarget>& GroundStationLinkStub::TargetOutput() {
    return target_output_;
}
common::Topic<common::MissionAssignment>&
GroundStationLinkStub::MissionAssignmentOutput() {
    return mission_assignment_output_;
}
void GroundStationLinkStub::SetMissionAssignmentAckInput(
    common::Topic<common::MissionAssignmentAck>&) {}
common::Topic<common::NetFireCommand>&
GroundStationLinkStub::NetFireCommandOutput() {
    return net_fire_command_output_;
}
void GroundStationLinkStub::SetNetFireAckInput(common::Topic<common::NetFireAck>&) {}
common::Topic<common::FenceZone>& GroundStationLinkStub::FenceZoneOutput() {
    return fence_zone_output_;
}
void GroundStationLinkStub::SetFenceStatusInput(
    common::Topic<common::FenceStatus>&) {}
void GroundStationLinkStub::SetFenceZoneReportInput(
    common::Topic<common::FenceZone>&) {}
common::Topic<common::MissionCommand>&
GroundStationLinkStub::MissionCommandOutput() {
    return mission_command_output_;
}
void GroundStationLinkStub::SetMissionCommandAckInput(
    common::Topic<common::MissionCommandAck>&) {}
void GroundStationLinkStub::SetFlightStateInput(
    common::Topic<common::FlightStateSnapshot>&) {}
void GroundStationLinkStub::SetMissionStatusInput(
    common::Topic<common::MissionStatus>&) {}
void GroundStationLinkStub::SetHealthInput(common::Topic<common::HealthStatus>&) {}
uint64_t GroundStationLinkStub::SendCount() const { return send_count_; }
uint64_t GroundStationLinkStub::ReceiveCount() const { return receive_count_; }
uint64_t GroundStationLinkStub::ErrorCount() const { return error_count_; }

}  // namespace drone::communication
