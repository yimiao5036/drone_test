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
            latest_state_.reset();
            latest_health_.reset();
            latest_mission_status_.reset();
            last_ground_station_boot_id_ = 0;
            last_target_update_seq_ = 0;
            last_gcs_heartbeat_ms_ = 0;
            ResetTimeSync(true);
            connected_.store(false, std::memory_order_release);
            running_.store(true, std::memory_order_release);
            worker_ = std::thread(&Impl::WorkerLoop, this);
        } catch (const std::exception& error) {
            running_.store(false, std::memory_order_release);
            connected_.store(false, std::memory_order_release);
            flight_state_subscription_.Reset();
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
        latest_state_.reset();
        latest_health_.reset();
        latest_mission_status_.reset();
        serial_.Close();
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

    // 工作线程主循环：阻塞读串口 → 增量解析 MAVLink → 消费飞行快照 →
    // 心跳/时间同步超时检查 → 各类遥测按各自截止时间发送。
    void WorkerLoop() {
        const auto now = std::chrono::steady_clock::now();
        Deadlines deadlines{now, now, now, now, now, now, now, now, now, now, now, now};
        std::array<uint8_t, 512> read_buffer{};

        while (running_.load(std::memory_order_acquire)) {
            // 一次读取尽可能多的串口字节并交给 MavlinkHandler 增量解析。
            const std::ptrdiff_t count = serial_.Read(read_buffer.data(), read_buffer.size());
            if (count > 0) {
                mavlink_.Feed(read_buffer.data(), static_cast<std::size_t>(count),
                              [this](const mavlink_message_t& message) {
                                  HandleMessage(message);
                              });
            } else if (count < 0) {
                error_count_.fetch_add(1, std::memory_order_relaxed);
                std::this_thread::sleep_for(config_.serial.read_timeout);
            }

            // 把各输入队列里最新的状态捞出，并判断是否有“事件”需要立即下行。
            DrainFlightState();
            DrainStatusInputs();
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
    template <typename Sender>
    void SendIfDue(const std::chrono::steady_clock::time_point& now,
                   std::chrono::steady_clock::time_point& deadline,
                   std::chrono::milliseconds interval, Sender&& sender) {
        if (now < deadline) {
            return;
        }
        (void)sender();
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
                               next.power_status_bits != latest_mission_status_->power_status_bits;
            latest_mission_status_ = next;
            if (event) {
                (void)SendMissionStatus(next);
            }
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

    // 处理 V2_EXTENSION：只收地面站来源，识别目标协议 message_type 并转入更新处理。
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
        if (extension.message_type != kTrackTargetUpdateMessageType) {
            return;
        }
        HandleTrackTargetUpdate(extension);
    }

    // 处理 TRACK_TARGET_UPDATE：依次做地址匹配、协议版本、坐标、身份、时间同步、
    // 序号、字段范围与时效校验，全部通过后发布 GroundStationTarget 并回 ACCEPTED ACK；
    // 任一失败按语义回对应拒绝 ACK。
    void HandleTrackTargetUpdate(const mavlink_v2_extension_t& extension) {
        const uint64_t receive_count = ++target_update_receive_count_;
        if (ShouldLogThrottled(receive_count)) {
            SPDLOG_INFO("收到目标UPDATE: target={}/{} count={}",
                        extension.target_system, extension.target_component,
                        receive_count);
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

        const auto payload = DecodeTrackTargetUpdate(extension.payload);
        if (!payload) {
            SendTrackTargetAck(0, 0, 0, 0, TrackTargetAckResult::kRejectedInvalidField,
                               TrackTargetAckReason::kInternalError);
            return;
        }
        const auto& update = *payload;
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
    std::optional<TrackTargetUpdatePayload> DecodeTrackTargetUpdate(
        const uint8_t* data) const {
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

    bool SendHealthStatus(const common::HealthStatus& health) {
        std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
        WriteLe<uint8_t>(payload, 0, kStatusProtocolVersion);
        WriteLe<uint32_t>(payload, 1, static_cast<uint32_t>(health.header.sequence));
        WriteLe<uint32_t>(payload, 5, health.link_health_bits);
        WriteLe<uint32_t>(payload, 9, health.device_health_bits);
        WriteLe<uint32_t>(payload, 13, health.data_freshness_bits);
        WriteLe<uint32_t>(payload, 17, health.error_bits);
        WriteLe<uint32_t>(payload, 21, CpuLoadX100(health.cpu_load_pct));
        WriteLe<uint32_t>(payload, 25, health.timeout_event_count);
        WriteLe<uint8_t>(payload, 29, config_.aircraft_system_id);
        WriteLe<uint8_t>(payload, 30, config_.aircraft_component_id);
        WriteLe<uint8_t>(payload, 31, 0);
        WriteLe<uint64_t>(payload, 32, health.header.receive_time_ms);
        // 保留尾字节非零，确保 MAVLink V2_EXTENSION 短帧 payload_len 固定为60。
        WriteLe<uint8_t>(payload, 54, 1);
        return SendStatusExtension(kHealthStatusMessageType, payload);
    }

    bool SendMissionStatus(const common::MissionStatus& mission) {
        std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
        WriteLe<uint8_t>(payload, 0, kStatusProtocolVersion);
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
        // 保留尾字节非零，确保 MAVLink V2_EXTENSION 短帧 payload_len 固定为60。
        WriteLe<uint8_t>(payload, 54, 1);
        return SendStatusExtension(kMissionStatusMessageType, payload);
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
        if (time_sync_samples_.size() >= config_.time_sync_minimum_samples &&
            jitter_ns <= max_jitter_ns) {
            const auto previous_state = time_sync_state_;
            time_sync_state_ = GroundStationTimeSyncState::kSynchronized;
            if (previous_state != time_sync_state_) {
                SPDLOG_INFO(
                    "地面站时间同步建立: offset_ms={:.3f} rtt_ms={:.3f} jitter_ms={:.3f} samples={}",
                    static_cast<double>(time_sync_offset_ns_) / 1000000.0,
                    static_cast<double>(time_sync_round_trip_time_ns_) / 1000000.0,
                    static_cast<double>(time_sync_jitter_ns_) / 1000000.0,
                    time_sync_samples_.size());
            }
        }
        PublishTimeSyncStatus(now_ns);
    }

    // 同步样本老化超时：超过 time_sync_timeout 即退回未同步。
    void CheckTimeSyncTimeout(uint64_t now_ms) {
        const int64_t now_ns = static_cast<int64_t>(now_ms) * 1000000LL;
        PrunePendingTimeSyncRequests(now_ns);
        if (last_time_sync_sample_ns_ == 0 || now_ns < last_time_sync_sample_ns_) {
            PublishTimeSyncStatus(now_ns);
            return;
        }
        const uint64_t sample_age_ms =
            static_cast<uint64_t>(now_ns - last_time_sync_sample_ns_) / 1000000ULL;
        if (sample_age_ms <=
            static_cast<uint64_t>(config_.time_sync_timeout.count())) {
            PublishTimeSyncStatus(now_ns);
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

    // 把当前时间同步快照发布到 status（带锁复制，供诊断线程查询）。
    void PublishTimeSyncStatus(int64_t now_ns) {
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
    bool SendGps(const common::FlightStateSnapshot& state) {
        if (!state.gps_state_valid) {
            return false;
        }
        const int32_t latitude = state.global_position_valid ? state.latitude_1e7 : 0;
        const int32_t longitude = state.global_position_valid ? state.longitude_1e7 : 0;
        const int32_t altitude = state.global_position_valid ? state.altitude_mm : 0;
        return EncodeAndWrite([this, &state, latitude, longitude, altitude](
                                  mavlink_status_t* status,
                                  mavlink_message_t* message) {
            return mavlink_msg_gps_raw_int_pack_status(
                config_.aircraft_system_id, config_.aircraft_component_id, status,
                message, 0, state.gps_fix_type, latitude, longitude, altitude,
                UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT8_MAX, 0,
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
    FlightSubscription flight_state_subscription_;
    HealthSubscription health_subscription_;
    MissionSubscription mission_status_subscription_;
    std::optional<common::FlightStateSnapshot> latest_state_;
    std::optional<common::HealthStatus> latest_health_;
    std::optional<common::MissionStatus> latest_mission_status_;
    common::Topic<common::GroundStationTarget> target_output_;
    uint32_t last_ground_station_boot_id_ = 0;
    uint32_t last_target_update_seq_ = 0;
    uint64_t target_v2_extension_count_ = 0;
    uint64_t target_update_receive_count_ = 0;
    uint64_t target_update_address_mismatch_count_ = 0;
    uint64_t target_publish_count_ = 0;
    uint64_t target_ack_send_count_ = 0;

    uint64_t last_gcs_heartbeat_ms_ = 0;

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
void GroundStationLinkStub::SetFlightStateInput(
    common::Topic<common::FlightStateSnapshot>&) {}
void GroundStationLinkStub::SetMissionStatusInput(
    common::Topic<common::MissionStatus>&) {}
void GroundStationLinkStub::SetHealthInput(common::Topic<common::HealthStatus>&) {}
uint64_t GroundStationLinkStub::SendCount() const { return send_count_; }
uint64_t GroundStationLinkStub::ReceiveCount() const { return receive_count_; }
uint64_t GroundStationLinkStub::ErrorCount() const { return error_count_; }

}  // namespace drone::communication
