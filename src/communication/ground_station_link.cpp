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

uint64_t MonotonicMs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

int64_t MonotonicNs() {
    return static_cast<int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

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

template <typename T>
T ClampRounded(float value, float scale, T minimum, T maximum) {
    if (!std::isfinite(value)) {
        return minimum;
    }
    const double scaled = std::round(static_cast<double>(value) * scale);
    return static_cast<T>(std::clamp(scaled, static_cast<double>(minimum),
                                     static_cast<double>(maximum)));
}

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

template <typename T>
T ReadLe(const uint8_t* payload, std::size_t offset) {
    T value{};
    std::memcpy(&value, payload + offset, sizeof(T));
    return value;
}

template <typename T>
void WriteLe(std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN>& payload,
             std::size_t offset, T value) {
    std::memcpy(payload.data() + offset, &value, sizeof(T));
}

}  // namespace

void GroundStationLinkConfig::Validate() const {
    serial.Validate();
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
        battery_send_interval.count() <= 0 || home_send_interval.count() <= 0) {
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
            serial_.Flush();
            mavlink_.Reset();
            flight_state_subscription_ = flight_state_topic_->Subscribe(
                config_.flight_state_queue_capacity,
                common::Topic<common::FlightStateSnapshot>::OverflowPolicy::kDropOldest);
            latest_state_.reset();
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

    void Stop() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (!running_.exchange(false, std::memory_order_acq_rel)) {
            return;
        }
        if (worker_.joinable()) {
            worker_.join();
        }
        flight_state_subscription_.Reset();
        latest_state_.reset();
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

    struct Deadlines {
        std::chrono::steady_clock::time_point heartbeat;
        std::chrono::steady_clock::time_point time_sync;
        std::chrono::steady_clock::time_point attitude;
        std::chrono::steady_clock::time_point local_position;
        std::chrono::steady_clock::time_point global_position;
        std::chrono::steady_clock::time_point gps;
        std::chrono::steady_clock::time_point extended_state;
        std::chrono::steady_clock::time_point system_status;
        std::chrono::steady_clock::time_point battery;
        std::chrono::steady_clock::time_point home;
    };

    void WorkerLoop() {
        const auto now = std::chrono::steady_clock::now();
        Deadlines deadlines{now, now, now, now, now, now, now, now, now, now};
        std::array<uint8_t, 512> read_buffer{};

        while (running_.load(std::memory_order_acquire)) {
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

            DrainFlightState();
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
        const auto reject = [&](TrackTargetAckResult result,
                                TrackTargetAckReason reason) {
            SendTrackTargetAck(update.ground_station_boot_id, update.update_seq,
                               update.target_id, measured_age_ms, result, reason);
        };

        if (update.protocol_version != kTrackTargetProtocolVersion) {
            reject(TrackTargetAckResult::kRejectedUnsupportedVersion,
                   TrackTargetAckReason::kProtocolVersionUnsupported);
            return;
        }
        if (update.coordinate_frame != kTrackTargetCoordinateFrameWgs84) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kLatitudeOrLongitudeInvalid);
            return;
        }
        if (update.ground_station_boot_id == 0) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kBootIdInvalidOrChanged);
            return;
        }
        if (time_sync_state_ != GroundStationTimeSyncState::kSynchronized) {
            reject(TrackTargetAckResult::kRejectedTimeSyncUnavailable,
                   TrackTargetAckReason::kTimeSyncUnavailable);
            return;
        }
        if (last_ground_station_boot_id_ != 0 &&
            update.ground_station_boot_id != last_ground_station_boot_id_) {
            last_ground_station_boot_id_ = update.ground_station_boot_id;
            last_target_update_seq_ = 0;
            ResetTimeSync(false);
            reject(TrackTargetAckResult::kRejectedTimeSyncUnavailable,
                   TrackTargetAckReason::kBootIdInvalidOrChanged);
            return;
        }
        if (update.update_seq <= last_target_update_seq_) {
            reject(TrackTargetAckResult::kRejectedStaleOrDuplicate,
                   TrackTargetAckReason::kUpdateSequenceStale);
            return;
        }
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
        if (update.valid_for_ms <
                static_cast<uint32_t>(config_.target_minimum_valid_for.count()) ||
            update.valid_for_ms >
                static_cast<uint32_t>(config_.target_maximum_valid_for.count())) {
            reject(TrackTargetAckResult::kRejectedInvalidField,
                   TrackTargetAckReason::kValidForInvalid);
            return;
        }
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
        if (measured_age_ms >
                static_cast<uint64_t>(config_.target_maximum_transport_delay.count()) ||
            measured_age_ms >= update.valid_for_ms) {
            reject(TrackTargetAckResult::kRejectedStaleOrDuplicate,
                   TrackTargetAckReason::kTargetExpired);
            return;
        }
        const uint64_t remaining_valid_ms = update.valid_for_ms - measured_age_ms;
        if (remaining_valid_ms <
            static_cast<uint64_t>(config_.target_minimum_remaining_valid.count())) {
            reject(TrackTargetAckResult::kRejectedStaleOrDuplicate,
                   TrackTargetAckReason::kRemainingValidityTooShort);
            return;
        }

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

    std::chrono::milliseconds TimeSyncSendInterval() const {
        return time_sync_state_ == GroundStationTimeSyncState::kSynchronized
                   ? config_.time_sync_steady_interval
                   : config_.time_sync_acquire_interval;
    }

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

    struct TimeSyncSample {
        int64_t offset_ns = 0;
        uint64_t round_trip_time_ns = 0;
        int64_t receive_time_ns = 0;
    };

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
    std::optional<common::FlightStateSnapshot> latest_state_;
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
