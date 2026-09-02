#include "communication/ground_station_link.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
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
    if (heartbeat_send_interval.count() <= 0 || heartbeat_timeout.count() <= 0 ||
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
}

class GroundStationLink::Impl final {
public:
    explicit Impl(GroundStationLinkConfig config)
        : config_(std::move(config)),
          serial_(config_.serial),
          mavlink_(config_.mavlink_version == 1 ? MavlinkVersion::kV1
                                                : MavlinkVersion::kV2) {
        config_.Validate();
        SPDLOG_INFO("地面站通信部件创建: serial={}@{} aircraft={}/{} type={} callsign={} gcs={}/{} mavlink={}",
                    config_.serial.device, config_.serial.baud_rate,
                    config_.aircraft_system_id, config_.aircraft_component_id,
                    config_.aircraft_type, config_.callsign,
                    config_.ground_system_id, config_.ground_component_id,
                    config_.mavlink_version);
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
            last_gcs_heartbeat_ms_ = 0;
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
        SPDLOG_INFO("地面站通信停止: serial={}", config_.serial.device);
    }

    bool IsRunning() const { return running_.load(std::memory_order_acquire); }
    bool IsConnected() const { return connected_.load(std::memory_order_acquire); }

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
        Deadlines deadlines{now, now, now, now, now, now, now, now, now};
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
            CheckHeartbeatTimeout(MonotonicMs());
            const auto current = std::chrono::steady_clock::now();
            if (current >= deadlines.heartbeat) {
                SendHeartbeat();
                deadlines.heartbeat = current + config_.heartbeat_send_interval;
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
        if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
            return;
        }
        mavlink_heartbeat_t heartbeat{};
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        if (heartbeat.type != MAV_TYPE_GCS) {
            return;
        }
        if (message.sysid != config_.ground_system_id ||
            message.compid != config_.ground_component_id) {
            return;
        }
        last_gcs_heartbeat_ms_ = MonotonicMs();
        if (!connected_.exchange(true, std::memory_order_acq_rel)) {
            SPDLOG_INFO("地面站心跳建立: system={} component={}", message.sysid,
                        message.compid);
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
            SPDLOG_WARN("地面站心跳超时: timeout_ms={}",
                        config_.heartbeat_timeout.count());
        }
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

    uint64_t last_gcs_heartbeat_ms_ = 0;
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
