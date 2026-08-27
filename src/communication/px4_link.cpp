#include "communication/px4_link.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <exception>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

#include "communication/mavlink_handler.h"

namespace drone::communication {

namespace {

bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

uint64_t MonotonicMs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace

void Px4LinkConfig::Validate() const {
    serial.Validate();
    if (firmware_version.empty()) {
        throw std::invalid_argument("PX4 固件版本标识不能为空");
    }
    if (onboard_system_id == 0 || onboard_component_id == 0) {
        throw std::invalid_argument("机载电脑 MAVLink system/component ID 不能为 0");
    }
    if (target_system_id == 0 || target_component_id == 0) {
        throw std::invalid_argument("PX4 MAVLink system/component ID 不能为 0");
    }
    if (mavlink_version != 1 && mavlink_version != 2) {
        throw std::invalid_argument("MAVLink 版本必须是 1 或 2");
    }
    if (serial.read_timeout.count() <= 0 ||
        heartbeat_send_interval.count() <= 0 || heartbeat_timeout.count() <= 0 ||
        telemetry_timeout.count() <= 0 || state_publish_interval.count() <= 0 ||
        reconnect_interval.count() <= 0 || command_ack_timeout.count() <= 0) {
        throw std::invalid_argument("PX4 链路读取、遥测、命令 ACK、周期与超时必须为正数");
    }
    if (setpoint_queue_capacity == 0 || command_queue_capacity == 0) {
        throw std::invalid_argument("PX4 设定值和命令队列容量必须大于 0");
    }
    for (uint32_t message_id : one_shot_message_requests) {
        if (message_id > 0xFFFFFFU) {
            throw std::invalid_argument("MAVLink 单次请求消息 ID 超出 24 位范围");
        }
    }
    for (const auto& request : message_interval_requests) {
        if (request.message_id > 0xFFFFFFU || request.interval_us <= 0) {
            throw std::invalid_argument("MAVLink 消息频率请求 ID 必须为 24 位且周期必须为正数");
        }
    }
}

class Px4Link::Impl final {
public:
    explicit Impl(Px4LinkConfig config)
        : config_(std::move(config)),
          serial_(config_.serial),
          mavlink_(config_.mavlink_version == 1 ? MavlinkVersion::kV1
                                                : MavlinkVersion::kV2) {
        config_.Validate();
        SPDLOG_INFO("PX4 通信部件创建: firmware={} device={} baud={} onboard={}/{} target={}/{} mavlink={}",
                    config_.firmware_version, config_.serial.device,
                    config_.serial.baud_rate, config_.onboard_system_id,
                    config_.onboard_component_id, config_.target_system_id,
                    config_.target_component_id, config_.mavlink_version);
    }

    ~Impl() {
        Stop();
        SPDLOG_INFO("PX4 通信部件销毁: device={}", config_.serial.device);
    }

    bool Start() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (running_.load(std::memory_order_acquire)) {
            return true;
        }

        try {
            serial_.Open();
            serial_.Flush();
            mavlink_.Reset();
            state_ = common::FlightStateSnapshot{};
            state_sequence_ = 0;
            last_heartbeat_time_ms_ = 0;
            ResetTelemetryTimes();
            ClearCommands();
            next_command_sequence_.store(1, std::memory_order_relaxed);
            telemetry_requests_queued_ = false;
            firmware_version_logged_ = false;
            connected_.store(false, std::memory_order_release);

            if (setpoint_topic_ != nullptr) {
                setpoint_subscription_ = setpoint_topic_->Subscribe(
                    config_.setpoint_queue_capacity,
                    common::Topic<common::Px4Setpoint>::OverflowPolicy::kDropOldest);
            }

            running_.store(true, std::memory_order_release);
            worker_ = std::thread(&Impl::WorkerLoop, this);
        } catch (const std::exception& error) {
            running_.store(false, std::memory_order_release);
            connected_.store(false, std::memory_order_release);
            setpoint_subscription_.Reset();
            ClearCommands();
            serial_.Close();
            const uint64_t count = error_count_.fetch_add(1, std::memory_order_relaxed) + 1;
            if (ShouldLogThrottled(count)) {
                SPDLOG_ERROR("PX4 通信启动失败: device={} error={}，累计 {}",
                             config_.serial.device, error.what(), count);
            }
            return false;
        }

        SPDLOG_INFO("PX4 通信启动: device={}", config_.serial.device);
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
        setpoint_subscription_.Reset();
        ClearCommands();
        serial_.Close();
        connected_.store(false, std::memory_order_release);
        SPDLOG_INFO("PX4 通信停止: device={}", config_.serial.device);
    }

    bool IsRunning() const {
        return running_.load(std::memory_order_acquire);
    }

    bool IsConnected() const {
        return connected_.load(std::memory_order_acquire);
    }

    void SetInput(common::Topic<common::Px4Setpoint>& setpoint) {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (running_.load(std::memory_order_acquire)) {
            const uint64_t count = error_count_.fetch_add(1, std::memory_order_relaxed) + 1;
            if (ShouldLogThrottled(count)) {
                SPDLOG_ERROR("PX4 设定值 Topic 必须在 Start 前绑定，累计 {}", count);
            }
            return;
        }
        setpoint_topic_ = &setpoint;
    }

    common::Topic<common::FlightStateSnapshot>& StateOutput() {
        return state_output_;
    }

    bool SendCommand(uint16_t command, float param1, float param2, float param3,
                     float param4, float param5, float param6, float param7) {
        if (!running_.load(std::memory_order_acquire) ||
            !connected_.load(std::memory_order_acquire)) {
            return false;
        }
        CommandRequest request;
        request.sequence = next_command_sequence_.fetch_add(1, std::memory_order_relaxed);
        request.command = command;
        request.params = {param1, param2, param3, param4, param5, param6, param7};
        request.internal = false;
        return EnqueueCommand(std::move(request));
    }

    uint64_t SetpointSendCount() const {
        return setpoint_send_count_.load(std::memory_order_relaxed);
    }

    uint64_t ReceiveCount() const {
        return receive_count_.load(std::memory_order_relaxed);
    }

    uint64_t AckMatchCount() const {
        return ack_match_count_.load(std::memory_order_relaxed);
    }

    uint64_t AckTimeoutCount() const {
        return ack_timeout_count_.load(std::memory_order_relaxed);
    }

    uint64_t ErrorCount() const {
        return error_count_.load(std::memory_order_relaxed);
    }

    uint64_t MessageReceiveCount(uint32_t message_id) const {
        if (message_id >= message_receive_counts_.size()) {
            return 0;
        }
        return message_receive_counts_[message_id].load(std::memory_order_relaxed);
    }

private:
    using SetpointSubscription = common::Topic<common::Px4Setpoint>::Subscription;

    struct CommandRequest {
        uint64_t sequence = 0;
        uint16_t command = 0;
        std::array<float, 7> params{};
        bool internal = false;
    };

    struct InFlightCommand {
        CommandRequest request;
        uint64_t sent_time_ms = 0;
    };

    bool EnqueueCommand(CommandRequest request) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (command_queue_.size() >= config_.command_queue_capacity) {
            const uint64_t count = error_count_.fetch_add(1, std::memory_order_relaxed) + 1;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("PX4 命令队列已满: capacity={} command={}，累计错误 {}",
                            config_.command_queue_capacity, request.command, count);
            }
            return false;
        }
        command_queue_.push_back(std::move(request));
        return true;
    }

    void ClearCommands() {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_queue_.clear();
        in_flight_command_.reset();
    }

    void QueueTelemetryRequests() {
        for (uint32_t message_id : config_.one_shot_message_requests) {
            CommandRequest request;
            request.sequence = next_command_sequence_.fetch_add(1, std::memory_order_relaxed);
            request.command = MAV_CMD_REQUEST_MESSAGE;
            request.params[0] = static_cast<float>(message_id);
            request.internal = true;
            (void)EnqueueCommand(std::move(request));
        }
        for (const auto& interval : config_.message_interval_requests) {
            CommandRequest request;
            request.sequence = next_command_sequence_.fetch_add(1, std::memory_order_relaxed);
            request.command = MAV_CMD_SET_MESSAGE_INTERVAL;
            request.params[0] = static_cast<float>(interval.message_id);
            request.params[1] = static_cast<float>(interval.interval_us);
            request.internal = true;
            (void)EnqueueCommand(std::move(request));
        }
        if (!config_.one_shot_message_requests.empty() ||
            !config_.message_interval_requests.empty()) {
            SPDLOG_INFO("PX4 遥测请求已入队: one_shot={} intervals={}",
                        config_.one_shot_message_requests.size(),
                        config_.message_interval_requests.size());
        }
    }

    void WorkerLoop() {
        auto next_heartbeat = std::chrono::steady_clock::now();
        auto next_state_publish = next_heartbeat;
        std::array<uint8_t, 512> read_buffer{};

        while (running_.load(std::memory_order_acquire)) {
            const std::ptrdiff_t count = serial_.Read(read_buffer.data(), read_buffer.size());
            if (count > 0) {
                mavlink_.Feed(
                    read_buffer.data(), static_cast<std::size_t>(count),
                    [this](const mavlink_message_t& message) { HandleMessage(message); });
            } else if (count < 0) {
                error_count_.fetch_add(1, std::memory_order_relaxed);
                // 串口持续异常时避免通信线程无等待自旋；自动关闭/重连在下一阶段接入。
                std::this_thread::sleep_for(config_.serial.read_timeout);
            }

            const auto now = std::chrono::steady_clock::now();
            const uint64_t now_ms = MonotonicMs();
            CheckHeartbeatTimeout(now_ms);
            CheckCommandTimeout(now_ms);
            SendNextCommand(now_ms);

            if (now >= next_heartbeat) {
                SendHeartbeat();
                next_heartbeat = now + config_.heartbeat_send_interval;
            }
            if (now >= next_state_publish) {
                PublishState(now_ms);
                next_state_publish = now + config_.state_publish_interval;
            }
        }

        if (connected_.exchange(false, std::memory_order_acq_rel)) {
            SPDLOG_INFO("PX4 通信线程停止，连接状态置为断开");
        }
        state_.connected = false;
        InvalidateVolatileState();
        PublishState(MonotonicMs());
    }

    void HandleMessage(const mavlink_message_t& message) {
        if (message.sysid != config_.target_system_id ||
            message.compid != config_.target_component_id) {
            return;
        }

        receive_count_.fetch_add(1, std::memory_order_relaxed);
        if (message.msgid < message_receive_counts_.size()) {
            message_receive_counts_[message.msgid].fetch_add(1, std::memory_order_relaxed);
        }
        const uint64_t now_ms = MonotonicMs();
        switch (message.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                HandleHeartbeat(message, now_ms);
                break;
            case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
                HandleExtendedSystemState(message, now_ms);
                break;
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                HandleGlobalPosition(message, now_ms);
                break;
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                HandleLocalPosition(message, now_ms);
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                HandleAttitude(message, now_ms);
                break;
            case MAVLINK_MSG_ID_GPS_RAW_INT:
                HandleGps(message, now_ms);
                break;
            case MAVLINK_MSG_ID_SYS_STATUS:
                HandleSystemStatus(message, now_ms);
                break;
            case MAVLINK_MSG_ID_BATTERY_STATUS:
                HandleBatteryStatus(message, now_ms);
                break;
            case MAVLINK_MSG_ID_HOME_POSITION:
                HandleHomePosition(message);
                break;
            case MAVLINK_MSG_ID_RC_CHANNELS:
                HandleRcChannels(message, now_ms);
                break;
            case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                HandleAutopilotVersion(message);
                break;
            case MAVLINK_MSG_ID_COMMAND_ACK:
                HandleCommandAck(message, now_ms);
                break;
            default:
                break;
        }
    }

    void HandleHeartbeat(const mavlink_message_t& message, uint64_t now_ms) {
        mavlink_heartbeat_t heartbeat{};
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        if (heartbeat.autopilot != MAV_AUTOPILOT_PX4) {
            return;
        }

        last_heartbeat_time_ms_ = now_ms;
        state_.connected = true;
        state_.armed =
            (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        state_.base_mode = heartbeat.base_mode;
        state_.custom_mode = heartbeat.custom_mode;
        state_.flight_mode = static_cast<uint8_t>((heartbeat.custom_mode >> 16U) & 0xFFU);
        state_.flight_sub_mode = static_cast<uint8_t>((heartbeat.custom_mode >> 24U) & 0xFFU);
        state_.system_status = heartbeat.system_status;

        if (!connected_.exchange(true, std::memory_order_acq_rel)) {
            SPDLOG_INFO("PX4 心跳已连接: source={}/{} mode={}/{} armed={}",
                        message.sysid, message.compid, state_.flight_mode,
                        state_.flight_sub_mode, state_.armed);
            if (!telemetry_requests_queued_) {
                telemetry_requests_queued_ = true;
                QueueTelemetryRequests();
            }
        }
        PublishState(now_ms);
    }

    void HandleExtendedSystemState(const mavlink_message_t& message, uint64_t now_ms) {
        mavlink_extended_sys_state_t extended{};
        mavlink_msg_extended_sys_state_decode(&message, &extended);
        state_.landed_state_valid =
            extended.landed_state != MAV_LANDED_STATE_UNDEFINED;
        state_.landed = extended.landed_state == MAV_LANDED_STATE_ON_GROUND;
        last_landed_time_ms_ = now_ms;
    }

    void HandleGlobalPosition(const mavlink_message_t& message, uint64_t now_ms) {
        mavlink_global_position_int_t position{};
        mavlink_msg_global_position_int_decode(&message, &position);
        state_.latitude_1e7 = position.lat;
        state_.longitude_1e7 = position.lon;
        state_.altitude_mm = position.alt;
        state_.vx_mps = static_cast<float>(position.vx) / 100.f;
        state_.vy_mps = static_cast<float>(position.vy) / 100.f;
        state_.vz_mps = static_cast<float>(position.vz) / 100.f;
        state_.global_position_valid = true;
        last_global_position_time_ms_ = now_ms;
    }

    void HandleLocalPosition(const mavlink_message_t& message, uint64_t now_ms) {
        mavlink_local_position_ned_t position{};
        mavlink_msg_local_position_ned_decode(&message, &position);
        state_.local_x_m = position.x;
        state_.local_y_m = position.y;
        state_.local_z_m = position.z;
        state_.vx_mps = position.vx;
        state_.vy_mps = position.vy;
        state_.vz_mps = position.vz;
        state_.local_position_valid = true;
        last_local_position_time_ms_ = now_ms;
    }

    void HandleAttitude(const mavlink_message_t& message, uint64_t now_ms) {
        mavlink_attitude_t attitude{};
        mavlink_msg_attitude_decode(&message, &attitude);
        state_.roll_rad = attitude.roll;
        state_.pitch_rad = attitude.pitch;
        state_.yaw_rad = attitude.yaw;
        state_.attitude_valid = true;
        last_attitude_time_ms_ = now_ms;
    }

    void HandleGps(const mavlink_message_t& message, uint64_t now_ms) {
        mavlink_gps_raw_int_t gps{};
        mavlink_msg_gps_raw_int_decode(&message, &gps);
        state_.gps_state_valid = true;
        state_.gps_fix_type = gps.fix_type;
        state_.gps_fix = gps.fix_type >= GPS_FIX_TYPE_3D_FIX;
        last_gps_time_ms_ = now_ms;
    }

    void HandleSystemStatus(const mavlink_message_t& message, uint64_t now_ms) {
        mavlink_sys_status_t status{};
        mavlink_msg_sys_status_decode(&message, &status);
        if (status.voltage_battery != std::numeric_limits<uint16_t>::max()) {
            state_.battery_voltage_v =
                static_cast<float>(status.voltage_battery) / 1000.f;
            state_.battery_valid = true;
            last_battery_time_ms_ = now_ms;
        }
        state_.battery_current_valid = status.current_battery >= 0;
        if (state_.battery_current_valid) {
            state_.battery_current_a =
                static_cast<float>(status.current_battery) / 100.f;
        }
        state_.battery_remaining_valid = status.battery_remaining >= 0;
        if (state_.battery_remaining_valid) {
            state_.battery_remaining_pct =
                static_cast<float>(status.battery_remaining);
        }
    }

    void HandleBatteryStatus(const mavlink_message_t& message, uint64_t now_ms) {
        mavlink_battery_status_t battery{};
        mavlink_msg_battery_status_decode(&message, &battery);
        if (battery.id != 0) {
            return;  // V1 仅使用动力电池 0，避免多电池状态互相覆盖。
        }

        uint32_t voltage_mv = 0;
        for (std::size_t index = 0; index < 10; ++index) {
            const uint16_t cell_voltage = battery.voltages[index];
            if (cell_voltage != std::numeric_limits<uint16_t>::max()) {
                voltage_mv += cell_voltage;
            }
        }
        for (std::size_t index = 0; index < 4; ++index) {
            const uint16_t cell_voltage = battery.voltages_ext[index];
            if (cell_voltage > 0) {
                voltage_mv += cell_voltage;
            }
        }
        if (voltage_mv > 0) {
            state_.battery_voltage_v = static_cast<float>(voltage_mv) / 1000.f;
            state_.battery_valid = true;
            last_battery_time_ms_ = now_ms;
        }
        state_.battery_current_valid = battery.current_battery >= 0;
        if (state_.battery_current_valid) {
            state_.battery_current_a =
                static_cast<float>(battery.current_battery) / 100.f;
        }
        state_.battery_remaining_valid = battery.battery_remaining >= 0;
        if (state_.battery_remaining_valid) {
            state_.battery_remaining_pct =
                static_cast<float>(battery.battery_remaining);
        }
    }

    void HandleHomePosition(const mavlink_message_t& message) {
        mavlink_home_position_t home{};
        mavlink_msg_home_position_decode(&message, &home);
        state_.home_lat_1e7 = home.latitude;
        state_.home_lon_1e7 = home.longitude;
        state_.home_altitude_mm = home.altitude;
        state_.home_valid = true;
    }

    void HandleRcChannels(const mavlink_message_t& message, uint64_t now_ms) {
        mavlink_rc_channels_t rc{};
        mavlink_msg_rc_channels_decode(&message, &rc);
        state_.rc_rssi = rc.rssi;
        state_.rc_state_valid = true;
        rc_link_present_ = rc.chancount > 0 &&
                           (rc.rssi == std::numeric_limits<uint8_t>::max() || rc.rssi > 0);
        state_.rc_connected = rc_link_present_;
        last_rc_time_ms_ = now_ms;
    }

    void HandleAutopilotVersion(const mavlink_message_t& message) {
        mavlink_autopilot_version_t version{};
        mavlink_msg_autopilot_version_decode(&message, &version);
        state_.flight_sw_version = version.flight_sw_version;
        state_.firmware_major =
            static_cast<uint8_t>((version.flight_sw_version >> 24U) & 0xFFU);
        state_.firmware_minor =
            static_cast<uint8_t>((version.flight_sw_version >> 16U) & 0xFFU);
        state_.firmware_patch =
            static_cast<uint8_t>((version.flight_sw_version >> 8U) & 0xFFU);
        state_.autopilot_capabilities = version.capabilities;
        state_.autopilot_version_valid = true;

        if (!firmware_version_logged_) {
            firmware_version_logged_ = true;
            const std::string reported =
                std::to_string(state_.firmware_major) + "." +
                std::to_string(state_.firmware_minor) + "." +
                std::to_string(state_.firmware_patch);
            SPDLOG_INFO("PX4 固件版本上报: reported={} configured={} capabilities={}",
                        reported, config_.firmware_version,
                        state_.autopilot_capabilities);
            if (reported != config_.firmware_version) {
                SPDLOG_WARN("PX4 固件版本与配置不一致: reported={} configured={}",
                            reported, config_.firmware_version);
            }
        }
    }

    void HandleCommandAck(const mavlink_message_t& message, uint64_t now_ms) {
        mavlink_command_ack_t ack{};
        mavlink_msg_command_ack_decode(&message, &ack);
        state_.last_ack_command = ack.command;
        state_.last_ack_result = ack.result;
        state_.last_ack_time_ms = now_ms;

        if (!in_flight_command_ ||
            in_flight_command_->request.command != ack.command) {
            return;
        }

        ack_match_count_.fetch_add(1, std::memory_order_relaxed);
        const bool in_progress = ack.result == MAV_RESULT_IN_PROGRESS;
        SPDLOG_INFO("PX4 命令 ACK: sequence={} command={} result={} internal={} progress={}",
                    in_flight_command_->request.sequence, ack.command, ack.result,
                    in_flight_command_->request.internal, ack.progress);
        if (in_progress) {
            in_flight_command_->sent_time_ms = now_ms;
        } else {
            in_flight_command_.reset();
        }
        PublishState(now_ms);
    }

    void CheckCommandTimeout(uint64_t now_ms) {
        if (!in_flight_command_ || now_ms < in_flight_command_->sent_time_ms) {
            return;
        }
        const uint64_t timeout_ms =
            static_cast<uint64_t>(config_.command_ack_timeout.count());
        if (now_ms - in_flight_command_->sent_time_ms <= timeout_ms) {
            return;
        }

        const auto timed_out = in_flight_command_->request;
        in_flight_command_.reset();
        const uint64_t count =
            ack_timeout_count_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (ShouldLogThrottled(count)) {
            SPDLOG_WARN("PX4 命令 ACK 超时: sequence={} command={} internal={} timeout_ms={}，累计 {}",
                        timed_out.sequence, timed_out.command, timed_out.internal,
                        config_.command_ack_timeout.count(), count);
        }
    }

    void SendNextCommand(uint64_t now_ms) {
        if (in_flight_command_ || !connected_.load(std::memory_order_acquire)) {
            return;
        }

        CommandRequest request;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (command_queue_.empty()) {
                return;
            }
            request = std::move(command_queue_.front());
            command_queue_.pop_front();
        }

        const auto frame = mavlink_.Encode(
            [this, &request](mavlink_status_t* status, mavlink_message_t* message) {
                return mavlink_msg_command_long_pack_status(
                    config_.onboard_system_id, config_.onboard_component_id,
                    status, message, config_.target_system_id,
                    config_.target_component_id, request.command, 0,
                    request.params[0], request.params[1], request.params[2],
                    request.params[3], request.params[4], request.params[5],
                    request.params[6]);
            });
        if (frame.empty() || !serial_.Write(frame.data(), frame.size())) {
            error_count_.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        in_flight_command_ = InFlightCommand{std::move(request), now_ms};
    }

    bool IsTelemetryFresh(uint64_t timestamp_ms, uint64_t now_ms) const {
        if (timestamp_ms == 0 || now_ms < timestamp_ms) {
            return false;
        }
        return now_ms - timestamp_ms <=
               static_cast<uint64_t>(config_.telemetry_timeout.count());
    }

    void RefreshTelemetryValidity(uint64_t now_ms) {
        state_.landed_state_valid =
            state_.landed_state_valid && IsTelemetryFresh(last_landed_time_ms_, now_ms);
        state_.global_position_valid =
            IsTelemetryFresh(last_global_position_time_ms_, now_ms);
        state_.local_position_valid =
            IsTelemetryFresh(last_local_position_time_ms_, now_ms);
        state_.attitude_valid = IsTelemetryFresh(last_attitude_time_ms_, now_ms);
        state_.gps_state_valid = IsTelemetryFresh(last_gps_time_ms_, now_ms);
        state_.gps_fix = state_.gps_state_valid &&
                         state_.gps_fix_type >= GPS_FIX_TYPE_3D_FIX;
        state_.battery_valid = IsTelemetryFresh(last_battery_time_ms_, now_ms);
        if (!state_.battery_valid) {
            state_.battery_current_valid = false;
            state_.battery_remaining_valid = false;
        }
        state_.rc_state_valid = IsTelemetryFresh(last_rc_time_ms_, now_ms);
        state_.rc_connected = state_.rc_state_valid && rc_link_present_;
    }

    void InvalidateVolatileState() {
        ResetTelemetryTimes();
        state_.landed_state_valid = false;
        state_.global_position_valid = false;
        state_.local_position_valid = false;
        state_.attitude_valid = false;
        state_.gps_state_valid = false;
        state_.gps_fix = false;
        state_.battery_valid = false;
        state_.battery_current_valid = false;
        state_.battery_remaining_valid = false;
        state_.rc_state_valid = false;
        state_.rc_connected = false;
        state_.home_valid = false;
        rc_link_present_ = false;
    }

    void ResetTelemetryTimes() {
        last_landed_time_ms_ = 0;
        last_global_position_time_ms_ = 0;
        last_local_position_time_ms_ = 0;
        last_attitude_time_ms_ = 0;
        last_gps_time_ms_ = 0;
        last_battery_time_ms_ = 0;
        last_rc_time_ms_ = 0;
    }

    void CheckHeartbeatTimeout(uint64_t now_ms) {
        if (!connected_.load(std::memory_order_acquire) || last_heartbeat_time_ms_ == 0) {
            return;
        }
        const uint64_t timeout_ms = static_cast<uint64_t>(config_.heartbeat_timeout.count());
        if (now_ms - last_heartbeat_time_ms_ <= timeout_ms) {
            return;
        }

        connected_.store(false, std::memory_order_release);
        state_.connected = false;
        telemetry_requests_queued_ = false;
        ClearCommands();
        InvalidateVolatileState();
        SPDLOG_WARN("PX4 心跳超时: source={}/{} timeout_ms={}",
                    config_.target_system_id, config_.target_component_id,
                    config_.heartbeat_timeout.count());
        PublishState(now_ms);
    }

    void SendHeartbeat() {
        const auto frame = mavlink_.Encode(
            [this](mavlink_status_t* status, mavlink_message_t* message) {
                return mavlink_msg_heartbeat_pack_status(
                    config_.onboard_system_id, config_.onboard_component_id,
                    status, message, MAV_TYPE_ONBOARD_CONTROLLER,
                    MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
            });
        if (frame.empty() || !serial_.Write(frame.data(), frame.size())) {
            error_count_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void PublishState(uint64_t now_ms) {
        RefreshTelemetryValidity(now_ms);
        common::FlightStateSnapshot snapshot = state_;
        snapshot.connected = connected_.load(std::memory_order_acquire);
        snapshot.header.sequence = ++state_sequence_;
        snapshot.header.source_time_ms = 0;
        snapshot.header.receive_time_ms = now_ms;
        snapshot.header.valid_for_ms =
            static_cast<uint64_t>(config_.heartbeat_timeout.count());
        snapshot.header.source_id = config_.target_system_id;
        snapshot.header.health = snapshot.connected ? 1 : 3;
        snapshot.header.frame_id =
            (snapshot.local_position_valid || snapshot.global_position_valid) ? 3 : 0;
        (void)state_output_.Publish(
            std::make_shared<const common::FlightStateSnapshot>(std::move(snapshot)));
    }

    Px4LinkConfig config_;
    SerialPort serial_;
    MavlinkHandler mavlink_;

    mutable std::mutex lifecycle_mutex_;
    std::thread worker_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    common::Topic<common::Px4Setpoint>* setpoint_topic_ = nullptr;
    SetpointSubscription setpoint_subscription_;
    common::Topic<common::FlightStateSnapshot> state_output_;
    common::FlightStateSnapshot state_;
    uint64_t state_sequence_ = 0;
    uint64_t last_heartbeat_time_ms_ = 0;
    uint64_t last_landed_time_ms_ = 0;
    uint64_t last_global_position_time_ms_ = 0;
    uint64_t last_local_position_time_ms_ = 0;
    uint64_t last_attitude_time_ms_ = 0;
    uint64_t last_gps_time_ms_ = 0;
    uint64_t last_battery_time_ms_ = 0;
    uint64_t last_rc_time_ms_ = 0;
    bool rc_link_present_ = false;
    bool firmware_version_logged_ = false;
    bool telemetry_requests_queued_ = false;

    std::mutex command_mutex_;
    std::deque<CommandRequest> command_queue_;
    std::optional<InFlightCommand> in_flight_command_;
    std::atomic<uint64_t> next_command_sequence_{1};

    std::atomic<uint64_t> setpoint_send_count_{0};
    std::atomic<uint64_t> receive_count_{0};
    std::atomic<uint64_t> ack_match_count_{0};
    std::atomic<uint64_t> ack_timeout_count_{0};
    std::atomic<uint64_t> error_count_{0};
    std::array<std::atomic<uint64_t>, 256> message_receive_counts_{};
};

Px4Link::Px4Link(Px4LinkConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

Px4Link::~Px4Link() = default;

bool Px4Link::Start() { return impl_->Start(); }
void Px4Link::Stop() { impl_->Stop(); }
bool Px4Link::IsRunning() const { return impl_->IsRunning(); }
bool Px4Link::IsConnected() const { return impl_->IsConnected(); }
void Px4Link::SetInput(common::Topic<common::Px4Setpoint>& setpoint) {
    impl_->SetInput(setpoint);
}
common::Topic<common::FlightStateSnapshot>& Px4Link::StateOutput() {
    return impl_->StateOutput();
}
bool Px4Link::SendCommand(uint16_t command, float param1, float param2,
                          float param3, float param4, float param5,
                          float param6, float param7) {
    return impl_->SendCommand(command, param1, param2, param3, param4,
                              param5, param6, param7);
}
uint64_t Px4Link::SetpointSendCount() const { return impl_->SetpointSendCount(); }
uint64_t Px4Link::ReceiveCount() const { return impl_->ReceiveCount(); }
uint64_t Px4Link::AckMatchCount() const { return impl_->AckMatchCount(); }
uint64_t Px4Link::AckTimeoutCount() const { return impl_->AckTimeoutCount(); }
uint64_t Px4Link::ErrorCount() const { return impl_->ErrorCount(); }
uint64_t Px4Link::MessageReceiveCount(uint32_t message_id) const {
    return impl_->MessageReceiveCount(message_id);
}

Px4LinkStub::Px4LinkStub() {
    SPDLOG_INFO("PX4 通信部件骨架创建");
}

Px4LinkStub::~Px4LinkStub() {
    SPDLOG_INFO("PX4 通信部件骨架销毁");
}

bool Px4LinkStub::Start() {
    running_ = true;
    SPDLOG_INFO("PX4 通信部件骨架启动");
    return true;
}

void Px4LinkStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("PX4 通信部件骨架停止");
}

bool Px4LinkStub::IsRunning() const {
    return running_;
}

bool Px4LinkStub::IsConnected() const {
    return false;
}

void Px4LinkStub::SetInput(common::Topic<common::Px4Setpoint>&) {}

common::Topic<common::FlightStateSnapshot>& Px4LinkStub::StateOutput() {
    return state_output_;
}

bool Px4LinkStub::SendCommand(uint16_t, float, float, float, float, float, float, float) {
    ++error_count_;
    if (ShouldLogThrottled(error_count_)) {
        SPDLOG_WARN("PX4 通信部件 SendCommand 未实现（骨架占位），累计调用 {}", error_count_);
    }
    return false;
}

uint64_t Px4LinkStub::SetpointSendCount() const { return setpoint_send_count_; }
uint64_t Px4LinkStub::ReceiveCount() const { return receive_count_; }
uint64_t Px4LinkStub::AckMatchCount() const { return ack_match_count_; }
uint64_t Px4LinkStub::AckTimeoutCount() const { return ack_timeout_count_; }
uint64_t Px4LinkStub::ErrorCount() const { return error_count_; }

}  // namespace drone::communication
