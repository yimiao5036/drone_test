#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include <sys/types.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "common/logger.h"
#include "communication/mavlink_handler.h"
#include "communication/px4_link.h"

namespace {

using json = nlohmann::json;
using drone::communication::Px4LinkConfig;

std::atomic<bool> g_stop{false};

void OnSignal(int) {
    g_stop.store(true, std::memory_order_release);
}

std::string ExecDir() {
    char buffer[4096];
    const ssize_t length = ::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (length <= 0) {
        return ".";
    }
    buffer[length] = '\0';
    std::string path(buffer);
    const auto separator = path.find_last_of('/');
    return separator == std::string::npos ? "." : path.substr(0, separator);
}

struct Options {
    std::string config_path;
    std::chrono::seconds duration{30};
    bool sitl_zero_velocity = false;
    bool sitl_offboard_disarmed = false;
    bool sitl_arm_zero_velocity = false;
    bool sitl_takeoff_land = false;
    bool sitl_horizontal_motion = false;
    bool sitl_offboard_loss = false;
};

void PrintUsage(const char* program) {
    std::cout << "用法: " << program
              << " [--config <config.json>] [--duration <秒>]\n"
              << "       [--sitl-zero-velocity | --sitl-offboard-disarmed | --sitl-arm-zero-velocity\n"
              << "        | --sitl-takeoff-land | --sitl-horizontal-motion\n"
              << "        | --sitl-offboard-loss]\n"
              << "SITL 参数仅允许 UDP；offboard-loss 起飞 1m 后主动停止 setpoint，\n"
              << "验证 PX4 退出 Offboard，再统一 AUTO.LAND 回收。\n";
}

Options ParseOptions(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help" || argument == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        }
        if (argument == "--config") {
            if (++index >= argc) {
                throw std::invalid_argument("--config 缺少路径");
            }
            options.config_path = argv[index];
            continue;
        }
        if (argument == "--sitl-zero-velocity") {
            options.sitl_zero_velocity = true;
            continue;
        }
        if (argument == "--sitl-offboard-disarmed") {
            options.sitl_offboard_disarmed = true;
            continue;
        }
        if (argument == "--sitl-arm-zero-velocity") {
            options.sitl_arm_zero_velocity = true;
            continue;
        }
        if (argument == "--sitl-takeoff-land") {
            options.sitl_takeoff_land = true;
            continue;
        }
        if (argument == "--sitl-horizontal-motion") {
            options.sitl_horizontal_motion = true;
            continue;
        }
        if (argument == "--sitl-offboard-loss") {
            options.sitl_offboard_loss = true;
            continue;
        }
        if (argument == "--duration") {
            if (++index >= argc) {
                throw std::invalid_argument("--duration 缺少秒数");
            }
            const int seconds = std::stoi(argv[index]);
            if (seconds <= 0 || seconds > 3600) {
                throw std::invalid_argument("--duration 必须在 1~3600 秒之间");
            }
            options.duration = std::chrono::seconds(seconds);
            continue;
        }
        throw std::invalid_argument("未知参数: " + argument);
    }
    const int sitl_stage_count = static_cast<int>(options.sitl_zero_velocity) +
                                 static_cast<int>(options.sitl_offboard_disarmed) +
                                 static_cast<int>(options.sitl_arm_zero_velocity) +
                                 static_cast<int>(options.sitl_takeoff_land) +
                                 static_cast<int>(options.sitl_horizontal_motion) +
                                 static_cast<int>(options.sitl_offboard_loss);
    if (sitl_stage_count > 1) {
        throw std::invalid_argument("多个 SITL 阶段参数不能同时使用");
    }
    if (options.sitl_arm_zero_velocity && options.duration < std::chrono::seconds(10)) {
        throw std::invalid_argument("解锁零速度测试时长不能少于 10 秒");
    }
    if (options.sitl_takeoff_land && options.duration < std::chrono::seconds(20)) {
        throw std::invalid_argument("受限起飞降落测试时长不能少于 20 秒");
    }
    if (options.sitl_horizontal_motion && options.duration < std::chrono::seconds(30)) {
        throw std::invalid_argument("受限水平运动测试时长不能少于 30 秒");
    }
    if (options.sitl_offboard_loss && options.duration < std::chrono::seconds(25)) {
        throw std::invalid_argument("Offboard 丢失保护测试时长不能少于 25 秒");
    }
    return options;
}

std::string ResolveConfigPath(const std::string& requested) {
    if (!requested.empty()) {
        std::ifstream requested_file(requested);
        if (!requested_file.is_open()) {
            throw std::runtime_error("无法打开配置文件: " + requested);
        }
        return requested;
    }

    const std::string beside_executable = ExecDir() + "/config/config.json";
    std::ifstream first(beside_executable);
    if (first.is_open()) {
        return beside_executable;
    }

    const std::string current_directory = "./config/config.json";
    std::ifstream second(current_directory);
    if (second.is_open()) {
        return current_directory;
    }
    throw std::runtime_error("未找到 config/config.json，请使用 --config 指定路径");
}

json LoadJson(const std::string& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("无法打开配置文件: " + path);
    }
    try {
        return json::parse(input);
    } catch (const std::exception& error) {
        throw std::runtime_error("配置文件解析失败: " + std::string(error.what()));
    }
}

uint8_t ReadUint8(const json& object, const char* key) {
    const int value = object.at(key).get<int>();
    if (value <= 0 || value > 255) {
        throw std::invalid_argument(std::string(key) + " 必须在 1~255 之间");
    }
    return static_cast<uint8_t>(value);
}

std::chrono::milliseconds ReadPositiveMilliseconds(const json& object,
                                                   const char* key) {
    const int64_t value = object.at(key).get<int64_t>();
    if (value <= 0) {
        throw std::invalid_argument(std::string(key) + " 必须为正数");
    }
    return std::chrono::milliseconds(value);
}

Px4LinkConfig LoadPx4Config(const json& root) {
    const json& identity = root.at("mavlink");
    const json& px4 = root.at("px4");
    const json& serial = px4.at("serial");
    const json& udp = px4.at("udp");

    Px4LinkConfig config;
    config.transport = px4.at("transport").get<std::string>();
    config.firmware_version = px4.at("firmware_version").get<std::string>();
    config.onboard_system_id = ReadUint8(identity, "onboard_system_id");
    config.onboard_component_id = ReadUint8(identity, "onboard_component_id");
    config.target_system_id = ReadUint8(px4, "target_system_id");
    config.target_component_id = ReadUint8(px4, "target_component_id");
    config.mavlink_version = ReadUint8(px4, "mavlink_version");
    config.heartbeat_send_interval =
        ReadPositiveMilliseconds(px4, "heartbeat_send_interval_ms");
    config.heartbeat_timeout = ReadPositiveMilliseconds(px4, "heartbeat_timeout_ms");
    config.telemetry_timeout = ReadPositiveMilliseconds(px4, "telemetry_timeout_ms");
    config.state_publish_interval =
        ReadPositiveMilliseconds(px4, "state_publish_interval_ms");
    config.reconnect_interval = ReadPositiveMilliseconds(px4, "reconnect_interval_ms");
    config.command_ack_timeout =
        ReadPositiveMilliseconds(px4, "command_ack_timeout_ms");
    config.setpoint_send_interval =
        ReadPositiveMilliseconds(px4, "setpoint_send_interval_ms");
    config.setpoint_timeout = ReadPositiveMilliseconds(px4, "setpoint_timeout_ms");

    const int64_t queue_capacity = px4.at("setpoint_queue_capacity").get<int64_t>();
    if (queue_capacity <= 0) {
        throw std::invalid_argument("setpoint_queue_capacity 必须为正数");
    }
    config.setpoint_queue_capacity = static_cast<std::size_t>(queue_capacity);
    const int64_t command_capacity = px4.at("command_queue_capacity").get<int64_t>();
    if (command_capacity <= 0) {
        throw std::invalid_argument("command_queue_capacity 必须为正数");
    }
    config.command_queue_capacity = static_cast<std::size_t>(command_capacity);

    for (const auto& item : px4.at("one_shot_message_requests")) {
        const int64_t message_id = item.get<int64_t>();
        if (message_id < 0 || message_id > 0xFFFFFF) {
            throw std::invalid_argument("one_shot_message_requests 包含非法消息 ID");
        }
        config.one_shot_message_requests.push_back(static_cast<uint32_t>(message_id));
    }
    for (const auto& item : px4.at("message_interval_requests")) {
        const int64_t message_id = item.at("message_id").get<int64_t>();
        const int64_t interval_us = item.at("interval_us").get<int64_t>();
        if (message_id < 0 || message_id > 0xFFFFFF ||
            interval_us <= 0 || interval_us > std::numeric_limits<int32_t>::max()) {
            throw std::invalid_argument("message_interval_requests 包含非法 ID 或周期");
        }
        config.message_interval_requests.push_back(
            {static_cast<uint32_t>(message_id), static_cast<int32_t>(interval_us)});
    }

    config.serial.device = serial.at("device").get<std::string>();
    config.serial.baud_rate = serial.at("baud_rate").get<int>();
    config.serial.data_bits = ReadUint8(serial, "data_bits");
    config.serial.stop_bits = ReadUint8(serial, "stop_bits");
    const std::string parity = serial.at("parity").get<std::string>();
    if (parity.size() != 1) {
        throw std::invalid_argument("serial.parity 必须是单个字符 N/E/O");
    }
    config.serial.parity = parity.front();
    config.serial.read_timeout = ReadPositiveMilliseconds(serial, "read_timeout_ms");
    config.serial.write_timeout = ReadPositiveMilliseconds(serial, "write_timeout_ms");

    config.udp.bind_address = udp.at("bind_address").get<std::string>();
    config.udp.bind_port = static_cast<uint16_t>(udp.at("bind_port").get<int>());
    config.udp.remote_address = udp.at("remote_address").get<std::string>();
    config.udp.remote_port = static_cast<uint16_t>(udp.at("remote_port").get<int>());
    config.udp.read_timeout = ReadPositiveMilliseconds(udp, "read_timeout_ms");
    config.udp.write_timeout = ReadPositiveMilliseconds(udp, "write_timeout_ms");
    config.Validate();
    return config;
}

std::string ResolveLogDirectory(const json& root) {
    const json log = root.value("log", json::object());
    return log.value("dir", std::string("logs/"));
}

struct ObservedState {
    bool heartbeat = false;
    bool version = false;
    bool landed = false;
    bool global_position = false;
    bool local_position = false;
    bool attitude = false;
    bool gps = false;
    bool gps_3d_fix = false;
    bool battery = false;
    bool home = false;
    bool rc = false;
};

void UpdateObserved(const drone::common::FlightStateSnapshot& state,
                    ObservedState* observed) {
    observed->heartbeat = observed->heartbeat || state.connected;
    observed->version = observed->version || state.autopilot_version_valid;
    observed->landed = observed->landed || state.landed_state_valid;
    observed->global_position = observed->global_position || state.global_position_valid;
    observed->local_position = observed->local_position || state.local_position_valid;
    observed->attitude = observed->attitude || state.attitude_valid;
    observed->gps = observed->gps || state.gps_state_valid;
    observed->gps_3d_fix = observed->gps_3d_fix || state.gps_fix;
    observed->battery = observed->battery || state.battery_valid;
    observed->home = observed->home || state.home_valid;
    observed->rc = observed->rc || state.rc_state_valid;
}

uint64_t MonotonicMs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

const char* YesNo(bool value) {
    return value ? "是" : "否";
}

void PrintSummary(const drone::common::FlightStateSnapshot& state,
                  std::chrono::seconds elapsed) {
    std::cout << "[" << elapsed.count() << "s]"
              << " connected=" << YesNo(state.connected)
              << " armed=" << YesNo(state.armed)
              << " mode=" << static_cast<int>(state.flight_mode) << '/'
              << static_cast<int>(state.flight_sub_mode)
              << " landed=" << (state.landed_state_valid ? YesNo(state.landed) : "未知")
              << " gps=" << (state.gps_fix ? "3D+" : "无/不足")
              << " global=" << YesNo(state.global_position_valid)
              << " local=" << YesNo(state.local_position_valid)
              << " attitude=" << YesNo(state.attitude_valid)
              << " battery=" << YesNo(state.battery_valid)
              << " home=" << YesNo(state.home_valid)
              << " rc=" << YesNo(state.rc_connected)
              << std::endl;
}

void PrintCheck(const char* name, bool passed, bool required) {
    const char* level = passed ? "PASS" : (required ? "FAIL" : "WARN");
    std::cout << '[' << level << "] " << name << std::endl;
}

void PrintMessageCheck(const char* name, uint64_t count, bool valid_seen) {
    if (count == 0) {
        std::cout << "[WARN] " << name << " 未收到" << std::endl;
    } else if (!valid_seen) {
        std::cout << "[WARN] " << name << " 收到 " << count
                  << " 条，但未形成有效状态" << std::endl;
    } else {
        std::cout << "[PASS] " << name << " 收到 " << count << " 条" << std::endl;
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = ParseOptions(argc, argv);
        const std::string config_path = ResolveConfigPath(options.config_path);
        const json root = LoadJson(config_path);
        const Px4LinkConfig config = LoadPx4Config(root);
        const bool sitl_flight_test =
            options.sitl_takeoff_land || options.sitl_horizontal_motion ||
            options.sitl_offboard_loss;
        const bool sitl_arm_test =
            options.sitl_arm_zero_velocity || sitl_flight_test;
        const bool sitl_offboard_test =
            options.sitl_offboard_disarmed || sitl_arm_test;
        const bool sitl_setpoint_test =
            options.sitl_zero_velocity || sitl_offboard_test;
        if (sitl_setpoint_test && config.transport != "udp") {
            throw std::invalid_argument("SITL 设定值参数只允许 transport=udp，禁止在真实串口运行");
        }

        const json log_config = root.value("log", json::object());
        const auto log_level = spdlog::level::from_str(
            log_config.value("level", std::string("info")));
        drone::common::InitializeAsyncLogger(ResolveLogDirectory(root), log_level);

        std::signal(SIGINT, OnSignal);
        std::signal(SIGTERM, OnSignal);

        std::cout << "PX4 链路冒烟测试\n"
                  << "  config: " << config_path << '\n'
                  << "  transport: " << config.transport << '\n'
                  << "  device: " << config.serial.device << '\n'
                  << "  baud: " << config.serial.baud_rate << '\n'
                  << "  onboard: " << static_cast<int>(config.onboard_system_id) << '/'
                  << static_cast<int>(config.onboard_component_id) << '\n'
                  << "  target: " << static_cast<int>(config.target_system_id) << '/'
                  << static_cast<int>(config.target_component_id) << '\n'
                  << "  firmware: " << config.firmware_version << '\n'
                  << "  duration: " << options.duration.count() << " 秒\n"
                  << "  安全边界: "
                  << (options.sitl_offboard_loss
                          ? "SITL UDP 起飞 1m 后停止 setpoint，验证 Offboard 丢失保护"
                          : (options.sitl_horizontal_motion
                                 ? "SITL UDP 起飞 1m、北向 0.5m/s 运动 2 秒、制动返回并降落"
                                 : (options.sitl_takeoff_land
                                 ? "SITL UDP 相对起飞 1m、悬停 3 秒、自动降落并主动上锁"
                                 : (options.sitl_arm_zero_velocity
                                 ? "SITL UDP 进入 Offboard 后解锁，始终零速度，结束前主动上锁"
                                 : (options.sitl_offboard_disarmed
                                 ? "SITL UDP 零速度流后切 Offboard，强制保持不解锁"
                                 : (options.sitl_zero_velocity
                                        ? "SITL UDP 零速度流，不切模式、不解锁"
                                        : "仅发送 HEARTBEAT 和遥测请求命令，不发送飞行控制指令"))))))
                  << '\n'
                  << std::endl;

        drone::communication::Px4Link link(config);
        drone::common::Topic<drone::common::Px4Setpoint> setpoint_topic;
        if (sitl_setpoint_test) {
            link.SetInput(setpoint_topic);
        }
        auto subscription = link.StateOutput().Subscribe(32);
        if (!link.Start()) {
            std::cerr << "[FAIL] PX4 串口启动失败，请检查设备、权限和波特率" << std::endl;
            return 2;
        }

        ObservedState observed;
        drone::common::FlightStateSnapshot latest;
        bool have_snapshot = false;
        bool last_connected = false;
        bool last_armed = false;
        uint8_t last_mode = 0;
        uint8_t last_sub_mode = 0;

        const auto started = std::chrono::steady_clock::now();
        auto next_summary = started;
        auto next_setpoint_publish = started;
        bool offboard_command_queued = false;
        bool offboard_ack_received = false;
        bool offboard_ack_accepted = false;
        bool offboard_mode_reached = false;
        uint64_t offboard_request_time_ms = 0;
        bool remained_disarmed = true;
        bool arm_command_queued = false;
        bool arm_ack_received = false;
        bool arm_ack_accepted = false;
        bool armed_reached = false;
        uint64_t arm_request_time_ms = 0;
        uint64_t armed_since_ms = 0;
        bool unexpected_disarm_before_command = false;
        bool arm_hold_complete = false;
        bool disarm_command_queued = false;
        bool disarm_ack_received = false;
        bool disarm_ack_accepted = false;
        bool disarmed_confirmed = false;
        uint64_t disarm_request_time_ms = 0;
        bool arm_start_position_valid = false;
        float arm_start_x = 0.f;
        float arm_start_y = 0.f;
        float arm_start_z = 0.f;
        float max_displacement_m = 0.f;
        bool takeoff_target_ready = false;
        float takeoff_target_x = 0.f;
        float takeoff_target_y = 0.f;
        float takeoff_target_z = 0.f;
        bool takeoff_height_reached = false;
        uint64_t hover_stable_since_ms = 0;
        bool hover_complete = false;
        float max_takeoff_height_m = 0.f;
        float max_horizontal_drift_m = 0.f;
        bool flight_safety_violation = false;
        bool land_command_queued = false;
        bool land_ack_received = false;
        bool land_ack_accepted = false;
        uint64_t land_request_time_ms = 0;
        bool landed_after_flight = false;
        bool horizontal_motion_started = false;
        uint64_t horizontal_motion_started_ms = 0;
        bool horizontal_motion_complete = false;
        bool horizontal_braking = false;
        uint64_t brake_stable_since_ms = 0;
        bool horizontal_brake_complete = false;
        bool returning_to_start = false;
        uint64_t return_stable_since_ms = 0;
        bool returned_to_start = false;
        float max_north_displacement_m = 0.f;
        float max_east_deviation_m = 0.f;
        float motion_command_end_north_displacement_m = 0.f;
        float brake_end_north_displacement_m = 0.f;
        bool intentional_setpoint_loss = false;
        uint64_t setpoint_loss_started_ms = 0;
        uint64_t setpoint_count_at_loss = 0;
        bool offboard_loss_mode_exited = false;
        uint64_t offboard_loss_mode_exited_ms = 0;
        uint8_t offboard_loss_reaction_mode = 0;
        uint8_t offboard_loss_reaction_sub_mode = 0;
        bool offboard_loss_stable = false;
        while (!g_stop.load(std::memory_order_acquire) &&
               std::chrono::steady_clock::now() - started < options.duration) {
            if (auto message = subscription.WaitTakeFor(std::chrono::milliseconds(100))) {
                latest = **message;
                have_snapshot = true;
                UpdateObserved(latest, &observed);

                if (latest.connected != last_connected || latest.armed != last_armed ||
                    latest.flight_mode != last_mode ||
                    latest.flight_sub_mode != last_sub_mode) {
                    std::cout << "[状态变化] connected=" << YesNo(latest.connected)
                              << " armed=" << YesNo(latest.armed)
                              << " mode=" << static_cast<int>(latest.flight_mode) << '/'
                              << static_cast<int>(latest.flight_sub_mode) << std::endl;
                    last_connected = latest.connected;
                    last_armed = latest.armed;
                    last_mode = latest.flight_mode;
                    last_sub_mode = latest.flight_sub_mode;
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (sitl_setpoint_test && link.IsConnected() &&
                !land_command_queued && !intentional_setpoint_loss &&
                now >= next_setpoint_publish) {
                drone::common::Px4Setpoint setpoint;
                setpoint.header.receive_time_ms = MonotonicMs();
                setpoint.header.valid_for_ms = 300;
                setpoint.header.frame_id = 3;
                if (options.sitl_horizontal_motion && horizontal_motion_started &&
                    !horizontal_motion_complete) {
                    setpoint.type = drone::common::SetpointType::kVelocity;
                    setpoint.x = 0.5f;
                    setpoint.y = 0.f;
                    setpoint.z = 0.f;
                } else if (options.sitl_horizontal_motion && horizontal_braking &&
                           !horizontal_brake_complete) {
                    setpoint.type = drone::common::SetpointType::kVelocity;
                    setpoint.x = 0.f;
                    setpoint.y = 0.f;
                    setpoint.z = 0.f;
                } else if (sitl_flight_test && takeoff_target_ready) {
                    setpoint.type = drone::common::SetpointType::kPosition;
                    setpoint.x = takeoff_target_x;
                    setpoint.y = takeoff_target_y;
                    setpoint.z = takeoff_target_z;
                } else {
                    setpoint.type = drone::common::SetpointType::kVelocity;
                    setpoint.x = 0.f;
                    setpoint.y = 0.f;
                    setpoint.z = 0.f;
                }
                setpoint.valid = true;
                (void)setpoint_topic.Emplace(setpoint);
                next_setpoint_publish = now + std::chrono::milliseconds(100);
            }
            if (sitl_offboard_test && link.IsConnected() &&
                !offboard_command_queued &&
                now - started >= std::chrono::seconds(2) &&
                link.SetpointSendCount() >= 20) {
                constexpr uint8_t kPx4MainModeOffboard = 6;
                offboard_command_queued = link.SendCommand(
                    MAV_CMD_DO_SET_MODE,
                    static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                    static_cast<float>(kPx4MainModeOffboard), 0.f,
                    0.f, 0.f, 0.f, 0.f);
                if (offboard_command_queued) {
                    offboard_request_time_ms = MonotonicMs();
                }
                std::cout << "[SITL] Offboard 模式请求"
                          << (offboard_command_queued ? "已入队" : "入队失败")
                          << std::endl;
            }
            if (have_snapshot) {
                if (offboard_command_queued && !offboard_ack_received &&
                    latest.last_ack_time_ms >= offboard_request_time_ms &&
                    latest.last_ack_command == MAV_CMD_DO_SET_MODE) {
                    offboard_ack_received = true;
                    offboard_ack_accepted =
                        latest.last_ack_result == MAV_RESULT_ACCEPTED;
                }
                if (arm_command_queued && !arm_ack_received &&
                    latest.last_ack_time_ms >= arm_request_time_ms &&
                    latest.last_ack_command == MAV_CMD_COMPONENT_ARM_DISARM) {
                    arm_ack_received = true;
                    arm_ack_accepted =
                        latest.last_ack_result == MAV_RESULT_ACCEPTED;
                }
                if (land_command_queued && !land_ack_received &&
                    latest.last_ack_time_ms >= land_request_time_ms &&
                    latest.last_ack_command == MAV_CMD_DO_SET_MODE) {
                    land_ack_received = true;
                    land_ack_accepted =
                        latest.last_ack_result == MAV_RESULT_ACCEPTED;
                }
                offboard_mode_reached = offboard_mode_reached ||
                                        (offboard_command_queued &&
                                         latest.flight_mode == 6);
                if (options.sitl_offboard_disarmed) {
                    remained_disarmed = remained_disarmed && !latest.armed;
                }
                if (sitl_arm_test && latest.armed) {
                    if (!armed_reached) {
                        armed_since_ms = MonotonicMs();
                    }
                    armed_reached = true;
                    if (!arm_start_position_valid && latest.local_position_valid) {
                        arm_start_position_valid = true;
                        arm_start_x = latest.local_x_m;
                        arm_start_y = latest.local_y_m;
                        arm_start_z = latest.local_z_m;
                        if (sitl_flight_test) {
                            takeoff_target_ready = true;
                            takeoff_target_x = arm_start_x;
                            takeoff_target_y = arm_start_y;
                            takeoff_target_z = arm_start_z - 1.f;
                            std::cout << "[SITL] 起飞目标已锁定: N="
                                      << takeoff_target_x << " E=" << takeoff_target_y
                                      << " D=" << takeoff_target_z << std::endl;
                        }
                    }
                    if (arm_start_position_valid && latest.local_position_valid) {
                        const float dx = latest.local_x_m - arm_start_x;
                        const float dy = latest.local_y_m - arm_start_y;
                        const float dz = latest.local_z_m - arm_start_z;
                        max_displacement_m = std::max(
                            max_displacement_m, std::sqrt(dx * dx + dy * dy + dz * dz));
                    }
                }
                if (sitl_flight_test && armed_reached && latest.armed &&
                    !land_command_queued &&
                    (!latest.connected ||
                     (!intentional_setpoint_loss && latest.flight_mode != 6) ||
                     !latest.local_position_valid || !latest.attitude_valid)) {
                    if (!flight_safety_violation) {
                        flight_safety_violation = true;
                        std::cout << "[SITL][安全回收] 连接、Offboard、位置或姿态失效"
                                  << std::endl;
                    }
                }
                if (sitl_flight_test && arm_start_position_valid &&
                    latest.local_position_valid && latest.armed) {
                    const float ascent_m = arm_start_z - latest.local_z_m;
                    const float dx = latest.local_x_m - arm_start_x;
                    const float dy = latest.local_y_m - arm_start_y;
                    const float horizontal_drift_m = std::sqrt(dx * dx + dy * dy);
                    max_takeoff_height_m = std::max(max_takeoff_height_m, ascent_m);
                    max_horizontal_drift_m =
                        std::max(max_horizontal_drift_m, horizontal_drift_m);
                    max_north_displacement_m =
                        std::max(max_north_displacement_m, latest.local_x_m - arm_start_x);
                    max_east_deviation_m =
                        std::max(max_east_deviation_m,
                                 std::fabs(latest.local_y_m - arm_start_y));
                    const bool height_out_of_range =
                        ascent_m > 1.30f ||
                        (horizontal_motion_started && ascent_m < 0.65f);
                    const bool horizontal_out_of_range =
                        options.sitl_horizontal_motion
                            ? (latest.local_x_m - arm_start_x > 1.50f ||
                               std::fabs(latest.local_y_m - arm_start_y) > 0.30f)
                            : horizontal_drift_m > 0.50f;
                    if (!land_command_queued && !flight_safety_violation &&
                        (height_out_of_range || horizontal_out_of_range)) {
                        flight_safety_violation = true;
                        std::cout << "[SITL][安全回收] 高度或水平位移超过硬限制"
                                  << std::endl;
                    }
                    const bool inside_hover_window =
                        std::fabs(latest.local_z_m - takeoff_target_z) <= 0.15f &&
                        horizontal_drift_m <= 0.30f && std::fabs(latest.vz_mps) <= 0.30f;
                    if (!horizontal_motion_started && inside_hover_window) {
                        takeoff_height_reached = true;
                        if (hover_stable_since_ms == 0) {
                            hover_stable_since_ms = MonotonicMs();
                        }
                        hover_complete =
                            MonotonicMs() - hover_stable_since_ms >= 3000;
                    } else if (!horizontal_motion_started) {
                        hover_stable_since_ms = 0;
                    }
                    if (horizontal_braking && !horizontal_brake_complete) {
                        const float horizontal_speed_mps =
                            std::sqrt(latest.vx_mps * latest.vx_mps +
                                      latest.vy_mps * latest.vy_mps);
                        if (horizontal_speed_mps <= 0.20f &&
                            std::fabs(latest.vz_mps) <= 0.20f) {
                            if (brake_stable_since_ms == 0) {
                                brake_stable_since_ms = MonotonicMs();
                            }
                            horizontal_brake_complete =
                                MonotonicMs() - brake_stable_since_ms >= 1000;
                        } else {
                            brake_stable_since_ms = 0;
                        }
                    }
                    if (returning_to_start && horizontal_brake_complete) {
                        const float horizontal_error_m = horizontal_drift_m;
                        const float horizontal_speed_mps =
                            std::sqrt(latest.vx_mps * latest.vx_mps +
                                      latest.vy_mps * latest.vy_mps);
                        const bool return_window =
                            horizontal_error_m <= 0.20f &&
                            std::fabs(latest.local_z_m - takeoff_target_z) <= 0.15f &&
                            horizontal_speed_mps <= 0.20f &&
                            std::fabs(latest.vz_mps) <= 0.20f;
                        if (return_window) {
                            if (return_stable_since_ms == 0) {
                                return_stable_since_ms = MonotonicMs();
                            }
                            returned_to_start =
                                MonotonicMs() - return_stable_since_ms >= 2000;
                        } else {
                            return_stable_since_ms = 0;
                        }
                    }
                }
                if (options.sitl_offboard_loss && intentional_setpoint_loss &&
                    !offboard_loss_mode_exited && latest.flight_mode != 6) {
                    offboard_loss_mode_exited = true;
                    offboard_loss_mode_exited_ms = MonotonicMs();
                    offboard_loss_reaction_mode = latest.flight_mode;
                    offboard_loss_reaction_sub_mode = latest.flight_sub_mode;
                    std::cout << "[SITL] PX4 已因 setpoint 丢失退出 Offboard: mode="
                              << static_cast<int>(offboard_loss_reaction_mode) << '/'
                              << static_cast<int>(offboard_loss_reaction_sub_mode)
                              << std::endl;
                }
                if (options.sitl_offboard_loss && offboard_loss_mode_exited &&
                    latest.flight_mode != 6 && latest.connected &&
                    MonotonicMs() - offboard_loss_mode_exited_ms >= 1000) {
                    offboard_loss_stable = true;
                }
                if (sitl_arm_test && armed_reached && !latest.armed &&
                    !disarm_command_queued) {
                    unexpected_disarm_before_command = true;
                }
            }
            if (sitl_arm_test && offboard_ack_accepted &&
                offboard_mode_reached && !arm_command_queued &&
                latest.connected && !latest.armed &&
                latest.landed_state_valid && latest.landed &&
                latest.local_position_valid && latest.attitude_valid) {
                arm_command_queued = link.SendCommand(
                    MAV_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f, 0.f, 0.f,
                    0.f, 0.f, 0.f);
                if (arm_command_queued) {
                    arm_request_time_ms = MonotonicMs();
                }
                std::cout << "[SITL] ARM 请求"
                          << (arm_command_queued ? "已入队" : "入队失败")
                          << std::endl;
            }
            if (have_snapshot && now >= next_summary) {
                PrintSummary(latest, std::chrono::duration_cast<std::chrono::seconds>(now - started));
                next_summary = now + std::chrono::seconds(1);
            }
            if (options.sitl_arm_zero_velocity && armed_reached &&
                latest.armed && armed_since_ms != 0 &&
                MonotonicMs() - armed_since_ms >= 5000) {
                arm_hold_complete = true;
                std::cout << "[SITL] armed 零速度保持 5 秒完成，准备主动 DISARM"
                          << std::endl;
                break;
            }
            if (options.sitl_offboard_loss && intentional_setpoint_loss &&
                !offboard_loss_mode_exited &&
                MonotonicMs() - setpoint_loss_started_ms >= 10000 &&
                !flight_safety_violation) {
                flight_safety_violation = true;
                std::cout << "[SITL][安全回收] 停止 setpoint 10 秒后仍未退出 Offboard"
                          << std::endl;
            }
            if (options.sitl_horizontal_motion && hover_complete &&
                !horizontal_motion_started && !flight_safety_violation) {
                horizontal_motion_started = true;
                horizontal_motion_started_ms = MonotonicMs();
                std::cout << "[SITL] 北向 0.5m/s 受限运动开始，固定持续 2 秒"
                          << std::endl;
            }
            if (options.sitl_horizontal_motion && horizontal_motion_started &&
                !horizontal_motion_complete &&
                MonotonicMs() - horizontal_motion_started_ms >= 2000) {
                horizontal_motion_complete = true;
                horizontal_braking = true;
                if (latest.local_position_valid) {
                    motion_command_end_north_displacement_m =
                        latest.local_x_m - arm_start_x;
                }
                std::cout << "[SITL] 水平运动 2 秒完成，切换零速度制动"
                          << std::endl;
            }
            if (options.sitl_horizontal_motion && horizontal_brake_complete &&
                !returning_to_start) {
                if (latest.local_position_valid) {
                    brake_end_north_displacement_m =
                        latest.local_x_m - arm_start_x;
                }
                returning_to_start = true;
                std::cout << "[SITL] 制动完成，切换位置目标返回起飞点上方"
                          << std::endl;
            }
            if (options.sitl_offboard_loss && hover_complete &&
                !intentional_setpoint_loss && !flight_safety_violation) {
                drone::common::Px4Setpoint stop_setpoint;
                stop_setpoint.valid = false;
                (void)setpoint_topic.Emplace(stop_setpoint);
                intentional_setpoint_loss = true;
                setpoint_loss_started_ms = MonotonicMs();
                setpoint_count_at_loss = link.SetpointSendCount();
                std::cout << "[SITL] 已主动停止 Offboard setpoint，等待 PX4 保护动作"
                          << std::endl;
            }
            const bool flight_mission_complete =
                options.sitl_takeoff_land ? hover_complete :
                (options.sitl_horizontal_motion ? returned_to_start :
                 (options.sitl_offboard_loss && offboard_loss_stable));
            if (sitl_flight_test &&
                (flight_mission_complete || flight_safety_violation) &&
                !land_command_queued && latest.connected && latest.armed) {
                drone::common::Px4Setpoint stop_setpoint;
                stop_setpoint.valid = false;
                (void)setpoint_topic.Emplace(stop_setpoint);
                constexpr uint8_t kPx4MainModeAuto = 4;
                constexpr uint8_t kPx4SubModeAutoLand = 6;
                land_command_queued = link.SendCommand(
                    MAV_CMD_DO_SET_MODE,
                    static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                    static_cast<float>(kPx4MainModeAuto),
                    static_cast<float>(kPx4SubModeAutoLand),
                    0.f, 0.f, 0.f, 0.f);
                if (land_command_queued) {
                    land_request_time_ms = MonotonicMs();
                }
                std::cout << "[SITL] AUTO.LAND 模式请求"
                          << (land_command_queued ? "已入队" : "入队失败")
                          << std::endl;
            }
            if (sitl_flight_test && land_ack_accepted &&
                latest.landed_state_valid && latest.landed) {
                landed_after_flight = true;
                std::cout << "[SITL] PX4 已确认降落，准备主动 DISARM" << std::endl;
                break;
            }
        }

        if (sitl_flight_test && armed_reached && latest.armed &&
            !landed_after_flight) {
            if (!land_command_queued) {
                drone::common::Px4Setpoint stop_setpoint;
                stop_setpoint.valid = false;
                (void)setpoint_topic.Emplace(stop_setpoint);
                constexpr uint8_t kPx4MainModeAuto = 4;
                constexpr uint8_t kPx4SubModeAutoLand = 6;
                land_command_queued = link.SendCommand(
                    MAV_CMD_DO_SET_MODE,
                    static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                    static_cast<float>(kPx4MainModeAuto),
                    static_cast<float>(kPx4SubModeAutoLand),
                    0.f, 0.f, 0.f, 0.f);
                if (land_command_queued) {
                    land_request_time_ms = MonotonicMs();
                }
                std::cout << "[SITL] 异常回收 AUTO.LAND 请求"
                          << (land_command_queued ? "已入队" : "入队失败")
                          << std::endl;
            }
            const auto recovery_deadline =
                std::chrono::steady_clock::now() + std::chrono::seconds(15);
            while (std::chrono::steady_clock::now() < recovery_deadline) {
                if (auto message = subscription.WaitTakeFor(std::chrono::milliseconds(100))) {
                    latest = **message;
                    UpdateObserved(latest, &observed);
                    if (latest.last_ack_time_ms >= land_request_time_ms &&
                        latest.last_ack_command == MAV_CMD_DO_SET_MODE) {
                        land_ack_received = true;
                        land_ack_accepted =
                            latest.last_ack_result == MAV_RESULT_ACCEPTED;
                    }
                    if (latest.landed_state_valid && latest.landed) {
                        landed_after_flight = true;
                        break;
                    }
                }
            }
        }

        if (sitl_arm_test && armed_reached &&
            (!sitl_flight_test || landed_after_flight)) {
            while (subscription.TryTake()) {
                // 清除排队的旧快照，避免用解锁前 armed=false 误判上锁完成。
            }
            disarm_command_queued = link.SendCommand(
                MAV_CMD_COMPONENT_ARM_DISARM, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f);
            if (disarm_command_queued) {
                disarm_request_time_ms = MonotonicMs();
            }
            std::cout << "[SITL] DISARM 请求"
                      << (disarm_command_queued ? "已入队" : "入队失败")
                      << std::endl;
            const auto disarm_deadline =
                std::chrono::steady_clock::now() + std::chrono::seconds(5);
            while (std::chrono::steady_clock::now() < disarm_deadline) {
                if (options.sitl_arm_zero_velocity) {
                    drone::common::Px4Setpoint hold;
                    hold.header.receive_time_ms = MonotonicMs();
                    hold.header.valid_for_ms = 300;
                    hold.header.frame_id = 3;
                    hold.type = drone::common::SetpointType::kVelocity;
                    hold.valid = true;
                    (void)setpoint_topic.Emplace(hold);
                }
                if (auto message = subscription.WaitTakeFor(std::chrono::milliseconds(100))) {
                    latest = **message;
                    UpdateObserved(latest, &observed);
                    if (latest.last_ack_time_ms >= disarm_request_time_ms &&
                        latest.last_ack_command == MAV_CMD_COMPONENT_ARM_DISARM) {
                        disarm_ack_received = true;
                        disarm_ack_accepted =
                            latest.last_ack_result == MAV_RESULT_ACCEPTED;
                    }
                    if (disarm_ack_accepted && !latest.armed) {
                        disarmed_confirmed = true;
                        break;
                    }
                }
            }
        }

        if (sitl_setpoint_test) {
            drone::common::Px4Setpoint stop_setpoint;
            stop_setpoint.valid = false;
            (void)setpoint_topic.Emplace(stop_setpoint);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        const bool connected_at_end = link.IsConnected();
        link.Stop();

        std::cout << "\n========== PX4 链路验收报告 ==========" << std::endl;
        PrintCheck("通信传输已成功打开并运行", true, true);
        PrintCheck("PX4 HEARTBEAT", observed.heartbeat, true);
        PrintCheck("测试结束时心跳仍连接", connected_at_end, true);
        if (sitl_setpoint_test) {
            PrintCheck(sitl_flight_test
                           ? "SITL 飞行设定值已发送"
                           : "SITL 零速度设定值已发送",
                       link.SetpointSendCount() > 0, true);
            std::cout << "设定值发送数: " << link.SetpointSendCount() << std::endl;
        }
        if (sitl_offboard_test) {
            PrintCheck("Offboard 模式请求已入队", offboard_command_queued, true);
            PrintCheck("Offboard 模式 ACK 已收到", offboard_ack_received, true);
            PrintCheck("Offboard 模式 ACK accepted", offboard_ack_accepted, true);
            PrintCheck("PX4 已进入 Offboard", offboard_mode_reached, true);
        }
        if (options.sitl_offboard_disarmed) {
            PrintCheck("测试全程保持 disarmed", remained_disarmed, true);
        }
        if (sitl_arm_test) {
            PrintCheck("ARM 请求已入队", arm_command_queued, true);
            PrintCheck("ARM ACK 已收到", arm_ack_received, true);
            PrintCheck("ARM ACK accepted", arm_ack_accepted, true);
            PrintCheck("PX4 已确认 armed", armed_reached, true);
            if (options.sitl_arm_zero_velocity) {
                PrintCheck("armed 零速度保持 5 秒", arm_hold_complete, true);
            }
            if (sitl_flight_test) {
                PrintCheck("相对起飞目标已建立", takeoff_target_ready, true);
                PrintCheck("达到相对 1m 高度", takeoff_height_reached, true);
                PrintCheck("目标高度稳定悬停 3 秒", hover_complete, true);
                if (options.sitl_horizontal_motion) {
                    PrintCheck("北向 0.5m/s 运动已开始",
                               horizontal_motion_started, true);
                    PrintCheck("北向运动持续 2 秒",
                               horizontal_motion_complete, true);
                    PrintCheck("零速度制动完成",
                               horizontal_brake_complete, true);
                    PrintCheck("已返回起飞点上方",
                               returned_to_start, true);
                    std::cout << "速度命令结束北向位移: "
                              << motion_command_end_north_displacement_m
                              << " m，制动完成北向位移: "
                              << brake_end_north_displacement_m
                              << " m，最大北向位移: "
                              << max_north_displacement_m
                              << " m，最大东西向偏差: "
                              << max_east_deviation_m << " m" << std::endl;
                }
                if (options.sitl_offboard_loss) {
                    const uint64_t sends_after_loss =
                        link.SetpointSendCount() >= setpoint_count_at_loss
                            ? link.SetpointSendCount() - setpoint_count_at_loss
                            : 0;
                    PrintCheck("已主动停止 Offboard setpoint",
                               intentional_setpoint_loss, true);
                    PrintCheck("PX4 已退出 Offboard",
                               offboard_loss_mode_exited, true);
                    PrintCheck("退出 Offboard 后模式稳定 1 秒",
                               offboard_loss_stable, true);
                    PrintCheck("停止后设定值发送已停止",
                               sends_after_loss <= 2, true);
                    std::cout << "Offboard 丢失反应时间: "
                              << (offboard_loss_mode_exited
                                      ? offboard_loss_mode_exited_ms -
                                            setpoint_loss_started_ms
                                      : 0)
                              << " ms，保护模式: "
                              << static_cast<int>(offboard_loss_reaction_mode) << '/'
                              << static_cast<int>(offboard_loss_reaction_sub_mode)
                              << "，停止请求后额外发送: " << sends_after_loss
                              << " 条" << std::endl;
                }
                PrintCheck("AUTO.LAND 请求已入队", land_command_queued, true);
                PrintCheck("AUTO.LAND ACK 已收到", land_ack_received, true);
                PrintCheck("AUTO.LAND ACK accepted", land_ack_accepted, true);
                PrintCheck("PX4 已确认 landed", landed_after_flight, true);
                PrintCheck("飞行未触发高度/漂移安全回收",
                           !flight_safety_violation, true);
                std::cout << "最大相对高度: " << max_takeoff_height_m
                          << " m，最大水平漂移: " << max_horizontal_drift_m
                          << " m" << std::endl;
            }
            PrintCheck("主动 DISARM 前未被自动上锁",
                       !unexpected_disarm_before_command, true);
            PrintCheck("DISARM 请求已入队", disarm_command_queued, true);
            PrintCheck("DISARM ACK 已收到", disarm_ack_received, true);
            PrintCheck("DISARM ACK accepted", disarm_ack_accepted, true);
            PrintCheck("PX4 已确认 disarmed", disarmed_confirmed, true);
            std::cout << "解锁期间最大位移: " << max_displacement_m << " m" << std::endl;
        }
        PrintCheck("遥测请求 COMMAND_ACK", link.AckMatchCount() > 0, false);
        PrintCheck("遥测请求无 ACK 超时", link.AckTimeoutCount() == 0, false);
        PrintMessageCheck("AUTOPILOT_VERSION",
                          link.MessageReceiveCount(MAVLINK_MSG_ID_AUTOPILOT_VERSION),
                          observed.version);
        PrintMessageCheck("EXTENDED_SYS_STATE",
                          link.MessageReceiveCount(MAVLINK_MSG_ID_EXTENDED_SYS_STATE),
                          observed.landed);
        PrintMessageCheck("GLOBAL_POSITION_INT",
                          link.MessageReceiveCount(MAVLINK_MSG_ID_GLOBAL_POSITION_INT),
                          observed.global_position);
        PrintMessageCheck("LOCAL_POSITION_NED",
                          link.MessageReceiveCount(MAVLINK_MSG_ID_LOCAL_POSITION_NED),
                          observed.local_position);
        PrintMessageCheck("ATTITUDE",
                          link.MessageReceiveCount(MAVLINK_MSG_ID_ATTITUDE),
                          observed.attitude);
        PrintMessageCheck("GPS_RAW_INT",
                          link.MessageReceiveCount(MAVLINK_MSG_ID_GPS_RAW_INT),
                          observed.gps);
        if (observed.gps && !observed.gps_3d_fix) {
            std::cout << "[WARN] GPS_RAW_INT 已收到，但测试期间没有 3D fix" << std::endl;
        }
        const uint64_t battery_message_count =
            link.MessageReceiveCount(MAVLINK_MSG_ID_SYS_STATUS) +
            link.MessageReceiveCount(MAVLINK_MSG_ID_BATTERY_STATUS);
        PrintMessageCheck("SYS_STATUS/BATTERY_STATUS", battery_message_count,
                          observed.battery);
        PrintMessageCheck("HOME_POSITION",
                          link.MessageReceiveCount(MAVLINK_MSG_ID_HOME_POSITION),
                          observed.home);
        PrintMessageCheck("RC_CHANNELS",
                          link.MessageReceiveCount(MAVLINK_MSG_ID_RC_CHANNELS),
                          observed.rc);
        std::cout << "接收目标消息数: " << link.ReceiveCount() << '\n'
                  << "ACK 匹配数: " << link.AckMatchCount() << '\n'
                  << "ACK 超时数: " << link.AckTimeoutCount() << '\n'
                  << "最近 ACK: command=" << latest.last_ack_command
                  << " result=" << static_cast<int>(latest.last_ack_result) << '\n'
                  << "链路错误数: " << link.ErrorCount() << '\n'
                  << "说明: WARN 通常表示 PX4 未配置该消息流，后续可主动请求消息频率。\n"
                  << std::endl;

        const bool setpoint_passed =
            !sitl_setpoint_test || link.SetpointSendCount() > 0;
        const bool offboard_passed = !sitl_offboard_test ||
                                     (offboard_command_queued && offboard_ack_received &&
                                      offboard_ack_accepted && offboard_mode_reached);
        const bool disarmed_stage_passed = !options.sitl_offboard_disarmed ||
                                           remained_disarmed;
        const bool common_arm_passed =
            arm_command_queued && arm_ack_received && arm_ack_accepted &&
            armed_reached && !unexpected_disarm_before_command &&
            disarm_command_queued && disarm_ack_received &&
            disarm_ack_accepted && disarmed_confirmed;
        const bool arm_stage_passed = !options.sitl_arm_zero_velocity ||
                                      (common_arm_passed && arm_hold_complete);
        const bool takeoff_stage_passed = !options.sitl_takeoff_land ||
            (common_arm_passed && takeoff_target_ready && takeoff_height_reached &&
             hover_complete && land_command_queued && land_ack_received &&
             land_ack_accepted && landed_after_flight &&
             !flight_safety_violation && max_takeoff_height_m <= 1.30f &&
             max_horizontal_drift_m <= 0.50f);
        const bool horizontal_stage_passed = !options.sitl_horizontal_motion ||
            (common_arm_passed && takeoff_target_ready && takeoff_height_reached &&
             hover_complete && horizontal_motion_started &&
             horizontal_motion_complete && horizontal_brake_complete &&
             returned_to_start && land_command_queued && land_ack_received &&
             land_ack_accepted && landed_after_flight &&
             !flight_safety_violation && brake_end_north_displacement_m >= 0.70f &&
             brake_end_north_displacement_m <= 1.30f &&
             max_north_displacement_m <= 1.50f &&
             max_east_deviation_m <= 0.30f && max_takeoff_height_m <= 1.30f);
        const uint64_t sends_after_loss =
            link.SetpointSendCount() >= setpoint_count_at_loss
                ? link.SetpointSendCount() - setpoint_count_at_loss
                : 0;
        const bool offboard_loss_stage_passed = !options.sitl_offboard_loss ||
            (common_arm_passed && takeoff_target_ready && takeoff_height_reached &&
             hover_complete && intentional_setpoint_loss &&
             offboard_loss_mode_exited && offboard_loss_stable &&
             offboard_loss_mode_exited_ms - setpoint_loss_started_ms <= 10000 &&
             sends_after_loss <= 2 && land_command_queued && land_ack_received &&
             land_ack_accepted && landed_after_flight &&
             !flight_safety_violation);
        const bool passed = observed.heartbeat && connected_at_end &&
                            link.ErrorCount() == 0 && setpoint_passed && offboard_passed &&
                            disarmed_stage_passed && arm_stage_passed &&
                            takeoff_stage_passed && horizontal_stage_passed &&
                            offboard_loss_stage_passed;
        std::cout << (passed ? "结果: 基础 PX4 链路通过"
                            : "结果: 基础 PX4 链路未通过")
                  << std::endl;
        return passed ? 0 : 3;
    } catch (const std::exception& error) {
        std::cerr << "PX4 冒烟测试启动失败: " << error.what() << std::endl;
        return 1;
    }
}
