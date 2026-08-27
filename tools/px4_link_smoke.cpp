#include <atomic>
#include <chrono>
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
};

void PrintUsage(const char* program) {
    std::cout << "用法: " << program
              << " [--config <config.json>] [--duration <秒>]\n"
              << "该程序只测试 PX4 串口和遥测，不发送解锁、模式或控制指令。\n";
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

    Px4LinkConfig config;
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

        const json log_config = root.value("log", json::object());
        const auto log_level = spdlog::level::from_str(
            log_config.value("level", std::string("info")));
        drone::common::InitializeAsyncLogger(ResolveLogDirectory(root), log_level);

        std::signal(SIGINT, OnSignal);
        std::signal(SIGTERM, OnSignal);

        std::cout << "PX4 只读串口冒烟测试\n"
                  << "  config: " << config_path << '\n'
                  << "  device: " << config.serial.device << '\n'
                  << "  baud: " << config.serial.baud_rate << '\n'
                  << "  onboard: " << static_cast<int>(config.onboard_system_id) << '/'
                  << static_cast<int>(config.onboard_component_id) << '\n'
                  << "  target: " << static_cast<int>(config.target_system_id) << '/'
                  << static_cast<int>(config.target_component_id) << '\n'
                  << "  firmware: " << config.firmware_version << '\n'
                  << "  duration: " << options.duration.count() << " 秒\n"
                  << "  安全边界: 仅发送 HEARTBEAT 和遥测请求命令，不发送飞行控制指令\n"
                  << std::endl;

        drone::communication::Px4Link link(config);
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
            if (have_snapshot && now >= next_summary) {
                PrintSummary(latest, std::chrono::duration_cast<std::chrono::seconds>(now - started));
                next_summary = now + std::chrono::seconds(1);
            }
        }

        const bool connected_at_end = link.IsConnected();
        link.Stop();

        std::cout << "\n========== PX4 串口验收报告 ==========" << std::endl;
        PrintCheck("串口已成功打开并运行", true, true);
        PrintCheck("PX4 HEARTBEAT", observed.heartbeat, true);
        PrintCheck("测试结束时心跳仍连接", connected_at_end, true);
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

        const bool passed = observed.heartbeat && connected_at_end && link.ErrorCount() == 0;
        std::cout << (passed ? "结果: 基础 PX4 串口链路通过"
                            : "结果: 基础 PX4 串口链路未通过")
                  << std::endl;
        return passed ? 0 : 3;
    } catch (const std::exception& error) {
        std::cerr << "PX4 冒烟测试启动失败: " << error.what() << std::endl;
        return 1;
    }
}
