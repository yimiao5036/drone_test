#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "common/logger.h"
#include "communication/mavlink_handler.h"
#include "communication/px4_link.h"
#include "config/config.h"
#include "control/flight_controller.h"
#include "perception/geodetic_converter.h"
#include "perception/target_estimator.h"
#include "state_machine/mission_state_machine.h"

namespace {

using json = nlohmann::json;
using drone::communication::Px4LinkConfig;
namespace common = drone::common;  // stage4-mission（G3 全链）段使用

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
    bool sitl_stage4_controller = false;
    bool sitl_stage4_mission = false;
    bool sitl_stage4_tune = false;
    std::string tune_csv_path;                       // 空 = logs/stage4_tune_<时间戳>.csv
    std::string tune_script = "full";                // static|heading|distance|vertical|full
    uint64_t tune_static_dwell_s = 25;               // 初始捕获段驻留
    uint64_t tune_dwell_s = 15;                      // 每个阶跃段驻留
    double tune_approach_until_m = 20.0;             // S5 接近到该射程内再注入视觉
    bool selftest = false;
};

void PrintUsage(const char* program) {
    std::cout << "用法: " << program
              << " [--config <config.json>] [--duration <秒>]\n"
              << "       [--sitl-zero-velocity | --sitl-offboard-disarmed | --sitl-arm-zero-velocity\n"
              << "        | --sitl-takeoff-land | --sitl-horizontal-motion\n"
              << "        | --sitl-offboard-loss | --sitl-stage4-controller\n"
              << "        | --sitl-stage4-mission]\n"
              << "SITL 参数仅允许 UDP；offboard-loss 起飞 1m 后主动停止 setpoint，\n"
              << "验证 PX4 退出 Offboard，再统一 AUTO.LAND 回收。\n"
              << "stage4-controller 装配真实 FlightController（意图脚本 → 限幅/门禁 → setpoint），\n"
              << "依次验收：零速漂移、速度阶跃斜率/稳态、停发刹车、Offboard 退出时延。\n"
              << "stage4-mission 装配状态机全链（G3）：注入地面站目标 + 模拟视觉观测走通\n"
              << "S5→S6→S7→回退→停发链路，另含 G2 补验（围栏钳制、切非 Offboard 停发 <200ms）。\n"
              << "stage4-tune 控制律整定场景：S7 长驻留 + 分通道阶跃换位，逐拍落 CSV；\n"
              << "  [--tune-script static|heading|distance|vertical|full] [--tune-csv <路径>]\n"
              << "  [--tune-static-dwell-s <秒>] [--tune-dwell-s <秒>] [--tune-approach-until-m <米>]\n"
              << "--selftest 离线自检工具纯逻辑（G3 意图分类等），不需要 SITL/配置。\n";
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
        if (argument == "--sitl-stage4-controller") {
            options.sitl_stage4_controller = true;
            continue;
        }
        if (argument == "--sitl-stage4-mission") {
            options.sitl_stage4_mission = true;
            continue;
        }
        if (argument == "--sitl-stage4-tune") {
            options.sitl_stage4_tune = true;
            continue;
        }
        if (argument == "--tune-csv") {
            if (++index >= argc) {
                throw std::invalid_argument("--tune-csv 缺少路径");
            }
            options.tune_csv_path = argv[index];
            continue;
        }
        if (argument == "--tune-script") {
            if (++index >= argc) {
                throw std::invalid_argument("--tune-script 缺少剧本名");
            }
            options.tune_script = argv[index];
            continue;
        }
        if (argument == "--tune-static-dwell-s") {
            if (++index >= argc) {
                throw std::invalid_argument("--tune-static-dwell-s 缺少秒数");
            }
            options.tune_static_dwell_s = std::stoull(argv[index]);
            continue;
        }
        if (argument == "--tune-dwell-s") {
            if (++index >= argc) {
                throw std::invalid_argument("--tune-dwell-s 缺少秒数");
            }
            options.tune_dwell_s = std::stoull(argv[index]);
            continue;
        }
        if (argument == "--tune-approach-until-m") {
            if (++index >= argc) {
                throw std::invalid_argument("--tune-approach-until-m 缺少米数");
            }
            options.tune_approach_until_m = std::stod(argv[index]);
            continue;
        }
        if (argument == "--selftest") {
            options.selftest = true;
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
                                 static_cast<int>(options.sitl_offboard_loss) +
                                 static_cast<int>(options.sitl_stage4_controller) +
                                 static_cast<int>(options.sitl_stage4_mission) +
                                 static_cast<int>(options.sitl_stage4_tune);
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
    if (options.sitl_stage4_controller && options.duration < std::chrono::seconds(60)) {
        throw std::invalid_argument("控制器执行层门禁测试时长不能少于 60 秒");
    }
    if (options.sitl_stage4_mission && options.duration < std::chrono::seconds(60)) {
        throw std::invalid_argument("全链（状态机）门禁测试时长不能少于 60 秒");
    }
    if (options.sitl_stage4_tune) {
        if (options.tune_script != "static" && options.tune_script != "heading" &&
            options.tune_script != "distance" && options.tune_script != "vertical" &&
            options.tune_script != "full") {
            throw std::invalid_argument(
                "--tune-script 只支持 static|heading|distance|vertical|full");
        }
        if (options.duration < std::chrono::seconds(120)) {
            throw std::invalid_argument("整定场景时长不能少于 120 秒");
        }
        if (options.tune_static_dwell_s < 15 || options.tune_dwell_s < 10) {
            throw std::invalid_argument("整定驻留时长过短（static ≥15s、阶跃段 ≥10s）");
        }
        if (options.tune_approach_until_m < 12.0 ||
            options.tune_approach_until_m >= 40.0) {
            throw std::invalid_argument("--tune-approach-until-m 须在 12~40 米之间");
        }
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
    // 机载身份自 2026-09-02（多机区分配置改造）起与正式主程序一致：
    // 读取 px4 节的 onboard_system_id/onboard_component_id，不再使用
    // 历史顶层 mavlink 节（现网 config/*.json 已无此节）。
    const json& px4 = root.at("px4");
    const json& serial = px4.at("serial");
    const json& udp = px4.at("udp");

    Px4LinkConfig config;
    config.transport = px4.at("transport").get<std::string>();
    config.firmware_version = px4.at("firmware_version").get<std::string>();
    config.onboard_system_id = ReadUint8(px4, "onboard_system_id");
    config.onboard_component_id = ReadUint8(px4, "onboard_component_id");
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

std::chrono::milliseconds ReadOptionalPositiveMilliseconds(
    const json& object, const char* key, std::chrono::milliseconds fallback) {
    if (!object.contains(key)) {
        return fallback;
    }
    return ReadPositiveMilliseconds(object, key);
}

double ReadOptionalPositiveDouble(const json& object, const char* key,
                                  double fallback) {
    if (!object.contains(key)) {
        return fallback;
    }
    const double value = object.at(key).get<double>();
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(key) + " 必须为有限正数");
    }
    return value;
}

/// 读取 controller 配置节（字段名与默认值同源 `config/config.h`）。
/// stage4（G2）保持轻量自读：只做断言依赖的跨字段校验，其余由 FlightController
/// 自身保证；stage4-mission（G3）需要 controller/handover/visual_tracking 等整节，
/// 改由 drone_config 的 LoadAppConfig 同源加载（见 RunStage4Mission）。
drone::config::FlightControllerConfig LoadControllerConfig(const json& root) {
    drone::config::FlightControllerConfig config;
    const json controller = root.value("controller", json::object());
    config.period =
        ReadOptionalPositiveMilliseconds(controller, "period_ms", config.period);
    config.max_speed_mps = ReadOptionalPositiveDouble(controller, "max_speed_mps",
                                                      config.max_speed_mps);
    config.max_accel_mps2 = ReadOptionalPositiveDouble(
        controller, "max_accel_mps2", config.max_accel_mps2);
    config.max_altitude_m = ReadOptionalPositiveDouble(
        controller, "max_altitude_m", config.max_altitude_m);
    config.max_radius_m =
        ReadOptionalPositiveDouble(controller, "max_radius_m", config.max_radius_m);
    config.intent_timeout = ReadOptionalPositiveMilliseconds(
        controller, "intent_timeout_ms", config.intent_timeout);
    config.coast_timeout = ReadOptionalPositiveMilliseconds(
        controller, "coast_timeout_ms", config.coast_timeout);
    if (config.coast_timeout <= config.intent_timeout) {
        throw std::invalid_argument(
            "controller.coast_timeout_ms 必须大于 intent_timeout_ms（与配置校验同源）");
    }
    return config;
}

std::string ResolveLogDirectory(const json& root) {
    const json log = root.value("log", json::object());
    return log.value("dir", std::string("logs/"));
}

/// 读取双目校正矩阵的深度系数 fxb_m_px（与 config.cpp 同源文件、同字段；
/// stage4-mission 注入模拟测距需要 fxb 通过 TargetEstimatorConfig::Validate）。
/// 路径按"与 config.json 同目录"解析（与 config.cpp 的 ResolveAdjacentPath 同规则）。
double LoadRectifyFxb(const std::string& config_path, const std::string& relative_path) {
    std::string path = relative_path;
    const bool absolute = !path.empty() && (path.front() == '/' || path.front() == '\\');
    if (!absolute) {
        std::string directory = config_path;
        const auto separator = directory.find_last_of("/\\");
        directory = separator == std::string::npos ? std::string(".")
                                                   : directory.substr(0, separator);
        path = directory + "/" + relative_path;
    }
    const json root = LoadJson(path);
    const double fxb = root.at("fxb_m_px").get<double>();
    if (!std::isfinite(fxb) || fxb <= 0.0) {
        throw std::invalid_argument("校正矩阵 fxb_m_px 必须为有限正数: " + path);
    }
    return fxb;
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

// ---------------------------------------------------------------------------
// stage4（G2 执行层门禁）：FlightController 真实接入 + 脚本化意图序列
//
// 工具扮演“状态机 + 地面站”角色：向 FlightController 发布脚本化 ControlIntent
// （带 reason_code bit31 控制权请求位），控制器经门禁链/限幅流水线生成
// Px4Setpoint，Px4Link 发送给 PX4。验收只看 PX4 遥测与控制器诊断计数器，
// 不直接检查控制器内部状态（除公开诊断接口）。
// ---------------------------------------------------------------------------

/// stage4 固定边界（与 stage3 流程对齐；不可经命令行放宽）。
constexpr float kStage4TakeoffHeightM = 1.0f;          // 相对起飞高度
constexpr float kStage4HoverZErrorM = 0.15f;           // 悬停窗口：高度误差
constexpr float kStage4HoverDriftM = 0.30f;            // 悬停窗口：水平漂移
constexpr float kStage4HoverVzMps = 0.30f;             // 悬停窗口：垂速
constexpr float kStage4MaxHeightM = 1.30f;             // armed 期间硬性高度上限
constexpr float kStage4MinHeightInMotionM = 0.50f;     // 运动段高度下限
constexpr float kStage4MaxDriftBeforeStepM = 0.50f;    // 阶跃前水平硬边界
constexpr float kStage4MaxNorthM = 6.0f;               // 阶跃段北向包线
constexpr float kStage4MaxEastM = 1.0f;                // 阶跃段东西包线
constexpr float kStage4StepSpeedMps = 2.0f;            // 阶跃意图速度
constexpr float kStage4ZeroDriftLimitM = 0.10f;        // 零速保持最大漂移上限
constexpr float kStage4ZeroSpeedMps = 0.10f;           // “速度过零”判据
constexpr float kStage4SteadyToleranceMps = 0.25f;     // 稳态速度容差
constexpr double kStage4SlopeTolerance = 1.2;          // 设定值爬升斜率容差倍数
constexpr uint64_t kStage4SlopeWindowMs = 300;         // 爬升斜率时间窗（按时间，不按样本数）
// 刹车过零上限：PX4 速度环对零速指令呈指数收敛（SITL 实测 τ≈0.45s），从
// max_speed=2m/s 收敛到 ≤0.1m/s 物理下限约 1.4s，加遥测量化余量取 2000ms；
// 早前用 coast_timeout(1000ms) 作上限物理不可达（2026-10-14 ulog 复核）。
constexpr uint64_t kStage4BrakeZeroLimitMs = 2000;     // COAST→速度过零上限
constexpr uint64_t kStage4IntentValidMs = 500;         // 意图有效期
constexpr uint64_t kStage4IntentPeriodMs = 100;        // 意图发布周期（10Hz）
constexpr uint64_t kStage4GuidePeriodMs = 100;         // 引导流注入周期（10Hz）
constexpr uint64_t kStage4GuideMinSends = 20;          // 引导流最小条数（stage3A 纪律）
constexpr uint64_t kStage4GuideMinMs = 2000;           // 引导流最短时长（stage3A 纪律）
constexpr uint64_t kStage4ZeroHoldMs = 3000;           // 零速意图保持时长
constexpr uint64_t kStage4StepHoldMs = 2500;           // 阶跃意图保持时长
constexpr uint64_t kStage4ConnectTimeoutMs = 10000;    // 未连接快速失败（负对照）
constexpr uint64_t kStage4BootstrapTimeoutMs = 20000;  // 引导/Offboard/ARM/接管上限
constexpr uint64_t kStage4TakeoffTimeoutMs = 15000;    // 起飞/悬停上限
constexpr uint64_t kStage4StopTimeoutMs = 15000;       // 停发→Offboard 退出上限
constexpr uint64_t kStage4LandTimeoutMs = 30000;       // AUTO.LAND→landed 上限
// IDLE→PX4 退出 Offboard 上限：COM_OF_LOSS_T(1.0s) + PX4 HEARTBEAT 1Hz 相位
// 抖动（0~1s，均匀随机）+ 200ms 余量 = 2200ms；八月基线 1270ms 只是心跳相位
// 走运，1300ms 上限按此测法必然随机失败（2026-10-14 复核当日 3D 段同环境
// 实测 1999ms 为同一量化效应）。
constexpr uint64_t kStage4OffboardExitLimitMs = 2200;  // IDLE→PX4 退出 Offboard 上限
constexpr uint64_t kStage4ExtraSendsAllowed = 2;       // 停发后容许的竞态额外发送

/// stage4 执行阶段。
enum class Stage4Phase : uint8_t {
    kBootstrap,    ///< 连接 → 引导零速度流 → Offboard → ARM → 控制器接管
    kTakeoff,      ///< 位置引导意图：相对起飞 1m 并稳定悬停 3 秒
    kZeroHold,     ///< 零速度意图保持 3 秒（最大漂移验收）
    kStepNorth,    ///< 北向 2m/s 阶跃（爬升斜率与稳态速度验收）
    kStopIntents,  ///< 停止发意图：COAST 刹车 → IDLE 停发 → PX4 退出 Offboard
    kLand,         ///< AUTO.LAND → landed
    kDone,
};

/// stage4 脚本每拍发布的意图内容。
enum class Stage4IntentMode : uint8_t {
    kNone = 0,          ///< 停发（超龄注入）
    kZeroVelocity,      ///< 零速度 + 请求位
    kNorthStep,         ///< 北向速度阶跃
    kPositionTakeoff,   ///< 位置引导：相对起飞 1m
};

/// 阶跃段速度样本（遥测 vx 或控制器发布的设定值 vx；2.5 秒 × 100ms，量级极小）。
struct Stage4SpeedSample {
    uint64_t time_ms = 0;
    float vx_mps = 0.f;
};

/// stage4 脚本运行体。
///
/// 进入 Offboard 的次序说明（引导流）：PX4 要求进入 Offboard 前已存在
/// setpoint 流（stage3A 纪律），而控制器授权链要求 armed + Offboard(6/0)
/// 才发布——先有鸡还是先有蛋。工具因此在校验通过后、接管前，直接向控制器
/// 输出主题注入零速度引导流（控制器 IDLE 期间不发布，无冲突）；控制器
/// 授权链全通并接管后立即停止注入，由控制器供流（停注后链路发送计数仍
/// 增长即证明接管成功）。
class Stage4Run {
public:
    Stage4Run(drone::communication::Px4Link& link,
              drone::control::FlightController& controller,
              drone::common::Topic<drone::common::ControlIntent>& intent_topic,
              const drone::config::FlightControllerConfig& controller_config,
              std::chrono::seconds duration,
              drone::common::Topic<drone::common::FlightStateSnapshot>::Subscription
                  subscription)
        : link_(link),
          controller_(controller),
          intent_topic_(intent_topic),
          controller_config_(controller_config),
          duration_ms_(static_cast<uint64_t>(duration.count()) * 1000),
          subscription_(std::move(subscription)) {
        // 订阅控制器设定值输出：阶跃斜率以发布侧（含控制器打戳）为准，
        // 不受 MAVLink 遥测成簇/抖动影响（2026-10-14 ulog 复核：遥测差分
        // 测出 4.63 m/s² 伪值，实际设定值斜率 2.12、实机加速度 1.77）。
        setpoint_subscription_ = controller_.SetpointOutput().Subscribe(64);
    }

    /// 执行脚本主循环（返回到期/中断/完成后仍可能 armed，收尾见 FinishFlight）。
    void Execute() {
        const uint64_t started_ms = MonotonicMs();
        uint64_t next_summary_ms = started_ms;
        while (!g_stop.load(std::memory_order_acquire) &&
               MonotonicMs() - started_ms < duration_ms_) {
            if (auto message = subscription_.WaitTakeFor(std::chrono::milliseconds(100))) {
                latest_ = **message;
                have_snapshot_ = true;
                UpdateObserved(latest_, &observed_);
                PrintStateChange();
                // 起飞起点锁定：首次 armed 且位置有效时（与 stage3B 同语义，
                // 不使用固定原点）。
                if (latest_.armed && !takeoff_target_ready_ &&
                    latest_.local_position_valid) {
                    takeoff_target_ready_ = true;
                    start_x_ = latest_.local_x_m;
                    start_y_ = latest_.local_y_m;
                    start_z_ = latest_.local_z_m;
                    takeoff_target_x_ = start_x_;
                    takeoff_target_y_ = start_y_;
                    takeoff_target_z_ = start_z_ - kStage4TakeoffHeightM;
                    std::cout << "[SITL] 起飞目标已锁定: N=" << takeoff_target_x_
                              << " E=" << takeoff_target_y_ << " D=" << takeoff_target_z_
                              << std::endl;
                }
            }
            // 排空控制器设定值输出：阶跃段斜率以发布侧样本为准（时间为
            // 控制器发布打戳 receive_time_ms，免疫本工具排空批处理）。
            while (auto setpoint = setpoint_subscription_.TryTake()) {
                if ((*setpoint)->valid &&
                    (*setpoint)->type == drone::common::SetpointType::kVelocity) {
                    setpoint_samples_.push_back(
                        {(*setpoint)->header.receive_time_ms, (*setpoint)->x});
                }
            }
            const uint64_t now_ms = MonotonicMs();
            PublishScriptIntent(now_ms);
            AdvancePhase(now_ms, now_ms - started_ms);
            if (have_snapshot_) {
                CheckFlightEnvelope();
            }
            if (now_ms >= next_summary_ms) {
                PrintSummary(latest_, std::chrono::duration_cast<std::chrono::seconds>(
                                           std::chrono::milliseconds(now_ms - started_ms)));
                next_summary_ms = now_ms + 1000;
            }
            if (aborted_ || safety_violation_ || phase_ == Stage4Phase::kDone) {
                break;
            }
        }
        FinishFlight();
    }

    /// 打印 stage4 验收报告并返回总判定。
    bool Evaluate(bool connected_at_end) const {
        const double slope_limit = controller_config_.max_accel_mps2 * kStage4SlopeTolerance;
        const double expected_steady =
            std::min<double>(kStage4StepSpeedMps, controller_config_.max_speed_mps);
        const bool brake_ok = coast_seen_ && brake_zero_seen_ &&
                              brake_zero_ms_ - coast_ms_ <= kStage4BrakeZeroLimitMs;
        const bool exit_ok = idle_seen_ && offboard_exit_seen_ &&
                             offboard_exit_ms_ - idle_ms_ <= kStage4OffboardExitLimitMs;
        const uint64_t sends_after_idle =
            idle_seen_ && link_.SetpointSendCount() >= sends_at_idle_
                ? link_.SetpointSendCount() - sends_at_idle_
                : 0;
        const bool takeoff_ok = takeoff_target_ready_ && takeoff_height_reached_ &&
                                hover_complete_ && takeoff_rejected_delta_ == 0;
        const bool zero_hold_ok = zero_hold_complete_ && zero_hold_drift_ok_;
        const bool step_ok = step_complete_ && step_slope_ok_ && step_steady_ok_;
        const bool stop_ok = coast_seen_ && brake_ok && idle_seen_ && exit_ok &&
                             sends_after_idle <= kStage4ExtraSendsAllowed;
        const bool land_ok = land_command_queued_ && land_ack_received_ &&
                             land_ack_accepted_ && landed_after_flight_;
        const bool disarm_ok = !unexpected_disarm_before_command_ &&
                               disarm_command_queued_ && disarm_ack_received_ &&
                               disarm_ack_accepted_ && disarmed_confirmed_;
        const bool handover_ok = handover_done_ && takeover_stream_ok_;

        std::cout << "\n========== stage4 执行层门禁报告（FlightController → Px4Link → PX4）=========="
                  << std::endl;
        PrintCheck("PX4 HEARTBEAT", observed_.heartbeat, true);
        PrintCheck("测试结束时心跳仍连接", connected_at_end, true);
        PrintCheck("引导零速度流已注入(≥20 条)", guide_injected_ >= kStage4GuideMinSends, true);
        PrintCheck("Offboard 模式请求已入队", offboard_command_queued_, true);
        PrintCheck("Offboard 模式 ACK 已收到", offboard_ack_received_, true);
        PrintCheck("Offboard 模式 ACK accepted", offboard_ack_accepted_, true);
        PrintCheck("PX4 已进入 Offboard(6/0)", offboard_mode_reached_, true);
        PrintCheck("ARM 请求已入队", arm_command_queued_, true);
        PrintCheck("ARM ACK 已收到", arm_ack_received_, true);
        PrintCheck("ARM ACK accepted", arm_ack_accepted_, true);
        PrintCheck("PX4 已确认 armed", armed_reached_, true);
        PrintCheck("控制器已接管设定值流(授权链全通)", handover_ok, true);
        PrintCheck("相对起飞目标已建立", takeoff_target_ready_, true);
        PrintCheck("达到相对 1m 高度", takeoff_height_reached_, true);
        PrintCheck("目标高度稳定悬停 3 秒", hover_complete_, true);
        PrintCheck("位置引导意图未被控制拒绝(含 Home 基准)",
                   takeoff_target_ready_ && takeoff_rejected_delta_ == 0, true);
        PrintCheck("零速意图 3 秒最大漂移 < 0.10m", zero_hold_ok, true);
        PrintCheck("阶跃设定值爬升斜率受限(≤max_accel×1.2，发布侧)",
                   step_complete_ && step_slope_ok_, true);
        PrintCheck("阶跃稳态速度=min(2, max_speed)", step_complete_ && step_steady_ok_, true);
        PrintCheck("停发后 COAST 零速刹车已触发", coast_seen_, true);
        PrintCheck("速度过零时间 ≤ 2000ms(指数收敛物理下限)", brake_ok, true);
        PrintCheck("控制器已转 IDLE 停发", idle_seen_, true);
        PrintCheck("PX4 已在 2.2s 内退出 Offboard(含 1Hz 心跳相位)", exit_ok, true);
        PrintCheck("停发后设定值发送已停止(≤2 条)",
                   idle_seen_ && sends_after_idle <= kStage4ExtraSendsAllowed, true);
        PrintCheck("AUTO.LAND 请求/ACK accepted/PX4 已确认 landed", land_ok, true);
        PrintCheck("主动 DISARM 前未被自动上锁",
                   armed_reached_ && !unexpected_disarm_before_command_, true);
        PrintCheck("DISARM ACK accepted 且已确认 disarmed", disarm_ok, true);
        PrintCheck("飞行未触发高度/漂移安全回收",
                   armed_reached_ && !safety_violation_, true);
        PrintCheck("控制器无异常计数(未支持意图/未知来源)",
                   armed_reached_ && controller_.ErrorCount() == 0, true);
        PrintMessageCheck("LOCAL_POSITION_NED",
                          link_.MessageReceiveCount(MAVLINK_MSG_ID_LOCAL_POSITION_NED),
                          observed_.local_position);
        PrintMessageCheck("ATTITUDE", link_.MessageReceiveCount(MAVLINK_MSG_ID_ATTITUDE),
                          observed_.attitude);
        PrintMessageCheck("EXTENDED_SYS_STATE",
                          link_.MessageReceiveCount(MAVLINK_MSG_ID_EXTENDED_SYS_STATE),
                          observed_.landed);
        std::cout << "实测值: 零速最大漂移 " << zero_hold_max_drift_m_
                  << " m（上限 " << kStage4ZeroDriftLimitM << "），阶跃设定值斜率(发布侧) "
                  << step_max_slope_mps2_ << " m/s²（上限 " << slope_limit
                  << "），阶跃稳态速度 " << step_steady_speed_mps_ << " m/s（期望 "
                  << expected_steady << "），阶跃段位移 北向 " << step_north_m_
                  << " m / 东西 " << step_east_m_ << " m" << std::endl;
        std::cout << "实测时延: COAST→速度过零 "
                  << (coast_seen_ && brake_zero_seen_ ? brake_zero_ms_ - coast_ms_ : 0)
                  << " ms（上限 " << kStage4BrakeZeroLimitMs << "），COAST→IDLE "
                  << (coast_seen_ && idle_seen_ ? idle_ms_ - coast_ms_ : 0)
                  << " ms，IDLE→PX4 退出 Offboard "
                  << (idle_seen_ && offboard_exit_seen_ ? offboard_exit_ms_ - idle_ms_ : 0)
                  << " ms（上限 " << kStage4OffboardExitLimitMs << "），退出模式 "
                  << static_cast<int>(offboard_exit_mode_) << '/'
                  << static_cast<int>(offboard_exit_sub_mode_) << "，停发后额外发送 "
                  << sends_after_idle << " 条" << std::endl;
        std::cout << "包线: 最大相对高度 " << max_takeoff_height_m_ << " m，最大水平漂移 "
                  << max_takeoff_drift_m_ << " m，最大北向位移 " << max_north_m_
                  << " m，控制权最终态 "
                  << (controller_.Authority() == drone::control::ControlAuthority::kIdle
                          ? "IDLE"
                          : (controller_.Authority() == drone::control::ControlAuthority::kActive
                                 ? "ACTIVE"
                                 : "COAST"))
                  << std::endl;
        std::cout << "控制器诊断: setpoint=" << controller_.SetpointCount()
                  << " rejected=" << controller_.RejectedIntentCount()
                  << " clamped=" << controller_.ClampedCount()
                  << " error=" << controller_.ErrorCount()
                  << " 引导注入=" << guide_injected_ << " 条" << std::endl;
        if (!abort_reason_.empty()) {
            std::cout << "[信息] 脚本中断原因: " << abort_reason_ << std::endl;
        }
        const bool passed = observed_.heartbeat && connected_at_end &&
                            link_.ErrorCount() == 0 && guide_injected_ >= kStage4GuideMinSends &&
                            offboard_command_queued_ && offboard_ack_received_ &&
                            offboard_ack_accepted_ && offboard_mode_reached_ &&
                            arm_command_queued_ && arm_ack_received_ && arm_ack_accepted_ &&
                            armed_reached_ && handover_ok && takeoff_ok && zero_hold_ok &&
                            step_ok && stop_ok && land_ok && disarm_ok &&
                            !safety_violation_ && controller_.ErrorCount() == 0;
        std::cout << (passed ? "结果: stage4 执行层门禁通过"
                             : "结果: stage4 执行层门禁未通过")
                  << std::endl;
        return passed;
    }

private:
    // ---- 脚本意图发布 ----
    /// 按当前意图模式发布 ControlIntent（10Hz；kNone 时自然停发=超龄注入）。
    void PublishScriptIntent(uint64_t now_ms) {
        if (intent_mode_ == Stage4IntentMode::kNone ||
            now_ms < next_intent_publish_ms_) {
            return;
        }
        drone::common::ControlIntent intent;
        intent.header.receive_time_ms = now_ms;
        intent.header.valid_for_ms = kStage4IntentValidMs;
        intent.header.frame_id = 3;  // 局部 NED（控制器坐标约定）
        intent.reason_code = drone::common::kControlIntentReasonRequestControl;
        intent.valid_until_ms = now_ms + kStage4IntentValidMs;
        switch (intent_mode_) {
            case Stage4IntentMode::kPositionTakeoff:
                intent.type = drone::common::ControlIntentType::kPositionGuide;
                intent.control_source = 0;  // 0=位置链（对应 S5 远端引导）
                intent.target_x = takeoff_target_x_;
                intent.target_y = takeoff_target_y_;
                intent.target_z = takeoff_target_z_;
                break;
            case Stage4IntentMode::kNorthStep:
                intent.type = drone::common::ControlIntentType::kVelocityHeading;
                intent.control_source = 1;  // 1=视觉（对应 S7 伺服）
                intent.target_x = kStage4StepSpeedMps;
                break;
            case Stage4IntentMode::kZeroVelocity:
            case Stage4IntentMode::kNone:
                intent.type = drone::common::ControlIntentType::kVelocityHeading;
                intent.control_source = 1;
                break;
        }
        (void)intent_topic_.Emplace(intent);
        next_intent_publish_ms_ = now_ms + kStage4IntentPeriodMs;
    }

    /// 引导零速度流注入（接管前的 setpoint 流来源，见类注释）。
    void InjectGuideSetpoint() {
        drone::common::Px4Setpoint setpoint;
        setpoint.header.receive_time_ms = MonotonicMs();
        setpoint.header.valid_for_ms = 300;
        setpoint.header.frame_id = 3;  // 局部 NED（Px4Link 结构合法性校验）
        setpoint.type = drone::common::SetpointType::kVelocity;
        setpoint.valid = true;
        (void)controller_.SetpointOutput().Emplace(setpoint);
        ++guide_injected_;
    }

    // ---- 命令发送 ----
    bool SendOffboardRequest() {
        constexpr uint8_t kPx4MainModeOffboard = 6;
        const bool queued = link_.SendCommand(
            MAV_CMD_DO_SET_MODE, static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            static_cast<float>(kPx4MainModeOffboard), 0.f, 0.f, 0.f, 0.f, 0.f);
        if (queued) {
            offboard_request_time_ms_ = MonotonicMs();
        }
        return queued;
    }

    bool SendArmDisarm(bool arm) {
        const bool queued = link_.SendCommand(MAV_CMD_COMPONENT_ARM_DISARM, arm ? 1.f : 0.f,
                                              0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
        if (queued) {
            if (arm) {
                arm_request_time_ms_ = MonotonicMs();
            } else {
                disarm_request_time_ms_ = MonotonicMs();
            }
        }
        return queued;
    }

    bool SendAutoLand() {
        constexpr uint8_t kPx4MainModeAuto = 4;
        constexpr uint8_t kPx4SubModeAutoLand = 6;
        const bool queued = link_.SendCommand(
            MAV_CMD_DO_SET_MODE, static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            static_cast<float>(kPx4MainModeAuto), static_cast<float>(kPx4SubModeAutoLand),
            0.f, 0.f, 0.f, 0.f);
        if (queued) {
            land_request_time_ms_ = MonotonicMs();
        }
        return queued;
    }

    /// 脚本中断：记录原因并按安全回收处理（FinishFlight 统一 AUTO.LAND）。
    void Abort(const std::string& reason) {
        if (aborted_) {
            return;
        }
        aborted_ = true;
        abort_reason_ = reason;
        std::cout << "[SITL][中断] " << reason << std::endl;
    }

    // ---- 阶段推进 ----
    void AdvancePhase(uint64_t now_ms, uint64_t elapsed_ms) {
        switch (phase_) {
            case Stage4Phase::kBootstrap:
                AdvanceBootstrap(now_ms, elapsed_ms);
                return;
            case Stage4Phase::kTakeoff:
                AdvanceTakeoff(now_ms);
                return;
            case Stage4Phase::kZeroHold:
                AdvanceZeroHold(now_ms);
                return;
            case Stage4Phase::kStepNorth:
                AdvanceStepNorth(now_ms);
                return;
            case Stage4Phase::kStopIntents:
                AdvanceStopIntents(now_ms);
                return;
            case Stage4Phase::kLand:
                AdvanceLand(now_ms);
                return;
            case Stage4Phase::kDone:
                return;
        }
    }

    void AdvanceBootstrap(uint64_t now_ms, uint64_t elapsed_ms) {
        if (!link_.IsConnected()) {
            if (elapsed_ms >= kStage4ConnectTimeoutMs) {
                Abort("连接超时：未收到 PX4 心跳（SITL 未启动或 UDP 配置不符）");
            }
            return;
        }
        if (!guide_started_) {
            guide_started_ = true;
            guide_start_ms_ = now_ms;
            next_guide_publish_ms_ = now_ms;
            std::cout << "[SITL] 引导阶段开始：注入零速度引导流（PX4 进入 Offboard 前"
                         "要求已有 setpoint 流，控制器授权链要求 armed+Offboard 才发布）"
                      << std::endl;
        }
        if (!handover_done_ && now_ms >= next_guide_publish_ms_) {
            InjectGuideSetpoint();
            next_guide_publish_ms_ = now_ms + kStage4GuidePeriodMs;
        }
        // stage3A 纪律：引导流满足“≥2 秒且 ≥20 条”后才请求 Offboard
        if (!offboard_command_queued_ && guide_injected_ >= kStage4GuideMinSends &&
            now_ms - guide_start_ms_ >= kStage4GuideMinMs) {
            offboard_command_queued_ = SendOffboardRequest();
            std::cout << "[SITL] Offboard 模式请求"
                      << (offboard_command_queued_ ? "已入队" : "入队失败") << std::endl;
        }
        if (have_snapshot_) {
            if (offboard_command_queued_ && !offboard_ack_received_ &&
                latest_.last_ack_time_ms >= offboard_request_time_ms_ &&
                latest_.last_ack_command == MAV_CMD_DO_SET_MODE) {
                offboard_ack_received_ = true;
                offboard_ack_accepted_ = latest_.last_ack_result == MAV_RESULT_ACCEPTED;
            }
            offboard_mode_reached_ =
                offboard_mode_reached_ ||
                (offboard_command_queued_ && latest_.flight_mode == 6 &&
                 latest_.flight_sub_mode == 0);
            if (arm_command_queued_ && !arm_ack_received_ &&
                latest_.last_ack_time_ms >= arm_request_time_ms_ &&
                latest_.last_ack_command == MAV_CMD_COMPONENT_ARM_DISARM) {
                arm_ack_received_ = true;
                arm_ack_accepted_ = latest_.last_ack_result == MAV_RESULT_ACCEPTED;
            }
            // stage3A 前置条件：本轮 Offboard ACK accepted + 真实 6/0 + landed +
            // 位置/姿态有效，才允许 ARM（禁止仅凭初始 mode=6 提前上锁）
            if (!arm_command_queued_ && offboard_ack_accepted_ && offboard_mode_reached_ &&
                latest_.connected && !latest_.armed && latest_.landed_state_valid &&
                latest_.landed && latest_.local_position_valid && latest_.attitude_valid) {
                arm_command_queued_ = SendArmDisarm(true);
                std::cout << "[SITL] ARM 请求" << (arm_command_queued_ ? "已入队" : "入队失败")
                          << std::endl;
            }
            if (latest_.armed) {
                armed_reached_ = true;
            }
        }
        if (armed_reached_) {
            if (!handover_done_ &&
                controller_.Authority() == drone::control::ControlAuthority::kActive) {
                handover_done_ = true;
                handover_ms_ = now_ms;
                send_count_at_handover_ = link_.SetpointSendCount();
                std::cout << "[SITL] 控制器已接管设定值流（授权链全通：armed+Offboard+请求位），"
                             "停止工具引导流"
                          << std::endl;
            }
            if (handover_done_ && !takeover_stream_ok_ &&
                link_.SetpointSendCount() > send_count_at_handover_) {
                takeover_stream_ok_ = true;  // 停注后链路发送仍在增长=控制器在供流
            }
            if (takeover_stream_ok_ && now_ms - handover_ms_ >= 500 &&
                takeoff_target_ready_ && !takeoff_started_) {
                takeoff_started_ = true;
                takeoff_start_ms_ = now_ms;
                rejected_at_takeoff_start_ = controller_.RejectedIntentCount();
                intent_mode_ = Stage4IntentMode::kPositionTakeoff;
                phase_ = Stage4Phase::kTakeoff;
                std::cout << "[SITL] 起飞阶段开始：位置引导意图 → 相对起飞 1m"
                          << std::endl;
                return;
            }
        }
        if (elapsed_ms >= kStage4BootstrapTimeoutMs) {
            Abort(takeover_stream_ok_
                      ? "控制器已接管但起飞基准未建立（armed 后 local_position 无效）"
                      : "引导/Offboard/ARM/接管阶段超时（检查 ACK、模式与授权链）");
        }
    }

    void AdvanceTakeoff(uint64_t now_ms) {
        if (!takeoff_target_ready_ || !latest_.local_position_valid) {
            if (now_ms - takeoff_start_ms_ >= kStage4TakeoffTimeoutMs) {
                Abort("起飞超时：位置基准缺失或位置遥测无效");
            }
            return;
        }
        const float ascent_m = start_z_ - latest_.local_z_m;
        const float dx = latest_.local_x_m - start_x_;
        const float dy = latest_.local_y_m - start_y_;
        const float drift_m = std::sqrt(dx * dx + dy * dy);
        max_takeoff_height_m_ = std::max(max_takeoff_height_m_, ascent_m);
        max_takeoff_drift_m_ = std::max(max_takeoff_drift_m_, drift_m);
        const bool inside_window =
            std::fabs(latest_.local_z_m - takeoff_target_z_) <= kStage4HoverZErrorM &&
            drift_m <= kStage4HoverDriftM && std::fabs(latest_.vz_mps) <= kStage4HoverVzMps;
        if (inside_window) {
            takeoff_height_reached_ = true;
            if (hover_stable_since_ms_ == 0) {
                hover_stable_since_ms_ = now_ms;
            }
            hover_complete_ = now_ms - hover_stable_since_ms_ >= 3000;
        } else {
            hover_stable_since_ms_ = 0;
        }
        if (hover_complete_) {
            takeoff_rejected_delta_ =
                controller_.RejectedIntentCount() - rejected_at_takeoff_start_;
            std::cout << "[SITL] 悬停稳定 3 秒完成（最大相对高度 " << max_takeoff_height_m_
                      << " m），切零速度意图保持" << std::endl;
            phase_ = Stage4Phase::kZeroHold;
            intent_mode_ = Stage4IntentMode::kZeroVelocity;
            zero_hold_start_ms_ = 0;
            return;
        }
        if (now_ms - takeoff_start_ms_ >= kStage4TakeoffTimeoutMs) {
            Abort("起飞/悬停超时（检查 home_valid 与位置意图是否被控制拒绝）");
        }
    }

    void AdvanceZeroHold(uint64_t now_ms) {
        if (!latest_.local_position_valid) {
            return;
        }
        if (zero_hold_start_ms_ == 0) {
            zero_hold_start_ms_ = now_ms;
            zero_hold_x0_ = latest_.local_x_m;
            zero_hold_y0_ = latest_.local_y_m;
            std::cout << "[SITL] 零速度意图保持 3 秒开始" << std::endl;
            return;
        }
        const float dx = latest_.local_x_m - zero_hold_x0_;
        const float dy = latest_.local_y_m - zero_hold_y0_;
        zero_hold_max_drift_m_ =
            std::max(zero_hold_max_drift_m_, std::sqrt(dx * dx + dy * dy));
        zero_hold_max_speed_mps_ =
            std::max(zero_hold_max_speed_mps_,
                     std::sqrt(latest_.vx_mps * latest_.vx_mps +
                               latest_.vy_mps * latest_.vy_mps));
        if (now_ms - zero_hold_start_ms_ >= kStage4ZeroHoldMs) {
            zero_hold_complete_ = true;
            zero_hold_drift_ok_ = zero_hold_max_drift_m_ < kStage4ZeroDriftLimitM;
            std::cout << "[SITL] 零速度保持完成：最大漂移 " << zero_hold_max_drift_m_
                      << " m，切北向 " << kStage4StepSpeedMps << " m/s 阶跃意图"
                      << std::endl;
            phase_ = Stage4Phase::kStepNorth;
            intent_mode_ = Stage4IntentMode::kNorthStep;
            step_start_ms_ = now_ms;
            clamped_at_step_start_ = controller_.ClampedCount();
            speed_samples_.clear();
            setpoint_samples_.clear();
        }
    }

    void AdvanceStepNorth(uint64_t now_ms) {
        if (latest_.local_position_valid) {
            speed_samples_.push_back({now_ms, latest_.vx_mps});
            const float north_m = latest_.local_x_m - start_x_;
            const float east_m = std::fabs(latest_.local_y_m - start_y_);
            step_north_m_ = std::max(step_north_m_, north_m);
            step_east_m_ = std::max(step_east_m_, east_m);
        }
        if (now_ms - step_start_ms_ < kStage4StepHoldMs) {
            return;
        }
        step_complete_ = true;
        step_max_slope_mps2_ = MaxRampSlope(setpoint_samples_);
        step_steady_speed_mps_ = SteadySpeed();
        const double expected =
            std::min<double>(kStage4StepSpeedMps, controller_config_.max_speed_mps);
        step_slope_ok_ =
            step_max_slope_mps2_ <= controller_config_.max_accel_mps2 * kStage4SlopeTolerance;
        step_steady_ok_ = std::fabs(step_steady_speed_mps_ - expected) <=
                          kStage4SteadyToleranceMps;
        clamped_delta_in_step_ = controller_.ClampedCount() - clamped_at_step_start_;
        std::cout << "[SITL] 阶跃段完成：设定值爬升斜率 " << step_max_slope_mps2_
                  << " m/s²（发布侧 " << setpoint_samples_.size()
                  << " 样本），稳态速度 " << step_steady_speed_mps_ << " m/s（期望 "
                  << expected << "，限幅触发 " << clamped_delta_in_step_
                  << " 次）；停止发意图（超龄注入）" << std::endl;
        phase_ = Stage4Phase::kStopIntents;
        intent_mode_ = Stage4IntentMode::kNone;
        stop_intents_ms_ = now_ms;
        expect_offboard_exit_ = true;
    }

    void AdvanceStopIntents(uint64_t now_ms) {
        if (!coast_seen_ &&
            controller_.Authority() == drone::control::ControlAuthority::kCoast) {
            coast_seen_ = true;
            coast_ms_ = now_ms;
            std::cout << "[SITL] 控制器进入 COAST（意图超龄）零速刹车，等待速度过零"
                      << std::endl;
        }
        if (coast_seen_ && !brake_zero_seen_ && latest_.local_position_valid) {
            const float speed = std::hypot(latest_.vx_mps, latest_.vy_mps);
            if (speed <= kStage4ZeroSpeedMps) {
                brake_zero_seen_ = true;
                brake_zero_ms_ = now_ms;
                std::cout << "[SITL] 速度过零（≤" << kStage4ZeroSpeedMps
                          << " m/s）：COAST 后 " << brake_zero_ms_ - coast_ms_ << " ms"
                          << std::endl;
            }
        }
        if (!idle_seen_ &&
            controller_.Authority() == drone::control::ControlAuthority::kIdle) {
            idle_seen_ = true;
            idle_ms_ = now_ms;
            sends_at_idle_ = link_.SetpointSendCount();
            std::cout << "[SITL] 控制器转 IDLE 停发（COAST 超时），等待 PX4 退出 Offboard"
                      << std::endl;
        }
        if (idle_seen_ && !offboard_exit_seen_ && have_snapshot_ && latest_.flight_mode != 6) {
            offboard_exit_seen_ = true;
            offboard_exit_ms_ = now_ms;
            offboard_exit_mode_ = latest_.flight_mode;
            offboard_exit_sub_mode_ = latest_.flight_sub_mode;
            std::cout << "[SITL] PX4 已退出 Offboard: mode="
                      << static_cast<int>(offboard_exit_mode_) << '/'
                      << static_cast<int>(offboard_exit_sub_mode_) << "（IDLE 后 "
                      << offboard_exit_ms_ - idle_ms_ << " ms）" << std::endl;
            land_command_queued_ = SendAutoLand();
            std::cout << "[SITL] AUTO.LAND 模式请求"
                      << (land_command_queued_ ? "已入队" : "入队失败") << std::endl;
            phase_ = Stage4Phase::kLand;
            return;
        }
        if (now_ms - stop_intents_ms_ >= kStage4StopTimeoutMs) {
            Abort("停发意图后 PX4 未在限时内退出 Offboard");
        }
    }

    void AdvanceLand(uint64_t now_ms) {
        if (have_snapshot_ && land_command_queued_ && !land_ack_received_ &&
            latest_.last_ack_time_ms >= land_request_time_ms_ &&
            latest_.last_ack_command == MAV_CMD_DO_SET_MODE) {
            land_ack_received_ = true;
            land_ack_accepted_ = latest_.last_ack_result == MAV_RESULT_ACCEPTED;
        }
        if (have_snapshot_ && land_ack_accepted_ && latest_.landed_state_valid &&
            latest_.landed) {
            landed_after_flight_ = true;
            std::cout << "[SITL] PX4 已确认降落，准备主动 DISARM" << std::endl;
            phase_ = Stage4Phase::kDone;
            return;
        }
        if (now_ms - land_request_time_ms_ >= kStage4LandTimeoutMs) {
            Abort("AUTO.LAND 后未确认 landed");
        }
    }

    // ---- 观测与安全 ----
    void PrintStateChange() {
        if (latest_.connected == last_connected_ && latest_.armed == last_armed_ &&
            latest_.flight_mode == last_mode_ &&
            latest_.flight_sub_mode == last_sub_mode_) {
            return;
        }
        std::cout << "[状态变化] connected=" << YesNo(latest_.connected)
                  << " armed=" << YesNo(latest_.armed)
                  << " mode=" << static_cast<int>(latest_.flight_mode) << '/'
                  << static_cast<int>(latest_.flight_sub_mode) << std::endl;
        last_connected_ = latest_.connected;
        last_armed_ = latest_.armed;
        last_mode_ = latest_.flight_mode;
        last_sub_mode_ = latest_.flight_sub_mode;
    }

    /// 飞行安全包线：armed 期间连接/位置/姿态/Offboard/高度/漂移异常立即
    /// AUTO.LAND 回收（与 stage3 同语义；停发阶段主动退出 Offboard 记为预期）。
    void CheckFlightEnvelope() {
        // 主动 DISARM 前被自动上锁（COM_DISARM_PRFLT 等）→ 记失败。
        if (armed_reached_ && !latest_.armed && !disarm_command_queued_) {
            unexpected_disarm_before_command_ = true;
        }
        if (!latest_.armed || land_command_queued_ || safety_violation_) {
            return;
        }
        if (!latest_.connected || !latest_.local_position_valid ||
            !latest_.attitude_valid) {
            safety_violation_ = true;
            std::cout << "[SITL][安全回收] 连接、位置或姿态失效" << std::endl;
            return;
        }
        if (!expect_offboard_exit_ && latest_.flight_mode != 6) {
            safety_violation_ = true;
            std::cout << "[SITL][安全回收] 非预期退出 Offboard（mode="
                      << static_cast<int>(latest_.flight_mode) << "）" << std::endl;
            return;
        }
        if (!takeoff_target_ready_) {
            return;
        }
        const bool motion_phase =
            phase_ == Stage4Phase::kStepNorth || phase_ == Stage4Phase::kStopIntents;
        const float ascent_m = start_z_ - latest_.local_z_m;
        const float north_m = latest_.local_x_m - start_x_;
        const float east_m = std::fabs(latest_.local_y_m - start_y_);
        const bool height_out =
            ascent_m > kStage4MaxHeightM ||
            (motion_phase && ascent_m < kStage4MinHeightInMotionM);
        const bool horizontal_out =
            motion_phase ? (north_m > kStage4MaxNorthM || east_m > kStage4MaxEastM)
                         : (std::hypot(north_m, east_m) > kStage4MaxDriftBeforeStepM);
        max_north_m_ = std::max(max_north_m_, north_m);
        if (height_out || horizontal_out) {
            safety_violation_ = true;
            std::cout << "[SITL][安全回收] 高度/水平位移超过包线: 相对高度 " << ascent_m
                      << " m，北向 " << north_m << " m，东西 " << east_m << " m"
                      << std::endl;
        }
    }

    /// 阶跃段设定值最大爬升斜率：按 300ms 时间窗差分（不以样本数代替时间），
    /// 输入为控制器发布侧样本（含发布打戳），验证 slew 限幅端到端生效。
    double MaxRampSlope(const std::vector<Stage4SpeedSample>& samples) const {
        double max_slope = 0.0;
        for (std::size_t head_index = 0; head_index < samples.size(); ++head_index) {
            const Stage4SpeedSample& head = samples[head_index];
            for (std::size_t tail_index = head_index + 1; tail_index < samples.size();
                 ++tail_index) {
                const Stage4SpeedSample& tail = samples[tail_index];
                const uint64_t dt_ms = tail.time_ms - head.time_ms;
                if (dt_ms > kStage4SlopeWindowMs) {
                    break;  // 样本按时间单调，超出窗口即停止
                }
                if (dt_ms < kStage4SlopeWindowMs * 5 / 6) {
                    continue;  // 窗口不足 250ms 的对子差分噪声过大，跳过
                }
                const double slope = (static_cast<double>(tail.vx_mps) -
                                      static_cast<double>(head.vx_mps)) /
                                     (static_cast<double>(dt_ms) / 1000.0);
                max_slope = std::max(max_slope, slope);
            }
        }
        return max_slope;
    }

    /// 阶跃段末 1 秒平均北向速度。
    double SteadySpeed() const {
        constexpr uint64_t kWindowMs = 1000;
        if (speed_samples_.empty()) {
            return 0.0;
        }
        const uint64_t end_ms = speed_samples_.back().time_ms;
        double sum = 0.0;
        std::size_t count = 0;
        for (auto iterator = speed_samples_.rbegin(); iterator != speed_samples_.rend();
             ++iterator) {
            if (end_ms - iterator->time_ms > kWindowMs) {
                break;
            }
            sum += iterator->vx_mps;
            ++count;
        }
        return count == 0 ? 0.0 : sum / static_cast<double>(count);
    }

    /// 收尾：未落地先强制 AUTO.LAND 回收（禁止空中 DISARM），确认 landed 后主动 DISARM。
    void FinishFlight() {
        const bool armed_now = armed_reached_ || latest_.armed;
        if (armed_now && !landed_after_flight_) {
            if (!land_command_queued_) {
                land_command_queued_ = SendAutoLand();
                std::cout << "[SITL] 异常回收 AUTO.LAND 请求"
                          << (land_command_queued_ ? "已入队" : "入队失败") << std::endl;
            }
            const uint64_t deadline = MonotonicMs() + kStage4LandTimeoutMs;
            while (MonotonicMs() < deadline) {
                if (auto message = subscription_.WaitTakeFor(std::chrono::milliseconds(100))) {
                    latest_ = **message;
                    UpdateObserved(latest_, &observed_);
                    if (land_command_queued_ && !land_ack_received_ &&
                        latest_.last_ack_time_ms >= land_request_time_ms_ &&
                        latest_.last_ack_command == MAV_CMD_DO_SET_MODE) {
                        land_ack_received_ = true;
                        land_ack_accepted_ = latest_.last_ack_result == MAV_RESULT_ACCEPTED;
                    }
                    if (latest_.landed_state_valid && latest_.landed) {
                        landed_after_flight_ = true;
                        break;
                    }
                }
            }
            std::cout << "[SITL] 回收结果: landed=" << YesNo(landed_after_flight_) << std::endl;
        }
        if (armed_now && landed_after_flight_) {
            while (subscription_.TryTake()) {
                // 清除排队旧快照，避免用解锁前 armed=false 误判上锁完成。
            }
            disarm_command_queued_ = SendArmDisarm(false);
            std::cout << "[SITL] DISARM 请求"
                      << (disarm_command_queued_ ? "已入队" : "入队失败") << std::endl;
            const uint64_t deadline = MonotonicMs() + 5000;
            while (MonotonicMs() < deadline) {
                if (auto message = subscription_.WaitTakeFor(std::chrono::milliseconds(100))) {
                    latest_ = **message;
                    UpdateObserved(latest_, &observed_);
                    if (disarm_command_queued_ &&
                        latest_.last_ack_time_ms >= disarm_request_time_ms_ &&
                        latest_.last_ack_command == MAV_CMD_COMPONENT_ARM_DISARM) {
                        disarm_ack_received_ = true;
                        disarm_ack_accepted_ =
                            latest_.last_ack_result == MAV_RESULT_ACCEPTED;
                    }
                    if (disarm_ack_accepted_ && !latest_.armed) {
                        disarmed_confirmed_ = true;
                        break;
                    }
                }
            }
        }
    }

    // ---- 依赖 ----
    drone::communication::Px4Link& link_;
    drone::control::FlightController& controller_;
    drone::common::Topic<drone::common::ControlIntent>& intent_topic_;
    const drone::config::FlightControllerConfig& controller_config_;
    uint64_t duration_ms_ = 0;
    drone::common::Topic<drone::common::FlightStateSnapshot>::Subscription subscription_;

    // ---- 观测 ----
    drone::common::FlightStateSnapshot latest_;
    bool have_snapshot_ = false;
    ObservedState observed_;
    bool last_connected_ = false;
    bool last_armed_ = false;
    uint8_t last_mode_ = 0;
    uint8_t last_sub_mode_ = 0;

    // ---- 序列推进 ----
    Stage4Phase phase_ = Stage4Phase::kBootstrap;
    Stage4IntentMode intent_mode_ = Stage4IntentMode::kZeroVelocity;
    bool aborted_ = false;
    std::string abort_reason_;
    uint64_t next_intent_publish_ms_ = 0;
    uint64_t next_guide_publish_ms_ = 0;
    bool guide_started_ = false;
    uint64_t guide_start_ms_ = 0;
    uint64_t guide_injected_ = 0;

    // 引导/Offboard/ARM/接管
    bool offboard_command_queued_ = false;
    bool offboard_ack_received_ = false;
    bool offboard_ack_accepted_ = false;
    bool offboard_mode_reached_ = false;
    uint64_t offboard_request_time_ms_ = 0;
    bool arm_command_queued_ = false;
    bool arm_ack_received_ = false;
    bool arm_ack_accepted_ = false;
    bool armed_reached_ = false;
    uint64_t arm_request_time_ms_ = 0;
    bool handover_done_ = false;
    bool takeover_stream_ok_ = false;
    uint64_t handover_ms_ = 0;
    uint64_t send_count_at_handover_ = 0;

    // 起飞起点/悬停
    bool takeoff_target_ready_ = false;
    float start_x_ = 0.f;
    float start_y_ = 0.f;
    float start_z_ = 0.f;
    float takeoff_target_x_ = 0.f;
    float takeoff_target_y_ = 0.f;
    float takeoff_target_z_ = 0.f;
    bool takeoff_started_ = false;
    uint64_t takeoff_start_ms_ = 0;
    uint64_t rejected_at_takeoff_start_ = 0;
    uint64_t takeoff_rejected_delta_ = 0;
    bool takeoff_height_reached_ = false;
    bool hover_complete_ = false;
    uint64_t hover_stable_since_ms_ = 0;
    float max_takeoff_height_m_ = 0.f;
    float max_takeoff_drift_m_ = 0.f;

    // 零速保持
    uint64_t zero_hold_start_ms_ = 0;
    float zero_hold_x0_ = 0.f;
    float zero_hold_y0_ = 0.f;
    float zero_hold_max_drift_m_ = 0.f;
    float zero_hold_max_speed_mps_ = 0.f;
    bool zero_hold_complete_ = false;
    bool zero_hold_drift_ok_ = false;

    // 北向阶跃
    uint64_t step_start_ms_ = 0;
    bool step_complete_ = false;
    bool step_slope_ok_ = false;
    bool step_steady_ok_ = false;
    double step_max_slope_mps2_ = 0.0;
    double step_steady_speed_mps_ = 0.0;
    float step_north_m_ = 0.f;
    float step_east_m_ = 0.f;
    uint64_t clamped_at_step_start_ = 0;
    uint64_t clamped_delta_in_step_ = 0;
    std::vector<Stage4SpeedSample> speed_samples_;
    // 控制器设定值输出订阅与样本（阶跃斜率发布侧测量）
    drone::common::Topic<drone::common::Px4Setpoint>::Subscription setpoint_subscription_;
    std::vector<Stage4SpeedSample> setpoint_samples_;

    // 停发/COAST/Offboard 退出
    uint64_t stop_intents_ms_ = 0;
    bool expect_offboard_exit_ = false;
    bool coast_seen_ = false;
    uint64_t coast_ms_ = 0;
    bool brake_zero_seen_ = false;
    uint64_t brake_zero_ms_ = 0;
    bool idle_seen_ = false;
    uint64_t idle_ms_ = 0;
    uint64_t sends_at_idle_ = 0;
    bool offboard_exit_seen_ = false;
    uint64_t offboard_exit_ms_ = 0;
    uint8_t offboard_exit_mode_ = 0;
    uint8_t offboard_exit_sub_mode_ = 0;

    // 降落/上锁
    bool land_command_queued_ = false;
    bool land_ack_received_ = false;
    bool land_ack_accepted_ = false;
    uint64_t land_request_time_ms_ = 0;
    bool landed_after_flight_ = false;
    bool disarm_command_queued_ = false;
    bool disarm_ack_received_ = false;
    bool disarm_ack_accepted_ = false;
    bool disarmed_confirmed_ = false;
    uint64_t disarm_request_time_ms_ = 0;
    bool unexpected_disarm_before_command_ = false;

    // 安全
    bool safety_violation_ = false;
    float max_north_m_ = 0.f;
};

/// stage4 入口：装配 FlightController → Px4Link 全链并运行脚本。
int RunStage4(drone::communication::Px4Link& link, const json& root,
              const Options& options) {
    const drone::config::FlightControllerConfig controller_config =
        LoadControllerConfig(root);
    std::cout << "  controller: period=" << controller_config.period.count()
              << "ms max_speed=" << controller_config.max_speed_mps
              << "m/s max_accel=" << controller_config.max_accel_mps2
              << "m/s² intent_timeout=" << controller_config.intent_timeout.count()
              << "ms coast_timeout=" << controller_config.coast_timeout.count()
              << "ms" << std::endl;

    drone::common::Topic<drone::common::ControlIntent> intent_topic;
    drone::control::FlightController controller(controller_config);
    controller.SetInput(intent_topic);
    controller.SetPx4LinkSource(&link);
    link.SetInput(controller.SetpointOutput());
    auto subscription = link.StateOutput().Subscribe(64);
    if (!link.Start()) {
        std::cerr << "[FAIL] PX4 链路启动失败，请检查 UDP 端口占用与配置" << std::endl;
        return 2;
    }
    if (!controller.Start()) {
        std::cerr << "[FAIL] 飞行控制器启动失败" << std::endl;
        link.Stop();
        return 2;
    }
    Stage4Run run(link, controller, intent_topic, controller_config, options.duration,
                  std::move(subscription));
    run.Execute();
    const bool connected_at_end = link.IsConnected();
    controller.Stop();
    link.Stop();
    return run.Evaluate(connected_at_end) ? 0 : 3;
}

// ---------------------------------------------------------------------------
// stage4-mission（G3 全链）：状态机转正——注入地面站目标 + 模拟视觉观测驱动
// S5→S6→S7→回退→停发全流程；另含 G2 表列但 stage4 未覆盖的两项补验
// （围栏钳制、切非 Offboard 立即停发 <200ms）。
//
// 与 stage4 的边界差异：stage4 由工具扮演状态机直接发布脚本意图；本段装配正式
// MissionStateMachine（唯一意图生产者），工具只扮演地面站/视觉监视器/双目测距
// 三路生产者并注入模拟数据，验收"迁移序列与设计 §2 判据一致、交接处输出连续、
// 回退后卡尔曼备份接管有效"。
//
// 数据流（与 DroneApplication 正式装配同构，上游换为注入）：
//   注入地面站目标（10Hz，WGS-84）→ TargetEstimator（真实卡尔曼 + 视觉融合）
//     → TargetState ──────────────────────┐
//   注入模拟视觉观测（VisualTargetStatus）├─► MissionStateMachine ──► ControlIntent
//   注入模拟测距（StereoTargetDistance）──┤   （S5 位置引导 / 交接判据 /
//   Px4Link 真实遥测 → FlightStateSnapshot┘    S7 视觉伺服 / 热备份核对）
//     → FlightController（门禁/限幅/无扰衔接）→ Px4Setpoint → Px4Link → PX4
//
// 脚本序列：
//   0) 引导零速度流 → Offboard → ARM → 工具引导意图接管（stage3A/stage4 同纪律）
//   1) [G2 补验] 飞行中切非 Offboard(AUTO.LOITER) → 控制器停发 <200ms；
//      随后引导流重入 Offboard 重新接管
//   2) 工具位置引导意图起飞至相对 1m 并稳定悬停（此时未注入地面站目标，
//      状态机停在 READY 静默，意图流唯一生产者仍是工具）
//   3) [G2 补验] 注入围栏外目标（150m > controller.max_radius_m）→ 位置设定值
//      被钳制在围栏内（ClampedCount 增长 + 实测设定值水平模长 ≤ 围栏）
//   4) 换注入近距目标（40m）→ S5（GPS_APPROACH）位置引导；随后注入
//      SEARCHING → LOCKED 帧序列 + 测距 → S6（冻结位置意图续发）→ S7（视觉伺服）
//   5) 短期 LOST（10 帧 < exit_lost_frames）→ 保持 S7 + 判丢零速刹车意图
//   6) 持续 LOST（达 exit_lost_frames）→ 回退 S5；卡尔曼备份接管
//      （位置意图恢复、控制权不中断）
//   7) 停止注入目标 → 过期 → 位置意图停发 → COAST 刹车 → IDLE → PX4 退出
//      Offboard（返航语义：本阶段 RETURN_HOME 无生产入口，停发后由工具按
//      台架纪律 AUTO.LAND 回收）
//   8) AUTO.LAND → landed → 主动 DISARM
// ---------------------------------------------------------------------------

/// G3 固定边界（不可经命令行放宽）。
constexpr uint8_t kMissionFrameLocalNed = 3;       // 局部 NED（types.h 文件头约定）
constexpr double kMissionDegToRad = 3.14159265358979323846 / 180.0;
constexpr uint32_t kMissionGroundBootId = 0x35430001U;  // 模拟地面站启动标识
/// 注入目标 validity_flags（与 docs/数据接口文档.md §3.4 同源：bit0 高度、
/// bit5 水平精度、bit6 垂直精度）。
constexpr uint32_t kMissionValidityAltitude = 1U << 0U;
constexpr uint32_t kMissionValidityHorizontal = 1U << 5U;
constexpr uint32_t kMissionValidityVertical = 1U << 6U;
/// 状态机意图 reason_code 低位段（编码表见 docs/数据接口文档.md §3.5；
/// 与 src/state_machine/mission_state_machine.cpp 的常量同源镜像，改动须同步）。
constexpr uint32_t kMissionReasonGroundStationTask = 7U;    // S5/S6 位置引导（含冻结续发）
constexpr uint32_t kMissionReasonVisualTracking = 100U;     // S7 视觉正常跟踪（速度航向）
constexpr uint32_t kMissionReasonVisualBrake = 102U;        // S7 判丢零速刹车
/// 低位段掩码（bit31 及以上为标志位段：状态机显式请求控制权）。
constexpr uint32_t kMissionReasonLowMask = 0xFFU;
/// MissionStatus 告警位（与 src/state_machine/mission_state_machine.cpp 定义同源；
/// 该实现文件的常量在匿名命名空间不可外链，工具侧镜像位语义，改动须同步）。
constexpr uint32_t kMissionWarningGroundTargetStaleBit = 1U << 1U;  // S5 目标过期
constexpr float kMissionTakeoffHeightM = 1.0f;     // 相对起飞高度（与 stage4 同）
constexpr float kMissionHoverZErrorM = 0.15f;      // 悬停窗口：高度误差
constexpr float kMissionHoverDriftM = 0.30f;       // 悬停窗口：水平漂移
constexpr float kMissionHoverVzMps = 0.30f;        // 悬停窗口：垂速
constexpr float kMissionMaxHeightM = 1.30f;        // armed 期间硬性高度上限
constexpr float kMissionMinHeightM = 0.50f;        // armed 期间硬性高度下限
constexpr float kMissionMaxEastM = 2.0f;           // 全段东西向硬边界
constexpr float kMissionNorthMarginM = 6.0f;       // 主目标北向包线余量
constexpr double kMissionTargetNorthM = 40.0;      // 主目标北向距离（Home 相对）
constexpr double kMissionFenceProbeNorthM = 150.0; // 围栏探针目标（须 > max_radius）
constexpr double kMissionTargetUpM = 1.0;          // 目标高度（相对 Home，与悬停同高）
constexpr uint64_t kMissionToolIntentValidMs = 500;  // 工具引导意图有效期（工具自用）
constexpr uint64_t kMissionIntentPeriodMs = 100;   // 工具引导意图周期（10Hz）
constexpr uint64_t kMissionGuidePeriodMs = 100;    // 引导 setpoint 流周期（10Hz）
constexpr uint64_t kMissionGuideMinSends = 20;     // 引导流最小条数（stage3A 纪律）
constexpr uint64_t kMissionGuideMinMs = 2000;      // 引导流最短时长（stage3A 纪律）
constexpr uint64_t kMissionConnectTimeoutMs = 10000;   // 未连接快速失败（负对照）
constexpr uint64_t kMissionBootstrapTimeoutMs = 20000; // 引导/Offboard/ARM/接管上限
constexpr uint64_t kMissionTakeoffTimeoutMs = 15000;   // 起飞/悬停上限
constexpr uint64_t kMissionHoverStableMs = 2000;       // 悬停稳定判据时长
constexpr uint64_t kMissionProbeTimeoutMs = 15000;     // 围栏/交接/回退各段上限
constexpr uint64_t kMissionTargetPeriodMs = 100;   // 地面站目标注入周期（10Hz）
constexpr uint64_t kMissionTargetValidMs = 2000;   // 注入目标有效期
constexpr uint32_t kMissionTargetIdProbe = 1;      // 围栏探针目标 ID
constexpr uint32_t kMissionTargetIdMain = 2;       // 主目标 ID（换 ID 触发卡尔曼重初始化）
constexpr uint64_t kMissionVisionPeriodMs = 50;    // 视觉/测距注入周期（20Hz）
constexpr uint64_t kMissionVisualSearchMs = 500;   // SEARCHING 帧注入时长（锁定前）
constexpr uint64_t kMissionShortLostFrames = 10;   // 短期 LOST 注入帧数（< exit_lost_frames）
constexpr uint64_t kMissionModeStopLimitMs = 200;  // 切非 Offboard 停发上限（设计 §4）
constexpr uint64_t kMissionModeStopSettleMs = 300; // 停发后再观察时长（发送冻结核验）
constexpr uint64_t kMissionStopTimeoutMs = 15000;  // 停发→Offboard 退出上限
// 上限含义同 kStage4OffboardExitLimitMs：COM_OF_LOSS_T(1.0s) + HEARTBEAT 1Hz
// 相位抖动（≤1s）+ 余量；早前 1300ms 按 1Hz 心跳测法随机失败。
constexpr uint64_t kMissionOffboardExitLimitMs = 2200;  // IDLE→PX4 退出 Offboard 上限
constexpr uint64_t kMissionExpiryTimeoutMs = 8000; // 目标过期→停发链上限
constexpr uint64_t kMissionLandTimeoutMs = 30000;  // AUTO.LAND→landed 上限
constexpr uint64_t kMissionExtraSendsAllowed = 2;  // 停发后容许的竞态额外发送
constexpr double kMissionSlewTolerance = 1.5;      // 输出连续性容差倍数
constexpr double kMissionFenceToleranceM = 0.5;    // 围栏钳制实测容差
constexpr uint64_t kMissionPollMs = 20;            // 主循环节拍（低于控制周期 50ms）

/// G3 意图来源分类（纯逻辑，`--selftest` 离线断言；工具不在 ctest 内的断言曾连漏
/// 两次——2026-10-14 终审要求抽成纯函数 + 离线自检防复发）。
enum class MissionIntentClass : uint8_t {
    kToolBootstrap = 0,      ///< 工具自身引导意图（reason 低字节 0）：不计入状态机轨迹
    kPositionGuide = 1,      ///< 状态机位置引导/冻结续发（低字节 7；S5/S6 及 S7 过渡期）
    kVisualServo = 2,        ///< 状态机 S7 视觉伺服速度意图（低字节 100）
    kVisualBrake = 3,        ///< 状态机 S7 判丢零速刹车意图（低字节 102）
    kStateMachineOther = 4,  ///< 状态机其它意图（其它低位码）：计入轨迹、不分类
};

/// 分类名（自检输出与日志用）。
inline const char* MissionIntentClassName(MissionIntentClass value) {
    switch (value) {
        case MissionIntentClass::kToolBootstrap:
            return "工具引导";
        case MissionIntentClass::kPositionGuide:
            return "位置引导(7)";
        case MissionIntentClass::kVisualServo:
            return "视觉伺服(100)";
        case MissionIntentClass::kVisualBrake:
            return "判丢刹车(102)";
        case MissionIntentClass::kStateMachineOther:
            return "其它低位码";
    }
    return "未知";
}

/// 意图来源分类（reason 低位段 + 意图类型双条件）。
/// **纪律**：只排除**工具自身引导意图**（低字节 0，工具是接管前的意图生产者）；
/// 不得改成"只认低字节 7"——状态机 S7 的伺服（100）/刹车（102）意图会被整体
/// 丢弃，servo_intent_seen_ 永假、交接判据永不计算、脚本在交接段超时中止
/// （2026-10-14 终审 Critical 的根因）。分类正确性由 `--selftest` 离线断言。
inline MissionIntentClass ClassifyMissionIntent(uint32_t reason_code,
                                               common::ControlIntentType type) {
    const uint32_t low = reason_code & kMissionReasonLowMask;
    if (low == 0U) {
        return MissionIntentClass::kToolBootstrap;
    }
    if (low == kMissionReasonGroundStationTask &&
        type == common::ControlIntentType::kPositionGuide) {
        return MissionIntentClass::kPositionGuide;
    }
    if (low == kMissionReasonVisualTracking &&
        type == common::ControlIntentType::kVelocityHeading) {
        return MissionIntentClass::kVisualServo;
    }
    if (low == kMissionReasonVisualBrake &&
        type == common::ControlIntentType::kBrakeHover) {
        return MissionIntentClass::kVisualBrake;
    }
    return MissionIntentClass::kStateMachineOther;
}

/// 是否计入状态机意图轨迹（仅排除工具自身引导意图；未知低位码按状态机意图计入，
/// 保证"漏分类"只影响细分判据、不影响"状态机是否在发意图"的总判据）。
inline bool CountsAsStateMachineIntent(MissionIntentClass value) {
    return value != MissionIntentClass::kToolBootstrap;
}

// ---------------------------------------------------------------------------
// Stage4InjectionRig：三路注入构件（地面站目标 WGS-84 反解 + 模拟视觉观测 +
// 测距），G3 门禁与整定场景共用——注入几何与估计器正变换互逆的单一真源。
/// 所有发布节拍由 now_ms 显式驱动；错误经 error() 粘性上报（首个为准），
/// 由调用方决定中止（Abort）。快照经参数显式传入（nullptr = 无快照）。
// ---------------------------------------------------------------------------
class Stage4InjectionRig {
public:
    Stage4InjectionRig(
        drone::common::Topic<drone::common::GroundStationTarget>& ground_target_topic,
        drone::common::Topic<drone::common::VisualTargetStatus>& visual_topic,
        drone::common::Topic<drone::common::StereoTargetDistance>& distance_topic,
        const drone::control::TrackingCameraGroupConfig& camera,
        const drone::perception::VisualMountExtrinsics& mount, double fxb_m_px)
        : ground_target_topic_(ground_target_topic),
          visual_topic_(visual_topic),
          distance_topic_(distance_topic),
          camera_(camera),
          mount_(mount),
          fxb_m_px_(fxb_m_px) {}

    /// 目标位置：Home 相对 NED（up 为正高，米）。阶跃换位由驱动类调此接口，
    /// 地面站目标与视觉/测距注入共用同一目标点，卡尔曼热备份不发散。
    void SetTargetNed(double north_m, double east_m, double up_m) {
        target_north_m_ = north_m;
        target_east_m_ = east_m;
        target_up_m_ = up_m;
    }
    void SetTargetEnabled(bool on) { target_enabled_ = on; }
    void SetVisionEnabled(bool on) { vision_enabled_ = on; }
    void SetDistanceValid(bool valid) { distance_valid_ = valid; }

    bool target_enabled() const { return target_enabled_; }
    bool vision_enabled() const { return vision_enabled_; }
    bool distance_valid() const { return distance_valid_; }
    const drone::control::TrackingCameraGroupConfig& camera() const { return camera_; }
    double target_north_m() const { return target_north_m_; }
    double target_east_m() const { return target_east_m_; }
    double target_up_m() const { return target_up_m_; }

    /// 粘性错误（首个为准）；设置后注入停止，调用方应中止脚本。
    bool HasError() const { return !error_.empty(); }
    const std::string& error() const { return error_; }

    /// 本拍注入几何（PublishSimulatedVision 内缓存；CSV 记录用）。
    double last_pixel_x() const { return last_pixel_x_; }
    double last_pixel_y() const { return last_pixel_y_; }
    double last_cam_z_m() const { return last_cam_z_m_; }

    /// 注入地面站目标：Home 相对 NED 位置反解 WGS-84 后按 10Hz 发布。
    /// 反解与 perception::GeodeticToLocalNed 同一局部切平面公式（正向回验，
    /// 偏差 >0.05m 判装配错误并置错误标志）。
    bool PublishGroundTarget(uint32_t target_id, uint64_t now_ms,
                             const drone::common::FlightStateSnapshot* latest) {
        if (HasError() || !target_enabled_ || now_ms < next_target_publish_ms_ ||
            latest == nullptr || !latest->home_valid) {
            return false;
        }
        const double ref_lat = static_cast<double>(latest->home_lat_1e7) * 1e-7;
        const double ref_lon = static_cast<double>(latest->home_lon_1e7) * 1e-7;
        const double ref_alt = static_cast<double>(latest->home_altitude_mm) * 1e-3;
        double lat = 0.0;
        double lon = 0.0;
        double alt = 0.0;
        if (!OffsetToGeodetic(ref_lat, ref_lon, ref_alt, target_north_m_, target_east_m_,
                              -target_up_m_, &lat, &lon, &alt)) {
            SetError("注入目标反解失败（Home 基准非法）");
            return false;
        }
        // 正向回验：同一公式必须还原（防装配式笔误导致注入点漂移）。
        const auto verify = drone::perception::GeodeticToLocalNed(
            {ref_lat, ref_lon, ref_alt}, {lat, lon, alt});
        if (!verify.has_value() ||
            std::fabs(verify->north_m - target_north_m_) > 0.05 ||
            std::fabs(verify->east_m - target_east_m_) > 0.05 ||
            std::fabs(verify->down_m + target_up_m_) > 0.05) {
            SetError("注入目标反解回验偏差超限（>0.05m）");
            return false;
        }

        drone::common::GroundStationTarget target;
        target.header.receive_time_ms = now_ms;
        target.header.valid_for_ms = kMissionTargetValidMs;
        target.header.frame_id = 1;  // 1=WGS84 经纬度（与 GroundStationLink 同约定）
        target.ground_station_boot_id = kMissionGroundBootId;
        target.update_seq = ++target_update_seq_;
        target.target_id = target_id;
        target.latitude_1e7 = static_cast<int32_t>(std::llround(lat * 1e7));
        target.longitude_1e7 = static_cast<int32_t>(std::llround(lon * 1e7));
        target.altitude_mm = static_cast<int32_t>(std::llround(alt * 1e3));
        target.alt_reference = 1;  // 1=MSL（估计器要求；Home 高度同基准）
        target.protocol_version = 1;
        target.transport_age_ms = 0;
        target.validity_flags = kMissionValidityAltitude | kMissionValidityHorizontal |
                                kMissionValidityVertical;
        target.horizontal_accuracy_m = 2.0f;
        target.vertical_accuracy_m = 3.0f;
        target.heading_deg = std::numeric_limits<float>::quiet_NaN();
        (void)ground_target_topic_.Emplace(target);
        next_target_publish_ms_ = now_ms + kMissionTargetPeriodMs;
        return true;
    }

    /// 注入一帧模拟视觉观测 + 对应测距：相机系坐标由（目标 NED − 自身 NED）
    /// 经 姿态/安装外参 反变换得到，与 VisualCameraToLocalNed 正变换严格互逆，
    /// 保证估计器视觉融合的横向位置与地面站目标一致（创新 ≈ 0，滤除几何分歧）。
    /// distance_valid 独立于视觉状态：模拟"测距仍跟踪但跟踪器判丢"，
    /// 使 S7 退出由 LOST 迟滞路径唯一决定（测距失效路径由单测覆盖）。
    /// @return 本拍是否实际注入了一帧（按 20Hz 节流；调用方计数用）。
    bool PublishSimulatedVision(common::VisualTargetState state, uint64_t now_ms,
                                const drone::common::FlightStateSnapshot* latest) {
        if (HasError() || !vision_enabled_ || now_ms < next_vision_publish_ms_ ||
            latest == nullptr || !latest->attitude_valid ||
            !latest->local_position_valid) {
            return false;
        }
        const double north_m =
            target_north_m_ - static_cast<double>(latest->local_x_m);
        const double east_m = target_east_m_ - static_cast<double>(latest->local_y_m);
        const double down_m = -target_up_m_ - static_cast<double>(latest->local_z_m);
        double cam_x = 0.0;
        double cam_y = 0.0;
        double cam_z = 0.0;
        NedToCameraFrame(north_m, east_m, down_m,
                         static_cast<double>(latest->roll_rad),
                         static_cast<double>(latest->pitch_rad),
                         static_cast<double>(latest->yaw_rad), &cam_x, &cam_y, &cam_z);
        if (cam_z <= 0.1) {
            SetError("注入视觉几何非法（目标不在相机前向）");
            return false;
        }
        last_cam_z_m_ = cam_z;
        // 像素：与 control::PixelToAngle（angle = atan((pixel − c) / f)）互逆，
        // 使状态机视线角/视场余量判据对注入几何成立。
        const double azimuth_rad = std::atan2(cam_x, cam_z);
        const double elevation_rad = std::atan2(cam_y, cam_z);
        const float pixel_x = static_cast<float>(
            camera_.cx_px + camera_.fx_px * std::tan(azimuth_rad));
        const float pixel_y = static_cast<float>(
            camera_.cy_px + camera_.fy_px * std::tan(elevation_rad));
        last_pixel_x_ = pixel_x;
        last_pixel_y_ = pixel_y;
        const bool locked = state == common::VisualTargetState::kLocked;
        const uint64_t frame_sequence = ++vision_frame_sequence_;

        drone::common::VisualTargetStatus visual;
        visual.header.receive_time_ms = now_ms;
        visual.state = state;
        visual.frame_sequence = frame_sequence;
        visual.last_seen_ms = locked ? now_ms : visual_last_seen_ms_;
        visual.consecutive_detected_frames = locked ? visual_locked_streak_ : 0;
        visual.consecutive_missed_frames = locked ? 0 : visual_lost_streak_;
        visual.confidence = locked ? 0.90f : 0.0f;
        visual.center_pixel_x = pixel_x;
        visual.center_pixel_y = pixel_y;
        visual.frame_width = static_cast<uint32_t>(camera_.image_width);
        visual.frame_height = static_cast<uint32_t>(camera_.image_height);
        visual.valid = locked;
        (void)visual_topic_.Emplace(visual);
        if (locked) {
            ++visual_locked_streak_;
            visual_lost_streak_ = 0;
            visual_last_seen_ms_ = now_ms;
        } else {
            ++visual_lost_streak_;
            visual_locked_streak_ = 0;
        }

        drone::common::StereoTargetDistance distance;
        distance.header.receive_time_ms = now_ms;
        distance.header.source_time_ms = now_ms;  // 估计器按 source_time 排序/超龄
        distance.frame_sequence = frame_sequence;
        distance.class_id = 0;
        distance.track_id = 0;
        distance.forward_distance_m = static_cast<float>(cam_z);
        distance.slant_range_m =
            static_cast<float>(std::sqrt(cam_x * cam_x + cam_y * cam_y + cam_z * cam_z));
        distance.disparity_px = static_cast<float>(fxb_m_px_ / cam_z);
        distance.confidence = 0.90f;
        distance.valid = distance_valid_;
        distance.center_pixel_x_mid = pixel_x;
        distance.camera_x_m = static_cast<float>(cam_x);
        distance.camera_y_m = static_cast<float>(cam_y);
        (void)distance_topic_.Emplace(distance);
        next_vision_publish_ms_ = now_ms + kMissionVisionPeriodMs;
        return true;
    }

    /// 装配期注入几何自检（开跑前调用）：反变换与正变换必须互逆——像素/测距
    /// （NedToCameraFrame ↔ perception::VisualCameraToLocalNed）与地理坐标
    /// （OffsetToGeodetic ↔ perception::GeodeticToLocalNed）。任一处不互逆即
    /// 拒绝开跑：否则估计器视觉融合会与注入目标偏离、创新门限误拒交接。
    bool VerifyInjectionGeometry() const {
        constexpr double kRoll = 0.02;
        constexpr double kPitch = -0.015;
        constexpr double kYaw = 0.25;
        const drone::perception::LocalNedCoordinate self{5.0, -2.0, -1.0};
        const double north_m = 40.0 - self.north_m;
        const double east_m = 0.0 - self.east_m;
        const double down_m = -1.0 - self.down_m;
        double cam_x = 0.0;
        double cam_y = 0.0;
        double cam_z = 0.0;
        NedToCameraFrame(north_m, east_m, down_m, kRoll, kPitch, kYaw, &cam_x, &cam_y,
                         &cam_z);
        const auto back = drone::perception::VisualCameraToLocalNed(
            cam_x, cam_y, cam_z, mount_, kRoll, kPitch, kYaw, self);
        const auto offset = [](const drone::perception::LocalNedCoordinate& a, double n,
                               double e, double d) {
            return std::sqrt((a.north_m - n) * (a.north_m - n) +
                             (a.east_m - e) * (a.east_m - e) +
                             (a.down_m - d) * (a.down_m - d));
        };
        const double camera_error = back.has_value()
            ? offset(*back, self.north_m + north_m, self.east_m + east_m,
                     self.down_m + down_m)
            : std::numeric_limits<double>::infinity();
        constexpr double kRefLat = 47.397742;
        constexpr double kRefLon = 8.545594;
        constexpr double kRefAlt = 488.0;
        double lat = 0.0;
        double lon = 0.0;
        double alt = 0.0;
        const bool solved = OffsetToGeodetic(kRefLat, kRefLon, kRefAlt, north_m, east_m,
                                             down_m, &lat, &lon, &alt);
        const auto geo = solved ? drone::perception::GeodeticToLocalNed(
                                      {kRefLat, kRefLon, kRefAlt}, {lat, lon, alt})
                                : std::nullopt;
        const double geodetic_error = geo.has_value()
            ? std::sqrt((geo->north_m - north_m) * (geo->north_m - north_m) +
                        (geo->east_m - east_m) * (geo->east_m - east_m) +
                        (geo->down_m - down_m) * (geo->down_m - down_m))
            : std::numeric_limits<double>::infinity();
        std::cout << "  注入几何自检: 相机系互逆偏差 " << camera_error
                  << " m，地理反解互逆偏差 " << geodetic_error << " m" << std::endl;
        return camera_error < 1e-6 && geodetic_error < 0.01;
    }

private:
    void SetError(const std::string& reason) {
        if (error_.empty()) {
            error_ = reason;
        }
    }

    /// 目标 NED 差值 → 基准相机系（右/下/前）坐标：VisualCameraToLocalNed
    /// 正变换的逐级逆（姿态 R^T → 安装外参逆 → 固定轴置换逆）。
    void NedToCameraFrame(double north_m, double east_m, double down_m, double roll_rad,
                          double pitch_rad, double yaw_rad, double* cam_x, double* cam_y,
                          double* cam_z) const {
        double r_body[9];
        RotationRpy(roll_rad, pitch_rad, yaw_rad, r_body);
        double bx = 0.0;
        double by = 0.0;
        double bz = 0.0;
        ApplyTranspose(r_body, north_m, east_m, down_m, &bx, &by, &bz);
        // 安装外参：正变换为 b' = R_mount·b + t ⇒ b = R_mount^T·(b' − t)
        double r_mount[9];
        RotationRpy(mount_.roll_deg * kMissionDegToRad, mount_.pitch_deg * kMissionDegToRad,
                    mount_.yaw_deg * kMissionDegToRad, r_mount);
        double mx = 0.0;
        double my = 0.0;
        double mz = 0.0;
        ApplyTranspose(r_mount, bx - mount_.x_m, by - mount_.y_m, bz - mount_.z_m, &mx, &my,
                       &mz);
        // 固定轴置换逆：body = (cam_z, cam_x, cam_y) ⇒ cam = (body_y, body_z, body_x)
        *cam_x = my;
        *cam_y = mz;
        *cam_z = mx;
    }

    /// 航空 RPY（ZYX）旋转矩阵（行主序，弧度；与 visual_ned_converter.cpp 同式）。
    static void RotationRpy(double roll, double pitch, double yaw, double out[9]) {
        const double cr = std::cos(roll);
        const double sr = std::sin(roll);
        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);
        out[0] = cy * cp;
        out[1] = cy * sp * sr - sy * cr;
        out[2] = cy * sp * cr + sy * sr;
        out[3] = sy * cp;
        out[4] = sy * sp * sr + cy * cr;
        out[5] = sy * sp * cr - cy * sr;
        out[6] = -sp;
        out[7] = cp * sr;
        out[8] = cp * cr;
    }

    /// R^T · v（转置左乘）。
    static void ApplyTranspose(const double m[9], double x, double y, double z,
                               double* out_x, double* out_y, double* out_z) {
        *out_x = m[0] * x + m[3] * y + m[6] * z;
        *out_y = m[1] * x + m[4] * y + m[7] * z;
        *out_z = m[2] * x + m[5] * y + m[8] * z;
    }

    /// Home（WGS-84 参考点）按 NED 偏移反解目标 WGS-84（GeodeticToLocalNed 同式反演）。
    static bool OffsetToGeodetic(double ref_lat_deg, double ref_lon_deg, double ref_alt_m,
                                 double north_m, double east_m, double down_m,
                                 double* lat_deg, double* lon_deg, double* alt_m) {
        constexpr double kWgs84SemiMajorAxisM = 6378137.0;
        constexpr double kWgs84Flattening = 1.0 / 298.257223563;
        constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
        const double lat_rad = ref_lat_deg * kDegToRad;
        const double e2 = kWgs84Flattening * (2.0 - kWgs84Flattening);
        const double sin_lat = std::sin(lat_rad);
        const double denom = std::sqrt(1.0 - e2 * sin_lat * sin_lat);
        const double prime_vertical_radius_m = kWgs84SemiMajorAxisM / denom;
        const double meridian_radius_m =
            kWgs84SemiMajorAxisM * (1.0 - e2) / (denom * denom * denom);
        const double delta_lat_rad = north_m / (meridian_radius_m + ref_alt_m);
        const double delta_lon_rad =
            east_m / ((prime_vertical_radius_m + ref_alt_m) * std::cos(lat_rad));
        *lat_deg = ref_lat_deg + delta_lat_rad / kDegToRad;
        *lon_deg = ref_lon_deg + delta_lon_rad / kDegToRad;
        *alt_m = ref_alt_m - down_m;
        return std::isfinite(*lat_deg) && std::isfinite(*lon_deg) &&
               std::isfinite(*alt_m);
    }

    // ---- 依赖 ----
    drone::common::Topic<drone::common::GroundStationTarget>& ground_target_topic_;
    drone::common::Topic<drone::common::VisualTargetStatus>& visual_topic_;
    drone::common::Topic<drone::common::StereoTargetDistance>& distance_topic_;
    const drone::control::TrackingCameraGroupConfig& camera_;
    const drone::perception::VisualMountExtrinsics& mount_;
    const double fxb_m_px_ = 0.0;  // 校正矩阵同源深度系数（装配入口注入）

    // ---- 目标/注入状态 ----
    double target_north_m_ = 0.0;
    double target_east_m_ = 0.0;
    double target_up_m_ = 1.0;
    bool target_enabled_ = true;
    bool vision_enabled_ = false;
    bool distance_valid_ = true;
    uint64_t next_target_publish_ms_ = 0;
    uint32_t target_update_seq_ = 0;
    uint64_t next_vision_publish_ms_ = 0;
    uint64_t vision_frame_sequence_ = 0;
    uint64_t visual_locked_streak_ = 0;
    uint64_t visual_lost_streak_ = 0;
    uint64_t visual_last_seen_ms_ = 0;
    double last_pixel_x_ = std::numeric_limits<double>::quiet_NaN();
    double last_pixel_y_ = std::numeric_limits<double>::quiet_NaN();
    double last_cam_z_m_ = std::numeric_limits<double>::quiet_NaN();
    std::string error_;
};

/// G3 脚本阶段。
enum class MissionPhase : uint8_t {
    kBootstrap,      ///< 连接 → 引导流 → Offboard → ARM → 工具引导意图接管
    kModeStopProbe,  ///< [G2 补验] 切非 Offboard → 控制器停发时延/发送冻结
    kRebootstrap,    ///< 引导流重入 Offboard 并重新接管
    kTakeoff,        ///< 工具位置引导意图：相对起飞 1m 稳定悬停
    kFenceProbe,     ///< [G2 补验] 注入围栏外目标 → 位置意图被钳制
    kApproach,       ///< 注入近距目标 → S5 位置引导（状态机接管意图流）
    kVisualInject,   ///< 注入 SEARCHING→LOCKED + 测距 → S6 → S7
    kShortLost,      ///< 短期 LOST → 保持 S7 + 零速刹车意图
    kSustainedLost,  ///< 持续 LOST → 回退 S5 + 卡尔曼备份接管
    kTargetExpiry,   ///< 停止注入目标 → 过期 → COAST → IDLE → 退出 Offboard
    kLand,           ///< AUTO.LAND → landed
    kDone,
};

/// G3 脚本运行体（见本节文件头注释）。
class Stage4MissionRun {
public:
    Stage4MissionRun(
        drone::communication::Px4Link& link,
        drone::control::FlightController& controller,
        drone::state_machine::MissionStateMachine& machine,
        drone::common::Topic<drone::common::GroundStationTarget>& ground_target_topic,
        drone::common::Topic<drone::common::VisualTargetStatus>& visual_topic,
        drone::common::Topic<drone::common::StereoTargetDistance>& distance_topic,
        drone::common::Topic<drone::common::TargetState>& estimated_topic,
        const drone::config::AppConfig& app_config, double fxb_m_px,
        uint64_t intent_valid_ms, std::chrono::seconds duration,
        drone::common::Topic<drone::common::FlightStateSnapshot>::Subscription
            subscription)
        : link_(link),
          controller_(controller),
          machine_(machine),
          rig_(ground_target_topic, visual_topic, distance_topic,
               app_config.visual_tracking.camera, app_config.target_estimator.visual.mount,
               fxb_m_px),
          estimated_topic_(estimated_topic),
          controller_config_(app_config.controller),
          handover_(app_config.handover),
          intent_valid_ms_(intent_valid_ms),
          duration_ms_(static_cast<uint64_t>(duration.count()) * 1000),
          subscription_(std::move(subscription)) {
        status_subscription_ = machine_.StatusOutput().Subscribe(64);
        intent_subscription_ = machine_.IntentOutput().Subscribe(64);
        setpoint_subscription_ = controller_.SetpointOutput().Subscribe(64);
        estimated_subscription_ = estimated_topic_.Subscribe(16);
    }

    /// 执行脚本主循环（返回到期/中断/完成后仍可能 armed，收尾见 FinishFlight）。
    void Execute() {
        const uint64_t started_ms = MonotonicMs();
        uint64_t next_summary_ms = started_ms;
        while (!g_stop.load(std::memory_order_acquire) &&
               MonotonicMs() - started_ms < duration_ms_) {
            if (auto message =
                    subscription_.WaitTakeFor(std::chrono::milliseconds(kMissionPollMs))) {
                latest_ = **message;
                have_snapshot_ = true;
                UpdateObserved(latest_, &observed_);
                PrintStateChange();
                // 起飞起点锁定：首次 armed 且位置有效时（与 stage3/stage4 同语义）。
                if (latest_.armed && !takeoff_target_ready_ &&
                    latest_.local_position_valid) {
                    takeoff_target_ready_ = true;
                    start_x_ = latest_.local_x_m;
                    start_y_ = latest_.local_y_m;
                    start_z_ = latest_.local_z_m;
                    takeoff_target_z_ = start_z_ - kMissionTakeoffHeightM;
                    std::cout << "[SITL] 起飞目标已锁定: N=" << start_x_
                              << " E=" << start_y_ << " D=" << takeoff_target_z_
                              << std::endl;
                }
            }
            const uint64_t now_ms = MonotonicMs();
            DrainOutputs();
            PublishBootstrapIntent(now_ms);
            AdvancePhase(now_ms, now_ms - started_ms);
            // 注入构件错误（几何非法/反解失败）→ 按中断处理（FinishFlight 统一回收）
            if (!aborted_ && rig_.HasError()) {
                Abort(rig_.error());
            }
            if (have_snapshot_) {
                CheckFlightEnvelope();
                CheckAuthorityContinuity();
            }
            if (now_ms >= next_summary_ms) {
                PrintSummary(latest_, std::chrono::duration_cast<std::chrono::seconds>(
                                           std::chrono::milliseconds(now_ms - started_ms)));
                PrintMissionSummary();
                next_summary_ms = now_ms + 1000;
            }
            if (aborted_ || safety_violation_ || phase_ == MissionPhase::kDone) {
                break;
            }
        }
        FinishFlight();
    }


    /// 装配期注入几何自检（开跑前调用）：委托注入构件（反变换与正变换必须互逆，
    /// 否则估计器视觉融合会与注入目标偏离、创新门限误拒交接）。
    bool VerifyInjectionGeometry() const { return rig_.VerifyInjectionGeometry(); }

    /// 打印 G3 验收报告并返回总判定。
    bool Evaluate(bool connected_at_end) const {
        const double slew_limit =
            controller_config_.max_accel_mps2 * kMissionSlewTolerance;
        const double max_slew = MaxObservedSlewMps2();
        const uint64_t sends_after_mode_stop =
            mode_stop_seen_ ? sends_after_mode_stop_ : 0;
        const uint64_t sends_after_idle =
            idle_seen_ && link_.SetpointSendCount() >= sends_at_idle_
                ? link_.SetpointSendCount() - sends_at_idle_
                : 0;
        const bool mode_stop_ok = mode_stop_seen_ && mode_change_seen_ &&
                                  mode_stop_ms_ >= mode_change_ms_ &&
                                  mode_stop_ms_ - mode_change_ms_ <=
                                      kMissionModeStopLimitMs &&
                                  sends_after_mode_stop <= kMissionExtraSendsAllowed;
        const bool takeover_ok = handover_done_ && takeover_stream_ok_;
        const bool rebootstrap_ok = rebootstrap_takeover_seen_;
        const bool takeoff_ok = takeoff_target_ready_ && takeoff_started_ &&
                                hover_complete_ && !safety_violation_;
        const bool fence_ok = fence_clamp_seen_ && max_position_norm_m_ > 0.0 &&
                              max_position_norm_m_ <=
                                  controller_config_.max_radius_m + kMissionFenceToleranceM;
        const bool sm_takeover_ok = sm_intent_seen_;
        const bool migration_ok = SequenceContains({common::MissionState::kGpsApproach,
                                                    common::MissionState::kVisualHandover,
                                                    common::MissionState::kVisualTracking,
                                                    common::MissionState::kGpsApproach});
        const bool seam_ok = handover_seam_ok_;
        const bool servo_ok = servo_intent_seen_;
        const bool short_lost_ok = short_lost_kept_s7_ && brake_intent_seen_;
        const bool fallback_ok = fallback_seen_ && backup_position_intent_seen_ &&
                                 authority_uninterrupted_;
        const bool continuity_ok = velocity_sample_count_ > 0 && max_slew <= slew_limit;
        const bool expiry_ok = expiry_warning_seen_ && expiry_intent_stop_seen_ &&
                               coast_seen_ && idle_seen_ && offboard_exit_seen_ &&
                               offboard_exit_ms_ - idle_ms_ <= kMissionOffboardExitLimitMs &&
                               sends_after_idle <= kMissionExtraSendsAllowed;
        const bool land_ok = land_command_queued_ && land_ack_received_ &&
                             land_ack_accepted_ && landed_after_flight_;
        const bool disarm_ok = !unexpected_disarm_before_command_ &&
                               disarm_command_queued_ && disarm_ack_received_ &&
                               disarm_ack_accepted_ && disarmed_confirmed_;

        std::cout << "\n========== stage4-mission 全链门禁报告（G3：状态机 → "
                     "FlightController → Px4Link → PX4）=========="
                  << std::endl;
        PrintCheck("PX4 HEARTBEAT", observed_.heartbeat, true);
        PrintCheck("测试结束时心跳仍连接", connected_at_end, true);
        PrintCheck("引导零速度流已注入(≥20 条)", guide_injected_ >= kMissionGuideMinSends,
                   true);
        PrintCheck("控制器已接管设定值流(第一轮授权链全通)", takeover_ok, true);
        PrintCheck("切非 Offboard 停发时延 ≤200ms",
                   mode_stop_ok, true);
        PrintCheck("切回 Offboard 重新接管(第二轮授权链全通)", rebootstrap_ok, true);
        PrintCheck("相对起飞 1m 稳定悬停完成", takeoff_ok, true);
        PrintCheck("围栏钳制: 位置设定值未超 max_radius",
                   fence_ok, true);
        PrintCheck("状态机接管意图流(S5 位置引导 reason=7)",
                   sm_takeover_ok, true);
        PrintCheck("迁移序列 S5→S6→S7→(LOST 回退)S5", migration_ok, true);
        PrintCheck("交接缝意图无超龄(末条S5位置意图→首条S7速度意图 < intent_timeout"
                   " 且未跌入 COAST)",
                   seam_ok, true);
        PrintCheck("S7 视觉伺服意图启动(control_source=1)", servo_ok, true);
        PrintCheck("短期 LOST 保持 S7 且输出零速刹车意图", short_lost_ok, true);
        PrintCheck("持续 LOST 回退 S5 且卡尔曼备份接管(位置意图恢复)",
                   fallback_ok, true);
        PrintCheck("交接/回退全程速度设定值连续(≤max_accel×1.5)",
                   continuity_ok, true);
        PrintCheck("目标过期: 告警位/停发/COAST/IDLE/退出 Offboard 链完整",
                   expiry_ok, true);
        PrintCheck("AUTO.LAND 请求/ACK accepted/PX4 已确认 landed", land_ok, true);
        PrintCheck("主动 DISARM 前未被自动上锁",
                   armed_reached_ && !unexpected_disarm_before_command_, true);
        PrintCheck("DISARM ACK accepted 且已确认 disarmed", disarm_ok, true);
        PrintCheck("飞行未触发高度/漂移安全回收",
                   armed_reached_ && !safety_violation_, true);
        PrintCheck("控制器无异常计数(未支持意图/未知来源)",
                   armed_reached_ && controller_.ErrorCount() == 0, true);
        PrintMessageCheck("LOCAL_POSITION_NED",
                          link_.MessageReceiveCount(MAVLINK_MSG_ID_LOCAL_POSITION_NED),
                          observed_.local_position);
        PrintMessageCheck("ATTITUDE", link_.MessageReceiveCount(MAVLINK_MSG_ID_ATTITUDE),
                          observed_.attitude);
        std::cout << "状态迁移序列: " << StateSequenceText() << std::endl;
        std::cout << "实测值: 切非Offboard→控制器停发 "
                  << (mode_stop_seen_ && mode_change_seen_
                          ? mode_stop_ms_ - mode_change_ms_
                          : 0)
                  << " ms（上限 " << kMissionModeStopLimitMs << "，停发后额外发送 "
                  << sends_after_mode_stop << " 条），围栏探针最大设定值水平模长 "
                  << max_position_norm_m_ << " m（围栏 " << controller_config_.max_radius_m
                  << " m），速度设定值最大隐含加速度 " << max_slew << " m/s²（上限 "
                  << slew_limit << "），交接缝（末条S5位置→首条S7速度） "
                  << handover_seam_gap_ms_ << " ms（意图有效期 " << intent_valid_ms_
                  << " ms，窗口内 COAST=" << YesNo(coast_during_seam_)
                  << "，伺服接管前意图最大间隔 " << pre_servo_max_intent_gap_ms_
                  << " ms，S6 冻结续发观察=" << YesNo(s6_frozen_intent_seen_) << "）"
                  << std::endl;
        std::cout << "实测时延: 过期→COAST "
                  << (coast_seen_ && expiry_stop_ms_ ? coast_ms_ - expiry_stop_ms_ : 0)
                  << " ms，COAST→IDLE "
                  << (coast_seen_ && idle_seen_ ? idle_ms_ - coast_ms_ : 0)
                  << " ms，IDLE→退出 Offboard "
                  << (idle_seen_ && offboard_exit_seen_ ? offboard_exit_ms_ - idle_ms_ : 0)
                  << " ms（上限 " << kMissionOffboardExitLimitMs << "），退出模式 "
                  << static_cast<int>(offboard_exit_mode_) << '/'
                  << static_cast<int>(offboard_exit_sub_mode_) << "，停发后额外发送 "
                  << sends_after_idle << " 条" << std::endl;
        std::cout << "包线: 最大相对高度 " << max_ascent_m_ << " m，最大北向 "
                  << max_north_m_ << " m，最大东西 " << max_east_m_ << " m，控制权最终态 "
                  << AuthorityText(controller_.Authority()) << std::endl;
        std::cout << "状态机诊断: 迁移 " << machine_.TransitionCount() << " 次，错误 "
                  << machine_.ErrorCount() << " 次（估计器错误计数见日志）"
                  << std::endl;
        std::cout << "控制器诊断: setpoint=" << controller_.SetpointCount()
                  << " rejected=" << controller_.RejectedIntentCount()
                  << " clamped=" << controller_.ClampedCount()
                  << " error=" << controller_.ErrorCount() << std::endl;
        if (!abort_reason_.empty()) {
            std::cout << "[信息] 脚本中断原因: " << abort_reason_ << std::endl;
        }
        const bool passed = observed_.heartbeat && connected_at_end &&
                            link_.ErrorCount() == 0 &&
                            guide_injected_ >= kMissionGuideMinSends && takeover_ok &&
                            mode_stop_ok && rebootstrap_ok && takeoff_ok && fence_ok &&
                            sm_takeover_ok && migration_ok && seam_ok && servo_ok &&
                            short_lost_ok && fallback_ok && continuity_ok && expiry_ok &&
                            land_ok && disarm_ok && !safety_violation_ &&
                            controller_.ErrorCount() == 0;
        std::cout << (passed ? "结果: stage4-mission 全链门禁通过"
                             : "结果: stage4-mission 全链门禁未通过")
                  << std::endl;
        return passed;
    }

private:
    // ---- 订阅抽取（状态/意图/设定值） ----

    /// 抽干状态机状态、意图、控制器设定值与估计目标四个订阅，保留验收轨迹。
    void DrainOutputs() {
        while (auto message = status_subscription_.TryTake()) {
            const auto& status = **message;
            warning_bits_seen_ |= status.active_warning_bits;
            if (status.active_warning_bits & kMissionWarningGroundTargetStaleBit) {
                expiry_warning_seen_ = true;
            }
            if (state_sequence_.empty() || state_sequence_.back() != status.state) {
                state_sequence_.push_back(status.state);
            }
            mission_state_ = status.state;
            mission_warning_bits_ = status.active_warning_bits;
        }
        while (auto message = intent_subscription_.TryTake()) {
            intents_.push_back(**message);
            NoteIntent(**message);
        }
        while (auto message = setpoint_subscription_.TryTake()) {
            setpoints_.push_back(**message);
            if ((*message)->valid &&
                (*message)->type == drone::common::SetpointType::kVelocity) {
                ++velocity_sample_count_;
            }
            if ((*message)->valid &&
                (*message)->type == drone::common::SetpointType::kPosition) {
                max_position_norm_m_ =
                    std::max(max_position_norm_m_,
                             std::hypot(static_cast<double>((*message)->x),
                                        static_cast<double>((*message)->y)));
            }
        }
        while (auto message = estimated_subscription_.TryTake()) {
            latest_estimate_ = **message;
            has_estimate_ = true;
        }
    }

    /// 意图轨迹记录（交接验收：交接缝意图无超龄、S7 伺服启动、回退后位置意图恢复）。
    ///
    /// 交接缝判据口径（T12 评审修复，Critical）：**不按 S6 驻留期取样**——lock_frames
    /// 门限 + 20Hz 注入使 S6 只驻留 1 决策拍（S6→S7 只需再推进 1 个新 LOCKED 帧），
    /// S6 标签样本恒 ≤1 条、间隔恒 0，按间隔断言会把行为正确的系统判 FAIL。改为
    /// 跨缝测量：末条 **S5 位置意图** → 首条 **S7 视觉速度意图** 的间隔必须小于状态机
    /// 意图有效期（`controller.intent_timeout`），且该窗口内控制器不得跌入 COAST
    /// （意图超龄的唯一直接后果）——等价于设计 §2.5"交接点不得出现意图超龄→刹车→
    /// 再起步"。S6 期间的冻结续发仅作诊断统计（是否观察到 S6 标签的冻结位置意图）。
    void NoteIntent(const drone::common::ControlIntent& intent) {
        const MissionIntentClass intent_class =
            ClassifyMissionIntent(intent.reason_code, intent.type);
        if (!CountsAsStateMachineIntent(intent_class)) {
            return;  // 工具引导意图不计入状态机轨迹（低字节 0）
        }
        sm_intent_seen_ = true;
        last_sm_intent_ms_ = intent.header.receive_time_ms;
        if (!servo_intent_seen_) {
            // 诊断（不断言）：伺服接管前（S5 引导 + 交接缝）相邻意图的最大间隔——
            // 交接缝判据 FAIL 时据此区分"意图链真断档"（间隔 ≥ 有效期内含 S6/S7
            // 冻结续发缺口）与"仅 S5→S7 过渡耗时偏长"（间隔仍 < 有效期）。
            if (pre_servo_last_intent_ms_ != 0) {
                pre_servo_max_intent_gap_ms_ =
                    std::max(pre_servo_max_intent_gap_ms_,
                             intent.header.receive_time_ms - pre_servo_last_intent_ms_);
            }
            pre_servo_last_intent_ms_ = intent.header.receive_time_ms;
        }
        switch (intent_class) {
            case MissionIntentClass::kPositionGuide:
                if (mission_state_ == common::MissionState::kGpsApproach) {
                    s5_last_position_intent_ms_ = intent.header.receive_time_ms;
                } else if (mission_state_ == common::MissionState::kVisualHandover) {
                    s6_frozen_intent_seen_ = true;  // 诊断：S6 驻留期的冻结续发已观察到
                }
                break;
            case MissionIntentClass::kVisualServo:
                if (!servo_intent_seen_) {
                    servo_intent_seen_ = true;
                    first_servo_intent_ms_ = intent.header.receive_time_ms;
                }
                break;
            case MissionIntentClass::kVisualBrake:
                brake_intent_seen_ = true;
                break;
            case MissionIntentClass::kToolBootstrap:
            case MissionIntentClass::kStateMachineOther:
                break;  // 工具引导已被前置过滤；其它低位码只计入轨迹
        }
        if (backup_position_intent_pending_ &&
            intent.type == common::ControlIntentType::kPositionGuide) {
            backup_position_intent_seen_ = true;
        }
    }

    // ---- 注入：工具引导意图 / 引导 setpoint 流 ----

    /// 工具引导意图（bootstrap/rebootstrap/模式探针/起飞段）：接管前工具是意图
    /// 唯一生产者（此时未注入地面站目标，状态机停在 READY 静默）；状态机意图
    /// 上线后停止注入（bootstrap_intent_active_=false）。
    void PublishBootstrapIntent(uint64_t now_ms) {
        if (!bootstrap_intent_active_ || now_ms < next_bootstrap_intent_ms_) {
            return;
        }
        drone::common::ControlIntent intent;
        intent.header.receive_time_ms = now_ms;
        intent.header.valid_for_ms = kMissionToolIntentValidMs;
        intent.header.frame_id = kMissionFrameLocalNed;
        intent.reason_code = drone::common::kControlIntentReasonRequestControl;
        intent.valid_until_ms = now_ms + kMissionToolIntentValidMs;
        if (phase_ == MissionPhase::kTakeoff && takeoff_target_ready_) {
            intent.type = drone::common::ControlIntentType::kPositionGuide;
            intent.control_source = 0;  // 位置链（对应 S5 远端引导）
            intent.target_x = start_x_;
            intent.target_y = start_y_;
            intent.target_z = takeoff_target_z_;
        } else {
            intent.type = drone::common::ControlIntentType::kVelocityHeading;
            intent.control_source = 1;  // 零速度保持
        }
        (void)intent_topic().Emplace(intent);
        next_bootstrap_intent_ms_ = now_ms + kMissionIntentPeriodMs;
    }

    /// 引导零速度流注入（接管前的 setpoint 流来源，见 stage4 类注释）。
    void InjectGuideSetpoint() {
        drone::common::Px4Setpoint setpoint;
        setpoint.header.receive_time_ms = MonotonicMs();
        setpoint.header.valid_for_ms = 300;
        setpoint.header.frame_id = kMissionFrameLocalNed;
        setpoint.type = drone::common::SetpointType::kVelocity;
        setpoint.valid = true;
        (void)controller_.SetpointOutput().Emplace(setpoint);
        ++guide_injected_;
    }

    // ---- 命令发送 ----

    bool SendOffboardRequest() {
        constexpr uint8_t kPx4MainModeOffboard = 6;
        const bool queued = link_.SendCommand(
            MAV_CMD_DO_SET_MODE, static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            static_cast<float>(kPx4MainModeOffboard), 0.f, 0.f, 0.f, 0.f, 0.f);
        if (queued) {
            offboard_request_time_ms_ = MonotonicMs();
        }
        return queued;
    }

    bool SendArmDisarm(bool arm) {
        const bool queued = link_.SendCommand(MAV_CMD_COMPONENT_ARM_DISARM, arm ? 1.f : 0.f,
                                              0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
        if (queued) {
            if (arm) {
                arm_request_time_ms_ = MonotonicMs();
            } else {
                disarm_request_time_ms_ = MonotonicMs();
            }
        }
        return queued;
    }

    bool SendAutoLand() {
        constexpr uint8_t kPx4MainModeAuto = 4;
        constexpr uint8_t kPx4SubModeAutoLand = 6;
        const bool queued = link_.SendCommand(
            MAV_CMD_DO_SET_MODE, static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            static_cast<float>(kPx4MainModeAuto), static_cast<float>(kPx4SubModeAutoLand),
            0.f, 0.f, 0.f, 0.f);
        if (queued) {
            land_request_time_ms_ = MonotonicMs();
        }
        return queued;
    }

    /// AUTO.LOITER（主模式 4 子模式 3）：G2 补验用"切非 Offboard"目标模式。
    /// 选 LOITER 而非 POSCTL：无遥控器输入时 LOITER 以 GPS 原地保持，安全可回收。
    bool SendAutoLoiter() {
        constexpr uint8_t kPx4MainModeAuto = 4;
        constexpr uint8_t kPx4SubModeAutoLoiter = 3;
        const bool queued = link_.SendCommand(
            MAV_CMD_DO_SET_MODE, static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            static_cast<float>(kPx4MainModeAuto), static_cast<float>(kPx4SubModeAutoLoiter),
            0.f, 0.f, 0.f, 0.f);
        if (queued) {
            mode_probe_request_ms_ = MonotonicMs();
        }
        return queued;
    }

    /// 脚本中断：记录原因并按安全回收处理（FinishFlight 统一 AUTO.LAND）。
    void Abort(const std::string& reason) {
        if (aborted_) {
            return;
        }
        aborted_ = true;
        abort_reason_ = reason;
        std::cout << "[SITL][中断] " << reason << std::endl;
    }

    /// 注入构件的快照视图：未收到快照时为 nullptr（rig 据此暂停注入）。
    const drone::common::FlightStateSnapshot* SnapshotOrNull() const {
        return have_snapshot_ ? &latest_ : nullptr;
    }

    // ---- 阶段推进 ----

    void AdvancePhase(uint64_t now_ms, uint64_t elapsed_ms) {
        switch (phase_) {
            case MissionPhase::kBootstrap:
            case MissionPhase::kRebootstrap:
                AdvanceBootstrap(now_ms, elapsed_ms);
                return;
            case MissionPhase::kModeStopProbe:
                AdvanceModeStopProbe(now_ms);
                return;
            case MissionPhase::kTakeoff:
                AdvanceTakeoff(now_ms);
                return;
            case MissionPhase::kFenceProbe:
                AdvanceFenceProbe(now_ms);
                return;
            case MissionPhase::kApproach:
                AdvanceApproach(now_ms);
                return;
            case MissionPhase::kVisualInject:
                AdvanceVisualInject(now_ms);
                return;
            case MissionPhase::kShortLost:
                AdvanceShortLost(now_ms);
                return;
            case MissionPhase::kSustainedLost:
                AdvanceSustainedLost(now_ms);
                return;
            case MissionPhase::kTargetExpiry:
                AdvanceTargetExpiry(now_ms);
                return;
            case MissionPhase::kLand:
                AdvanceLand(now_ms);
                return;
            case MissionPhase::kDone:
                return;
        }
    }

    /// 引导/接管段（两轮共用）：第一轮连接 + ARM；第二轮跳过 ARM（已 armed），
    /// 由引导流 + Offboard 命令重新接管。
    void AdvanceBootstrap(uint64_t now_ms, uint64_t elapsed_ms) {
        const bool rebootstrap = phase_ == MissionPhase::kRebootstrap;
        if (!link_.IsConnected()) {
            if (elapsed_ms >= kMissionConnectTimeoutMs) {
                Abort("连接超时：未收到 PX4 心跳（SITL 未启动或 UDP 配置不符）");
            }
            return;
        }
        if (bootstrap_start_ms_ == 0) {
            bootstrap_start_ms_ = now_ms;
        }
        if (!guide_started_) {
            guide_started_ = true;
            guide_start_ms_ = now_ms;
            guide_round_start_count_ = guide_injected_;
            next_guide_publish_ms_ = now_ms;
            std::cout << "[SITL] 引导阶段开始"
                      << (rebootstrap ? "（重入 Offboard）" : "") << "：注入零速度引导流"
                      << std::endl;
        }
        if (!rebootstrap && !handover_done_ && now_ms >= next_guide_publish_ms_) {
            InjectGuideSetpoint();
            next_guide_publish_ms_ = now_ms + kMissionGuidePeriodMs;
        }
        if (rebootstrap && !handover_done_ && now_ms >= next_guide_publish_ms_) {
            InjectGuideSetpoint();
            next_guide_publish_ms_ = now_ms + kMissionGuidePeriodMs;
        }
        // stage3A 纪律：引导流满足"≥2 秒且 ≥20 条"后才请求 Offboard（按轮计数）
        if (!offboard_command_queued_ &&
            guide_injected_ - guide_round_start_count_ >= kMissionGuideMinSends &&
            now_ms - guide_start_ms_ >= kMissionGuideMinMs) {
            offboard_command_queued_ = SendOffboardRequest();
            std::cout << "[SITL] Offboard 模式请求"
                      << (offboard_command_queued_ ? "已入队" : "入队失败") << std::endl;
        }
        if (have_snapshot_) {
            if (offboard_command_queued_ && !offboard_ack_received_ &&
                latest_.last_ack_time_ms >= offboard_request_time_ms_ &&
                latest_.last_ack_command == MAV_CMD_DO_SET_MODE) {
                offboard_ack_received_ = true;
                offboard_ack_accepted_ = latest_.last_ack_result == MAV_RESULT_ACCEPTED;
            }
            offboard_mode_reached_ =
                offboard_mode_reached_ ||
                (offboard_command_queued_ && latest_.flight_mode == 6 &&
                 latest_.flight_sub_mode == 0);
            if (latest_.flight_mode == 6 && latest_.flight_sub_mode == 0) {
                out_of_offboard_ = false;  // 已回到 Offboard（模式探针段收场）
            }
            if (arm_command_queued_ && !arm_ack_received_ &&
                latest_.last_ack_time_ms >= arm_request_time_ms_ &&
                latest_.last_ack_command == MAV_CMD_COMPONENT_ARM_DISARM) {
                arm_ack_received_ = true;
                arm_ack_accepted_ = latest_.last_ack_result == MAV_RESULT_ACCEPTED;
            }
            // stage3A 前置条件：Offboard ACK accepted + 真实 6/0 + landed +
            // 位置/姿态有效，才允许 ARM（第二轮已 armed，本块自动跳过）
            if (!arm_command_queued_ && offboard_ack_accepted_ && offboard_mode_reached_ &&
                latest_.connected && !latest_.armed && latest_.landed_state_valid &&
                latest_.landed && latest_.local_position_valid && latest_.attitude_valid) {
                arm_command_queued_ = SendArmDisarm(true);
                std::cout << "[SITL] ARM 请求" << (arm_command_queued_ ? "已入队" : "入队失败")
                          << std::endl;
            }
            if (latest_.armed) {
                armed_reached_ = true;
            }
        }
        if (armed_reached_) {
            if (!handover_done_ &&
                controller_.Authority() == drone::control::ControlAuthority::kActive) {
                handover_done_ = true;
                handover_ms_ = now_ms;
                send_count_at_handover_ = link_.SetpointSendCount();
                std::cout << "[SITL] 控制器已接管设定值流（授权链全通：armed+Offboard+"
                             "请求位）"
                          << (rebootstrap ? "（第二轮）" : "") << std::endl;
            }
            if (handover_done_ && !takeover_stream_ok_ &&
                link_.SetpointSendCount() > send_count_at_handover_) {
                takeover_stream_ok_ = true;  // 停注后链路发送仍在增长=控制器在供流
            }
            if (takeover_stream_ok_ && now_ms - handover_ms_ >= 500) {
                if (!rebootstrap) {
                    std::cout << "[SITL] 第一轮接管完成，进入切非 Offboard 停发补验段"
                              << std::endl;
                    phase_ = MissionPhase::kModeStopProbe;
                    probe_start_ms_ = now_ms;
                    return;
                }
                rebootstrap_takeover_seen_ = true;
                std::cout << "[SITL] 第二轮接管完成，进入工具位置引导起飞段" << std::endl;
                phase_ = MissionPhase::kTakeoff;
                takeoff_start_ms_ = now_ms;
                takeoff_started_ = true;
                return;
            }
        }
        const uint64_t segment_start_ms =
            rebootstrap ? rebootstrap_start_ms_ : bootstrap_start_ms_;
        if (segment_start_ms != 0 &&
            now_ms - segment_start_ms >= kMissionBootstrapTimeoutMs) {
            Abort(rebootstrap
                      ? "重入 Offboard 接管超时（检查引导流/模式 ACK/授权链快照）"
                      : "引导/Offboard/ARM/接管阶段超时（检查 ACK、模式与授权链）");
        }
    }

    /// [G2 补验] 切非 Offboard：飞行/意图活跃中命令 AUTO.LOITER，核验控制器
    /// 在首个非 Offboard 快照后 ≤200ms（设计 §4）停止发布，并核验发送冻结。
    void AdvanceModeStopProbe(uint64_t now_ms) {
        if (!mode_probe_command_queued_) {
            mode_probe_command_queued_ = SendAutoLoiter();
            std::cout << "[SITL] 切非 Offboard(AUTO.LOITER) 请求"
                      << (mode_probe_command_queued_ ? "已入队" : "入队失败") << std::endl;
        }
        if (mode_probe_command_queued_ && !mode_change_seen_ && have_snapshot_ &&
            latest_.flight_mode != 6 && !out_of_offboard_) {
            mode_change_seen_ = true;
            mode_change_ms_ = now_ms;
            out_of_offboard_ = true;
            sends_at_mode_change_ = link_.SetpointSendCount();
        }
        if (mode_change_seen_ && !mode_stop_seen_ &&
            controller_.Authority() != drone::control::ControlAuthority::kActive) {
            mode_stop_seen_ = true;
            mode_stop_ms_ = now_ms;
        }
        if (mode_stop_seen_) {
            // 停发后观察一小段，核验设定值发送冻结（残留队列竞态允许 ≤2 条）
            if (now_ms - mode_stop_ms_ < kMissionModeStopSettleMs) {
                return;
            }
            sends_after_mode_stop_ = link_.SetpointSendCount() > sends_at_mode_change_
                ? link_.SetpointSendCount() - sends_at_mode_change_
                : 0;
            std::cout << "[SITL] 控制器已停发：首见非 Offboard→停发 "
                      << mode_stop_ms_ - mode_change_ms_ << " ms（上限 "
                      << kMissionModeStopLimitMs << "ms），停发后额外发送 "
                      << sends_after_mode_stop_ << " 条" << std::endl;
            // 复位引导/接管标志，进入第二轮（重入 Offboard）
            guide_started_ = false;
            offboard_command_queued_ = false;
            offboard_ack_received_ = false;
            offboard_ack_accepted_ = false;
            offboard_mode_reached_ = false;
            handover_done_ = false;
            takeover_stream_ok_ = false;
            phase_ = MissionPhase::kRebootstrap;
            rebootstrap_start_ms_ = now_ms;
            return;
        }
        if (now_ms - probe_start_ms_ >= kMissionProbeTimeoutMs) {
            Abort("切非 Offboard 后控制器未在限时内停发");
        }
    }

    /// 工具位置引导意图起飞至相对 1m 并稳定悬停（与 stage4 起飞段同窗口）。
    void AdvanceTakeoff(uint64_t now_ms) {
        if (!takeoff_target_ready_ || !latest_.local_position_valid) {
            if (now_ms - takeoff_start_ms_ >= kMissionTakeoffTimeoutMs) {
                Abort("起飞超时：位置基准缺失或位置遥测无效");
            }
            return;
        }
        const float ascent_m = start_z_ - latest_.local_z_m;
        const float dx = latest_.local_x_m - start_x_;
        const float dy = latest_.local_y_m - start_y_;
        const float drift_m = std::sqrt(dx * dx + dy * dy);
        if (ascent_m > kMissionMaxHeightM ||
            drift_m > kMissionHoverDriftM + kMissionFenceToleranceM) {
            Abort("起飞段高度/漂移超包线");
            return;
        }
        const bool inside_window =
            std::fabs(latest_.local_z_m - takeoff_target_z_) <= kMissionHoverZErrorM &&
            drift_m <= kMissionHoverDriftM && std::fabs(latest_.vz_mps) <= kMissionHoverVzMps;
        if (inside_window) {
            if (hover_stable_since_ms_ == 0) {
                hover_stable_since_ms_ = now_ms;
            }
            hover_complete_ = now_ms - hover_stable_since_ms_ >= kMissionHoverStableMs;
        } else {
            hover_stable_since_ms_ = 0;
        }
        if (hover_complete_) {
            std::cout << "[SITL] 悬停稳定完成，注入围栏外目标（北向 "
                      << kMissionFenceProbeNorthM << "m > max_radius "
                      << controller_config_.max_radius_m << "m）" << std::endl;
            rig_.SetTargetNed(kMissionFenceProbeNorthM, 0.0, kMissionTargetUpM);
            fence_probe_clamped_at_ = controller_.ClampedCount();
            phase_ = MissionPhase::kFenceProbe;
            phase_start_ms_ = now_ms;
            return;
        }
        if (now_ms - takeoff_start_ms_ >= kMissionTakeoffTimeoutMs) {
            Abort("起飞/悬停超时（检查 home_valid 与位置意图是否被控制拒绝）");
        }
    }

    /// 注入围栏外目标：位置引导意图指向 150m，控制器应钳制到 max_radius。
    /// 验收证据：ClampedCount 增长 + 实测位置设定值水平模长 ≤ 围栏（+容差）。
    void AdvanceFenceProbe(uint64_t now_ms) {
        (void)rig_.PublishGroundTarget(kMissionTargetIdProbe, now_ms, SnapshotOrNull());
        if (mission_state_ == common::MissionState::kGpsApproach && sm_intent_seen_ &&
            !bootstrap_intent_active_stopped_) {
            bootstrap_intent_active_ = false;  // 状态机意图已上线：工具停止注入意图
            bootstrap_intent_active_stopped_ = true;
            std::cout << "[SITL] 状态机已接管意图流（S5 位置引导），工具停止注入引导意图"
                      << std::endl;
        }
        if (mission_state_ == common::MissionState::kGpsApproach &&
            controller_.ClampedCount() > fence_probe_clamped_at_) {
            fence_clamp_seen_ = true;
        }
        if (fence_clamp_seen_ && sm_intent_seen_ &&
            now_ms - phase_start_ms_ >= 2000) {
            std::cout << "[SITL] 围栏钳制证据成立：ClampedCount +"
                      << controller_.ClampedCount() - fence_probe_clamped_at_
                      << "，最大位置设定值水平模长 " << max_position_norm_m_ << " m（围栏 "
                      << controller_config_.max_radius_m << " m）；换注入近距目标（北向 "
                      << kMissionTargetNorthM << "m）" << std::endl;
            rig_.SetTargetNed(kMissionTargetNorthM, 0.0, kMissionTargetUpM);
            phase_ = MissionPhase::kApproach;
            phase_start_ms_ = now_ms;
            return;
        }
        if (now_ms - phase_start_ms_ >= kMissionProbeTimeoutMs) {
            Abort("围栏探针段超时：未观测到状态机位置意图或围栏钳制");
        }
    }

    /// 注入近距目标：S5 位置引导（状态机唯一意图生产者）。
    void AdvanceApproach(uint64_t now_ms) {
        (void)rig_.PublishGroundTarget(kMissionTargetIdMain, now_ms, SnapshotOrNull());
        if (mission_state_ == common::MissionState::kGpsApproach && sm_intent_seen_ &&
            now_ms - phase_start_ms_ >= 2000) {
            std::cout << "[SITL] S5 位置引导稳定，开始注入模拟视觉观测"
                         "（SEARCHING → LOCKED）"
                      << std::endl;
            rig_.SetVisionEnabled(true);
            vision_search_until_ms_ = now_ms + kMissionVisualSearchMs;
            phase_ = MissionPhase::kVisualInject;
            phase_start_ms_ = now_ms;
            return;
        }
        if (now_ms - phase_start_ms_ >= kMissionProbeTimeoutMs) {
            Abort("S5 位置引导段超时：状态机未发布位置引导意图");
        }
    }

    /// 注入 SEARCHING → LOCKED 帧序列：S5→S6（冻结位置意图续发）→ S7（视觉伺服）。
    void AdvanceVisualInject(uint64_t now_ms) {
        (void)rig_.PublishGroundTarget(kMissionTargetIdMain, now_ms, SnapshotOrNull());
        if (now_ms < vision_search_until_ms_) {
            (void)rig_.PublishSimulatedVision(common::VisualTargetState::kSearching,
                                              now_ms, SnapshotOrNull());
            return;
        }
        if (vision_lock_started_ms_ == 0) {
            vision_lock_started_ms_ = now_ms;
            std::cout << "[SITL] 注入 LOCKED 帧序列（lock_frames=" << handover_.lock_frames
                      << "，exit_lost_frames=" << handover_.exit_lost_frames
                      << "，创新门限 " << handover_.innovation_gate_m << "m）" << std::endl;
        }
        (void)rig_.PublishSimulatedVision(common::VisualTargetState::kLocked, now_ms,
                                          SnapshotOrNull());
        if (mission_state_ == common::MissionState::kVisualTracking && servo_intent_seen_) {
            // 交接缝判据在视觉伺服接管时刻定格（设计 §2.5 无扰衔接）：
            // 末条 S5 位置意图 → 首条 S7 速度意图 < 意图有效期，且窗口内未跌入 COAST。
            if (first_servo_intent_ms_ != 0 && s5_last_position_intent_ms_ != 0 &&
                first_servo_intent_ms_ >= s5_last_position_intent_ms_) {
                handover_seam_gap_ms_ =
                    first_servo_intent_ms_ - s5_last_position_intent_ms_;
            }
            handover_seam_ok_ = handover_seam_gap_ms_ > 0 &&
                                handover_seam_gap_ms_ < intent_valid_ms_ &&
                                !coast_during_seam_;
            std::cout << "[SITL] S6→S7 完成：交接缝间隔 " << handover_seam_gap_ms_
                      << " ms（上限 " << intent_valid_ms_ << " ms，窗口内 COAST="
                      << YesNo(coast_during_seam_) << "；伺服接管前意图最大间隔 "
                      << pre_servo_max_intent_gap_ms_ << " ms；S6 冻结续发意图观察="
                      << YesNo(s6_frozen_intent_seen_) << "），视觉伺服意图已启动，"
                      << "进入短期 LOST 注入段" << std::endl;
            short_lost_frames_ = 0;
            phase_ = MissionPhase::kShortLost;
            phase_start_ms_ = now_ms;
            return;
        }
        if (now_ms - phase_start_ms_ >= kMissionProbeTimeoutMs) {
            Abort("交接段超时：未进入 S7 或伺服意图未启动（查创新/测距/姿态与门限）");
        }
    }

    /// 短期 LOST（< exit_lost_frames）：保持 S7，输出判丢零速刹车意图。
    void AdvanceShortLost(uint64_t now_ms) {
        (void)rig_.PublishGroundTarget(kMissionTargetIdMain, now_ms, SnapshotOrNull());
        if (short_lost_frames_ < kMissionShortLostFrames) {
            if (rig_.PublishSimulatedVision(common::VisualTargetState::kLost, now_ms,
                                            SnapshotOrNull())) {
                ++short_lost_frames_;
            }
            return;
        }
        short_lost_kept_s7_ = mission_state_ == common::MissionState::kVisualTracking;
        std::cout << "[SITL] 短期 LOST " << kMissionShortLostFrames << " 帧注入完成：状态="
                  << static_cast<int>(mission_state_) << "（期望 S7 保持），刹车意图="
                  << YesNo(brake_intent_seen_) << std::endl;
        sustained_lost_frames_ = 0;
        phase_ = MissionPhase::kSustainedLost;
        phase_start_ms_ = now_ms;
    }

    /// 持续 LOST（达 exit_lost_frames）：回退 S5，卡尔曼备份接管位置引导。
    void AdvanceSustainedLost(uint64_t now_ms) {
        (void)rig_.PublishGroundTarget(kMissionTargetIdMain, now_ms, SnapshotOrNull());
        if (mission_state_ == common::MissionState::kVisualTracking) {
            if (sustained_lost_frames_ <= handover_.exit_lost_frames + 5) {
                if (rig_.PublishSimulatedVision(common::VisualTargetState::kLost, now_ms,
                                                SnapshotOrNull())) {
                    ++sustained_lost_frames_;
                }
            }
            return;
        }
        // 已回退（S5）：确认卡尔曼备份位置意图恢复（估计仍新鲜）
        if (mission_state_ == common::MissionState::kGpsApproach) {
            if (!fallback_seen_) {
                fallback_seen_ = true;
                backup_position_intent_pending_ = true;
                std::cout << "[SITL] 持续 LOST 回退 S5（控制权不中断），等待卡尔曼备份接管"
                          << std::endl;
            }
            if (backup_position_intent_seen_ || now_ms - phase_start_ms_ >= 3000) {
                rig_.SetVisionEnabled(false);  // 视觉链随目标一起收场
                std::cout << "[SITL] 卡尔曼备份位置意图恢复："
                          << YesNo(backup_position_intent_seen_) << "；停止注入目标（过期）"
                          << std::endl;
                rig_.SetTargetEnabled(false);
                expiry_stop_ms_ = now_ms;
                expect_offboard_exit_ = true;  // 停发链的 Offboard 退出属预期
                phase_ = MissionPhase::kTargetExpiry;
                phase_start_ms_ = now_ms;
            }
            return;
        }
        if (now_ms - phase_start_ms_ >= kMissionProbeTimeoutMs) {
            Abort("持续 LOST 段超时：状态机未回退 S5");
        }
    }

    /// 目标过期：位置意图停发 → COAST 刹车 → IDLE → PX4 退出 Offboard。
    void AdvanceTargetExpiry(uint64_t now_ms) {
        if (!coast_seen_ &&
            controller_.Authority() == drone::control::ControlAuthority::kCoast) {
            coast_seen_ = true;
            coast_ms_ = now_ms;
            std::cout << "[SITL] 目标过期后意图停发，控制器进入 COAST 零速刹车" << std::endl;
        }
        if (!idle_seen_ &&
            controller_.Authority() == drone::control::ControlAuthority::kIdle) {
            idle_seen_ = true;
            idle_ms_ = now_ms;
            sends_at_idle_ = link_.SetpointSendCount();
            std::cout << "[SITL] 控制器转 IDLE 停发（COAST 超时），等待 PX4 退出 Offboard"
                      << std::endl;
        }
        // 意图停发：距最后一条状态机意图超过意图有效期（valid_for_ms）仍无新意图。
        expiry_intent_stop_seen_ =
            expiry_intent_stop_seen_ ||
            (last_sm_intent_ms_ != 0 && now_ms - last_sm_intent_ms_ > intent_valid_ms_);
        if (idle_seen_ && !offboard_exit_seen_ && have_snapshot_ &&
            latest_.flight_mode != 6) {
            offboard_exit_seen_ = true;
            offboard_exit_ms_ = now_ms;
            offboard_exit_mode_ = latest_.flight_mode;
            offboard_exit_sub_mode_ = latest_.flight_sub_mode;
            std::cout << "[SITL] PX4 已退出 Offboard: mode="
                      << static_cast<int>(offboard_exit_mode_) << '/'
                      << static_cast<int>(offboard_exit_sub_mode_) << "（IDLE 后 "
                      << offboard_exit_ms_ - idle_ms_ << " ms）" << std::endl;
            land_command_queued_ = SendAutoLand();
            std::cout << "[SITL] AUTO.LAND 模式请求"
                      << (land_command_queued_ ? "已入队" : "入队失败") << std::endl;
            phase_ = MissionPhase::kLand;
            return;
        }
        if (now_ms - phase_start_ms_ >= kMissionExpiryTimeoutMs) {
            Abort("目标过期停发链未在限时内完成（COAST/IDLE/退出 Offboard）");
        }
    }

    void AdvanceLand(uint64_t now_ms) {
        if (have_snapshot_ && land_command_queued_ && !land_ack_received_ &&
            latest_.last_ack_time_ms >= land_request_time_ms_ &&
            latest_.last_ack_command == MAV_CMD_DO_SET_MODE) {
            land_ack_received_ = true;
            land_ack_accepted_ = latest_.last_ack_result == MAV_RESULT_ACCEPTED;
        }
        if (have_snapshot_ && land_ack_accepted_ && latest_.landed_state_valid &&
            latest_.landed) {
            landed_after_flight_ = true;
            std::cout << "[SITL] PX4 已确认降落，准备主动 DISARM" << std::endl;
            phase_ = MissionPhase::kDone;
            return;
        }
        if (now_ms - land_request_time_ms_ >= kMissionLandTimeoutMs) {
            Abort("AUTO.LAND 后未确认 landed");
        }
    }

    // ---- 观测与安全 ----

    void PrintStateChange() {
        if (latest_.connected == last_connected_ && latest_.armed == last_armed_ &&
            latest_.flight_mode == last_mode_ &&
            latest_.flight_sub_mode == last_sub_mode_) {
            return;
        }
        std::cout << "[状态变化] connected=" << YesNo(latest_.connected)
                  << " armed=" << YesNo(latest_.armed)
                  << " mode=" << static_cast<int>(latest_.flight_mode) << '/'
                  << static_cast<int>(latest_.flight_sub_mode) << std::endl;
        last_connected_ = latest_.connected;
        last_armed_ = latest_.armed;
        last_mode_ = latest_.flight_mode;
        last_sub_mode_ = latest_.flight_sub_mode;
    }

    /// 每秒打印任务摘要（状态机状态 + 告警位 + 估计目标 + 注入状态）。
    void PrintMissionSummary() const {
        std::cout << "  状态机: state=" << static_cast<int>(mission_state_)
                  << " warnings=0x" << std::hex << mission_warning_bits_ << std::dec
                  << " 注入目标北向=" << rig_.target_north_m()
                  << " 视觉=" << (rig_.vision_enabled() ? "注入中" : "停") << " 卡尔曼="
                  << (has_estimate_ ? (latest_estimate_.valid ? "有效" : "失效")
                                    : "未初始化");
        if (has_estimate_) {
            std::cout << " 估计(N/E/D)=(" << latest_estimate_.pos_x_m << ","
                      << latest_estimate_.pos_y_m << "," << latest_estimate_.pos_z_m
                      << ")";
        }
        std::cout << " 控制权=" << AuthorityText(controller_.Authority()) << std::endl;
    }

    /// 飞行安全包线：armed 期间连接/位置/姿态异常或越界立即 AUTO.LAND 回收。
    void CheckFlightEnvelope() {
        if (armed_reached_ && !latest_.armed && !disarm_command_queued_) {
            unexpected_disarm_before_command_ = true;
        }
        if (!latest_.armed || land_command_queued_ || safety_violation_) {
            return;
        }
        if (!latest_.connected || !latest_.local_position_valid ||
            !latest_.attitude_valid) {
            safety_violation_ = true;
            std::cout << "[SITL][安全回收] 连接、位置或姿态失效" << std::endl;
            return;
        }
        // 非预期退出 Offboard（模式探针段主动切出、目标过期停发链属预期，其余异常）
        if (!out_of_offboard_ && !expect_offboard_exit_ && latest_.flight_mode != 6) {
            safety_violation_ = true;
            std::cout << "[SITL][安全回收] 非预期退出 Offboard（mode="
                      << static_cast<int>(latest_.flight_mode) << "）" << std::endl;
            return;
        }
        if (!takeoff_target_ready_) {
            return;
        }
        const float ascent_m = start_z_ - latest_.local_z_m;
        const float north_m = latest_.local_x_m - start_x_;
        const float east_m = std::fabs(latest_.local_y_m - start_y_);
        max_ascent_m_ = std::max(max_ascent_m_, ascent_m);
        max_north_m_ = std::max(max_north_m_, north_m);
        max_east_m_ = std::max(max_east_m_, east_m);
        const double radius = std::hypot(static_cast<double>(latest_.local_x_m),
                                         static_cast<double>(latest_.local_y_m));
        // 高度下限只对升空后（起飞完成后的引导/交接/停发段）生效：起飞前
        // 机体在地面（相对高度 0），不得按"低于下限"误判安全回收。
        const bool airborne = phase_ == MissionPhase::kFenceProbe ||
                              phase_ == MissionPhase::kApproach ||
                              phase_ == MissionPhase::kVisualInject ||
                              phase_ == MissionPhase::kShortLost ||
                              phase_ == MissionPhase::kSustainedLost ||
                              phase_ == MissionPhase::kTargetExpiry;
        const bool height_out =
            ascent_m > kMissionMaxHeightM || (airborne && ascent_m < kMissionMinHeightM);
        const bool horizontal_out =
            north_m > kMissionNorthMarginM + static_cast<float>(kMissionTargetNorthM) ||
            east_m > kMissionMaxEastM ||
            radius > controller_config_.max_radius_m + kMissionFenceToleranceM;
        if (height_out || horizontal_out) {
            safety_violation_ = true;
            std::cout << "[SITL][安全回收] 高度/水平位移超包线: 相对高度 " << ascent_m
                      << " m，北向 " << north_m << " m，东西 " << east_m
                      << " m，半径 " << radius << " m" << std::endl;
        }
    }

    /// 交接/回退段（S5 位置引导→S7 伺服→回退 S5）控制权必须连续保持 ACTIVE。
    /// 另单独记录**交接缝窗口内**（伺服启动前的引导/交接段）是否跌入 COAST——
    /// 该窗口内 COAST 只可能由意图超龄触发，是"交接点意图断档"的直接证据。
    void CheckAuthorityContinuity() {
        if (phase_ == MissionPhase::kFenceProbe || phase_ == MissionPhase::kApproach ||
            phase_ == MissionPhase::kVisualInject ||
            phase_ == MissionPhase::kShortLost || phase_ == MissionPhase::kSustainedLost) {
            if (controller_.Authority() != drone::control::ControlAuthority::kActive) {
                authority_uninterrupted_ = false;
            }
            if (!servo_intent_seen_ &&
                (phase_ == MissionPhase::kApproach ||
                 phase_ == MissionPhase::kVisualInject) &&
                controller_.Authority() == drone::control::ControlAuthority::kCoast) {
                coast_during_seam_ = true;
            }
        }
    }

    /// 交接/回退全程速度式设定值的最大隐含加速度（输出连续性验收；段首按
    /// "位置式→速度式从 0 起步"守卫核验，见 FlightController 无扰衔接）。
    double MaxObservedSlewMps2() const {
        double max_slew = 0.0;
        bool have_prev_velocity = false;
        uint64_t prev_velocity_ms = 0;
        float prev_vx = 0.f;
        float prev_vy = 0.f;
        float prev_vz = 0.f;
        uint64_t last_sample_ms = 0;
        for (const auto& setpoint : setpoints_) {
            if (setpoint.header.receive_time_ms == 0) {
                continue;
            }
            if (!setpoint.valid ||
                setpoint.type != drone::common::SetpointType::kVelocity) {
                have_prev_velocity = false;
                last_sample_ms = setpoint.header.receive_time_ms;
                continue;
            }
            const uint64_t sample_ms = setpoint.header.receive_time_ms;
            if (have_prev_velocity && sample_ms > prev_velocity_ms) {
                const double dt = static_cast<double>(sample_ms - prev_velocity_ms) / 1000.0;
                const double dv = std::sqrt(Squared(setpoint.x - prev_vx) +
                                            Squared(setpoint.y - prev_vy) +
                                            Squared(setpoint.z - prev_vz));
                max_slew = std::max(max_slew, dv / dt);
            } else if (!have_prev_velocity && sample_ms > last_sample_ms) {
                const double dt = static_cast<double>(sample_ms - last_sample_ms) / 1000.0;
                const double dv = std::sqrt(Squared(setpoint.x) + Squared(setpoint.y) +
                                            Squared(setpoint.z));
                max_slew = std::max(max_slew, dv / dt);
            }
            have_prev_velocity = true;
            prev_velocity_ms = sample_ms;
            prev_vx = setpoint.x;
            prev_vy = setpoint.y;
            prev_vz = setpoint.z;
            last_sample_ms = sample_ms;
        }
        return max_slew;
    }

    static double Squared(float value) {
        return static_cast<double>(value) * static_cast<double>(value);
    }

    /// 状态序列包含给定子序列（按出现顺序）。
    bool SequenceContains(const std::vector<common::MissionState>& expected) const {
        std::size_t index = 0;
        for (const auto state : state_sequence_) {
            if (index < expected.size() && state == expected[index]) {
                ++index;
            }
        }
        return index == expected.size();
    }

    std::string StateSequenceText() const {
        std::string text;
        for (const auto state : state_sequence_) {
            if (!text.empty()) {
                text += " → ";
            }
            text += std::to_string(static_cast<int>(state));
        }
        return text.empty() ? "（无）" : text;
    }

    static const char* AuthorityText(drone::control::ControlAuthority authority) {
        switch (authority) {
            case drone::control::ControlAuthority::kIdle:
                return "IDLE";
            case drone::control::ControlAuthority::kActive:
                return "ACTIVE";
            case drone::control::ControlAuthority::kCoast:
                return "COAST";
        }
        return "未知";
    }

    /// 收尾：未落地先强制 AUTO.LAND 回收（禁止空中 DISARM），确认 landed 后主动 DISARM。
    void FinishFlight() {
        const bool armed_now = armed_reached_ || latest_.armed;
        if (armed_now && !landed_after_flight_) {
            if (!land_command_queued_) {
                land_command_queued_ = SendAutoLand();
                std::cout << "[SITL] 异常回收 AUTO.LAND 请求"
                          << (land_command_queued_ ? "已入队" : "入队失败") << std::endl;
            }
            const uint64_t deadline = MonotonicMs() + kMissionLandTimeoutMs;
            while (MonotonicMs() < deadline) {
                if (auto message =
                        subscription_.WaitTakeFor(std::chrono::milliseconds(100))) {
                    latest_ = **message;
                    UpdateObserved(latest_, &observed_);
                    if (land_command_queued_ && !land_ack_received_ &&
                        latest_.last_ack_time_ms >= land_request_time_ms_ &&
                        latest_.last_ack_command == MAV_CMD_DO_SET_MODE) {
                        land_ack_received_ = true;
                        land_ack_accepted_ = latest_.last_ack_result == MAV_RESULT_ACCEPTED;
                    }
                    if (latest_.landed_state_valid && latest_.landed) {
                        landed_after_flight_ = true;
                        break;
                    }
                }
            }
            std::cout << "[SITL] 回收结果: landed=" << YesNo(landed_after_flight_)
                      << std::endl;
        }
        if (armed_now && landed_after_flight_) {
            while (subscription_.TryTake()) {
                // 清除排队旧快照，避免用解锁前 armed=false 误判上锁完成。
            }
            disarm_command_queued_ = SendArmDisarm(false);
            std::cout << "[SITL] DISARM 请求"
                      << (disarm_command_queued_ ? "已入队" : "入队失败") << std::endl;
            const uint64_t deadline = MonotonicMs() + 5000;
            while (MonotonicMs() < deadline) {
                if (auto message =
                        subscription_.WaitTakeFor(std::chrono::milliseconds(100))) {
                    latest_ = **message;
                    UpdateObserved(latest_, &observed_);
                    if (disarm_command_queued_ &&
                        latest_.last_ack_time_ms >= disarm_request_time_ms_ &&
                        latest_.last_ack_command == MAV_CMD_COMPONENT_ARM_DISARM) {
                        disarm_ack_received_ = true;
                        disarm_ack_accepted_ =
                            latest_.last_ack_result == MAV_RESULT_ACCEPTED;
                    }
                    if (disarm_ack_accepted_ && !latest_.armed) {
                        disarmed_confirmed_ = true;
                        break;
                    }
                }
            }
        }
    }

    drone::common::Topic<drone::common::ControlIntent>& intent_topic() {
        return machine_.IntentOutput();
    }

    // ---- 依赖 ----
    drone::communication::Px4Link& link_;
    drone::control::FlightController& controller_;
    drone::state_machine::MissionStateMachine& machine_;
    Stage4InjectionRig rig_;  // 三路注入构件（目标/视觉/测距，几何互逆单一真源）
    drone::common::Topic<drone::common::TargetState>& estimated_topic_;
    const drone::config::FlightControllerConfig& controller_config_;
    const drone::config::HandoverConfig& handover_;
    uint64_t intent_valid_ms_ = 0;  // 状态机意图有效期（= controller.intent_timeout）
    uint64_t duration_ms_ = 0;
    drone::common::Topic<drone::common::FlightStateSnapshot>::Subscription subscription_;
    drone::common::Topic<drone::common::MissionStatus>::Subscription status_subscription_;
    drone::common::Topic<drone::common::ControlIntent>::Subscription intent_subscription_;
    drone::common::Topic<drone::common::Px4Setpoint>::Subscription setpoint_subscription_;
    drone::common::Topic<drone::common::TargetState>::Subscription estimated_subscription_;

    // ---- 观测 ----
    drone::common::FlightStateSnapshot latest_;
    bool have_snapshot_ = false;
    ObservedState observed_;
    bool last_connected_ = false;
    bool last_armed_ = false;
    uint8_t last_mode_ = 0;
    uint8_t last_sub_mode_ = 0;
    std::vector<drone::common::ControlIntent> intents_;
    std::vector<drone::common::Px4Setpoint> setpoints_;
    std::vector<common::MissionState> state_sequence_;
    common::MissionState mission_state_ = common::MissionState::kBoot;
    uint32_t mission_warning_bits_ = 0;
    uint32_t warning_bits_seen_ = 0;

    // ---- 脚本推进 ----
    MissionPhase phase_ = MissionPhase::kBootstrap;
    bool aborted_ = false;
    std::string abort_reason_;
    uint64_t phase_start_ms_ = 0;
    uint64_t probe_start_ms_ = 0;
    bool bootstrap_intent_active_ = true;
    bool bootstrap_intent_active_stopped_ = false;
    uint64_t next_bootstrap_intent_ms_ = 0;
    uint64_t next_guide_publish_ms_ = 0;
    bool guide_started_ = false;
    uint64_t guide_start_ms_ = 0;
    uint64_t guide_injected_ = 0;
    uint64_t vision_search_until_ms_ = 0;
    uint64_t vision_lock_started_ms_ = 0;
    bool has_estimate_ = false;
    drone::common::TargetState latest_estimate_;

    // 引导/Offboard/ARM/接管
    bool offboard_command_queued_ = false;
    bool offboard_ack_received_ = false;
    bool offboard_ack_accepted_ = false;
    bool offboard_mode_reached_ = false;
    uint64_t offboard_request_time_ms_ = 0;
    bool arm_command_queued_ = false;
    bool arm_ack_received_ = false;
    bool arm_ack_accepted_ = false;
    bool armed_reached_ = false;
    uint64_t arm_request_time_ms_ = 0;
    bool handover_done_ = false;
    bool takeover_stream_ok_ = false;
    uint64_t handover_ms_ = 0;
    uint64_t send_count_at_handover_ = 0;
    bool rebootstrap_takeover_seen_ = false;
    uint64_t bootstrap_start_ms_ = 0;
    uint64_t rebootstrap_start_ms_ = 0;
    uint64_t guide_round_start_count_ = 0;
    bool expect_offboard_exit_ = false;

    // 模式停发补验
    bool mode_probe_command_queued_ = false;
    uint64_t mode_probe_request_ms_ = 0;
    bool mode_change_seen_ = false;
    uint64_t mode_change_ms_ = 0;
    bool mode_stop_seen_ = false;
    uint64_t mode_stop_ms_ = 0;
    uint64_t sends_at_mode_change_ = 0;
    uint64_t sends_after_mode_stop_ = 0;
    bool out_of_offboard_ = false;

    // 起飞/悬停
    bool takeoff_target_ready_ = false;
    float start_x_ = 0.f;
    float start_y_ = 0.f;
    float start_z_ = 0.f;
    float takeoff_target_z_ = 0.f;
    bool takeoff_started_ = false;
    uint64_t takeoff_start_ms_ = 0;
    bool hover_complete_ = false;
    uint64_t hover_stable_since_ms_ = 0;

    // 围栏/交接/回退
    uint64_t fence_probe_clamped_at_ = 0;
    bool fence_clamp_seen_ = false;
    double max_position_norm_m_ = 0.0;  // 位置设定值水平模长峰值（围栏钳制验收）
    uint64_t velocity_sample_count_ = 0;  // 速度式设定值样本数（连续性判据非空前提）
    bool sm_intent_seen_ = false;
    bool servo_intent_seen_ = false;
    bool brake_intent_seen_ = false;
    uint64_t s5_last_position_intent_ms_ = 0;  ///< 末条 S5 位置引导意图时刻
    uint64_t first_servo_intent_ms_ = 0;       ///< 首条 S7 视觉速度意图时刻
    uint64_t handover_seam_gap_ms_ = 0;        ///< 交接缝间隔（上述两者之差）
    bool handover_seam_ok_ = false;            ///< 交接缝判据（含窗口内 COAST 核验）
    bool coast_during_seam_ = false;           ///< 交接缝窗口内是否跌入 COAST
    bool s6_frozen_intent_seen_ = false;       ///< 诊断：观察到 S6 标签的冻结续发意图
    uint64_t pre_servo_last_intent_ms_ = 0;    ///< 诊断：伺服接管前最近一条意图时刻
    uint64_t pre_servo_max_intent_gap_ms_ = 0; ///< 诊断：伺服接管前意图最大间隔
    uint64_t short_lost_frames_ = 0;
    bool short_lost_kept_s7_ = false;
    uint64_t sustained_lost_frames_ = 0;
    bool fallback_seen_ = false;
    bool backup_position_intent_pending_ = false;
    bool backup_position_intent_seen_ = false;
    bool authority_uninterrupted_ = true;

    // 过期/停发/降落
    uint64_t expiry_stop_ms_ = 0;
    uint64_t last_sm_intent_ms_ = 0;  // 最近一条状态机意图时刻（停发判定）
    bool expiry_warning_seen_ = false;
    bool expiry_intent_stop_seen_ = false;
    bool coast_seen_ = false;
    uint64_t coast_ms_ = 0;
    bool idle_seen_ = false;
    uint64_t idle_ms_ = 0;
    uint64_t sends_at_idle_ = 0;
    bool offboard_exit_seen_ = false;
    uint64_t offboard_exit_ms_ = 0;
    uint8_t offboard_exit_mode_ = 0;
    uint8_t offboard_exit_sub_mode_ = 0;
    bool land_command_queued_ = false;
    bool land_ack_received_ = false;
    bool land_ack_accepted_ = false;
    uint64_t land_request_time_ms_ = 0;
    bool landed_after_flight_ = false;
    bool disarm_command_queued_ = false;
    bool disarm_ack_received_ = false;
    bool disarm_ack_accepted_ = false;
    bool disarmed_confirmed_ = false;
    uint64_t disarm_request_time_ms_ = 0;
    bool unexpected_disarm_before_command_ = false;

    // 安全/包线
    bool safety_violation_ = false;
    float max_ascent_m_ = 0.f;
    float max_north_m_ = 0.f;
    float max_east_m_ = 0.f;
};

// ---------------------------------------------------------------------------
// --sitl-stage4-tune：控制律参数整定场景（SITL，详见 tools/px4_link_smoke.md §2.3）
//
// 装配与 G3 同构（MissionStateMachine + FlightController + Px4Link + 注入构件），
// 剧本改为：起飞悬停 → 注入主目标 → S5 接近至射程内 → SEARCHING→LOCKED → S7
// 长驻留（初始捕获暂态 + 稳态）→ 分通道阶跃换位（横向/距离/垂直）→ 停视觉回退
// S5 → 停目标过期停发链 → AUTO.LAND/DISARM。逐拍数据落 CSV，
// tools/tune_report.py 计算整定指标。
//
// 飞行甲板逻辑（引导流/Offboard/ARM/起飞/降落/包线/回收）移植自
// Stage4MissionRun 同款实现并精简（无模式停发探针/围栏探针/重入段）。
// 有意不做基类抽取：G3 是已通过的线下门禁，回归保护优先于代码复用。
// ---------------------------------------------------------------------------

/// 整定剧本选择（哪些阶跃通道要跑）。
enum class TuneScript : uint8_t {
    kStatic,    ///< 仅初始捕获长驻留（基线观测）
    kHeading,   ///< 静态 + 横向阶跃（航向通道）
    kDistance,  ///< 静态 + 距离阶跃
    kVertical,  ///< 静态 + 垂直阶跃
    kFull,      ///< 全部
};

/// 解析 --tune-script 取值（无效值返回 false）。
bool ParseTuneScript(const std::string& text, TuneScript* out) {
    if (text == "static") {
        *out = TuneScript::kStatic;
    } else if (text == "heading") {
        *out = TuneScript::kHeading;
    } else if (text == "distance") {
        *out = TuneScript::kDistance;
    } else if (text == "vertical") {
        *out = TuneScript::kVertical;
    } else if (text == "full") {
        *out = TuneScript::kFull;
    } else {
        return false;
    }
    return true;
}

/// 整定场景固定边界与节律。
// 整定场景起飞 1m（与门禁一致）。注意：S5 位置引导是 3D 的，接近段会把机体
// 拉向目标高度（up=1m），起飞高度的余量在接近段即被抹平——垂直通道的增益
// 安全边界不能靠抬高起飞高度获得（2026-10-14 实测：vz_gain=0.2 时姿态瞬态
// 耦合可把 1m 悬停下压 0.7m，故垂直增益取保守侧）。
constexpr float kTuneTakeoffHeightM = 1.0f;      // 相对起飞高度（与 stage3/4 同）
constexpr float kTuneHoverZErrorM = 0.15f;       // 悬停窗口：高度误差
constexpr float kTuneHoverDriftM = 0.30f;        // 悬停窗口：水平漂移
constexpr float kTuneHoverVzMps = 0.30f;         // 悬停窗口：垂速
constexpr uint64_t kTuneHoverStableMs = 2000;    // 悬停稳定判据时长
constexpr uint64_t kTuneGuidePeriodMs = 100;     // 引导流周期（stage3A 纪律）
constexpr uint64_t kTuneGuideMinSends = 20;      // 引导流最小条数
constexpr uint64_t kTuneGuideMinMs = 2000;       // 引导流最短时长
constexpr uint64_t kTuneIntentPeriodMs = 100;    // 工具引导意图周期（10Hz）
constexpr uint64_t kTuneToolIntentValidMs = 500; // 工具引导意图有效期
constexpr uint64_t kTuneConnectTimeoutMs = 10000;
constexpr uint64_t kTuneBootstrapTimeoutMs = 20000;
constexpr uint64_t kTuneTakeoffTimeoutMs = 15000;
constexpr uint64_t kTuneApproachTimeoutMs = 45000;   // S5 接近段上限（MPC 限速 2m/s）
constexpr uint64_t kTuneHandoverTimeoutMs = 15000;   // 视觉交接段上限
constexpr uint64_t kTuneRecoverTimeoutMs = 20000;    // 停视觉→回退 S5 上限
constexpr uint64_t kTuneExpiryTimeoutMs = 15000;     // 停目标→退出 Offboard 上限
constexpr uint64_t kTuneLandTimeoutMs = 30000;
constexpr uint64_t kTunePollMs = 20;
constexpr uint64_t kTuneCsvPeriodMs = 50;            // CSV 记录 20Hz
// 整定场景硬包线（SITL 专用）：垂直阶跃会让机体爬升至 ~4m，高度上限相应放宽；
// 水平半径不贴近控制器围栏（默认 100m），留足余量。
constexpr float kTuneMaxHeightM = 6.0f;
constexpr float kTuneMinHeightM = 0.3f;   // 升空后下限
constexpr double kTuneMaxRadiusM = 90.0;
constexpr double kTuneBaseTargetNorthM = 40.0;  // 主目标北向（Home 相对）
constexpr double kTuneBaseTargetUpM = 1.0;      // 主目标高度（与悬停同高）

/// 整定 CSV 记录器（20Hz 逐拍一行；分析脚本见 tools/tune_report.py）。
class TuneCsvLogger {
public:
    explicit TuneCsvLogger(const std::string& path) : out_(path) {
        if (out_.is_open()) {
            out_ << "t_ms,phase,mission_state,"
                    "target_north_m,target_east_m,target_up_m,"
                    "self_north_m,self_east_m,self_down_m,self_yaw_deg,"
                    "pixel_x,pixel_y,tracked,beta_deg,alpha_deg,"
                    "distance_m,distance_valid,range_true_m,"
                    "intent_present,intent_type,intent_vx_mps,intent_vy_mps,"
                    "intent_vz_mps,intent_yaw_rate_dps,"
                    "setpoint_valid,setpoint_vx,setpoint_vy,setpoint_vz,"
                    "setpoint_yaw_rate_dps,d_exp_m\n";
            out_.flush();
        }
    }
    bool ok() const { return out_.is_open(); }

    void WriteRow(uint64_t t_ms, const char* phase, int mission_state,
                  const Stage4InjectionRig& rig,
                  const drone::common::FlightStateSnapshot& self, bool have_self,
                  bool tracked, double d_exp_m,
                  const drone::common::ControlIntent* intent,
                  const drone::common::Px4Setpoint* setpoint) {
        if (!out_.is_open()) {
            return;
        }
        const double pixel_x = rig.last_pixel_x();
        const double pixel_y = rig.last_pixel_y();
        double beta_deg = std::numeric_limits<double>::quiet_NaN();
        double alpha_deg = std::numeric_limits<double>::quiet_NaN();
        if (std::isfinite(pixel_x) && std::isfinite(pixel_y)) {
            // 与控制律同式（atan((px−c)/f)），覆盖注入→控制律全链一致性
            beta_deg = std::atan((pixel_x - rig.camera().cx_px) / rig.camera().fx_px) *
                       180.0 / 3.14159265358979323846;
            alpha_deg = std::atan((pixel_y - rig.camera().cy_px) / rig.camera().fy_px) *
                        180.0 / 3.14159265358979323846;
        }
        double range_true = std::numeric_limits<double>::quiet_NaN();
        if (have_self) {
            range_true = std::sqrt(
                (rig.target_north_m() - self.local_x_m) * (rig.target_north_m() - self.local_x_m) +
                (rig.target_east_m() - self.local_y_m) * (rig.target_east_m() - self.local_y_m) +
                (-rig.target_up_m() - self.local_z_m) * (-rig.target_up_m() - self.local_z_m));
        }
        out_ << t_ms << ',' << phase << ',' << mission_state << ','
             << rig.target_north_m() << ',' << rig.target_east_m() << ','
             << rig.target_up_m() << ',';
        if (have_self) {
            out_ << self.local_x_m << ',' << self.local_y_m << ',' << self.local_z_m
                 << ',' << static_cast<double>(self.yaw_rad) * 180.0 / 3.14159265358979323846
                 << ',';
        } else {
            out_ << ",,,";
        }
        out_ << pixel_x << ',' << pixel_y << ',' << (tracked ? 1 : 0) << ','
             << beta_deg << ',' << alpha_deg << ','
             << rig.last_cam_z_m() << ',' << (rig.distance_valid() ? 1 : 0) << ','
             << range_true << ',';
        if (intent != nullptr) {
            out_ << "1," << static_cast<int>(intent->type) << ',' << intent->target_x
                 << ',' << intent->target_y << ',' << intent->target_z << ','
                 << intent->yaw_rate_dps << ',';
        } else {
            out_ << "0,,,,,,";
        }
        if (setpoint != nullptr) {
            out_ << (setpoint->valid ? 1 : 0) << ',' << setpoint->x << ',' << setpoint->y
                 << ',' << setpoint->z << ',' << setpoint->yaw_rate_dps << ',';
        } else {
            out_ << ",,,,,";
        }
        out_ << d_exp_m << '\n';
        out_.flush();
    }

private:
    std::ofstream out_;
};

/// 整定脚本阶段。
enum class TunePhase : uint8_t {
    kBootstrap,     ///< 连接 → 引导流 → Offboard → ARM → 控制器接管
    kTakeoff,       ///< 工具位置引导：相对起飞 1m 稳定悬停
    kApproach,      ///< 注入主目标 → S5 位置引导接近至射程内
    kVisualInject,  ///< SEARCHING → LOCKED → S6 → S7（伺服意图启动）
    kDwellStatic,   ///< S7 静态目标长驻留（初始捕获暂态 + 稳态）
    kStepLateral,   ///< 横向阶跃序列（航向通道）
    kStepRange,     ///< 距离阶跃序列（距离 PID 通道）
    kStepVertical,  ///< 垂直阶跃序列（垂直通道）
    kRecover,       ///< 停视觉注入 → 断流回退 S5
    kTargetExpiry,  ///< 停目标 → 过期 → COAST → IDLE → 退出 Offboard
    kLand,          ///< AUTO.LAND → landed
    kDone,
};

const char* TunePhaseName(TunePhase phase) {
    switch (phase) {
        case TunePhase::kBootstrap: return "bootstrap";
        case TunePhase::kTakeoff: return "takeoff";
        case TunePhase::kApproach: return "approach";
        case TunePhase::kVisualInject: return "visual_inject";
        case TunePhase::kDwellStatic: return "dwell_static";
        case TunePhase::kStepLateral: return "step_lateral";
        case TunePhase::kStepRange: return "step_range";
        case TunePhase::kStepVertical: return "step_vertical";
        case TunePhase::kRecover: return "recover";
        case TunePhase::kTargetExpiry: return "target_expiry";
        case TunePhase::kLand: return "land";
        case TunePhase::kDone: return "done";
    }
    return "unknown";
}

/// 整定场景运行体。
class Stage4TuneRun {
public:
    Stage4TuneRun(drone::communication::Px4Link& link,
                  drone::control::FlightController& controller,
                  drone::state_machine::MissionStateMachine& machine,
                  const drone::config::AppConfig& app_config, TuneScript script,
                  uint64_t static_dwell_s, uint64_t dwell_s, double approach_until_m,
                  const std::string& csv_path, std::chrono::seconds duration,
                  drone::common::Topic<drone::common::FlightStateSnapshot>::Subscription
                      subscription,
                  Stage4InjectionRig rig)
        : link_(link),
          controller_(controller),
          machine_(machine),
          controller_config_(app_config.controller),
          tracking_(app_config.visual_tracking),
          script_(script),
          static_dwell_ms_(static_dwell_s * 1000),
          dwell_ms_(dwell_s * 1000),
          approach_until_m_(approach_until_m),
          duration_ms_(static_cast<uint64_t>(duration.count()) * 1000),
          subscription_(std::move(subscription)),
          rig_(std::move(rig)),
          csv_(csv_path) {
        status_subscription_ = machine_.StatusOutput().Subscribe(64);
        intent_subscription_ = machine_.IntentOutput().Subscribe(64);
        setpoint_subscription_ = controller_.SetpointOutput().Subscribe(64);
        rig_.SetTargetNed(kTuneBaseTargetNorthM, 0.0, kTuneBaseTargetUpM);
    }

    void Execute() {
        const uint64_t started_ms = MonotonicMs();
        uint64_t next_csv_ms = started_ms;
        uint64_t next_summary_ms = started_ms;
        while (!g_stop.load(std::memory_order_acquire) &&
               MonotonicMs() - started_ms < duration_ms_) {
            if (auto message =
                    subscription_.WaitTakeFor(std::chrono::milliseconds(kTunePollMs))) {
                latest_ = **message;
                have_snapshot_ = true;
                if (latest_.armed && !takeoff_target_ready_ &&
                    latest_.local_position_valid) {
                    takeoff_target_ready_ = true;
                    start_x_ = latest_.local_x_m;
                    start_y_ = latest_.local_y_m;
                    start_z_ = latest_.local_z_m;
                    takeoff_target_z_ = start_z_ - kTuneTakeoffHeightM;
                    std::cout << "[SITL] 起飞目标已锁定: N=" << start_x_
                              << " E=" << start_y_ << " D=" << takeoff_target_z_
                              << std::endl;
                }
            }
            const uint64_t now_ms = MonotonicMs();
            DrainOutputs();
            PublishBootstrapIntent(now_ms);
            AdvancePhase(now_ms, now_ms - started_ms);
            if (!aborted_ && rig_.HasError()) {
                Abort(rig_.error());
            }
            if (have_snapshot_) {
                CheckFlightEnvelope();
            }
            if (now_ms >= next_csv_ms) {
                WriteCsvRow(now_ms - started_ms);
                next_csv_ms = now_ms + kTuneCsvPeriodMs;
            }
            if (now_ms >= next_summary_ms) {
                std::cout << "  整定: phase=" << TunePhaseName(phase_)
                          << " 状态机=" << static_cast<int>(mission_state_)
                          << " 目标N=" << rig_.target_north_m()
                          << " E=" << rig_.target_east_m() << " up=" << rig_.target_up_m()
                          << " 射程=" << rig_.last_cam_z_m()
                          << " 控制权=" << AuthorityText(controller_.Authority())
                          << std::endl;
                next_summary_ms = now_ms + 1000;
            }
            if (aborted_ || safety_violation_ || phase_ == TunePhase::kDone) {
                break;
            }
        }
        FinishFlight();
    }

    /// CSV 输出可写性（装配期 fail-fast 用）。
    bool csv_ok() const { return csv_.ok(); }

    static const char* AuthorityText(drone::control::ControlAuthority authority) {
        switch (authority) {
            case drone::control::ControlAuthority::kIdle:
                return "IDLE";
            case drone::control::ControlAuthority::kActive:
                return "ACTIVE";
            case drone::control::ControlAuthority::kCoast:
                return "COAST";
        }
        return "未知";
    }

    /// 结束判定：剧本走通（含回收链）且 CSV 可写、无中断/安全回收。
    bool Evaluate() const {
        const bool passed = phase_ == TunePhase::kDone && csv_.ok() && !aborted_ &&
                            !safety_violation_ && controller_.ErrorCount() == 0 &&
                            landed_after_flight_ && disarmed_confirmed_;
        std::cout << "\n========== stage4-tune 整定场景报告 ==========" << std::endl;
        PrintCheck("剧本走通（含 LOST 回退与过期停发链）", phase_ == TunePhase::kDone, true);
        PrintCheck("CSV 记录可写", csv_.ok(), true);
        PrintCheck("飞行未触发安全回收", !safety_violation_, true);
        PrintCheck("控制器无异常计数", controller_.ErrorCount() == 0, true);
        PrintCheck("AUTO.LAND landed + 主动 DISARM 确认",
                   landed_after_flight_ && disarmed_confirmed_, true);
        std::cout << "包线: 最大相对高度 " << max_ascent_m_ << " m，最大北向 "
                  << max_north_m_ << " m，最大东西 " << max_east_m_
                  << " m；状态机迁移 " << machine_.TransitionCount() << " 次，错误 "
                  << machine_.ErrorCount() << " 次" << std::endl;
        if (!abort_reason_.empty()) {
            std::cout << "[信息] 脚本中断原因: " << abort_reason_ << std::endl;
        }
        std::cout << (passed ? "结果: 整定场景完成" : "结果: 整定场景未完成") << std::endl;
        return passed;
    }

private:
    void Abort(const std::string& reason) {
        if (aborted_) {
            return;
        }
        aborted_ = true;
        abort_reason_ = reason;
        std::cout << "[SITL][中断] " << reason << std::endl;
    }

    const drone::common::FlightStateSnapshot* SnapshotOrNull() const {
        return have_snapshot_ ? &latest_ : nullptr;
    }

    void DrainOutputs() {
        while (auto message = status_subscription_.TryTake()) {
            mission_state_ = (*message)->state;
        }
        while (auto message = intent_subscription_.TryTake()) {
            latest_intent_ = **message;
            have_intent_ = true;
            const MissionIntentClass intent_class =
                ClassifyMissionIntent((*message)->reason_code, (*message)->type);
            if (intent_class == MissionIntentClass::kPositionGuide) {
                sm_intent_seen_ = true;
            }
            if (intent_class == MissionIntentClass::kVisualServo) {
                servo_intent_seen_ = true;
            }
        }
        while (auto message = setpoint_subscription_.TryTake()) {
            latest_setpoint_ = **message;
            have_setpoint_ = true;
        }
    }

    void WriteCsvRow(uint64_t t_ms) {
        csv_.WriteRow(t_ms, TunePhaseName(phase_), static_cast<int>(mission_state_), rig_,
                      latest_, have_snapshot_, current_tracked_,
                      tracking_.distance.d_exp_m,
                      have_intent_ ? &latest_intent_ : nullptr,
                      have_setpoint_ ? &latest_setpoint_ : nullptr);
    }

    // ---- 引导/接管（stage3A 纪律，与 G3 同款） ----

    void InjectGuideSetpoint() {
        drone::common::Px4Setpoint setpoint;
        setpoint.header.receive_time_ms = MonotonicMs();
        setpoint.header.valid_for_ms = 300;
        setpoint.header.frame_id = kMissionFrameLocalNed;
        setpoint.type = drone::common::SetpointType::kVelocity;
        setpoint.valid = true;
        (void)controller_.SetpointOutput().Emplace(setpoint);
        ++guide_injected_;
    }

    /// 工具引导意图：接管前零速度保持、起飞段位置引导（发在状态机输出主题上，
    /// 低字节 0 = 工具引导，语义同 G3）。
    void PublishBootstrapIntent(uint64_t now_ms) {
        if (!bootstrap_intent_active_ || now_ms < next_bootstrap_intent_ms_) {
            return;
        }
        drone::common::ControlIntent intent;
        intent.header.receive_time_ms = now_ms;
        intent.header.valid_for_ms = kTuneToolIntentValidMs;
        intent.header.frame_id = kMissionFrameLocalNed;
        intent.reason_code = drone::common::kControlIntentReasonRequestControl;
        intent.valid_until_ms = now_ms + kTuneToolIntentValidMs;
        if (phase_ == TunePhase::kTakeoff && takeoff_target_ready_) {
            intent.type = drone::common::ControlIntentType::kPositionGuide;
            intent.control_source = 0;
            intent.target_x = start_x_;
            intent.target_y = start_y_;
            intent.target_z = takeoff_target_z_;
        } else {
            intent.type = drone::common::ControlIntentType::kVelocityHeading;
            intent.control_source = 1;
        }
        (void)machine_.IntentOutput().Emplace(intent);
        next_bootstrap_intent_ms_ = now_ms + kTuneIntentPeriodMs;
    }

    void AdvanceBootstrap(uint64_t now_ms, uint64_t elapsed_ms) {
        if (!link_.IsConnected()) {
            if (elapsed_ms >= kTuneConnectTimeoutMs) {
                Abort("连接超时：未收到 PX4 心跳（SITL 未启动或 UDP 配置不符）");
            }
            return;
        }
        if (bootstrap_start_ms_ == 0) {
            bootstrap_start_ms_ = now_ms;
        }
        if (!guide_started_) {
            guide_started_ = true;
            guide_start_ms_ = now_ms;
            next_guide_publish_ms_ = now_ms;
            std::cout << "[SITL] 引导阶段开始：注入零速度引导流" << std::endl;
        }
        if (!handover_done_ && now_ms >= next_guide_publish_ms_) {
            InjectGuideSetpoint();
            next_guide_publish_ms_ = now_ms + kTuneGuidePeriodMs;
        }
        if (!offboard_command_queued_ &&
            guide_injected_ >= kTuneGuideMinSends &&
            now_ms - guide_start_ms_ >= kTuneGuideMinMs) {
            offboard_command_queued_ = SendOffboardRequest();
            std::cout << "[SITL] Offboard 模式请求"
                      << (offboard_command_queued_ ? "已入队" : "入队失败") << std::endl;
        }
        if (have_snapshot_) {
            if (offboard_command_queued_ && !offboard_ack_received_ &&
                latest_.last_ack_time_ms >= offboard_request_time_ms_ &&
                latest_.last_ack_command == MAV_CMD_DO_SET_MODE) {
                offboard_ack_received_ = true;
                offboard_ack_accepted_ = latest_.last_ack_result == MAV_RESULT_ACCEPTED;
            }
            offboard_mode_reached_ =
                offboard_mode_reached_ ||
                (offboard_command_queued_ && latest_.flight_mode == 6 &&
                 latest_.flight_sub_mode == 0);
            if (arm_command_queued_ && !arm_ack_received_ &&
                latest_.last_ack_time_ms >= arm_request_time_ms_ &&
                latest_.last_ack_command == MAV_CMD_COMPONENT_ARM_DISARM) {
                arm_ack_received_ = true;
                arm_ack_accepted_ = latest_.last_ack_result == MAV_RESULT_ACCEPTED;
            }
            // stage3A 前置条件：Offboard ACK accepted + 真实 6/0 + landed + 位置/姿态有效
            if (!arm_command_queued_ && offboard_ack_accepted_ && offboard_mode_reached_ &&
                latest_.connected && !latest_.armed && latest_.landed_state_valid &&
                latest_.landed && latest_.local_position_valid && latest_.attitude_valid) {
                arm_command_queued_ = SendArmDisarm(true);
                std::cout << "[SITL] ARM 请求"
                          << (arm_command_queued_ ? "已入队" : "入队失败") << std::endl;
            }
            if (latest_.armed) {
                armed_reached_ = true;
            }
        }
        if (armed_reached_) {
            if (!handover_done_ &&
                controller_.Authority() == drone::control::ControlAuthority::kActive) {
                handover_done_ = true;
                handover_ms_ = now_ms;
                send_count_at_handover_ = link_.SetpointSendCount();
                std::cout << "[SITL] 控制器已接管设定值流（授权链全通）" << std::endl;
            }
            if (handover_done_ && !takeover_stream_ok_ &&
                link_.SetpointSendCount() > send_count_at_handover_) {
                takeover_stream_ok_ = true;
            }
            if (takeover_stream_ok_ && now_ms - handover_ms_ >= 500) {
                std::cout << "[SITL] 接管完成，进入工具位置引导起飞段" << std::endl;
                phase_ = TunePhase::kTakeoff;
                takeoff_start_ms_ = now_ms;
                return;
            }
        }
        if (bootstrap_start_ms_ != 0 &&
            now_ms - bootstrap_start_ms_ >= kTuneBootstrapTimeoutMs) {
            Abort("引导/Offboard/ARM/接管阶段超时（检查 ACK、模式与授权链）");
        }
    }

    void AdvanceTakeoff(uint64_t now_ms) {
        if (!takeoff_target_ready_ || !latest_.local_position_valid) {
            if (now_ms - takeoff_start_ms_ >= kTuneTakeoffTimeoutMs) {
                Abort("起飞超时：位置基准缺失或位置遥测无效");
            }
            return;
        }
        const float dx = latest_.local_x_m - start_x_;
        const float dy = latest_.local_y_m - start_y_;
        const float drift_m = std::sqrt(dx * dx + dy * dy);
        const bool inside_window =
            std::fabs(latest_.local_z_m - takeoff_target_z_) <= kTuneHoverZErrorM &&
            drift_m <= kTuneHoverDriftM && std::fabs(latest_.vz_mps) <= kTuneHoverVzMps;
        if (inside_window) {
            if (hover_stable_since_ms_ == 0) {
                hover_stable_since_ms_ = now_ms;
            }
            hover_complete_ = now_ms - hover_stable_since_ms_ >= kTuneHoverStableMs;
        } else {
            hover_stable_since_ms_ = 0;
        }
        if (hover_complete_) {
            std::cout << "[SITL] 悬停稳定完成，注入主目标（北向 " << kTuneBaseTargetNorthM
                      << "m），S5 位置引导接近至 " << approach_until_m_ << "m 内"
                      << std::endl;
            phase_ = TunePhase::kApproach;
            phase_start_ms_ = now_ms;
            return;
        }
        if (now_ms - takeoff_start_ms_ >= kTuneTakeoffTimeoutMs) {
            Abort("起飞/悬停超时（检查 home_valid 与位置意图是否被控制拒绝）");
        }
    }

    /// S5 位置引导接近：状态机接管意图流后，等待估计射程进入注入窗口。
    void AdvanceApproach(uint64_t now_ms) {
        (void)rig_.PublishGroundTarget(kMissionTargetIdMain, now_ms, SnapshotOrNull());
        // 状态机意图上线后工具停发引导意图（与 G3 同语义）
        if (mission_state_ == common::MissionState::kGpsApproach && sm_intent_seen_ &&
            bootstrap_intent_active_) {
            bootstrap_intent_active_ = false;
            std::cout << "[SITL] 状态机已接管意图流（S5 位置引导），工具停止注入引导意图"
                      << std::endl;
        }
        if (mission_state_ == common::MissionState::kGpsApproach && sm_intent_seen_ &&
            have_snapshot_ && latest_.local_position_valid) {
            const double north_gap =
                rig_.target_north_m() - static_cast<double>(latest_.local_x_m);
            const double east_gap =
                rig_.target_east_m() - static_cast<double>(latest_.local_y_m);
            if (std::hypot(north_gap, east_gap) <= approach_until_m_) {
                std::cout << "[SITL] 已进入射程（水平间隙 "
                          << std::hypot(north_gap, east_gap)
                          << "m），开始注入模拟视觉观测（SEARCHING → LOCKED）" << std::endl;
                rig_.SetVisionEnabled(true);
                vision_search_until_ms_ = now_ms + kMissionVisualSearchMs;
                phase_ = TunePhase::kVisualInject;
                phase_start_ms_ = now_ms;
                return;
            }
        }
        if (now_ms - phase_start_ms_ >= kTuneApproachTimeoutMs) {
            Abort("接近段超时：未进入注入窗口（检查 MPC_XY_VEL_MAX 限速与 S5 意图）");
        }
    }

    void AdvanceVisualInject(uint64_t now_ms) {
        (void)rig_.PublishGroundTarget(kMissionTargetIdMain, now_ms, SnapshotOrNull());
        if (now_ms < vision_search_until_ms_) {
            current_tracked_ = false;
            (void)rig_.PublishSimulatedVision(common::VisualTargetState::kSearching,
                                              now_ms, SnapshotOrNull());
            return;
        }
        current_tracked_ = true;
        (void)rig_.PublishSimulatedVision(common::VisualTargetState::kLocked, now_ms,
                                          SnapshotOrNull());
        if (mission_state_ == common::MissionState::kVisualTracking &&
            servo_intent_seen_) {
            std::cout << "[SITL] S7 视觉伺服已启动，进入静态驻留段（"
                      << static_dwell_ms_ / 1000 << "s）" << std::endl;
            phase_ = TunePhase::kDwellStatic;
            phase_start_ms_ = now_ms;
            return;
        }
        if (now_ms - phase_start_ms_ >= kTuneHandoverTimeoutMs) {
            Abort("交接段超时：未进入 S7 或伺服意图未启动（查创新/测距/姿态与门限）");
        }
    }

    /// 静态驻留 + 阶跃段的公共节拍：持续注入 LOCKED 并按时刻推进目标换位。
    void AdvanceServoDwell(uint64_t now_ms) {
        (void)rig_.PublishGroundTarget(kMissionTargetIdMain, now_ms, SnapshotOrNull());
        (void)rig_.PublishSimulatedVision(common::VisualTargetState::kLocked, now_ms,
                                          SnapshotOrNull());
        const uint64_t segment_index = (now_ms - phase_start_ms_) / dwell_ms_;
        switch (phase_) {
            case TunePhase::kDwellStatic:
                if (now_ms - phase_start_ms_ >= static_dwell_ms_) {
                    NextStepPhase(now_ms);
                }
                return;
            case TunePhase::kStepLateral: {
                // E: 0 → +2 → 0 → +4 → 0 → −2 → 0 → −4 → 0（每段驻留 dwell_ms）
                static constexpr double kSequence[] = {0.0, 2.0, 0.0, 4.0,
                                                       0.0, -2.0, 0.0, -4.0, 0.0};
                ApplyStepTarget(kTuneBaseTargetNorthM, kSequence[std::min<size_t>(
                                                             segment_index, 8)],
                                kTuneBaseTargetUpM);
                if (segment_index >= 8) {
                    NextStepPhase(now_ms);
                }
                return;
            }
            case TunePhase::kStepRange: {
                // N: 40 → +2.5 → 40 → +5 → 40 → −2.5 → 40 → −5 → 40（远/近交替）。
                // 幅值上限 5m：车辆以 d_exp=10m 尾随目标，远跳追赶后目标回跳时
                // 几何余量最小（45←40 回跳≈5m），±8m 会跌破换位前向余量（2m）。
                static constexpr double kSequence[] = {0.0, 2.5, 0.0, 5.0, 0.0,
                                                       -2.5, 0.0, -5.0, 0.0};
                ApplyStepTarget(kTuneBaseTargetNorthM + kSequence[std::min<size_t>(
                                                                      segment_index, 8)],
                                0.0, kTuneBaseTargetUpM);
                if (segment_index >= 8) {
                    NextStepPhase(now_ms);
                }
                return;
            }
            case TunePhase::kStepVertical: {
                // up: 1 → 2 → 1 → 4 → 1（α 阶跃；机体会爬升，包线已放宽）
                static constexpr double kSequence[] = {0.0, 1.0, 0.0, 3.0, 0.0};
                ApplyStepTarget(kTuneBaseTargetNorthM, 0.0,
                                kTuneBaseTargetUpM + kSequence[std::min<size_t>(
                                                                   segment_index, 4)]);
                if (segment_index >= 4) {
                    NextStepPhase(now_ms);
                }
                return;
            }
            default:
                return;
        }
    }

    /// 阶跃换位：合法性校验（围栏内 + 换位后预算 cam_z ≥ 2m）后移动目标。
    void ApplyStepTarget(double north_m, double east_m, double up_m) {
        if (north_m == step_target_north_m_ && east_m == step_target_east_m_ &&
            up_m == step_target_up_m_) {
            return;  // 段内不换
        }
        const double radius = std::hypot(north_m, east_m);
        if (radius > controller_config_.max_radius_m - 10.0) {
            Abort("整定阶跃目标越出围栏余量");
            return;
        }
        if (have_snapshot_ && latest_.local_position_valid && latest_.attitude_valid) {
            // 换位后预算前向距离（仅 yaw 近似，姿态小角下足够保守）
            const double north_gap = north_m - static_cast<double>(latest_.local_x_m);
            const double east_gap = east_m - static_cast<double>(latest_.local_y_m);
            const double forward =
                std::cos(static_cast<double>(latest_.yaw_rad)) * north_gap +
                std::sin(static_cast<double>(latest_.yaw_rad)) * east_gap;
            if (forward < 2.0) {
                Abort("整定阶跃换位后目标将不在相机前向余量内（<2m）");
                return;
            }
        }
        std::cout << "[SITL] 目标阶跃换位: N=" << north_m << " E=" << east_m
                  << " up=" << up_m << std::endl;
        rig_.SetTargetNed(north_m, east_m, up_m);
        step_target_north_m_ = north_m;
        step_target_east_m_ = east_m;
        step_target_up_m_ = up_m;
    }

    /// 阶跃序列推进：严格单向 dwell→横向→距离→垂直→回收，剧本按通道裁剪。
    void NextStepPhase(uint64_t now_ms) {
        const bool want_heading =
            script_ == TuneScript::kHeading || script_ == TuneScript::kFull;
        const bool want_distance =
            script_ == TuneScript::kDistance || script_ == TuneScript::kFull;
        const bool want_vertical =
            script_ == TuneScript::kVertical || script_ == TuneScript::kFull;
        TunePhase next;
        if (phase_ == TunePhase::kDwellStatic) {
            next = want_heading ? TunePhase::kStepLateral
                 : want_distance ? TunePhase::kStepRange
                 : want_vertical ? TunePhase::kStepVertical
                                 : TunePhase::kRecover;
        } else if (phase_ == TunePhase::kStepLateral) {
            next = want_distance ? TunePhase::kStepRange
                 : want_vertical ? TunePhase::kStepVertical
                                 : TunePhase::kRecover;
        } else if (phase_ == TunePhase::kStepRange) {
            next = want_vertical ? TunePhase::kStepVertical : TunePhase::kRecover;
        } else {
            next = TunePhase::kRecover;
        }
        if (next == TunePhase::kRecover) {
            std::cout << "[SITL] 阶跃段完成，停止视觉注入（断流回退 S5）" << std::endl;
            rig_.SetVisionEnabled(false);
            current_tracked_ = false;
        } else {
            std::cout << "[SITL] 进入"
                      << (next == TunePhase::kStepLateral ? "横向"
                          : next == TunePhase::kStepRange ? "距离" : "垂直")
                      << "阶跃段" << std::endl;
        }
        phase_ = next;
        // 回零位再进下一段（保证下一通道激励时其余通道已收敛）
        rig_.SetTargetNed(kTuneBaseTargetNorthM, 0.0, kTuneBaseTargetUpM);
        step_target_north_m_ = kTuneBaseTargetNorthM;
        step_target_east_m_ = 0.0;
        step_target_up_m_ = kTuneBaseTargetUpM;
        phase_start_ms_ = now_ms;
    }

    /// 停视觉注入 → 状态机断流回退 S5（卡尔曼备份接管）。
    void AdvanceRecover(uint64_t now_ms) {
        (void)rig_.PublishGroundTarget(kMissionTargetIdMain, now_ms, SnapshotOrNull());
        if (mission_state_ == common::MissionState::kGpsApproach) {
            std::cout << "[SITL] 已回退 S5（卡尔曼备份接管），停止注入目标（过期链）"
                      << std::endl;
            rig_.SetTargetEnabled(false);
            phase_ = TunePhase::kTargetExpiry;
            phase_start_ms_ = now_ms;
            return;
        }
        if (now_ms - phase_start_ms_ >= kTuneRecoverTimeoutMs) {
            Abort("回退段超时：停视觉后未回退 S5");
        }
    }

    /// 停目标 → 过期 → COAST → IDLE → PX4 退出 Offboard（同 G3 收尾语义）。
    void AdvanceTargetExpiry(uint64_t now_ms) {
        if (have_snapshot_ && latest_.flight_mode != 6) {
            std::cout << "[SITL] PX4 已退出 Offboard: mode="
                      << static_cast<int>(latest_.flight_mode) << '/'
                      << static_cast<int>(latest_.flight_sub_mode) << std::endl;
            land_command_queued_ = SendAutoLand();
            std::cout << "[SITL] AUTO.LAND 模式请求"
                      << (land_command_queued_ ? "已入队" : "入队失败") << std::endl;
            phase_ = TunePhase::kLand;
            phase_start_ms_ = now_ms;
            return;
        }
        if (now_ms - phase_start_ms_ >= kTuneExpiryTimeoutMs) {
            Abort("过期停发链超时：PX4 未退出 Offboard");
        }
    }

    void AdvanceLand(uint64_t now_ms) {
        if (have_snapshot_ && land_command_queued_ && !land_ack_received_ &&
            latest_.last_ack_time_ms >= land_request_time_ms_ &&
            latest_.last_ack_command == MAV_CMD_DO_SET_MODE) {
            land_ack_received_ = true;
            land_ack_accepted_ = latest_.last_ack_result == MAV_RESULT_ACCEPTED;
        }
        if (land_ack_received_ &&
            (!land_ack_accepted_ || !latest_.connected) &&
            now_ms - phase_start_ms_ >= kTuneExpiryTimeoutMs) {
            Abort("AUTO.LAND 未被接受");
            return;
        }
        if (have_snapshot_ && latest_.landed_state_valid && latest_.landed) {
            landed_after_flight_ = true;
            std::cout << "[SITL] PX4 已确认降落，准备主动 DISARM" << std::endl;
            phase_ = TunePhase::kDone;
        }
        if (now_ms - phase_start_ms_ >= kTuneLandTimeoutMs) {
            Abort("降落超时");
        }
    }

    void AdvancePhase(uint64_t now_ms, uint64_t elapsed_ms) {
        switch (phase_) {
            case TunePhase::kBootstrap:
                AdvanceBootstrap(now_ms, elapsed_ms);
                return;
            case TunePhase::kTakeoff:
                AdvanceTakeoff(now_ms);
                return;
            case TunePhase::kApproach:
                AdvanceApproach(now_ms);
                return;
            case TunePhase::kVisualInject:
                AdvanceVisualInject(now_ms);
                return;
            case TunePhase::kDwellStatic:
            case TunePhase::kStepLateral:
            case TunePhase::kStepRange:
            case TunePhase::kStepVertical:
                AdvanceServoDwell(now_ms);
                return;
            case TunePhase::kRecover:
                AdvanceRecover(now_ms);
                return;
            case TunePhase::kTargetExpiry:
                AdvanceTargetExpiry(now_ms);
                return;
            case TunePhase::kLand:
                AdvanceLand(now_ms);
                return;
            case TunePhase::kDone:
                return;
        }
    }

    /// 包线检查：armed 期间连接/位置/姿态失效、非预期退出 Offboard、
    /// 高度/水平越界 → 安全回收（FinishFlight 统一 AUTO.LAND）。
    void CheckFlightEnvelope() {
        if (armed_reached_ && !latest_.armed && !disarm_command_queued_) {
            unexpected_disarm_before_command_ = true;
        }
        if (!latest_.armed || land_command_queued_ || safety_violation_) {
            return;
        }
        if (!latest_.connected || !latest_.local_position_valid ||
            !latest_.attitude_valid) {
            safety_violation_ = true;
            std::cout << "[SITL][安全回收] 连接、位置或姿态失效" << std::endl;
            return;
        }
        if (!expect_offboard_exit_ && latest_.flight_mode != 6) {
            safety_violation_ = true;
            std::cout << "[SITL][安全回收] 非预期退出 Offboard（mode="
                      << static_cast<int>(latest_.flight_mode) << "）" << std::endl;
            return;
        }
        if (!takeoff_target_ready_) {
            return;
        }
        const float ascent_m = start_z_ - latest_.local_z_m;
        const double north_m =
            static_cast<double>(latest_.local_x_m) - static_cast<double>(start_x_);
        const double east_m =
            std::fabs(static_cast<double>(latest_.local_y_m) - start_y_);
        max_ascent_m_ = std::max(max_ascent_m_, ascent_m);
        max_north_m_ = std::max(max_north_m_, static_cast<float>(north_m));
        max_east_m_ = std::max(max_east_m_, static_cast<float>(east_m));
        const double radius = std::hypot(static_cast<double>(latest_.local_x_m),
                                         static_cast<double>(latest_.local_y_m));
        const bool airborne = phase_ != TunePhase::kBootstrap &&
                              phase_ != TunePhase::kTakeoff;
        const bool height_out =
            ascent_m > kTuneMaxHeightM || (airborne && ascent_m < kTuneMinHeightM);
        const bool horizontal_out = radius > kTuneMaxRadiusM;
        if (height_out || horizontal_out) {
            safety_violation_ = true;
            std::cout << "[SITL][安全回收] 高度/水平位移超过整定包线: 相对高度 "
                      << ascent_m << " m，半径 " << radius << " m" << std::endl;
        }
    }

    /// AUTO.LAND 回收 + 主动 DISARM（与 G3 FinishFlight 同语义）。
    void FinishFlight() {
        expect_offboard_exit_ = true;  // 回收路径的退出属预期
        const bool armed_now = armed_reached_ || latest_.armed;
        if (armed_now && !landed_after_flight_) {
            if (!land_command_queued_) {
                land_command_queued_ = SendAutoLand();
                std::cout << "[SITL] 异常回收 AUTO.LAND 请求"
                          << (land_command_queued_ ? "已入队" : "入队失败") << std::endl;
            }
            const uint64_t deadline = MonotonicMs() + kTuneLandTimeoutMs;
            while (MonotonicMs() < deadline) {
                if (auto message =
                        subscription_.WaitTakeFor(std::chrono::milliseconds(100))) {
                    latest_ = **message;
                    have_snapshot_ = true;
                    if (latest_.landed_state_valid && latest_.landed) {
                        landed_after_flight_ = true;
                        break;
                    }
                }
            }
            std::cout << "[SITL] 回收结果: landed=" << YesNo(landed_after_flight_)
                      << std::endl;
        }
        if (armed_now && landed_after_flight_) {
            while (subscription_.TryTake()) {
                // 清除排队旧快照，避免用解锁前 armed=false 误判上锁完成。
            }
            disarm_command_queued_ = SendArmDisarm(false);
            std::cout << "[SITL] DISARM 请求"
                      << (disarm_command_queued_ ? "已入队" : "入队失败") << std::endl;
            const uint64_t deadline = MonotonicMs() + 5000;
            while (MonotonicMs() < deadline) {
                if (auto message =
                        subscription_.WaitTakeFor(std::chrono::milliseconds(100))) {
                    latest_ = **message;
                    have_snapshot_ = true;
                    if (disarm_command_queued_ && !disarm_ack_received_ &&
                        latest_.last_ack_time_ms >= disarm_request_time_ms_ &&
                        latest_.last_ack_command == MAV_CMD_COMPONENT_ARM_DISARM) {
                        disarm_ack_received_ = true;
                        disarm_ack_accepted_ =
                            latest_.last_ack_result == MAV_RESULT_ACCEPTED;
                    }
                    if (disarm_ack_accepted_ && !latest_.armed) {
                        disarmed_confirmed_ = true;
                        break;
                    }
                }
            }
        }
    }

    // ---- 命令发送（与 G3 同款） ----

    bool SendOffboardRequest() {
        constexpr uint8_t kPx4MainModeOffboard = 6;
        const bool queued = link_.SendCommand(
            MAV_CMD_DO_SET_MODE, static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            static_cast<float>(kPx4MainModeOffboard), 0.f, 0.f, 0.f, 0.f, 0.f);
        if (queued) {
            offboard_request_time_ms_ = MonotonicMs();
        }
        return queued;
    }

    bool SendArmDisarm(bool arm) {
        const bool queued = link_.SendCommand(MAV_CMD_COMPONENT_ARM_DISARM, arm ? 1.f : 0.f,
                                              0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
        if (queued) {
            if (arm) {
                arm_request_time_ms_ = MonotonicMs();
            } else {
                disarm_request_time_ms_ = MonotonicMs();
            }
        }
        return queued;
    }

    bool SendAutoLand() {
        constexpr uint8_t kPx4MainModeAuto = 4;
        constexpr uint8_t kPx4SubModeAutoLand = 6;
        const bool queued = link_.SendCommand(
            MAV_CMD_DO_SET_MODE, static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            static_cast<float>(kPx4MainModeAuto), static_cast<float>(kPx4SubModeAutoLand),
            0.f, 0.f, 0.f, 0.f);
        if (queued) {
            land_request_time_ms_ = MonotonicMs();
        }
        return queued;
    }

    // ---- 依赖 ----
    drone::communication::Px4Link& link_;
    drone::control::FlightController& controller_;
    drone::state_machine::MissionStateMachine& machine_;
    const drone::config::FlightControllerConfig& controller_config_;
    const drone::control::TrackingConfig& tracking_;
    const TuneScript script_;
    const uint64_t static_dwell_ms_;
    const uint64_t dwell_ms_;
    const double approach_until_m_;
    uint64_t duration_ms_ = 0;
    drone::common::Topic<drone::common::FlightStateSnapshot>::Subscription subscription_;
    drone::common::Topic<drone::common::MissionStatus>::Subscription status_subscription_;
    drone::common::Topic<drone::common::ControlIntent>::Subscription intent_subscription_;
    drone::common::Topic<drone::common::Px4Setpoint>::Subscription setpoint_subscription_;
    Stage4InjectionRig rig_;
    TuneCsvLogger csv_;

    // ---- 观测 ----
    drone::common::FlightStateSnapshot latest_;
    bool have_snapshot_ = false;
    common::MissionState mission_state_ = common::MissionState::kBoot;
    drone::common::ControlIntent latest_intent_;
    bool have_intent_ = false;
    drone::common::Px4Setpoint latest_setpoint_;
    bool have_setpoint_ = false;
    bool current_tracked_ = false;
    bool sm_intent_seen_ = false;
    bool servo_intent_seen_ = false;

    // ---- 脚本推进 ----
    TunePhase phase_ = TunePhase::kBootstrap;
    bool aborted_ = false;
    std::string abort_reason_;
    uint64_t phase_start_ms_ = 0;
    bool bootstrap_intent_active_ = true;
    uint64_t next_bootstrap_intent_ms_ = 0;
    uint64_t next_guide_publish_ms_ = 0;
    bool guide_started_ = false;
    uint64_t guide_start_ms_ = 0;
    uint64_t guide_injected_ = 0;
    uint64_t vision_search_until_ms_ = 0;
    uint64_t bootstrap_start_ms_ = 0;
    double step_target_north_m_ = kTuneBaseTargetNorthM;
    double step_target_east_m_ = 0.0;
    double step_target_up_m_ = kTuneBaseTargetUpM;

    // 引导/Offboard/ARM/接管
    bool offboard_command_queued_ = false;
    bool offboard_ack_received_ = false;
    bool offboard_ack_accepted_ = false;
    bool offboard_mode_reached_ = false;
    uint64_t offboard_request_time_ms_ = 0;
    bool arm_command_queued_ = false;
    bool arm_ack_received_ = false;
    bool arm_ack_accepted_ = false;
    bool armed_reached_ = false;
    uint64_t arm_request_time_ms_ = 0;
    bool handover_done_ = false;
    bool takeover_stream_ok_ = false;
    uint64_t handover_ms_ = 0;
    uint64_t send_count_at_handover_ = 0;
    bool expect_offboard_exit_ = false;

    // 起飞/悬停
    bool takeoff_target_ready_ = false;
    float start_x_ = 0.f;
    float start_y_ = 0.f;
    float start_z_ = 0.f;
    float takeoff_target_z_ = 0.f;
    uint64_t takeoff_start_ms_ = 0;
    bool hover_complete_ = false;
    uint64_t hover_stable_since_ms_ = 0;

    // 降落/上锁
    bool land_command_queued_ = false;
    bool land_ack_received_ = false;
    bool land_ack_accepted_ = false;
    uint64_t land_request_time_ms_ = 0;
    bool landed_after_flight_ = false;
    bool disarm_command_queued_ = false;
    bool disarm_ack_received_ = false;
    bool disarm_ack_accepted_ = false;
    bool disarmed_confirmed_ = false;
    uint64_t disarm_request_time_ms_ = 0;
    bool unexpected_disarm_before_command_ = false;

    // 安全/包线
    bool safety_violation_ = false;
    float max_ascent_m_ = 0.f;
    float max_north_m_ = 0.f;
    float max_east_m_ = 0.f;
};

/// stage4-tune 入口：装配与 G3 同构，运行整定剧本并落 CSV。
int RunStage4Tune(drone::communication::Px4Link& link, const std::string& config_path,
                  const Options& options) {
    const drone::config::AppConfig app =
        drone::config::LoadAppConfig(config_path, ExecDir());
    TuneScript tune_script = TuneScript::kFull;
    (void)ParseTuneScript(options.tune_script, &tune_script);  // ParseOptions 已校验
    std::string csv_path = options.tune_csv_path;
    if (csv_path.empty()) {
        char name[64];
        const std::time_t now = std::time(nullptr);
        std::tm tm_buf {};
        localtime_r(&now, &tm_buf);
        std::strftime(name, sizeof(name), "stage4_tune_%Y%m%d_%H%M%S.csv", &tm_buf);
        csv_path = ExecDir() + "/logs/" + name;
    }
    // CSV 父目录可能尚不存在（日志库推迟建目录）；忽略已存在错误。
    const std::string csv_dir = csv_path.substr(0, csv_path.find_last_of('/'));
    if (!csv_dir.empty()) {
        (void)::mkdir(csv_dir.c_str(), 0755);
    }

    drone::perception::TargetEstimatorConfig estimator_config = app.target_estimator;
    const double fxb_m_px =
        LoadRectifyFxb(config_path, app.stereo_ranger.rectify_matrices_path);
    estimator_config.visual.enabled = true;
    estimator_config.visual.fxb_m_px = fxb_m_px;
    estimator_config.Validate();

    drone::common::Topic<drone::common::GroundStationTarget> ground_target_topic;
    drone::common::Topic<drone::common::HealthStatus> health_topic;
    drone::common::Topic<drone::common::VisualTargetStatus> visual_topic;
    drone::common::Topic<drone::common::StereoTargetDistance> distance_topic;

    drone::perception::TargetEstimator estimator(estimator_config);
    estimator.SetGroundTargetInput(ground_target_topic);
    estimator.SetFlightStateInput(link.StateOutput());
    estimator.SetVisualDistanceInput(distance_topic);

    drone::state_machine::MissionStateMachineConfig mission_config;
    mission_config.intent_valid_for = app.controller.intent_timeout;
    mission_config.handover = app.handover;
    mission_config.tracking = app.visual_tracking;
    mission_config.visual_stale = std::chrono::milliseconds(
        app.visual_tracking.control.visual_stale_ms);
    mission_config.distance_stale = std::chrono::milliseconds(
        app.visual_tracking.control.distance_stale_ms);
    mission_config.actuator = app.actuator;
    drone::state_machine::MissionStateMachine machine(mission_config);

    drone::control::FlightController controller(app.controller);
    controller.SetInput(machine.IntentOutput());
    controller.SetPx4LinkSource(&link);
    link.SetInput(controller.SetpointOutput());

    machine.SetInputs(ground_target_topic, link.StateOutput(), health_topic);
    machine.SetEstimationInputs(estimator.EstimatedOutput(), visual_topic, distance_topic);
    machine.SetEstimationDiagnostics(&estimator);

    std::cout << "  整定参数: heading.gain=" << app.visual_tracking.heading.gain
              << " vertical.vz_gain=" << app.visual_tracking.vertical.vz_gain_mps_per_deg
              << " distance kp=" << app.visual_tracking.distance.kp
              << " ki=" << app.visual_tracking.distance.ki
              << " kd=" << app.visual_tracking.distance.kd
              << " d_exp=" << app.visual_tracking.distance.d_exp_m
              << "m；剧本 dwell=" << options.tune_static_dwell_s << "/"
              << options.tune_dwell_s << "s，接近窗口 ≤" << options.tune_approach_until_m
              << "m" << std::endl;

    Stage4InjectionRig rig(ground_target_topic, visual_topic, distance_topic,
                           app.visual_tracking.camera, app.target_estimator.visual.mount,
                           fxb_m_px);
    // 装配期自检先于任何 Start（fail-fast，与 G3 同）。
    if (!rig.VerifyInjectionGeometry()) {
        std::cerr << "[FAIL] 注入几何自检失败（反变换与正变换不互逆），拒绝开跑"
                  << std::endl;
        return 1;
    }

    auto subscription = link.StateOutput().Subscribe(128);
    Stage4TuneRun run(link, controller, machine, app, tune_script,
                      options.tune_static_dwell_s, options.tune_dwell_s,
                      options.tune_approach_until_m, csv_path,
                      options.duration, std::move(subscription), std::move(rig));
    std::cout << "  整定 CSV: " << csv_path << std::endl;
    if (!run.csv_ok()) {
        std::cerr << "[FAIL] 无法打开整定 CSV 输出文件: " << csv_path << std::endl;
        return 1;
    }
    if (!link.Start()) {
        std::cerr << "[FAIL] PX4 链路启动失败，请检查 UDP 端口占用与配置" << std::endl;
        return 2;
    }
    if (!estimator.Start()) {
        std::cerr << "[FAIL] 目标估计器启动失败" << std::endl;
        link.Stop();
        return 2;
    }
    if (!machine.Start()) {
        std::cerr << "[FAIL] 任务状态机启动失败" << std::endl;
        estimator.Stop();
        link.Stop();
        return 2;
    }
    if (!controller.Start()) {
        std::cerr << "[FAIL] 飞行控制器启动失败" << std::endl;
        machine.Stop();
        estimator.Stop();
        link.Stop();
        return 2;
    }
    run.Execute();
    controller.Stop();
    machine.Stop();
    estimator.Stop();
    link.Stop();
    return run.Evaluate() ? 0 : 3;
}

/// 工具离线自检（`--selftest`）：不需要 SITL、配置或硬件，验证 G3 意图分类等
/// 纯逻辑。工具自身不在 GoogleTest 覆盖范围内，而这类断言已连漏两次
/// （2026-10-14 终审 Critical：S7 意图被原因码过滤误丢）——故抽成纯函数并纳入
/// ctest（`px4_link_smoke_selftest`）。
int RunToolSelfTest() {
    struct Case {
        const char* name;
        uint32_t reason_code;
        drone::common::ControlIntentType type;
        MissionIntentClass expected;
        bool expected_counted;
    };
    const Case cases[] = {
        {"工具引导零速意图(位31,低字节0,速度航向)",
         drone::common::kControlIntentReasonRequestControl,
         drone::common::ControlIntentType::kVelocityHeading,
         MissionIntentClass::kToolBootstrap, false},
        {"工具起飞位置意图(位31,低字节0,位置引导)",
         drone::common::kControlIntentReasonRequestControl,
         drone::common::ControlIntentType::kPositionGuide,
         MissionIntentClass::kToolBootstrap, false},
        {"S5/S6 位置引导(位31|7)", 0x80000007U,
         drone::common::ControlIntentType::kPositionGuide,
         MissionIntentClass::kPositionGuide, true},
        {"S7 视觉伺服(位31|100)", 0x80000064U,
         drone::common::ControlIntentType::kVelocityHeading,
         MissionIntentClass::kVisualServo, true},
        {"S7 判丢刹车(位31|102)", 0x80000066U,
         drone::common::ControlIntentType::kBrakeHover,
         MissionIntentClass::kVisualBrake, true},
        {"状态机其它低位码(位31|9)", 0x80000009U,
         drone::common::ControlIntentType::kVelocityHeading,
         MissionIntentClass::kStateMachineOther, true},
        {"低位码100但类型不符(位置引导)", 0x80000064U,
         drone::common::ControlIntentType::kPositionGuide,
         MissionIntentClass::kStateMachineOther, true},
        {"低位码7但类型不符(速度航向)", 0x80000007U,
         drone::common::ControlIntentType::kVelocityHeading,
         MissionIntentClass::kStateMachineOther, true},
    };
    std::cout << "tools/px4_link_smoke 离线自检（G3 意图分类）" << std::endl;
    bool passed = true;
    for (const auto& item : cases) {
        const MissionIntentClass actual =
            ClassifyMissionIntent(item.reason_code, item.type);
        const bool counted = CountsAsStateMachineIntent(actual);
        const bool ok = actual == item.expected && counted == item.expected_counted;
        std::cout << (ok ? "[PASS] " : "[FAIL] ") << item.name << " → 分类="
                  << MissionIntentClassName(actual) << " 计入轨迹=" << YesNo(counted)
                  << std::endl;
        passed = passed && ok;
    }
    std::cout << (passed ? "结果: 工具离线自检通过" : "结果: 工具离线自检未通过")
              << std::endl;
    return passed ? 0 : 1;
}

/// stage4-mission 入口：装配状态机全链（G3）并运行脚本。
int RunStage4Mission(drone::communication::Px4Link& link, const std::string& config_path,
                     const Options& options) {
    // 全量配置：controller/handover/visual_tracking/target_estimator 等整节同源，
    // 与正式装配一致（config 缺节时用默认值，与 LoadAppConfig 行为一致）。
    const drone::config::AppConfig app =
        drone::config::LoadAppConfig(config_path, ExecDir());

    // 视觉融合：正式装配置位依赖 runtime 开关（工具注入模拟测距，不经相机链路，
    // 故等价补齐）；fxb 深度系数仍从校正矩阵同源文件读取（单一真源）。
    drone::perception::TargetEstimatorConfig estimator_config = app.target_estimator;
    const double fxb_m_px =
        LoadRectifyFxb(config_path, app.stereo_ranger.rectify_matrices_path);
    estimator_config.visual.enabled = true;
    estimator_config.visual.fxb_m_px = fxb_m_px;
    estimator_config.Validate();

    drone::common::Topic<drone::common::GroundStationTarget> ground_target_topic;
    drone::common::Topic<drone::common::HealthStatus> health_topic;  // 状态机未消费内容
    drone::common::Topic<drone::common::VisualTargetStatus> visual_topic;
    drone::common::Topic<drone::common::StereoTargetDistance> distance_topic;

    drone::perception::TargetEstimator estimator(estimator_config);
    estimator.SetGroundTargetInput(ground_target_topic);
    estimator.SetFlightStateInput(link.StateOutput());
    estimator.SetVisualDistanceInput(distance_topic);

    drone::state_machine::MissionStateMachineConfig mission_config;
    mission_config.intent_valid_for = app.controller.intent_timeout;
    mission_config.handover = app.handover;
    mission_config.tracking = app.visual_tracking;
    mission_config.visual_stale = std::chrono::milliseconds(
        app.visual_tracking.control.visual_stale_ms);
    mission_config.distance_stale = std::chrono::milliseconds(
        app.visual_tracking.control.distance_stale_ms);
    mission_config.actuator = app.actuator;
    drone::state_machine::MissionStateMachine machine(mission_config);

    drone::control::FlightController controller(app.controller);
    controller.SetInput(machine.IntentOutput());
    controller.SetPx4LinkSource(&link);
    link.SetInput(controller.SetpointOutput());

    machine.SetInputs(ground_target_topic, link.StateOutput(), health_topic);
    machine.SetEstimationInputs(estimator.EstimatedOutput(), visual_topic, distance_topic);
    machine.SetEstimationDiagnostics(&estimator);

    std::cout << "  状态机: lock_frames=" << app.handover.lock_frames
              << " exit_lost_frames=" << app.handover.exit_lost_frames
              << " handover_timeout=" << app.handover.timeout.count()
              << "ms innovation_gate=" << app.handover.innovation_gate_m
              << "m fow_margin=" << app.handover.fow_margin_deg << "deg"
              << " intent_valid=" << app.controller.intent_timeout.count() << "ms"
              << std::endl;
    std::cout << "  注入几何: 探针目标北向 " << kMissionFenceProbeNorthM
              << "m（> max_radius " << app.controller.max_radius_m
              << "m），主目标北向 " << kMissionTargetNorthM << "m，目标高度 +"
              << kMissionTargetUpM << "m；相机 " << app.visual_tracking.camera.image_width
              << "x" << app.visual_tracking.camera.image_height << " fx="
              << app.visual_tracking.camera.fx_px << std::endl;

    auto subscription = link.StateOutput().Subscribe(128);
    Stage4MissionRun run(link, controller, machine, ground_target_topic, visual_topic,
                         distance_topic, estimator.EstimatedOutput(), app, fxb_m_px,
                         static_cast<uint64_t>(app.controller.intent_timeout.count()),
                         options.duration, std::move(subscription));
    // 装配期自检先于任何 Start（fail-fast，不进入运行态）：注入几何反变换必须
    // 与估计器正变换互逆，否则注入观测与卡尔曼航迹偏离、创新门限误拒交接。
    if (!run.VerifyInjectionGeometry()) {
        std::cerr << "[FAIL] 注入几何自检失败（反变换与正变换不互逆），拒绝开跑"
                  << std::endl;
        return 1;
    }
    if (!link.Start()) {
        std::cerr << "[FAIL] PX4 链路启动失败，请检查 UDP 端口占用与配置" << std::endl;
        return 2;
    }
    if (!estimator.Start()) {
        std::cerr << "[FAIL] 目标估计器启动失败" << std::endl;
        link.Stop();
        return 2;
    }
    if (!machine.Start()) {
        std::cerr << "[FAIL] 任务状态机启动失败" << std::endl;
        estimator.Stop();
        link.Stop();
        return 2;
    }
    if (!controller.Start()) {
        std::cerr << "[FAIL] 飞行控制器启动失败" << std::endl;
        machine.Stop();
        estimator.Stop();
        link.Stop();
        return 2;
    }
    run.Execute();
    const bool connected_at_end = link.IsConnected();
    controller.Stop();
    machine.Stop();
    estimator.Stop();
    link.Stop();
    return run.Evaluate(connected_at_end) ? 0 : 3;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = ParseOptions(argc, argv);
        if (options.selftest) {
            return RunToolSelfTest();  // 离线自检：不读配置、不触硬件
        }
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
        if ((sitl_setpoint_test || options.sitl_stage4_controller ||
             options.sitl_stage4_mission || options.sitl_stage4_tune) &&
            config.transport != "udp") {
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
                  << (options.sitl_stage4_tune
                          ? "SITL UDP 控制律整定场景：长驻留 + 分通道阶跃，落 CSV"
                          : (options.sitl_stage4_mission
                          ? "SITL UDP 装配状态机全链（G3）：注入地面站目标 + 模拟视觉"
                            "观测，走通 S5/S6/S7/回退/停发；含围栏钳制与模式停发补验"
                          : (options.sitl_stage4_controller
                          ? "SITL UDP 装配真实 FlightController，意图脚本驱动全链；"
                            "零速漂移/阶跃斜率/停发刹车/Offboard 退出时延验收"
                          : (options.sitl_offboard_loss
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
                                        : "仅发送 HEARTBEAT 和遥测请求命令，不发送飞行控制指令")))))))))
                  << '\n'
                  << std::endl;

        drone::communication::Px4Link link(config);
        if (options.sitl_stage4_controller) {
            // stage4 独立装配（意图主题 + FlightController + Px4Link）与独立报告，
            // 与 stage 0~3D 的直发 setpoint 路径完全分离。
            return RunStage4(link, root, options);
        }
        if (options.sitl_stage4_mission) {
            // stage4-mission（G3）独立装配：状态机全链 + 注入上游，独立报告。
            return RunStage4Mission(link, config_path, options);
        }
        if (options.sitl_stage4_tune) {
            // stage4-tune 控制律整定场景：装配与 G3 同构，剧本为长驻留 + 分通道
            // 阶跃，逐拍落 CSV 供 tools/tune_report.py 计算整定指标。
            return RunStage4Tune(link, config_path, options);
        }
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
