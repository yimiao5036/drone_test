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
#include <vector>

#include <sys/types.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "common/logger.h"
#include "communication/mavlink_handler.h"
#include "communication/px4_link.h"
#include "config/config.h"
#include "control/flight_controller.h"

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
    bool sitl_stage4_controller = false;
};

void PrintUsage(const char* program) {
    std::cout << "用法: " << program
              << " [--config <config.json>] [--duration <秒>]\n"
              << "       [--sitl-zero-velocity | --sitl-offboard-disarmed | --sitl-arm-zero-velocity\n"
              << "        | --sitl-takeoff-land | --sitl-horizontal-motion\n"
              << "        | --sitl-offboard-loss | --sitl-stage4-controller]\n"
              << "SITL 参数仅允许 UDP；offboard-loss 起飞 1m 后主动停止 setpoint，\n"
              << "验证 PX4 退出 Offboard，再统一 AUTO.LAND 回收。\n"
              << "stage4-controller 装配真实 FlightController（意图脚本 → 限幅/门禁 → setpoint），\n"
              << "依次验收：零速漂移、速度阶跃斜率/稳态、停发刹车、Offboard 退出时延。\n";
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
                                 static_cast<int>(options.sitl_stage4_controller);
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
/// 本工具不链接 drone_config（避免为 smoke 拖入视频/感知依赖），
/// 故只做 stage4 断言依赖的跨字段校验，其余由 FlightController 自身保证。
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
constexpr double kStage4SlopeTolerance = 1.2;          // 爬升斜率采样容差倍数
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
constexpr uint64_t kStage4OffboardExitLimitMs = 1300;  // IDLE→PX4 退出 Offboard 上限
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

/// 阶跃段北向速度样本（2.5 秒 × 100ms，量级极小）。
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
          subscription_(std::move(subscription)) {}

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
        const uint64_t coast_limit_ms =
            static_cast<uint64_t>(controller_config_.coast_timeout.count());
        const bool brake_ok = coast_seen_ && brake_zero_seen_ &&
                              brake_zero_ms_ - coast_ms_ < coast_limit_ms;
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
        PrintCheck("阶跃爬升斜率受限(≤max_accel×1.2)", step_complete_ && step_slope_ok_, true);
        PrintCheck("阶跃稳态速度=min(2, max_speed)", step_complete_ && step_steady_ok_, true);
        PrintCheck("停发后 COAST 零速刹车已触发", coast_seen_, true);
        PrintCheck("速度过零时间 < coast_timeout", brake_ok, true);
        PrintCheck("控制器已转 IDLE 停发", idle_seen_, true);
        PrintCheck("PX4 已在 1.3s 内退出 Offboard", exit_ok, true);
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
                  << " m（上限 " << kStage4ZeroDriftLimitM << "），阶跃爬升斜率 "
                  << step_max_slope_mps2_ << " m/s²（上限 " << slope_limit
                  << "），阶跃稳态速度 " << step_steady_speed_mps_ << " m/s（期望 "
                  << expected_steady << "），阶跃段位移 北向 " << step_north_m_
                  << " m / 东西 " << step_east_m_ << " m" << std::endl;
        std::cout << "实测时延: COAST→速度过零 "
                  << (coast_seen_ && brake_zero_seen_ ? brake_zero_ms_ - coast_ms_ : 0)
                  << " ms（上限 " << coast_limit_ms << "），COAST→IDLE "
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
        step_max_slope_mps2_ = MaxRampSlope();
        step_steady_speed_mps_ = SteadySpeed();
        const double expected =
            std::min<double>(kStage4StepSpeedMps, controller_config_.max_speed_mps);
        step_slope_ok_ =
            step_max_slope_mps2_ <= controller_config_.max_accel_mps2 * kStage4SlopeTolerance;
        step_steady_ok_ = std::fabs(step_steady_speed_mps_ - expected) <=
                          kStage4SteadyToleranceMps;
        clamped_delta_in_step_ = controller_.ClampedCount() - clamped_at_step_start_;
        std::cout << "[SITL] 阶跃段完成：爬升斜率 " << step_max_slope_mps2_
                  << " m/s²，稳态速度 " << step_steady_speed_mps_ << " m/s（期望 "
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

    /// 阶跃段最大爬升斜率：以 300ms 窗口差分抑制单拍噪声。
    double MaxRampSlope() const {
        constexpr std::size_t kWindow = 3;
        double max_slope = 0.0;
        for (std::size_t index = kWindow; index < speed_samples_.size(); ++index) {
            const Stage4SpeedSample& head = speed_samples_[index - kWindow];
            const Stage4SpeedSample& tail = speed_samples_[index];
            const uint64_t dt_ms = tail.time_ms - head.time_ms;
            if (dt_ms == 0) {
                continue;
            }
            const double slope = (static_cast<double>(tail.vx_mps) -
                                  static_cast<double>(head.vx_mps)) /
                                 (static_cast<double>(dt_ms) / 1000.0);
            max_slope = std::max(max_slope, slope);
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
        if ((sitl_setpoint_test || options.sitl_stage4_controller) &&
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
                  << (options.sitl_stage4_controller
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
                                        : "仅发送 HEARTBEAT 和遥测请求命令，不发送飞行控制指令")))))))
                  << '\n'
                  << std::endl;

        drone::communication::Px4Link link(config);
        if (options.sitl_stage4_controller) {
            // stage4 独立装配（意图主题 + FlightController + Px4Link）与独立报告，
            // 与 stage 0~3D 的直发 setpoint 路径完全分离。
            return RunStage4(link, root, options);
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
