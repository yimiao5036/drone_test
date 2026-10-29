#include "config/config.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

namespace drone::config {
namespace {

using json = nlohmann::json;

// ============================================================================
// config —— 正式主程序配置文件加载与校验。
//
// 职责：
//   - 从 JSON 读取日志、runtime 开关、视频、YOLO、PX4、地面站等配置节；
//   - 统一做类型/范围/一致性校验（身份、端口、超时、队列容量、开关约束）；
//   - 将资源相对路径按可执行文件目录解析为绝对路径（如模型、日志目录）；
//   - 对 video.output_rtsp 的身份占位符做展开（如 /drone_{component}_{system}）。
// 边界：只做解析与校验，不创建任何设备/线程，不读取业务运行期状态。
// ============================================================================

// 读取 JSON 文件并解析为 nlohmann::json 对象；失败时抛带上下文的运行时异常。
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

// 读取 1~255 之间的整数（常用于 MAVLink system/component ID 与数据位）。
uint8_t ReadUint8(const json& object, const char* key) {
    const int value = object.at(key).get<int>();
    if (value <= 0 || value > 255) {
        throw std::invalid_argument(std::string(key) + " 必须在 1~255 之间");
    }
    return static_cast<uint8_t>(value);
}

// 读取合法 UDP/TCP 端口（1~65535）。
uint16_t ReadPort(const json& object, const char* key) {
    const int value = object.at(key).get<int>();
    if (value <= 0 || value > 65535) {
        throw std::invalid_argument(std::string(key) + " 必须在 1~65535 之间");
    }
    return static_cast<uint16_t>(value);
}

// 读取正数毫秒值，用于各类心跳/遥测/超时/发送间隔。
std::chrono::milliseconds ReadPositiveMilliseconds(const json& object,
                                                   const char* key) {
    const int64_t value = object.at(key).get<int64_t>();
    if (value <= 0) {
        throw std::invalid_argument(std::string(key) + " 必须为正数");
    }
    return std::chrono::milliseconds(value);
}

// 允许为零的毫秒时长（bootstrap_lead、低电防抖等），显式提供时不得为负。
std::chrono::milliseconds ReadOptionalNonNegativeMilliseconds(
    const json& object, const char* key, std::chrono::milliseconds default_value) {
    if (object.find(key) == object.end()) {
        return default_value;
    }
    const int64_t value = object.at(key).get<int64_t>();
    if (value < 0) {
        throw std::invalid_argument(std::string(key) + " 必须 >= 0");
    }
    return std::chrono::milliseconds(value);
}

// 健康阈值允许整个 health 配置节缺省，但只要显式提供字段就必须为正数。
std::chrono::milliseconds ReadOptionalPositiveMilliseconds(
    const json& object, const char* key, std::chrono::milliseconds default_value) {
    if (object.find(key) == object.end()) {
        return default_value;
    }
    return ReadPositiveMilliseconds(object, key);
}

// 目标估计器噪声参数允许配置节缺省；显式提供时必须为有限正数。
double ReadOptionalPositiveDouble(const json& object, const char* key,
                                  double default_value) {
    if (object.find(key) == object.end()) {
        return default_value;
    }
    const double value = object.at(key).get<double>();
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(key) + " 必须为有限正数");
    }
    return value;
}

// 视觉跟踪 PID 等增益允许为零（整定前占位）；显式提供时必须为有限非负数。
double ReadOptionalNonNegativeDouble(const json& object, const char* key,
                                     double default_value) {
    if (object.find(key) == object.end()) {
        return default_value;
    }
    const double value = object.at(key).get<double>();
    if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(std::string(key) + " 必须为有限非负数");
    }
    return value;
}

std::size_t ReadOptionalPositiveSize(const json& object, const char* key,
                                     std::size_t default_value) {
    if (object.find(key) == object.end()) {
        return default_value;
    }
    const int64_t value = object.at(key).get<int64_t>();
    if (value <= 0) {
        throw std::invalid_argument(std::string(key) + " 必须为正数");
    }
    return static_cast<std::size_t>(value);
}

int ReadOptionalPositiveInt(const json& object, const char* key,
                            int default_value) {
    if (object.find(key) == object.end()) {
        return default_value;
    }
    const int64_t value = object.at(key).get<int64_t>();
    if (value <= 0 || value > std::numeric_limits<int>::max()) {
        throw std::invalid_argument(std::string(key) + " 必须为正整数");
    }
    return static_cast<int>(value);
}

// 读取 3×3 / 3×4 矩阵字段；形状不符抛异常。
void ReadMatrixField(const json& node, const char* field, double* out, int rows,
                     int cols) {
    if (!node.contains(field) || !node.at(field).is_array() ||
        node.at(field).size() != static_cast<std::size_t>(rows)) {
        throw std::invalid_argument(
            std::string("校正矩阵缺少字段或形状不符: ") + field);
    }
    for (int r = 0; r < rows; ++r) {
        const json& row = node.at(field).at(r);
        if (!row.is_array() || row.size() != static_cast<std::size_t>(cols)) {
            throw std::invalid_argument(
                std::string("校正矩阵字段形状不符: ") + field);
        }
        for (int c = 0; c < cols; ++c) {
            out[r * cols + c] = row.at(c).get<double>();
        }
    }
}

// 读取畸变系数向量；形状不符抛异常。
void ReadCoeffsField(const json& node, const char* field, double* out,
                     int count) {
    if (!node.contains(field) || !node.at(field).is_array() ||
        node.at(field).size() != static_cast<std::size_t>(count)) {
        throw std::invalid_argument(
            std::string("校正矩阵缺少字段或形状不符: ") + field);
    }
    for (int i = 0; i < count; ++i) {
        out[i] = node.at(field).at(i).get<double>();
    }
}

// 预置校正矩阵加载（stereo_ranger.rectify_matrices_path）。
// 基准 = SBS 右半 = 物理左目 = 主工程左目 Topic；由
// tools/camera_calibration/export_rectify.py 生成，重标定后必须重跑。
// 字段缺失/非法、view 尺寸或焦距/基线非法、fx_rect/fxb 与 P 不自洽时抛异常。
perception::StereoRectifyMatrices LoadStereoRectifyMatrices(
    const std::string& path) {
    json root;
    try {
        root = LoadJson(path);
    } catch (const std::exception& error) {
        throw std::invalid_argument("校正矩阵文件不可用: " + path + " (" +
                                    error.what() + ")");
    }
    perception::StereoRectifyMatrices m;
    try {
        m.view_width = root.at("view_width").get<int>();
        m.view_height = root.at("view_height").get<int>();
        m.baseline_m = root.at("baseline_m").get<double>();
        m.fx_rect_px = root.at("fx_rect_px").get<double>();
        m.fxb_m_px = root.at("fxb_m_px").get<double>();
        ReadMatrixField(root, "K_ref", m.k_ref, 3, 3);
        ReadCoeffsField(root, "D_ref", m.d_ref, 5);
        ReadMatrixField(root, "R_ref", m.r_ref, 3, 3);
        ReadMatrixField(root, "P_ref", m.p_ref, 3, 4);
        ReadMatrixField(root, "K_aux", m.k_aux, 3, 3);
        ReadCoeffsField(root, "D_aux", m.d_aux, 5);
        ReadMatrixField(root, "R_aux", m.r_aux, 3, 3);
        ReadMatrixField(root, "P_aux", m.p_aux, 3, 4);
    } catch (const std::exception& error) {
        throw std::invalid_argument("校正矩阵字段缺失/非法: " + path + " (" +
                                    error.what() + ")");
    }
    if (m.view_width <= 0 || m.view_height <= 0) {
        throw std::invalid_argument("校正矩阵 view 尺寸非法: " + path);
    }
    if (!(m.k_ref[0] > 0.0) || !(m.k_aux[0] > 0.0) || !(m.p_ref[0] > 0.0) ||
        !(m.baseline_m > 0.0)) {
        throw std::invalid_argument("校正矩阵焦距/基线非法: " + path);
    }
    // 自洽校验：fxb 必须等于 fx_rect × baseline（防导出脚本与字段不一致）
    const double expected = m.fx_rect_px * m.baseline_m;
    if (std::abs(m.p_ref[0] - m.fx_rect_px) > 1e-6 * std::abs(m.fx_rect_px) ||
        std::abs(m.fxb_m_px - expected) > 1e-6 * std::abs(expected)) {
        throw std::invalid_argument(
            "校正矩阵 fx_rect/fxb 与 P 不自洽（重跑 export_rectify.py）: " + path);
    }
    return m;
}

// 配置文件所在目录（用于解析与 config.json 同目录的伴随文件）。
std::string ConfigDirectoryOf(const std::string& config_path) {
    const std::size_t pos = config_path.find_last_of("/\\");
    if (pos == std::string::npos) {
        return ".";
    }
    return pos == 0 ? config_path.substr(0, 1) : config_path.substr(0, pos);
}

// 伴随文件路径解析：绝对路径（以 / 开头）或空串原样返回，否则相对
// config.json 所在目录拼接（生产部署中 config.json 在 <可执行文件>/config/）。
std::string ResolveAdjacentPath(const std::string& config_directory,
                                const std::string& value) {
    if (value.empty() || value.front() == '/') {
        return value;
    }
    return config_directory + "/" + value;
}

// 资源相对路径解析：绝对路径（以 / 开头）或空串原样返回，
// 否则拼接在可执行文件目录后，避免依赖启动时的当前工作目录。
std::string ResolveAssetPath(const std::string& value,
                             const std::string& executable_directory) {
    if (value.empty() || value.front() == '/') {
        return value;
    }
    return executable_directory + '/' + value;
}

// uint8_t 转十进制字符串，用于输出身份占位符替换。
std::string ToString(uint8_t value) {
    return std::to_string(static_cast<unsigned int>(value));
}

// 字符串原地替换所有出现位置（用于占位符展开）。
void ReplaceAll(std::string& value, const std::string& from,
                const std::string& to) {
    std::size_t position = 0;
    while ((position = value.find(from, position)) != std::string::npos) {
        value.replace(position, from.size(), to);
        position += to.size();
    }
}

// 判断图传输出地址是否包含任一身份占位符。
bool HasIdentityPlaceholder(const std::string& value) {
    return value.find("{aircraft_system_id}") != std::string::npos ||
           value.find("{aircraft_component_id}") != std::string::npos ||
           value.find("{aircraft_number}") != std::string::npos;
}

// 图传 RTSP 输出地址展开：把身份占位符替换为地面站配置中的二元身份/编号，
// 使不同飞机的推流路径彼此区分（如 /drone_25_1）。含占位符却无地面站配置则报错。
std::string ResolveVideoOutputRtsp(
    std::string value,
    const communication::GroundStationLinkConfig* ground_station) {
    if (!HasIdentityPlaceholder(value)) {
        return value;
    }
    if (ground_station == nullptr) {
        throw std::invalid_argument(
            "video.output_rtsp使用身份占位符时必须提供ground_station配置节");
    }
    ReplaceAll(value, "{aircraft_system_id}",
               ToString(ground_station->aircraft_system_id));
    ReplaceAll(value, "{aircraft_component_id}",
               ToString(ground_station->aircraft_component_id));
    ReplaceAll(value, "{aircraft_number}", ToString(ground_station->aircraft_number));
    return value;
}

// 解析地面站配置节：身份、心跳、时间同步、目标协议、遥测发送周期、串口参数。
communication::GroundStationLinkConfig ParseGroundStationConfig(
    const json& root) {
    const json& ground = root.at("ground_station");
    const json& serial = ground.at("serial");
    const json& rates = ground.at("send_interval_ms");
    const json& time_sync = ground.at("time_sync");
    const json& target_input = ground.at("target_input");

    communication::GroundStationLinkConfig config;
    // 飞机二元身份：system=同类型编号，component=功能类别（网捕25/火箭26）。
    config.aircraft_system_id = ReadUint8(ground, "aircraft_system_id");
    config.aircraft_component_id = ReadUint8(ground, "aircraft_component_id");
    config.aircraft_type = ground.at("aircraft_type").get<std::string>();
    config.aircraft_number = ReadUint8(ground, "aircraft_number");
    config.callsign = ground.at("callsign").get<std::string>();
    // 地面站固定来源，生产为 255/190（MAV_TYPE_GCS 心跳）。
    config.ground_system_id = ReadUint8(ground, "ground_system_id");
    config.ground_component_id = ReadUint8(ground, "ground_component_id");
    config.mavlink_version = ReadUint8(ground, "mavlink_version");
    config.heartbeat_send_interval =
        ReadPositiveMilliseconds(ground, "heartbeat_send_interval_ms");
    config.heartbeat_timeout =
        ReadPositiveMilliseconds(ground, "heartbeat_timeout_ms");
    // TIMESYNC 第一阶段参数：采样周期、同步窗口、超时、RTT/offset 门限与样本窗口。
    config.enable_time_sync = time_sync.at("enabled").get<bool>();
    config.time_sync_acquire_interval =
        ReadPositiveMilliseconds(time_sync, "acquire_interval_ms");
    config.time_sync_steady_interval =
        ReadPositiveMilliseconds(time_sync, "steady_interval_ms");
    config.time_sync_timeout =
        ReadPositiveMilliseconds(time_sync, "sync_timeout_ms");
    config.time_sync_max_rtt =
        ReadPositiveMilliseconds(time_sync, "max_rtt_ms");
    config.time_sync_max_offset_jump =
        ReadPositiveMilliseconds(time_sync, "max_offset_jump_ms");
    // 样本数与窗口容量必须为正数，用于中位数估计与低 RTT 样本窗口。
    const int64_t minimum_samples =
        time_sync.at("minimum_samples").get<int64_t>();
    const int64_t window_capacity =
        time_sync.at("window_capacity").get<int64_t>();
    if (minimum_samples <= 0 || window_capacity <= 0) {
        throw std::invalid_argument("TIMESYNC样本数和窗口容量必须为正数");
    }
    config.time_sync_minimum_samples =
        static_cast<std::size_t>(minimum_samples);
    config.time_sync_window_capacity =
        static_cast<std::size_t>(window_capacity);
    // 目标 UPDATE/ACK 协议的有效期上下限、剩余有效/传输延迟/未来容忍门限。
    config.target_minimum_valid_for =
        ReadPositiveMilliseconds(target_input, "minimum_valid_for_ms");
    config.target_maximum_valid_for =
        ReadPositiveMilliseconds(target_input, "maximum_valid_for_ms");
    config.target_minimum_remaining_valid =
        ReadPositiveMilliseconds(target_input, "minimum_remaining_valid_ms");
    config.target_maximum_transport_delay =
        ReadPositiveMilliseconds(target_input, "maximum_transport_delay_ms");
    config.target_future_tolerance =
        ReadPositiveMilliseconds(target_input, "future_tolerance_ms");
    // 各类标准遥测的下行发送周期（毫秒），按 JSON 节流限频。
    config.attitude_send_interval = ReadPositiveMilliseconds(rates, "attitude");
    config.local_position_send_interval =
        ReadPositiveMilliseconds(rates, "local_position");
    config.global_position_send_interval =
        ReadPositiveMilliseconds(rates, "global_position");
    config.gps_send_interval = ReadPositiveMilliseconds(rates, "gps");
    config.extended_state_send_interval =
        ReadPositiveMilliseconds(rates, "extended_state");
    config.system_status_send_interval =
        ReadPositiveMilliseconds(rates, "system_status");
    config.battery_send_interval = ReadPositiveMilliseconds(rates, "battery");
    config.home_send_interval = ReadPositiveMilliseconds(rates, "home");
    const json status_rates = ground.value("status_send_interval_ms", json::object());
    config.health_status_send_interval = ReadOptionalPositiveMilliseconds(
        status_rates, "health", config.health_status_send_interval);
    config.mission_status_send_interval = ReadOptionalPositiveMilliseconds(
        status_rates, "mission", config.mission_status_send_interval);

    // 地面站订阅 PX4 FlightStateSnapshot 的队列容量。
    const int64_t queue_capacity =
        ground.at("flight_state_queue_capacity").get<int64_t>();
    if (queue_capacity <= 0) {
        throw std::invalid_argument("ground_station飞行状态队列容量必须为正数");
    }
    config.flight_state_queue_capacity =
        static_cast<std::size_t>(queue_capacity);

    // 地面站串口参数（如 /dev/ttyS6 @115200 8N1），parity 必须是单个字符 N/E/O。
    config.serial.device = serial.at("device").get<std::string>();
    config.serial.baud_rate = serial.at("baud_rate").get<int>();
    config.serial.data_bits = ReadUint8(serial, "data_bits");
    config.serial.stop_bits = ReadUint8(serial, "stop_bits");
    const std::string parity = serial.at("parity").get<std::string>();
    if (parity.size() != 1) {
        throw std::invalid_argument(
            "ground_station.serial.parity必须是单个字符N/E/O");
    }
    config.serial.parity = parity.front();
    config.serial.read_timeout =
        ReadPositiveMilliseconds(serial, "read_timeout_ms");
    config.serial.write_timeout =
        ReadPositiveMilliseconds(serial, "write_timeout_ms");
    config.Validate();
    return config;
}

// 解析 PX4 配置节：传输方式、身份、心跳/遥测/命令/setpoint 参数与串口/UDP。
communication::Px4LinkConfig ParsePx4Config(const json& root) {
    const json& px4 = root.at("px4");
    const json& serial = px4.at("serial");
    const json& udp = px4.at("udp");

    communication::Px4LinkConfig config;
    config.transport = px4.at("transport").get<std::string>();
    config.firmware_version = px4.at("firmware_version").get<std::string>();
    // 机载电脑身份（生产 1/191）与目标 PX4 身份（生产 1/1）。
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

    // setpoint 丢旧留新，command 有序队列；两者容量都必须为正数。
    const int64_t setpoint_capacity =
        px4.at("setpoint_queue_capacity").get<int64_t>();
    const int64_t command_capacity =
        px4.at("command_queue_capacity").get<int64_t>();
    if (setpoint_capacity <= 0 || command_capacity <= 0) {
        throw std::invalid_argument("PX4 setpoint/command队列容量必须为正数");
    }
    config.setpoint_queue_capacity = static_cast<std::size_t>(setpoint_capacity);
    config.command_queue_capacity = static_cast<std::size_t>(command_capacity);

    // 一次性遥测请求：AUTOPILOT_VERSION / HOME_POSITION 等。
    for (const auto& item : px4.at("one_shot_message_requests")) {
        const int64_t message_id = item.get<int64_t>();
        if (message_id < 0 || message_id > 0xFFFFFF) {
            throw std::invalid_argument("one_shot_message_requests包含非法消息ID");
        }
        config.one_shot_message_requests.push_back(static_cast<uint32_t>(message_id));
    }
    // 频率性遥测请求：按 message_id + interval_us 设置消息发送周期。
    for (const auto& item : px4.at("message_interval_requests")) {
        const int64_t message_id = item.at("message_id").get<int64_t>();
        const int64_t interval_us = item.at("interval_us").get<int64_t>();
        if (message_id < 0 || message_id > 0xFFFFFF || interval_us <= 0 ||
            interval_us > std::numeric_limits<int32_t>::max()) {
            throw std::invalid_argument("message_interval_requests包含非法ID或周期");
        }
        config.message_interval_requests.push_back(
            {static_cast<uint32_t>(message_id), static_cast<int32_t>(interval_us)});
    }

    // PX4 生产串口参数（/dev/ttyS1 @115200 8N1）。
    config.serial.device = serial.at("device").get<std::string>();
    config.serial.baud_rate = serial.at("baud_rate").get<int>();
    config.serial.data_bits = ReadUint8(serial, "data_bits");
    config.serial.stop_bits = ReadUint8(serial, "stop_bits");
    const std::string parity = serial.at("parity").get<std::string>();
    if (parity.size() != 1) {
        throw std::invalid_argument("px4.serial.parity必须是单个字符N/E/O");
    }
    config.serial.parity = parity.front();
    config.serial.read_timeout =
        ReadPositiveMilliseconds(serial, "read_timeout_ms");
    config.serial.write_timeout =
        ReadPositiveMilliseconds(serial, "write_timeout_ms");

    // SITL / 网络数传的 UDP 参数；生产默认 transport=serial 时此节仅作校验用。
    config.udp.bind_address = udp.at("bind_address").get<std::string>();
    config.udp.bind_port = ReadPort(udp, "bind_port");
    config.udp.remote_address = udp.at("remote_address").get<std::string>();
    config.udp.remote_port = ReadPort(udp, "remote_port");
    config.udp.read_timeout = ReadPositiveMilliseconds(udp, "read_timeout_ms");
    config.udp.write_timeout = ReadPositiveMilliseconds(udp, "write_timeout_ms");
    config.Validate();
    return config;
}

}  // namespace

void FlightControllerConfig::Validate() const {
    if (period.count() <= 0) throw std::invalid_argument("controller.period_ms 必须为正");
    if (max_speed_mps <= 0.0) throw std::invalid_argument("controller.max_speed_mps 必须为正");
    if (max_accel_mps2 <= 0.0) throw std::invalid_argument("controller.max_accel_mps2 必须为正");
    if (max_jerk_mps3 < 0.0) throw std::invalid_argument("controller.max_jerk_mps3 不得为负");
    if (max_altitude_m <= 0.0 || max_radius_m <= 0.0)
        throw std::invalid_argument("controller 围栏参数必须为正");
    if (!std::isfinite(max_feedforward_mps) || max_feedforward_mps <= 0.0)
        throw std::invalid_argument("controller.max_feedforward_mps 必须为有限正数");
    if (intent_timeout.count() <= 0)
        throw std::invalid_argument("controller.intent_timeout_ms 必须为正");
    if (coast_timeout.count() <= intent_timeout.count())
        throw std::invalid_argument("controller.coast_timeout_ms 必须大于 intent_timeout_ms");
    if (slew_transition.count() <= 0)
        throw std::invalid_argument("controller.slew_transition_ms 必须为正");
    if (bootstrap_timeout.count() <= 0)
        throw std::invalid_argument("controller.bootstrap_timeout_ms 必须为正");
    if (!std::isfinite(fence_margin_m) || fence_margin_m < 0.0)
        throw std::invalid_argument("controller.fence_margin_m 必须为有限非负数");
}

void HandoverConfig::Validate() const {
    if (lock_frames == 0) throw std::invalid_argument("handover.lock_frames 必须为正");
    if (exit_lost_frames <= lock_frames)
        throw std::invalid_argument("handover.exit_lost_frames 必须大于 lock_frames（迟滞）");
    if (fow_margin_deg <= 0.0) throw std::invalid_argument("handover.fow_margin_deg 必须为正");
    if (innovation_gate_m <= 0.0) throw std::invalid_argument("handover.innovation_gate_m 必须为正");
    if (timeout.count() <= 0) throw std::invalid_argument("handover.timeout_ms 必须为正");
    if (distance_lost_ms.count() <= 0)
        throw std::invalid_argument("handover.distance_lost_ms 必须为正");
}

void InterceptionConfig::Validate() const {
    if (lock_frames == 0) {
        throw std::invalid_argument("interception.lock_frames 必须为正");
    }
    if (!std::isfinite(min_range_m) || min_range_m <= 0.0) {
        throw std::invalid_argument("interception.min_range_m 必须为有限正数");
    }
    if (!std::isfinite(max_range_m) || max_range_m <= min_range_m) {
        throw std::invalid_argument(
            "interception.max_range_m 必须为有限正数且大于 min_range_m");
    }
    if (!std::isfinite(min_battery_percent) || min_battery_percent < 0.0 ||
        min_battery_percent > 100.0) {
        throw std::invalid_argument(
            "interception.min_battery_percent 必须在 [0,100]");
    }
    if (authorization_timeout.count() <= 0) {
        throw std::invalid_argument("interception.authorization_timeout_ms 必须为正");
    }
}

void ReturnHomeConfig::Validate() const {
    if (!std::isfinite(altitude_m) || altitude_m <= 0.0) {
        throw std::invalid_argument("return_home.altitude_m 必须为有限正数");
    }
    if (!std::isfinite(arrival_radius_m) || arrival_radius_m <= 0.0) {
        throw std::invalid_argument("return_home.arrival_radius_m 必须为有限正数");
    }
    if (!std::isfinite(arrival_vertical_m) || arrival_vertical_m <= 0.0) {
        throw std::invalid_argument("return_home.arrival_vertical_m 必须为有限正数");
    }
    if (!std::isfinite(arrival_speed_mps) || arrival_speed_mps <= 0.0) {
        throw std::invalid_argument("return_home.arrival_speed_mps 必须为有限正数");
    }
    if (arrival_hold.count() <= 0) {
        throw std::invalid_argument("return_home.arrival_hold_ms 必须为正");
    }
    if (timeout.count() <= 0) {
        throw std::invalid_argument("return_home.timeout_ms 必须为正");
    }
    if (fire_timeout.count() <= 0) {
        throw std::invalid_argument("return_home.fire_timeout_ms 必须为正");
    }
    if (command_retry.count() <= 0) {
        throw std::invalid_argument("return_home.command_retry_ms 必须为正");
    }
    // 返航机头模式（2026-10-29）：0=保持原机头 1=朝飞行方向；其它值拒绝。
    if (heading_mode > 1) {
        throw std::invalid_argument("return_home.heading_mode 必须为 0 或 1");
    }
    if (!std::isfinite(heading_rate_limit_dps) || heading_rate_limit_dps <= 0.0) {
        throw std::invalid_argument("return_home.heading_rate_limit_dps 必须为有限正数");
    }
    if (!std::isfinite(heading_accel_limit_dps2) || heading_accel_limit_dps2 <= 0.0) {
        throw std::invalid_argument("return_home.heading_accel_limit_dps2 必须为有限正数");
    }
}

void FenceConfig::Validate() const {
    if (zone_cache_path.empty()) {
        throw std::invalid_argument("fence.zone_cache_path 不能为空");
    }
    if (violation_debounce.count() < 0) {
        throw std::invalid_argument("fence.violation_debounce_ms 必须 >= 0");
    }
    if (!std::isfinite(boundary_margin_m) || boundary_margin_m < 0.0) {
        throw std::invalid_argument("fence.boundary_margin_m 必须为有限非负数");
    }
    if (!std::isfinite(boundary_warn_m) || boundary_warn_m < 0.0) {
        throw std::invalid_argument("fence.boundary_warn_m 必须为有限非负数");
    }
    if (boundary_warn_m < boundary_margin_m) {
        throw std::invalid_argument(
            "fence.boundary_warn_m 必须 >= fence.boundary_margin_m");
    }
    if (status_period.count() <= 0) {
        throw std::invalid_argument("fence.status_period_ms 必须为正");
    }
    const auto action = static_cast<uint8_t>(violation_action);
    if (action > static_cast<uint8_t>(common::FenceAction::kLand)) {
        throw std::invalid_argument("fence.violation_action 必须为 0/1/2");
    }
}

void TakeoffConfig::Validate() const {
    if (!std::isfinite(height_m) || height_m <= 0.0) {
        throw std::invalid_argument("takeoff.height_m 必须为有限正数");
    }
    if (!std::isfinite(climb_rate_mps) || climb_rate_mps <= 0.0) {
        throw std::invalid_argument("takeoff.climb_rate_mps 必须为有限正数");
    }
    // bootstrap_lead 允许为 0（起飞意图后立即发 176），其余超时必须为正。
    if (bootstrap_lead.count() < 0) {
        throw std::invalid_argument("takeoff.bootstrap_lead_ms 必须 >= 0");
    }
    if (mode_timeout.count() <= 0) {
        throw std::invalid_argument("takeoff.mode_timeout_ms 必须为正");
    }
    if (arm_timeout.count() <= 0) {
        throw std::invalid_argument("takeoff.arm_timeout_ms 必须为正");
    }
    if (bootstrap_timeout.count() <= 0) {
        throw std::invalid_argument("takeoff.bootstrap_timeout_ms 必须为正");
    }
    // Bootstrap 必须先于 Offboard/解锁命令超时结束，否则原点保持流会在
    // 模式/解锁流程尚未完成时失效。
    if (bootstrap_timeout.count() <= bootstrap_lead.count() +
                                        mode_timeout.count() +
                                        arm_timeout.count()) {
        throw std::invalid_argument(
            "takeoff.bootstrap_timeout_ms 必须大于 "
            "bootstrap_lead_ms+mode_timeout_ms+arm_timeout_ms");
    }
    if (height_timeout.count() <= 0) {
        throw std::invalid_argument("takeoff.height_timeout_ms 必须为正");
    }
    if (stable_time.count() <= 0) {
        throw std::invalid_argument("takeoff.stable_time_ms 必须为正");
    }
    if (!std::isfinite(z_tolerance_m) || z_tolerance_m < 0.0) {
        throw std::invalid_argument("takeoff.z_tolerance_m 必须为有限非负数");
    }
    if (!std::isfinite(xy_tolerance_m) || xy_tolerance_m < 0.0) {
        throw std::invalid_argument("takeoff.xy_tolerance_m 必须为有限非负数");
    }
    if (!std::isfinite(max_vz_mps) || max_vz_mps < 0.0) {
        throw std::invalid_argument("takeoff.max_vz_mps 必须为有限非负数");
    }
    if (target_wait_timeout.count() <= 0) {
        throw std::invalid_argument("takeoff.target_wait_timeout_ms 必须为正");
    }
    if (target_lost_timeout.count() <= 0) {
        throw std::invalid_argument("takeoff.target_lost_timeout_ms 必须为正");
    }
    if (target_hold_max.count() <= 0) {
        throw std::invalid_argument("takeoff.target_hold_max_ms 必须为正");
    }
    if (battery_low_debounce.count() < 0) {
        throw std::invalid_argument("takeoff.battery_low_debounce_ms 必须 >= 0");
    }
}

void ActuatorConfig::Validate() const {
    if (type != "single_pwm")
        throw std::invalid_argument(
            "actuator.type 目前仅支持 single_pwm（多发实现待接入）: " + type);
    if (net_channel < 1 || net_channel > 6)
        throw std::invalid_argument("actuator.net_channel 必须在 1~6（执行器组）之间");
    if (!std::isfinite(net_fire_norm) || net_fire_norm < -1.0 ||
        net_fire_norm > 1.0)
        throw std::invalid_argument("actuator.net_fire_norm 必须在 [-1, 1] 内");
    if (!std::isfinite(net_idle_norm) || net_idle_norm < -1.0 ||
        net_idle_norm > 1.0)
        throw std::invalid_argument("actuator.net_idle_norm 必须在 [-1, 1] 内");
    if (net_fire_norm == net_idle_norm)
        throw std::invalid_argument("actuator.net_fire_norm 与 net_idle_norm 不得相同");
    if (reset_delay.count() < 0)
        throw std::invalid_argument("actuator.reset_delay_ms 必须 >= 0");
    if (reset_settle.count() < 0)
        throw std::invalid_argument("actuator.reset_settle_ms 必须 >= 0");
    if (gcs_fire_cooldown.count() < 100 || gcs_fire_cooldown.count() > 60000)
        throw std::invalid_argument(
            "actuator.gcs_fire_cooldown_ms 必须在 100~60000 之间");
}

// 解析配置文件路径：优先取调用方显式指定的路径，其次找可执行文件旁的
// config/config.json，最后回退当前目录的 ./config/config.json；都失败则抛错。
// 这样从部署目录直接运行也能稳定加载配置，不依赖启动时的工作目录。
std::string ResolveConfigPath(const std::string& executable_directory,
                              const std::string& requested_path) {
    if (!requested_path.empty()) {
        std::ifstream requested(requested_path);
        if (!requested.is_open()) {
            throw std::runtime_error("无法打开配置文件: " + requested_path);
        }
        return requested_path;
    }

    const std::string beside_executable =
        executable_directory + "/config/config.json";
    std::ifstream first(beside_executable);
    if (first.is_open()) {
        return beside_executable;
    }

    const std::string current_directory = "./config/config.json";
    std::ifstream second(current_directory);
    if (second.is_open()) {
        return current_directory;
    }
    throw std::runtime_error("未找到config/config.json");
}

// 加载并校验正式主程序配置。整体流程：读 JSON → 逐节填充 → 交叉约束校验。
// 资源相对路径（模型、日志目录）按可执行文件目录解析。
AppConfig LoadAppConfig(const std::string& path,
                        const std::string& executable_directory) {
    const json root = LoadJson(path);
    AppConfig config;

    // 日志配置：输出目录与等级。
    const json log = root.value("log", json::object());
    config.log.directory = log.value("dir", std::string("logs/"));
    config.log.level = log.value("level", std::string("info"));

    // runtime 开关与一致性校验：控制依赖 PX4 与地面站（状态机意图/目标来源）、
    // 地面站依赖 PX4、控制装配由 enable_control 门禁接管（见下）。
    const json runtime = root.value("runtime", json::object());
    config.runtime.enable_video = runtime.value("enable_video", true);
    config.runtime.enable_px4 = runtime.value("enable_px4", true);
    config.runtime.enable_ground_station =
        runtime.value("enable_ground_station", false);
    config.runtime.enable_target_estimator =
        runtime.value("enable_target_estimator", false);
    config.runtime.enable_visual_monitor =
        runtime.value("enable_visual_monitor", false);
    config.runtime.enable_visual_tracking =
        runtime.value("enable_visual_tracking", false);
    config.runtime.enable_stereo_ranging =
        runtime.value("enable_stereo_ranging", false);
    config.runtime.enable_visual_position_fusion =
        runtime.value("enable_visual_position_fusion", false);
    config.runtime.enable_control = runtime.value("enable_control", false);
    if (!config.runtime.enable_video && !config.runtime.enable_px4 &&
        !config.runtime.enable_ground_station) {
        throw std::invalid_argument("runtime至少启用一个数据链路模块");
    }
    if (config.runtime.enable_control && !config.runtime.enable_px4) {
        throw std::invalid_argument("enable_control=true时必须启用PX4链路");
    }
    if (config.runtime.enable_ground_station && !config.runtime.enable_px4) {
        throw std::invalid_argument("当前地面站遥测链路启用时必须同时启用PX4链路");
    }
    if (config.runtime.enable_target_estimator &&
        (!config.runtime.enable_ground_station || !config.runtime.enable_px4)) {
        throw std::invalid_argument(
            "enable_target_estimator=true时必须同时启用地面站和PX4链路");
    }
    if (config.runtime.enable_visual_position_fusion &&
        (!config.runtime.enable_target_estimator ||
         !config.runtime.enable_stereo_ranging || !config.runtime.enable_px4)) {
        throw std::invalid_argument(
            "enable_visual_position_fusion=true时必须同时启用目标估计器、"
            "双目测距与PX4链路");
    }
    if (config.runtime.enable_visual_monitor && !config.runtime.enable_video) {
        throw std::invalid_argument(
            "enable_visual_monitor=true时必须启用视频链路（观测来源为YoloDetector）");
    }
    if (config.runtime.enable_visual_tracking &&
        (!config.runtime.enable_visual_monitor || !config.runtime.enable_video)) {
        throw std::invalid_argument(
            "enable_visual_tracking=true时必须启用视频链路与视觉稳定性判定"
            "（视觉输入来源为VisualTargetMonitor）");
    }
    // 阶段四装配门禁（部署门禁①）：enable_control=true 要求状态机依赖链齐备
    // （PX4+地面站，上面已分别校验 PX4；此处补地面站），controller 配置节在
    // 后段解析并无条件 Validate——非法即加载期 fail-fast，不静默降级。
    // 仓库生产 config.json 永远保持 false，打开只发生在现场部署目录本地副本。
    if (config.runtime.enable_control && !config.runtime.enable_ground_station) {
        throw std::invalid_argument(
            "enable_control=true时必须同时启用地面站链路（状态机意图与目标来源）");
    }

    const json health = root.value("health", json::object());
    config.health.cpu_sample_period = ReadOptionalPositiveMilliseconds(
        health, "cpu_sample_period_ms", config.health.cpu_sample_period);
    config.health.startup_grace_period = ReadOptionalPositiveMilliseconds(
        health, "startup_grace_ms", config.health.startup_grace_period);
    config.health.camera_max_age = ReadOptionalPositiveMilliseconds(
        health, "camera_max_age_ms", config.health.camera_max_age);
    config.health.decoder_max_age = ReadOptionalPositiveMilliseconds(
        health, "decoder_max_age_ms", config.health.decoder_max_age);
    config.health.yolo_max_age = ReadOptionalPositiveMilliseconds(
        health, "yolo_max_age_ms", config.health.yolo_max_age);
    config.health.video_max_age = ReadOptionalPositiveMilliseconds(
        health, "video_max_age_ms", config.health.video_max_age);
    config.health.px4_max_age = ReadOptionalPositiveMilliseconds(
        health, "px4_max_age_ms", config.health.px4_max_age);
    config.health.ground_station_max_age = ReadOptionalPositiveMilliseconds(
        health, "ground_station_max_age_ms", config.health.ground_station_max_age);

    // 单目标线性卡尔曼影子参数。数值为第一阶段初值，必须由录制数据复核整定。
    const json target_estimator =
        root.value("target_estimator", json::object());
    config.target_estimator.ground_target_queue_capacity =
        ReadOptionalPositiveSize(target_estimator, "ground_target_queue_capacity",
                                 config.target_estimator.ground_target_queue_capacity);
    config.target_estimator.flight_state_queue_capacity =
        ReadOptionalPositiveSize(target_estimator, "flight_state_queue_capacity",
                                 config.target_estimator.flight_state_queue_capacity);
    config.target_estimator.publish_interval =
        ReadOptionalPositiveMilliseconds(target_estimator, "publish_interval_ms",
                                         config.target_estimator.publish_interval);
    config.target_estimator.process_acceleration_std_mps2 =
        ReadOptionalPositiveDouble(target_estimator,
                                   "process_acceleration_std_mps2",
                                   config.target_estimator.process_acceleration_std_mps2);
    config.target_estimator.initial_velocity_std_mps =
        ReadOptionalPositiveDouble(target_estimator, "initial_velocity_std_mps",
                                   config.target_estimator.initial_velocity_std_mps);
    config.target_estimator.velocity_measurement_std_mps =
        ReadOptionalPositiveDouble(target_estimator,
                                   "velocity_measurement_std_mps",
                                   config.target_estimator.velocity_measurement_std_mps);
    config.target_estimator.default_horizontal_accuracy_m =
        ReadOptionalPositiveDouble(target_estimator,
                                   "default_horizontal_accuracy_m",
                                   config.target_estimator.default_horizontal_accuracy_m);
    config.target_estimator.default_vertical_accuracy_m =
        ReadOptionalPositiveDouble(target_estimator,
                                   "default_vertical_accuracy_m",
                                   config.target_estimator.default_vertical_accuracy_m);
    config.target_estimator.minimum_measurement_std_m =
        ReadOptionalPositiveDouble(target_estimator,
                                   "minimum_measurement_std_m",
                                   config.target_estimator.minimum_measurement_std_m);
    config.target_estimator.min_velocity_position_updates = ReadOptionalPositiveSize(
        target_estimator, "min_velocity_position_updates",
        config.target_estimator.min_velocity_position_updates);
    // 视觉融合子组（runtime.enable_visual_position_fusion 开启后生效；
    // enabled 与 fxb_m_px 在校正矩阵加载后装配填充并复核 Validate）。
    // mount_* 默认全 0 = 准直安装假设（未标定），允许 0/负值故直接 value()。
    const json visual_fusion =
        target_estimator.value("visual", json::object());
    auto& visual_cfg = config.target_estimator.visual;
    visual_cfg.queue_capacity = ReadOptionalPositiveSize(
        visual_fusion, "queue_capacity", visual_cfg.queue_capacity);
    visual_cfg.max_observation_age_ms = ReadOptionalPositiveMilliseconds(
        visual_fusion, "max_observation_age_ms",
        visual_cfg.max_observation_age_ms);
    visual_cfg.flight_state_max_age_ms = ReadOptionalPositiveMilliseconds(
        visual_fusion, "flight_state_max_age_ms",
        visual_cfg.flight_state_max_age_ms);
    visual_cfg.disparity_std_px = ReadOptionalPositiveDouble(
        visual_fusion, "disparity_std_px", visual_cfg.disparity_std_px);
    visual_cfg.min_confidence = ReadOptionalNonNegativeDouble(
        visual_fusion, "min_confidence", visual_cfg.min_confidence);
    visual_cfg.mount.roll_deg =
        visual_fusion.value("mount_roll_deg", visual_cfg.mount.roll_deg);
    visual_cfg.mount.pitch_deg =
        visual_fusion.value("mount_pitch_deg", visual_cfg.mount.pitch_deg);
    visual_cfg.mount.yaw_deg =
        visual_fusion.value("mount_yaw_deg", visual_cfg.mount.yaw_deg);
    visual_cfg.mount.x_m =
        visual_fusion.value("mount_x_m", visual_cfg.mount.x_m);
    visual_cfg.mount.y_m =
        visual_fusion.value("mount_y_m", visual_cfg.mount.y_m);
    visual_cfg.mount.z_m =
        visual_fusion.value("mount_z_m", visual_cfg.mount.z_m);
    config.target_estimator.Validate();

    // FlightController 参数（controller 节，阶段四控制接入）：全部字段可选，
    // 缺席回退默认值；Validate 做范围与跨字段校验（如 coast_timeout > intent_timeout）。
    // max_jerk_mps3 允许 0（不启用 jerk 限制，v1 预留），故直接 value() 后由
    // Validate 拦负值；slew_transition 语义统一为 >0（与加载路径
    // ReadOptionalPositiveMilliseconds 一致）。
    const json controller = root.value("controller", json::object());
    config.controller.period = ReadOptionalPositiveMilliseconds(
        controller, "period_ms", config.controller.period);
    config.controller.max_speed_mps = ReadOptionalPositiveDouble(
        controller, "max_speed_mps", config.controller.max_speed_mps);
    config.controller.max_accel_mps2 = ReadOptionalPositiveDouble(
        controller, "max_accel_mps2", config.controller.max_accel_mps2);
    config.controller.max_jerk_mps3 =
        controller.value("max_jerk_mps3", config.controller.max_jerk_mps3);
    config.controller.max_altitude_m = ReadOptionalPositiveDouble(
        controller, "max_altitude_m", config.controller.max_altitude_m);
    config.controller.max_radius_m = ReadOptionalPositiveDouble(
        controller, "max_radius_m", config.controller.max_radius_m);
    config.controller.enable_position_velocity_feedforward =
        controller.value("enable_position_velocity_feedforward",
                         config.controller.enable_position_velocity_feedforward);
    config.controller.max_feedforward_mps = ReadOptionalPositiveDouble(
        controller, "max_feedforward_mps", config.controller.max_feedforward_mps);
    config.controller.intent_timeout = ReadOptionalPositiveMilliseconds(
        controller, "intent_timeout_ms", config.controller.intent_timeout);
    config.controller.coast_timeout = ReadOptionalPositiveMilliseconds(
        controller, "coast_timeout_ms", config.controller.coast_timeout);
    config.controller.slew_transition = ReadOptionalPositiveMilliseconds(
        controller, "slew_transition_ms", config.controller.slew_transition);
    config.controller.Validate();

    // 交接缝迟滞参数（handover 节，阶段四控制接入）：exit_lost_frames 必须
    // 大于 lock_frames，避免 S6/S7 边界抖动。
    const json handover = root.value("handover", json::object());
    config.handover.lock_frames = handover.value("lock_frames", config.handover.lock_frames);
    config.handover.fow_margin_deg = ReadOptionalPositiveDouble(
        handover, "fow_margin_deg", config.handover.fow_margin_deg);
    config.handover.innovation_gate_m = ReadOptionalPositiveDouble(
        handover, "innovation_gate_m", config.handover.innovation_gate_m);
    config.handover.timeout = ReadOptionalPositiveMilliseconds(
        handover, "timeout_ms", config.handover.timeout);
    config.handover.exit_lost_frames =
        handover.value("exit_lost_frames", config.handover.exit_lost_frames);
    config.handover.distance_lost_ms = ReadOptionalPositiveMilliseconds(
        handover, "distance_lost_ms", config.handover.distance_lost_ms);
    config.handover.Validate();

    // 拦截入口/授权（interception 节，2026-10-19）：默认关闭；射程上限已按
    // 双目测距极限冻结为 8m（2026-10-23），下限仍占位；现场本地副本开启前
    // 还须确认授权策略。
    const json interception = root.value("interception", json::object());
    config.interception.enabled =
        interception.value("enabled", config.interception.enabled);
    config.interception.auto_authorize = interception.value(
        "auto_authorize", config.interception.auto_authorize);
    config.interception.lock_frames = static_cast<uint32_t>(
        ReadOptionalPositiveSize(interception, "lock_frames",
                                 config.interception.lock_frames));
    config.interception.min_range_m = ReadOptionalPositiveDouble(
        interception, "min_range_m", config.interception.min_range_m);
    config.interception.max_range_m = ReadOptionalPositiveDouble(
        interception, "max_range_m", config.interception.max_range_m);
    config.interception.min_battery_percent = ReadOptionalNonNegativeDouble(
        interception, "min_battery_percent",
        config.interception.min_battery_percent);
    config.interception.authorization_timeout = ReadOptionalPositiveMilliseconds(
        interception, "authorization_timeout_ms",
        config.interception.authorization_timeout);
    config.interception.Validate();

    // 拦截后返航（return_home 节，S11~S14）：全部可选，缺席回退默认值。
    const json return_home = root.value("return_home", json::object());
    config.return_home.enabled =
        return_home.value("enabled", config.return_home.enabled);
    config.return_home.altitude_m = ReadOptionalPositiveDouble(
        return_home, "altitude_m", config.return_home.altitude_m);
    config.return_home.arrival_radius_m = ReadOptionalPositiveDouble(
        return_home, "arrival_radius_m", config.return_home.arrival_radius_m);
    config.return_home.arrival_vertical_m = ReadOptionalPositiveDouble(
        return_home, "arrival_vertical_m", config.return_home.arrival_vertical_m);
    config.return_home.arrival_speed_mps = ReadOptionalPositiveDouble(
        return_home, "arrival_speed_mps", config.return_home.arrival_speed_mps);
    config.return_home.arrival_hold = ReadOptionalPositiveMilliseconds(
        return_home, "arrival_hold_ms", config.return_home.arrival_hold);
    config.return_home.timeout = ReadOptionalPositiveMilliseconds(
        return_home, "timeout_ms", config.return_home.timeout);
    config.return_home.fire_timeout = ReadOptionalPositiveMilliseconds(
        return_home, "fire_timeout_ms", config.return_home.fire_timeout);
    config.return_home.command_retry = ReadOptionalPositiveMilliseconds(
        return_home, "command_retry_ms", config.return_home.command_retry);
    // 返航机头朝飞行方向（2026-10-29）：模式 0=保持 1=朝速度方向，
    // 并限制机头转速/角加速度（与视觉跟踪同量级，避免突转）。
    config.return_home.heading_mode = static_cast<uint8_t>(
        return_home.value("heading_mode", static_cast<int>(config.return_home.heading_mode)));
    config.return_home.heading_rate_limit_dps = ReadOptionalPositiveDouble(
        return_home, "heading_rate_limit_dps",
        config.return_home.heading_rate_limit_dps);
    config.return_home.heading_accel_limit_dps2 = ReadOptionalPositiveDouble(
        return_home, "heading_accel_limit_dps2",
        config.return_home.heading_accel_limit_dps2);
    config.return_home.Validate();
    if (config.interception.enabled && !config.return_home.enabled) {
        throw std::invalid_argument(
            "interception.enabled=true 时 return_home.enabled 必须为 true"
            "（发射后必须有返航收口）");
    }

    // 自动起飞（takeoff 节，2026-10-22）：S3/S4 时序、到达判据与目标收口；
    // 全部可选，缺席回退默认值。低电门限不使用本地电量百分比，唯一来源是
    // PX4 BAT_LOW_THR（后续任务接入），因此本节点没有 min_battery_percent。
    const json takeoff = root.value("takeoff", json::object());
    config.takeoff.height_m = ReadOptionalPositiveDouble(
        takeoff, "height_m", config.takeoff.height_m);
    config.takeoff.climb_rate_mps = ReadOptionalPositiveDouble(
        takeoff, "climb_rate_mps", config.takeoff.climb_rate_mps);
    config.takeoff.bootstrap_lead = ReadOptionalNonNegativeMilliseconds(
        takeoff, "bootstrap_lead_ms", config.takeoff.bootstrap_lead);
    config.takeoff.mode_timeout = ReadOptionalPositiveMilliseconds(
        takeoff, "mode_timeout_ms", config.takeoff.mode_timeout);
    config.takeoff.arm_timeout = ReadOptionalPositiveMilliseconds(
        takeoff, "arm_timeout_ms", config.takeoff.arm_timeout);
    config.takeoff.bootstrap_timeout = ReadOptionalPositiveMilliseconds(
        takeoff, "bootstrap_timeout_ms", config.takeoff.bootstrap_timeout);
    config.takeoff.height_timeout = ReadOptionalPositiveMilliseconds(
        takeoff, "height_timeout_ms", config.takeoff.height_timeout);
    config.takeoff.stable_time = ReadOptionalPositiveMilliseconds(
        takeoff, "stable_time_ms", config.takeoff.stable_time);
    config.takeoff.z_tolerance_m = ReadOptionalNonNegativeDouble(
        takeoff, "z_tolerance_m", config.takeoff.z_tolerance_m);
    config.takeoff.xy_tolerance_m = ReadOptionalNonNegativeDouble(
        takeoff, "xy_tolerance_m", config.takeoff.xy_tolerance_m);
    config.takeoff.max_vz_mps = ReadOptionalNonNegativeDouble(
        takeoff, "max_vz_mps", config.takeoff.max_vz_mps);
    config.takeoff.target_wait_timeout = ReadOptionalPositiveMilliseconds(
        takeoff, "target_wait_timeout_ms", config.takeoff.target_wait_timeout);
    config.takeoff.target_lost_timeout = ReadOptionalPositiveMilliseconds(
        takeoff, "target_lost_timeout_ms", config.takeoff.target_lost_timeout);
    config.takeoff.target_hold_max = ReadOptionalPositiveMilliseconds(
        takeoff, "target_hold_max_ms", config.takeoff.target_hold_max);
    config.takeoff.battery_low_debounce = ReadOptionalNonNegativeMilliseconds(
        takeoff, "battery_low_debounce_ms",
        config.takeoff.battery_low_debounce);
    config.takeoff.Validate();

    // 多边形围栏（fence 节，2026-10-29）：全部可选，缺省关闭并回退 Home 圆形围栏。
    const json fence = root.value("fence", json::object());
    config.fence.enabled = fence.value("enabled", config.fence.enabled);
    config.fence.zone_cache_path =
        fence.value("zone_cache_path", config.fence.zone_cache_path);
    config.fence.violation_debounce = ReadOptionalNonNegativeMilliseconds(
        fence, "violation_debounce_ms", config.fence.violation_debounce);
    config.fence.boundary_margin_m = ReadOptionalNonNegativeDouble(
        fence, "boundary_margin_m", config.fence.boundary_margin_m);
    config.fence.boundary_warn_m = ReadOptionalNonNegativeDouble(
        fence, "boundary_warn_m", config.fence.boundary_warn_m);
    config.fence.follow_zone_action =
        fence.value("follow_zone_action", config.fence.follow_zone_action);
    config.fence.violation_action = static_cast<common::FenceAction>(
        fence.value("violation_action",
                    static_cast<int>(config.fence.violation_action)));
    config.fence.status_period = ReadOptionalPositiveMilliseconds(
        fence, "status_period_ms", config.fence.status_period);
    config.fence.report_period = ReadOptionalPositiveMilliseconds(
        fence, "report_period_ms", config.fence.report_period);
    config.fence.default_half_side_m = ReadOptionalNonNegativeDouble(
        fence, "default_half_side_m", config.fence.default_half_side_m);
    config.fence.default_alt_min_m = ReadOptionalNonNegativeDouble(
        fence, "default_alt_min_m", config.fence.default_alt_min_m);
    config.fence.default_alt_max_m = ReadOptionalNonNegativeDouble(
        fence, "default_alt_max_m", config.fence.default_alt_max_m);
    config.fence.default_zone_id = static_cast<uint16_t>(
        fence.value("default_zone_id", static_cast<int>(config.fence.default_zone_id)));
    config.fence.default_version = static_cast<uint16_t>(
        fence.value("default_version", static_cast<int>(config.fence.default_version)));
    config.fence.default_action = static_cast<common::FenceAction>(
        fence.value("default_action", static_cast<int>(config.fence.default_action)));
    config.fence.Validate();
    // 控制器钳制余量与围栏配置保持单一真源：FlightController 不读 fence 节。
    config.controller.fence_margin_m = config.fence.boundary_margin_m;
    config.controller.Validate();

    // 捕网执行器（actuator 节，阶段四控制接入 Task 10）：机载只保存
    // Actuator Set N 与归一化控制量；PWM 输出功能/区间/舵机行程在 PX4 侧配置。
    const json actuator = root.value("actuator", json::object());
    config.actuator.type =
        actuator.value("type", config.actuator.type);
    config.actuator.net_channel =
        actuator.value("net_channel", config.actuator.net_channel);
    config.actuator.net_fire_norm =
        actuator.value("net_fire_norm", config.actuator.net_fire_norm);
    config.actuator.net_idle_norm =
        actuator.value("net_idle_norm", config.actuator.net_idle_norm);
    config.actuator.reset_delay = std::chrono::milliseconds(
        actuator.value("reset_delay_ms",
                       static_cast<int>(config.actuator.reset_delay.count())));
    config.actuator.reset_settle = std::chrono::milliseconds(
        actuator.value("reset_settle_ms",
                       static_cast<int>(config.actuator.reset_settle.count())));
    config.actuator.gcs_fire_enabled =
        actuator.value("gcs_fire_enabled", config.actuator.gcs_fire_enabled);
    config.actuator.gcs_fire_cooldown = std::chrono::milliseconds(
        actuator.value("gcs_fire_cooldown_ms",
                       static_cast<int>(config.actuator.gcs_fire_cooldown.count())));
    config.actuator.Validate();
    // 直控发射（65016）与自动拦截发射共用同一个执行器，两条路径不得同时开启：
    // 装配期 fail-fast，避免地面站直控与状态机 S10 各建一个实例重复点火。
    if (config.actuator.gcs_fire_enabled && config.interception.enabled) {
        throw std::invalid_argument(
            "actuator.gcs_fire_enabled 与 interception.enabled 不得同时为 true"
            "（地面站直控发射与自动拦截发射互斥）");
    }

    // 单目标视觉稳定性判定：帧级观测定节拍，门限为影子初值。
    const json visual_monitor = root.value("visual_monitor", json::object());
    config.visual_monitor.input_queue_capacity = ReadOptionalPositiveSize(
        visual_monitor, "input_queue_capacity",
        config.visual_monitor.input_queue_capacity);
    config.visual_monitor.publish_interval = ReadOptionalPositiveMilliseconds(
        visual_monitor, "publish_interval_ms",
        config.visual_monitor.publish_interval);
    config.visual_monitor.observation_timeout = ReadOptionalPositiveMilliseconds(
        visual_monitor, "observation_timeout_ms",
        config.visual_monitor.observation_timeout);
    config.visual_monitor.lock_frames = ReadOptionalPositiveInt(
        visual_monitor, "lock_frames", config.visual_monitor.lock_frames);
    config.visual_monitor.lost_frames = ReadOptionalPositiveInt(
        visual_monitor, "lost_frames", config.visual_monitor.lost_frames);
    config.visual_monitor.smoothing_alpha = ReadOptionalPositiveDouble(
        visual_monitor, "smoothing_alpha", config.visual_monitor.smoothing_alpha);
    if (config.visual_monitor.smoothing_alpha > 1.0) {
        throw std::invalid_argument("visual_monitor.smoothing_alpha必须在(0,1]");
    }
    config.visual_monitor.transition_log_interval =
        ReadOptionalPositiveMilliseconds(visual_monitor,
                                         "transition_log_interval_ms",
                                         config.visual_monitor.transition_log_interval);
    config.visual_monitor.status_log_interval =
        ReadOptionalPositiveMilliseconds(visual_monitor, "status_log_interval_ms",
                                         config.visual_monitor.status_log_interval);
    config.visual_monitor.Validate();

    // 双目测距 v2（稀疏 Census，影子）：全部字段可选，默认值即参考实现实测配置。
    // rectify_matrices_path 按可执行文件目录解析（与模型路径同规则）；
    // 矩阵文件由 tools/camera_calibration/export_rectify.py 生成，构造期加载并校验。
    const json stereo_ranger = root.value("stereo_ranger", json::object());
    // 存原始值，待依赖链校验后再按 config.json 同目录解析（见文件后段）
    config.stereo_ranger.rectify_matrices_path = stereo_ranger.value(
        "rectify_matrices_path", config.stereo_ranger.rectify_matrices_path);
    config.stereo_ranger.disp_min_px = ReadOptionalPositiveDouble(
        stereo_ranger, "disp_min_px", config.stereo_ranger.disp_min_px);
    config.stereo_ranger.disp_max_px = ReadOptionalPositiveDouble(
        stereo_ranger, "disp_max_px", config.stereo_ranger.disp_max_px);
    config.stereo_ranger.coarse_stride = ReadOptionalPositiveInt(
        stereo_ranger, "coarse_stride", config.stereo_ranger.coarse_stride);
    config.stereo_ranger.fine_halfwidth = ReadOptionalPositiveInt(
        stereo_ranger, "fine_halfwidth", config.stereo_ranger.fine_halfwidth);
    config.stereo_ranger.uniqueness_ratio = ReadOptionalPositiveDouble(
        stereo_ranger, "uniqueness_ratio", config.stereo_ranger.uniqueness_ratio);
    config.stereo_ranger.lr_tol_px =
        stereo_ranger.value("lr_tol_px", config.stereo_ranger.lr_tol_px);
    config.stereo_ranger.census_win = ReadOptionalPositiveInt(
        stereo_ranger, "census_win", config.stereo_ranger.census_win);
    config.stereo_ranger.max_points_per_target = ReadOptionalPositiveInt(
        stereo_ranger, "max_points_per_target",
        config.stereo_ranger.max_points_per_target);
    config.stereo_ranger.grid_cells = ReadOptionalPositiveInt(
        stereo_ranger, "grid_cells", config.stereo_ranger.grid_cells);
    config.stereo_ranger.warmup_frames =
        stereo_ranger.value("warmup_frames", config.stereo_ranger.warmup_frames);
    config.stereo_ranger.black_frame_mean_threshold =
        stereo_ranger.value("black_frame_mean_threshold",
                            config.stereo_ranger.black_frame_mean_threshold);
    config.stereo_ranger.bbox_erode_px =
        stereo_ranger.value("bbox_erode_px", config.stereo_ranger.bbox_erode_px);
    config.stereo_ranger.min_points = ReadOptionalPositiveInt(
        stereo_ranger, "min_points", config.stereo_ranger.min_points);
    config.stereo_ranger.near_pct =
        stereo_ranger.value("near_pct", config.stereo_ranger.near_pct);
    config.stereo_ranger.near_tol_px =
        stereo_ranger.value("near_tol_px", config.stereo_ranger.near_tol_px);
    config.stereo_ranger.smooth_alpha =
        stereo_ranger.value("smooth_alpha", config.stereo_ranger.smooth_alpha);
    // 诊断开关默认 false，不在生产热路径留诊断调用（同 yolo.collect_* 约定）
    config.stereo_ranger.collect_detailed_latency = stereo_ranger.value(
        "collect_detailed_latency", config.stereo_ranger.collect_detailed_latency);
    config.stereo_ranger.input_queue_capacity = ReadOptionalPositiveSize(
        stereo_ranger, "input_queue_capacity",
        config.stereo_ranger.input_queue_capacity);
    config.stereo_ranger.frame_buffer_depth = ReadOptionalPositiveSize(
        stereo_ranger, "frame_buffer_depth",
        config.stereo_ranger.frame_buffer_depth);
    config.stereo_ranger.pair_window = ReadOptionalPositiveMilliseconds(
        stereo_ranger, "pair_window_ms", config.stereo_ranger.pair_window);
    config.stereo_ranger.settle = ReadOptionalPositiveMilliseconds(
        stereo_ranger, "settle_ms", config.stereo_ranger.settle);
    config.stereo_ranger.summary_log_interval = ReadOptionalPositiveMilliseconds(
        stereo_ranger, "summary_log_interval_ms",
        config.stereo_ranger.summary_log_interval);
    config.stereo_ranger.Validate();

    // 视觉跟踪控制律（影子）：全部字段可选，缺席用默认值（camera 组默认值即
    // 2026-09-23 标定实测）；Validate 做跨字段校验。
    const json visual_tracking = root.value("visual_tracking", json::object());
    {
        const json control = visual_tracking.value("control", json::object());
        config.visual_tracking.control.frequency_hz = ReadOptionalPositiveDouble(
            control, "frequency_hz", config.visual_tracking.control.frequency_hz);
        config.visual_tracking.control.visual_stale_ms = ReadOptionalPositiveInt(
            control, "visual_stale_ms",
            static_cast<int>(config.visual_tracking.control.visual_stale_ms));
        config.visual_tracking.control.distance_stale_ms = ReadOptionalPositiveInt(
            control, "distance_stale_ms",
            static_cast<int>(config.visual_tracking.control.distance_stale_ms));
        config.visual_tracking.control.attitude_stale_ms = ReadOptionalPositiveInt(
            control, "attitude_stale_ms",
            static_cast<int>(config.visual_tracking.control.attitude_stale_ms));
        const json camera = visual_tracking.value("camera", json::object());
        config.visual_tracking.camera.image_width = ReadOptionalPositiveInt(
            camera, "image_width", config.visual_tracking.camera.image_width);
        config.visual_tracking.camera.image_height = ReadOptionalPositiveInt(
            camera, "image_height", config.visual_tracking.camera.image_height);
        config.visual_tracking.camera.fx_px =
            camera.value("fx_px", config.visual_tracking.camera.fx_px);
        config.visual_tracking.camera.fy_px =
            camera.value("fy_px", config.visual_tracking.camera.fy_px);
        config.visual_tracking.camera.cx_px =
            camera.value("cx_px", config.visual_tracking.camera.cx_px);
        config.visual_tracking.camera.cy_px =
            camera.value("cy_px", config.visual_tracking.camera.cy_px);
        const json heading = visual_tracking.value("heading", json::object());
        const std::string heading_mode = heading.value("mode", std::string("rate"));
        if (heading_mode == "rate") {
            config.visual_tracking.heading.mode = control::HeadingMode::kRate;
        } else if (heading_mode == "position") {
            config.visual_tracking.heading.mode = control::HeadingMode::kPosition;
        } else {
            throw std::invalid_argument(
                "visual_tracking.heading.mode 仅支持 rate/position");
        }
        config.visual_tracking.heading.gain = ReadOptionalNonNegativeDouble(
            heading, "gain", config.visual_tracking.heading.gain);
        config.visual_tracking.heading.yaw_rate_limit_dps =
            ReadOptionalPositiveDouble(heading, "yaw_rate_limit_dps",
                                       config.visual_tracking.heading.yaw_rate_limit_dps);
        config.visual_tracking.heading.yaw_slew_limit_dps2 =
            ReadOptionalPositiveDouble(heading, "yaw_slew_limit_dps2",
                                       config.visual_tracking.heading.yaw_slew_limit_dps2);
        const json vertical = visual_tracking.value("vertical", json::object());
        config.visual_tracking.vertical.vz_gain_mps_per_deg =
            ReadOptionalNonNegativeDouble(
                vertical, "vz_gain_mps_per_deg",
                config.visual_tracking.vertical.vz_gain_mps_per_deg);
        config.visual_tracking.vertical.vz_limit_mps = ReadOptionalPositiveDouble(
            vertical, "vz_limit_mps", config.visual_tracking.vertical.vz_limit_mps);
        const json distance = visual_tracking.value("distance", json::object());
        config.visual_tracking.distance.d_exp_m = ReadOptionalPositiveDouble(
            distance, "d_exp_m", config.visual_tracking.distance.d_exp_m);
        config.visual_tracking.distance.kp = ReadOptionalNonNegativeDouble(
            distance, "kp", config.visual_tracking.distance.kp);
        config.visual_tracking.distance.ki = ReadOptionalNonNegativeDouble(
            distance, "ki", config.visual_tracking.distance.ki);
        config.visual_tracking.distance.kd = ReadOptionalNonNegativeDouble(
            distance, "kd", config.visual_tracking.distance.kd);
        config.visual_tracking.distance.integral_limit = ReadOptionalNonNegativeDouble(
            distance, "integral_limit", config.visual_tracking.distance.integral_limit);
        config.visual_tracking.distance.derivative_filter_coef =
            distance.value("derivative_filter_coef",
                           config.visual_tracking.distance.derivative_filter_coef);
        config.visual_tracking.distance.approach_velocity_limit_mps =
            ReadOptionalPositiveDouble(
                distance, "approach_velocity_limit_mps",
                config.visual_tracking.distance.approach_velocity_limit_mps);
        config.visual_tracking.distance.retreat_velocity_limit_mps =
            ReadOptionalPositiveDouble(
                distance, "retreat_velocity_limit_mps",
                config.visual_tracking.distance.retreat_velocity_limit_mps);
        const std::string no_distance_action =
            distance.value("no_distance_action", std::string("slow_approach"));
        if (no_distance_action == "hold") {
            config.visual_tracking.distance.no_distance_action =
                control::NoDistanceAction::kHold;
        } else if (no_distance_action == "slow_approach") {
            config.visual_tracking.distance.no_distance_action =
                control::NoDistanceAction::kSlowApproach;
        } else if (no_distance_action == "exit") {
            config.visual_tracking.distance.no_distance_action =
                control::NoDistanceAction::kExit;
        } else {
            throw std::invalid_argument(
                "visual_tracking.distance.no_distance_action 仅支持 "
                "hold/slow_approach/exit");
        }
        config.visual_tracking.distance.no_distance_approach_limit_mps =
            ReadOptionalNonNegativeDouble(
                distance, "no_distance_approach_limit_mps",
                config.visual_tracking.distance.no_distance_approach_limit_mps);
        const json accel_limit = visual_tracking.value("accel_limit", json::object());
        config.visual_tracking.accel_limit.ax_mps2 = ReadOptionalPositiveDouble(
            accel_limit, "ax_mps2", config.visual_tracking.accel_limit.ax_mps2);
        config.visual_tracking.accel_limit.ay_mps2 = ReadOptionalPositiveDouble(
            accel_limit, "ay_mps2", config.visual_tracking.accel_limit.ay_mps2);
        config.visual_tracking.accel_limit.az_mps2 = ReadOptionalPositiveDouble(
            accel_limit, "az_mps2", config.visual_tracking.accel_limit.az_mps2);
        const json shadow = visual_tracking.value("shadow", json::object());
        config.visual_tracking_shadow.virtual_attitude_when_absent =
            shadow.value("virtual_attitude_when_absent",
                         config.visual_tracking_shadow.virtual_attitude_when_absent);
        config.visual_tracking_shadow.summary_log_interval =
            ReadOptionalPositiveMilliseconds(
                shadow, "summary_log_interval_ms",
                config.visual_tracking_shadow.summary_log_interval);
        config.visual_tracking.Validate();
        config.visual_tracking_shadow.Validate();
    }

    if (root.find("ground_station") != root.end()) {
        config.ground_station = ParseGroundStationConfig(root);
    } else if (config.runtime.enable_ground_station) {
        throw std::invalid_argument(
            "enable_ground_station=true时必须提供ground_station配置节");
    }

    const json video = root.value("video", json::object());

    // 输入源二选一：rtsp（现有单目链路，默认）| uvc（USB 双目 SBS 链路）。
    config.video_source.camera_source =
        video.value("camera_source", std::string("rtsp"));
    if (config.video_source.camera_source != "rtsp" &&
        config.video_source.camera_source != "uvc") {
        throw std::invalid_argument("video.camera_source仅支持rtsp/uvc");
    }
    config.video_source.stereo_split = video.value("stereo_split", false);
    if (config.video_source.stereo_split &&
        config.video_source.camera_source != "uvc") {
        throw std::invalid_argument(
            "stereo_split=true时camera_source必须为uvc（拆分依赖USB双目源）");
    }

    // UVC 双目采集参数；camera_source=uvc 时强制 SBS 偶数宽（左右拆分前提）。
    config.uvc_camera.device = video.value("uvc_device", std::string("/dev/video0"));
    config.uvc_camera.width = static_cast<std::uint32_t>(
        ReadOptionalPositiveInt(video, "uvc_width", 2560));
    config.uvc_camera.height = static_cast<std::uint32_t>(
        ReadOptionalPositiveInt(video, "uvc_height", 720));
    config.uvc_camera.fps = static_cast<std::uint32_t>(
        ReadOptionalPositiveInt(video, "uvc_fps", 30));
    if (config.video_source.camera_source == "uvc" &&
        (config.uvc_camera.width % 2) != 0) {
        throw std::invalid_argument("camera_source=uvc时video.uvc_width必须为偶数");
    }
    config.stereo_splitter.width = config.uvc_camera.width;
    config.stereo_splitter.height = config.uvc_camera.height;
    // UVC 链路解码器预知 SBS 全幅分辨率：ffmpeg-rockchip 8.1 的 mjpeg_rkmpp
    // 在送包路径按 avctx->width×height 预分配输出缓冲，宽高为 0 会首包即失败
    // 且永远触发不了 info_change（详见 video_decoder.cpp TryCreateCodec 注释）。
    // RTSP 链路保持 0，分辨率由码流参数集到达后再确定。
    if (config.video_source.camera_source == "uvc") {
        config.decoder.width = config.uvc_camera.width;
        config.decoder.height = config.uvc_camera.height;
    }

    // 双目测距依赖链：右目帧来自 uvc 拆分，距离消费者是视觉跟踪影子。
    if (config.runtime.enable_stereo_ranging &&
        (!config.runtime.enable_video ||
         config.video_source.camera_source != "uvc" ||
         !config.video_source.stereo_split ||
         !config.runtime.enable_visual_tracking)) {
        throw std::invalid_argument(
            "enable_stereo_ranging=true时必须启用视频链路、camera_source=uvc、"
            "stereo_split=true且启用视觉跟踪影子");
    }
    // 预置校正矩阵：仅启用双目测距时加载（禁用时不依赖该文件）；路径与
    // config.json 同目录解析（生产部署即 <可执行文件>/config/）。缺失/不自洽
    // 即启动失败，避免带病影子运行。
    if (config.runtime.enable_stereo_ranging) {
        config.stereo_ranger.rectify_matrices_path = ResolveAdjacentPath(
            ConfigDirectoryOf(path), config.stereo_ranger.rectify_matrices_path);
        config.stereo_rectify =
            LoadStereoRectifyMatrices(config.stereo_ranger.rectify_matrices_path);
    }
    // 围栏区域缓存路径与 config.json 同目录解析（生产部署即 <exe>/config/）。
    // 缓存缺失不阻塞启动：机载按 kNoZone 上报，等地面站 65018 下发落盘。
    if (config.fence.enabled) {
        config.fence.zone_cache_path =
            ResolveAdjacentPath(ConfigDirectoryOf(path), config.fence.zone_cache_path);
    }
    // 视觉融合装配：enabled 由 runtime 开关写入；fxb_m_px 从校正矩阵拷贝
    //（深度系数单一真源，不进 JSON）；填充后复核 Validate（enabled 时要求
    // fxb>0）。
    if (config.runtime.enable_visual_position_fusion) {
        config.target_estimator.visual.enabled = true;
        config.target_estimator.visual.fxb_m_px = config.stereo_rectify.fxb_m_px;
        config.target_estimator.Validate();
    }

    config.camera.rtsp_url = video.value(
        "input_rtsp", std::string("rtsp://192.168.1.100:8554/live"));
    config.camera.rtsp_transport = video.value("rtsp_transport", std::string("tcp"));
    config.camera.open_timeout = std::chrono::milliseconds(
        video.value("open_timeout_ms", 5000));
    config.camera.reconnect_delay = std::chrono::milliseconds(
        video.value("reconnect_delay_ms", 3000));

    config.decoder.pool_capacity = static_cast<std::size_t>(
        video.value("decoder_pool_capacity", 16));
    config.decoder.stride_alignment = static_cast<uint32_t>(
        video.value("stride_alignment", 64));
    config.decoder.prefer_hardware = video.value("prefer_hardware_decode", true);
    config.decoder.prefer_rga_dma_transfer =
        video.value("prefer_rga_dma_transfer", false);

    const json yolo = root.value("yolo", json::object());
    config.yolo.model_path = ResolveAssetPath(
        yolo.value("model_path", std::string("models/yolo26n-int8.rknn")),
        executable_directory);
    config.yolo.conf_threshold = yolo.value("conf_threshold", 0.25f);
    config.yolo.nms_threshold = yolo.value("nms_threshold", 0.45f);
    config.yolo.input_queue_capacity = static_cast<std::size_t>(
        yolo.value("input_queue_capacity", 2));
    config.yolo.npu_core_mode = yolo.value("npu_core_mode", std::string("all"));
    config.yolo.collect_npu_internal_perf =
        yolo.value("collect_npu_internal_perf", false);
    config.yolo.collect_npu_perf_detail =
        yolo.value("collect_npu_perf_detail", false);
    if (config.yolo.npu_core_mode != "auto" &&
        config.yolo.npu_core_mode != "core0" &&
        config.yolo.npu_core_mode != "core01" &&
        config.yolo.npu_core_mode != "core012" &&
        config.yolo.npu_core_mode != "all") {
        throw std::invalid_argument(
            "yolo.npu_core_mode仅支持auto/core0/core01/core012/all");
    }

    config.compositor.pool_capacity = static_cast<std::size_t>(
        video.value("compositor_pool_capacity", 8));
    config.compositor.stride_alignment = config.decoder.stride_alignment;
    config.compositor.draw_text = true;
    config.compositor.max_detection_frame_lag =
        yolo.value("max_detection_frame_lag", 10ULL);
    const auto class_names = yolo.find("class_names");
    if (class_names != yolo.end()) {
        if (!class_names->is_array() || class_names->empty()) {
            throw std::invalid_argument("yolo.class_names必须是非空字符串数组");
        }
        config.compositor.class_names.clear();
        for (const auto& item : *class_names) {
            if (!item.is_string() || item.get<std::string>().empty()) {
                throw std::invalid_argument("yolo.class_names必须是非空字符串数组");
            }
            config.compositor.class_names.push_back(item.get<std::string>());
        }
    }

    const std::string default_output_rtsp =
        root.find("ground_station") != root.end()
            ? "rtsp://127.0.0.1:8554/drone_{aircraft_component_id}_{aircraft_system_id}"
            : "rtsp://127.0.0.1:8554/drone_out";
    config.video_sender.encode.url = ResolveVideoOutputRtsp(
        video.value("output_rtsp", default_output_rtsp),
        root.find("ground_station") != root.end() ? &config.ground_station : nullptr);
    config.video_sender.encode.codec = video.value("output_codec", std::string("h264"));
    config.video_sender.encode.prefer_hardware =
        video.value("prefer_hardware_encode", true);
    config.video_sender.encode.width = static_cast<uint32_t>(video.value("width", 1920));
    config.video_sender.encode.height = static_cast<uint32_t>(video.value("height", 1080));
    config.video_sender.encode.fps = video.value("fps", 25);
    config.video_sender.encode.bitrate = video.value("bitrate", 6LL * 1024 * 1024);
    config.video_sender.encode.gop =
        video.value("gop", config.video_sender.encode.fps * 2);
    config.video_sender.input_queue = static_cast<std::size_t>(
        video.value("sender_input_queue", 2));

    config.px4 = ParsePx4Config(root);
    return config;
}

}  // namespace drone::config
