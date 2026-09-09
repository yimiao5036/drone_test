#include "config/config.h"

#include <chrono>
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

// 健康阈值允许整个 health 配置节缺省，但只要显式提供字段就必须为正数。
std::chrono::milliseconds ReadOptionalPositiveMilliseconds(
    const json& object, const char* key, std::chrono::milliseconds default_value) {
    if (object.find(key) == object.end()) {
        return default_value;
    }
    return ReadPositiveMilliseconds(object, key);
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

    // runtime 开关与一致性校验：控制依赖 PX4、地面站依赖 PX4、控制装配未开放。
    const json runtime = root.value("runtime", json::object());
    config.runtime.enable_video = runtime.value("enable_video", true);
    config.runtime.enable_px4 = runtime.value("enable_px4", true);
    config.runtime.enable_ground_station =
        runtime.value("enable_ground_station", false);
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
    if (config.runtime.enable_control) {
        throw std::invalid_argument("正式控制装配尚未开放，enable_control必须为false");
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

    if (root.find("ground_station") != root.end()) {
        config.ground_station = ParseGroundStationConfig(root);
    } else if (config.runtime.enable_ground_station) {
        throw std::invalid_argument(
            "enable_ground_station=true时必须提供ground_station配置节");
    }

    const json video = root.value("video", json::object());
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
