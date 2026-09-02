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

uint16_t ReadPort(const json& object, const char* key) {
    const int value = object.at(key).get<int>();
    if (value <= 0 || value > 65535) {
        throw std::invalid_argument(std::string(key) + " 必须在 1~65535 之间");
    }
    return static_cast<uint16_t>(value);
}

std::chrono::milliseconds ReadPositiveMilliseconds(const json& object,
                                                   const char* key) {
    const int64_t value = object.at(key).get<int64_t>();
    if (value <= 0) {
        throw std::invalid_argument(std::string(key) + " 必须为正数");
    }
    return std::chrono::milliseconds(value);
}

std::string ResolveAssetPath(const std::string& value,
                             const std::string& executable_directory) {
    if (value.empty() || value.front() == '/') {
        return value;
    }
    return executable_directory + '/' + value;
}

communication::GroundStationLinkConfig ParseGroundStationConfig(
    const json& root) {
    const json& ground = root.at("ground_station");
    const json& serial = ground.at("serial");
    const json& rates = ground.at("send_interval_ms");

    communication::GroundStationLinkConfig config;
    config.aircraft_system_id = ReadUint8(ground, "aircraft_system_id");
    config.aircraft_component_id = ReadUint8(ground, "aircraft_component_id");
    config.aircraft_type = ground.at("aircraft_type").get<std::string>();
    config.aircraft_number = ReadUint8(ground, "aircraft_number");
    config.callsign = ground.at("callsign").get<std::string>();
    config.ground_system_id = ReadUint8(ground, "ground_system_id");
    config.ground_component_id = ReadUint8(ground, "ground_component_id");
    config.mavlink_version = ReadUint8(ground, "mavlink_version");
    config.heartbeat_send_interval =
        ReadPositiveMilliseconds(ground, "heartbeat_send_interval_ms");
    config.heartbeat_timeout =
        ReadPositiveMilliseconds(ground, "heartbeat_timeout_ms");
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

    const int64_t queue_capacity =
        ground.at("flight_state_queue_capacity").get<int64_t>();
    if (queue_capacity <= 0) {
        throw std::invalid_argument("ground_station飞行状态队列容量必须为正数");
    }
    config.flight_state_queue_capacity =
        static_cast<std::size_t>(queue_capacity);

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

communication::Px4LinkConfig ParsePx4Config(const json& root) {
    const json& px4 = root.at("px4");
    const json& serial = px4.at("serial");
    const json& udp = px4.at("udp");

    communication::Px4LinkConfig config;
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

    const int64_t setpoint_capacity =
        px4.at("setpoint_queue_capacity").get<int64_t>();
    const int64_t command_capacity =
        px4.at("command_queue_capacity").get<int64_t>();
    if (setpoint_capacity <= 0 || command_capacity <= 0) {
        throw std::invalid_argument("PX4 setpoint/command队列容量必须为正数");
    }
    config.setpoint_queue_capacity = static_cast<std::size_t>(setpoint_capacity);
    config.command_queue_capacity = static_cast<std::size_t>(command_capacity);

    for (const auto& item : px4.at("one_shot_message_requests")) {
        const int64_t message_id = item.get<int64_t>();
        if (message_id < 0 || message_id > 0xFFFFFF) {
            throw std::invalid_argument("one_shot_message_requests包含非法消息ID");
        }
        config.one_shot_message_requests.push_back(static_cast<uint32_t>(message_id));
    }
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

AppConfig LoadAppConfig(const std::string& path,
                        const std::string& executable_directory) {
    const json root = LoadJson(path);
    AppConfig config;

    const json log = root.value("log", json::object());
    config.log.directory = log.value("dir", std::string("logs/"));
    config.log.level = log.value("level", std::string("info"));

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

    config.video_sender.encode.url = video.value(
        "output_rtsp", std::string("rtsp://127.0.0.1:8554/drone_out"));
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
    if (root.find("ground_station") != root.end()) {
        config.ground_station = ParseGroundStationConfig(root);
    } else if (config.runtime.enable_ground_station) {
        throw std::invalid_argument(
            "enable_ground_station=true时必须提供ground_station配置节");
    }
    return config;
}

}  // namespace drone::config
