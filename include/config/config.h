#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "communication/ground_station_link.h"
#include "communication/px4_link.h"
#include "perception/yolo_detector.h"
#include "video/camera_receiver.h"
#include "video/frame_compositor.h"
#include "video/video_decoder.h"
#include "video_transmission/video_sender.h"

namespace drone::config {

struct LogConfig {
    std::string directory = "logs/";
    std::string level = "info";
};

struct RuntimeConfig {
    bool enable_video = true;
    bool enable_px4 = true;
    bool enable_ground_station = false;
    bool enable_control = false;
};

struct AppConfig {
    LogConfig log;
    RuntimeConfig runtime;
    video::CameraReceiverConfig camera;
    video::VideoDecoderConfig decoder;
    perception::YoloDetectorConfig yolo;
    video::CompositorConfig compositor;
    video_transmission::VideoSenderConfig video_sender;
    communication::Px4LinkConfig px4;
    communication::GroundStationLinkConfig ground_station;
};

/// 按“可执行文件旁 config/config.json → 当前目录 config/config.json”查找配置。
std::string ResolveConfigPath(const std::string& executable_directory,
                              const std::string& requested_path = {});

/// 加载并校验正式主程序配置。资源相对路径按可执行文件目录解析。
AppConfig LoadAppConfig(const std::string& path,
                        const std::string& executable_directory);

}  // namespace drone::config
