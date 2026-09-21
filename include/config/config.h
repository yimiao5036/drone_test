#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "communication/ground_station_link.h"
#include "communication/px4_link.h"
#include "perception/target_estimator.h"
#include "perception/visual_target_monitor.h"
#include "perception/yolo_detector.h"
#include "video/camera_receiver.h"
#include "video/frame_compositor.h"
#include "video/stereo_frame_splitter.h"
#include "video/uvc_camera_receiver.h"
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
    bool enable_target_estimator = false;
    bool enable_visual_monitor = false;
    bool enable_control = false;
};

/// HealthManager 数据源最大允许无数据时间。各值必须为正数。
struct HealthManagerConfig {
    std::chrono::milliseconds cpu_sample_period{1000};
    std::chrono::milliseconds startup_grace_period{5000};
    std::chrono::milliseconds camera_max_age{1000};
    std::chrono::milliseconds decoder_max_age{1000};
    std::chrono::milliseconds yolo_max_age{1000};
    std::chrono::milliseconds video_max_age{1000};
    std::chrono::milliseconds px4_max_age{3000};
    std::chrono::milliseconds ground_station_max_age{3000};
};

/// 视频输入源选择（video 配置节）：RTSP 单目链路与 USB UVC 双目链路二选一。
struct VideoSourceConfig {
    std::string camera_source = "rtsp";  ///< rtsp | uvc
    bool stereo_split = false;           ///< uvc 时是否装配 StereoFrameSplitter 左右拆分
};

struct AppConfig {
    LogConfig log;
    RuntimeConfig runtime;
    HealthManagerConfig health;
    VideoSourceConfig video_source;
    video::CameraReceiverConfig camera;
    video::UvcCameraReceiverConfig uvc_camera;
    video::VideoDecoderConfig decoder;
    video::StereoFrameSplitterConfig stereo_splitter;
    perception::YoloDetectorConfig yolo;
    video::CompositorConfig compositor;
    video_transmission::VideoSenderConfig video_sender;
    communication::Px4LinkConfig px4;
    communication::GroundStationLinkConfig ground_station;
    perception::TargetEstimatorConfig target_estimator;
    perception::VisualTargetMonitorConfig visual_monitor;
};

/// 按“可执行文件旁 config/config.json → 当前目录 config/config.json”查找配置。
std::string ResolveConfigPath(const std::string& executable_directory,
                              const std::string& requested_path = {});

/// 加载并校验正式主程序配置。资源相对路径按可执行文件目录解析。
AppConfig LoadAppConfig(const std::string& path,
                        const std::string& executable_directory);

}  // namespace drone::config
