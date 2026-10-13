#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "communication/ground_station_link.h"
#include "communication/px4_link.h"
#include "control/tracking_config.h"
#include "control/visual_tracking_shadow.h"
#include "perception/stereo_ranger_core.h"
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
    bool enable_visual_tracking = false;  ///< 视觉跟踪控制律影子（kControlIntent 无消费者）
    bool enable_stereo_ranging = false;  ///< 双目测距影子（依赖 uvc+stereo_split+视觉跟踪）
    /// 双目视觉观测 → 局部 NED 融合进目标估计器（影子；依赖 target_estimator +
    /// stereo_ranging + px4；不延展地面站有效期，docs/物理追踪思路.md §6.3）
    bool enable_visual_position_fusion = false;
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

/// FlightController 配置（controller 配置节，阶段四控制接入）。
/// 全部字段可选，缺席回退默认值；Validate 做范围与跨字段校验，
/// 非法值在配置加载期 fail-fast（抛 std::invalid_argument，消息带字段名）。
struct FlightControllerConfig {
    std::chrono::milliseconds period{50};        // 控制周期
    double max_speed_mps = 2.0;                  // 速度幅值上限
    double max_accel_mps2 = 2.0;                 // 加速度上限（slew）
    double max_jerk_mps3 = 0.0;                  // jerk 上限；0=不启用（v1 预留）
    double max_altitude_m = 30.0;                // 围栏：相对 Home 最大高度
    double max_radius_m = 100.0;                 // 围栏：相对 Home 最大水平半径
    std::chrono::milliseconds intent_timeout{300};   // 意图超龄阈值
    std::chrono::milliseconds coast_timeout{1000};   // COAST 最长保持
    std::chrono::milliseconds slew_transition{500};  // 无扰衔接斜坡时长
    void Validate() const;
};

/// 交接缝配置（handover 配置节，阶段四控制接入）。
/// 视觉锁定↔卡尔曼跟踪的状态切换迟滞：exit_lost_frames 必须大于 lock_frames。
struct HandoverConfig {
    uint32_t lock_frames = 10;          // S5→S6：LOCKED 连续帧数
    double fow_margin_deg = 5.0;        // 视线角视场余量
    double innovation_gate_m = 3.0;     // 视觉观测与卡尔曼预测创新门限（米）
    std::chrono::milliseconds timeout{3000};  // S6 驻留超时
    uint32_t exit_lost_frames = 30;     // S7 退出：LOST 连续帧数（必须 > lock_frames）
    void Validate() const;
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
    control::TrackingConfig visual_tracking;              ///< 视觉跟踪控制律（含标定内参）
    control::VisualTrackingShadowConfig visual_tracking_shadow;
    perception::StereoRangerConfig stereo_ranger;  ///< 双目测距 v2（稀疏 Census，影子）
    /// 预置校正矩阵：由 stereo_ranger.rectify_matrices_path 指向的 JSON 加载
    /// （基准=SBS右半=物理左目；重标定后必须重跑 export_rectify.py）。
    perception::StereoRectifyMatrices stereo_rectify;
    FlightControllerConfig controller;  ///< FlightController 参数（阶段四控制接入）
    HandoverConfig handover;            ///< 交接缝迟滞参数（阶段四控制接入）
};

/// 按“可执行文件旁 config/config.json → 当前目录 config/config.json”查找配置。
std::string ResolveConfigPath(const std::string& executable_directory,
                              const std::string& requested_path = {});

/// 加载并校验正式主程序配置。资源相对路径按可执行文件目录解析。
AppConfig LoadAppConfig(const std::string& path,
                        const std::string& executable_directory);

}  // namespace drone::config
