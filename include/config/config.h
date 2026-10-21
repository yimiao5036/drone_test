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
    // 位置型意图速度前馈（P3，2026-10-19）：开关默认关闭；上限独立于
    // max_speed_mps（后者只约束速度式意图），水平模长/垂直分量分别钳制。
    bool enable_position_velocity_feedforward = false;
    double max_feedforward_mps = 2.0;
    std::chrono::milliseconds intent_timeout{300};   // 意图超龄阈值
    std::chrono::milliseconds coast_timeout{1000};   // COAST 最长保持
    std::chrono::milliseconds slew_transition{500};  // 无扰衔接斜坡时长
    void Validate() const;
};

/// 捕网执行器配置（actuator 配置节，阶段四控制接入 Task 10）。
/// 机载只负责发送 MAV_CMD_DO_SET_ACTUATOR 的归一化控制量（-1..1）；
/// PWM 通道功能、PWM_AUX_MINn/MAXn、舵机行程与方向在 PX4/QGC 侧配置，
/// 机载不再保存 PWM 微秒或输出映射区间，避免两套系统重复维护。
/// `net_channel` 是 Actuator Set N（1..6），不是物理 AUX 通道号。
struct ActuatorConfig {
    /// 执行器实现类型：当前 `single_pwm`=单发舵机式；后续多发实现通过该键
    /// 在工厂切换，状态机只依赖 INetActuator 抽象接口。
    std::string type = "single_pwm";
    int32_t net_channel = 1;        ///< PX4 Actuator Set N（1..6）
    /// 归一化控制量：PX4 按该通道 PWM_AUX_MINn/MAXn 线性映射为 PWM。
    /// 标准 1000..2000µs 映射下，-0.7→1150µs（发射）、0.0→1500µs（待机）。
    double net_fire_norm = -0.7;    ///< 发射控制量（-1..1）
    double net_idle_norm = 0.0;     ///< 待机/复位控制量（-1..1）
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
    /// S7 退出：双目测距失效（invalid 或超龄）连续达该时长即回退——覆盖
    /// "测距丢失但视觉仍锁定"的收口（no_distance_action 不触发 kExit 的档位）
    std::chrono::milliseconds distance_lost_ms{3000};
    void Validate() const;
};

/// 拦截入口/授权（interception 节，2026-10-19）。
/// V1 默认关闭：生产只有在 G4/实机射程与授权策略验证后，才允许在本地副本开启。
struct InterceptionConfig {
    bool enabled = false;
    /// V1 自动授权。false 时 S9 等待授权并最终按超时返航；地面站确认协议尚未实现。
    bool auto_authorize = true;
    /// 连续 LOCKED 帧门限（拦截质量，与 handover.lock_frames 独立）。
    uint32_t lock_frames = 10;
    /// 允许发射的测距窗口（米）；默认值为占位，必须按捕网有效射程实机冻结。
    double min_range_m = 2.0;
    double max_range_m = 10.0;
    /// 动力电池最低剩余百分比；0=不启用（当前尚无冻结门限，按需配置）。
    double min_battery_percent = 0.0;
    /// S9 等待授权的超时；超时按设计返航。
    std::chrono::milliseconds authorization_timeout{1000};
    void Validate() const;
};

/// 拦截后返航（return_home 节，2026-10-19）。
struct ReturnHomeConfig {
    /// 单发发射后是否自动进入 S11/S12（默认 true）。测试可关闭以隔离捕网
    /// 执行器语义（S10 停留/复位重试用例）。
    bool enabled = true;
    /// 相对 Home 的返航高度：低于该值先原地爬升，再水平返航。
    double altitude_m = 5.0;
    /// 到达判据：水平半径、垂直误差、水平速度与稳定保持时长。
    double arrival_radius_m = 2.0;
    double arrival_vertical_m = 0.5;
    double arrival_speed_mps = 0.5;
    std::chrono::milliseconds arrival_hold{1000};
    /// 返航超时：超时后原地悬停告警（不自动转 RTL，交地面站/人工）。
    std::chrono::milliseconds timeout{60000};
    /// S10 发射命令入队超时：超时按"发射失败"结束并返航。
    std::chrono::milliseconds fire_timeout{3000};
    /// AUTO.LAND / DISARM 命令重试间隔。
    std::chrono::milliseconds command_retry{1000};
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
    ActuatorConfig actuator;            ///< 捕网执行器（阶段四控制接入 Task 10）
    InterceptionConfig interception;    ///< S9/S10 入口判据与授权（2026-10-19）
    ReturnHomeConfig return_home;       ///< S11~S14 拦截后返航（2026-10-19）
};

/// 按“可执行文件旁 config/config.json → 当前目录 config/config.json”查找配置。
std::string ResolveConfigPath(const std::string& executable_directory,
                              const std::string& requested_path = {});

/// 加载并校验正式主程序配置。资源相对路径按可执行文件目录解析。
AppConfig LoadAppConfig(const std::string& path,
                        const std::string& executable_directory);

}  // namespace drone::config
