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
    /// kBootstrap 预授权原点保持流最长时长（防卡死兜底）。状态机 S3 的
    /// 模式/解锁超时是权威，本值须覆盖 bootstrap_lead + mode + arm；
    /// 装配时由 DroneApplication 从 takeoff.bootstrap_timeout 注入。
    std::chrono::milliseconds bootstrap_timeout{6000};
    /// 多边形围栏钳制内缩余量（由 fence.boundary_margin_m 拷贝到此处，
    /// 控制器不自读 fence 节；2026-10-29 多边形围栏）。
    double fence_margin_m = 1.0;
    void Validate() const;
};

/// 捕网执行器配置（actuator 配置节，阶段四控制接入 Task 10）。
/// 机载只负责发送 MAV_CMD_DO_SET_ACTUATOR 的归一化控制量（-1..1）；
/// 输出功能、对应 PWM 映射区间（MAIN 输出为 PWM_MAIN_MINn/MAXn，AUX 输出为
/// PWM_AUX_MINn/MAXn）、舵机行程与方向在 PX4/QGC 侧配置，机载不再保存 PWM
/// 微秒或输出映射区间，避免两套系统重复维护。
/// `net_channel` 是 Actuator Set N（1..6），不是物理输出通道号。
/// 当前装机（2026-10-21 确认）：捕网接 MAIN5，Output Function 为
/// `Peripheral via Actuator Set 1`，故 net_channel=1。
struct ActuatorConfig {
    /// 执行器实现类型：当前 `single_pwm`=单发舵机式；后续多发实现通过该键
    /// 在工厂切换，状态机只依赖 INetActuator 抽象接口。
    std::string type = "single_pwm";
    int32_t net_channel = 1;        ///< PX4 Actuator Set N（1..6）
    /// 归一化控制量：PX4 按该输出通道的 PWM_MAIN_MINn/MAXn（或 AUX）线性
    /// 映射为 PWM。
    /// 标准 1000..2000µs 映射下，-0.7→1150µs（发射）、0.0→1500µs（待机）。
    double net_fire_norm = -0.7;    ///< 发射控制量（-1..1）
    double net_idle_norm = 0.0;     ///< 待机/复位控制量（-1..1）
    /// 触发 ACK ACCEPTED 后到发复位的保持时间；当前捕网枪说明书要求 1~2s。
    std::chrono::milliseconds reset_delay{1500};
    /// 复位 ACK ACCEPTED 后到 Ready 的稳定时间（多发用；单发默认 0）。
    std::chrono::milliseconds reset_settle{0};
    /// 地面站直控发射（65016）开关，默认 false。与 `interception.enabled` 互斥：
    /// 直控发射是拆桨台架/测试用途，不经过拦截授权链，装配期二者同时为 true
    /// 直接 fail-fast（避免两条发射路径抢同一个执行器）。
    bool gcs_fire_enabled = false;
    /// 两次成功直控发射之间的最小间隔，防止地面站误连点。默认 3000ms。
    std::chrono::milliseconds gcs_fire_cooldown{3000};
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
    /// 允许发射的测距窗口（米）。上限已按双目测距极限冻结为 8m（2026-10-23，
    /// 维护者确认：测距极限 8m 即有效射程，进入 ≤8m 才算有效）；下限 2.0 仍为
    /// 占位，待实机最小可用距离确认。
    double min_range_m = 2.0;
    double max_range_m = 8.0;
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
    /// 返航机头模式（2026-10-29）：0=不指定航向（保持原机头平移）；
    /// 1=机头朝飞行方向（前视避障传感器需要，维护者 2026-10-29 确认）。
    uint8_t heading_mode = 1;
    /// 机头朝飞行方向时的偏航角速度/角加速度上限（与视觉跟踪同量级）。
    double heading_rate_limit_dps = 30.0;
    double heading_accel_limit_dps2 = 60.0;
    void Validate() const;
};

/// 多边形围栏（fence 节，2026-10-29）。
/// 区域由地面站 65018 下发（机载缓存 `fence.zone_cache_path`），机载自治执行：
/// 越界判定 + 位置意图钳制 + 越界中止（与电池守卫共用中止入口）。
/// 默认关闭：关闭时完全回退旧的 Home 圆形围栏
/// （`controller.max_radius_m` / `controller.max_altitude_m`）。
struct FenceConfig {
    bool enabled = false;
    /// 区域缓存路径（相对可执行文件目录）；缺失/非法按 kNoZone 处理（不中止）。
    std::string zone_cache_path = "config/zone.json";
    /// 越界判定防抖：连续处于区域外达该时长才判定越界并执行动作。
    std::chrono::milliseconds violation_debounce{1500};
    /// 位置意图钳制内缩余量：钳制落点距边界至少这么多米（>0）。
    double boundary_margin_m = 1.0;
    /// 接近边界告警距离（m）：区域内但距边界小于该值报 kNearBoundary。
    double boundary_warn_m = 10.0;
    /// 越界动作：follow_zone_action=true 时以 65018 下发的动作为准，
    /// 否则固定使用本字段。
    bool follow_zone_action = true;
    common::FenceAction violation_action = common::FenceAction::kReturn;
    /// 围栏状态回传（65019）周期。
    std::chrono::milliseconds status_period{1000};
    /// 65022 区域回读周期（机载 → 地面站）：地面站据此比对本地定义与机载实况，
    /// 并在刷新/重连后恢复图层。5s 一次、约 56B，对串口负担可忽略。
    std::chrono::milliseconds report_period{5000};

    // 机载初始区域（2026-10-29）：既没有 zone.json 缓存、也还没收到 65018 时，
    // 以 Home 为中心生成该方形区域并立即落盘，保证"开机即有围栏"，避免出现
    // "无区域则不限制"的空窗。地面站连上后可比对并整体覆盖。
    double default_half_side_m = 100.0;
    double default_alt_min_m = 0.0;
    double default_alt_max_m = 30.0;
    uint16_t default_zone_id = 1;
    uint16_t default_version = 1;
    common::FenceAction default_action = common::FenceAction::kReturn;
    void Validate() const;
};

/// 自动起飞（takeoff 节，2026-10-22 多机任务分配与自动起飞）。
/// 约束 S3 解锁/Offboard 与 S4 爬升的时序、到达判据和目标丢失收口；
/// 电池门限不使用本地百分比，唯一来源为 PX4 `BAT_LOW_THR`（后续任务接入）。
/// 全部字段可选，缺席回退默认值；Validate 做范围与跨字段校验。
struct TakeoffConfig {
    double height_m = 2.0;                         ///< 相对起飞点爬升高度
    double climb_rate_mps = 0.5;                   ///< S4 爬升速度
    std::chrono::milliseconds bootstrap_lead{300}; ///< 起飞意图到 176 的前置保持
    std::chrono::milliseconds mode_timeout{2000};  ///< 等待 Offboard(6/0) 的超时
    std::chrono::milliseconds arm_timeout{3000};   ///< 等待解锁的超时
    std::chrono::milliseconds bootstrap_timeout{6000};  ///< Bootstrap 原点保持流最长时长
    std::chrono::milliseconds height_timeout{15000};    ///< S4 到达任务高度的超时
    std::chrono::milliseconds stable_time{1000};   ///< 到达判据持续时长
    double z_tolerance_m = 0.2;                    ///< 到达判据：垂直误差
    double xy_tolerance_m = 0.5;                   ///< 到达判据：水平误差
    double max_vz_mps = 0.3;                       ///< 到达判据：垂直速度
    std::chrono::milliseconds target_wait_timeout{10000};  ///< 等待目标位置超时
    std::chrono::milliseconds target_lost_timeout{10000};  ///< 目标丢失收口超时
    /// S5 目标失效期间“原地保持”的累计时长上限（绝对兜底）。目标反复闪烁时
    /// target_lost_timeout 每次恢复都会清零，单靠它无法防止无限悬停；本上限按
    /// 本次 S5 内累计保持时长收口，超限 → 65015 aborted + S12 返航。
    std::chrono::milliseconds target_hold_max{60000};
    std::chrono::milliseconds battery_low_debounce{1000};  ///< 低电防抖时长
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
    TakeoffConfig takeoff;              ///< S3/S4 自动起飞时序（2026-10-22）
    FenceConfig fence;                  ///< 多边形围栏（2026-10-29）
};

/// 按“可执行文件旁 config/config.json → 当前目录 config/config.json”查找配置。
std::string ResolveConfigPath(const std::string& executable_directory,
                              const std::string& requested_path = {});

/// 加载并校验正式主程序配置。资源相对路径按可执行文件目录解析。
AppConfig LoadAppConfig(const std::string& path,
                        const std::string& executable_directory);

}  // namespace drone::config
