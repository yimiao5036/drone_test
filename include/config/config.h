#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "common/types.h"
#include "communication/ground_station_link.h"
#include "communication/px4_link.h"
#include "control/approach_speed_law.h"
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
    bool enable_visual_tracking = false;  ///< 视觉跟踪控制律独立观测节拍（产物 Topic 无消费者）
    bool enable_stereo_ranging = false;  ///< 双目测距（观测：输出供控制律距离通道；依赖 uvc+stereo_split+视觉跟踪）
    /// 双目视觉观测 → 局部 NED 融合进目标估计器（观测；依赖 target_estimator +
    /// stereo_ranging + px4；不延展地面站有效期，docs/物理追踪思路.md §6.3）
    bool enable_visual_position_fusion = false;
    /// **运行模式开关（2026-11-17 重新定义）**，只回答"本次运行允不允许向 PX4 下发控制"：
    /// - `true` = **控制运行**（生产默认）：装配层注入 PX4 快照源、绑定 `Px4Setpoint`、
    ///   给状态机注入命令窄接口；
    /// - `false` = **只读运行**（地面排查：事故后采日志、链路排查、新机体首上电）：
    ///   上面三项都不做，只读遥测/视频/回传。
    ///
    /// **它不是阶段标记，也不是功能分支依据**：功能代码不得读本字段，
    /// 只能按可观测能力判断（命令接口是否注入、快照是否有效等）。
    /// 唯一允许读它的地方是 `DroneApplication::BindTopics()` 的装配门禁与
    /// `LoadAppConfig()` 的跨字段校验。
    /// 定义与理由见 `docs/运行模式与模块状态定义.md`。
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
    // 位置型意图速度前馈（P3，2026-10-19）：开关默认关闭（保守：未知速度
    // 时不叠加前馈）；生产配置显式打开并给出上限。上限独立于 max_speed_mps
    // （后者只约束速度式意图），水平模长/垂直分量分别钳制。
    bool enable_position_velocity_feedforward = false;
    double max_feedforward_mps = 2.0;
    /// 位置引导速率上限（2026-11-02，P0-2）：位置型意图不再把目标点原样
    /// 直通给 PX4，而是按本速率把引导点从当前位置向目标推进（避免目标跳变
    /// 时 PX4 用自身 MPC_XY_VEL_MAX 冲过去）。仅当意图带 max_speed_mps>0
    /// 时启用；实际速率取 min(本值, 意图 max_speed_mps)。垂直分量另限。
    double position_guide_speed_mps = 6.0;
    double position_guide_vz_mps = 1.5;
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
    /// 在工厂切换，状态机只依赖 IInterceptionDevice 抽象接口。
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
/// 视觉锁定↔卡尔曼跟踪的状态切换迟滞：exit_lost_frames 必须大于 enter_s6_lock_frames。
struct HandoverConfig {
    uint32_t enter_s6_lock_frames = 10;  // S5→S6：LOCKED 连续帧数
    double fow_margin_deg = 5.0;        // 视线角视场余量
    double innovation_gate_m = 3.0;     // 视觉观测与卡尔曼预测创新门限（米）
    std::chrono::milliseconds timeout{3000};  // S6 驻留超时
    uint32_t exit_lost_frames = 30;     // S7 退出：LOST 连续帧数（必须 > enter_s6_lock_frames）
    /// S7 退出：双目测距失效（invalid 或超龄）连续达该时长即回退——覆盖
    /// "测距丢失但视觉仍锁定"的收口（no_distance_action 不触发 kExit 的档位）
    std::chrono::milliseconds distance_lost_ms{3000};
    // ---- 到点判定与"到点后视觉未识别"等待（2026-11-05，S5 到点不告警）----
    // 目标看门狗只在目标失效时计时；目标以 5Hz 持续送达时它永不启动，
    // 于是"飞到目标点上方悬停、视觉一直没锁定"没有任何提示。以下四组参数
    // 用独立的判据补这一段，语义与 target_wait/lost/hold 完全无关。
    /// 到点水平半径（m）：飞机与分配目标点的水平距离进入该值视为到点候选。
    double arrival_radius_m = 3.0;
    /// 到点垂直半径（m）：相对目标点高度差的绝对值上限。
    double arrival_vertical_m = 2.0;
    /// 到点需连续保持的时长：进入半径且地速 ≤ 0.5m/s 持续该时长才判到点，
    /// 避免掠过目标点就当"到点"。
    std::chrono::milliseconds arrival_hold_ms{2000};
    /// 到点后视觉未锁定的告警阈值：超过即置 warning bit6「到点未识别」。
    std::chrono::milliseconds visual_wait_warn_ms{20000};
    /// 到点后视觉未锁定的自动处置阈值：0 = 只提示、不自动返航（现场默认）。
    std::chrono::milliseconds visual_wait_abort_ms{0};
    void Validate() const;
};

/// 拦截入口/授权（interception 节，2026-10-19）。
/// V1 默认关闭：生产只有在 G4/实机射程与授权策略验证后，才允许在本地副本开启。
struct InterceptionConfig {
    bool enabled = false;
    /// V1 自动授权。false 时 S9 等待授权并最终按超时返航；地面站确认协议尚未实现。
    bool auto_authorize = true;
    /// 连续 LOCKED 帧门限（拦截质量，与 handover.enter_s6_lock_frames 独立）。
    uint32_t enter_s9_lock_frames = 10;
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
    /// 返航爬升率（m/s）：爬升段经控制器“位置引导限速”推进，避免位置目标
    /// 阶跃时 PX4 用自身 MPC_Z_VEL_MAX_UP（默认 3m/s）冲上去——2026-11-03
    /// 实机反馈“低于 5m 时上升很快”即此原因。
    double climb_rate_mps = 0.5;
    /// 水平返航速度上限（m/s）：爬升完成后朝 Home 推进的最大速度。
    /// **独立于追踪速度**（`controller.max_speed_mps=10`）：返航是安全收口，
    /// 顶速不需要跟追踪同量级（2026-11-12 维护者裁决：4.0）。
    double cruise_speed_mps = 4.0;
    /// 返航水平速度剖面的加/减速度（m/s²，2026-11-12）。
    /// 状态机在 S12 内按“分段恒定加速度”生成速度指令（无 jerk），两种情况
    /// 都必须有斜坡：①进入返航时飞机可能静止（目标未运动/已掉落），需要
    /// 加速到 cruise；②追不上目标转返航时飞机速度可能高于 cruise，需要先
    /// 减速。**不复用 `controller.max_accel_mps2`**（那是追踪/位置引导共用的
    /// 限幅参数），返航有自己的安全收口节奏。
    /// `accel_plan_mps2=0` 或 `decel_plan_mps2=0` 表示该段不做斜坡（直通）。
    double accel_plan_mps2 = 1.5;
    double decel_plan_mps2 = 2.0;
    /// 末端制动包络的规划减速度余量：巡航段按 v ≤ √(2·decel·d) 收敛即可，
    /// 该值只用于末端包络的安全系数（1.0=不加余量）。
    double brake_safety = 1.3;
    /// 末端低速档（m/s）：距目标进入 `final_zone_m` 后巡航顶速降为该值，
    /// 使末端平顺、到达判据（`arrival_speed_mps`）自然满足。
    double final_speed_mps = 1.0;
    double final_zone_m = 10.0;
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

/// 降落两段（landing 节，2026-11-17 降落与估计器保护 P2）。
/// 背景：2026-11-17 10:24 现场 `AUTO.LAND` 从 27 m 一路无人监督地降到底，
/// 期间 EKF 全状态重置把 local z 平移 9 m，PX4 认为"已触地"而砍推力 → 翻滚。
/// 改为两段收口：
/// - **S13a**（机载受控下降）：Offboard 位置引导，把相对高度压到 `descend_target_ag_m`
///   （默认 1.2 m），全程我们持有控制权，可随时中止/提前交段；
/// - **S13b**（交 PX4）：发 `DO_SET_MODE(AUTO.LAND 4/6)`，确认后停止发布 intent，
///   触地仍由 PX4 land detector 判定。
/// `enabled=false` 是回滚开关：S13 直接发 AUTO.LAND（2026-10 之前的旧行为）。
/// 纯影子装配（`runtime.enable_control=false`）不注入命令接口，状态机因此
/// 自然走旧路径，不需要另设装配门禁字段。
struct LandingConfig {
    /// 两段降落总开关（回滚开关）。false = S13 直接 AUTO.LAND。
    bool enabled = true;
    /// 交 PX4 时的相对高度（m，相对起飞点/Home 基准，见 RelativeAltitudeBaseZ）。
    /// 不用 0：留余量给 PX4 的 land detector 与姿态收敛；也不用 ≥3 m：
    /// 交段越晚，无人监督的下降段越短。
    double descend_target_ag_m = 1.2;
    /// 主下降速度（m/s）：作为位置引导意图的 max_speed_mps。
    double descend_speed_mps = 0.8;
    /// 低速档下降速度（m/s）：相对高度进入 `descend_slow_zone_m` 后使用，
    /// 让交段判据（高度/垂速/水平误差）有收敛时间，避免 1~2 m 处冲过头。
    double descend_slow_speed_mps = 0.3;
    /// 低速档切换高度（m）：相对高度 ≤ 该值用低速档。
    double descend_slow_zone_m = 3.0;
    /// 交段判据：相对高度容差（m）→ 要求相对高度 ≤ 目标 + 容差。
    /// 只用 EKF/baro 差分，baro 受桨风影响 0.2~0.5 m，实机首轮先看日志再定。
    double handover_alt_tolerance_m = 0.15;
    /// 交段判据：垂直速度上限（m/s）。
    double handover_vz_mps = 0.2;
    /// 交段判据：水平误差上限（m，相对 Home 水平目标）。
    double handover_horizontal_m = 0.5;
    /// 交段判据持续成立时长：避免"擦过 1.2 m"就交段。
    std::chrono::milliseconds handover_hold{500};
    /// S13a 超时（27 m 按 0.8 m/s 约 33 s，留余量）：到点即交 PX4。
    std::chrono::milliseconds descent_timeout{120000};
    /// 高度源交叉校验阈值（m）：气压 AGL 与 EKF 相对高度之差超过该值，
    /// 且持续 `baro_mismatch_hold`，立即交 PX4（不再硬降到 1.2 m）。
    double baro_mismatch_m = 1.5;
    std::chrono::milliseconds baro_mismatch_hold{2000};
    /// 落地确认门禁（2026-11-18 P5）：PX4 报 `landed` 且**仍上锁**时，要求单一高度
    /// 出口（`AboveTakeoffM()`）给出的离地高度 ≤ 本值，才认账并发 DISARM。
    /// 背景：事故形态是"EKF 高度被重置 → land detector 误判已落地 → 砍推力"，
    /// 在 2 m 高度上锁与砍推力同样危险，所以落地确认也必须有独立高度见证。
    /// 0 = 不做该判据。**能力边界**：它只能约束"我们是否认账并上锁"，
    /// 拦不住 PX4 自己在 AUTO.LAND 里做的 land detector 决策。
    double landed_max_agl_m = 1.0;
    /// 上面这条门禁的兜底等待：PX4 报 landed 但与高度见证矛盾超过该时长，
    /// 仍按 PX4 处置（打 ERROR 说明矛盾），避免雷达卡死时任务永远收不了尾。
    std::chrono::milliseconds landed_agl_timeout{3000};
    void Validate() const;
};

/// 估计器降级判据与处置（estimator_guard 节，2026-11-17 降落与估计器保护 P3）。
///
/// 背景：2026-11-17 10:25 现场 EKF 全状态重置（local z 平移 9 m）后，`AUTO.LAND`
/// 无人监督一路降到底、PX4 误判触地砍推力而翻滚。判据模块 `EstimatorGuard`
/// （`include/state_machine/estimator_guard.h`）在 P1 已落地并接通遥测，P3 起由
/// `MissionStateMachine` **每拍喂快照**：任一判据命中即**锁存**，切 `POSCTL` 悬停、
/// 停发一切控制意图、冻结任务推进，等飞手接 `STABILIZED` 处置
/// （设计 §5：不自动降落、不自动返航、不自动上锁、不重试切模式）。
///
/// **`enabled=false` 是唯一回滚开关**：关掉后判据完全不跑（不告警、不切模式、
/// 不冻结），调度行为回到 P2 状态。阈值全部外置的原因：这些阈值错了就是漏报或
/// 误报，实机首轮必须能按日志调，不能改代码重编。
///
/// 判据实现与极性说明见 `include/state_machine/estimator_guard.md`（含"为什么
/// `ESTIMATOR_STATUS.flags` 掩码默认 0=关闭"）。本结构只承载参数，不做判定。
struct EstimatorGuardConfig {
    /// 总开关（回滚开关）。false = 判据不跑（P1/P2 行为）。
    bool enabled = true;

    // ---- 单拍跳变判据（本次事故是 z 单拍跳 9 m，一帧即命中）----
    /// EKF 局部 z 单拍跳变门限（m）：|Δz| > 本值 **且** |Δz| > |vz|·Δt + 余量。
    double z_jump_m = 0.5;
    /// 跳变判据里扣掉"速度本身位移"之外的余量（m）。
    double z_jump_speed_margin_m = 0.1;
    /// EKF 局部 xy 单拍跳变门限（m）：同样要求大于地速位移 + 余量。
    double xy_jump_m = 1.0;
    double xy_jump_speed_margin_m = 0.3;

    // ---- 持续型判据（必须"持续"才判，单帧尖峰不算）----
    /// 新息检验比门限（PX4 口径 >1 = 该路观测与状态不一致）。
    double ratio_limit = 1.0;
    /// 任一路比值超限需要持续的时长。
    std::chrono::milliseconds ratio_hold{1000};
    /// 气压高度（AGL）与 EKF 相对高度允许偏差（m）：两条独立物理路径交叉校验。
    double baro_divergence_m = 1.5;
    /// 高度源不一致需要持续的时长。
    std::chrono::milliseconds baro_hold{2000};
    /// 关键传感器健康位（present 置位但 health 清零）持续掉线的判定时长。
    std::chrono::milliseconds sensor_hold{1000};
    /// `LOCAL_POSITION_NED` 新鲜度门限。**瞬时判据**（断链/丢帧本身就要降级），
    /// 不受 hold 保护。
    std::chrono::milliseconds local_position_stale{500};

    /// 关键传感器掩码（MAV_SYS_STATUS_SENSOR_* 位值）。
    /// **0 = 用模块默认**（陀螺|加速度计|磁罗盘|AHRS，不含 GPS）。
    uint32_t critical_sensor_mask = 0;
    /// `ESTIMATOR_STATUS.flags` 位掩码：置位视为退化。
    /// **默认 0 = 关闭**：本项目 third_party 的位定义是"置位=可用"的旧语义，
    /// 未与固件核对前不能先验判（先验判会误报或漏报，都是安全风险）。
    uint16_t bad_estimator_flag_mask = 0;

    void Validate() const;
};

/// 任务级重置动作（mission 节，2026-11-17 降落与估计器保护 P4b）。
///
/// 背景：2026-11-17 10:24 现场 EKF 全状态重置 + AUTO.LAND 翻滚之后，飞控自身状态
/// 已经不可信（估计器、Home、起飞基准都可能错），只清我们状态机的锁存解决不了问题。
/// 因此 65020 `type=2` 的语义从"只清状态"改为**清任务状态 + 重启 PX4**（设计 §6）：
/// 受理后发 `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(246) param1=1`，迁移 S1 SELF_CHECK
/// 等飞控回来（心跳 + GPS fix + 全局位置 + Home），再走常规 S1→S2。
///
/// **重启治标不治本**：罗盘失效、Z 轴振动、EKF 参数问题重启后仍会出现，
/// 这只是把飞控恢复到"已知初始状态"，不修复硬件。
struct MissionConfig {
    /// 回滚开关：false = 恢复 2026-11-04 的旧语义（只清状态、不重启 PX4，
    /// 直接回 S2 READY）。老地面站（不知道重启语义）连新机载时用这个降级。
    bool reset_reboot_px4 = true;
    /// 等 PX4 回来的上限。PX4 冷启动 15~30 s，留足余量；到点仍无遥测则保持
    /// S1 SELF_CHECK 并打 ERROR（不自动重启机载程序）。
    std::chrono::milliseconds reboot_timeout{90000};
    /// 246 命令的最大发送次数（含首次）。每次失败/无 ACK 后按固定 1 s 间隔重发，
    /// 用完即停（**不无限重发**，避免把正在重启的飞控再打一遍）。
    int reboot_retry = 3;
    /// 两次重启的最小间隔：期间收到的新重置指令直接拒绝（回 kDuplicateCommand），
    /// 避免刚起来的飞控又被重启。
    std::chrono::milliseconds reboot_min_interval{30000};
    void Validate() const;
};

/// 起飞前门禁（preflight_gate 节，2026-11-18 维护者裁决 H 项）。
///
/// 背景：2026-11-17 的事故源头是**罗盘 #1 失效 + 高振动**（`振动 5.24 m/s²`）。
/// P3 的 `EstimatorGuard` 只在**飞行中**兜底（判据命中 → 切 POSCTL 冻结）；
/// 地面这一层之前没有任何门禁——PX4 拒绝解锁时我们只会等 `arm_timeout` 再回
/// `kArmTimeout`，把"罗盘没校准"报成"解锁超时"。
///
/// 本门禁**只作用于 65014 任务分配预检**（不阻断 S1→S2，联调不受影响）：
/// - 传感器健康：`SYS_STATUS` 里 `present` 置位但 `health` 清零的关键位
///   （陀螺/加速度计/磁罗盘/AHRS）持续 `sensor_health_hold` → 拒绝分配
///   （`kSensorUnhealthy`）。能抓"缺磁/无数据/磁故障"；**抓不到"未校准"**
///   （校准状态不是健康位），那类仍由 PX4 在 ARM 时拒绝 + 我们把
///   `STATUSTEXT` 原样打进日志。
/// - 振动：**维护者 2026-11-18 裁决只告警、不默认阻断**。理由：PX4 自身
///   `HealthAndArmingChecks` 里没有振动项（不是 PX4 门禁）；常引用的 30 m/s²
///   出自 ArduPilot 文档；事故当天 `5.24` 远低于 30，卡它反而误禁飞。
///   `vibration_block_mps2 > 0` 时才真的拒绝（`kVibrationHigh`）。
///
/// 指标口径（不要当成加速度 RMS）：`VIBRATION` 的 `vibration_x/y/z` 在 PX4 里分别是
/// 陀螺锥动 / 陀螺高频 / **加速度计高频**指标，见 `VIBRATION.hpp`；只有 `z`（accel）
/// 参与本门禁阈值比较。三个量都**不来自磁力计**——所以"振动伤罗盘"这个说法不成立，
/// 振动真正伤的是加速度计与 EKF 姿态/速度。
struct PreflightGateConfig {
    /// 传感器健康门禁总开关。false = 回到 2026-11-17 之前（不做任何地面预检）。
    bool require_sensor_health = true;
    /// 关键传感器（陀螺|加速度计|磁罗盘|AHRS）健康位持续掉线多久才拒绝分配。
    /// 取 1 s 与 `EstimatorGuard` 的飞行中判据同量级，避免上电瞬间的抖动误拒。
    std::chrono::milliseconds sensor_health_hold{1000};
    /// 振动告警阈值（m/s²，`VIBRATION.vibration_z` 加速度计高频指标）。
    /// 默认 20：事故当天 5.24 与之相比仍有明显余量，又不至于等到 ArduPilot 的
    /// 30 才提示。0 = 关闭告警。
    double vibration_warn_mps2 = 20.0;
    /// 振动**阻断**阈值（m/s²）。**默认 0 = 不阻断**（维护者裁决）；设 >0 才启用。
    /// 启用后的取舍：宁可地面拒飞，也不要带着高振动去飞 EKF。
    double vibration_block_mps2 = 0.0;
    /// 振动超阈值需要持续的时长（防单帧尖峰误告警）。
    std::chrono::milliseconds vibration_hold{1000};
    void Validate() const;
};

/// 测距雷达（radar 节，2026-11-18 降落与估计器保护 P5 软件侧）。
///
/// 背景：本次事故的决定性因素是"高度量混淆"——EKF z 被重置约 9 m，而气压平滑。
/// 装下视雷达后，低空多了一个**完全不经 EKF** 的物理量（离地距离），S13a 交段
/// 判据与落地确认都改用它；`common::AboveTakeoffM()` 是唯一取高度出口，
/// 退化链为 雷达 AGL → 气压 AGL → EKF 差分（见 `include/common/types.h`）。
///
/// **软件侧先落地、硬件后到**：`enabled=false` 时高度出口行为与 P5 之前逐位一致，
/// 所以本配置节可以先随固件发出去，等雷达到位只改配置 + 接线上电。
///
/// 两条与设计稿不同的实现口径（2026-11-18 核对 PX4 源码后修正，见设计文档 §7）：
/// 1. **不能用 `SYS_STATUS.LASER_POSITION` 做门禁**：本版 PX4 的
///    `mavlink/streams/SYS_STATUS.hpp` 只填 battery/motors/parachute/accel/gyro/
///    mag/gps/rc/attitude/local_position/AHRS/logging/pressure 这些组件，
///    `MAV_SYS_STATUS_SENSOR_LASER_POSITION(bit8=0x100)` **永远不置位**。
///    因此雷达门禁改为判"`DISTANCE_SENSOR` 是否有新鲜有效读数"（自己算），
///    并把"PX4 侧是否要求装测距"交给 PX4 参数 `SYS_HAS_NUM_DIST`。
/// 2. **`signal_quality` 不能按"0=未知就算差"处理**：PX4 的 DISTANCE_SENSOR 出口
///    把驱动的 `quality<0`（TFmini 与 SITL 的 gz 雷达都是）映射成 MAVLink 0，
///    把 `quality==0` 映射成 1（invalid）。0 是"没有质量指标"，1 才是无效，
///    2~100 才参与 `min_signal_quality` 门槛。
struct RadarConfig {
    /// 高度源是否使用雷达（回滚开关）。false = 完全保持 P5 之前的行为。
    bool enabled = false;
    /// 起飞前是否**硬要求**雷达在报数（受理 65014 前判，拒绝走 `kRadarUnavailable`）。
    /// 判据是 `common::RadarAlive()`——"收到过且时效内"，**不判读数量程**：
    /// 落在地上的下视测距必然在盲区（TFmini-S 最小 0.4 m），拿量程有效当起飞门禁
    /// 等于任何时刻都拒绝起飞。要求 `enabled=true` 才有意义（`Validate()` 会拒绝
    /// "只装雷达不进高度链"这种组合）。
    bool preflight_required = false;
    /// 信号质量门槛（2~100 的实测百分比）。0 = 不做质量判据。
    /// 默认 30 是保守起点：TFmini 不给质量（恒 0=未知，不受该值约束），
    /// 一旦换到给质量的器件，首轮先看日志里的实测值再调整（设计 §7 待实测 #1）。
    uint8_t min_signal_quality = 30;
    /// 数据时效上限（ms）：PX4 的 TFmini 驱动在超量程/无回波时**干脆不发布**
    /// `distance_sensor`（`TFMINI.cpp` 里 `distance_m<0` 直接 `-EAGAIN`），
    /// 所以"没新数据"就是"超量程"的主要表现，时效判据是超量程的主要防线。
    std::chrono::milliseconds stale{200};
    /// 把本配置映射成高度出口的判据参数（单一入口，避免各处各写一份阈值）。
    common::RadarGateParams ToGateParams() const;
    void Validate() const;
};

/// 多边形围栏（fence 节，2026-10-29）。
/// 区域由地面站 65018 下发（机载缓存 `fence.zone_cache_path`），机载自治执行：
/// 越界判定 + 位置意图钳制 + 越界中止（与电池守卫共用中止入口）。
/// 默认**开启**（2026-10-29 维护者裁决）：开启后无区域时按 `default_*` 以 Home
/// 为中心生成默认区域，因此没有"圆形兜底围栏"这一层——围栏只有多边形一种。
struct FenceConfig {
    bool enabled = true;
    /// 区域缓存路径（相对可执行文件目录）；缺失/非法按 kNoZone 处理（不中止）。
    std::string zone_cache_path = "config/zone.json";
    /// 越界判定防抖：连续处于区域外达该时长才判定越界并执行动作。
    std::chrono::milliseconds violation_debounce{1500};
    /// 越界**深度**容差（m，2026-11-12）：飞机自身位置必须越出多边形边界
    /// 超过该深度才算越界。0=零容差（历史行为）。现场 0119/0120 日志显示
    /// 目标贴边时飞机会被位置环推出去 1~3m 又被拉回，零容差会把一次瞬时
    /// 穿透判成越界中止（`波动两下就返航`）。建议 1.0~2.0。
    double violation_depth_m = 1.5;
    /// 越界恢复滞回（m，2026-11-12）：已判越界后，带符号距离必须回到
    /// `−violation_depth_m + violation_hysteresis_m`（默认 −1.0m）以上才清除，
    /// 即越界深度收敛到 1.0m 以内（仍在边界外也可以，配合 depth 避免抖动）。
    /// **必须 ≤ 本值对应的 violation_depth_m**：滞回要落在容差带内，不能放到
    /// 边界内侧——否则飞机停在界外但仍在容差内时会一直累计越界时长，容差失效。
    double violation_hysteresis_m = 0.5;
    /// 位置意图钳制内缩余量：钳制落点距边界至少这么多米（>0）。
    double boundary_margin_m = 1.0;
    /// 接近边界告警距离（m）：区域内但距边界小于该值报 kNearBoundary。
    double boundary_warn_m = 10.0;
    /// 目标跑出区域时的制导策略（2026-11-12 维护者裁决）：
    /// true=S5 引导点被围栏挡住时改为“贴边保持 + 告警、不中止任务”
    /// （S6 冻结续发沿用最后一条 S5 意图），
    /// 目标回到区域内（带 `guide_hold_release_m` 滞回）后自动恢复跟随；
    /// false=只依赖控制器逐拍钳制（2026-11-11 之前行为，会在边界来回冲）。
    bool guide_hold_enabled = true;
    /// 贴边保持的解除滞回（m）：目标重新回到区域内、且距边界超过该距离，
    /// 才恢复正常的接近制导（避免目标贴边时保持/跟随反复切换）。
    /// 实现取 `max(boundary_margin_m, 本值)`：低于进入余量时不产生滞回，
    /// 保持/跟随会每拍切换，故按进入余量兜底（2026-11-12 复审）。
    double guide_hold_release_m = 2.0;
    /// **围栏停止点预测**（2026-11-12 复审三轮，维护者裁决）：沿飞机当前
    /// 速度方向做停止点预测 `p + v̂·(v·reaction + v²/(2·decel))`，预测点越界
    /// 就提前压速度/贴边保持。`decel=0` 关闭预测（回退仅几何保持）。
    /// 用可达最大减速度，**不要用接近律当前相位的计划加速度**。
    double guard_decel_mps2 = 2.0;
    /// 停止点预测的反应/姿态延迟（秒）：`v·reaction` 计入刹停距离。
    double guard_reaction_s = 0.2;
    /// **目标出区保持**（2026-11-13，维护者裁决）：目标 NED 位置落在围栏外时，
    /// 立即停止追击、位置保持（不中止任务、不执行 return/land、无超时兜底）。
    /// 解除条件：目标回到区内且距边界 ≥ `target_outside_release_m`，持续
    /// `target_outside_release_hold`，**并且飞机自身已回到区内**。
    /// 0 = 目标一回到区内即可解除（仅调试用）。
    double target_outside_release_m = 50.0;
    /// 目标出区保持的解除保持时长。
    std::chrono::milliseconds target_outside_release_hold{10000};
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

/// 接近段策略（approach 节，2026-11-02 引入；2026-11-11 起只保留速度律）：
/// S5 由 `control::ApproachSpeedLaw` 产出 `(p_ref, v_ref)`，旧“固定提前 +
/// 位置引导点”实现已删除，不再保留回退字段。
struct ApproachConfig {
    /// S5 接近段速度律（2026-11-11 落机载，唯一 S5 引导方式）。
    control::ApproachSpeedLawConfig speed_law;
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
    LandingConfig landing;              ///< S13 拆两段（2026-11-17 P2）
    EstimatorGuardConfig estimator_guard;              ///< 估计器降级判据与处置（2026-11-17 P3）
    MissionConfig mission;                  ///< 65020 重置 = 清状态 + 重启 PX4（2026-11-17 P4b）
    PreflightGateConfig preflight_gate;     ///< 起飞前门禁：传感器健康 + 振动（2026-11-18 H 项）
    RadarConfig radar;                       ///< 测距雷达：单一高度出口 + 预检门禁（2026-11-18 P5）
    TakeoffConfig takeoff;              ///< S3/S4 自动起飞时序（2026-10-22）
    FenceConfig fence;                  ///< 多边形围栏（2026-10-29）
    ApproachConfig approach;            ///< S5/S6 接近段速率与预测（2026-11-02）
};

/// 按“可执行文件旁 config/config.json → 当前目录 config/config.json”查找配置。
std::string ResolveConfigPath(const std::string& executable_directory,
                              const std::string& requested_path = {});

/// 加载并校验正式主程序配置。资源相对路径按可执行文件目录解析。
AppConfig LoadAppConfig(const std::string& path,
                        const std::string& executable_directory);

}  // namespace drone::config
