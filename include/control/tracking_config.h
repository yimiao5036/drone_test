#pragma once

// =============================================================================
// tracking_config.h —— 视觉跟踪控制律配置（TrackingConfig）
//
// 对应规格：docs/物理追踪思路.md
//   §8  限幅、阈值与配置化（所有阈值集中配置，不得散落在代码中，
//       需求分析.md NFR-007）
//   §4  水平航向控制（模式、增益、限幅）
//   §5  垂直/高度控制（垂直速度增益与限幅；统一速度支路）
//   §6  距离控制（射程、PID、限幅、无距离降级动作）
//   §7  控制周期与时序（频率、各输入超龄阈值）
//
// 移植说明（2026-09-23，自 visual_track_control/vtc_config.h）：
//   - camera 组从 FOV 改为标定内参（fx/fy/cx/cy），默认值即 2026-09-23
//     左目实测（1280×720 档）；分辨率与内参绑定，换档位必须重新标定；
//   - 主工程约定：字段解析在 config.cpp（配置段缺席时用默认值），本文件
//     只保留跨字段/取值校验 Validate()（构造/初始化抛异常规范）；
//   - 热路径不触碰配置源；调参重启生效。
// =============================================================================

#include <cstdint>

namespace drone::control {

/// 航向通道模式（§4.2：位置式/速率式按 PX4 Offboard 实测选择）
///
/// 默认速率式：yaw_rate 指令天然是相对量，无坐标系歧义。位置式输出为
/// 绝对 NED 航向（当前航向 + 限幅修正量合成），依赖新鲜自机姿态；
/// 姿态接线未完成前不得用于实飞。
enum class HeadingMode : std::uint8_t {
    kRate,      // 速率式（默认）：β → 偏航角速度，输出 yaw_rate_dps
    kPosition,  // 位置式：β → 修正量，与当前航向合成绝对航向输出 yaw_deg
};

/// 目标距离不可用（无效/超龄）时的距离通道降级动作（§6.3）
enum class NoDistanceAction : std::uint8_t {
    kHold,         // 保持：前向速度指令清零（安全占位语义）
    kSlowApproach, // 限速接近：按 no_distance_approach_limit_mps 缓速接近
    kExit,         // 退出近距操作：持续无距离达 no_distance_exit_grace_ms 才
                   // 降级保持（kDegradedHold）；grace 期间按 kSlowApproach 行为
};

/// control 组：控制周期与各输入超龄阈值（§7）
struct TrackingControlGroupConfig {
    double frequency_hz = 20.0;             // 控制频率（Hz）
    std::int64_t visual_stale_ms = 200;     // 视觉追踪结果超龄阈值（毫秒）
    std::int64_t distance_stale_ms = 500;   // 目标距离超龄阈值（毫秒）
    std::int64_t attitude_stale_ms = 300;   // 自机姿态超龄阈值（毫秒）
};

/// camera 组：标定内参（2026-09-23 左目实测 @1280×720，单目链路使用左目；
/// 主点 cy 偏离画面中心约 172px，不得假设居中）。
/// 内参只对采集档位（2560×720 SBS → 单目 1280×720）有效；换档位必须重新
/// 标定或按比例换算（优先重新标定）。
struct TrackingCameraGroupConfig {
    int image_width = 1280;      // 画面宽度 W（像素）
    int image_height = 720;      // 画面高度 H（像素）
    double fx_px = 1007.82;      // 水平像素焦距（标定实测）
    double fy_px = 1008.02;      // 垂直像素焦距（标定实测）
    double cx_px = 620.73;       // 主点 x（标定实测，非画面中心 640）
    double cy_px = 532.07;       // 主点 y（标定实测，非画面中心 360）
};

/// heading 组：水平航向通道（§4）
struct TrackingHeadingGroupConfig {
    HeadingMode mode = HeadingMode::kRate;  // 默认速率式（§4.2，见 HeadingMode 注释）
    double gain = 1.0;                  // 增益：视线角(度) → 航向指令（占位，待整定）
    double yaw_rate_limit_dps = 30.0;   // 偏航角速度上限（度/秒，FR-033）
    double yaw_slew_limit_dps2 = 60.0;  // 方向变化率上限（度/秒^2，FR-033、S6 首条限幅）
};

/// vertical 组：垂直/高度控制（§5）
///
/// 规格 §5.2 原设"小误差姿态微调 / 大误差垂直速度"双支路，实测评审后
/// 统一为速度支路：切换阈值与姿态微调参数随之移除。小误差经 vz_gain
/// 线性映射为小垂直速度，行为连续且无模式翻转。
struct TrackingVerticalGroupConfig {
    double vz_gain_mps_per_deg = 0.1;    // 垂直速度增益：α(度) → 垂直速度(米/秒，占位)
    double vz_limit_mps = 2.0;           // 垂直速度限幅（米/秒）
};

/// distance 组：距离通道（§6）；PID 参数为占位，待双目测距落地后整定
struct TrackingDistanceGroupConfig {
    double d_exp_m = 10.0;          // 射程 d_exp（米，由拦截设备接口给出，待定）
    double kp = 0.0;                // PID 比例（待定）
    double ki = 0.0;                // PID 积分
    double kd = 0.0;                // PID 微分
    double integral_limit = 0.0;    // 积分项限幅
    double derivative_filter_coef = 0.0;  // 微分一阶低通系数 ∈ [0,1)：0=不滤波
    double approach_velocity_limit_mps = 3.0;  // 接近速度限幅（米/秒，§8）
    double retreat_velocity_limit_mps = 1.0;   // 后退速度限幅（米/秒，§8）
    // 默认定速接近（D4 远场策略）：双目有效域外无距离是常态，退出仅留给
    // 持续无距离（传感器级故障）场景
    NoDistanceAction no_distance_action = NoDistanceAction::kSlowApproach;  // §6.3
    double no_distance_approach_limit_mps = 0.5;  // kSlowApproach 限速值（米/秒）
    std::int64_t no_distance_exit_grace_ms = 3000;  // kExit 门限：持续无距离达
                                                    // 此时长才退出（毫秒）
};

/// accel_limit 组：各轴加速度限幅（§8，待定）
struct TrackingAccelLimitConfig {
    double ax_mps2 = 2.0;  // 前向加速度限幅（米/秒^2）
    double ay_mps2 = 2.0;  // 侧向加速度限幅（米/秒^2，预留）
    double az_mps2 = 1.5;  // 垂直加速度限幅（米/秒^2）
};

/// 视觉跟踪控制律全量配置（§8：所有阈值集中于此）
struct TrackingConfig {
    TrackingControlGroupConfig control;
    TrackingCameraGroupConfig camera;
    TrackingHeadingGroupConfig heading;
    TrackingVerticalGroupConfig vertical;
    TrackingDistanceGroupConfig distance;
    TrackingAccelLimitConfig accel_limit;

    /// 跨字段/取值校验；非法抛 std::invalid_argument 并携带字段名
    /// （启动期由 config.cpp 解析后调用）。
    void Validate() const;
};

}  // namespace drone::control
