#pragma once

// =============================================================================
// vtc_config.h —— 视觉跟踪控制律配置（TrackingConfig + LoadFromJson）
//
// 对应规格：docs/物理追踪思路.md
//   §8  限幅、阈值与配置化（所有阈值集中配置，不得散落在代码中，
//       需求分析.md NFR-007）
//   §4  水平航向控制（模式、增益、限幅）
//   §5  垂直/高度控制（垂直速度增益与限幅；原双支路已统一为速度支路）
//   §6  距离控制（射程、PID、限幅、无距离降级动作）
//   §7  控制周期与时序（频率、各输入超龄阈值）
//
// 设计约束：
//   - 全部数值待定：本结构默认值与示例配置均为占位，标定前仅限回放验证；
//   - LoadFromJson 启动期一次性解析：缺字段/类型错/非法值抛
//     std::runtime_error 并携带字段名（初始化抛异常规范）；
//   - 热路径绝不触碰 JSON 对象，不做热更新（调参重启即可）。
// =============================================================================

#include <cstdint>
#include <string>

namespace drone::vtc {

/// 航向通道模式（§4.2：位置式/速率式按 PX4 1.11.3 Offboard 实测选择）
///
/// 默认速率式：yaw_rate 指令天然是相对量，无坐标系歧义。位置式输出为
/// 绝对 NED 航向（当前航向 + 限幅修正量合成），依赖新鲜自机姿态；
/// 姿态接线未完成前不得用于实飞。
enum class HeadingMode : std::uint8_t {
    kRate,      // 速率式（默认）：β → 偏航角速度，输出 yaw_rate_dps
    kPosition,  // 位置式：β → 修正量，与当前航向合成绝对航向输出 yaw_deg
};

/// 雷达距离不可用（未关联/超龄）时的距离通道降级动作（§6.3，具体动作待定）
enum class NoDistanceAction : std::uint8_t {
    kHold,         // 保持：前向速度指令清零（安全占位语义）
    kSlowApproach, // 限速接近：按 no_distance_approach_limit_mps 缓速接近
    kExit,         // 退出近距操作：输出降级保持（kDegradedHold）
};

/// control 组：控制周期与各输入超龄阈值（§7）
struct ControlGroupConfig {
    double frequency_hz = 20.0;             // 控制频率（Hz，§7.1 待定）
    std::int64_t visual_stale_ms = 200;     // 视觉追踪结果超龄阈值（毫秒）
    std::int64_t radar_stale_ms = 500;      // 雷达距离超龄阈值（毫秒）
    std::int64_t attitude_stale_ms = 300;   // 自机姿态超龄阈值（毫秒）
};

/// camera 组：画面与视场角（§3.2、§4.1、§5.1；内参/畸变未标定，占位）
struct CameraGroupConfig {
    int image_width = 1920;    // 画面宽度 W（像素）
    int image_height = 1080;   // 画面高度 H（像素）
    double fov_h_deg = 60.0;   // 水平视场角 θ_h（度，仅水平，§4.1）
    double fov_v_deg = 35.0;   // 垂直视场角 θ_v（度，§5.1）
};

/// heading 组：水平航向通道（§4）
struct HeadingGroupConfig {
    HeadingMode mode = HeadingMode::kRate;  // 默认速率式（§4.2，见 HeadingMode 注释）
    double gain = 1.0;                  // 增益：视线角(度) → 航向指令（无量纲占位）
    double yaw_rate_limit_dps = 30.0;   // 偏航角速度上限（度/秒，FR-033）
    double yaw_slew_limit_dps2 = 60.0;  // 方向变化率上限（度/秒^2，FR-033、S6 首条限幅）
};

/// vertical 组：垂直/高度控制（§5）
///
/// 规格 §5.2 原设"小误差姿态微调 / 大误差垂直速度"双支路，实测评审后
/// 统一为速度支路（见 ControlMode 注释）：切换阈值与姿态微调参数随之移除。
/// 小误差经 vz_gain 线性映射为小垂直速度，行为连续且无模式翻转。
struct VerticalGroupConfig {
    double vz_gain_mps_per_deg = 0.1;    // 垂直速度增益：α(度) → 垂直速度(米/秒)
    double vz_limit_mps = 2.0;           // 垂直速度限幅（米/秒）
};

/// distance 组：距离通道（§6）
struct DistanceGroupConfig {
    double d_exp_m = 10.0;          // 射程 d_exp（米，由拦截设备接口给出，§6.1 待定）
    double kp = 0.0;                // PID 比例（§6.2 待定）
    double ki = 0.0;                // PID 积分
    double kd = 0.0;                // PID 微分
    double integral_limit = 0.0;    // 积分项限幅
    double derivative_filter_coef = 0.0;  // 微分一阶低通系数 ∈ [0,1)：0=不滤波
    double approach_velocity_limit_mps = 3.0;  // 接近速度限幅（米/秒，§8）
    double retreat_velocity_limit_mps = 1.0;   // 后退速度限幅（米/秒，§8）
    NoDistanceAction no_distance_action = NoDistanceAction::kHold;  // 无距离降级（§6.3）
    double no_distance_approach_limit_mps = 0.5;  // kSlowApproach 限速值（米/秒）
};

/// accel_limit 组：各轴加速度限幅（§8，待定）
struct AccelLimitConfig {
    double ax_mps2 = 2.0;  // 前向加速度限幅（米/秒^2）
    double ay_mps2 = 2.0;  // 侧向加速度限幅（米/秒^2，预留）
    double az_mps2 = 1.5;  // 垂直加速度限幅（米/秒^2）
};

/// 视觉跟踪控制律全量配置（§8：所有阈值集中于此）
struct TrackingConfig {
    ControlGroupConfig control;
    CameraGroupConfig camera;
    HeadingGroupConfig heading;
    VerticalGroupConfig vertical;
    DistanceGroupConfig distance;
    AccelLimitConfig accel_limit;

    /// 从 JSON 文件一次性加载配置（启动期调用）。
    /// 缺字段 / 类型错误 / 非法取值均抛 std::runtime_error，
    /// 异常消息携带字段名（如 "heading.gain"）。
    static TrackingConfig LoadFromJson(const std::string& path);
};

}  // namespace drone::vtc
