#pragma once

// =============================================================================
// vtc_types.h —— 视觉跟踪控制律的中立输入/输出结构
//
// 对应规格：docs/物理追踪思路.md
//   §2   输入输出与数据流（控制意图枚举语义）
//   §6.4 与追踪库的行为衔接（tracked / is_predicted 语义）
//   §7.2 与 target_tracker 的衔接（TrackResult 无时间戳，由调用方在
//        LastResult() 瞬间打戳）
//
// 设计约束：
//   - ControlSnapshot / ControlOutput 均为纯值类型（POD 风格），
//     控制律热路径零堆分配；
//   - 本库不依赖主工程 include/ 的任何类型，独立可编译；
//   - 雷达距离必须带 associated_to_target 关联标志：未关联不得当作
//     目标距离（§6.3、需求分析.md FR-041）。
// =============================================================================

#include <cstdint>

#include <target_tracker/target_tracker.h>

namespace drone::vtc {

/// 控制输出模式（对应物理追踪思路 §2.2 的控制意图语义）
///
/// 注意：规格 §5.2 原设"垂直小误差姿态微调支路"（kAttitudeTrim →
/// SET_ATTITUDE_TARGET），实测评审后取消——姿态微调支路会把整个输出切成
/// 姿态消息，导致同周期的距离通道前向速度被丢弃（逼近停滞），且双消息
/// 类型逐拍翻转会扰动 PX4 offboard。现垂直通道一律走速度支路（小误差
/// 对应小 vz），全周期单一 offboard 消息。
enum class ControlMode : std::uint8_t {
    kVelocityHeading,  // 速度与航向意图：速度分量 + yaw（§4.2、§5、§6.2 主输出）
    kBrakeHover,       // 零速刹车/悬停：判丢等场景（§6.4、§9.1）
    kDegradedHold,     // 降级保持：超龄/退出近距操作，禁止正常控制（§7.1、§9.2）
};

/// 降级原因（仅 kDegradedHold 模式有意义）
enum class DegradedReason : std::uint8_t {
    kNone,             // 未降级
    kAttitudeStale,    // PX4 姿态过期：禁止空间转换和控制（§9.2）
    kVisionStale,      // 视觉追踪结果超龄：不得继续生成正常指令（§7.1）
    kNoDistanceExit,   // 无距离关联且配置为退出近距操作（§6.3）
};

/// 控制律输入快照：调用方在同一控制周期内采集并打戳
///
/// 注意：TrackResult 本身不含时间戳（§7.2），调用方必须在读取
/// LastResult() 的瞬间用单调时钟打戳填入 track_time_ms；
/// 超龄判定一律使用"取样时间 + valid_for_ms"（§7.1）。
struct ControlSnapshot {
    // ---- 视觉追踪结果（控制律主输入） ----
    drone::tracker::TrackResult track;  // 追踪库结果（值拷贝）
    std::int64_t track_time_ms = 0;     // 取样单调时钟（毫秒），<=0 视为从未取样

    // ---- 雷达距离（§6.3：未关联仅为前向安全距离，不得当目标距离） ----
    double radar_distance_m = 0.0;      // 机头前向单点距离（米）
    bool associated_to_target = false;  // 相机—雷达空间关联成立标志
    std::int64_t radar_time_ms = 0;     // 取样单调时钟（毫秒）

    // ---- 自机姿态/速度 ----
    // 姿态过期 → 禁止控制（§9.2）。`yaw_rad` 参与输出端机体系→NED 旋转
    // （§2.3）与位置式绝对航向合成；其余字段预留，接入主工程时由
    // FlightStateSnapshot 填充（机体补偿见 §4.1）。
    bool attitude_present = false;      // 姿态源当前是否可用
    std::int64_t attitude_time_ms = 0;  // 姿态取样单调时钟（毫秒）
    double roll_rad = 0.0;              // 横滚（弧度，FRD）
    double pitch_rad = 0.0;             // 俯仰（弧度，FRD）
    double yaw_rad = 0.0;               // 航向（弧度，NED）
    double yaw_rate_radps = 0.0;        // 偏航角速度（弧度/秒，机体补偿预留，§4.1）
    double velocity_x_mps = 0.0;        // 自机速度分量（预留）
    double velocity_y_mps = 0.0;
    double velocity_z_mps = 0.0;
};

/// 控制律输出：限幅后的控制意图（§8：所有输出必须经过限幅）
///
/// 坐标语义：速度分量在机体系（前/右/下，§2.3）内计算与限幅，输出前
/// 已随自机航向（ControlSnapshot.yaw_rad）旋转为局部 NED（北/东/地），
/// 与网关使用的 MAV_FRAME_LOCAL_NED 一致；因此旋转后 vy_mps 可能非零
/// （机体系侧向恒为 0，旋转不改变合速度大小，限幅在旋转前完成）。
struct ControlOutput {
    ControlMode mode = ControlMode::kDegradedHold;  // 输出模式
    bool valid = false;                             // 指令是否可用于发送

    // ---- 速度分量（局部 NED：北/东/地，米/秒；由机体系随航向旋转得到） ----
    double vx_mps = 0.0;   // 北向（距离通道输出经旋转，§6.2）
    double vy_mps = 0.0;   // 东向（机体系恒 0，航向旋转后可非零）
    double vz_mps = 0.0;   // 垂直（NED 下正，垂直速度支路输出，§5.2）

    // ---- 航向通道（§4.2：位置式输出 yaw_deg，速率式输出 yaw_rate_dps） ----
    double yaw_deg = 0.0;       // 目标绝对航向（度，NED，右转为正；仅位置式，
                                // 由当前航向 + 限幅修正量合成并回绕 [-180,180)）
    double yaw_rate_dps = 0.0;  // 偏航角速度指令（度/秒；速率式为默认，天然相对量）

    // ---- 状态标注 ----
    DegradedReason degraded_reason = DegradedReason::kNone;  // 降级原因
    bool coast_active = false;  // 本周期处于 Coast（is_predicted==true，§6.4）
};

}  // namespace drone::vtc
