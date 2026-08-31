#pragma once

// =============================================================================
// tracking_control_law.h —— 视觉跟踪主控律（TrackingControlLaw）
//
// 对应规格：docs/物理追踪思路.md
//   §4  水平航向控制（增益 → slew → 角速度限幅，位置式/速率式按配置）
//   §5  垂直/高度控制（统一速度支路：α → 垂直速度，原双支路已合并）
//   §6  距离控制（PID(Δd) → 前向速度；未关联按配置降级，§6.3）
//   §6.4 Coast 清积分、判丢零速刹车
//   §7.1 超龄判定（姿态/视觉超龄 → 降级保持）
//   §8   统一限幅
//   §2.3 输出端随机体航向把机体系速度旋转为局部 NED
//
// 线程模型：单线程调用（状态机与控制线程），成员无锁、无堆对象；
// 日志仅状态切换点（刹车进出、降级切换），高频热路径不打日志。
// =============================================================================

#include <cstdint>

#include "visual_track_control/line_of_sight.h"
#include "visual_track_control/pid_controller.h"
#include "visual_track_control/rate_limiter.h"
#include "visual_track_control/vtc_config.h"
#include "visual_track_control/vtc_types.h"

namespace drone::vtc {

/// 视觉跟踪主控律：消费控制快照，输出限幅后的控制意图
class TrackingControlLaw {
public:
    /// cfg：启动期加载的全量配置（值拷贝持有，运行期不再触碰配置源）
    explicit TrackingControlLaw(const TrackingConfig& cfg);

    /// 单周期控制计算（§7.1 固定周期持续运行）。
    /// now_ms：当前单调时钟毫秒；内部据此推算 dt 并做超龄判定。
    /// 流程：①超龄判定 → ②判丢刹车 → ③Coast 清积分 → ④像素→视线角
    ///       → ⑤水平航向 → ⑥垂直速度 → ⑦距离通道 → ⑧机体系→NED 旋转输出
    ControlOutput Update(const ControlSnapshot& snapshot, std::int64_t now_ms);

    /// 停止视觉闭环时清全部状态（积分、限幅器历史、节拍时钟，§6.4）
    void Reset();

private:
    TrackingConfig cfg_;  // 配置值拷贝（运行期只读）

    // ---- 预计算构件（构造期完成，热路径零分配） ----
    PixelToAngle pixel_to_beta_;   // 水平：x_offset → β（§4.1）
    PixelToAngle pixel_to_alpha_;  // 垂直：y_offset → α（§5.1）
    PidController distance_pid_;   // 距离通道 PID（§6.2）

    // ---- 变化率限幅器（§8 / S6 首条指令限幅） ----
    SlewRateLimiter yaw_deg_limiter_;    // 位置式航向变化率（度/秒^2）
    SlewRateLimiter yaw_rate_limiter_;   // 速率式角速度变化率（度/秒^2）
    SlewRateLimiter vx_limiter_;         // 前向加速度限幅（米/秒^2）
    SlewRateLimiter vz_limiter_;         // 垂直加速度限幅（米/秒^2）

    // ---- 节拍与状态（仅打日志用，不参与数学） ----
    std::int64_t last_now_ms_ = 0;                       // 上次计算时刻
    ControlMode last_mode_ = ControlMode::kDegradedHold; // 上次输出模式（状态切换日志）
};

}  // namespace drone::vtc
