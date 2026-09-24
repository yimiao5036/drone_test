#pragma once

// =============================================================================
// visual_tracking_shadow.h —— 视觉跟踪控制律影子适配层（VisualTrackingShadow）
//
// 对应设计：docs/superpowers/specs/2026-09-23-视觉跟踪控制律移植-design.md
//
// 职责：
//   以固定周期（control.frequency_hz）组装控制快照（视觉稳定性状态 +
//   飞行姿态 + 距离占位），驱动 TrackingControlLaw，把 ControlOutput 映射为
//   ControlIntent 发布到 kControlIntent。纯影子运行：kControlIntent 当前无
//   消费者，enable_control=false 基线不动，不产生任何真实控制输出。
//
// 边界：
//   - 双目测距（可选输入 kStereoTargetDistance）：最新一条 valid 且
//     receive_time_ms>0 → distance_valid=true 并填 target_distance_m；
//     否则维持 no_distance_action 支路（超龄判定由控制律按
//     control.distance_stale_ms 执行，影子只透传）；
//   - 现场无 PX4 时姿态缺失：virtual_attitude_when_absent=true（默认）用
//     零姿态继续计算（仅供观察控制律行为，日志明确标注）；false 时保持
//     真实语义（姿态缺失每拍降级保持）；
//   - 不做多目标选举、不做 NED 空间转换（后者属 TargetEstimator 融合阶段）。
//
// 状态映射：VisualTargetStatus.state==kLocked → tracked=true，其余 false；
// is_predicted 恒 false（主链路无预测）。
//
// ControlIntent.reason_code（影子段，100 起，避开骨架 0-7）：
//   100=正常跟踪 101=Coast 102=判丢刹车 103=姿态超龄降级
//   104=视觉超龄降级 105=持续无距离退出降级
// 模式映射：kVelocityHeading→kVelocityHeading（target_x/y/z=NED 速度，
// 速率式填 yaw_rate_dps、位置式填 yaw_deg）；kBrakeHover→kBrakeHover；
// kDegradedHold→kNone。
//
// 数据流：
//   Topic<VisualTargetStatus> ──┐
//   Topic<FlightStateSnapshot>──┼─► VisualTrackingShadow ──► Topic<ControlIntent>
//   Topic<StereoTargetDistance>─┘   （20Hz 固定节拍）
//
// 线程模型：独立节拍线程（固定周期，不等待消息被动触发，§7.1）。
// Stop() 置停止标志后 join；Start/Stop 幂等。
// =============================================================================

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include "common/topic.h"
#include "common/types.h"
#include "control/tracking_config.h"
#include "control/tracking_control_law.h"

namespace drone::control {

/// 影子适配层配置（控制律参数在 TrackingConfig；此处只含影子行为）。
struct VisualTrackingShadowConfig {
    /// 姿态缺失/超龄时是否以零姿态继续计算（现场无 PX4 的观察开关）。
    bool virtual_attitude_when_absent = true;
    /// 周期性摘要日志间隔（观察控制律输出趋势）。
    std::chrono::milliseconds summary_log_interval{10000};

    void Validate() const;
};

/// 视觉跟踪控制律影子部件。
class VisualTrackingShadow final {
public:
    VisualTrackingShadow(TrackingConfig tracking_config,
                         VisualTrackingShadowConfig shadow_config);
    ~VisualTrackingShadow();

    VisualTrackingShadow(const VisualTrackingShadow&) = delete;
    VisualTrackingShadow& operator=(const VisualTrackingShadow&) = delete;

    // ---- 生命周期 ----
    bool Start();   ///< 输入未接线或已运行返回 false；幂等
    void Stop();    ///< 幂等
    bool IsRunning() const;

    // ---- 输入 ----
    /// 视觉稳定性状态输入（必需，kVisualTargetStatus）。
    void SetVisualInput(common::Topic<common::VisualTargetStatus>& input);
    /// 飞行姿态输入（可选；未接线时按姿态缺失处理，由虚拟姿态配置兜底）。
    void SetFlightInput(common::Topic<common::FlightStateSnapshot>& input);
    /// 双目测距距离输入（可选，kStereoTargetDistance；未接线时距离恒无效）。
    void SetDistanceInput(common::Topic<common::StereoTargetDistance>& input);

    // ---- 输出 ----
    common::Topic<common::ControlIntent>& IntentOutput();

    // ---- 状态查询 ----
    std::uint64_t IntentCount() const;       ///< 累计发布意图数
    std::uint64_t TickCount() const;         ///< 累计控制节拍数
    bool LastUsedVirtualAttitude() const;    ///< 最近一拍是否使用虚拟姿态

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace drone::control
