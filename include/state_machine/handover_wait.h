#pragma once

#include <chrono>
#include <cstdint>

namespace drone {
namespace state_machine {

/// 到点判定与「到点后视觉未识别」等待计时的纯逻辑实现。
///
/// 背景（2026-11-05，S5 到点不告警问题）：现有目标看门狗
/// `EvaluateTargetWatchdog()` 只在目标**失效**时计时（wait/lost/hold 三条），
/// 而现场 65010 以 5 Hz 持续送达、`HasFreshEstimate()` 每拍为真，三条计时器
/// 都不会启动——飞机在目标点上方悬停数分钟、视觉整段 SEARCHING，地面站却
/// 收不到任何提示。本模块补上「飞机已经到点、但视觉还没锁定目标」这一段
/// 独立的判据，与目标看门狗的语义互不干扰：
///
/// - 到点：水平/垂直距离都在半径内、地速足够小，且**连续保持** arrival_hold；
/// - 到点后等待：从首次判定到点起累计"视觉未锁定"的时长；
/// - 告警：等待超过 visual_warn 抬告警位（bit6）；visual_abort 非 0 时再超时
///   交给状态机中止（默认 0 = 只提示、不自动处置）。
///
/// 本文件只做判断与计时，不持有 Topic、不打印日志、不参与控制意图，
/// 便于在单元测试中直接构造样本序列验证迟滞与边界。
struct HandoverWaitParams {
    double arrival_radius_m = 3.0;      ///< 到点水平半径（m）
    double arrival_vertical_m = 2.0;    ///< 到点垂直半径（m）
    double arrival_speed_mps = 0.5;     ///< 判为"停稳"的地速上限（m/s）
    std::chrono::milliseconds arrival_hold{2000};    ///< 到点需连续保持的时长
    std::chrono::milliseconds visual_warn{20000};    ///< 到点后未识别的告警阈值
    std::chrono::milliseconds visual_abort{0};       ///< 0 = 不自动处置
};

/// 单拍到点样本：水平/垂直相对目标点的距离、地速、视觉是否锁定。
struct HandoverWaitSample {
    double horizontal_m = 0.0;      ///< 水平距离（m）
    double vertical_m = 0.0;        ///< 垂直距离（m，取绝对值）
    double ground_speed_mps = 0.0;  ///< 地速（m/s）
    bool visual_locked = false;     ///< 视觉状态是否 LOCKED 且未超龄
};

/// 判据阶段。idle=没有可用目标点；enroute=未到点；settling=已进半径但保持
/// 时长未满；arrived=已到点且视觉已锁定；no_visual=已到点但视觉未锁定。
enum class HandoverWaitPhase {
    kIdle,
    kEnroute,
    kSettling,
    kArrived,
    kNoVisual,
};

/// 一拍评估结果，供状态机抬告警位/中止使用。
struct HandoverWaitStatus {
    HandoverWaitPhase phase = HandoverWaitPhase::kIdle;
    uint64_t waited_ms = 0;   ///< 到点后累计未识别时长（视觉锁定/离开到点即清零）
    bool warn = false;        ///< waited_ms >= visual_warn
    bool abort = false;       ///< visual_abort > 0 且 waited_ms >= visual_abort
};

/// 到点/等待计时器。状态全在成员里，`Reset()` 之后与新建等价。
///
/// 迟滞：进入半径用 arrival_radius_m / arrival_vertical_m，离开用其 1.5 倍，
/// 避免在半径边缘抖动导致计时反复清零（设计口径与 handover.exit_lost_frames
/// 的"进入/退出门限不同"一致）。
class HandoverWaitTracker {
public:
    /// 推进一步。`sample` 由调用方每拍组装（无可用目标点时请直接调用 Reset）。
    HandoverWaitStatus Update(const HandoverWaitSample& sample, uint64_t now_ms,
                              const HandoverWaitParams& params) {
        const bool inside = InsideEntryBoundary(sample, params);
        if (!inside && !arrived_ && !settling_) {
            // 未到点且之前也没到过：保持 enroute，清掉残留计时。
            arrived_at_ms_ = 0;
            waiting_since_ms_ = 0;
            return MakeStatus(HandoverWaitPhase::kEnroute, 0, params);
        }
        if (!inside) {
            if (InsideExitBoundary(sample, params)) {
                // 迟滞带内：维持上一拍的到点/保持判定，不推进也不清零。
                return Emit(now_ms, sample.visual_locked, params);
            }
            // 真正离开：放弃到点判定与全部计时。
            Reset();
            return MakeStatus(HandoverWaitPhase::kEnroute, 0, params);
        }

        const bool stopped = sample.ground_speed_mps <= params.arrival_speed_mps;
        if (!arrived_) {
            if (!stopped) {
                settling_ = false;
                settle_since_ms_ = 0;
                return MakeStatus(HandoverWaitPhase::kEnroute, 0, params);
            }
            if (!settling_) {
                settling_ = true;
                settle_since_ms_ = now_ms;
            }
            const uint64_t hold_ms = static_cast<uint64_t>(params.arrival_hold.count());
            if (now_ms - settle_since_ms_ >= hold_ms) {
                arrived_ = true;
                arrived_at_ms_ = now_ms;
                waiting_since_ms_ = now_ms;
            } else {
                return MakeStatus(HandoverWaitPhase::kSettling, 0, params);
            }
        }
        return Emit(now_ms, sample.visual_locked, params);
    }

    /// 视觉锁定 / 目标点失效 / 状态迁移时复位全部计时。
    void Reset() {
        arrived_ = false;
        settling_ = false;
        settle_since_ms_ = 0;
        arrived_at_ms_ = 0;
        waiting_since_ms_ = 0;
    }

    bool arrived() const { return arrived_; }
    /// 到点时刻（0 表示未到点）。
    uint64_t arrived_at_ms() const { return arrived_at_ms_; }

private:
    static bool InsideEntryBoundary(const HandoverWaitSample& sample,
                                    const HandoverWaitParams& params) {
        return sample.horizontal_m <= params.arrival_radius_m &&
               sample.vertical_m <= params.arrival_vertical_m;
    }

    static bool InsideExitBoundary(const HandoverWaitSample& sample,
                                   const HandoverWaitParams& params) {
        return sample.horizontal_m <= params.arrival_radius_m * 1.5 &&
               sample.vertical_m <= params.arrival_vertical_m * 1.5;
    }

    HandoverWaitStatus Emit(uint64_t now_ms, bool visual_locked,
                            const HandoverWaitParams& params) {
        if (visual_locked) {
            // 视觉已锁定：等待计时清零，交回交接链路继续推进（S5→S6）。
            waiting_since_ms_ = 0;
            return MakeStatus(HandoverWaitPhase::kArrived, 0, params);
        }
        if (waiting_since_ms_ == 0) {
            waiting_since_ms_ = now_ms;
        }
        const uint64_t waited_ms = now_ms - waiting_since_ms_;
        return MakeStatus(HandoverWaitPhase::kNoVisual, waited_ms, params);
    }

    static HandoverWaitStatus MakeStatus(HandoverWaitPhase phase, uint64_t waited_ms,
                                         const HandoverWaitParams& params) {
        HandoverWaitStatus status;
        status.phase = phase;
        status.waited_ms = waited_ms;
        const uint64_t warn_ms = static_cast<uint64_t>(params.visual_warn.count());
        status.warn = warn_ms > 0 && waited_ms >= warn_ms;
        const uint64_t abort_ms = static_cast<uint64_t>(params.visual_abort.count());
        status.abort = abort_ms > 0 && waited_ms >= abort_ms;
        return status;
    }

    bool arrived_ = false;            ///< 已满足到点保持条件
    bool settling_ = false;           ///< 已进半径、正在累计保持时长
    uint64_t settle_since_ms_ = 0;    ///< 保持计时起点
    uint64_t arrived_at_ms_ = 0;      ///< 判定到点的时刻（日志/状态用）
    uint64_t waiting_since_ms_ = 0;   ///< 未识别计时起点（0 = 视觉已锁定）
};

}  // namespace state_machine
}  // namespace drone
