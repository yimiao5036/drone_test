#include "state_machine/mission_assignment.h"

namespace drone::state_machine {
namespace {

/// 活动分配阶段：kAssigned~kReturning。kIdle 与 kTerminal 不持有分配。
bool IsActivePhase(AssignmentPhase phase) {
    return phase == AssignmentPhase::kAssigned ||
           phase == AssignmentPhase::kTakingOff ||
           phase == AssignmentPhase::kAirborne ||
           phase == AssignmentPhase::kEngaged ||
           phase == AssignmentPhase::kReturning;
}

}  // namespace

bool MissionAssignmentTracker::Accept(const common::MissionAssignment& assignment,
                                      uint64_t now_ms) {
    // 一次上电一次空中任务：已消耗机会后即使处于 kIdle 也不再接受。
    if (phase_ != AssignmentPhase::kIdle || consumed_ ||
        assignment.target_id == 0) {
        return false;
    }

    assignment_id_ = assignment.assignment_id;
    target_id_ = assignment.target_id;
    interceptor_type_ = assignment.interceptor_type;
    // 原点在 BeginTakeoff 时才记录，避免沿用上一次地面中止的旧值。
    origin_x_ = 0.f;
    origin_y_ = 0.f;
    origin_z_ = 0.f;
    (void)now_ms;  // 助手不读时钟；分配有效期由状态机按调用方时间判定
    phase_ = AssignmentPhase::kAssigned;
    return true;
}

void MissionAssignmentTracker::BeginTakeoff(float x, float y, float z,
                                            uint64_t now_ms) {
    if (phase_ != AssignmentPhase::kAssigned) {
        return;
    }
    origin_x_ = x;
    origin_y_ = y;
    origin_z_ = z;
    (void)now_ms;  // 起飞时刻由状态机的 S3 超时逻辑另行计时
    phase_ = AssignmentPhase::kTakingOff;
}

void MissionAssignmentTracker::MarkAirborne(uint64_t now_ms) {
    // 只有已持有分配时才谈得上“消耗本次任务机会”；无分配（手动飞行）
    // 不得把整机永久锁死。
    if (!IsActivePhase(phase_)) {
        return;
    }
    // 幂等：无论调用多少次，本次上电的机会只消耗一次。
    consumed_ = true;
    (void)now_ms;
}

void MissionAssignmentTracker::TakeoffComplete() {
    if (phase_ == AssignmentPhase::kTakingOff) {
        phase_ = AssignmentPhase::kAirborne;
    }
}

void MissionAssignmentTracker::BeginEngage() {
    // 正常路径为 kAirborne；已观测到实际离地但尚未显式 TakeoffComplete 时
    // 也允许进入任务阶段，避免因可选状态推进缺失而卡在起飞阶段。
    if (phase_ == AssignmentPhase::kAirborne ||
        (consumed_ && phase_ == AssignmentPhase::kTakingOff)) {
        phase_ = AssignmentPhase::kEngaged;
    }
}

void MissionAssignmentTracker::BeginReturn() {
    if (phase_ == AssignmentPhase::kAirborne ||
        phase_ == AssignmentPhase::kEngaged) {
        phase_ = AssignmentPhase::kReturning;
    }
}

void MissionAssignmentTracker::Terminal() {
    // 保留 assignment_id/target_id 供 65013 展示最后一次分配；Accept 由
    // kTerminal 阶段直接拒绝。
    phase_ = AssignmentPhase::kTerminal;
}

void MissionAssignmentTracker::AbortOnGround() {
    if (consumed_ || (phase_ != AssignmentPhase::kAssigned &&
                      phase_ != AssignmentPhase::kTakingOff)) {
        return;
    }
    ClearAssignment();
    phase_ = AssignmentPhase::kIdle;
}

void MissionAssignmentTracker::AbortAirborne() {
    if (phase_ == AssignmentPhase::kIdle || phase_ == AssignmentPhase::kTerminal) {
        return;
    }
    // 实际离地是权威判据：只要消耗过任务机会，即使阶段记录滞后也按空中中止。
    if (!consumed_ && phase_ != AssignmentPhase::kAirborne &&
        phase_ != AssignmentPhase::kEngaged &&
        phase_ != AssignmentPhase::kReturning) {
        return;
    }
    phase_ = AssignmentPhase::kReturning;
}

bool MissionAssignmentTracker::AllowsTarget(uint32_t target_id) const {
    return target_id != 0 && IsActivePhase(phase_) && target_id == target_id_;
}

common::MissionAvailability MissionAssignmentTracker::Availability() const {
    switch (phase_) {
        case AssignmentPhase::kIdle:
            // 可调度性（控制路径、健康、电池）由状态机预检覆盖。
            return common::MissionAvailability::kReady;
        case AssignmentPhase::kAssigned:
        case AssignmentPhase::kTakingOff:
            return common::MissionAvailability::kAssigned;
        case AssignmentPhase::kAirborne:
        case AssignmentPhase::kEngaged:
            return common::MissionAvailability::kEngaged;
        case AssignmentPhase::kReturning:
            return common::MissionAvailability::kReturning;
        case AssignmentPhase::kTerminal:
            return common::MissionAvailability::kTerminal;
    }
    return common::MissionAvailability::kUnavailable;
}

bool MissionAssignmentTracker::HasActiveAssignment() const {
    return IsActivePhase(phase_);
}

void MissionAssignmentTracker::RememberResult(
    uint32_t assignment_id, common::MissionAssignmentResult result,
    common::MissionAssignmentReason reason) {
    // 同一 assignment_id 覆盖为最新结果，保证重复消息重发的是最终回执。
    for (std::size_t i = 0; i < result_count_; ++i) {
        if (results_[i].assignment_id == assignment_id) {
            results_[i].result = result;
            results_[i].reason = reason;
            return;
        }
    }

    if (result_count_ < kMaxRememberedResults) {
        results_[result_count_] = RememberedResult{assignment_id, result, reason};
        ++result_count_;
        return;
    }

    // 有界环形覆盖：写满后覆盖最旧记录，不无界增长。
    results_[result_next_] = RememberedResult{assignment_id, result, reason};
    result_next_ = (result_next_ + 1) % kMaxRememberedResults;
}

bool MissionAssignmentTracker::WasHandled(
    uint32_t assignment_id, common::MissionAssignmentResult* result,
    common::MissionAssignmentReason* reason) const {
    for (std::size_t i = 0; i < result_count_; ++i) {
        if (results_[i].assignment_id != assignment_id) {
            continue;
        }
        if (result != nullptr) {
            *result = results_[i].result;
        }
        if (reason != nullptr) {
            *reason = results_[i].reason;
        }
        return true;
    }
    return false;
}

void MissionAssignmentTracker::ClearAssignment() {
    assignment_id_ = 0;
    target_id_ = 0;
    interceptor_type_ = 0;
    origin_x_ = 0.f;
    origin_y_ = 0.f;
    origin_z_ = 0.f;
}

}  // namespace drone::state_machine
