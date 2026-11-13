#include <gtest/gtest.h>

#include <cstdint>

#include "common/types.h"
#include "state_machine/mission_assignment.h"

namespace {

using drone::common::MissionAssignment;
using drone::common::MissionAssignmentReason;
using drone::common::MissionAssignmentResult;
using drone::common::MissionAvailability;
using drone::state_machine::AssignmentPhase;
using drone::state_machine::MissionAssignmentTracker;

MissionAssignment MakeAssignment(uint32_t assignment_id, uint32_t target_id,
                                 uint8_t interceptor_type = 25) {
    MissionAssignment assignment;
    assignment.assignment_id = assignment_id;
    assignment.target_id = target_id;
    assignment.interceptor_type = interceptor_type;
    assignment.flags = 1;  // 授权自动起飞（预检由状态机负责，助手不解释）
    assignment.valid_for_ms = 10000;
    assignment.received_ms = 1000;
    return assignment;
}

}  // namespace

TEST(MissionAssignmentTest, InitialStateIsIdleReady) {
    MissionAssignmentTracker tracker;

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kIdle);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kReady);
    EXPECT_FALSE(tracker.HasActiveAssignment());
    EXPECT_FALSE(tracker.ConsumedAirborneMission());
    EXPECT_EQ(tracker.AssignmentId(), 0U);
    EXPECT_EQ(tracker.TargetId(), 0U);
    EXPECT_EQ(tracker.InterceptorType(), 0U);
    EXPECT_FLOAT_EQ(tracker.OriginX(), 0.f);
    EXPECT_FLOAT_EQ(tracker.OriginY(), 0.f);
    EXPECT_FLOAT_EQ(tracker.OriginZ(), 0.f);
    EXPECT_FALSE(tracker.AllowsTarget(1));
}

TEST(MissionAssignmentTest, AcceptRecordsAssignment) {
    MissionAssignmentTracker tracker;

    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42, 26), 1000));

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kAssigned);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kAssigned);
    EXPECT_TRUE(tracker.HasActiveAssignment());
    EXPECT_FALSE(tracker.ConsumedAirborneMission());
    EXPECT_EQ(tracker.AssignmentId(), 7U);
    EXPECT_EQ(tracker.TargetId(), 42U);
    EXPECT_EQ(tracker.InterceptorType(), 26U);
}

TEST(MissionAssignmentTest, AcceptRejectsWhenNotIdle) {
    MissionAssignmentTracker tracker;

    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    EXPECT_FALSE(tracker.Accept(MakeAssignment(8, 43), 1100));

    // 原分配保持不变，重复消息不得覆盖。
    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kAssigned);
    EXPECT_EQ(tracker.AssignmentId(), 7U);
    EXPECT_EQ(tracker.TargetId(), 42U);
}

TEST(MissionAssignmentTest, AcceptRejectsZeroTarget) {
    MissionAssignmentTracker tracker;

    EXPECT_FALSE(tracker.Accept(MakeAssignment(7, 0), 1000));

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kIdle);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kReady);
    EXPECT_EQ(tracker.AssignmentId(), 0U);
}

TEST(MissionAssignmentTest, BeginTakeoffRecordsOrigin) {
    MissionAssignmentTracker tracker;

    // 空转：未接受分配时不得记录原点。
    tracker.BeginTakeoff(1.f, 2.f, 3.f, 1000);
    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kIdle);
    EXPECT_FLOAT_EQ(tracker.OriginX(), 0.f);

    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    tracker.BeginTakeoff(1.5f, -2.5f, -0.25f, 1200);

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kTakingOff);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kAssigned);
    EXPECT_FLOAT_EQ(tracker.OriginX(), 1.5f);
    EXPECT_FLOAT_EQ(tracker.OriginY(), -2.5f);
    EXPECT_FLOAT_EQ(tracker.OriginZ(), -0.25f);
    EXPECT_FALSE(tracker.ConsumedAirborneMission());
}

TEST(MissionAssignmentTest, MarkAirborneConsumesMissionWithoutAdvancingPhase) {
    MissionAssignmentTracker tracker;
    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    tracker.BeginTakeoff(0.f, 0.f, 0.f, 1200);

    tracker.MarkAirborne(1500);

    // 实际离地只消耗任务机会；S4 到达高度由 TakeoffComplete 推进。
    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kTakingOff);
    EXPECT_TRUE(tracker.ConsumedAirborneMission());
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kAssigned);

    // 重复观测幂等。
    tracker.MarkAirborne(1600);
    EXPECT_TRUE(tracker.ConsumedAirborneMission());
    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kTakingOff);
}

TEST(MissionAssignmentTest, MarkAirborneWithoutAssignmentIsIgnored) {
    MissionAssignmentTracker tracker;

    tracker.MarkAirborne(1000);

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kIdle);
    EXPECT_FALSE(tracker.ConsumedAirborneMission());
}

TEST(MissionAssignmentTest, TakeoffCompleteAdvancesToAirborne) {
    MissionAssignmentTracker tracker;
    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    tracker.BeginTakeoff(0.f, 0.f, 0.f, 1200);
    tracker.MarkAirborne(1500);

    tracker.TakeoffComplete();

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kAirborne);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kEngaged);
    EXPECT_TRUE(tracker.ConsumedAirborneMission());

    // 重复调用幂等，不改变任务机会。
    tracker.TakeoffComplete();
    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kAirborne);
    EXPECT_TRUE(tracker.ConsumedAirborneMission());
}

TEST(MissionAssignmentTest, BeginEngageFromAirborne) {
    MissionAssignmentTracker tracker;
    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    tracker.BeginTakeoff(0.f, 0.f, 0.f, 1200);
    tracker.MarkAirborne(1500);
    tracker.TakeoffComplete();

    tracker.BeginEngage();

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kEngaged);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kEngaged);
    EXPECT_TRUE(tracker.HasActiveAssignment());
}

TEST(MissionAssignmentTest, BeginEngageAllowedWhenAlreadyConsumed) {
    MissionAssignmentTracker tracker;
    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    tracker.BeginTakeoff(0.f, 0.f, 0.f, 1200);
    tracker.MarkAirborne(1500);

    // 调用方未显式 TakeoffComplete，但实际离地已确认。
    tracker.BeginEngage();

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kEngaged);
}

TEST(MissionAssignmentTest, BeginReturnThenTerminal) {
    MissionAssignmentTracker tracker;
    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    tracker.BeginTakeoff(0.f, 0.f, 0.f, 1200);
    tracker.MarkAirborne(1500);
    tracker.TakeoffComplete();
    tracker.BeginEngage();

    tracker.BeginReturn();

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kReturning);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kReturning);
    EXPECT_TRUE(tracker.HasActiveAssignment());

    tracker.Terminal();

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kTerminal);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kTerminal);
    EXPECT_FALSE(tracker.HasActiveAssignment());
}

TEST(MissionAssignmentTest, AbortOnGroundAllowsNewAssignment) {
    MissionAssignmentTracker tracker;
    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    tracker.BeginTakeoff(1.f, 2.f, 3.f, 1200);

    tracker.AbortOnGround();

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kIdle);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kReady);
    EXPECT_FALSE(tracker.HasActiveAssignment());
    EXPECT_FALSE(tracker.ConsumedAirborneMission());
    EXPECT_EQ(tracker.AssignmentId(), 0U);
    EXPECT_EQ(tracker.TargetId(), 0U);
    EXPECT_FLOAT_EQ(tracker.OriginX(), 0.f);

    // 地面中止不消耗任务机会，地面站可用新 assignment_id 重新分配。
    EXPECT_TRUE(tracker.Accept(MakeAssignment(8, 43), 2000));
    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kAssigned);
    EXPECT_EQ(tracker.AssignmentId(), 8U);
    EXPECT_EQ(tracker.TargetId(), 43U);
}

TEST(MissionAssignmentTest, AbortOnGroundAfterAirborneIsIgnored) {
    MissionAssignmentTracker tracker;
    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    tracker.BeginTakeoff(0.f, 0.f, 0.f, 1200);
    tracker.MarkAirborne(1500);

    tracker.AbortOnGround();

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kTakingOff);
    EXPECT_TRUE(tracker.ConsumedAirborneMission());
    EXPECT_TRUE(tracker.HasActiveAssignment());
}

TEST(MissionAssignmentTest, AbortAirborneRequiresActualAirborne) {
    MissionAssignmentTracker tracker;
    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));

    // 仅接受分配、尚未离地：空中中止不应生效。
    tracker.AbortAirborne();
    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kAssigned);

    tracker.BeginTakeoff(0.f, 0.f, 0.f, 1200);
    tracker.MarkAirborne(1500);
    tracker.AbortAirborne();

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kReturning);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kReturning);
    EXPECT_TRUE(tracker.ConsumedAirborneMission());

    tracker.Terminal();
    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kTerminal);
    EXPECT_FALSE(tracker.Accept(MakeAssignment(8, 43), 3000));
}

TEST(MissionAssignmentTest, AbortAirborneEntersReturningFromEngaged) {
    MissionAssignmentTracker tracker;
    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    tracker.BeginTakeoff(0.f, 0.f, 0.f, 1200);
    tracker.MarkAirborne(1500);
    tracker.TakeoffComplete();
    tracker.BeginEngage();

    tracker.AbortAirborne();

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kReturning);
    EXPECT_TRUE(tracker.ConsumedAirborneMission());
}

TEST(MissionAssignmentTest, AllowsTargetOnlyMatchesActiveTarget) {
    MissionAssignmentTracker tracker;
    EXPECT_FALSE(tracker.AllowsTarget(42));

    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    EXPECT_TRUE(tracker.AllowsTarget(42));
    EXPECT_FALSE(tracker.AllowsTarget(43));
    EXPECT_FALSE(tracker.AllowsTarget(0));

    tracker.BeginTakeoff(0.f, 0.f, 0.f, 1200);
    tracker.MarkAirborne(1500);
    tracker.TakeoffComplete();
    tracker.BeginEngage();
    tracker.BeginReturn();
    // 返航阶段仍持有该分配，但仅匹配分配目标。
    EXPECT_TRUE(tracker.AllowsTarget(42));
    EXPECT_FALSE(tracker.AllowsTarget(43));

    tracker.Terminal();
    EXPECT_FALSE(tracker.AllowsTarget(42));
}

TEST(MissionAssignmentTest, AvailabilityMapsEveryPhase) {
    MissionAssignmentTracker tracker;
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kReady);

    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kAssigned);

    tracker.BeginTakeoff(0.f, 0.f, 0.f, 1200);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kAssigned);

    tracker.MarkAirborne(1500);
    tracker.TakeoffComplete();
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kEngaged);

    tracker.BeginEngage();
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kEngaged);

    tracker.BeginReturn();
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kReturning);

    tracker.Terminal();
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kTerminal);
}

TEST(MissionAssignmentTest, TerminalFromIdleRejectsAssignment) {
    MissionAssignmentTracker tracker;

    tracker.Terminal();

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kTerminal);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kTerminal);
    EXPECT_FALSE(tracker.Accept(MakeAssignment(7, 42), 1000));
    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kTerminal);
    EXPECT_EQ(tracker.AssignmentId(), 0U);
}

TEST(MissionAssignmentTest, IllegalCallsDoNotChangeState) {
    MissionAssignmentTracker tracker;

    tracker.TakeoffComplete();
    tracker.BeginEngage();
    tracker.BeginReturn();
    tracker.AbortAirborne();
    tracker.AbortOnGround();

    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kIdle);
    EXPECT_EQ(tracker.Availability(), MissionAvailability::kReady);

    ASSERT_TRUE(tracker.Accept(MakeAssignment(7, 42), 1000));
    // 已接受但未起飞/未消耗时，任务/返航推进均不生效。
    tracker.BeginEngage();
    tracker.BeginReturn();
    EXPECT_EQ(tracker.Phase(), AssignmentPhase::kAssigned);
}

TEST(MissionAssignmentTest, RememberedResultCanBeResentForRepeatedId) {
    MissionAssignmentTracker tracker;
    tracker.RememberResult(7, MissionAssignmentResult::kAccepted,
                           MissionAssignmentReason::kNone);

    MissionAssignmentResult result = MissionAssignmentResult::kRejected;
    MissionAssignmentReason reason = MissionAssignmentReason::kInternalError;
    EXPECT_TRUE(tracker.WasHandled(7, &result, &reason));
    EXPECT_EQ(result, MissionAssignmentResult::kAccepted);
    EXPECT_EQ(reason, MissionAssignmentReason::kNone);

    EXPECT_FALSE(tracker.WasHandled(8, &result, &reason));
    // 未命中不修改输出参数。
    EXPECT_EQ(result, MissionAssignmentResult::kAccepted);
    EXPECT_EQ(reason, MissionAssignmentReason::kNone);

    // 空指针查询只判断是否已处理。
    EXPECT_TRUE(tracker.WasHandled(7, nullptr, nullptr));
}

TEST(MissionAssignmentTest, RememberedResultKeepsLatestForSameId) {
    MissionAssignmentTracker tracker;
    tracker.RememberResult(7, MissionAssignmentResult::kAccepted,
                           MissionAssignmentReason::kNone);
    tracker.RememberResult(7, MissionAssignmentResult::kCancelledOrAborted,
                           MissionAssignmentReason::kBatteryLow);

    MissionAssignmentResult result = MissionAssignmentResult::kAccepted;
    MissionAssignmentReason reason = MissionAssignmentReason::kNone;
    ASSERT_TRUE(tracker.WasHandled(7, &result, &reason));
    EXPECT_EQ(result, MissionAssignmentResult::kCancelledOrAborted);
    EXPECT_EQ(reason, MissionAssignmentReason::kBatteryLow);
}

TEST(MissionAssignmentTest, RememberedResultsAreBounded) {
    MissionAssignmentTracker tracker;
    ASSERT_TRUE(tracker.Accept(MakeAssignment(100, 42), 1000));
    tracker.RememberResult(100, MissionAssignmentResult::kAccepted,
                           MissionAssignmentReason::kNone);

    // 再记录 7 条不同 id，共 8 条（上限）。
    for (uint32_t id = 1; id <= 7; ++id) {
        tracker.RememberResult(id, MissionAssignmentResult::kRejected,
                               MissionAssignmentReason::kNotReady);
    }
    for (uint32_t id = 1; id <= 7; ++id) {
        EXPECT_TRUE(tracker.WasHandled(id, nullptr, nullptr)) << id;
    }
    EXPECT_TRUE(tracker.WasHandled(100, nullptr, nullptr));

    // 第 9 条覆盖最旧的 id=100，内存不无界增长。
    tracker.RememberResult(8, MissionAssignmentResult::kRejected,
                           MissionAssignmentReason::kNotReady);
    EXPECT_FALSE(tracker.WasHandled(100, nullptr, nullptr));
    EXPECT_TRUE(tracker.WasHandled(8, nullptr, nullptr));
}
