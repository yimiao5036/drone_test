// 到点判定与「到点后视觉未识别」等待计时（handover_wait.h）单元测试。
//
// 覆盖：进入/退出半径迟滞、保持时长门槛、地速门槛、告警与中止阈值、
// 视觉锁定清零、Reset 语义。全部用整数毫秒模拟决策拍，不依赖真实时钟。
#include "state_machine/handover_wait.h"

#include <gtest/gtest.h>

#include <chrono>

namespace drone {
namespace state_machine {
namespace {

HandoverWaitParams MakeParams() {
    HandoverWaitParams params;
    params.arrival_radius_m = 3.0;
    params.arrival_vertical_m = 2.0;
    params.arrival_speed_mps = 0.5;
    params.arrival_hold = std::chrono::milliseconds(2000);
    params.visual_warn = std::chrono::milliseconds(20000);
    params.visual_abort = std::chrono::milliseconds(0);
    return params;
}

HandoverWaitSample MakeSample(double horizontal_m, double vertical_m, double speed_mps,
                              bool visual_locked) {
    HandoverWaitSample sample;
    sample.horizontal_m = horizontal_m;
    sample.vertical_m = vertical_m;
    sample.ground_speed_mps = speed_mps;
    sample.visual_locked = visual_locked;
    return sample;
}

TEST(HandoverWaitTrackerTest, StaysEnrouteWithoutAccumulatingWaitWhenOutsideRadius) {
    HandoverWaitTracker tracker;
    const auto params = MakeParams();
    const auto status = tracker.Update(MakeSample(10.0, 0.0, 0.0, false), 1000, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kEnroute);
    EXPECT_EQ(status.waited_ms, 0U);
    EXPECT_FALSE(status.warn);
    EXPECT_FALSE(tracker.arrived());
}

TEST(HandoverWaitTrackerTest, SpeedAboveLimitDoesNotCountAsArrived) {
    HandoverWaitTracker tracker;
    const auto params = MakeParams();
    auto status = tracker.Update(MakeSample(1.0, 0.5, 2.0, false), 1000, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kEnroute);
    status = tracker.Update(MakeSample(1.0, 0.5, 2.0, false), 5000, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kEnroute);
    EXPECT_FALSE(tracker.arrived());
}

TEST(HandoverWaitTrackerTest, SettlingUntilHoldElapsedThenNoVisual) {
    HandoverWaitTracker tracker;
    const auto params = MakeParams();
    auto status = tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 1000, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kSettling);
    EXPECT_FALSE(tracker.arrived());
    // 保持未满 2 s，仍在 settling。
    status = tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 2500, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kSettling);
    // 满 2 s 当拍判定到点；视觉未锁定 → no_visual，等待从当拍起算。
    status = tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 3000, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kNoVisual);
    EXPECT_TRUE(tracker.arrived());
    EXPECT_EQ(tracker.arrived_at_ms(), 3000U);
    EXPECT_EQ(status.waited_ms, 0U);
    // 再推 10 s：累计 10 s。
    status = tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 13000, params);
    EXPECT_EQ(status.waited_ms, 10000U);
    EXPECT_FALSE(status.warn);
}

TEST(HandoverWaitTrackerTest, VisualLockClearsWaitTimer) {
    HandoverWaitTracker tracker;
    const auto params = MakeParams();
    tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 1000, params);
    tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 3000, params);
    auto status = tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 25000, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kNoVisual);
    EXPECT_TRUE(status.warn);
    // 视觉在这一拍锁定：回到 arrived，等待归零，warn 撤掉。
    status = tracker.Update(MakeSample(1.0, 0.5, 0.1, true), 26000, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kArrived);
    EXPECT_EQ(status.waited_ms, 0U);
    EXPECT_FALSE(status.warn);
    // 再次丢失锁定：从当前拍重新起算，而不是沿用旧起点。
    status = tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 27000, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kNoVisual);
    EXPECT_EQ(status.waited_ms, 0U);
}

TEST(HandoverWaitTrackerTest, HysteresisBandKeepsArrivedAndWaitTimer) {
    HandoverWaitTracker tracker;
    const auto params = MakeParams();
    tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 1000, params);
    auto status = tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 3000, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kNoVisual);
    // 飘到 1.2R（仍在 1.5R 迟滞带内）：继续等待，不回到 enroute。
    status = tracker.Update(MakeSample(3.6, 0.5, 0.1, false), 8000, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kNoVisual);
    EXPECT_EQ(status.waited_ms, 5000U);
    // 越过 1.5R：放弃到点判定，等待清零。
    status = tracker.Update(MakeSample(4.6, 0.5, 0.1, false), 9000, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kEnroute);
    EXPECT_EQ(status.waited_ms, 0U);
    EXPECT_FALSE(tracker.arrived());
}

TEST(HandoverWaitTrackerTest, VerticalBeyondHysteresisDropsArrival) {
    HandoverWaitTracker tracker;
    const auto params = MakeParams();
    tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 1000, params);
    tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 3000, params);
    // 垂直 3.5 m > 2.0*1.5 = 3.0 m。
    const auto status = tracker.Update(MakeSample(1.0, 3.5, 0.1, false), 4000, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kEnroute);
    EXPECT_FALSE(tracker.arrived());
}

TEST(HandoverWaitTrackerTest, WarnAndAbortThresholdsAreIndependent) {
    auto params = MakeParams();
    params.visual_warn = std::chrono::milliseconds(5000);
    params.visual_abort = std::chrono::milliseconds(12000);
    HandoverWaitTracker tracker;
    tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 1000, params);
    tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 3000, params);
    auto status = tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 7000, params);
    EXPECT_EQ(status.waited_ms, 4000U);
    EXPECT_FALSE(status.warn);
    status = tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 8000, params);
    EXPECT_TRUE(status.warn);
    EXPECT_FALSE(status.abort);
    status = tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 15000, params);
    EXPECT_TRUE(status.warn);
    EXPECT_TRUE(status.abort);
}

TEST(HandoverWaitTrackerTest, ZeroAbortMeansNoAutomaticAction) {
    auto params = MakeParams();
    params.visual_warn = std::chrono::milliseconds(1000);
    HandoverWaitTracker tracker;
    tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 1000, params);
    tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 3000, params);
    const auto status =
        tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 600000, params);
    EXPECT_TRUE(status.warn);
    EXPECT_FALSE(status.abort);
}

TEST(HandoverWaitTrackerTest, ResetClearsTimersAndArrivalFlag) {
    HandoverWaitTracker tracker;
    const auto params = MakeParams();
    tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 1000, params);
    tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 3000, params);
    ASSERT_TRUE(tracker.arrived());
    tracker.Reset();
    EXPECT_FALSE(tracker.arrived());
    EXPECT_EQ(tracker.arrived_at_ms(), 0U);
    // Reset 后重新走 settling，而不是立刻判到点。
    const auto status = tracker.Update(MakeSample(1.0, 0.5, 0.1, false), 3100, params);
    EXPECT_EQ(status.phase, HandoverWaitPhase::kSettling);
    EXPECT_EQ(status.waited_ms, 0U);
}

}  // namespace
}  // namespace state_machine
}  // namespace drone
