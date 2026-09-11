/**
 * @file visual_target_monitor_test.cpp
 * @brief 单目标视觉稳定性判定（VisualTargetMonitor）单元测试
 *
 * 不依赖硬件，直接注入帧级观测验证状态机与线程行为：
 * - 配置校验：非法帧数、周期与平滑系数拒绝构造
 * - 生命周期：未绑定输入时拒绝启动、Start/Stop 幂等、重启复位状态
 * - 锁定：连续检测达到 lock_frames 进入 LOCKED，之前为 ACQUIRING
 * - 丢失：锁定后短时漏检保持 LOCKED，达到 lost_frames 进入 LOST
 * - 搜索：从未检测时保持 SEARCHING
 * - 观测超时：有观测后流中断判定 LOST（区别于“本帧无目标”）
 * - 多目标候选：按置信度最高者处理并计入错误计数
 */
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

#include "common/topic.h"
#include "common/types.h"
#include "perception/visual_target_monitor.h"

namespace drone::perception {
namespace {

/// 轮询等待条件满足（带超时）。
bool WaitFor(const std::function<bool()>& condition, int timeout_ms = 3000) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return condition();
}

common::VisualTargetObservation MakeObservation(uint64_t sequence, bool detected,
                                                float center_x = 100.f,
                                                float center_y = 60.f,
                                                float confidence = 0.9f,
                                                uint32_t candidates = 1) {
    common::VisualTargetObservation observation;
    observation.header.sequence = sequence;
    observation.header.source_time_ms = 1;
    observation.frame_sequence = sequence;
    observation.detected = detected;
    observation.candidate_count = candidates;
    observation.frame_width = 1280;
    observation.frame_height = 720;
    observation.confidence = confidence;
    observation.center_pixel_x = center_x;
    observation.center_pixel_y = center_y;
    observation.bbox_x = center_x - 10.f;
    observation.bbox_y = center_y - 10.f;
    observation.bbox_w = 20.f;
    observation.bbox_h = 20.f;
    return observation;
}

/// 测试用配置：门限小、发布快、不平滑，便于确定性断言。
VisualTargetMonitorConfig MakeConfig() {
    VisualTargetMonitorConfig config;
    config.input_queue_capacity = 32;
    config.publish_interval = std::chrono::milliseconds(10);
    config.observation_timeout = std::chrono::seconds(5);
    config.lock_frames = 3;
    config.lost_frames = 3;
    config.smoothing_alpha = 1.0;
    return config;
}

class VisualTargetMonitorTest : public testing::Test {
protected:
    void SetUp() override {
        monitor_ = std::make_unique<VisualTargetMonitor>(MakeConfig());
        monitor_->SetInput(observation_topic_);
    }

    void TearDown() override {
        if (monitor_ != nullptr) {
            monitor_->Stop();
        }
    }

    void Publish(const common::VisualTargetObservation& observation) {
        (void)observation_topic_.Publish(
            std::make_shared<const common::VisualTargetObservation>(observation));
    }

    std::unique_ptr<VisualTargetMonitor> monitor_;
    common::Topic<common::VisualTargetObservation> observation_topic_;
};

TEST(VisualTargetMonitorConfigTest, RejectsInvalidValues) {
    VisualTargetMonitorConfig zero_lock;
    zero_lock.lock_frames = 0;
    EXPECT_THROW(VisualTargetMonitor monitor(zero_lock), std::invalid_argument);

    VisualTargetMonitorConfig zero_lost;
    zero_lost.lost_frames = 0;
    EXPECT_THROW(VisualTargetMonitor monitor(zero_lost), std::invalid_argument);

    VisualTargetMonitorConfig zero_alpha;
    zero_alpha.smoothing_alpha = 0.0;
    EXPECT_THROW(VisualTargetMonitor monitor(zero_alpha), std::invalid_argument);

    VisualTargetMonitorConfig big_alpha;
    big_alpha.smoothing_alpha = 1.5;
    EXPECT_THROW(VisualTargetMonitor monitor(big_alpha), std::invalid_argument);

    VisualTargetMonitorConfig zero_queue;
    zero_queue.input_queue_capacity = 0;
    EXPECT_THROW(VisualTargetMonitor monitor(zero_queue), std::invalid_argument);
}

TEST(VisualTargetMonitorLifecycleTest, StartWithoutInputFails) {
    VisualTargetMonitor monitor(MakeConfig());
    EXPECT_FALSE(monitor.Start());
    EXPECT_FALSE(monitor.IsRunning());
    EXPECT_EQ(monitor.ErrorCount(), 1u);
}

TEST_F(VisualTargetMonitorTest, StartStopIsIdempotent) {
    ASSERT_TRUE(monitor_->Start());
    EXPECT_TRUE(monitor_->IsRunning());
    EXPECT_TRUE(monitor_->Start());  // 幂等
    monitor_->Stop();
    EXPECT_FALSE(monitor_->IsRunning());
    monitor_->Stop();  // 幂等
}

TEST_F(VisualTargetMonitorTest, NeverDetectedStaysSearching) {
    ASSERT_TRUE(monitor_->Start());
    EXPECT_TRUE(WaitFor([this] {
        return monitor_->LastStatus().header.sequence > 0;
    }));
    EXPECT_EQ(monitor_->LastStatus().state, common::VisualTargetState::kSearching);
    EXPECT_FALSE(monitor_->LastStatus().valid);
    EXPECT_EQ(monitor_->ObservationCount(), 0u);
}

TEST_F(VisualTargetMonitorTest, ConsecutiveDetectionsLockAfterThreshold) {
    ASSERT_TRUE(monitor_->Start());

    Publish(MakeObservation(1, true));
    Publish(MakeObservation(2, true));
    ASSERT_TRUE(WaitFor([this] { return monitor_->ObservationCount() == 2; }));
    EXPECT_TRUE(WaitFor([this] {
        return monitor_->LastStatus().state == common::VisualTargetState::kAcquiring;
    }));

    Publish(MakeObservation(3, true));
    ASSERT_TRUE(WaitFor([this] { return monitor_->ObservationCount() == 3; }));
    EXPECT_TRUE(WaitFor([this] {
        return monitor_->LastStatus().state == common::VisualTargetState::kLocked;
    }));
    EXPECT_TRUE(monitor_->LastStatus().valid);
    EXPECT_EQ(monitor_->LastStatus().consecutive_detected_frames, 3);
    EXPECT_EQ(monitor_->LastStatus().frame_sequence, 3u);
    EXPECT_EQ(monitor_->LastStatus().frame_width, 1280u);
    EXPECT_EQ(monitor_->LastStatus().frame_height, 720u);
}

TEST_F(VisualTargetMonitorTest, LockedToleratesBriefMissesThenGoesLost) {
    ASSERT_TRUE(monitor_->Start());
    for (uint64_t i = 1; i <= 3; ++i) {
        Publish(MakeObservation(i, true));
    }
    ASSERT_TRUE(WaitFor([this] { return monitor_->ObservationCount() == 3; }));
    ASSERT_TRUE(WaitFor([this] {
        return monitor_->LastStatus().state == common::VisualTargetState::kLocked;
    }));

    // 短时漏检（未达丢失门限）应继续保持锁定，容忍YOLO偶发漏检。
    Publish(MakeObservation(4, false));
    Publish(MakeObservation(5, false));
    ASSERT_TRUE(WaitFor([this] { return monitor_->ObservationCount() == 5; }));
    EXPECT_TRUE(WaitFor([this] {
        return monitor_->LastStatus().consecutive_missed_frames == 2;
    }));
    EXPECT_EQ(monitor_->LastStatus().state, common::VisualTargetState::kLocked);

    // 达到丢失门限 → LOST。
    Publish(MakeObservation(6, false));
    ASSERT_TRUE(WaitFor([this] { return monitor_->ObservationCount() == 6; }));
    EXPECT_TRUE(WaitFor([this] {
        return monitor_->LastStatus().state == common::VisualTargetState::kLost;
    }));
    EXPECT_FALSE(monitor_->LastStatus().valid);
    EXPECT_EQ(monitor_->LastStatus().consecutive_detected_frames, 0);
}

TEST_F(VisualTargetMonitorTest, MissFromUnlockedGoesBackToSearching) {
    ASSERT_TRUE(monitor_->Start());

    Publish(MakeObservation(1, true));
    ASSERT_TRUE(WaitFor([this] { return monitor_->ObservationCount() == 1; }));
    ASSERT_TRUE(WaitFor([this] {
        return monitor_->LastStatus().state == common::VisualTargetState::kAcquiring;
    }));

    // 未锁定时的漏检立即退回 SEARCHING（仍在丢失门限内）。
    Publish(MakeObservation(2, false));
    ASSERT_TRUE(WaitFor([this] { return monitor_->ObservationCount() == 2; }));
    EXPECT_TRUE(WaitFor([this] {
        return monitor_->LastStatus().state == common::VisualTargetState::kSearching;
    }));
}

TEST(VisualTargetMonitorLifecycleTest, ObservationTimeoutMarksLost) {
    VisualTargetMonitorConfig config = MakeConfig();
    config.observation_timeout = std::chrono::milliseconds(100);
    VisualTargetMonitor monitor(config);
    common::Topic<common::VisualTargetObservation> topic;
    monitor.SetInput(topic);
    ASSERT_TRUE(monitor.Start());

    (void)topic.Publish(
        std::make_shared<const common::VisualTargetObservation>(MakeObservation(1, true)));
    ASSERT_TRUE(WaitFor([&] { return monitor.ObservationCount() == 1; }));

    // 观测流中断超过超时门限 → LOST（区别于“本帧无目标”）。
    EXPECT_TRUE(WaitFor([&] {
        return monitor.LastStatus().state == common::VisualTargetState::kLost;
    }, 2000));
    EXPECT_GT(monitor.ErrorCount(), 0u);
    monitor.Stop();
}

TEST_F(VisualTargetMonitorTest, MultipleCandidatesUseHighestConfidenceAndCountError) {
    ASSERT_TRUE(monitor_->Start());

    // 观测层已按单目标约定给出唯一目标；多候选只作为诊断计数。
    Publish(MakeObservation(1, true, 100.f, 60.f, 0.8f, 3));
    ASSERT_TRUE(WaitFor([this] { return monitor_->ObservationCount() == 1; }));
    EXPECT_TRUE(WaitFor([this] { return monitor_->ErrorCount() == 1u; }));
    EXPECT_TRUE(WaitFor([this] {
        return monitor_->LastStatus().consecutive_detected_frames == 1;
    }));
    EXPECT_EQ(monitor_->LastStatus().state, common::VisualTargetState::kAcquiring);
}

TEST_F(VisualTargetMonitorTest, CountsOnlyRealStateTransitions) {
    // 计数只反映真实状态变化，而不是逐帧推进，便于通过累计转换数发现检测抖动。
    ASSERT_TRUE(monitor_->Start());

    Publish(MakeObservation(1, true));   // ACQUIRING
    Publish(MakeObservation(2, true));   // 仍为 ACQUIRING
    Publish(MakeObservation(3, true));   // LOCKED
    Publish(MakeObservation(4, false));  // 锁定容忍，仍为 LOCKED
    Publish(MakeObservation(5, false));  // 仍为 LOCKED
    Publish(MakeObservation(6, false));  // LOST

    ASSERT_TRUE(WaitFor([this] { return monitor_->ObservationCount() == 6; }));
    ASSERT_TRUE(WaitFor([this] {
        return monitor_->LastStatus().state == common::VisualTargetState::kLost;
    }));
    EXPECT_EQ(monitor_->StateTransitionCount(), 3u);
}

TEST_F(VisualTargetMonitorTest, RestartResetsStabilityState) {
    ASSERT_TRUE(monitor_->Start());
    for (uint64_t i = 1; i <= 3; ++i) {
        Publish(MakeObservation(i, true));
    }
    ASSERT_TRUE(WaitFor([this] { return monitor_->ObservationCount() == 3; }));
    ASSERT_TRUE(WaitFor([this] {
        return monitor_->LastStatus().state == common::VisualTargetState::kLocked;
    }));

    monitor_->Stop();
    ASSERT_TRUE(monitor_->Start());

    EXPECT_TRUE(WaitFor([this] {
        return monitor_->LastStatus().state == common::VisualTargetState::kSearching;
    }));
    EXPECT_EQ(monitor_->LastStatus().consecutive_detected_frames, 0);
    EXPECT_EQ(monitor_->LastStatus().consecutive_missed_frames, 0);
    EXPECT_FALSE(monitor_->LastStatus().valid);
}

}  // namespace
}  // namespace drone::perception
