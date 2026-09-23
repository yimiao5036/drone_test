// =============================================================================
// visual_tracking_shadow_test.cpp —— VisualTrackingShadow 影子适配层单元测试
//
// 覆盖：未接线启动失败、生命周期幂等、虚拟姿态开/关、仅 kLocked 输出正常
// 控制、kLost → kBrakeHover 映射、视觉超龄 → kNone+降级码、
// 速率式 yaw_rate_dps 进 ControlIntent。
// =============================================================================

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "control/visual_tracking_shadow.h"

namespace {

using drone::common::ControlIntent;
using drone::common::ControlIntentType;
using drone::common::FlightStateSnapshot;
using drone::common::Topic;
using drone::common::VisualTargetState;
using drone::common::VisualTargetStatus;
using drone::control::TrackingConfig;
using drone::control::VisualTrackingShadow;
using drone::control::VisualTrackingShadowConfig;

std::int64_t NowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

/// 测试配置：50Hz 节拍缩短测试时长；内参 fx=fy=500、主点居中便于口算。
TrackingConfig MakeTrackingConfig() {
    TrackingConfig cfg;
    cfg.control.frequency_hz = 50.0;
    cfg.camera.fx_px = 500.0;
    cfg.camera.fy_px = 500.0;
    cfg.camera.cx_px = 640.0;
    cfg.camera.cy_px = 360.0;
    cfg.heading.yaw_slew_limit_dps2 = 100000.0;  // 限幅透明化
    cfg.accel_limit.ax_mps2 = 1000.0;
    cfg.accel_limit.az_mps2 = 1000.0;
    return cfg;
}

VisualTrackingShadowConfig MakeShadowConfig() {
    VisualTrackingShadowConfig cfg;
    cfg.summary_log_interval = std::chrono::milliseconds(60000);  // 测试不打摘要
    return cfg;
}

void PublishStatus(Topic<VisualTargetStatus>& topic, VisualTargetState state,
                   float cx = 640.f, float cy = 360.f) {
    auto status = std::make_shared<VisualTargetStatus>();
    status->state = state;
    status->center_pixel_x = cx;
    status->center_pixel_y = cy;
    status->header.receive_time_ms = static_cast<std::uint64_t>(NowMs());
    (void)topic.Publish(std::move(status));
}

/// 轮询等待条件成立（最长 2s）。
template <typename F>
bool WaitFor(F&& pred) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

class VisualTrackingShadowTest : public ::testing::Test {
protected:
    void TearDown() override {
        if (shadow_ != nullptr) {
            shadow_->Stop();
        }
    }

    void BuildShadow(VisualTrackingShadowConfig shadow_cfg = MakeShadowConfig()) {
        shadow_ = std::make_unique<VisualTrackingShadow>(MakeTrackingConfig(),
                                                         shadow_cfg);
        shadow_->SetVisualInput(status_topic_);
        auto sub = shadow_->IntentOutput().Subscribe(64);
        intent_sub_ = std::move(sub);
    }

    std::optional<ControlIntent> TakeLatestIntent() {
        std::optional<ControlIntent> latest;
        while (auto msg = intent_sub_.TryTake()) {
            latest = **msg;
        }
        return latest;
    }

    Topic<VisualTargetStatus> status_topic_;
    Topic<FlightStateSnapshot> flight_topic_;
    std::unique_ptr<VisualTrackingShadow> shadow_;
    Topic<ControlIntent>::Subscription intent_sub_;
};

TEST_F(VisualTrackingShadowTest, StartFailsWithoutVisualInput) {
    VisualTrackingShadow shadow(MakeTrackingConfig(), MakeShadowConfig());
    EXPECT_FALSE(shadow.Start());  // 视觉输入未接线
}

TEST_F(VisualTrackingShadowTest, LifecycleIdempotent) {
    BuildShadow();
    EXPECT_TRUE(shadow_->Start());
    EXPECT_TRUE(shadow_->Start());  // 幂等
    EXPECT_TRUE(shadow_->IsRunning());
    shadow_->Stop();
    EXPECT_FALSE(shadow_->IsRunning());
    shadow_->Stop();  // 幂等
}

TEST_F(VisualTrackingShadowTest, LockedStateProducesVelocityHeadingIntent) {
    BuildShadow();
    ASSERT_TRUE(shadow_->Start());
    // 谓词内反复发布新鲜状态（单条 200ms 后超龄，轮询期间必须保持新鲜）；
    // 满足条件的意图由谓词顺手捕获（TakeLatestIntent 会排空队列，
    // 不能在 WaitFor 之后再取）
    std::optional<ControlIntent> found;
    ASSERT_TRUE(WaitFor([this, &found] {
        PublishStatus(status_topic_, VisualTargetState::kLocked, 740.f, 360.f);
        auto intent = TakeLatestIntent();
        if (intent.has_value() &&
            intent->type == ControlIntentType::kVelocityHeading) {
            found = intent;
            return true;
        }
        return false;
    }));
    ASSERT_TRUE(found.has_value());
    // 目标偏右 100px：β=atan(100/500)=11.31°，速率式 gain=1 → 11.31 dps
    EXPECT_NEAR(found->yaw_rate_dps,
                std::atan(0.2) * 180.0 / 3.14159265358979, 0.5);
    EXPECT_EQ(found->reason_code, 100u);       // 正常跟踪
    EXPECT_EQ(found->control_source, 1u);      // 视觉
    EXPECT_GT(found->header.valid_for_ms, 0u);
    // 无距离（distance_valid=false）→ kSlowApproach 支路 vx=0.5
    EXPECT_NEAR(found->target_x, 0.5f, 0.05f);
}

TEST_F(VisualTrackingShadowTest, LostStateMapsToBrakeHover) {
    BuildShadow();
    ASSERT_TRUE(shadow_->Start());
    std::optional<ControlIntent> found;
    ASSERT_TRUE(WaitFor([this, &found] {
        PublishStatus(status_topic_, VisualTargetState::kLost);
        auto intent = TakeLatestIntent();
        if (intent.has_value() &&
            intent->type == ControlIntentType::kBrakeHover) {
            found = intent;
            return true;
        }
        return false;
    }));
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(found->reason_code, 102u);  // 判丢刹车
    EXPECT_DOUBLE_EQ(found->target_x, 0.f);
}

TEST_F(VisualTrackingShadowTest, VirtualAttitudeEnabledByDefault) {
    BuildShadow();  // 不接姿态输入 + 默认虚拟姿态
    ASSERT_TRUE(shadow_->Start());
    // 虚拟姿态兜底：能出正常控制意图而不是姿态降级
    ASSERT_TRUE(WaitFor([this] {
        PublishStatus(status_topic_, VisualTargetState::kLocked);
        auto intent = TakeLatestIntent();
        return intent.has_value() &&
               intent->type == ControlIntentType::kVelocityHeading;
    }));
    EXPECT_TRUE(shadow_->LastUsedVirtualAttitude());
}

TEST_F(VisualTrackingShadowTest, RealSemanticsWhenVirtualAttitudeDisabled) {
    auto shadow_cfg = MakeShadowConfig();
    shadow_cfg.virtual_attitude_when_absent = false;
    BuildShadow(shadow_cfg);
    ASSERT_TRUE(shadow_->Start());
    // 姿态缺失 → 每拍降级保持（kNone + 姿态超龄码）
    std::optional<ControlIntent> found;
    ASSERT_TRUE(WaitFor([this, &found] {
        PublishStatus(status_topic_, VisualTargetState::kLocked);
        auto intent = TakeLatestIntent();
        if (intent.has_value() && intent->type == ControlIntentType::kNone) {
            found = intent;
            return true;
        }
        return false;
    }));
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(found->reason_code, 103u);  // 姿态超龄降级
}

TEST_F(VisualTrackingShadowTest, VisionStaleDegradesToNone) {
    BuildShadow();
    ASSERT_TRUE(shadow_->Start());
    // 发布一条旧时间戳的状态（visual_stale_ms=200）
    auto status = std::make_shared<VisualTargetStatus>();
    status->state = VisualTargetState::kLocked;
    status->center_pixel_x = 640.f;
    status->center_pixel_y = 360.f;
    status->header.receive_time_ms = static_cast<std::uint64_t>(NowMs() - 5000);
    (void)status_topic_.Publish(std::move(status));
    ASSERT_TRUE(WaitFor([this] {
        auto intent = TakeLatestIntent();
        return intent.has_value() &&
               intent->type == ControlIntentType::kNone &&
               intent->reason_code == 104u;  // 视觉超龄降级
    }));
}

TEST_F(VisualTrackingShadowTest, AcquiringIsNotTracked) {
    BuildShadow();
    ASSERT_TRUE(shadow_->Start());
    // kAcquiring（未锁定）不视为 tracked → 判丢刹车
    ASSERT_TRUE(WaitFor([this] {
        PublishStatus(status_topic_, VisualTargetState::kAcquiring);
        auto intent = TakeLatestIntent();
        return intent.has_value() &&
               intent->type == ControlIntentType::kBrakeHover;
    }));
}

}  // namespace
