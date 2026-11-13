// =============================================================================
// visual_control_law_test.cpp —— 视觉控制段接口与工厂
//
// 覆盖（2026-11-11）：
//   - 捕网/干扰共用 TrackingControlLaw，剖面各自由工厂标注；
//   - 撞击剖面预留：工厂 fail-fast（不得静默回退捕网律）；
//   - 剖面名用于日志/诊断。
// =============================================================================

#include <gtest/gtest.h>

#include <stdexcept>

#include "control/tracking_config.h"
#include "control/visual_control_law.h"

namespace {

using drone::control::CreateVisualControlLaw;
using drone::control::TrackingConfig;
using drone::control::VisualProfile;
using drone::control::VisualProfileName;

TEST(VisualControlLawTest, NetCaptureAndJammerShareImplementation) {
    TrackingConfig cfg;
    auto net = CreateVisualControlLaw(VisualProfile::kNetCapture, cfg);
    auto jammer = CreateVisualControlLaw(VisualProfile::kJammer, cfg);
    ASSERT_NE(net, nullptr);
    ASSERT_NE(jammer, nullptr);
    EXPECT_EQ(net->Profile(), VisualProfile::kNetCapture);
    EXPECT_EQ(jammer->Profile(), VisualProfile::kJammer);
}

TEST(VisualControlLawTest, ImpactProfileFailsFast) {
    TrackingConfig cfg;
    // 撞击剖面尚未实现：配置成撞击机必须在装配期失败，而不是套用捕网
    // “保持距离”律飞到目标前方悬停。
    EXPECT_THROW((void)CreateVisualControlLaw(VisualProfile::kImpact, cfg),
                 std::invalid_argument);
}

TEST(VisualControlLawTest, ProfileNames) {
    EXPECT_STREQ(VisualProfileName(VisualProfile::kNetCapture), "net_capture");
    EXPECT_STREQ(VisualProfileName(VisualProfile::kJammer), "jammer");
    EXPECT_STREQ(VisualProfileName(VisualProfile::kImpact), "impact");
}

}  // namespace
