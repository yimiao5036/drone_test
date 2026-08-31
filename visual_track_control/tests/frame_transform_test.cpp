// =============================================================================
// frame_transform_test.cpp —— RotateBodyToNed 单元测试
//
// 覆盖（物理追踪思路 §2.3，A4）：
//   零航向恒等、朝东时机体前向 → 东向、垂直分量不变、模长守恒。
// =============================================================================

#include <cmath>

#include <gtest/gtest.h>

#include "visual_track_control/frame_transform.h"

namespace {

using drone::vtc::RotateBodyToNed;

constexpr double kPi = 3.14159265358979323846;

TEST(FrameTransformTest, ZeroYawIsIdentity) {
    double x = 1.0, y = 0.5, z = -2.0;
    RotateBodyToNed(x, y, z, 0.0);
    EXPECT_NEAR(x, 1.0, 1e-12);   // 北 = 前
    EXPECT_NEAR(y, 0.5, 1e-12);   // 东 = 右
    EXPECT_NEAR(z, -2.0, 1e-12);
}

TEST(FrameTransformTest, YawEast90MapsForwardToEast) {
    double x = 2.0, y = 0.0, z = 0.0;
    RotateBodyToNed(x, y, z, kPi / 2.0);  // 机头朝东
    EXPECT_NEAR(x, 0.0, 1e-9);  // 北向 ≈ 0
    EXPECT_NEAR(y, 2.0, 1e-9);  // 东向 = 前向速度
}

TEST(FrameTransformTest, RightIsEastWhenHeadingNorth) {
    double x = 0.0, y = 1.0, z = 0.0;
    RotateBodyToNed(x, y, z, 0.0);
    EXPECT_NEAR(x, 0.0, 1e-12);
    EXPECT_NEAR(y, 1.0, 1e-12);
}

TEST(FrameTransformTest, DownComponentUnchanged) {
    double x = 1.0, y = 1.0, z = 3.5;
    RotateBodyToNed(x, y, z, kPi / 3.0);
    EXPECT_DOUBLE_EQ(z, 3.5);
}

TEST(FrameTransformTest, PreservesHorizontalNorm) {
    // 旋转为正交变换：水平合速度不变（限幅约束不被破坏的前提）
    double x = 3.0, y = -4.0, z = 1.0;
    const double norm_before = std::hypot(x, y);
    RotateBodyToNed(x, y, z, 0.7);
    EXPECT_NEAR(std::hypot(x, y), norm_before, 1e-9);
}

}  // namespace
