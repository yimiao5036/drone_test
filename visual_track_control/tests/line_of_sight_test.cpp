// =============================================================================
// line_of_sight_test.cpp —— PixelToAngle 单元测试
//
// 覆盖（物理追踪思路 §4.1 / §5.1，C2）：
//   零偏移、画面边缘偏移 = 半视场角、奇对称、单调性、
//   与线性近似的偏离（全程针孔模型）、构造期非法参数抛异常。
// =============================================================================

#include <cmath>

#include <gtest/gtest.h>

#include "visual_track_control/line_of_sight.h"

namespace {

using drone::vtc::PixelToAngle;

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

TEST(PixelToAngleTest, ZeroOffsetGivesZeroAngle) {
    PixelToAngle conv(1000.0, 90.0 * kPi / 180.0);
    EXPECT_DOUBLE_EQ(conv.Convert(0.0), 0.0);
}

TEST(PixelToAngleTest, HalfSizeOffsetGivesHalfFov) {
    // 针孔模型：偏移 = size/2 时 angle = atan(tan(θ/2)) = θ/2
    PixelToAngle conv(1000.0, 90.0 * kPi / 180.0);
    EXPECT_NEAR(conv.Convert(500.0), 45.0 * kPi / 180.0, 1e-12);
    EXPECT_NEAR(conv.Convert(-500.0), -45.0 * kPi / 180.0, 1e-12);
}

TEST(PixelToAngleTest, OddSymmetry) {
    PixelToAngle conv(1920.0, 60.0 * kPi / 180.0);
    for (const double offset : {1.0, 100.0, 730.5}) {
        EXPECT_NEAR(conv.Convert(-offset), -conv.Convert(offset), 1e-15);
    }
}

TEST(PixelToAngleTest, MonotonicIncreasing) {
    PixelToAngle conv(1080.0, 35.0 * kPi / 180.0);
    double prev = conv.Convert(-600.0);
    for (double x = -500.0; x <= 600.0; x += 100.0) {
        const double cur = conv.Convert(x);
        EXPECT_GT(cur, prev);
        prev = cur;
    }
}

TEST(PixelToAngleTest, DeviatesFromLinearApproximation) {
    // C2：全程针孔模型。θ=90°、偏移比例 0.25 处线性近似误差约 4°，
    // 针孔结果必须与之明显偏离（证明未走线性支路）。
    PixelToAngle conv(1000.0, 90.0 * kPi / 180.0);
    const double pinhole_deg = conv.Convert(250.0) * kRadToDeg;  // atan(250·2·tan45°/1000)
    const double linear_deg = 250.0 * (90.0 / 1000.0);           // θ·offset/size = 22.5°
    EXPECT_NEAR(pinhole_deg, std::atan(0.5) * kRadToDeg, 1e-9);  // 26.565°
    EXPECT_GT(std::fabs(pinhole_deg - linear_deg), 2.0);
}

TEST(PixelToAngleTest, ConstructorRejectsInvalidArgs) {
    EXPECT_THROW(PixelToAngle(0.0, 1.0), std::invalid_argument);
    EXPECT_THROW(PixelToAngle(-10.0, 1.0), std::invalid_argument);
    EXPECT_THROW(PixelToAngle(1000.0, 0.0), std::invalid_argument);
    EXPECT_THROW(PixelToAngle(1000.0, kPi), std::invalid_argument);
    EXPECT_THROW(PixelToAngle(1000.0, kPi + 0.1), std::invalid_argument);
}

}  // namespace
