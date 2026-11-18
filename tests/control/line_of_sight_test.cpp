// =============================================================================
// line_of_sight_test.cpp —— PixelToAngle 单元测试（内参式）
//
// 覆盖（物理追踪思路 §4.1 / §5.1）：
//   主点处零角、焦距一半偏移 = atan(0.5)、奇对称、单调性、
//   构造期非法参数抛异常、**主点偏中心的标定实测量值**（2026-09-23 标定：
//   cy=532.1 @720p，画面中心 360 处垂直角约 +9.7° 向下——画面中心不是零位）。
// =============================================================================

#include <cmath>

#include <gtest/gtest.h>

#include "control/line_of_sight.h"

namespace {

using drone::control::PixelToAngle;

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

TEST(PixelToAngleTest, PrincipalPointGivesZeroAngle) {
    PixelToAngle conv(1000.0, 640.0);
    EXPECT_DOUBLE_EQ(conv.Convert(640.0), 0.0);
}

TEST(PixelToAngleTest, HalfFocalOffsetGivesAtanHalf) {
    // 内参式：偏移 = f/2 时 angle = atan(0.5)
    PixelToAngle conv(1000.0, 640.0);
    EXPECT_NEAR(conv.Convert(640.0 + 500.0), std::atan(0.5), 1e-12);
    EXPECT_NEAR(conv.Convert(640.0 - 500.0), -std::atan(0.5), 1e-12);
}

TEST(PixelToAngleTest, OddSymmetryAroundPrincipalPoint) {
    PixelToAngle conv(1008.0, 532.1);
    for (const double offset : {1.0, 100.0, 300.5}) {
        EXPECT_NEAR(conv.Convert(532.1 - offset), -conv.Convert(532.1 + offset),
                    1e-12);
    }
}

TEST(PixelToAngleTest, MonotonicIncreasing) {
    PixelToAngle conv(1000.0, 360.0);
    double prev = conv.Convert(0.0);
    for (double x = 100.0; x <= 720.0; x += 100.0) {
        const double cur = conv.Convert(x);
        EXPECT_GT(cur, prev);
        prev = cur;
    }
}

TEST(PixelToAngleTest, CalibratedPrincipalPointShiftsZeroReference) {
    // 2026-09-23 标定实测：fy=1008.02、cy=532.07（画面中心 360 偏上 172px）。
    // 画面中心处的垂直角 = atan((360-532.07)/1008.02) ≈ -9.68°（向上），
    // 若仍按"画面中心=零位"假设会引入等量系统偏差——本用例守住主点语义。
    PixelToAngle conv(1008.02, 532.07);
    const double center_angle_deg = conv.Convert(360.0) * kRadToDeg;
    EXPECT_NEAR(center_angle_deg, -9.68, 0.01);
    EXPECT_DOUBLE_EQ(conv.Convert(532.07), 0.0);
}

TEST(PixelToAngleTest, ConstructorRejectsInvalidArgs) {
    EXPECT_THROW(PixelToAngle(0.0, 640.0), std::invalid_argument);
    EXPECT_THROW(PixelToAngle(-10.0, 640.0), std::invalid_argument);
    EXPECT_THROW(PixelToAngle(1000.0, -1.0), std::invalid_argument);
}

}  // namespace
