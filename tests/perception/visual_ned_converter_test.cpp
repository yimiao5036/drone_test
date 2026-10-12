// =============================================================================
// visual_ned_converter_test.cpp —— 视觉观测 → 局部 NED 转换纯函数单测
//
// 覆盖：固定轴置换、姿态 RPY 各轴旋转方向、mount 旋转/平移、自身位置叠加、
// 非有限输入拒绝、与 control 库 RotateBodyToNed 的约定交叉验证。
// =============================================================================

#include "perception/visual_ned_converter.h"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "control/frame_transform.h"

namespace drone::perception {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;
constexpr double kEps = 1e-9;

const VisualMountExtrinsics kZeroMount{};

LocalNedCoordinate Self(double n, double e, double d) {
    LocalNedCoordinate p;
    p.north_m = n;
    p.east_m = e;
    p.down_m = d;
    return p;
}

// 零姿态 + 准直安装：相机系（右/下/前）→ 机体 FRD → NED 的轴置换
TEST(VisualNedConverterTest, ZeroAttitudeIsAxisPermutation) {
    const auto out =
        VisualCameraToLocalNed(1.0, -2.0, 10.0, kZeroMount, 0.0, 0.0, 0.0,
                               Self(0.0, 0.0, 0.0));
    ASSERT_TRUE(out.has_value());
    EXPECT_NEAR(out->north_m, 10.0, kEps);  // cam_z → north（前）
    EXPECT_NEAR(out->east_m, 1.0, kEps);    // cam_x → east（右）
    EXPECT_NEAR(out->down_m, -2.0, kEps);   // cam_y → down（下）
}

// yaw=90°（机头朝东）：相机前方 → +East
TEST(VisualNedConverterTest, Yaw90TurnsForwardToEast) {
    const auto out =
        VisualCameraToLocalNed(0.0, 0.0, 5.0, kZeroMount, 0.0, 0.0, kHalfPi,
                               Self(0.0, 0.0, 0.0));
    ASSERT_TRUE(out.has_value());
    EXPECT_NEAR(out->north_m, 0.0, 1e-9);
    EXPECT_NEAR(out->east_m, 5.0, 1e-9);
    EXPECT_NEAR(out->down_m, 0.0, 1e-9);
}

// pitch=+90°（仰头，航空约定俯仰正=抬头）：相机前方 → −Down（向上）
TEST(VisualNedConverterTest, PitchUp90TurnsForwardToUp) {
    const auto out =
        VisualCameraToLocalNed(0.0, 0.0, 5.0, kZeroMount, 0.0, kHalfPi, 0.0,
                               Self(0.0, 0.0, 0.0));
    ASSERT_TRUE(out.has_value());
    EXPECT_NEAR(out->north_m, 0.0, 1e-9);
    EXPECT_NEAR(out->east_m, 0.0, 1e-9);
    EXPECT_NEAR(out->down_m, -5.0, 1e-9);
}

// roll=+90°（右机翼下沉）：相机右向（cam_x）→ +Down
TEST(VisualNedConverterTest, Roll90TurnsCameraRightToDown) {
    const auto out =
        VisualCameraToLocalNed(5.0, 0.0, 0.0, kZeroMount, kHalfPi, 0.0, 0.0,
                               Self(0.0, 0.0, 0.0));
    ASSERT_TRUE(out.has_value());
    EXPECT_NEAR(out->north_m, 0.0, 1e-9);
    EXPECT_NEAR(out->east_m, 0.0, 1e-9);
    EXPECT_NEAR(out->down_m, 5.0, 1e-9);
}

// mount_yaw=45°：安装偏航在机体系内旋转观测方向
TEST(VisualNedConverterTest, MountYawRotatesObservationInBodyFrame) {
    VisualMountExtrinsics mount;
    mount.yaw_deg = 45.0;
    const auto out =
        VisualCameraToLocalNed(0.0, 0.0, 5.0, mount, 0.0, 0.0, 0.0,
                               Self(0.0, 0.0, 0.0));
    ASSERT_TRUE(out.has_value());
    EXPECT_NEAR(out->north_m, 5.0 * std::cos(kPi / 4.0), 1e-9);
    EXPECT_NEAR(out->east_m, 5.0 * std::sin(kPi / 4.0), 1e-9);
    EXPECT_NEAR(out->down_m, 0.0, 1e-9);
}

// mount 平移（杠杆臂）：旋转后叠加机体系平移
TEST(VisualNedConverterTest, MountTranslationAddsLeverArm) {
    VisualMountExtrinsics mount;
    mount.x_m = 2.0;  // 相机在机体原点前方 2m
    mount.z_m = 0.5;  // 下方 0.5m
    const auto out =
        VisualCameraToLocalNed(0.0, 0.0, 5.0, mount, 0.0, 0.0, 0.0,
                               Self(0.0, 0.0, 0.0));
    ASSERT_TRUE(out.has_value());
    EXPECT_NEAR(out->north_m, 7.0, kEps);
    EXPECT_NEAR(out->east_m, 0.0, kEps);
    EXPECT_NEAR(out->down_m, 0.5, kEps);
}

// 自身局部位置叠加
TEST(VisualNedConverterTest, SelfPositionIsAdded) {
    const auto out =
        VisualCameraToLocalNed(1.0, -2.0, 10.0, kZeroMount, 0.0, 0.0, 0.0,
                               Self(100.0, 50.0, -30.0));
    ASSERT_TRUE(out.has_value());
    EXPECT_NEAR(out->north_m, 110.0, kEps);
    EXPECT_NEAR(out->east_m, 51.0, kEps);
    EXPECT_NEAR(out->down_m, -32.0, kEps);
}

// 非有限输入 → nullopt
TEST(VisualNedConverterTest, NonFiniteInputRejected) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(VisualCameraToLocalNed(nan, 0.0, 1.0, kZeroMount, 0.0, 0.0,
                                        0.0, Self(0.0, 0.0, 0.0))
                     .has_value());
    EXPECT_FALSE(VisualCameraToLocalNed(0.0, 0.0, 1.0, kZeroMount, 0.0, inf,
                                        0.0, Self(0.0, 0.0, 0.0))
                     .has_value());
    EXPECT_FALSE(VisualCameraToLocalNed(0.0, 0.0, 1.0, kZeroMount, 0.0, 0.0,
                                        0.0, Self(nan, 0.0, 0.0))
                     .has_value());
    VisualMountExtrinsics bad_mount;
    bad_mount.yaw_deg = nan;
    EXPECT_FALSE(VisualCameraToLocalNed(0.0, 0.0, 1.0, bad_mount, 0.0, 0.0,
                                        0.0, Self(0.0, 0.0, 0.0))
                     .has_value());
}

// roll=pitch=0 时与 control 库 RotateBodyToNed（yaw-only）结果一致：
// 交叉验证两套旋转约定相同
TEST(VisualNedConverterTest, MatchesRotateBodyToNedAtZeroRollPitch) {
    const double yaw = 0.7;
    // cam(1,-2,10) → body(10,1,-2)
    double f = 10.0, r = 1.0, d = -2.0;
    control::RotateBodyToNed(f, r, d, yaw);
    const auto out =
        VisualCameraToLocalNed(1.0, -2.0, 10.0, kZeroMount, 0.0, 0.0, yaw,
                               Self(0.0, 0.0, 0.0));
    ASSERT_TRUE(out.has_value());
    EXPECT_NEAR(out->north_m, f, 1e-12);
    EXPECT_NEAR(out->east_m, r, 1e-12);
    EXPECT_NEAR(out->down_m, d, 1e-12);
}

}  // namespace
}  // namespace drone::perception
