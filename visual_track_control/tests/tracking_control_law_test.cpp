// =============================================================================
// tracking_control_law_test.cpp —— TrackingControlLaw 主控律单元测试
//
// 覆盖（物理追踪思路 §2.3 / §4 / §5 / §6 / §7 / §9）：
//   超龄降级（姿态缺失/过期、视觉超龄、降级优先级）、判丢零速刹车、
//   Coast 清积分与标志、速率式/位置式航向通道（含绝对航向合成与回绕，
//   A2）、垂直统一速度支路（A1）、距离通道三种无距离降级动作、
//   首拍变化率限幅（S6）、机体系→NED 输出旋转（A4）。
// =============================================================================

#include <cmath>

#include <gtest/gtest.h>

#include "visual_track_control/tracking_control_law.h"
#include "visual_track_control/vtc_config.h"
#include "visual_track_control/vtc_types.h"

namespace {

using namespace drone::vtc;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

/// 测试基准配置：大限幅使限幅器近似透明，便于独立验证各通道数学。
/// 画面 1000×1000、视场角 90° → 针孔系数 2·tan(45°)/1000 = 0.002，
/// 偏移 100 像素 → 视线角 atan(0.2) ≈ 11.3099°。
TrackingConfig MakeConfig() {
    TrackingConfig cfg;
    cfg.control.frequency_hz = 20.0;
    cfg.control.visual_stale_ms = 200;
    cfg.control.radar_stale_ms = 500;
    cfg.control.attitude_stale_ms = 300;
    cfg.camera.image_width = 1000;
    cfg.camera.image_height = 1000;
    cfg.camera.fov_h_deg = 90.0;
    cfg.camera.fov_v_deg = 90.0;
    cfg.heading.mode = HeadingMode::kRate;
    cfg.heading.gain = 1.0;
    cfg.heading.yaw_rate_limit_dps = 30.0;
    cfg.heading.yaw_slew_limit_dps2 = 100000.0;
    cfg.vertical.vz_gain_mps_per_deg = 0.1;
    cfg.vertical.vz_limit_mps = 2.0;
    cfg.distance.d_exp_m = 10.0;
    cfg.distance.kp = 0.5;
    cfg.distance.ki = 0.0;
    cfg.distance.kd = 0.0;
    cfg.distance.integral_limit = 0.0;
    cfg.distance.derivative_filter_coef = 0.0;
    cfg.distance.approach_velocity_limit_mps = 3.0;
    cfg.distance.retreat_velocity_limit_mps = 1.0;
    cfg.distance.no_distance_action = NoDistanceAction::kHold;
    cfg.distance.no_distance_approach_limit_mps = 0.5;
    cfg.accel_limit.ax_mps2 = 1000.0;
    cfg.accel_limit.ay_mps2 = 1000.0;
    cfg.accel_limit.az_mps2 = 1000.0;
    return cfg;
}

/// 新鲜快照：目标在画面中心，姿态/追踪/雷达均新鲜
ControlSnapshot MakeSnapshot(std::int64_t now_ms) {
    ControlSnapshot s;
    s.track.tracked = true;
    s.track.is_predicted = false;
    s.track.center = {500.0, 500.0};
    s.track_time_ms = now_ms;
    s.attitude_present = true;
    s.attitude_time_ms = now_ms;
    return s;
}

// ---- 超龄降级（§7.1 / §9.2） ----

TEST(TrackingControlLawTest, AttitudeMissingDegrades) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.attitude_present = false;
    const auto out = law.Update(snap, 1000);
    EXPECT_EQ(out.mode, ControlMode::kDegradedHold);
    EXPECT_EQ(out.degraded_reason, DegradedReason::kAttitudeStale);
    EXPECT_FALSE(out.valid);
}

TEST(TrackingControlLawTest, AttitudeStaleDegrades) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.attitude_time_ms = 600;  // 400 ms 前 > 300 ms 阈值
    const auto out = law.Update(snap, 1000);
    EXPECT_EQ(out.mode, ControlMode::kDegradedHold);
    EXPECT_EQ(out.degraded_reason, DegradedReason::kAttitudeStale);
}

TEST(TrackingControlLawTest, VisionStaleDegrades) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.track_time_ms = 700;  // 300 ms 前 > 200 ms 阈值
    const auto out = law.Update(snap, 1000);
    EXPECT_EQ(out.mode, ControlMode::kDegradedHold);
    EXPECT_EQ(out.degraded_reason, DegradedReason::kVisionStale);
}

TEST(TrackingControlLawTest, NeverSampledTrackDegrades) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.track_time_ms = 0;  // 从未取样
    const auto out = law.Update(snap, 1000);
    EXPECT_EQ(out.mode, ControlMode::kDegradedHold);
    EXPECT_EQ(out.degraded_reason, DegradedReason::kVisionStale);
}

TEST(TrackingControlLawTest, AttitudeStaleBeatsVisionStale) {
    // 降级优先级：姿态校验在前（§9.2 姿态过期禁止一切空间转换）
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.attitude_present = false;
    snap.track_time_ms = 0;
    const auto out = law.Update(snap, 1000);
    EXPECT_EQ(out.degraded_reason, DegradedReason::kAttitudeStale);
}

// ---- 判丢刹车（§6.4） ----

TEST(TrackingControlLawTest, LostTargetBrakes) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.track.tracked = false;
    const auto out = law.Update(snap, 1000);
    EXPECT_EQ(out.mode, ControlMode::kBrakeHover);
    EXPECT_TRUE(out.valid);
    EXPECT_DOUBLE_EQ(out.vx_mps, 0.0);
    EXPECT_DOUBLE_EQ(out.vy_mps, 0.0);
    EXPECT_DOUBLE_EQ(out.vz_mps, 0.0);
}

// ---- Coast（§6.4） ----

TEST(TrackingControlLawTest, CoastSetsFlagAndKeepsTracking) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.track.is_predicted = true;
    const auto out = law.Update(snap, 1000);
    EXPECT_EQ(out.mode, ControlMode::kVelocityHeading);
    EXPECT_TRUE(out.coast_active);
    EXPECT_TRUE(out.valid);
}

// ---- 水平航向通道（§4.2，A2） ----

TEST(TrackingControlLawTest, RateModeMapsBetaToYawRate) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.track.center = {600.0, 500.0};  // 目标偏右 100 像素
    const auto out = law.Update(snap, 1000);
    const double beta_deg = std::atan(0.2) * kRadToDeg;  // ≈ 11.31°
    EXPECT_NEAR(out.yaw_rate_dps, beta_deg, 1e-6);       // gain=1，右转正
    EXPECT_DOUBLE_EQ(out.yaw_deg, 0.0);
}

TEST(TrackingControlLawTest, RateModeClampsToYawRateLimit) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.track.center = {950.0, 500.0};  // 大偏移：β≈43.5° > 30°/s 上限
    const auto out = law.Update(snap, 1000);
    EXPECT_NEAR(out.yaw_rate_dps, 30.0, 1e-9);
}

TEST(TrackingControlLawTest, PositionModeComposesAbsoluteHeading) {
    // A2：位置式输出为绝对 NED 航向 = 当前航向 + 限幅修正量
    auto cfg = MakeConfig();
    cfg.heading.mode = HeadingMode::kPosition;
    TrackingControlLaw law(cfg);
    auto snap = MakeSnapshot(1000);
    snap.track.center = {600.0, 500.0};
    snap.yaw_rad = 45.0 * kDegToRad;
    const auto out = law.Update(snap, 1000);
    const double beta_deg = std::atan(0.2) * kRadToDeg;
    EXPECT_NEAR(out.yaw_deg, 45.0 + beta_deg, 1e-6);
    EXPECT_DOUBLE_EQ(out.yaw_rate_dps, 0.0);
}

TEST(TrackingControlLawTest, PositionModeWrapsAround180) {
    auto cfg = MakeConfig();
    cfg.heading.mode = HeadingMode::kPosition;
    TrackingControlLaw law(cfg);
    auto snap = MakeSnapshot(1000);
    snap.track.center = {600.0, 500.0};
    snap.yaw_rad = 175.0 * kDegToRad;
    const auto out = law.Update(snap, 1000);
    const double beta_deg = std::atan(0.2) * kRadToDeg;
    // 175 + 11.31 = 186.31 → 回绕到 ≈ -173.69
    EXPECT_NEAR(out.yaw_deg, 175.0 + beta_deg - 360.0, 1e-6);
    EXPECT_GT(out.yaw_deg, -180.0);
    EXPECT_LT(out.yaw_deg, 180.0);
}

// ---- 垂直统一速度支路（§5，A1） ----

TEST(TrackingControlLawTest, VerticalOffsetDrivesVz) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.track.center = {500.0, 600.0};  // 目标偏低 100 像素 → α>0 → 下降
    const auto out = law.Update(snap, 1000);
    const double alpha_deg = std::atan(0.2) * kRadToDeg;
    EXPECT_NEAR(out.vz_mps, 0.1 * alpha_deg, 1e-6);
    EXPECT_EQ(out.mode, ControlMode::kVelocityHeading);  // A1：不切姿态微调模式
}

TEST(TrackingControlLawTest, VerticalVelocityClamped) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.track.center = {500.0, 990.0};  // α≈44.5° × 0.1 = 4.45 > 2 上限
    const auto out = law.Update(snap, 1000);
    EXPECT_NEAR(out.vz_mps, 2.0, 1e-9);
}

// ---- 距离通道（§6.2 / §6.3） ----

TEST(TrackingControlLawTest, DistanceApproachWhenFar) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.radar_distance_m = 30.0;  // Δd = +20 → kp 输出 10 → 限幅 3
    snap.associated_to_target = true;
    snap.radar_time_ms = 1000;
    const auto out = law.Update(snap, 1000);
    EXPECT_NEAR(out.vx_mps, 3.0, 1e-9);
}

TEST(TrackingControlLawTest, DistanceRetreatClampedWhenTooClose) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.radar_distance_m = 5.0;  // Δd = -5 → kp 输出 -2.5 → 后退限幅 -1
    snap.associated_to_target = true;
    snap.radar_time_ms = 1000;
    const auto out = law.Update(snap, 1000);
    EXPECT_NEAR(out.vx_mps, -1.0, 1e-9);
}

TEST(TrackingControlLawTest, NoDistanceHoldZerosVx) {
    TrackingControlLaw law(MakeConfig());  // 缺省 kHold
    auto snap = MakeSnapshot(1000);
    snap.associated_to_target = false;
    const auto out = law.Update(snap, 1000);
    EXPECT_EQ(out.mode, ControlMode::kVelocityHeading);
    EXPECT_DOUBLE_EQ(out.vx_mps, 0.0);
}

TEST(TrackingControlLawTest, NoDistanceSlowApproach) {
    auto cfg = MakeConfig();
    cfg.distance.no_distance_action = NoDistanceAction::kSlowApproach;
    TrackingControlLaw law(cfg);
    auto snap = MakeSnapshot(1000);
    snap.associated_to_target = false;
    const auto out = law.Update(snap, 1000);
    EXPECT_NEAR(out.vx_mps, 0.5, 1e-9);
}

TEST(TrackingControlLawTest, NoDistanceExitDegrades) {
    auto cfg = MakeConfig();
    cfg.distance.no_distance_action = NoDistanceAction::kExit;
    TrackingControlLaw law(cfg);
    auto snap = MakeSnapshot(1000);
    snap.associated_to_target = false;
    const auto out = law.Update(snap, 1000);
    EXPECT_EQ(out.mode, ControlMode::kDegradedHold);
    EXPECT_EQ(out.degraded_reason, DegradedReason::kNoDistanceExit);
    EXPECT_FALSE(out.valid);
}

TEST(TrackingControlLawTest, StaleRadarFallsBackToNoDistance) {
    TrackingControlLaw law(MakeConfig());  // kHold
    auto snap = MakeSnapshot(1000);
    snap.radar_distance_m = 30.0;
    snap.associated_to_target = true;
    snap.radar_time_ms = 400;  // 600 ms 前 > 500 ms 阈值 → 距离不可用
    const auto out = law.Update(snap, 1000);
    EXPECT_EQ(out.mode, ControlMode::kVelocityHeading);
    EXPECT_DOUBLE_EQ(out.vx_mps, 0.0);
}

// ---- 首拍/限幅（§8、S6） ----

TEST(TrackingControlLawTest, FirstTickVxSlewsFromZero) {
    // S6：首条速度指令必须经变化率限幅，从 0 渐变而非阶跃
    auto cfg = MakeConfig();
    cfg.accel_limit.ax_mps2 = 2.0;
    TrackingControlLaw law(cfg);
    auto snap = MakeSnapshot(1000);
    snap.radar_distance_m = 30.0;  // 通道输出本应为 3.0
    snap.associated_to_target = true;
    snap.radar_time_ms = 1000;
    const auto out = law.Update(snap, 1000);  // 首拍：dt 按 1/20 Hz 兜底
    EXPECT_NEAR(out.vx_mps, 2.0 * (1.0 / 20.0), 1e-9);  // 0.1
}

// ---- 机体系 → NED 旋转（§2.3，A4） ----

TEST(TrackingControlLawTest, OutputRotatedToNedByYaw) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.yaw_rad = kPi / 2.0;  // 机头朝东
    snap.radar_distance_m = 30.0;
    snap.associated_to_target = true;
    snap.radar_time_ms = 1000;
    const auto out = law.Update(snap, 1000);
    // 机体前向 3.0 → 东向 3.0，北向 ≈ 0
    EXPECT_NEAR(out.vx_mps, 0.0, 1e-9);
    EXPECT_NEAR(out.vy_mps, 3.0, 1e-9);
}

TEST(TrackingControlLawTest, ZeroYawKeepsForwardAsNorth) {
    TrackingControlLaw law(MakeConfig());
    auto snap = MakeSnapshot(1000);
    snap.yaw_rad = 0.0;
    snap.radar_distance_m = 30.0;
    snap.associated_to_target = true;
    snap.radar_time_ms = 1000;
    const auto out = law.Update(snap, 1000);
    EXPECT_NEAR(out.vx_mps, 3.0, 1e-9);
    EXPECT_NEAR(out.vy_mps, 0.0, 1e-9);
}

// ---- Reset（§6.4） ----

TEST(TrackingControlLawTest, ResetRestoresSlewFromZero) {
    auto cfg = MakeConfig();
    cfg.accel_limit.ax_mps2 = 2.0;
    TrackingControlLaw law(cfg);
    auto snap = MakeSnapshot(1000);
    snap.radar_distance_m = 30.0;
    snap.associated_to_target = true;
    snap.radar_time_ms = 1000;
    for (std::int64_t t = 1000; t < 3000; t += 50) {
        snap.track_time_ms = t;
        snap.radar_time_ms = t;
        snap.attitude_time_ms = t;
        law.Update(snap, t);  // 若干拍后 vx 已爬升到限幅 3.0
    }
    law.Reset();
    snap.track_time_ms = 3000;
    snap.radar_time_ms = 3000;
    snap.attitude_time_ms = 3000;
    const auto out = law.Update(snap, 3000);
    // Reset 后限幅器归零：又从 0.1 开始渐变
    EXPECT_NEAR(out.vx_mps, 0.1, 1e-9);
}

}  // namespace
