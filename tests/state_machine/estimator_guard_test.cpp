// 估计器健康判据单测（降落与估计器保护 P1，2026-11-17）。
//
// 现场依据：2026-11-17 10:24 EKF 全状态重置 delta_z≈9m + fs_bad_acc_vertical，
// AUTO.LAND 中姿态外扩到 45° 后翻滚。这组用例锁住三件事：
// 1. 9m 级 z 跳变必须**一帧**命中（这是当时唯一来得及的信号）；
// 2. 正常下降/悬停不能误报（误报会打断任务、切错模式）；
// 3. 判据锁存、未上锁不判（地面 EKF 重置无害）。
#include "state_machine/estimator_guard.h"

#include <gtest/gtest.h>

#include <string>

namespace drone::state_machine {
namespace {

using drone::common::FlightStateSnapshot;

/// 构造一帧"一切正常"的空中快照：10m 高、悬停、位置遥测新鲜。
FlightStateSnapshot NormalState(uint64_t stamp_ms) {
    FlightStateSnapshot state;
    state.connected = true;
    state.armed = true;
    state.local_position_valid = true;
    state.local_position_time_valid = true;
    state.local_position_time_ms = stamp_ms;
    state.local_x_m = 0.f;
    state.local_y_m = 0.f;
    state.local_z_m = -10.f;  // NED 向下为正 → 离地 10m
    state.vx_mps = 0.f;
    state.vy_mps = 0.f;
    state.vz_mps = 0.f;
    state.home_local_z_valid = true;
    state.home_local_z_m = 0.f;  // 相对高度 = base - local_z = 10m
    state.baro_altitude_valid = true;
    state.baro_amsl_m = 500.f;
    state.baro_takeoff_ref_valid = true;
    state.baro_takeoff_ref_amsl_m = 490.f;  // 气压 AGL = 10m，与 EKF 一致
    state.sensors_status_valid = true;
    state.sensors_present = 0x200007u;  // GYRO|ACCEL|MAG|AHRS
    state.sensors_health = state.sensors_present;
    return state;
}

TEST(EstimatorGuardTest, 未上锁时地面EKF重置不判降级) {
    EstimatorGuard guard;
    auto first = NormalState(1000);
    ASSERT_FALSE(guard.Update(first, 1000, false).degraded);
    auto jumped = NormalState(1040);
    jumped.local_z_m = -1.f;  // 9m 跳变
    // 未上锁：地面上的重置无害，只刷新基线。
    EXPECT_FALSE(guard.Update(jumped, 1040, false).degraded);
    EXPECT_EQ(guard.result().issue, EstimatorIssue::kNone);
}

TEST(EstimatorGuardTest, EKF高度单拍跳变一帧命中) {
    EstimatorGuard guard;
    ASSERT_FALSE(guard.Update(NormalState(1000), 1000, true).degraded);
    auto jumped = NormalState(1040);
    jumped.local_z_m = -1.f;  // 现场是 z 从 -8.32 跳到 +0.74
    const auto result = guard.Update(jumped, 1040, true);
    ASSERT_TRUE(result.degraded);
    EXPECT_EQ(result.issue, EstimatorIssue::kVerticalJump);
    EXPECT_EQ(result.since_ms, 1040u);
    EXPECT_NEAR(result.magnitude, 9.0, 0.01);
}

TEST(EstimatorGuardTest, 正常下降不因单拍位移误报) {
    EstimatorGuard guard;
    double z = -10.0;
    uint64_t stamp = 1000;
    // 0.8 m/s 慢降（P2 的下降速度），每 40ms 一帧：单拍位移 0.032m。
    for (int i = 0; i < 100; ++i) {
        auto state = NormalState(stamp);
        state.local_z_m = static_cast<float>(z);
        state.vz_mps = 0.8f;  // 向下为正
        state.baro_amsl_m = 490.f + static_cast<float>(-z);
        ASSERT_FALSE(guard.Update(state, stamp, true).degraded) << "第 " << i << " 帧误报";
        z += 0.8 * 0.04;
        stamp += 40;
    }
}

TEST(EstimatorGuardTest, 水平位置跳变命中但被真实高速位移排除) {
    EstimatorGuard guard;
    ASSERT_FALSE(guard.Update(NormalState(1000), 1000, true).degraded);
    auto jumped = NormalState(1040);
    jumped.local_x_m = 5.f;  // 4m/40ms = 100m/s，不是机动
    jumped.vx_mps = 0.f;
    const auto result = guard.Update(jumped, 1040, true);
    ASSERT_TRUE(result.degraded);
    EXPECT_EQ(result.issue, EstimatorIssue::kHorizontalJump);

    // 反向：真的 10m/s 水平机动（40ms 内 0.4m）不该判跳变。
    EstimatorGuard fast;
    auto first = NormalState(2000);
    first.vx_mps = 10.f;
    ASSERT_FALSE(fast.Update(first, 2000, true).degraded);
    auto second = NormalState(2040);
    second.vx_mps = 10.f;
    second.local_x_m = 0.4f;
    EXPECT_FALSE(fast.Update(second, 2040, true).degraded);
}

TEST(EstimatorGuardTest, 局部位置遥测超时判降级) {
    EstimatorGuard guard;
    auto state = NormalState(1000);
    // 600ms 没更新超过 500ms 门限，且位置值本身没变。
    const auto result = guard.Update(state, 1600, true);
    ASSERT_TRUE(result.degraded);
    EXPECT_EQ(result.issue, EstimatorIssue::kLocalPositionStale);
}

TEST(EstimatorGuardTest, 气压高度与EKF不一致持续两秒命中) {
    EstimatorGuard guard;
    uint64_t stamp = 1000;
    // EKF 说 10m、气压说 1m（EKF 被重置但 z 是"自洽"的缓慢平移，跳变判据抓不到）。
    for (int i = 0; i < 25; ++i) {
        auto state = NormalState(stamp);
        state.baro_amsl_m = 491.f;  // 气压 AGL = 1m
        const auto result = guard.Update(state, stamp, true);
        if (result.degraded) {
            EXPECT_EQ(result.issue, EstimatorIssue::kBaroDivergence);
            EXPECT_GE(stamp, 3000u) << "应持续 2s 才判，实际在第 " << i << " 帧就报了";
            return;
        }
        stamp += 100;
    }
    FAIL() << "气压与 EKF 高度差 9m 持续 2.4s 仍未判降级";
}

TEST(EstimatorGuardTest, 关键传感器健康位掉线持续一秒命中) {
    EstimatorGuard guard;
    uint64_t stamp = 1000;
    for (int i = 0; i < 20; ++i) {
        auto state = NormalState(stamp);
        state.sensors_health = state.sensors_present & ~0x4u;  // MAG 不健康
        const auto result = guard.Update(state, stamp, true);
        if (result.degraded) {
            EXPECT_EQ(result.issue, EstimatorIssue::kSensorUnhealthy);
            EXPECT_GE(stamp, 2000u) << "应持续 1s 才判";
            return;
        }
        stamp += 100;
    }
    FAIL() << "MAG 健康位持续掉线 1.9s 仍未判降级";
}

TEST(EstimatorGuardTest, 新息检验比持续超限命中) {
    EstimatorGuard guard;
    uint64_t stamp = 1000;
    for (int i = 0; i < 20; ++i) {
        auto state = NormalState(stamp);
        state.estimator_status_valid = true;
        state.estimator_pos_vert_ratio = 1.8f;
        const auto result = guard.Update(state, stamp, true);
        if (result.degraded) {
            EXPECT_EQ(result.issue, EstimatorIssue::kInnovationRatio);
            EXPECT_GE(stamp, 2000u) << "单帧尖峰不该判，应持续 1s";
            return;
        }
        stamp += 100;
    }
    FAIL() << "比值持续 1.8 超过 1s 仍未判降级";
}

TEST(EstimatorGuardTest, PX4估计器告警文本触发但INFO不触发) {
    EstimatorGuard guard;
    auto info = NormalState(1000);
    info.statustext_valid = true;
    info.statustext_severity = 6;  // INFO
    info.statustext_text = "EKF2 IMU0 is using GPS";
    info.statustext_time_ms = 1000;
    EXPECT_FALSE(guard.Update(info, 1000, true).degraded);

    auto warning = NormalState(1100);
    warning.statustext_valid = true;
    warning.statustext_severity = 3;  // ERROR
    warning.statustext_text = "EKF2 emergency reset due to bad accel";
    warning.statustext_time_ms = 1100;
    const auto result = guard.Update(warning, 1100, true);
    ASSERT_TRUE(result.degraded);
    EXPECT_EQ(result.issue, EstimatorIssue::kStatustext);
}

TEST(EstimatorGuardTest, 判据锁存后不因恢复自动清除) {
    EstimatorGuard guard;
    ASSERT_FALSE(guard.Update(NormalState(1000), 1000, true).degraded);
    auto jumped = NormalState(1040);
    jumped.local_z_m = -1.f;
    ASSERT_TRUE(guard.Update(jumped, 1040, true).degraded);
    // 后续帧完全正常：锁存必须保持，避免抖动下反复切模式。
    uint64_t stamp = 1080;
    for (int i = 0; i < 50; ++i) {
        const auto result = guard.Update(NormalState(stamp), stamp, true);
        ASSERT_TRUE(result.degraded);
        EXPECT_EQ(result.issue, EstimatorIssue::kVerticalJump);
        stamp += 40;
    }
    // 重置后（清任务状态/程序重启）才允许重新判定。
    guard.Reset();
    EXPECT_FALSE(guard.degraded());
    EXPECT_FALSE(guard.Update(NormalState(stamp), stamp, true).degraded);
}

TEST(EstimatorGuardTest, flags掩码默认关闭时置位也不判) {
    EstimatorGuard guard;  // 默认 bad_estimator_flag_mask = 0
    auto state = NormalState(1000);
    state.estimator_status_valid = true;
    state.estimator_flags = 0xFFFFu;
    EXPECT_FALSE(guard.Update(state, 1000, true).degraded);
    EXPECT_EQ(guard.result().issue, EstimatorIssue::kNone);
}

TEST(EstimatorGuardTest, flags掩码打开后按配置位判定) {
    EstimatorGuardThresholds thresholds;
    thresholds.bad_estimator_flag_mask = 0x8u;
    EstimatorGuard guard(thresholds);
    auto state = NormalState(1000);
    state.estimator_status_valid = true;
    state.estimator_flags = 0x8u;
    const auto result = guard.Update(state, 1000, true);
    ASSERT_TRUE(result.degraded);
    EXPECT_EQ(result.issue, EstimatorIssue::kEstimatorFlag);
}

TEST(EstimatorGuardTest, 判据文案非空且可读) {
    for (auto issue : {EstimatorIssue::kNone, EstimatorIssue::kVerticalJump,
                       EstimatorIssue::kHorizontalJump,
                       EstimatorIssue::kEstimatorFlag,
                       EstimatorIssue::kInnovationRatio,
                       EstimatorIssue::kSensorUnhealthy,
                       EstimatorIssue::kBaroDivergence,
                       EstimatorIssue::kStatustext,
                       EstimatorIssue::kLocalPositionStale}) {
        const char* label = EstimatorIssueLabel(issue);
        ASSERT_NE(label, nullptr);
        EXPECT_GT(std::string(label).size(), 0u);
    }
}

}  // namespace
}  // namespace drone::state_machine
