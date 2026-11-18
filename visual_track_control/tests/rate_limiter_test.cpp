// =============================================================================
// rate_limiter_test.cpp —— Clamp / SlewRateLimiter 单元测试
//
// 覆盖（物理追踪思路 §8、状态机设计 S6）：
//   静态限幅、变化率限幅步进、首条指令从零渐变（S6）、
//   异常节拍保护、Reset 归零。
// =============================================================================

#include <gtest/gtest.h>

#include "visual_track_control/rate_limiter.h"

namespace {

using drone::vtc::Clamp;
using drone::vtc::SlewRateLimiter;

TEST(ClampTest, ClampsValue) {
    EXPECT_DOUBLE_EQ(Clamp(5.0, -2.0, 2.0), 2.0);
    EXPECT_DOUBLE_EQ(Clamp(-5.0, -2.0, 2.0), -2.0);
    EXPECT_DOUBLE_EQ(Clamp(1.5, -2.0, 2.0), 1.5);
    // 边界值：落在区间内
    EXPECT_DOUBLE_EQ(Clamp(2.0, -2.0, 2.0), 2.0);
    EXPECT_DOUBLE_EQ(Clamp(-2.0, -2.0, 2.0), -2.0);
}

TEST(SlewRateLimiterTest, FirstCommandRampsFromZero) {
    // S6：初始输出为 0，首条大指令只能按变化率渐变
    SlewRateLimiter lim(/*rate_limit=*/2.0);
    EXPECT_DOUBLE_EQ(lim.Limit(10.0, 0.5), 1.0);   // 0 + 2×0.5
    EXPECT_DOUBLE_EQ(lim.Limit(10.0, 0.5), 2.0);   // 继续渐变
    EXPECT_DOUBLE_EQ(lim.Limit(10.0, 10.0), 10.0); // 步长足够 → 到达目标
}

TEST(SlewRateLimiterTest, TracksTargetWithinStep) {
    SlewRateLimiter lim(10.0);
    // 步长 10×1=10，目标 3 在步长内 → 直接到达
    EXPECT_DOUBLE_EQ(lim.Limit(3.0, 1.0), 3.0);
    EXPECT_DOUBLE_EQ(lim.Limit(3.0, 1.0), 3.0);  // 已在目标：保持
}

TEST(SlewRateLimiterTest, NegativeDirection) {
    SlewRateLimiter lim(2.0);
    EXPECT_DOUBLE_EQ(lim.Limit(-10.0, 0.5), -1.0);
    EXPECT_DOUBLE_EQ(lim.Limit(-10.0, 0.5), -2.0);
}

TEST(SlewRateLimiterTest, ZeroOrNegativeDtHolds) {
    SlewRateLimiter lim(2.0);
    lim.Limit(10.0, 1.0);  // 输出 2.0
    EXPECT_DOUBLE_EQ(lim.Limit(10.0, 0.0), 2.0);
    EXPECT_DOUBLE_EQ(lim.Limit(10.0, -1.0), 2.0);
}

TEST(SlewRateLimiterTest, ZeroRateLimitFreezesOutput) {
    SlewRateLimiter lim(0.0);
    EXPECT_DOUBLE_EQ(lim.Limit(5.0, 1.0), 0.0);
}

TEST(SlewRateLimiterTest, ResetToValue) {
    SlewRateLimiter lim(2.0);
    lim.Limit(10.0, 1.0);
    lim.Reset();
    EXPECT_DOUBLE_EQ(lim.LastOutput(), 0.0);
    lim.Reset(3.5);
    EXPECT_DOUBLE_EQ(lim.LastOutput(), 3.5);
    // 复位后从新值继续渐变
    EXPECT_DOUBLE_EQ(lim.Limit(10.0, 1.0), 5.5);  // 3.5 + 2×1
}

TEST(SlewRateLimiterTest, CustomInitialValue) {
    SlewRateLimiter lim(1.0, /*initial=*/5.0);
    EXPECT_DOUBLE_EQ(lim.Limit(10.0, 1.0), 6.0);
}

}  // namespace
