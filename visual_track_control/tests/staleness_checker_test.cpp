// =============================================================================
// staleness_checker_test.cpp —— StalenessChecker 单元测试
//
// 覆盖（物理追踪思路 §7.1）：
//   从未取样判超龄、有效期内新鲜、超出有效期超龄、
//   边界恰好在有效期上（不算超龄）、时间戳不一致判超龄（C4）。
// =============================================================================

#include <gtest/gtest.h>

#include "visual_track_control/staleness_checker.h"

namespace {

using drone::vtc::StalenessChecker;

TEST(StalenessCheckerTest, NeverSampledIsStale) {
    EXPECT_TRUE(StalenessChecker::IsStale(0, 1000, 200));
    EXPECT_TRUE(StalenessChecker::IsStale(-5, 1000, 200));
}

TEST(StalenessCheckerTest, WithinValidityIsFresh) {
    EXPECT_FALSE(StalenessChecker::IsStale(900, 1000, 200));
    EXPECT_FALSE(StalenessChecker::IsStale(1000, 1000, 200));  // 刚取样
}

TEST(StalenessCheckerTest, BeyondValidityIsStale) {
    EXPECT_TRUE(StalenessChecker::IsStale(799, 1000, 200));  // 201 > 200
}

TEST(StalenessCheckerTest, ExactlyAtBoundaryIsFresh) {
    // 超龄判定为严格大于：恰好等于有效期仍算新鲜
    EXPECT_FALSE(StalenessChecker::IsStale(800, 1000, 200));
}

TEST(StalenessCheckerTest, InconsistentTimestampIsStale) {
    // C4：now < sample 时间戳不一致 → 年龄不可信 → 判超龄（宁降级不轻信）
    EXPECT_TRUE(StalenessChecker::IsStale(2000, 1000, 200));
}

}  // namespace
