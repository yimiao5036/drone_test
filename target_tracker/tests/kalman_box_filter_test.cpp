// =============================================================================
// kalman_box_filter_test.cpp —— KalmanBoxFilter 单元测试
//
// 覆盖 docs/视觉追踪技术路线.md §4 行为：
//   - 初始化状态正确（§4.3）
//   - 静止目标多帧收敛到观测（§4.5）
//   - 匀速目标速度收敛且预测超前一个 dt（§4.4）
//   - 未初始化 predict 返回 nullopt（§4.4）
// =============================================================================

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "target_tracker/kalman_box_filter.h"

using drone::tracker::BoxCXCYWH;
using drone::tracker::KalmanBoxFilter;

namespace {

// 未初始化时 predict 返回 nullopt（§4.4）；update 直接返回不改变状态（§4.5）
TEST(KalmanBoxFilterTest, UninitializedPredictReturnsNullopt) {
    KalmanBoxFilter kf;
    EXPECT_FALSE(kf.Initialized());
    EXPECT_FALSE(kf.Predict().has_value());

    // 未初始化时 update 无副作用
    kf.Update({100.0, 100.0, 20.0, 20.0});
    EXPECT_FALSE(kf.Initialized());
    EXPECT_FALSE(kf.Predict().has_value());
    for (const double v : kf.State()) {
        EXPECT_DOUBLE_EQ(v, 0.0);
    }
}

// 初始化状态正确（§4.3）：速度置零、位置/尺寸取首次观测
TEST(KalmanBoxFilterTest, InitiateSetsStateCorrectly) {
    KalmanBoxFilter kf;
    kf.Initiate({320.0, 240.0, 40.0, 30.0});
    EXPECT_TRUE(kf.Initialized());

    const std::array<double, 8> expected = {
        {320.0, 240.0, 40.0, 30.0, 0.0, 0.0, 0.0, 0.0}};
    for (size_t i = 0; i < expected.size(); ++i) {
        EXPECT_DOUBLE_EQ(kf.State()[i], expected[i]) << "index=" << i;
    }

    // 初始化后 predict 返回状态前 4 维副本（速度为零 → 位置不变，§4.4）
    const auto pred = kf.Predict();
    ASSERT_TRUE(pred.has_value());
    EXPECT_NEAR(pred->cx, 320.0, 1e-9);
    EXPECT_NEAR(pred->cy, 240.0, 1e-9);
    EXPECT_NEAR(pred->w, 40.0, 1e-9);
    EXPECT_NEAR(pred->h, 30.0, 1e-9);
}

// 静止目标：多帧重复观测同一框，滤波输出应收敛到观测值（§4.5）
TEST(KalmanBoxFilterTest, StaticTargetConvergesToObservation) {
    KalmanBoxFilter kf;
    const BoxCXCYWH z{100.0, 150.0, 40.0, 60.0};
    kf.Initiate({104.0, 156.0, 42.0, 62.0});  // 带初始偏差，验证收敛能力

    for (int i = 0; i < 50; ++i) {
        kf.Predict();
        kf.Update(z);
    }

    EXPECT_NEAR(kf.State()[0], z.cx, 0.5);  // cx
    EXPECT_NEAR(kf.State()[1], z.cy, 0.5);  // cy
    EXPECT_NEAR(kf.State()[2], z.w, 0.5);   // w
    EXPECT_NEAR(kf.State()[3], z.h, 0.5);   // h
    // 静止目标速度应收敛到约 0
    EXPECT_NEAR(kf.State()[4], 0.0, 0.1);
    EXPECT_NEAR(kf.State()[5], 0.0, 0.1);
}

// 匀速目标：速度估计收敛到真实速度，且 predict 超前一个 dt（§4.4）
TEST(KalmanBoxFilterTest, ConstantVelocityConvergesAndPredictLeadsOneDt) {
    KalmanBoxFilter kf;
    const double vx = 3.0;   // 像素/帧
    const double vy = 2.0;   // 像素/帧

    // 目标从 (100, 100) 出发匀速运动，每帧观测一次（predict → update）
    BoxCXCYWH z{100.0, 100.0, 30.0, 30.0};
    kf.Initiate(z);
    for (int i = 0; i < 60; ++i) {
        kf.Predict();
        z.cx += vx;
        z.cy += vy;
        kf.Update(z);
    }

    // 速度收敛到真实速度（像素/帧）
    EXPECT_NEAR(kf.State()[4], vx, 0.3) << "vx 未收敛";
    EXPECT_NEAR(kf.State()[5], vy, 0.3) << "vy 未收敛";
    // 位置贴合当前观测
    EXPECT_NEAR(kf.State()[0], z.cx, 1.5);
    EXPECT_NEAR(kf.State()[1], z.cy, 1.5);

    // 再预测一帧：应超前当前观测一个 dt（≈ 速度 × 1 帧）
    const auto pred = kf.Predict();
    ASSERT_TRUE(pred.has_value());
    EXPECT_NEAR(pred->cx, z.cx + vx, 1.0) << "预测未超前一个 dt";
    EXPECT_NEAR(pred->cy, z.cy + vy, 1.0) << "预测未超前一个 dt";
}

// 动态噪声不退化：小尺寸目标多帧观测仍稳定收敛（§4.2 噪声与尺寸成比例）
TEST(KalmanBoxFilterTest, SmallTargetNoiseScalingStable) {
    KalmanBoxFilter kf;
    const BoxCXCYWH z{50.0, 60.0, 4.0, 4.0};  // 小目标（4×4 像素）
    kf.Initiate(z);
    for (int i = 0; i < 30; ++i) {
        kf.Predict();
        kf.Update(z);
    }
    EXPECT_NEAR(kf.State()[0], z.cx, 0.5);
    EXPECT_NEAR(kf.State()[1], z.cy, 0.5);
    EXPECT_TRUE(std::isfinite(kf.State()[2]));
    EXPECT_TRUE(std::isfinite(kf.State()[3]));
}

}  // namespace
