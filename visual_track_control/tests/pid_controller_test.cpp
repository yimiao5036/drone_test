// =============================================================================
// pid_controller_test.cpp —— PidController 单元测试
//
// 覆盖（物理追踪思路 §6.2 / §6.4）：
//   位置式 P/I/D 三项、积分钳位、输出限幅、异常节拍保护、
//   anti-windup 条件积分（C1 修复回归：饱和冻结 + 误差反号回卷）、
//   ResetIntegral / Reset 行为。
// =============================================================================

#include <gtest/gtest.h>

#include "visual_track_control/pid_controller.h"

namespace {

using drone::vtc::PidController;
using drone::vtc::PidParams;

PidParams MakeParams(double kp, double ki, double kd, double integral_limit,
                     double output_limit, double filter_coef = 0.0) {
    PidParams p;
    p.kp = kp;
    p.ki = ki;
    p.kd = kd;
    p.integral_limit = integral_limit;
    p.derivative_filter_coef = filter_coef;
    p.output_limit = output_limit;
    return p;
}

TEST(PidControllerTest, ProportionalOnly) {
    PidController pid(MakeParams(2.0, 0.0, 0.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(pid.Update(3.0, 0.1), 6.0);
    EXPECT_DOUBLE_EQ(pid.Update(-1.5, 0.1), -3.0);
}

TEST(PidControllerTest, IntegralAccumulates) {
    PidController pid(MakeParams(0.0, 1.0, 0.0, 0.0, 0.0));
    // 两拍 error=2、dt=0.5：积分累计 1.0 → 2.0
    EXPECT_DOUBLE_EQ(pid.Update(2.0, 0.5), 1.0);
    EXPECT_DOUBLE_EQ(pid.Update(2.0, 0.5), 2.0);
}

TEST(PidControllerTest, IntegralClampedToLimit) {
    PidController pid(MakeParams(0.0, 1.0, 0.0, /*integral_limit=*/1.0, 0.0));
    for (int i = 0; i < 5; ++i) {
        pid.Update(10.0, 1.0);
    }
    // 积分候选值被钳位在 1.0 → 输出恒为 1.0
    EXPECT_DOUBLE_EQ(pid.LastOutput(), 1.0);
}

TEST(PidControllerTest, OutputClampedToLimit) {
    PidController pid(MakeParams(10.0, 0.0, 0.0, 0.0, /*output_limit=*/3.0));
    EXPECT_DOUBLE_EQ(pid.Update(1.0, 0.1), 3.0);
    EXPECT_DOUBLE_EQ(pid.Update(-1.0, 0.1), -3.0);
}

TEST(PidControllerTest, AntiWindupFreezeWhileSaturatedSameDirection) {
    // kp=0、ki=2、output_limit=5：error=+1 持续饱和时积分冻结，
    // 输出恒钉在 +5（10 拍累计 10 拍也不会让积分继续增长）。
    PidController pid(MakeParams(0.0, 2.0, 0.0, 0.0, 5.0));
    for (int i = 0; i < 10; ++i) {
        EXPECT_DOUBLE_EQ(pid.Update(10.0, 1.0), 5.0);
    }
}

TEST(PidControllerTest, AntiWindupUnwindsAfterErrorFlips) {
    // C1 回归：误差反号后积分必须立即回卷（旧判据"积分与输出同向"会
    // 冻结回卷、输出一直钉在 +5）。
    // kp=0、ki=2、limit=5：+10 饱和 3 拍、积分冻结在 0；
    // error=-1 首拍 → 输出 2×(-1) = -2（立即回卷）；
    // 持续负误差使输出到达负饱和 -5（积分冻结在 -2，输出 = clamp(-6)）。
    PidController pid(MakeParams(0.0, 2.0, 0.0, 0.0, 5.0));
    for (int i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(pid.Update(10.0, 1.0), 5.0);
    }
    EXPECT_DOUBLE_EQ(pid.Update(-1.0, 1.0), -2.0);  // 首拍立即回卷
    double out = -2.0;
    for (int i = 1; i < 12; ++i) {
        out = pid.Update(-1.0, 1.0);
    }
    EXPECT_NEAR(out, -5.0, 1e-9);  // 旧判据此处仍为 +5.0
}

TEST(PidControllerTest, DerivativeFirstStepIsZero) {
    PidController pid(MakeParams(0.0, 0.0, 1.0, 0.0, 0.0));
    // 首拍无微分历史：微分项为 0
    EXPECT_DOUBLE_EQ(pid.Update(5.0, 1.0), 0.0);
}

TEST(PidControllerTest, DerivativeWithLowPassFilter) {
    PidController pid(MakeParams(0.0, 0.0, 1.0, 0.0, 0.0, /*filter=*/0.5));
    pid.Update(0.0, 1.0);  // 建立微分历史（导数 0）
    // 第二拍原始导数 1.0，低通后 0.5×0 + 0.5×1.0 = 0.5
    EXPECT_DOUBLE_EQ(pid.Update(1.0, 1.0), 0.5);
}

TEST(PidControllerTest, ZeroOrNegativeDtHoldsLastOutput) {
    PidController pid(MakeParams(1.0, 0.0, 0.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(pid.Update(2.0, 0.1), 2.0);
    EXPECT_DOUBLE_EQ(pid.Update(99.0, 0.0), 2.0);
    EXPECT_DOUBLE_EQ(pid.Update(99.0, -1.0), 2.0);
}

TEST(PidControllerTest, ResetIntegralClearsOnlyIntegral) {
    PidController pid(MakeParams(0.0, 1.0, 1.0, 0.0, 0.0));
    pid.Update(2.0, 1.0);  // 积分 = 2
    pid.ResetIntegral();
    // 积分清零、微分历史保留：输出应为 kd×(0−2)/1 = −2
    EXPECT_DOUBLE_EQ(pid.Update(0.0, 1.0), -2.0);
}

TEST(PidControllerTest, ResetClearsEverything) {
    PidController pid(MakeParams(1.0, 1.0, 1.0, 0.0, 0.0));
    pid.Update(5.0, 1.0);
    pid.Reset();
    EXPECT_DOUBLE_EQ(pid.LastOutput(), 0.0);
    // 全清后等同全新控制器：首拍无微分历史
    EXPECT_DOUBLE_EQ(pid.Update(3.0, 1.0), 3.0 + 3.0);  // kp·e + ki·∫e
}

}  // namespace
