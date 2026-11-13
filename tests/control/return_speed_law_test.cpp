// 返航水平速度剖面单测（2026-11-12）。
//
// 覆盖维护者明确的三种情形：
//   ① 起步加速：进入返航时飞机静止（目标未运动/已掉落被追停到 0）；
//   ② 超速减速：始终追不上目标转返航，飞机速度高于巡航顶速；
//   ③ 末端制动：接近目标时按制动包络收敛并在末端低速档压顶。
// 以及两条硬约束：不做 jerk/不变加速度总量、绝不出现"追不上就加速"。
#include "control/return_speed_law.h"

#include <cmath>

#include <gtest/gtest.h>

namespace drone::control {
namespace {

/// 生产默认参数（config/config.json return_home 节同值）。
ReturnSpeedLawConfig MakeConfig() {
    ReturnSpeedLawConfig config;
    config.cruise_mps = 4.0;
    config.accel_mps2 = 1.5;
    config.decel_mps2 = 2.0;
    config.brake_safety = 1.3;
    config.final_speed_mps = 1.0;
    config.final_zone_m = 10.0;
    config.arrival_radius_m = 2.0;
    return config;
}

ReturnSpeedInput MakeInput(double remaining_m, double ground_speed_mps,
                           double dt_s = 0.1) {
    ReturnSpeedInput input;
    input.valid = true;
    input.remaining_m = remaining_m;
    input.ground_speed_mps = ground_speed_mps;
    input.dt_s = dt_s;
    return input;
}

// ---------------------------------------------------------------------------
// ① 起步加速：飞机静止时必须给出加速斜坡，而不是"没速度就一直不动"
// ---------------------------------------------------------------------------

TEST(ReturnSpeedLawTest, AcceleratesFromStandstillWhenTargetFarAway) {
    ReturnSpeedLawConfig config = MakeConfig();
    ReturnSpeedState state;
    // 远处（400m）、当前静止：制动包络 √(2·(2/1.3)·398)≈35m/s 远高于顶速，
    // 所以本拍应完全受 accel 斜坡限制：1.5m/s² × 0.1s = 0.15m/s。
    const auto out = ComputeReturnSpeed(config, MakeInput(400.0, 0.0), state);
    ASSERT_TRUE(out.valid);
    EXPECT_NEAR(out.speed_mps, 0.15, 1e-6);
    EXPECT_EQ(out.phase, ReturnSpeedPhase::kAccel);
    EXPECT_FALSE(out.brake_limited);

    // 用新的状态从 0 起步跑 40 拍，检查收敛到巡航顶速。
    ReturnSpeedState fresh;
    double speed = 0.0;
    for (int i = 0; i < 40; ++i) {
        const auto step = ComputeReturnSpeed(config, MakeInput(400.0, speed), fresh);
        speed = step.speed_mps;
    }
    EXPECT_NEAR(speed, 4.0, 1e-6) << "加速斜坡应在约 2.7s 后到位并停在巡航顶速";
}

TEST(ReturnSpeedLawTest, NeverJumpsToCruiseInOneStep) {
    ReturnSpeedLawConfig config = MakeConfig();
    // 无论剩余距离多远，静止起步的首拍都只能是 accel×dt。
    for (double remaining : {12.0, 50.0, 400.0, 2000.0}) {
        ReturnSpeedState fresh;
        const auto out =
            ComputeReturnSpeed(config, MakeInput(remaining, 0.0), fresh);
        EXPECT_LE(out.speed_mps, 0.15 + 1e-9)
            << "remaining=" << remaining << " 首拍不得直接给巡航顶速（无 jerk 但有斜坡）";
    }
}

// ---------------------------------------------------------------------------
// ② 超速减速：追不上目标转返航时，飞机速度可能高于巡航顶速
// ---------------------------------------------------------------------------

TEST(ReturnSpeedLawTest, DeceleratesFromOverspeedDownToCruise) {
    ReturnSpeedLawConfig config = MakeConfig();
    ReturnSpeedState state;
    // 8 m/s 进入返航（追踪 10 m/s 被中止），距目标 300m（制动包络不构成
    // 限制），本拍应受 decel 斜坡限制：8 − 2.0×0.1 = 7.8。
    const auto out = ComputeReturnSpeed(config, MakeInput(300.0, 8.0), state);
    ASSERT_TRUE(out.valid);
    EXPECT_NEAR(out.speed_mps, 7.8, 1e-6);
    EXPECT_EQ(out.phase, ReturnSpeedPhase::kDecel);
    EXPECT_FALSE(out.brake_limited);

    // 连续推进到收敛：8 → 4 需要 4/0.2 = 20 拍。
    ReturnSpeedState fresh;
    double speed = 8.0;
    int steps = 0;
    while (speed > 4.0 + 1e-6 && steps < 100) {
        const auto step = ComputeReturnSpeed(config, MakeInput(300.0, speed), fresh);
        speed = step.speed_mps;
        ++steps;
    }
    EXPECT_NEAR(speed, 4.0, 1e-6);
    EXPECT_EQ(steps, 20) << "8→4m/s 在 2m/s² 下应为 20 拍（每拍 0.2）";
}

TEST(ReturnSpeedLawTest, OverspeedCommandMayExceedCruiseWhileDecaying) {
    // 硬约束：减速段指令允许**暂时高于** cruise（那是正在衰减的现有速度）。
    // 若把指令一律钳到 cruise，超速回落会变成瞬跳、斜坡失效。
    ReturnSpeedLawConfig config = MakeConfig();
    ReturnSpeedState state;
    const auto out = ComputeReturnSpeed(config, MakeInput(300.0, 10.0), state);
    ASSERT_TRUE(out.valid);
    EXPECT_GT(out.speed_mps, config.cruise_mps);
    EXPECT_NEAR(out.speed_mps, 9.8, 1e-6);
}

TEST(ReturnSpeedLawTest, NeverExceedsCruiseWhenAccelerating) {
    ReturnSpeedLawConfig config = MakeConfig();
    ReturnSpeedState state;
    const auto out = ComputeReturnSpeed(config, MakeInput(300.0, 3.9), state);
    ASSERT_TRUE(out.valid);
    EXPECT_LE(out.speed_mps, config.cruise_mps + 1e-9);
    EXPECT_NEAR(out.speed_mps, 4.0, 1e-6) << "3.9 + 0.15 应被 cruise 封顶";
}

// ---------------------------------------------------------------------------
// ③ 末端制动 + 低速档
// ---------------------------------------------------------------------------

TEST(ReturnSpeedLawTest, BrakingEnvelopeLimitsApproachSpeed) {
    ReturnSpeedLawConfig config = MakeConfig();
    ReturnSpeedState state;
    // 距目标 6m（进入 final_zone=10m，顶速降为 1.0）；制动包络
    // √(2·(2/1.3)·(6−2)) = √12.3 ≈ 3.51 > 1.0，因此顶速由低速档决定。
    // **减速必须逐拍走斜坡**（本拍 4.0 → 3.8），不能一步跳到 1.0——那正是
    // 维护者要的"不变加速度、不做 jerk 的匀速顶速降低"。
    const auto out = ComputeReturnSpeed(config, MakeInput(6.0, 4.0), state);
    ASSERT_TRUE(out.valid);
    EXPECT_NEAR(out.speed_mps, 3.8, 1e-6);
    EXPECT_LE(out.speed_mps, config.cruise_mps + 1e-9);

    // 收敛到末端低速档（4.0 → 1.0 需 15 拍，40 拍足够）。
    ReturnSpeedState fresh;
    double speed = 4.0;
    for (int i = 0; i < 40; ++i) {
        speed = ComputeReturnSpeed(config, MakeInput(6.0, speed), fresh).speed_mps;
    }
    EXPECT_NEAR(speed, 1.0, 1e-6) << "末端低速档应把速度压到 final_speed_mps";
}

TEST(ReturnSpeedLawTest, FinalSpeedNotHigherThanBrakingEnvelopeNearTarget) {
    ReturnSpeedLawConfig config = MakeConfig();
    ReturnSpeedState state;
    // 距目标仅 2.5m（arrival_radius=2.0，可用 0.5m）：包络
    // √(2·(2/1.3)·0.5) ≈ 1.24 > 末端低速档 1.0，本拍限制项是低速档；
    // 反复推进后应精确收敛到 final_speed_mps，且始终 ≤ 包络允许值。
    const double envelope = std::sqrt(2.0 * (config.decel_mps2 / config.brake_safety) * 0.5);
    double speed = 1.0;
    for (int i = 0; i < 40; ++i) {
        speed = ComputeReturnSpeed(config, MakeInput(2.5, speed), state).speed_mps;
        EXPECT_LE(speed, envelope + 1e-2) << "任何一拍都不得高于制动包络允许值";
    }
    EXPECT_NEAR(speed, config.final_speed_mps, 1e-6)
        << "包络高于低速档时应收敛到 final_speed_mps";
}

TEST(ReturnSpeedLawTest, ArrivalRadiusGivesZeroEnvelope) {
    ReturnSpeedLawConfig config = MakeConfig();
    ReturnSpeedState state;
    // 已在到达半径内：包络为 0。单拍只走 decel 斜坡（1.0 − 0.2 = 0.8）。
    const auto first = ComputeReturnSpeed(config, MakeInput(1.5, 1.0), state);
    ASSERT_TRUE(first.valid);
    EXPECT_NEAR(first.speed_mps, 0.8, 1e-6);
    EXPECT_TRUE(first.brake_limited);

    // 反复推进后**不能**收敛到 0：本工程把 max_speed_mps==0 解释为"未填相位
    // 上限"（控制器回退硬上限），输出 0 会让末端的限速整个解除。剖面给下限
    // kReturnSpeedMinCapMps=0.1，实际停车交给 S12 到达判据 → S13。
    double speed = first.speed_mps;
    for (int i = 0; i < 40; ++i) {
        speed = ComputeReturnSpeed(config, MakeInput(1.5, speed), state).speed_mps;
    }
    EXPECT_NEAR(speed, kReturnSpeedMinCapMps, 1e-9)
        << "到达半径内包络为 0 时输出下限，不得输出 0（控制器 0=不限速）";
}

// ---------------------------------------------------------------------------
// 无效输入 / 复位 / 直通语义
// ---------------------------------------------------------------------------

TEST(ReturnSpeedLawTest, InvalidInputKeepsStateUntouched) {
    ReturnSpeedLawConfig config = MakeConfig();
    ReturnSpeedState state;
    state.prev_speed_mps = 2.5;
    state.initialized = true;
    ReturnSpeedInput input;  // valid=false
    const auto out = ComputeReturnSpeed(config, input, state);
    EXPECT_FALSE(out.valid);
    EXPECT_EQ(state.prev_speed_mps, 2.5);
    EXPECT_TRUE(state.initialized);
}

TEST(ReturnSpeedLawTest, ZeroDtDoesNotJumpToCeiling) {
    // dt<=0（首拍/时钟回退）的契约是"不推进斜坡、也不跳变"：保持实测速度，
    // 不能因为 step<=0 就直通 ceiling（否则静止起步首拍会瞬跳到 4 m/s）。
    ReturnSpeedLawConfig config = MakeConfig();
    ReturnSpeedState state;
    const auto accel = ComputeReturnSpeed(config, MakeInput(300.0, 0.0, 0.0), state);
    ASSERT_TRUE(accel.valid);
    EXPECT_NEAR(accel.speed_mps, kReturnSpeedMinCapMps, 1e-9)
        << "静止 + dt=0：保持实测 0（钳到输出下限），不得跳到 cruise";

    ReturnSpeedState state2;
    const auto decel =
        ComputeReturnSpeed(config, MakeInput(300.0, 8.0, -0.1), state2);
    ASSERT_TRUE(decel.valid);
    EXPECT_NEAR(decel.speed_mps, 8.0, 1e-9)
        << "超速 + dt<0：保持实测 8，不得瞬降到 cruise";
}

TEST(ReturnSpeedLawTest, ResetClearsProfileState) {
    ReturnSpeedState state;
    state.prev_speed_mps = 3.0;
    state.initialized = true;
    ResetReturnSpeedState(state);
    EXPECT_DOUBLE_EQ(state.prev_speed_mps, 0.0);
    EXPECT_FALSE(state.initialized);
}

TEST(ReturnSpeedLawTest, ZeroAccelPassesThroughToCeiling) {
    // accel=0 表示该方向不设斜坡（直通），用于"整套剖面关掉"的排障配置。
    ReturnSpeedLawConfig config = MakeConfig();
    config.accel_mps2 = 0.0;
    ReturnSpeedState state;
    const auto out = ComputeReturnSpeed(config, MakeInput(400.0, 0.0), state);
    ASSERT_TRUE(out.valid);
    EXPECT_NEAR(out.speed_mps, config.cruise_mps, 1e-9);
}

TEST(ReturnSpeedLawTest, NoBoostWhenStalled) {
    // 硬约束：返航目标是固定点，永不"追不上"，因此剖面里**没有任何**加速
    // 补偿钩子。连续贴近目标却不动（剩余距离不缩短）时，速度只会被制动
    // 包络压低，绝不上升。
    ReturnSpeedLawConfig config = MakeConfig();
    ReturnSpeedState state;
    double speed = 4.0;
    double previous = speed;
    for (int i = 0; i < 200; ++i) {
        // 剩余距离恒为 8m（模拟被风顶着飞不动）。
        speed = ComputeReturnSpeed(config, MakeInput(8.0, speed), state).speed_mps;
        EXPECT_LE(speed, previous + 1e-9)
            << "剩余距离不缩短时速度不得上升（无 auto_boost） 第" << i << "拍";
        previous = speed;
    }
    EXPECT_LE(speed, config.final_speed_mps + 1e-6);
}

}  // namespace
}  // namespace drone::control
