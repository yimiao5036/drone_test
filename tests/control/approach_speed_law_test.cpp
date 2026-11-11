// =============================================================================
// approach_speed_law_test.cpp —— S5 接近段速度律
//
// 覆盖（2026-11-11 落机载）：
//   - 静态目标：加速—制动—收敛到控制间距，不穿透；
//   - 运动目标：速度匹配后保持间距；
//   - 目标快于机体上限：接近预算为 0，不产生虚假接近；
//   - 垂直通道：目标在上方时输出向上速度（NED 为负）且受速度上限约束；
//   - 配置校验：非法参数启动期 fail-fast。
// =============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "control/approach_speed_law.h"

namespace {

using drone::control::ApproachInput;
using drone::control::ApproachOutput;
using drone::control::ApproachSpeedLaw;
using drone::control::ApproachSpeedLawConfig;

ApproachSpeedLawConfig MakeConfig() {
    ApproachSpeedLawConfig cfg;
    return cfg;
}

/// 以完美速度跟踪推进自机位置：每拍用上一条速度指令积分，验证制导律本身。
void IntegrateOwn(ApproachInput& input, const ApproachOutput& output, double dt) {
    input.own_n_m += output.v_n_mps * dt;
    input.own_e_m += output.v_e_mps * dt;
    input.own_d_m += output.v_d_mps * dt;
    input.own_vn_mps = output.v_n_mps;
    input.own_ve_mps = output.v_e_mps;
    input.own_vd_mps = output.v_d_mps;
}

double HorizontalDistance(const ApproachInput& input) {
    return std::hypot(input.target_n_m - input.own_n_m,
                      input.target_e_m - input.own_e_m);
}

TEST(ApproachSpeedLawTest, StaticTargetSettlesAtStandoffWithoutPenetration) {
    ApproachSpeedLaw law(MakeConfig(), 6.0);
    ApproachInput input;
    input.own_valid = true;
    input.target_valid = true;
    input.target_n_m = 40.0;

    constexpr double kDt = 0.1;
    std::int64_t now_ms = 0;
    double min_distance = 1e9;
    ApproachOutput output;
    for (int i = 0; i < 800; ++i) {
        output = law.Update(input, now_ms);
        ASSERT_TRUE(output.valid);
        min_distance = std::min(min_distance, output.distance_m);
        IntegrateOwn(input, output, kDt);
        now_ms += 100;
    }
    const double final_distance = HorizontalDistance(input);
    EXPECT_GE(min_distance, 2.4) << "制动包络不得穿透控制间距";
    EXPECT_GE(final_distance, 2.5) << "不得冲进间距内";
    EXPECT_LE(final_distance, 3.4) << "应稳定在控制间距附近";
    EXPECT_LT(output.closing_mps, 0.5) << "稳定后相对接近速度应接近 0";
}

TEST(ApproachSpeedLawTest, MovingTargetSpeedMatchHoldsStandoff) {
    ApproachSpeedLaw law(MakeConfig(), 6.0);
    ApproachInput input;
    input.own_valid = true;
    input.target_valid = true;
    input.target_n_m = 40.0;
    input.target_vn_mps = 2.0;

    constexpr double kDt = 0.1;
    std::int64_t now_ms = 0;
    ApproachOutput output;
    for (int i = 0; i < 800; ++i) {
        input.target_sample_ms = now_ms;
        output = law.Update(input, now_ms);
        ASSERT_TRUE(output.valid);
        IntegrateOwn(input, output, kDt);
        input.target_n_m += input.target_vn_mps * kDt;
        now_ms += 100;
    }
    const double final_distance = HorizontalDistance(input);
    EXPECT_GE(final_distance, 2.4);
    EXPECT_LE(final_distance, 6.0) << "速度匹配后应跟住目标，不得被越拉越远";
    EXPECT_NEAR(output.v_n_mps, input.target_vn_mps, 1.0);
}

TEST(ApproachSpeedLawTest, TargetFasterThanCapDoesNotFakeClosing) {
    ApproachSpeedLawConfig cfg = MakeConfig();
    ApproachSpeedLaw law(cfg, 6.0);
    ApproachInput input;
    input.own_valid = true;
    input.target_valid = true;
    input.target_n_m = 40.0;
    input.target_vn_mps = 8.0;

    constexpr double kDt = 0.1;
    std::int64_t now_ms = 0;
    ApproachOutput output;
    for (int i = 0; i < 200; ++i) {
        input.target_sample_ms = now_ms;
        output = law.Update(input, now_ms);
        ASSERT_TRUE(output.valid);
        IntegrateOwn(input, output, kDt);
        input.target_n_m += input.target_vn_mps * kDt;
        now_ms += 100;
    }
    EXPECT_LE(output.closing_mps, 0.5) << "追不上时不得产生虚假接近速度";
    EXPECT_GT(HorizontalDistance(input), 20.0) << "距离应被拉开而不是收敛";
}

TEST(ApproachSpeedLawTest, VerticalChannelPointsUpAndIsLimited) {
    ApproachSpeedLaw law(MakeConfig(), 6.0);
    ApproachInput input;
    input.own_valid = true;
    input.target_valid = true;
    input.target_n_m = 10.0;
    input.target_d_m = -10.0;  // NED：目标在上方 10 m

    const ApproachOutput output = law.Update(input, 0);
    ASSERT_TRUE(output.valid);
    EXPECT_LT(output.v_d_mps, 0.0) << "目标在上方时 NED 垂直速度应为负（上升）";
    EXPECT_LE(std::abs(output.v_d_mps), 2.0) << "垂直速度应受配置上限约束";
}

TEST(ApproachSpeedLawTest, InvalidConfigFailsFast) {
    ApproachSpeedLawConfig cfg;
    cfg.standoff_m = 0.0;
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);

    ApproachSpeedLawConfig bad_safety;
    bad_safety.brake_safety = 1.0;
    EXPECT_THROW(bad_safety.Validate(), std::invalid_argument);
}

}  // namespace
