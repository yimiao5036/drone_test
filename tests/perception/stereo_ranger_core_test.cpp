#include "perception/stereo_ranger_core.h"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace drone::perception {
namespace {

// 构造左目检测：x=左边缘，y_center=框中心 y
common::DetectionResult MakeLeft(double x, double y_center, double w, double h,
                                 std::uint32_t class_id = 0,
                                 float confidence = 0.9f) {
    common::DetectionResult d;
    d.class_id = class_id;
    d.confidence = confidence;
    d.bbox_x = static_cast<float>(x);
    d.bbox_y = static_cast<float>(y_center - h / 2);
    d.bbox_w = static_cast<float>(w);
    d.bbox_h = static_cast<float>(h);
    d.center_pixel_x = static_cast<float>(x + w / 2);
    d.center_pixel_y = static_cast<float>(y_center);
    return d;
}

// 由目标距离 z_m 与左目检测生成几何一致的右目检测（去主点视差 + 去 cy 纵向）
common::DetectionResult MakeRightFor(double z_m,
                                     const common::DetectionResult& left,
                                     const StereoRangerConfig& cfg) {
    const double u_left = (left.bbox_x - cfg.cx_left_px) / cfg.fx_left_px;
    const double u_right = u_left - cfg.baseline_m / z_m;
    const double x_right = cfg.cx_right_px + u_right * cfg.fx_right_px;
    const double v_left =
        (left.center_pixel_y - cfg.cy_left_px) / cfg.fy_left_px;
    const double y_right = cfg.cy_right_px + v_left * cfg.fy_right_px;
    common::DetectionResult d = left;
    d.bbox_x = static_cast<float>(x_right);
    d.bbox_y = static_cast<float>(y_right - left.bbox_h / 2);
    d.center_pixel_x = static_cast<float>(x_right + left.bbox_w / 2);
    d.center_pixel_y = static_cast<float>(y_right);
    return d;
}

const StereoRangerConfig kCfg{};  // 默认值即 2026-09-23 标定实测

TEST(GreedyStereoMatcherTest, MatchesGeometricallyConsistentPair) {
    const GreedyStereoMatcher matcher(kCfg);
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    const auto right = MakeRightFor(5.0, left, kCfg);
    const auto matches = matcher.Match({left}, {right});
    ASSERT_EQ(matches.size(), 1u);
    EXPECT_EQ(matches[0].left_index, 0u);
    EXPECT_EQ(matches[0].right_index, 0u);
    EXPECT_LT(matches[0].cost, 1.0);
}

TEST(GreedyStereoMatcherTest, RejectsClassMismatch) {
    const GreedyStereoMatcher matcher(kCfg);
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0, /*class_id=*/0);
    auto right = MakeRightFor(5.0, left, kCfg);
    right.class_id = 1;
    EXPECT_TRUE(matcher.Match({left}, {right}).empty());
}

TEST(GreedyStereoMatcherTest, RejectsVerticalResidualOverTauY) {
    const GreedyStereoMatcher matcher(kCfg);
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    auto right = MakeRightFor(5.0, left, kCfg);
    // Δy（去 cy 后）= +3px > tau_y_px=2
    right.center_pixel_y += 3.f;
    right.bbox_y += 3.f;
    EXPECT_TRUE(matcher.Match({left}, {right}).empty());
}

TEST(GreedyStereoMatcherTest, RejectsDistanceOutOfRange) {
    const GreedyStereoMatcher matcher(kCfg);
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    // 15m 超出 distance_max_m=11（同时是有效域外回归）
    EXPECT_TRUE(matcher.Match({left}, {MakeRightFor(15.0, left, kCfg)}).empty());
    // 0.5m 低于 distance_min_m=1.0
    EXPECT_TRUE(matcher.Match({left}, {MakeRightFor(0.5, left, kCfg)}).empty());
}

TEST(GreedyStereoMatcherTest, RejectsHeightRatioMismatch) {
    const GreedyStereoMatcher matcher(kCfg);
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    auto right = MakeRightFor(5.0, left, kCfg);
    // |100−70|/100 = 0.30 > tau_h_ratio=0.25
    right.bbox_h = 70.f;
    EXPECT_TRUE(matcher.Match({left}, {right}).empty());
}

TEST(GreedyStereoMatcherTest, GreedyOneToOnePrefersLowerCost) {
    const GreedyStereoMatcher matcher(kCfg);
    const auto left_a = MakeLeft(400.0, 400.0, 80.0, 100.0);
    const auto left_b = MakeLeft(900.0, 400.0, 80.0, 100.0);
    // 两个右目候选都在左 A 的容差内：right_near 完全一致（cost≈0），
    // right_far 偏离 Δy=1px（cost≈0.5）；贪心必须把 right_near 配给 A。
    const auto right_near = MakeRightFor(5.0, left_a, kCfg);
    auto right_far = right_near;
    right_far.center_pixel_y += 1.f;
    right_far.bbox_y += 1.f;
    const auto matches =
        matcher.Match({left_a, left_b}, {right_far, right_near});
    ASSERT_EQ(matches.size(), 1u);
    EXPECT_EQ(matches[0].left_index, 0u);
    EXPECT_EQ(matches[0].right_index, 1u);
}

TEST(StereoRangerConfigTest, DefaultsMatchSpecAndValidatePasses) {
    const StereoRangerConfig cfg;
    EXPECT_DOUBLE_EQ(cfg.baseline_m, 0.060085);
    EXPECT_DOUBLE_EQ(cfg.cx_left_px, 620.73);
    EXPECT_DOUBLE_EQ(cfg.cx_right_px, 674.76);
    EXPECT_DOUBLE_EQ(cfg.distance_max_m, 11.0);
    EXPECT_NO_THROW(cfg.Validate());
}

TEST(StereoRangerConfigTest, ValidateRejectsInvalidValues) {
    StereoRangerConfig cfg;
    cfg.distance_min_m = 12.0;  // min >= max
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
    cfg = StereoRangerConfig{};
    cfg.tau_h_ratio = 1.5;      // 超出 (0,1]
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
    cfg = StereoRangerConfig{};
    cfg.smooth_alpha = 0.0;     // 超出 (0,1]
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
    cfg = StereoRangerConfig{};
    cfg.baseline_m = 0.0;
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
}

TEST(StereoRangerCoreTest, TriangulatesFiveMeters) {
    StereoRangerCore core(kCfg);
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    const auto right = MakeRightFor(5.0, left, kCfg);
    const auto out = core.Process({left}, {right}, /*frame_sequence=*/7,
                                  /*source_time_ms=*/12345);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_TRUE(out[0].valid);
    EXPECT_NEAR(out[0].forward_distance_m, 5.0, 1e-3);  // 首帧滤波直通
    EXPECT_NEAR(out[0].disparity_px, 12.111, 1e-2);     // F·B/Z=60.57/5
    EXPECT_GT(out[0].slant_range_m, out[0].forward_distance_m);
    EXPECT_EQ(out[0].frame_sequence, 7u);
    EXPECT_EQ(out[0].header.source_time_ms, 12345u);
    EXPECT_EQ(out[0].track_id, 0u);
    EXPECT_FLOAT_EQ(out[0].bbox_x, 400.f);
    EXPECT_EQ(core.MatchedPairCount(), 1u);
    EXPECT_EQ(core.ValidDistanceCount(), 1u);
}

TEST(StereoRangerCoreTest, NearDistanceSignRegression_PrincipalPointGap54px) {
    // 两目 cx 差 54px 的坑：Z=3m 时 xL−xR = 400−434.14 ≈ −34px（符号为负），
    // 不去主点直接相减会算出负视差/负距离。去主点后必须得到正确 Z。
    StereoRangerCore core(kCfg);
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    const auto right = MakeRightFor(3.0, left, kCfg);
    ASSERT_LT(static_cast<double>(left.bbox_x) - right.bbox_x, 0.0)
        << "测试前提：原始像素差为负（不处理主点差必翻车）";
    const auto out = core.Process({left}, {right}, 1, 0);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_TRUE(out[0].valid);
    EXPECT_NEAR(out[0].forward_distance_m, 3.0, 1e-3);
}

TEST(StereoRangerCoreTest, ValidityDomainBoundaries) {
    StereoRangerCore core(kCfg);
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    {   // 11m（δZ≤2m 边界）有效
        const auto out =
            core.Process({left}, {MakeRightFor(11.0, left, kCfg)}, 1, 0);
        ASSERT_EQ(out.size(), 1u);
        EXPECT_TRUE(out[0].valid);
    }
    {   // 15m 超出匹配距离域 → 未匹配 → valid=false 但方位照常
        const auto out =
            core.Process({left}, {MakeRightFor(15.0, left, kCfg)}, 2, 0);
        ASSERT_EQ(out.size(), 1u);
        EXPECT_FALSE(out[0].valid);
        EXPECT_TRUE(std::isnan(out[0].forward_distance_m));
        EXPECT_FLOAT_EQ(out[0].bbox_x, 400.f);
        EXPECT_FLOAT_EQ(out[0].center_pixel_x_mid, 440.f);  // 左目中心
    }
}

TEST(StereoRangerCoreTest, DisparityBelowMinimumIsInvalid) {
    StereoRangerConfig cfg = kCfg;
    cfg.distance_max_m = 30.0;  // 放宽匹配域以单独触发视差门限
    StereoRangerCore core(cfg);
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    // Z=25m → D_px ≈ 2.42 < disparity_min_px=3.0：匹配成功但无效
    const auto out = core.Process({left}, {MakeRightFor(25.0, left, cfg)}, 1, 0);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_FALSE(out[0].valid);
    EXPECT_NEAR(out[0].disparity_px, 2.4225, 1e-2);
}

TEST(StereoRangerCoreTest, LowPassFilterAndJumpReset) {
    StereoRangerCore core(kCfg);
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    // 帧1 Z=5：初始化直通
    auto out = core.Process({left}, {MakeRightFor(5.0, left, kCfg)}, 1, 0);
    EXPECT_NEAR(out[0].forward_distance_m, 5.0, 1e-3);
    // 帧2 Z=6：α=0.3 → 0.3·6+0.7·5 = 5.3
    out = core.Process({left}, {MakeRightFor(6.0, left, kCfg)}, 2, 0);
    EXPECT_NEAR(out[0].forward_distance_m, 5.3, 1e-3);
    // 帧3 Z=11：|11−5.3|=5.7 > jump_reset_m=5 → 重置为 11
    out = core.Process({left}, {MakeRightFor(11.0, left, kCfg)}, 3, 0);
    EXPECT_NEAR(out[0].forward_distance_m, 11.0, 1e-3);
}

TEST(StereoRangerCoreTest, UnmatchedLeftPublishesBearingOnly) {
    StereoRangerCore core(kCfg);
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    const auto out = core.Process({left}, /*right=*/{}, 9, 77);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_FALSE(out[0].valid);
    EXPECT_TRUE(std::isnan(out[0].forward_distance_m));
    EXPECT_TRUE(std::isnan(out[0].slant_range_m));
    EXPECT_FLOAT_EQ(out[0].confidence, 0.9f);
    EXPECT_EQ(out[0].frame_sequence, 9u);
    EXPECT_EQ(core.MatchedPairCount(), 0u);
}

TEST(StereoRangerCoreTest, DeltaYP95TracksWindow) {
    StereoRangerCore core(kCfg);
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    auto right = MakeRightFor(5.0, left, kCfg);
    right.center_pixel_y += 1.5f;  // Δy=1.5px（仍在 tau_y=2 内）
    right.bbox_y += 1.5f;
    EXPECT_TRUE(std::isnan(core.DeltaYP95Px()));  // 无样本
    (void)core.Process({left}, {right}, 1, 0);
    // fyL≠fyR（1008.02 vs 1006.07）：设计 §3.5 的像素域 Δy=|(yL−cyL)−(yR−cyR)|
    // 对几何一致对自带 |vL|·(fyL−fyR) 固有残差（y_center=400 处 ≈0.2555px），
    // 窗口值 = 注入偏移 1.5 + 固有残差（计划原稿 1.5±1e-6 未计此项，算术上不可达）
    const double inherent_residual =
        std::abs((400.0 - kCfg.cy_left_px) / kCfg.fy_left_px) *
        (kCfg.fy_left_px - kCfg.fy_right_px);
    EXPECT_NEAR(core.DeltaYP95Px(), 1.5 + inherent_residual, 1e-3);
}

TEST(StereoRangerCoreTest, InjectedMockMatcherIsUsed) {
    class FixedMatcher final : public IStereoMatcher {
    public:
        std::vector<StereoMatch> Match(
            const std::vector<common::DetectionResult>& left,
            const std::vector<common::DetectionResult>& right) const override {
            if (left.empty() || right.empty()) {
                return {};
            }
            // 故意反配：left[0] ↔ right[最后一个]，证明注入路径生效
            return {StereoMatch{0, right.size() - 1, 0.0}};
        }
    };
    StereoRangerCore core(kCfg, std::make_shared<FixedMatcher>());
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    const auto junk = MakeLeft(100.0, 100.0, 10.0, 10.0);  // 几何不一致
    const auto right = MakeRightFor(5.0, left, kCfg);
    const auto out = core.Process({left}, {junk, right}, 1, 0);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_NEAR(out[0].forward_distance_m, 5.0, 1e-3);  // 用的是 right[1]
}

}  // namespace
}  // namespace drone::perception
