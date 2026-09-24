#include "perception/stereo_ranger.h"

#include <chrono>
#include <cmath>
#include <thread>

#include <gtest/gtest.h>

#include "common/topic.h"

namespace drone::perception {
namespace {

using namespace std::chrono_literals;

std::uint64_t NowMs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

common::DetectionResult MakeLeft(double x, double y_center, double w,
                                 double h) {
    common::DetectionResult d;
    d.confidence = 0.9f;
    d.bbox_x = static_cast<float>(x);
    d.bbox_y = static_cast<float>(y_center - h / 2);
    d.bbox_w = static_cast<float>(w);
    d.bbox_h = static_cast<float>(h);
    d.center_pixel_x = static_cast<float>(x + w / 2);
    d.center_pixel_y = static_cast<float>(y_center);
    return d;
}

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

StereoRangerConfig TestConfig() {
    StereoRangerConfig cfg;
    cfg.pair_window = 50ms;
    cfg.summary_log_interval = 24h;  // 测试不打摘要
    return cfg;
}

struct Fixture {
    StereoRangerConfig cfg = TestConfig();
    common::Topic<common::DetectionResult> left_topic;
    common::Topic<common::DetectionResult> right_topic;
    StereoRanger ranger{cfg};
    Fixture() {
        ranger.SetLeftInput(left_topic);
        ranger.SetRightInput(right_topic);
    }
};

TEST(StereoRangerTest, StartRequiresBothInputs) {
    StereoRangerConfig cfg = TestConfig();
    common::Topic<common::DetectionResult> left_topic;
    StereoRanger ranger(cfg);
    ranger.SetLeftInput(left_topic);  // 右目未接线
    EXPECT_FALSE(ranger.Start());
    EXPECT_FALSE(ranger.IsRunning());
}

TEST(StereoRangerTest, PairsLeftAndRightWithinWindow) {
    Fixture f;
    auto out_sub = f.ranger.DistanceOutput().Subscribe(4);
    ASSERT_TRUE(f.ranger.Start());
    const StereoRangerConfig cfg;  // 默认标定参数
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    auto left_msg = left;
    left_msg.header.source_time_ms = NowMs();
    (void)f.left_topic.Emplace(left_msg);
    auto right_msg = MakeRightFor(5.0, left, cfg);
    right_msg.header.source_time_ms = left_msg.header.source_time_ms;
    (void)f.right_topic.Emplace(right_msg);

    const auto msg = out_sub.WaitTakeFor(2s);
    ASSERT_TRUE(msg.has_value());
    EXPECT_TRUE((*msg)->valid);
    EXPECT_NEAR((*msg)->forward_distance_m, 5.0, 1e-2);
    EXPECT_EQ((*msg)->header.sequence, 1u);
    EXPECT_GT((*msg)->header.receive_time_ms, 0u);
    f.ranger.Stop();
    EXPECT_GE(f.ranger.ProcessedPairCount(), 1u);
    EXPECT_EQ(f.ranger.ValidDistanceCount(), 1u);
}

TEST(StereoRangerTest, RightBeforeLeftOutOfOrderStillPairs) {
    Fixture f;
    auto out_sub = f.ranger.DistanceOutput().Subscribe(4);
    ASSERT_TRUE(f.ranger.Start());
    const StereoRangerConfig cfg;
    const auto left = MakeLeft(400.0, 400.0, 80.0, 100.0);
    // 右目先发（同 source_time），左目后到
    auto right_msg = MakeRightFor(5.0, left, cfg);
    right_msg.header.source_time_ms = NowMs();
    (void)f.right_topic.Emplace(right_msg);
    std::this_thread::sleep_for(10ms);
    auto left_msg = left;
    left_msg.header.source_time_ms = right_msg.header.source_time_ms;
    (void)f.left_topic.Emplace(left_msg);

    const auto msg = out_sub.WaitTakeFor(2s);
    ASSERT_TRUE(msg.has_value());
    EXPECT_TRUE((*msg)->valid);
    f.ranger.Stop();
}

TEST(StereoRangerTest, UnpairedLeftExpiresWithInvalidDistance) {
    Fixture f;
    auto out_sub = f.ranger.DistanceOutput().Subscribe(4);
    ASSERT_TRUE(f.ranger.Start());
    auto left_msg = MakeLeft(400.0, 400.0, 80.0, 100.0);
    left_msg.header.source_time_ms = NowMs();
    (void)f.left_topic.Emplace(left_msg);  // 右目始终无消息

    const auto msg = out_sub.WaitTakeFor(2s);
    ASSERT_TRUE(msg.has_value());
    EXPECT_FALSE((*msg)->valid);  // 超龄未配对，方位照常发布
    EXPECT_TRUE(std::isnan((*msg)->forward_distance_m));
    EXPECT_FLOAT_EQ((*msg)->bbox_x, 400.f);
    f.ranger.Stop();
    EXPECT_EQ(f.ranger.MatchedPairCount(), 0u);
}

TEST(StereoRangerTest, StopIsIdempotentAndRestartable) {
    Fixture f;
    ASSERT_TRUE(f.ranger.Start());
    f.ranger.Stop();
    f.ranger.Stop();  // 幂等
    EXPECT_FALSE(f.ranger.IsRunning());
    EXPECT_TRUE(f.ranger.Start());  // 可重启（重新订阅）
    f.ranger.Stop();
}

}  // namespace
}  // namespace drone::perception
