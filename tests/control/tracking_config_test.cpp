// =============================================================================
// tracking_config_test.cpp —— TrackingConfig::Validate 单元测试
//
// 覆盖：默认配置（标定内参）合法；频率/超龄/焦距/主点范围/增益/PID/限幅
// 各非法取值抛 std::invalid_argument。
// =============================================================================

#include <gtest/gtest.h>

#include "control/tracking_config.h"

namespace {

using drone::control::TrackingConfig;

TEST(TrackingConfigTest, DefaultsAreValid) {
    // 默认值即 2026-09-23 标定实测内参，必须直接可用
    TrackingConfig cfg;
    EXPECT_NO_THROW(cfg.Validate());
    EXPECT_DOUBLE_EQ(cfg.camera.fx_px, 1007.82);
    EXPECT_DOUBLE_EQ(cfg.camera.cy_px, 532.07);  // 偏中心实测值，非 360
}

TEST(TrackingConfigTest, RejectsBadControlGroup) {
    TrackingConfig cfg;
    cfg.control.frequency_hz = 0.0;
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
    cfg = TrackingConfig{};
    cfg.control.visual_stale_ms = 0;
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
}

TEST(TrackingConfigTest, RejectsBadCameraIntrinsics) {
    TrackingConfig cfg;
    cfg.camera.fx_px = 0.0;
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
    cfg = TrackingConfig{};
    cfg.camera.cx_px = -1.0;
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
    cfg = TrackingConfig{};
    cfg.camera.cy_px = 721.0;  // 超出画面高 720
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
    cfg = TrackingConfig{};
    cfg.camera.cy_px = 720.0;  // 边界值合法（主点可贴边）
    EXPECT_NO_THROW(cfg.Validate());
}

TEST(TrackingConfigTest, RejectsBadChannelParams) {
    TrackingConfig cfg;
    cfg.heading.yaw_rate_limit_dps = 0.0;
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
    cfg = TrackingConfig{};
    cfg.distance.derivative_filter_coef = 1.0;  // 必须 ∈ [0,1)
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
    cfg = TrackingConfig{};
    cfg.distance.d_exp_m = -1.0;
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
    cfg = TrackingConfig{};
    cfg.accel_limit.az_mps2 = 0.0;
    EXPECT_THROW(cfg.Validate(), std::invalid_argument);
}

}  // namespace
