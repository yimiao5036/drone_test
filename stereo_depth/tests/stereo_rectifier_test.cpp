#include <gtest/gtest.h>

#include <cmath>

#include "stereo_depth/stereo_rectifier.h"

namespace sd = stereo_depth;
namespace {

// 合成理想标定：R=I，T=[B,0,0]，无畸变，主点居中，fx=fy=f。
sd::StereoCalibration MakeIdealCalibration(std::uint32_t w, std::uint32_t h, double f,
                                           double baseline) {
    sd::StereoCalibration c;
    c.image_width = w;
    c.image_height = h;
    const double cx = 0.5 * (w - 1), cy = 0.5 * (h - 1);
    c.left.camera_matrix = {f, 0, cx, 0, f, cy, 0, 0, 1};
    c.right.camera_matrix = {f, 0, cx, 0, f, cy, 0, 0, 1};
    c.left.dist_coeffs = {0, 0, 0, 0, 0};
    c.right.dist_coeffs = {0, 0, 0, 0, 0};
    c.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    c.T_m = {baseline, 0, 0};
    c.baseline_m = baseline;
    return c;
}

}  // namespace

TEST(StereoRectifier, IdealCalibrationIsIdentityRectification) {
    sd::StereoRectifier r;
    r.Build(MakeIdealCalibration(64, 64, 200.0, 0.06), 64, 64);
    EXPECT_TRUE(r.Ready());
    EXPECT_NEAR(r.Focal(), 200.0, 1e-6);
    EXPECT_NEAR(r.Baseline(), 0.06, 1e-9);
    EXPECT_NEAR(r.FocalTimesBaseline(), 12.0, 1e-6);

    // 理想标定下 source→rectified 应近似恒等
    double u = 0, v = 0;
    r.SourceToRectifiedLeft(20.0, 30.0, &u, &v);
    EXPECT_NEAR(u, 20.0, 1e-6);
    EXPECT_NEAR(v, 30.0, 1e-6);
}

TEST(StereoRectifier, DepthFromDisparity) {
    sd::StereoRectifier r;
    r.Build(MakeIdealCalibration(64, 64, 200.0, 0.06), 64, 64);
    // Z = f·B/d = 12/12 = 1.0m（对应 d = f·B/Z = 200*0.06/1 = 12px）
    EXPECT_NEAR(r.DepthFromDisparity(12.0), 1.0, 1e-9);
    EXPECT_NEAR(r.DepthFromDisparity(6.0), 2.0, 1e-9);
    EXPECT_TRUE(std::isnan(r.DepthFromDisparity(0.0)));
    EXPECT_TRUE(std::isnan(r.DepthFromDisparity(-1.0)));
}

TEST(StereoRectifier, RejectsInvalid) {
    sd::StereoRectifier r;
    sd::StereoCalibration bad = MakeIdealCalibration(64, 64, 200.0, 0.0);
    EXPECT_THROW(r.Build(bad, 64, 64), std::invalid_argument);
    EXPECT_FALSE(r.Ready());
}

#ifndef STEREO_DEPTH_TEST_CALIB
#define STEREO_DEPTH_TEST_CALIB ""
#endif

TEST(StereoRectifier, RealCalibrationBuilds) {
    sd::StereoCalibration cal = sd::LoadStereoCalibration(STEREO_DEPTH_TEST_CALIB);
    sd::StereoRectifier r;
    r.Build(cal, cal.image_width, cal.image_height);
    EXPECT_TRUE(r.Ready());
    EXPECT_EQ(r.RectWidth(), 1280u);
    EXPECT_EQ(r.RectHeight(), 720u);
    EXPECT_NEAR(r.Baseline(), 0.0601, 1e-3);
    EXPECT_GT(r.Focal(), 900.0);
    EXPECT_LT(r.Focal(), 1100.0);
    // 12m 处视差 ≈ f·B/12 ≈ 60.5/12 ≈ 5px
    const double d12 = r.FocalTimesBaseline() / 12.0;
    EXPECT_NEAR(r.DepthFromDisparity(d12), 12.0, 1e-6);
}
