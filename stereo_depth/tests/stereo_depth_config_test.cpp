#include <gtest/gtest.h>

#include <stdexcept>

#include "stereo_depth/stereo_depth_config.h"

namespace sd = stereo_depth;

#ifndef STEREO_DEPTH_TEST_CONFIG
#define STEREO_DEPTH_TEST_CONFIG ""
#endif
#ifndef STEREO_DEPTH_TEST_CALIB
#define STEREO_DEPTH_TEST_CALIB ""
#endif

TEST(Config, DefaultsAreValid) {
    sd::StereoDepthConfig c;
    EXPECT_NO_THROW(c.Validate());
}

TEST(Config, RejectsBadValues) {
    {
        sd::StereoDepthConfig c;
        c.census_window = 6;
        EXPECT_THROW(c.Validate(), std::invalid_argument);
    }
    {
        sd::StereoDepthConfig c;
        c.sgm_paths = 6;
        EXPECT_THROW(c.Validate(), std::invalid_argument);
    }
    {
        sd::StereoDepthConfig c;
        c.penalty_p2 = c.penalty_p1;  // 需 P2 > P1
        EXPECT_THROW(c.Validate(), std::invalid_argument);
    }
    {
        sd::StereoDepthConfig c;
        c.distance_max_m = c.distance_min_m;
        EXPECT_THROW(c.Validate(), std::invalid_argument);
    }
    {
        sd::StereoDepthConfig c;
        c.subpixel = "bogus";
        EXPECT_THROW(c.Validate(), std::invalid_argument);
    }
    {
        sd::StereoDepthConfig c;
        c.smooth_alpha = 0.0;
        EXPECT_THROW(c.Validate(), std::invalid_argument);
    }
}

TEST(Config, LoadFromJson) {
    sd::StereoDepthConfig c = sd::LoadStereoDepthConfig(STEREO_DEPTH_TEST_CONFIG);
    EXPECT_EQ(c.source_width, 1280u);
    EXPECT_EQ(c.max_disparity, 96);
    EXPECT_EQ(c.sgm_paths, 8);
    EXPECT_EQ(c.census_window, 7);
    EXPECT_EQ(c.subpixel, "linear");
    EXPECT_FALSE(c.calibration_path.empty());
}

TEST(Calibration, LoadRealFile) {
    sd::StereoCalibration cal = sd::LoadStereoCalibration(STEREO_DEPTH_TEST_CALIB);
    EXPECT_TRUE(cal.Valid());
    EXPECT_EQ(cal.image_width, 1280u);
    EXPECT_EQ(cal.image_height, 720u);
    EXPECT_NEAR(cal.baseline_m, 0.0601, 1e-3);
    EXPECT_NEAR(cal.left.Fx(), 1007.82, 0.5);
    // 两目主点差约 54px（去主点视差的历史坑）
    EXPECT_NEAR(cal.right.Cx() - cal.left.Cx(), 54.0, 1.0);
}

TEST(Calibration, MissingFileThrows) {
    EXPECT_THROW(sd::LoadStereoCalibration("/nonexistent/path.json"), std::runtime_error);
}
