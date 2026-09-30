#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "stereo_depth/stereo_depth_sensor.h"

namespace sd = stereo_depth;
namespace {

constexpr std::uint32_t W = 128, H = 128;
constexpr int kTrueDisp = 12;      // 视差 12px
constexpr double kF = 200.0;       // 焦距
constexpr double kB = 0.06;        // 基线
// 期望距离 Z = f·B/d = 200*0.06/12 = 1.0m
constexpr double kExpectedZ = 1.0;

std::uint8_t Tex(int x, int y) {
    std::uint32_t h = static_cast<std::uint32_t>(x) * 374761393u +
                      static_cast<std::uint32_t>(y) * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return static_cast<std::uint8_t>((h ^ (h >> 16)) & 0xFF);
}

// 写一个理想合成标定 JSON，返回路径。
std::string WriteIdealCalibration() {
    const std::string path =
        (std::filesystem::temp_directory_path() / "sd_test_ideal_calib.json").string();
    std::ofstream f(path);
    f << R"({
      "image_width": 128, "image_height": 128,
      "left":  {"camera_matrix": [[200,0,63.5],[0,200,63.5],[0,0,1]],
                "dist_coeffs": [0,0,0,0,0], "rms": 0.1},
      "right": {"camera_matrix": [[200,0,63.5],[0,200,63.5],[0,0,1]],
                "dist_coeffs": [0,0,0,0,0], "rms": 0.1},
      "stereo": {"R": [[1,0,0],[0,1,0],[0,0,1]],
                 "T_m": [0.06,0,0], "baseline_m": 0.06, "rms": 0.1}
    })";
    return path;
}

sd::StereoDepthConfig MakeSyntheticConfig() {
    sd::StereoDepthConfig c;
    c.source_width = W;
    c.source_height = H;
    c.max_disparity = 32;
    c.distance_min_m = 0.5;
    c.distance_max_m = 5.0;
    c.disparity_min_px = 1.0;
    c.roi_margin_px = 8;
    c.census_window = 7;
    c.sgm_paths = 8;
    c.min_valid_ratio = 0.2;
    c.subpixel = "linear";
    return c;
}

}  // namespace

TEST(StereoDepthSensor, QueryBeforeProcessIsInvalid) {
    sd::StereoDepthSensor s(sd::LoadStereoDepthConfig(STEREO_DEPTH_TEST_CONFIG));
    ASSERT_TRUE(s.Init(STEREO_DEPTH_TEST_CALIB));
    EXPECT_FALSE(s.HasFrame());
    sd::QueryRect q{100, 100, 40, 40, 0};
    sd::DistanceResult r = s.Query(q);
    EXPECT_FALSE(r.valid);
}

TEST(StereoDepthSensor, ProcessRejectsWrongDims) {
    sd::StereoDepthSensor s(sd::LoadStereoDepthConfig(STEREO_DEPTH_TEST_CONFIG));
    ASSERT_TRUE(s.Init(STEREO_DEPTH_TEST_CALIB));
    std::vector<std::uint8_t> buf(64 * 64, 128);
    sd::GrayImage g{buf.data(), 64, 64, 64, 0};  // 与 source 1280x720 不符
    sd::StereoFramePair pair{g, g};
    EXPECT_FALSE(s.Process(pair));
}

TEST(StereoDepthSensor, EndToEndSyntheticDistance) {
    sd::StereoDepthSensor s(MakeSyntheticConfig());
    ASSERT_TRUE(s.Init(WriteIdealCalibration()));

    std::vector<std::uint8_t> left(W * H), right(W * H);
    for (std::uint32_t y = 0; y < H; ++y) {
        for (std::uint32_t x = 0; x < W; ++x) {
            left[y * W + x] = Tex(static_cast<int>(x), static_cast<int>(y));
            const int sx = static_cast<int>(x) + kTrueDisp;  // right(x)=left(x+d)
            right[y * W + x] =
                Tex(sx < static_cast<int>(W) ? sx : static_cast<int>(W) - 1,
                    static_cast<int>(y));
        }
    }
    sd::GrayImage L{left.data(), W, H, W, 0};
    sd::GrayImage R{right.data(), W, H, W, 0};
    sd::StereoFramePair pair{L, R};
    ASSERT_TRUE(s.Process(pair));
    EXPECT_TRUE(s.HasFrame());
    EXPECT_EQ(s.ProcessedFrameCount(), 1u);

    sd::QueryRect q{48, 48, 32, 32, /*track_id=*/7};
    sd::DistanceResult r = s.Query(q);
    ASSERT_TRUE(r.valid) << "valid_ratio=" << r.valid_pixel_ratio_x100;
    EXPECT_NEAR(r.distance_m, kExpectedZ, 0.15);
    EXPECT_NEAR(r.disparity_px, static_cast<double>(kTrueDisp), 1.0);
    EXPECT_EQ(s.ValidCount(), 1u);
}

TEST(StereoDepthSensor, FilterSmoothsAndResetClears) {
    sd::StereoDepthSensor s(MakeSyntheticConfig());
    ASSERT_TRUE(s.Init(WriteIdealCalibration()));
    std::vector<std::uint8_t> left(W * H), right(W * H);
    for (std::uint32_t y = 0; y < H; ++y)
        for (std::uint32_t x = 0; x < W; ++x) {
            left[y * W + x] = Tex(static_cast<int>(x), static_cast<int>(y));
            const int sx = static_cast<int>(x) + kTrueDisp;
            right[y * W + x] =
                Tex(sx < static_cast<int>(W) ? sx : static_cast<int>(W) - 1,
                    static_cast<int>(y));
        }
    sd::GrayImage L{left.data(), W, H, W, 0};
    sd::GrayImage R{right.data(), W, H, W, 0};
    sd::StereoFramePair pair{L, R};
    ASSERT_TRUE(s.Process(pair));
    sd::QueryRect q{48, 48, 32, 32, 3};
    sd::DistanceResult r1 = s.Query(q);
    sd::DistanceResult r2 = s.Query(q);  // 同一 track 再查，滤波后应仍接近真值
    ASSERT_TRUE(r1.valid && r2.valid);
    EXPECT_NEAR(r2.distance_m, kExpectedZ, 0.15);
    s.ResetFilters();  // 不应崩溃，后续查询重新初始化滤波
    sd::DistanceResult r3 = s.Query(q);
    EXPECT_TRUE(r3.valid);
}
