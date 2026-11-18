#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "stereo_depth/disparity_postprocess.h"

namespace sd = stereo_depth;

TEST(MedianFilter3x3, RemovesIsolatedOutlier) {
    const std::uint32_t W = 5, H = 5;
    std::vector<float> in(W * H, 5.0F), out(W * H, 0.0F);
    in[2 * W + 2] = 100.0F;  // 中心离群
    sd::MedianFilter3x3(in.data(), W, H, out.data());
    EXPECT_NEAR(out[2 * W + 2], 5.0F, 1e-6);
}

TEST(MedianFilter3x3, FillsFromValidNeighbors) {
    const std::uint32_t W = 3, H = 3;
    std::vector<float> in(W * H, 7.0F), out(W * H, 0.0F);
    in[1 * W + 1] = NAN;  // 中心无效
    sd::MedianFilter3x3(in.data(), W, H, out.data());
    EXPECT_NEAR(out[1 * W + 1], 7.0F, 1e-6);
}

TEST(SpeckleFilter, RemovesSmallComponent) {
    const std::uint32_t W = 8, H = 8;
    std::vector<float> d(W * H, NAN);
    // 一个 2 像素的小连通域（视差 10），应被移除（min_size=5）
    d[3 * W + 3] = 10.0F;
    d[3 * W + 4] = 10.0F;
    // 一个 6 像素的大连通域（视差 20），应保留
    for (int i = 0; i < 6; ++i) d[0 * W + i] = 20.0F;
    sd::SpeckleFilter(d.data(), W, H, 1.0, 5);
    EXPECT_TRUE(std::isnan(d[3 * W + 3]));
    EXPECT_TRUE(std::isnan(d[3 * W + 4]));
    EXPECT_NEAR(d[0 * W + 0], 20.0F, 1e-6);
}

TEST(AggregateRegion, MedianAndRatio) {
    const std::uint32_t W = 4, H = 4;
    std::vector<float> d(W * H, 20.0F);
    // fb=60 → Z=60/20=3.0m，落在 [0.5,10]
    sd::RegionStat st =
        sd::AggregateRegion(d.data(), W, H, 0, 0, 4, 4, 60.0, 0.5, 10.0, true);
    EXPECT_EQ(st.valid_count, 16u);
    EXPECT_EQ(st.total_count, 16u);
    EXPECT_NEAR(st.valid_ratio, 1.0, 1e-9);
    EXPECT_NEAR(st.disparity_px, 20.0, 1e-6);
    EXPECT_NEAR(st.distance_m, 3.0, 1e-6);
}

TEST(AggregateRegion, DomainGatingRejectsFar) {
    const std::uint32_t W = 4, H = 4;
    std::vector<float> d(W * H, 1.0F);  // Z=60/1=60m，超 [0.5,10]
    sd::RegionStat st =
        sd::AggregateRegion(d.data(), W, H, 0, 0, 4, 4, 60.0, 0.5, 10.0, true);
    EXPECT_EQ(st.valid_count, 0u);
    EXPECT_TRUE(std::isnan(st.distance_m));
}

TEST(AggregateRegion, MeanVsMedian) {
    const std::uint32_t W = 3, H = 1;
    std::vector<float> d = {10.0F, 10.0F, 40.0F};  // fb=400
    // 中位数=10 → Z=40；均值=20 → Z=20
    sd::RegionStat med =
        sd::AggregateRegion(d.data(), W, H, 0, 0, 3, 1, 400.0, 1.0, 100.0, true);
    sd::RegionStat mean =
        sd::AggregateRegion(d.data(), W, H, 0, 0, 3, 1, 400.0, 1.0, 100.0, false);
    EXPECT_NEAR(med.disparity_px, 10.0, 1e-6);
    EXPECT_NEAR(mean.disparity_px, 20.0, 1e-6);
}
