#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "stereo_depth/census_transform.h"
#include "stereo_depth/sgm_matcher.h"
#include "stereo_depth/subpixel_refiner.h"

namespace sd = stereo_depth;
namespace {

constexpr std::uint32_t W = 128, H = 96;
constexpr int kTrueDisp = 12;

std::uint8_t Tex(int x, int y) {
    // 确定性纹理，保证局部有足够变化供 Census 区分
    std::uint32_t h = static_cast<std::uint32_t>(x) * 374761393u +
                      static_cast<std::uint32_t>(y) * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return static_cast<std::uint8_t>((h ^ (h >> 16)) & 0xFF);
}

}  // namespace

TEST(SgmMatcher, RecoversKnownDisparity) {
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
    std::vector<std::uint64_t> cL(W * H), cR(W * H);
    sd::CensusTransform(left.data(), W, H, W, 7, cL.data());
    sd::CensusTransform(right.data(), W, H, W, 7, cR.data());

    sd::SgmConfig cfg;
    cfg.max_disparity = 32;
    cfg.paths = 8;
    cfg.p1 = 8;
    cfg.p2 = 32;
    cfg.uniqueness_ratio = 0.95;
    sd::SgmMatcher m(cfg, std::make_unique<sd::LinearSubpixel>());

    std::vector<float> disp(W * H);
    m.ComputeDisparity(cL.data(), cR.data(), W, H, disp.data());

    // 内部区域统计视差命中率（真值 ±0.6px）
    int hit = 0, total = 0;
    for (std::uint32_t y = 20; y < H - 20; ++y) {
        for (std::uint32_t x = 40; x < W - 20; ++x) {
            const float d = disp[y * W + x];
            if (!std::isfinite(d)) continue;
            ++total;
            if (std::abs(d - kTrueDisp) <= 0.6F) ++hit;
        }
    }
    ASSERT_GT(total, 100);
    EXPECT_GT(static_cast<double>(hit) / total, 0.9)
        << "hit=" << hit << " total=" << total;
}

TEST(SgmMatcher, RejectsBadConfig) {
    sd::SgmConfig bad;
    bad.max_disparity = 1;
    EXPECT_THROW(sd::SgmMatcher(bad, std::make_unique<sd::LinearSubpixel>()),
                 std::invalid_argument);
    sd::SgmConfig bad2;
    bad2.paths = 6;
    EXPECT_THROW(sd::SgmMatcher(bad2, std::make_unique<sd::LinearSubpixel>()),
                 std::invalid_argument);
    EXPECT_THROW(sd::SgmMatcher(sd::SgmConfig{}, nullptr), std::invalid_argument);
}
