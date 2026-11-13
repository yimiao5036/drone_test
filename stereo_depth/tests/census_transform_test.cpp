#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "stereo_depth/census_transform.h"

namespace sd = stereo_depth;
namespace {

constexpr std::uint32_t W = 32, H = 32;

std::uint8_t Pattern(int x, int y) {
    return static_cast<std::uint8_t>((x * 31 + y * 17 + ((x * y) % 13) * 7) & 0xFF);
}

}  // namespace

TEST(CensusTransform, ConstantImageIsZero) {
    std::vector<std::uint8_t> img(W * H, 128);
    std::vector<std::uint64_t> out(W * H, 0xDEAD);
    sd::CensusTransform(img.data(), W, H, W, 7, out.data());
    // 内部像素：邻域均等于中心 → 无 bit 置位
    EXPECT_EQ(out[16 * W + 16], 0u);
}

TEST(CensusTransform, BorderIsZero) {
    std::vector<std::uint8_t> img(W * H);
    for (std::uint32_t y = 0; y < H; ++y)
        for (std::uint32_t x = 0; x < W; ++x) img[y * W + x] = Pattern(x, y);
    std::vector<std::uint64_t> out(W * H, 0xDEAD);
    sd::CensusTransform(img.data(), W, H, W, 7, out.data());
    EXPECT_EQ(out[0], 0u);              // 角
    EXPECT_EQ(out[2 * W + 2], 0u);      // margin 内
}

TEST(CensusTransform, TranslationEquivariance) {
    // right(x) = left(x+4) ⇒ censusR(x) == censusL(x+4)
    const int shift = 4;
    std::vector<std::uint8_t> left(W * H), right(W * H);
    for (std::uint32_t y = 0; y < H; ++y) {
        for (std::uint32_t x = 0; x < W; ++x) {
            left[y * W + x] = Pattern(static_cast<int>(x), static_cast<int>(y));
            const int sx = static_cast<int>(x) + shift;
            right[y * W + x] =
                Pattern(sx < static_cast<int>(W) ? sx : static_cast<int>(W) - 1,
                        static_cast<int>(y));
        }
    }
    std::vector<std::uint64_t> cL(W * H), cR(W * H);
    sd::CensusTransform(left.data(), W, H, W, 7, cL.data());
    sd::CensusTransform(right.data(), W, H, W, 7, cR.data());
    for (int x = 12; x <= 20; ++x) {
        const int y = 16;
        EXPECT_EQ(sd::HammingDistance(cL[y * W + x + shift], cR[y * W + x]), 0)
            << "x=" << x;
    }
}

TEST(HammingDistance, Basics) {
    EXPECT_EQ(sd::HammingDistance(0xFF, 0xFF), 0);
    EXPECT_EQ(sd::HammingDistance(0xF0, 0x0F), 8);
}

TEST(CensusTransform, RejectsBadWindow) {
    std::vector<std::uint8_t> img(W * H, 100);
    std::vector<std::uint64_t> out(W * H, 7);
    sd::CensusTransform(img.data(), W, H, W, 6, out.data());  // 非法窗口
    EXPECT_EQ(out[16 * W + 16], 7u);                          // 未被写入
}
