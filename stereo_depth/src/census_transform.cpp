/**
 * @file census_transform.cpp
 * @brief Census 变换实现（标量；NEON 可后续按行向量化）
 */
#include "stereo_depth/census_transform.h"

#include <cstring>

namespace stereo_depth {

void CensusTransform(const std::uint8_t* img, std::uint32_t width,
                     std::uint32_t height, std::uint32_t stride,
                     std::int32_t window, std::uint64_t* out) {
    if (img == nullptr || out == nullptr || width == 0 || height == 0) return;
    if (window != 5 && window != 7) return;  // 仅支持 5/7

    const int half = window / 2;
    // 边界像素置 0
    std::memset(out, 0, sizeof(std::uint64_t) * static_cast<std::size_t>(width) * height);

    if (width <= static_cast<std::uint32_t>(2 * half) ||
        height <= static_cast<std::uint32_t>(2 * half)) {
        return;  // 图像太小，全部边界
    }

    const int num_bits = window * window - 1;  // 48 或 24
    (void)num_bits;
    for (std::uint32_t y = static_cast<std::uint32_t>(half);
         y < height - static_cast<std::uint32_t>(half); ++y) {
        const std::uint8_t* row = img + static_cast<std::size_t>(y) * stride;
        for (std::uint32_t x = static_cast<std::uint32_t>(half);
             x < width - static_cast<std::uint32_t>(half); ++x) {
            const std::uint8_t c = row[x];
            std::uint64_t bits = 0;
            int bit = 0;
            for (int dy = -half; dy <= half; ++dy) {
                const std::uint8_t* wr = img + static_cast<std::size_t>(y + dy) * stride;
                for (int dx = -half; dx <= half; ++dx) {
                    if (dx == 0 && dy == 0) continue;  // 跳过中心
                    if (wr[x + dx] < c) bits |= (std::uint64_t{1} << bit);
                    ++bit;
                }
            }
            out[static_cast<std::size_t>(y) * width + x] = bits;
        }
    }
}

}  // namespace stereo_depth
