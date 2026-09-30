/**
 * @file census_transform.h
 * @brief Census 变换与 Hamming 距离
 *
 * Census 将每个像素与其窗口邻域比较，生成比特串：对光照与左右目独立 AE
 * 亮度差鲁棒（硬件记录 §4：左右目曝光独立），是选它而非 SAD/NCC 的关键。
 * 7×7 窗口去中心共 48 位，5×5 共 24 位，均可存入 uint64_t。
 *
 * 边界（margin < window/2 的边缘像素）无完整窗口，置 0（后续代价/后处理会
 * 因缺乏纹理自然过滤）。
 */
#pragma once

#include <cstdint>

namespace stereo_depth {

/// 计算 rectified 灰度图的 Census 符号串。
/// @param img    rectified 灰度平面（行主序，行跨距 stride）
/// @param width  宽
/// @param height 高
/// @param stride 行跨距（字节，>= width）
/// @param window 窗口边长（5 或 7）
/// @param out    输出符号串数组，容量 >= width*height
void CensusTransform(const std::uint8_t* img, std::uint32_t width,
                     std::uint32_t height, std::uint32_t stride,
                     std::int32_t window, std::uint64_t* out);

/// 两个 Census 符号串的 Hamming 距离（不同比特数）。
inline int HammingDistance(std::uint64_t a, std::uint64_t b) {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_popcountll(a ^ b);
#else
    std::uint64_t v = a ^ b;
    int c = 0;
    while (v) { v &= v - 1; ++c; }
    return c;
#endif
}

}  // namespace stereo_depth
