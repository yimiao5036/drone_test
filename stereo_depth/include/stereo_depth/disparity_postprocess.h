/**
 * @file disparity_postprocess.h
 * @brief 视差图后处理与区域聚合
 *
 * - MedianFilter3x3：3×3 NaN 感知中值，抹平孤立噪点。
 * - SpeckleFilter：连通域去斑，小于阈值的同视差小块置无效（离群剔除）。
 * - AggregateRegion：在目标子区域内做鲁棒聚合（中位数/均值），并按有效域
 *   [distance_min, distance_max] 过滤、统计有效占比，输出聚合视差/距离/置信度。
 *
 * 视差图约定：float，NaN 表示无效像素。区域坐标为 ROI patch 内的局部坐标。
 */
#pragma once

#include <cmath>
#include <cstdint>

namespace stereo_depth {

/// 区域聚合结果。
struct RegionStat {
    double disparity_px = NAN;  ///< 聚合视差（左目 rectified 像素）；无效为 NaN
    double distance_m = NAN;    ///< 由聚合视差换算的距离；无效为 NaN
    std::uint32_t valid_count = 0;    ///< 有效且落在有效域内的像素数
    std::uint32_t total_count = 0;    ///< 区域总像素数
    double valid_ratio = 0.0;         ///< valid_count / total_count
};

/// 3×3 NaN 感知中值滤波（in→out，可原地时须 out!=in）。
void MedianFilter3x3(const float* in, std::uint32_t width, std::uint32_t height,
                     float* out);

/// Speckle 去斑：4 连通、视差容差内的连通域若小于 min_size 则全部置 NaN。
/// @param tolerance 同连通域允许的视差差（像素）
/// @param min_size  保留的最小连通域像素数（<=1 则不处理）
void SpeckleFilter(float* disp, std::uint32_t width, std::uint32_t height,
                   double tolerance, int min_size);

/// 在视差图的子区域 [rx,rx+rw)×[ry,ry+rh) 内做鲁棒聚合。
/// @param focal_x_baseline f·B（米·像素），用于视差↔距离换算与有效域过滤
/// @param distance_min_m/distance_max_m 有效域（米）
/// @param use_median true=中位数（抗离群，默认），false=均值
RegionStat AggregateRegion(const float* disp, std::uint32_t width,
                           std::uint32_t height, int rx, int ry, int rw, int rh,
                           double focal_x_baseline, double distance_min_m,
                           double distance_max_m, bool use_median);

}  // namespace stereo_depth
