/**
 * @file sgm_matcher.h
 * @brief 简化 SGM（半全局匹配）立体匹配器
 *
 * 输入左右目 rectified 图像的 Census 符号串，输出逐像素亚像素视差（float，
 * NaN=无效）。代价 = Census Hamming；聚合 = 4/8 路径动态规划（P1 连续、P2 跳变
 * 惩罚）；WTA 取最小 + 唯一性比剔除歧义 + 注入的 ISubpixelRefiner 细化。
 *
 * 视差约定：左视差。ref 像素 (x,y) 与 tgt 像素 (x-d, y) 匹配，d∈[0,D)。
 * x-d<0（左边缘）视为无效。
 *
 * 内存：内部复用代价体（uint16）与聚合/路径缓冲（int32），大小 W*H*D；
 * 每个 worker 线程持有独立 matcher 实例，避免并发争用。
 */
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "stereo_depth/subpixel_refiner.h"

namespace stereo_depth {

/// SGM 参数。
struct SgmConfig {
    std::int32_t max_disparity = 96;  ///< 视差搜索上限 D
    std::int32_t paths = 8;           ///< 路径数（4 或 8）
    double p1 = 8.0;                  ///< 连续视差惩罚
    double p2 = 32.0;                 ///< 视差跳变惩罚
    double uniqueness_ratio = 0.9;    ///< 唯一性：best <= ratio*second 才保留
};

/// 立体匹配器接口（可替换）。
class IStereoMatcher {
public:
    virtual ~IStereoMatcher() = default;

    /// 计算左视差。
    /// @param ref_census 参考（左）Census，width*height
    /// @param tgt_census 目标（右）Census，width*height
    /// @param width/height ROI 尺寸
    /// @param out_disparity 输出 float 视差，width*height；NaN=无效
    virtual void ComputeDisparity(const std::uint64_t* ref_census,
                                  const std::uint64_t* tgt_census,
                                  std::uint32_t width, std::uint32_t height,
                                  float* out_disparity) = 0;
};

/// 默认简化 SGM 实现。
class SgmMatcher final : public IStereoMatcher {
public:
    /// @param cfg SGM 参数
    /// @param refiner 亚像素估计器（本对象接管所有权）
    SgmMatcher(const SgmConfig& cfg, std::unique_ptr<ISubpixelRefiner> refiner);
    ~SgmMatcher() override;

    SgmMatcher(const SgmMatcher&) = delete;
    SgmMatcher& operator=(const SgmMatcher&) = delete;

    void ComputeDisparity(const std::uint64_t* ref_census,
                          const std::uint64_t* tgt_census, std::uint32_t width,
                          std::uint32_t height, float* out_disparity) override;

private:
    /// 沿一个方向 (dr,dc) 做路径聚合并累加到 agg_。
    void AggregatePath(const std::uint16_t* cost, std::uint32_t width,
                       std::uint32_t height, int dr, int dc);

    SgmConfig cfg_;
    std::unique_ptr<ISubpixelRefiner> refiner_;
    std::int32_t p1i_ = 0, p2i_ = 0;      ///< 取整后的惩罚
    std::vector<std::uint16_t> cost_;     ///< Census Hamming 代价体 W*H*D
    std::vector<std::int32_t> agg_;       ///< 聚合代价 W*H*D
    std::vector<std::int32_t> path_;      ///< 单路径缓冲 W*H*D
};

}  // namespace stereo_depth
