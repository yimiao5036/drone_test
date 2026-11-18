/**
 * @file sgm_matcher.cpp
 * @brief 简化 SGM 实现（代价体 + 多路径聚合 + WTA + 亚像素 + 唯一性）
 */
#include "stereo_depth/sgm_matcher.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "stereo_depth/census_transform.h"

namespace stereo_depth {
namespace {
/// 越界（x-d<0）代价：远大于真实 Hamming（<=48/64），使 WTA 避开。
constexpr std::uint16_t kInvalidCost = 4096;
}  // namespace

SgmMatcher::SgmMatcher(const SgmConfig& cfg,
                       std::unique_ptr<ISubpixelRefiner> refiner)
    : cfg_(cfg), refiner_(std::move(refiner)) {
    if (!refiner_) throw std::invalid_argument("SgmMatcher 需要非空亚像素估计器");
    if (cfg_.max_disparity < 2) throw std::invalid_argument("max_disparity 必须 >= 2");
    if (cfg_.paths != 4 && cfg_.paths != 8)
        throw std::invalid_argument("paths 仅支持 4 或 8");
    p1i_ = static_cast<std::int32_t>(cfg_.p1);
    p2i_ = static_cast<std::int32_t>(cfg_.p2);
}

SgmMatcher::~SgmMatcher() = default;

void SgmMatcher::AggregatePath(const std::uint16_t* cost, std::uint32_t width,
                               std::uint32_t height, int dr, int dc) {
    const int W = static_cast<int>(width);
    const int H = static_cast<int>(height);
    const int D = cfg_.max_disparity;

    const int r_start = (dr >= 0) ? 0 : H - 1;
    const int r_end = (dr >= 0) ? H : -1;
    const int r_step = (dr >= 0) ? 1 : -1;
    const int c_start = (dc >= 0) ? 0 : W - 1;
    const int c_end = (dc >= 0) ? W : -1;
    const int c_step = (dc >= 0) ? 1 : -1;

    for (int r = r_start; r != r_end; r += r_step) {
        for (int c = c_start; c != c_end; c += c_step) {
            const std::size_t cur =
                (static_cast<std::size_t>(r) * width + static_cast<std::size_t>(c)) *
                static_cast<std::size_t>(D);
            const int pr = r - dr;
            const int pc = c - dc;
            const bool has_pred = (pr >= 0 && pr < H && pc >= 0 && pc < W);
            if (!has_pred) {
                for (int d = 0; d < D; ++d) {
                    const std::int32_t l = static_cast<std::int32_t>(cost[cur + d]);
                    path_[cur + d] = l;
                    agg_[cur + d] += l;
                }
                continue;
            }
            const std::size_t p =
                (static_cast<std::size_t>(pr) * width + static_cast<std::size_t>(pc)) *
                static_cast<std::size_t>(D);
            std::int32_t min_prev = path_[p];
            for (int d = 1; d < D; ++d) min_prev = std::min(min_prev, path_[p + d]);

            for (int d = 0; d < D; ++d) {
                std::int32_t best = path_[p + d];
                if (d > 0) best = std::min(best, path_[p + d - 1] + p1i_);
                if (d + 1 < D) best = std::min(best, path_[p + d + 1] + p1i_);
                best = std::min(best, min_prev + p2i_);
                const std::int32_t l =
                    static_cast<std::int32_t>(cost[cur + d]) + best - min_prev;
                path_[cur + d] = l;
                agg_[cur + d] += l;
            }
        }
    }
}

void SgmMatcher::ComputeDisparity(const std::uint64_t* ref_census,
                                  const std::uint64_t* tgt_census,
                                  std::uint32_t width, std::uint32_t height,
                                  float* out_disparity) {
    if (ref_census == nullptr || tgt_census == nullptr || out_disparity == nullptr ||
        width == 0 || height == 0) {
        return;
    }
    const int D = cfg_.max_disparity;
    const std::size_t npix = static_cast<std::size_t>(width) * height;
    const std::size_t vol = npix * static_cast<std::size_t>(D);
    cost_.assign(vol, 0);
    agg_.assign(vol, 0);
    path_.assign(vol, 0);

    // ---- 代价体：Census Hamming，x-d<0 记无效 ----
    for (std::uint32_t y = 0; y < height; ++y) {
        for (std::uint32_t x = 0; x < width; ++x) {
            const std::uint64_t cl = ref_census[static_cast<std::size_t>(y) * width + x];
            const std::size_t base =
                (static_cast<std::size_t>(y) * width + x) * static_cast<std::size_t>(D);
            for (int d = 0; d < D; ++d) {
                const std::int64_t tx = static_cast<std::int64_t>(x) - d;
                if (tx < 0) {
                    cost_[base + d] = kInvalidCost;
                } else {
                    const std::uint64_t cr =
                        tgt_census[static_cast<std::size_t>(y) * width +
                                   static_cast<std::size_t>(tx)];
                    cost_[base + d] =
                        static_cast<std::uint16_t>(HammingDistance(cl, cr));
                }
            }
        }
    }

    // ---- 多路径聚合 ----
    if (cfg_.paths == 4) {
        AggregatePath(cost_.data(), width, height, 0, -1);
        AggregatePath(cost_.data(), width, height, 0, 1);
        AggregatePath(cost_.data(), width, height, -1, 0);
        AggregatePath(cost_.data(), width, height, 1, 0);
    } else {
        AggregatePath(cost_.data(), width, height, 0, -1);
        AggregatePath(cost_.data(), width, height, 0, 1);
        AggregatePath(cost_.data(), width, height, -1, 0);
        AggregatePath(cost_.data(), width, height, 1, 0);
        AggregatePath(cost_.data(), width, height, -1, -1);
        AggregatePath(cost_.data(), width, height, -1, 1);
        AggregatePath(cost_.data(), width, height, 1, -1);
        AggregatePath(cost_.data(), width, height, 1, 1);
    }

    // ---- WTA + 唯一性 + 亚像素 ----
    for (std::size_t idx = 0; idx < npix; ++idx) {
        const std::size_t base = idx * static_cast<std::size_t>(D);
        std::int32_t best = agg_[base];
        std::int32_t second = INT32_MAX;
        int d_best = 0;
        for (int d = 1; d < D; ++d) {
            const std::int32_t v = agg_[base + d];
            if (v < best) {
                second = best;
                best = v;
                d_best = d;
            } else if (v < second) {
                second = v;
            }
        }
        if (best >= static_cast<std::int32_t>(kInvalidCost) || second == INT32_MAX) {
            out_disparity[idx] = NAN;
            continue;
        }
        // 唯一性：best 必须明显小于次优，否则歧义
        if (static_cast<double>(best) >
            cfg_.uniqueness_ratio * static_cast<double>(second)) {
            out_disparity[idx] = NAN;
            continue;
        }
        const double c0 = static_cast<double>(best);
        const double c_prev =
            (d_best > 0) ? static_cast<double>(agg_[base + d_best - 1]) : c0;
        const double c_next =
            (d_best + 1 < D) ? static_cast<double>(agg_[base + d_best + 1]) : c0;
        const double delta = refiner_->Refine(c_prev, c0, c_next);
        out_disparity[idx] = static_cast<float>(static_cast<double>(d_best) + delta);
    }
}

}  // namespace stereo_depth
