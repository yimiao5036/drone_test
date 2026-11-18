/**
 * @file disparity_postprocess.cpp
 * @brief 视差后处理与区域聚合实现
 */
#include "stereo_depth/disparity_postprocess.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace stereo_depth {

void MedianFilter3x3(const float* in, std::uint32_t width, std::uint32_t height,
                     float* out) {
    if (in == nullptr || out == nullptr || width == 0 || height == 0) return;
    const int W = static_cast<int>(width);
    const int H = static_cast<int>(height);
    float buf[9];
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int n = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                const int yy = y + dy;
                if (yy < 0 || yy >= H) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = x + dx;
                    if (xx < 0 || xx >= W) continue;
                    const float v = in[static_cast<std::size_t>(yy) * width + xx];
                    if (std::isfinite(v)) buf[n++] = v;
                }
            }
            const std::size_t idx = static_cast<std::size_t>(y) * width + x;
            if (n == 0) {
                out[idx] = NAN;
            } else {
                std::nth_element(buf, buf + n / 2, buf + n);
                out[idx] = buf[n / 2];
            }
        }
    }
}

void SpeckleFilter(float* disp, std::uint32_t width, std::uint32_t height,
                   double tolerance, int min_size) {
    if (disp == nullptr || width == 0 || height == 0 || min_size <= 1) return;
    const int W = static_cast<int>(width);
    const int H = static_cast<int>(height);
    const std::size_t npix = static_cast<std::size_t>(W) * H;
    std::vector<std::uint8_t> visited(npix, 0);
    std::vector<std::size_t> stack;
    std::vector<std::size_t> component;

    for (std::size_t seed = 0; seed < npix; ++seed) {
        if (visited[seed] || !std::isfinite(disp[seed])) continue;
        const float seed_val = disp[seed];
        stack.clear();
        component.clear();
        visited[seed] = 1;
        stack.push_back(seed);
        while (!stack.empty()) {
            const std::size_t cur = stack.back();
            stack.pop_back();
            component.push_back(cur);
            const int cx = static_cast<int>(cur % static_cast<std::size_t>(W));
            const int cy = static_cast<int>(cur / static_cast<std::size_t>(W));
            const int nbr[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
            for (auto& d : nbr) {
                const int nx = cx + d[0];
                const int ny = cy + d[1];
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                const std::size_t ni =
                    static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx);
                if (visited[ni] || !std::isfinite(disp[ni])) continue;
                if (std::fabs(static_cast<double>(disp[ni]) -
                              static_cast<double>(seed_val)) > tolerance) {
                    continue;
                }
                visited[ni] = 1;
                stack.push_back(ni);
            }
        }
        if (static_cast<int>(component.size()) < min_size) {
            for (const std::size_t idx : component) disp[idx] = NAN;
        }
    }
}

RegionStat AggregateRegion(const float* disp, std::uint32_t width,
                           std::uint32_t height, int rx, int ry, int rw, int rh,
                           double focal_x_baseline, double distance_min_m,
                           double distance_max_m, bool use_median) {
    RegionStat stat;
    if (disp == nullptr || rw <= 0 || rh <= 0) return stat;
    const int W = static_cast<int>(width);
    const int H = static_cast<int>(height);
    // 裁剪到图像范围
    const int x0 = std::max(0, rx);
    const int y0 = std::max(0, ry);
    const int x1 = std::min(W, rx + rw);
    const int y1 = std::min(H, ry + rh);
    if (x1 <= x0 || y1 <= y0) return stat;

    std::vector<float> samples;
    samples.reserve(static_cast<std::size_t>((x1 - x0)) * (y1 - y0));
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            const float d = disp[static_cast<std::size_t>(y) * width + x];
            if (!std::isfinite(d) || d <= 0.0F) continue;
            const double z = focal_x_baseline / static_cast<double>(d);
            if (z < distance_min_m || z > distance_max_m) continue;
            samples.push_back(d);
        }
    }
    stat.total_count = static_cast<std::uint32_t>((x1 - x0)) *
                       static_cast<std::uint32_t>(y1 - y0);
    stat.valid_count = static_cast<std::uint32_t>(samples.size());
    if (stat.total_count > 0) {
        stat.valid_ratio =
            static_cast<double>(stat.valid_count) / static_cast<double>(stat.total_count);
    }
    if (samples.empty()) return stat;

    double agg;
    if (use_median) {
        const std::size_t mid = samples.size() / 2;
        std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
        agg = static_cast<double>(samples[mid]);
    } else {
        double sum = 0.0;
        for (const float v : samples) sum += static_cast<double>(v);
        agg = sum / static_cast<double>(samples.size());
    }
    stat.disparity_px = agg;
    stat.distance_m = focal_x_baseline / agg;
    return stat;
}

}  // namespace stereo_depth
