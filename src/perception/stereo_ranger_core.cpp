#include "perception/stereo_ranger_core.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace drone::perception {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// ============================================================================
// stereo_ranger_core —— 双目测距 v2（稀疏 Census）纯算法实现。
//
// 语义与参考实现 tools/camera_test_ok/stereo_core.py 逐条对齐（设计文档 §4）：
//   基准图 = SBS 右半 = 物理左目（主工程左目 Topic）；d = u_ref - u_aux > 0。
//   Census 窗口预提取 + 批量 gather；粗(stride)→精(±halfwidth)→抛物线亚像素
//   →唯一性(峰位±2 外次优)→左右一致性(x_L = x_R + d)；bbox 近端簇分位聚合。
//
// 实现注记（与 Python 的已知差异，均不影响深度正确性）：
//   - 影像只在检测框联合区域的行带（±kBandMargin）上做 remap/census/提点，
//     带内结果与全幅逐像素一致（census 只依赖自身邻域）；边界按复制延拓裁剪。
//   - Shi-Tomasi 为等价自研（Sobel3×3 梯度 + blockSize5 矩形和 + 3×3 局部极大
//     + minDistance 贪心抑制 + 质量降序），与 OpenCV 非逐位一致；单测用注入
//     关键点隔离该差异。
//   - remap 为 double 精度双线性（OpenCV 为定点），差 ≤1 LSB；越界抽头取 0。
// ============================================================================

// 行带余量：覆盖 Census 半径(r≤3)、Shi-Tomasi 块(±2)与双线性采样余量。
constexpr int kBandMargin = 8;

// numpy 语义百分位（线性插值）；v 需已升序，空数组返回 NaN。
double PercentileSorted(const std::vector<double>& sorted_v, double q) {
    const std::size_t n = sorted_v.size();
    if (n == 0) {
        return kNaN;
    }
    if (n == 1) {
        return sorted_v[0];
    }
    const double pos = q / 100.0 * static_cast<double>(n - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
    const std::size_t hi = std::min(lo + 1, n - 1);
    const double frac = pos - static_cast<double>(lo);
    return sorted_v[lo] + (sorted_v[hi] - sorted_v[lo]) * frac;
}

double MedianSorted(const std::vector<double>& sorted_v) {
    return PercentileSorted(sorted_v, 50.0);
}

inline int ClampInt(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// 四舍五入到最近整数（半值取偶，同 numpy.rint）后转 int。
inline int RintToInt(double v) {
    return static_cast<int>(std::nearbyint(v));
}

// remap 双线性采样；越界抽头取 0（对齐 OpenCV BORDER_CONSTANT 语义）。
inline double SampleBilinearZero(const std::uint8_t* src, int width, int height,
                                 int stride, double sx, double sy) {
    if (sx < -1.0 || sx > static_cast<double>(width) || sy < -1.0 ||
        sy > static_cast<double>(height)) {
        return 0.0;
    }
    const int x0 = static_cast<int>(std::floor(sx));
    const int y0 = static_cast<int>(std::floor(sy));
    const double fx = sx - static_cast<double>(x0);
    const double fy = sy - static_cast<double>(y0);
    double value = 0.0;
    for (int dy = 0; dy < 2; ++dy) {
        const int y = y0 + dy;
        if (y < 0 || y >= height) {
            continue;
        }
        const double wy = dy == 0 ? 1.0 - fy : fy;
        if (wy == 0.0) {
            continue;
        }
        const std::uint8_t* row = src + static_cast<std::size_t>(y) * stride;
        for (int dx = 0; dx < 2; ++dx) {
            const int x = x0 + dx;
            if (x < 0 || x >= width) {
                continue;
            }
            const double wx = dx == 0 ? 1.0 - fx : fx;
            value += wx * wy * static_cast<double>(row[x]);
        }
    }
    return value;
}

// remap 双线性快路径：四抽头全部在图内时无分支、无逐抽头越界判断
// （行带 remap 的绝大多数像素走此路径）；否则退回通用零边界实现。
// 两者数学等价，浮点末位可能有差（≤1 LSB，落回 uint8 前），与既有
// "remap 与 OpenCV 定点实现差 ≤1 LSB" 的口径一致。
inline double SampleBilinearFast(const std::uint8_t* src, int width, int height,
                                 int stride, double sx, double sy) {
    if (sx >= 0.0 && sx < static_cast<double>(width - 1) && sy >= 0.0 &&
        sy < static_cast<double>(height - 1)) {
        const int x0 = static_cast<int>(sx);
        const int y0 = static_cast<int>(sy);
        const double fx = sx - static_cast<double>(x0);
        const double fy = sy - static_cast<double>(y0);
        const std::uint8_t* row0 =
            src + static_cast<std::size_t>(y0) * stride + x0;
        const std::uint8_t* row1 = row0 + stride;
        const double top = static_cast<double>(row0[0]) * (1.0 - fx) +
                           static_cast<double>(row0[1]) * fx;
        const double bottom = static_cast<double>(row1[0]) * (1.0 - fx) +
                              static_cast<double>(row1[1]) * fx;
        return top * (1.0 - fy) + bottom * fy;
    }
    return SampleBilinearZero(src, width, height, stride, sx, sy);
}

// 构建 rect→source 映射表（等价 cv2.initUndistortRectifyMap 的正向畸变模型）。
void BuildRectToSourceMap(const double k[9], const double d[5], const double r[9],
                          const double p[12], int width, int height,
                          std::vector<float>* map_x, std::vector<float>* map_y) {
    const double fx_p = p[0];
    const double fy_p = p[5];
    const double cx_p = p[2];
    const double cy_p = p[6];
    const double fx_k = k[0];
    const double fy_k = k[4];
    const double cx_k = k[2];
    const double cy_k = k[5];
    const double k1 = d[0], k2 = d[1], p1 = d[2], p2 = d[3], k3 = d[4];
    map_x->resize(static_cast<std::size_t>(width) * height);
    map_y->resize(static_cast<std::size_t>(width) * height);
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            const double x = (u - cx_p) / fx_p;
            const double y = (v - cy_p) / fy_p;
            // [X,Y,Z]ᵀ = Rᵀ·[x,y,1]ᵀ（R 行主序：Rᵀ 的行 = R 的列）
            const double rx = r[0] * x + r[3] * y + r[6];
            const double ry = r[1] * x + r[4] * y + r[7];
            const double rz = r[2] * x + r[5] * y + r[8];
            const double xr = rx / rz;
            const double yr = ry / rz;
            const double rr = xr * xr + yr * yr;
            const double radial = 1.0 + ((k3 * rr + k2) * rr + k1) * rr;
            const double xd =
                xr * radial + 2.0 * p1 * xr * yr + p2 * (rr + 2.0 * xr * xr);
            const double yd =
                yr * radial + p1 * (rr + 2.0 * yr * yr) + 2.0 * p2 * xr * yr;
            const std::size_t idx = static_cast<std::size_t>(v) * width + u;
            (*map_x)[idx] = static_cast<float>(fx_k * xd + cx_k);
            (*map_y)[idx] = static_cast<float>(fy_k * yd + cy_k);
        }
    }
}

// 源图 → 校正图（解析式）：K⁻¹ 归一化 → 畸变逆解（定点迭代 10 次，同
// OpenCV undistortPoints 定点法思路）→ R 旋转 → P 投影。用于检测框映射。
void SourceToRectPoint(const double k[9], const double d[5], const double r[9],
                       const double p[12], double u_src, double v_src,
                       double* u_rect, double* v_rect) {
    const double fx_k = k[0];
    const double fy_k = k[4];
    const double cx_k = k[2];
    const double cy_k = k[5];
    const double k1 = d[0], k2 = d[1], p1 = d[2], p2 = d[3], k3 = d[4];
    const double xd = (u_src - cx_k) / fx_k;
    const double yd = (v_src - cy_k) / fy_k;
    double x = xd;
    double y = yd;
    for (int i = 0; i < 10; ++i) {
        const double rr = x * x + y * y;
        const double radial = 1.0 + ((k3 * rr + k2) * rr + k1) * rr;
        if (std::abs(radial) < 1e-12) {
            break;
        }
        const double delta_x = 2.0 * p1 * x * y + p2 * (rr + 2.0 * x * x);
        const double delta_y = p1 * (rr + 2.0 * y * y) + 2.0 * p2 * x * y;
        x = (xd - delta_x) / radial;
        y = (yd - delta_y) / radial;
    }
    // [X,Y,Z]ᵀ = R·[x,y,1]ᵀ
    const double rx = r[0] * x + r[1] * y + r[2];
    const double ry = r[3] * x + r[4] * y + r[5];
    const double rz = r[6] * x + r[7] * y + r[8];
    const double inv_z = 1.0 / rz;
    *u_rect = p[0] * rx * inv_z + p[2];
    *v_rect = p[5] * ry * inv_z + p[6];
}

// 8 位灰度 Sobel 3×3 梯度（边界 1px 留 0）。
void SobelGradients(const std::uint8_t* img, int width, int height, int stride,
                    std::vector<double>* gx, std::vector<double>* gy) {
    const std::size_t count = static_cast<std::size_t>(width) * height;
    gx->assign(count, 0.0);
    gy->assign(count, 0.0);
    for (int y = 1; y + 1 < height; ++y) {
        const std::uint8_t* up = img + static_cast<std::size_t>(y - 1) * stride;
        const std::uint8_t* mid = img + static_cast<std::size_t>(y) * stride;
        const std::uint8_t* down = img + static_cast<std::size_t>(y + 1) * stride;
        for (int x = 1; x + 1 < width; ++x) {
            const double tl = up[x - 1], tc = up[x], tr = up[x + 1];
            const double ml = mid[x - 1], mr = mid[x + 1];
            const double bl = down[x - 1], bc = down[x], br = down[x + 1];
            const std::size_t idx = static_cast<std::size_t>(y) * width + x;
            (*gx)[idx] = (tr + 2.0 * mr + br) - (tl + 2.0 * ml + bl);
            (*gy)[idx] = (bl + 2.0 * bc + br) - (tl + 2.0 * tc + tr);
        }
    }
}

// 矩形和（积分图）：blockSize 窗口；越界部分按 0 贡献。
// 缓冲区由调用方复用（assign 不缩容，跨帧不再反复分配 MB 级数组）。
class BoxSumIntegral {
public:
    BoxSumIntegral(const std::vector<double>& src, int width, int height,
                   std::vector<double>* scratch)
        : width_(width), height_(height), integral_(scratch) {
        integral_->assign(static_cast<std::size_t>(width + 1) * (height + 1),
                          0.0);
        std::vector<double>& integral = *integral_;
        for (int y = 0; y < height; ++y) {
            double row_sum = 0.0;
            for (int x = 0; x < width; ++x) {
                row_sum += src[static_cast<std::size_t>(y) * width + x];
                integral[static_cast<std::size_t>(y + 1) * (width + 1) + (x + 1)] =
                    integral[static_cast<std::size_t>(y) * (width + 1) + (x + 1)] +
                    row_sum;
            }
        }
    }

    /// 以 (cx,cy) 为中心、半宽 half 的窗口和（越界部分按 0）。
    double Sum(int cx, int cy, int half) const {
        const std::vector<double>& integral = *integral_;
        const int x0 = ClampInt(cx - half, 0, width_);
        const int x1 = ClampInt(cx + half + 1, 0, width_);
        const int y0 = ClampInt(cy - half, 0, height_);
        const int y1 = ClampInt(cy + half + 1, 0, height_);
        const std::size_t w = static_cast<std::size_t>(width_ + 1);
        return integral[static_cast<std::size_t>(y1) * w + x1] -
               integral[static_cast<std::size_t>(y0) * w + x1] -
               integral[static_cast<std::size_t>(y1) * w + x0] +
               integral[static_cast<std::size_t>(y0) * w + x0];
    }

private:
    int width_;
    int height_;
    std::vector<double>* integral_;
};

/// Shi-Tomasi 内部临时缓冲（组件复用；避免每帧重复分配 MB 级数组）。
struct CornerScratch {
    std::vector<double> gx;
    std::vector<double> gy;
    std::vector<double> ixx;
    std::vector<double> iyy;
    std::vector<double> ixy;
    std::vector<double> eigen;
    std::vector<double> integral_x;
    std::vector<double> integral_y;
    std::vector<double> integral_xy;
    std::vector<std::uint8_t> suppressed;
};

/// Shi-Tomasi 角点（等价 goodFeaturesToTrack 语义：Sobel3×3 → blockSize5 矩阵和
/// → 最小特征值 → 3×3 局部极大 → 质量降序 → minDistance 贪心抑制 → 掩膜过滤）。
/// 返回行带局部坐标（按质量降序）。
/// 性能要点（2026-10-10）：临时缓冲复用（scratch）；最小特征值/候选只在
/// 掩膜包围盒 ±(half+1) 区域内计算——区域外不存在候选、区域包含全部掩膜
/// 像素，故 max_eigen/阈值/候选集合与全带计算完全一致，仅省算力。
std::vector<std::pair<float, float>> DetectCornersShiTomasi(
    const std::uint8_t* img, const std::uint8_t* mask, int width, int height,
    int stride, int block_size, double quality, int min_distance,
    int max_corners, int region_x0, int region_x1, int region_y0, int region_y1,
    CornerScratch* scratch) {
    std::vector<std::pair<float, float>> result;
    if (width < block_size + 2 || height < block_size + 2 || max_corners <= 0) {
        return result;
    }
    const int half = block_size / 2;
    const int rx0 = ClampInt(region_x0, half, width);
    const int rx1 = ClampInt(region_x1, rx0, width);
    const int ry0 = ClampInt(region_y0, half, height);
    const int ry1 = ClampInt(region_y1, ry0, height);
    if (rx0 >= rx1 || ry0 >= ry1) {
        return result;
    }

    SobelGradients(img, width, height, stride, &scratch->gx, &scratch->gy);

    const std::size_t count = static_cast<std::size_t>(width) * height;
    std::vector<double>& gx = scratch->gx;
    std::vector<double>& gy = scratch->gy;
    scratch->ixx.resize(count);
    scratch->iyy.resize(count);
    scratch->ixy.resize(count);
    std::vector<double>& ixx = scratch->ixx;
    std::vector<double>& iyy = scratch->iyy;
    std::vector<double>& ixy = scratch->ixy;
    for (std::size_t i = 0; i < count; ++i) {
        ixx[i] = gx[i] * gx[i];
        iyy[i] = gy[i] * gy[i];
        ixy[i] = gx[i] * gy[i];
    }
    const BoxSumIntegral sx(ixx, width, height, &scratch->integral_x);
    const BoxSumIntegral sy(iyy, width, height, &scratch->integral_y);
    const BoxSumIntegral sxy(ixy, width, height, &scratch->integral_xy);

    scratch->eigen.assign(count, 0.0);
    std::vector<double>& eigen = scratch->eigen;
    double max_eigen = 0.0;
    for (int y = ry0; y < ry1; ++y) {
        for (int x = rx0; x < rx1; ++x) {
            const std::size_t idx = static_cast<std::size_t>(y) * width + x;
            const double a = sx.Sum(x, y, half);
            const double c = sy.Sum(x, y, half);
            const double b = sxy.Sum(x, y, half);
            const double mid = 0.5 * (a + c);
            const double root =
                std::sqrt(std::max(0.25 * (a - c) * (a - c) + b * b, 0.0));
            eigen[idx] = mid - root;
            // 阈值基准只统计掩膜内像素（对齐 OpenCV minMaxLoc(..., mask)）
            if ((mask == nullptr || mask[idx] != 0) && eigen[idx] > max_eigen) {
                max_eigen = eigen[idx];
            }
        }
    }
    if (max_eigen <= 0.0) {
        return result;
    }
    const double threshold = quality * max_eigen;

    // 3×3 局部极大（含相等，等价 dilate+CMP_GE）+ 阈值 + 掩膜，行主序收集
    struct Candidate {
        double value;
        int x;
        int y;
    };
    std::vector<Candidate> candidates;
    const int cx0 = std::max(rx0, half + 1);
    const int cx1 = std::min(rx1, width - half - 1);
    const int cy0 = std::max(ry0, half + 1);
    const int cy1 = std::min(ry1, height - half - 1);
    for (int y = cy0; y < cy1; ++y) {
        for (int x = cx0; x < cx1; ++x) {
            const std::size_t idx = static_cast<std::size_t>(y) * width + x;
            if (mask != nullptr && mask[idx] == 0) {
                continue;
            }
            const double value = eigen[idx];
            if (value < threshold) {
                continue;
            }
            bool is_max = true;
            for (int dy = -1; dy <= 1 && is_max; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    const std::size_t n =
                        static_cast<std::size_t>(y + dy) * width + (x + dx);
                    if (eigen[n] > value) {
                        is_max = false;
                        break;
                    }
                }
            }
            if (is_max) {
                candidates.push_back({value, x, y});
            }
        }
    }
    // 质量降序（稳定排序：同值保持行主序，贴近 OpenCV 排序语义）
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const Candidate& a, const Candidate& b) {
                         return a.value > b.value;
                     });

    // minDistance 贪心抑制（矩形半径）
    const int md = std::max(min_distance, 1);
    scratch->suppressed.assign(count, 0);
    std::vector<std::uint8_t>& suppressed = scratch->suppressed;
    for (const Candidate& c : candidates) {
        const std::size_t idx = static_cast<std::size_t>(c.y) * width + c.x;
        if (suppressed[idx] != 0) {
            continue;
        }
        result.emplace_back(static_cast<float>(c.x), static_cast<float>(c.y));
        if (static_cast<int>(result.size()) >= max_corners) {
            break;
        }
        const int y0 = std::max(0, c.y - md);
        const int y1 = std::min(height - 1, c.y + md);
        const int x0 = std::max(0, c.x - md);
        const int x1 = std::min(width - 1, c.x + md);
        for (int y = y0; y <= y1; ++y) {
            std::uint8_t* row =
                suppressed.data() + static_cast<std::size_t>(y) * width;
            for (int x = x0; x <= x1; ++x) {
                row[x] = 1;
            }
        }
    }
    return result;
}

/// 网格轮转取点（对齐参考实现 detect_keypoints 的"网格分散"语义：
/// 按整幅校正图尺寸划分 grid_cells×grid_cells 网格，坐标用全局像素）。
std::vector<std::pair<float, float>> SpreadByGrid(
    const std::vector<std::pair<float, float>>& pts, int max_points,
    int grid_cells, int view_w, int view_h) {
    std::vector<std::pair<float, float>> out;
    const std::size_t limit =
        std::min<std::size_t>(pts.size(), static_cast<std::size_t>(max_points));
    if (grid_cells <= 1 || pts.size() <= limit) {
        out.assign(pts.begin(), pts.begin() + static_cast<std::ptrdiff_t>(limit));
        return out;
    }
    const double cw = static_cast<double>(view_w) / grid_cells;
    const double ch = static_cast<double>(view_h) / grid_cells;
    std::vector<std::vector<std::pair<float, float>>> buckets(
        static_cast<std::size_t>(grid_cells) * grid_cells);
    for (const auto& p : pts) {
        const int ci = ClampInt(static_cast<int>(p.second / ch), 0, grid_cells - 1);
        const int cj = ClampInt(static_cast<int>(p.first / cw), 0, grid_cells - 1);
        buckets[static_cast<std::size_t>(ci) * grid_cells + cj].push_back(p);
    }
    std::size_t idx = 0;
    bool any_left = true;
    while (out.size() < limit && any_left) {
        any_left = false;
        for (auto& bucket : buckets) {
            if (idx < bucket.size()) {
                out.push_back(bucket[idx]);
                any_left = true;
                if (out.size() >= limit) {
                    break;
                }
            }
        }
        ++idx;
    }
    return out;
}

/// 通用 Census 单像素（全列裁剪路径；win=7 的边界列与 win≠7 使用）。
/// bit 序：按 (dy, dx) 行主序跳过中心，与常量展开路径严格一致。
inline std::uint64_t CensusPixelClamped(const std::uint8_t* const* row_ptr,
                                        int win, int r, int x, int width) {
    const int center = row_ptr[r][x];
    std::uint64_t code = 0;
    int bit = 0;
    for (int i = 0; i < win; ++i) {
        const std::uint8_t* row = row_ptr[i];
        for (int j = 0; j < win; ++j) {
            if (i == r && j == r) {
                continue;
            }
            const int nx = ClampInt(x + j - r, 0, width - 1);
            code |= static_cast<std::uint64_t>(row[nx] >= center) << bit;
            ++bit;
        }
    }
    return code;
}

// win=7 内部列的 48 邻位常量展开（bit 序号为 (dy,dx) 行主序跳过中心的编号，
// 位移与列偏移均为编译期常量；与 CensusPixelClamped 位序一致）。
#define DRONE_CENSUS_BIT(row_idx, off, bit_index)                            \
    code |= static_cast<std::uint64_t>(row_ptr[row_idx][x + (off)] >= center) \
            << (bit_index)

/// Census 变换（win×win 邻域，>= 判据，跳过中心；边界复制延拓）。
/// 输出行对应全局 y ∈ [row0, row1)。
/// 性能要点（2026-10-10 板上前 42ms→ 优化）：源行指针按输出行只算一次；
/// 内部列 x ∈ [r, width−r) 无列裁剪分支（只比较、移位、或）；win=7 全展开。
std::vector<std::uint64_t> CensusTransform(const std::uint8_t* img, int width,
                                           int img_height, int stride, int win,
                                           int row0, int row1) {
    const int r = win / 2;
    const int rows = row1 - row0;
    std::vector<std::uint64_t> out(static_cast<std::size_t>(rows) * width, 0);
    const int x_lo = ClampInt(r, 0, width);
    const int x_hi = ClampInt(width - r, x_lo, width);
    for (int y = row0; y < row1; ++y) {
        const std::uint8_t* row_ptr[7];
        for (int i = 0; i < win; ++i) {
            row_ptr[i] = img + static_cast<std::size_t>(
                                   ClampInt(y + i - r, 0, img_height - 1)) * stride;
        }
        std::uint64_t* dst =
            out.data() + static_cast<std::size_t>(y - row0) * width;
        if (win == 7) {
            for (int x = x_lo; x < x_hi; ++x) {
                const int center = row_ptr[3][x];
                std::uint64_t code = 0;
                DRONE_CENSUS_BIT(0, -3, 0);  DRONE_CENSUS_BIT(0, -2, 1);
                DRONE_CENSUS_BIT(0, -1, 2);  DRONE_CENSUS_BIT(0, 0, 3);
                DRONE_CENSUS_BIT(0, 1, 4);   DRONE_CENSUS_BIT(0, 2, 5);
                DRONE_CENSUS_BIT(0, 3, 6);
                DRONE_CENSUS_BIT(1, -3, 7);  DRONE_CENSUS_BIT(1, -2, 8);
                DRONE_CENSUS_BIT(1, -1, 9);  DRONE_CENSUS_BIT(1, 0, 10);
                DRONE_CENSUS_BIT(1, 1, 11);  DRONE_CENSUS_BIT(1, 2, 12);
                DRONE_CENSUS_BIT(1, 3, 13);
                DRONE_CENSUS_BIT(2, -3, 14); DRONE_CENSUS_BIT(2, -2, 15);
                DRONE_CENSUS_BIT(2, -1, 16); DRONE_CENSUS_BIT(2, 0, 17);
                DRONE_CENSUS_BIT(2, 1, 18);  DRONE_CENSUS_BIT(2, 2, 19);
                DRONE_CENSUS_BIT(2, 3, 20);
                DRONE_CENSUS_BIT(3, -3, 21); DRONE_CENSUS_BIT(3, -2, 22);
                DRONE_CENSUS_BIT(3, -1, 23);
                DRONE_CENSUS_BIT(3, 1, 24);  DRONE_CENSUS_BIT(3, 2, 25);
                DRONE_CENSUS_BIT(3, 3, 26);
                DRONE_CENSUS_BIT(4, -3, 27); DRONE_CENSUS_BIT(4, -2, 28);
                DRONE_CENSUS_BIT(4, -1, 29); DRONE_CENSUS_BIT(4, 0, 30);
                DRONE_CENSUS_BIT(4, 1, 31);  DRONE_CENSUS_BIT(4, 2, 32);
                DRONE_CENSUS_BIT(4, 3, 33);
                DRONE_CENSUS_BIT(5, -3, 34); DRONE_CENSUS_BIT(5, -2, 35);
                DRONE_CENSUS_BIT(5, -1, 36); DRONE_CENSUS_BIT(5, 0, 37);
                DRONE_CENSUS_BIT(5, 1, 38);  DRONE_CENSUS_BIT(5, 2, 39);
                DRONE_CENSUS_BIT(5, 3, 40);
                DRONE_CENSUS_BIT(6, -3, 41); DRONE_CENSUS_BIT(6, -2, 42);
                DRONE_CENSUS_BIT(6, -1, 43); DRONE_CENSUS_BIT(6, 0, 44);
                DRONE_CENSUS_BIT(6, 1, 45);  DRONE_CENSUS_BIT(6, 2, 46);
                DRONE_CENSUS_BIT(6, 3, 47);
                dst[x] = code;
            }
            for (int x = 0; x < x_lo; ++x) {
                dst[x] = CensusPixelClamped(row_ptr, 7, 3, x, width);
            }
            for (int x = x_hi; x < width; ++x) {
                dst[x] = CensusPixelClamped(row_ptr, 7, 3, x, width);
            }
        } else {
            for (int x = 0; x < width; ++x) {
                dst[x] = CensusPixelClamped(row_ptr, win, r, x, width);
            }
        }
    }
    return out;
}

#undef DRONE_CENSUS_BIT

inline int Popcount64(std::uint64_t v) {
    return __builtin_popcountll(v);
}

}  // namespace

// ============================================================================
// 配置校验
// ============================================================================
void StereoRangerConfig::Validate() const {
    auto require = [](bool ok, const char* field, const char* reason) {
        if (!ok) {
            throw std::invalid_argument(std::string("stereo_ranger.") + field +
                                        " 非法: " + reason);
        }
    };
    require(disp_min_px > 0.0, "disp_min_px", "必须为正");
    require(disp_max_px > disp_min_px, "disp_max_px", "必须大于 disp_min_px");
    require(coarse_stride >= 1, "coarse_stride", "必须 ≥1");
    require(fine_halfwidth >= 1, "fine_halfwidth", "必须 ≥1");
    require(uniqueness_ratio >= 1.0, "uniqueness_ratio", "必须 ≥1");
    require(lr_tol_px >= 0.0, "lr_tol_px", "不得为负");
    require(census_win == 3 || census_win == 5 || census_win == 7, "census_win",
            "仅支持 3/5/7");
    require(max_points_per_target >= 1, "max_points_per_target", "必须 ≥1");
    require(grid_cells >= 1, "grid_cells", "必须 ≥1");
    require(warmup_frames >= 0, "warmup_frames", "不得为负");
    require(black_frame_mean_threshold >= 0.0, "black_frame_mean_threshold",
            "不得为负");
    require(bbox_erode_px >= 0, "bbox_erode_px", "不得为负");
    require(min_points >= 1, "min_points", "必须 ≥1");
    require(near_pct > 0.0 && near_pct <= 100.0, "near_pct", "取值 (0,100]");
    require(near_tol_px >= 0.0, "near_tol_px", "不得为负");
    require(smooth_alpha >= 0.0 && smooth_alpha <= 1.0, "smooth_alpha",
            "取值 [0,1]");
    require(input_queue_capacity >= 1, "input_queue_capacity", "必须 ≥1");
    require(frame_buffer_depth >= 2, "frame_buffer_depth", "必须 ≥2");
    require(pair_window.count() > 0, "pair_window_ms", "必须为正");
    require(settle.count() > 0, "settle_ms", "必须为正");
    require(summary_log_interval.count() > 0, "summary_log_interval_ms",
            "必须为正");
}

// ============================================================================
// 核心实现
// ============================================================================
struct StereoRangerCore::Impl {
    StereoRangerConfig config;
    StereoRectifyMatrices matrices;

    // 映射表（rect→source，float32，view_width×view_height）
    std::vector<float> map_ref_x;
    std::vector<float> map_ref_y;
    std::vector<float> map_aux_x;
    std::vector<float> map_aux_y;

    // EMA 状态（键：类别 + 原始框 40px 粗格）
    std::unordered_map<std::uint64_t, double> ema;

    // 测试注入关键点（校正图坐标；非空时跳过 Shi-Tomasi）
    std::vector<std::pair<float, float>> keypoints_override;

    // 复用缓冲（避免每帧分配）
    std::vector<std::uint8_t> band_ref;
    std::vector<std::uint8_t> band_aux;
    std::vector<std::uint8_t> band_mask;
    std::vector<std::uint64_t> census_ref;
    std::vector<std::uint64_t> census_aux;
    std::vector<std::uint64_t> ref_window;  // N × win²
    std::vector<int> win_row;               // N × win²（census 行，带内局部）
    std::vector<int> win_col;               // N × win²（参考列，已裁剪）

    // Shi-Tomasi 复用缓冲（避免每帧分配 MB 级数组）
    CornerScratch corner_scratch;

    // 测试接口用的逐点快照（生产路径只写入，供 LastPointDebugForTest 读取）
    std::vector<StereoRangerCore::PointDebug> last_points;

    // 统计
    std::uint64_t processed_frames = 0;
    std::uint64_t matched_frames = 0;
    std::uint64_t available_count = 0;
    std::uint64_t clipped_count = 0;
    std::uint64_t skipped_frames = 0;
    double last_match_ms = 0.0;
    double last_total_ms = 0.0;

    // 阶段耗时滚动窗口（每帧 Add 5 次：互斥+存值 ≈ 百纳秒级，远小于帧耗时；
    // 摘要时才 Snapshot 复制排序，符合"热路径不分配"约定）。
    common::LatencyStatistics stage_remap{512};
    common::LatencyStatistics stage_keypoints{512};
    common::LatencyStatistics stage_census{512};
    common::LatencyStatistics stage_match{512};
    common::LatencyStatistics stage_aggregate{512};
};

// ---- 构造 / 销毁 ------------------------------------------------------------

StereoRangerCore::StereoRangerCore(StereoRangerConfig config,
                                   StereoRectifyMatrices matrices)
    : impl_(std::make_unique<Impl>()) {
    config.Validate();
    // 矩阵自洽校验（config 加载路径已校验，此处兜底防结构体直构误用）：
    // view 尺寸、焦距、基线必须为正，fxb 必须等于 fx_rect × baseline。
    if (matrices.view_width <= 0 || matrices.view_height <= 0 ||
        !(matrices.k_ref[0] > 0.0) || !(matrices.k_aux[0] > 0.0) ||
        !(matrices.p_ref[0] > 0.0) || !(matrices.p_ref[5] > 0.0) ||
        !(matrices.baseline_m > 0.0)) {
        throw std::invalid_argument(
            "双目测距: 校正矩阵 view/焦距/基线非法（检查 export_rectify.py 产物）");
    }
    const double expected_fxb = matrices.p_ref[0] * matrices.baseline_m;
    if (std::abs(matrices.fx_rect_px - matrices.p_ref[0]) >
            1e-6 * std::abs(matrices.p_ref[0]) ||
        std::abs(matrices.fxb_m_px - expected_fxb) >
            1e-6 * std::abs(expected_fxb)) {
        throw std::invalid_argument(
            "双目测距: 校正矩阵 fx_rect/fxb 与 P 不自洽（重跑 export_rectify.py）");
    }
    impl_->config = std::move(config);
    impl_->matrices = matrices;
    const int w = impl_->matrices.view_width;
    const int h = impl_->matrices.view_height;
    BuildRectToSourceMap(impl_->matrices.k_ref, impl_->matrices.d_ref,
                         impl_->matrices.r_ref, impl_->matrices.p_ref, w, h,
                         &impl_->map_ref_x, &impl_->map_ref_y);
    BuildRectToSourceMap(impl_->matrices.k_aux, impl_->matrices.d_aux,
                         impl_->matrices.r_aux, impl_->matrices.p_aux, w, h,
                         &impl_->map_aux_x, &impl_->map_aux_y);
}

StereoRangerCore::~StereoRangerCore() = default;

void StereoRangerCore::SetKeypointsOverride(
    std::vector<std::pair<float, float>> rect_points) {
    impl_->keypoints_override = std::move(rect_points);
}

std::vector<std::uint64_t> StereoRangerCore::CensusEncodeForTest(
    const std::uint8_t* data, int width, int height, int stride, int win) {
    return CensusTransform(data, width, height, stride, win, 0, height);
}

std::vector<StereoRangerCore::PointDebug>
StereoRangerCore::LastPointDebugForTest() const {
    return impl_->last_points;
}

bool StereoRangerCore::RectToSourceRef(float u_rect, float v_rect, float* u_src,
                                       float* v_src) const {
    const int w = impl_->matrices.view_width;
    const int h = impl_->matrices.view_height;
    const int x = static_cast<int>(std::lround(u_rect));
    const int y = static_cast<int>(std::lround(v_rect));
    if (x < 0 || x >= w || y < 0 || y >= h) {
        return false;
    }
    const std::size_t idx = static_cast<std::size_t>(y) * w + x;
    *u_src = impl_->map_ref_x[idx];
    *v_src = impl_->map_ref_y[idx];
    return true;
}

bool StereoRangerCore::SourceToRectRef(double u_src, double v_src,
                                       double* u_rect, double* v_rect) const {
    SourceToRectPoint(impl_->matrices.k_ref, impl_->matrices.d_ref,
                      impl_->matrices.r_ref, impl_->matrices.p_ref, u_src,
                      v_src, u_rect, v_rect);
    return std::isfinite(*u_rect) && std::isfinite(*v_rect);
}

std::uint64_t StereoRangerCore::ProcessedFrameCount() const {
    return impl_->processed_frames;
}
std::uint64_t StereoRangerCore::MatchedFrameCount() const {
    return impl_->matched_frames;
}
std::uint64_t StereoRangerCore::AvailableCount() const {
    return impl_->available_count;
}
std::uint64_t StereoRangerCore::ClippedCount() const {
    return impl_->clipped_count;
}
std::uint64_t StereoRangerCore::SkippedFrameCount() const {
    return impl_->skipped_frames;
}
double StereoRangerCore::LastMatchMs() const { return impl_->last_match_ms; }
double StereoRangerCore::LastTotalMs() const { return impl_->last_total_ms; }

StereoStageLatency StereoRangerCore::StageLatency() const {
    StereoStageLatency stage;
    stage.remap = impl_->stage_remap.Snapshot();
    stage.keypoints = impl_->stage_keypoints.Snapshot();
    stage.census = impl_->stage_census.Snapshot();
    stage.match = impl_->stage_match.Snapshot();
    stage.aggregate = impl_->stage_aggregate.Snapshot();
    return stage;
}

void StereoRangerCore::Reset() {
    impl_->ema.clear();
    impl_->last_match_ms = 0.0;
    impl_->last_total_ms = 0.0;
}

// ---- 主流程 -----------------------------------------------------------------

std::vector<StereoRangeResult> StereoRangerCore::Process(
    const GrayImageView& ref, const GrayImageView& aux,
    const std::vector<common::DetectionResult>& boxes) {
    const StereoRangerConfig& cfg = impl_->config;
    const StereoRectifyMatrices& mat = impl_->matrices;
    const int view_w = mat.view_width;
    const int view_h = mat.view_height;

    const auto t_total0 = std::chrono::steady_clock::now();
    ++impl_->processed_frames;

    std::vector<StereoRangeResult> results(boxes.size());

    // 输入合法性（尺寸/stride）；非法帧按跳过处理，不抛异常
    const bool input_ok = ref.Valid() && aux.Valid() &&
                          ref.width == view_w && ref.height == view_h &&
                          aux.width == view_w && aux.height == view_h;
    if (!input_ok ||
        impl_->processed_frames <= static_cast<std::uint64_t>(cfg.warmup_frames)) {
        ++impl_->skipped_frames;
        return results;
    }

    // ---- 1. 检测框：原始左图坐标 → 校正图坐标（8 点包围盒）----
    struct RectBox {
        double x0 = 0.0;
        double y0 = 0.0;
        double x1 = 0.0;
        double y1 = 0.0;
        bool valid = false;
    };
    std::vector<RectBox> rect_boxes(boxes.size());
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        const common::DetectionResult& b = boxes[i];
        const double x0 = b.bbox_x;
        const double y0 = b.bbox_y;
        const double x1 = b.bbox_x + b.bbox_w;
        const double y1 = b.bbox_y + b.bbox_h;
        const double probes[8][2] = {
            {x0, y0},
            {x1, y0},
            {x1, y1},
            {x0, y1},
            {0.5 * (x0 + x1), y0},
            {x1, 0.5 * (y0 + y1)},
            {0.5 * (x0 + x1), y1},
            {x0, 0.5 * (y0 + y1)}};
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();
        bool ok = true;
        for (const auto& c : probes) {
            double ru = 0.0;
            double rv = 0.0;
            SourceToRectPoint(mat.k_ref, mat.d_ref, mat.r_ref, mat.p_ref, c[0],
                              c[1], &ru, &rv);
            if (!std::isfinite(ru) || !std::isfinite(rv)) {
                ok = false;
                break;
            }
            min_x = std::min(min_x, ru);
            min_y = std::min(min_y, rv);
            max_x = std::max(max_x, ru);
            max_y = std::max(max_y, rv);
        }
        if (!ok) {
            continue;
        }
        // 向外取整并裁剪到画面（映射后框略放大，避免关键点/簇点被裁掉）
        RectBox rb;
        rb.x0 = static_cast<double>(
            ClampInt(static_cast<int>(std::floor(min_x)) - 1, 0, view_w));
        rb.y0 = static_cast<double>(
            ClampInt(static_cast<int>(std::floor(min_y)) - 1, 0, view_h));
        rb.x1 = static_cast<double>(
            ClampInt(static_cast<int>(std::ceil(max_x)) + 1, 0, view_w));
        rb.y1 = static_cast<double>(
            ClampInt(static_cast<int>(std::ceil(max_y)) + 1, 0, view_h));
        rb.valid = rb.x1 - rb.x0 >= 2.0 && rb.y1 - rb.y0 >= 2.0;
        rect_boxes[i] = rb;
    }

    // ---- 2. 联合包围盒 → 行带 ----
    const auto t_remap0 = std::chrono::steady_clock::now();
    double ux0 = std::numeric_limits<double>::max();
    double uy0 = std::numeric_limits<double>::max();
    double ux1 = std::numeric_limits<double>::lowest();
    double uy1 = std::numeric_limits<double>::lowest();
    bool any_box = false;
    for (const RectBox& rb : rect_boxes) {
        if (!rb.valid) {
            continue;
        }
        ux0 = std::min(ux0, rb.x0);
        uy0 = std::min(uy0, rb.y0);
        ux1 = std::max(ux1, rb.x1);
        uy1 = std::max(uy1, rb.y1);
        any_box = true;
    }
    if (!any_box) {
        return results;
    }
    // 行带余量 = 采样余量 + Census 半径（census 直接在已校正行带上算，
    // 保证任何关键点的窗口都在带内，带外只在画面边缘发生复制延拓）。
    const int cr = cfg.census_win / 2;
    const int band_y0 = ClampInt(
        static_cast<int>(std::floor(uy0)) - kBandMargin - cr - 1, 0, view_h - 1);
    const int band_y1 = ClampInt(
        static_cast<int>(std::ceil(uy1)) + kBandMargin + cr + 1, 1, view_h);
    const int band_h = band_y1 - band_y0;
    if (band_h <= 0) {
        return results;
    }

    // ---- 2.5 列裁剪（与全幅逐像素等价的推导）----
    // 窗口与代价只访问以下列：参考窗口 = 关键点 ±wr（关键点必在框内）；
    // 目标窗口 = 参考列 − ceil(disp_max)；census 邻域 = 上述列再 ±r。
    // 取 [max(0, x0 − 2r − ceil(disp_max)), min(W, x1 + 2r)) 即覆盖全部访问，
    // 且任一被访问列的 census 邻域要么真实落在带内、要么与全幅在同一画面边缘
    // 处裁剪，故列裁剪不改变任何结果（仅省去带宽外的算力）。
    const int max_shift = static_cast<int>(std::ceil(cfg.disp_max_px));
    int box_x0 = view_w;
    int box_x1 = 0;
    for (const RectBox& rb : rect_boxes) {
        if (!rb.valid) {
            continue;
        }
        box_x0 = std::min(box_x0, static_cast<int>(std::lround(rb.x0)));
        box_x1 = std::max(box_x1, static_cast<int>(std::lround(rb.x1)));
    }
    const int crop_x0 = ClampInt(box_x0 - 2 * cr - max_shift, 0, view_w - 1);
    const int crop_x1 = ClampInt(box_x1 + 2 * cr, 0, view_w);
    const int crop_w = crop_x1 - crop_x0;
    if (crop_w <= 0) {
        return results;
    }

    // ---- 3. 行带 remap（基准/辅助，双线性；列裁剪范围内）+ 黑帧检查 ----
    impl_->band_ref.resize(static_cast<std::size_t>(band_h) * crop_w);
    impl_->band_aux.resize(static_cast<std::size_t>(band_h) * crop_w);
    double mean_sum = 0.0;
    for (int ly = 0; ly < band_h; ++ly) {
        const int y = band_y0 + ly;
        const float* mxr =
            impl_->map_ref_x.data() + static_cast<std::size_t>(y) * view_w;
        const float* myr =
            impl_->map_ref_y.data() + static_cast<std::size_t>(y) * view_w;
        const float* mxa =
            impl_->map_aux_x.data() + static_cast<std::size_t>(y) * view_w;
        const float* mya =
            impl_->map_aux_y.data() + static_cast<std::size_t>(y) * view_w;
        std::uint8_t* dr =
            impl_->band_ref.data() + static_cast<std::size_t>(ly) * crop_w;
        std::uint8_t* da =
            impl_->band_aux.data() + static_cast<std::size_t>(ly) * crop_w;
        for (int x = crop_x0; x < crop_x1; ++x) {
            const int lx = x - crop_x0;
            dr[lx] = static_cast<std::uint8_t>(std::lround(SampleBilinearFast(
                ref.data, ref.width, ref.height, ref.stride, mxr[x], myr[x])));
            da[lx] = static_cast<std::uint8_t>(std::lround(SampleBilinearFast(
                aux.data, aux.width, aux.height, aux.stride, mxa[x], mya[x])));
            mean_sum += dr[lx];
        }
    }
    const double band_mean = mean_sum / (static_cast<double>(band_h) * crop_w);
    impl_->stage_remap.Add(
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_remap0).count());
    if (band_mean < cfg.black_frame_mean_threshold) {
        ++impl_->skipped_frames;
        return results;
    }

    // ---- 4. 联合掩膜（带内局部坐标：行 −band_y0，列 −crop_x0）----
    impl_->band_mask.assign(static_cast<std::size_t>(band_h) * crop_w, 0);
    int mask_x0 = crop_w;
    int mask_x1 = 0;
    int mask_y0 = band_h;
    int mask_y1 = 0;
    for (const RectBox& rb : rect_boxes) {
        if (!rb.valid) {
            continue;
        }
        const int x0 = ClampInt(static_cast<int>(std::lround(rb.x0)) - crop_x0, 0, crop_w);
        const int x1 = ClampInt(static_cast<int>(std::lround(rb.x1)) - crop_x0, 0, crop_w);
        const int y0 =
            ClampInt(static_cast<int>(std::lround(rb.y0)) - band_y0, 0, band_h);
        const int y1 =
            ClampInt(static_cast<int>(std::lround(rb.y1)) - band_y0, 0, band_h);
        if (x0 < x1 && y0 < y1) {
            mask_x0 = std::min(mask_x0, x0);
            mask_x1 = std::max(mask_x1, x1);
            mask_y0 = std::min(mask_y0, y0);
            mask_y1 = std::max(mask_y1, y1);
        }
        for (int ly = y0; ly < y1; ++ly) {
            std::uint8_t* row =
                impl_->band_mask.data() + static_cast<std::size_t>(ly) * crop_w;
            for (int x = x0; x < x1; ++x) {
                row[x] = 1;
            }
        }
    }

    // ---- 5. 关键点（注入优先；否则 Shi-Tomasi + 网格分散）----
    const auto t_kp0 = std::chrono::steady_clock::now();
    std::vector<std::pair<float, float>> keypoints;  // 校正图全局坐标
    if (!impl_->keypoints_override.empty()) {
        keypoints = impl_->keypoints_override;
    } else {
        int box_count = 0;
        for (const RectBox& rb : rect_boxes) {
            box_count += rb.valid ? 1 : 0;
        }
        const int max_corners =
            std::max(cfg.max_points_per_target * std::max(box_count, 1) * 4, 16);
        const auto local = DetectCornersShiTomasi(
            impl_->band_ref.data(), impl_->band_mask.data(), crop_w, band_h,
            crop_w, /*block_size=*/5, /*quality=*/0.01, /*min_distance=*/8,
            max_corners,
            /*region=*/mask_x0, mask_x1, mask_y0, mask_y1,
            &impl_->corner_scratch);
        std::vector<std::pair<float, float>> global;
        global.reserve(local.size());
        for (const auto& p : local) {
            global.emplace_back(p.first + static_cast<float>(crop_x0),
                                p.second + static_cast<float>(band_y0));
        }
        keypoints = SpreadByGrid(global,
                                 cfg.max_points_per_target * std::max(box_count, 1),
                                 cfg.grid_cells, view_w, view_h);
    }

    const int n = static_cast<int>(keypoints.size());
    impl_->stage_keypoints.Add(
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_kp0).count());
    std::vector<double> disp(keypoints.size(), kNaN);
    std::vector<bool> point_valid(keypoints.size(), false);

    if (n > 0) {
        // ---- 6. Census（直接在已校正行带上计算；行/列越界按复制延拓裁剪）----
        // 列范围为裁剪区间 [crop_x0, crop_x1)：被访问列的邻域或真实在带内、
        // 或与全幅在同一画面边缘处裁剪（见 §2.5 推导），结果与全幅一致。
        const auto t_census0 = std::chrono::steady_clock::now();
        impl_->census_ref = CensusTransform(impl_->band_ref.data(), crop_w, band_h,
                                            crop_w, cfg.census_win, 0, band_h);
        impl_->census_aux = CensusTransform(impl_->band_aux.data(), crop_w, band_h,
                                            crop_w, cfg.census_win, 0, band_h);
        impl_->stage_census.Add(
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_census0).count());

        const int k = cfg.census_win * cfg.census_win;
        const int wr = cr;
        impl_->ref_window.resize(static_cast<std::size_t>(n) * k);
        impl_->win_row.resize(static_cast<std::size_t>(n) * k);
        impl_->win_col.resize(static_cast<std::size_t>(n) * k);
        for (int i = 0; i < n; ++i) {
            const int px = ClampInt(RintToInt(keypoints[i].first), crop_x0, crop_x1 - 1);
            const int py = ClampInt(RintToInt(keypoints[i].second) - band_y0, 0, band_h - 1);
            std::uint64_t* win =
                impl_->ref_window.data() + static_cast<std::size_t>(i) * k;
            int* bases = impl_->win_row.data() + static_cast<std::size_t>(i) * k;
            int* cols = impl_->win_col.data() + static_cast<std::size_t>(i) * k;
            int slot = 0;
            for (int oy = -wr; oy <= wr; ++oy) {
                const int ny = ClampInt(py + oy, 0, band_h - 1);
                // base = 行索引起点 − crop_x0，与该列的全局号相加即得带内索引
                const int base = ny * crop_w - crop_x0;
                for (int ox = -wr; ox <= wr; ++ox) {
                    const int nx = ClampInt(px + ox, crop_x0, crop_x1 - 1);
                    bases[slot] = base;
                    cols[slot] = nx;
                    win[slot] = impl_->census_ref[base + nx];
                    ++slot;
                }
            }
        }

        // 目标图代价：列平移 di、行不变、二次裁剪（对齐参考实现；裁剪下界换成
        // 列裁剪区间——由 §2.5 推导，被访问列恒在该区间内，故与全幅语义一致）
        auto target_costs = [&](const int* disps, int count,
                                const std::uint64_t* target_census,
                                std::vector<double>* out) {
            out->assign(static_cast<std::size_t>(n) * count, 0.0);
            for (int c = 0; c < count; ++c) {
                const int di = disps[c];
                for (int i = 0; i < n; ++i) {
                    const std::uint64_t* win =
                        impl_->ref_window.data() + static_cast<std::size_t>(i) * k;
                    const int* bases =
                        impl_->win_row.data() + static_cast<std::size_t>(i) * k;
                    const int* cols =
                        impl_->win_col.data() + static_cast<std::size_t>(i) * k;
                    int sum = 0;
                    for (int slot = 0; slot < k; ++slot) {
                        const int nx = ClampInt(cols[slot] - di, crop_x0, crop_x1 - 1);
                        const std::uint64_t t = target_census[bases[slot] + nx];
                        sum += Popcount64(win[slot] ^ t);
                    }
                    (*out)[static_cast<std::size_t>(i) * count + c] =
                        static_cast<double>(sum);
                }
            }
        };

        // ---- 7. 粗搜索（stride）----
        const auto t_match0 = std::chrono::steady_clock::now();
        const double d0 = cfg.disp_min_px;
        const double d1 = cfg.disp_max_px;
        std::vector<double> coarse;
        for (double d = d0; d <= d1 + 1e-6; d += cfg.coarse_stride) {
            coarse.push_back(d);
        }
        if (coarse.size() < 3) {
            coarse.clear();
            for (int i = 0; i < 9; ++i) {
                coarse.push_back(d0 + (d1 - d0) * i / 8.0);
            }
        }
        std::vector<int> coarse_rounded(coarse.size());
        for (std::size_t c = 0; c < coarse.size(); ++c) {
            coarse_rounded[c] = RintToInt(coarse[c]);
        }
        std::vector<double> coarse_cost;
        target_costs(coarse_rounded.data(), static_cast<int>(coarse.size()),
                     impl_->census_aux.data(), &coarse_cost);
        std::vector<double> d_coarse(n);
        for (int i = 0; i < n; ++i) {
            int best = 0;
            double best_cost =
                coarse_cost[static_cast<std::size_t>(i) * coarse.size()];
            for (std::size_t c = 1; c < coarse.size(); ++c) {
                const double cost =
                    coarse_cost[static_cast<std::size_t>(i) * coarse.size() + c];
                if (cost < best_cost) {  // 严格小于：并列取先出现（同 np.argmin）
                    best_cost = cost;
                    best = static_cast<int>(c);
                }
            }
            d_coarse[i] = coarse[best];
        }

        // ---- 8. 精搜索（±halfwidth，step=1）+ 抛物线亚像素 + 唯一性 ----
        const int fine_n = 2 * cfg.fine_halfwidth + 1;
        std::vector<double> fine_disp(static_cast<std::size_t>(n) * fine_n, 0.0);
        std::vector<int> fine_rounded(static_cast<std::size_t>(n) * fine_n, 0);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < fine_n; ++j) {
                const double cand = std::max(
                    d0, std::min(d1, d_coarse[i] + (j - cfg.fine_halfwidth)));
                fine_disp[static_cast<std::size_t>(i) * fine_n + j] = cand;
                fine_rounded[static_cast<std::size_t>(i) * fine_n + j] =
                    RintToInt(cand);
            }
        }
        std::vector<double> fine_cost(static_cast<std::size_t>(n) * fine_n, 0.0);
        {
            std::vector<int> uniq(fine_rounded.begin(), fine_rounded.end());
            std::sort(uniq.begin(), uniq.end());
            uniq.erase(std::unique(uniq.begin(), uniq.end()), uniq.end());
            std::vector<double> uniq_cost;
            target_costs(uniq.data(), static_cast<int>(uniq.size()),
                         impl_->census_aux.data(), &uniq_cost);
            for (std::size_t p = 0; p < fine_rounded.size(); ++p) {
                const std::size_t pos = static_cast<std::size_t>(
                    std::lower_bound(uniq.begin(), uniq.end(),
                                     fine_rounded[p]) - uniq.begin());
                fine_cost[p] = uniq_cost[
                    static_cast<std::size_t>(p / fine_n) * uniq.size() + pos];
            }
        }
        std::vector<double> ratio(keypoints.size(), 0.0);
        for (int i = 0; i < n; ++i) {
            const double* fc = fine_cost.data() + static_cast<std::size_t>(i) * fine_n;
            int best = 0;
            for (int j = 1; j < fine_n; ++j) {
                if (fc[j] < fc[best]) {
                    best = j;
                }
            }
            const int km = ClampInt(best - 1, 0, fine_n - 1);
            const int kp = ClampInt(best + 1, 0, fine_n - 1);
            const double denom = fc[km] - 2.0 * fc[best] + fc[kp];
            double sub = 0.0;
            if (std::abs(denom) > 1e-9) {
                sub = 0.5 * (fc[km] - fc[kp]) / denom;
            }
            sub = std::max(-1.0, std::min(1.0, sub));
            disp[i] = std::max(
                d0, std::min(d1, fine_disp[static_cast<std::size_t>(i) * fine_n + best] + sub));
            // 唯一性：峰位 ±2 之外的次优代价 / 最优代价
            double second = std::numeric_limits<double>::infinity();
            for (int j = 0; j < fine_n; ++j) {
                if (std::abs(j - best) <= 2) {
                    continue;
                }
                second = std::min(second, fc[j]);
            }
            ratio[i] = second > 0.0
                           ? second / std::max(fc[best], 1e-6)
                           : std::numeric_limits<double>::infinity();
        }

        // ---- 9. 左右一致性（回搜索 x_L = x_R + d）----
        std::vector<bool> lr_ok(keypoints.size(), true);
        std::vector<double> lr_best_back(keypoints.size(), kNaN);
        if (cfg.lr_tol_px > 0.0) {
            std::vector<std::uint64_t> back_window(static_cast<std::size_t>(n) * k);
            std::vector<int> back_row(static_cast<std::size_t>(n) * k);
            std::vector<int> back_col(static_cast<std::size_t>(n) * k);
            for (int i = 0; i < n; ++i) {
                const int xr = ClampInt(
                    RintToInt(keypoints[i].first) - RintToInt(disp[i]), crop_x0,
                    crop_x1 - 1);
                const int py = ClampInt(
                    RintToInt(keypoints[i].second) - band_y0, 0, band_h - 1);
                std::uint64_t* win =
                    back_window.data() + static_cast<std::size_t>(i) * k;
                int* bases = back_row.data() + static_cast<std::size_t>(i) * k;
                int* cols = back_col.data() + static_cast<std::size_t>(i) * k;
                int slot = 0;
                for (int oy = -wr; oy <= wr; ++oy) {
                    const int ny = ClampInt(py + oy, 0, band_h - 1);
                    const int base = ny * crop_w - crop_x0;
                    for (int ox = -wr; ox <= wr; ++ox) {
                        const int nx = ClampInt(xr + ox, crop_x0, crop_x1 - 1);
                        bases[slot] = base;
                        cols[slot] = nx;
                        win[slot] = impl_->census_aux[base + nx];
                        ++slot;
                    }
                }
            }
            const int hr = static_cast<int>(std::ceil(cfg.lr_tol_px)) + 2;
            const int back_n = 2 * hr + 1;
            for (int i = 0; i < n; ++i) {
                const std::uint64_t* win =
                    back_window.data() + static_cast<std::size_t>(i) * k;
                const int* bases =
                    back_row.data() + static_cast<std::size_t>(i) * k;
                const int* cols =
                    back_col.data() + static_cast<std::size_t>(i) * k;
                double best_cost = std::numeric_limits<double>::infinity();
                double best_back = kNaN;
                for (int j = 0; j < back_n; ++j) {
                    const double cand =
                        std::max(d0, std::min(d1, disp[i] + (j - hr)));
                    // 语义加号：引擎内做 x - d，传 -d 即得 x_R + d
                    const int di = -RintToInt(cand);
                    int sum = 0;
                    for (int slot = 0; slot < k; ++slot) {
                        const int nx = ClampInt(cols[slot] - di, crop_x0, crop_x1 - 1);
                        const std::uint64_t t = impl_->census_ref[bases[slot] + nx];
                        sum += Popcount64(win[slot] ^ t);
                    }
                    if (static_cast<double>(sum) < best_cost) {
                        best_cost = static_cast<double>(sum);
                        best_back = cand;
                    }
                }
                lr_best_back[i] = best_back;
                lr_ok[i] = std::isfinite(best_back) &&
                           std::abs(disp[i] - best_back) <= cfg.lr_tol_px;
            }
        }

        for (int i = 0; i < n; ++i) {
            point_valid[i] = std::isfinite(disp[i]) &&
                             ratio[i] >= cfg.uniqueness_ratio && lr_ok[i];
        }
        impl_->last_points.resize(keypoints.size());
        for (int i = 0; i < n; ++i) {
            StereoRangerCore::PointDebug debug;
            debug.x = keypoints[i].first;
            debug.y = keypoints[i].second;
            debug.disparity = static_cast<float>(disp[i]);
            debug.uniqueness_ratio = static_cast<float>(ratio[i]);
            debug.lr_ok = lr_ok[i];
            debug.lr_best_back = static_cast<float>(lr_best_back[i]);
            debug.valid = point_valid[i];
            impl_->last_points[static_cast<std::size_t>(i)] = debug;
        }
        impl_->last_match_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_match0).count();
        impl_->stage_match.Add(impl_->last_match_ms);
    }
    ++impl_->matched_frames;

    // ---- 10. 逐框近端簇聚合 + EMA ----
    const auto t_agg0 = std::chrono::steady_clock::now();
    const double fxb = mat.fxb_m_px;
    const double fx = mat.p_ref[0];
    const double fy = mat.p_ref[5];
    const double cx = mat.p_ref[2];
    const double cy = mat.p_ref[6];
    const int erode = cfg.bbox_erode_px;

    for (std::size_t bi = 0; bi < boxes.size(); ++bi) {
        StereoRangeResult& out = results[bi];
        const RectBox& rb = rect_boxes[bi];
        if (!rb.valid) {
            continue;
        }
        // 腐蚀：左上 +erode、右下 -erode，裁剪到画面（对齐参考实现）
        const int x1 = ClampInt(RintToInt(rb.x0) + erode, 0, view_w);
        const int x2 = ClampInt(RintToInt(rb.x1) - erode, 0, view_w);
        const int y1c = ClampInt(RintToInt(rb.y0) + erode, 0, view_h);
        const int y2c = ClampInt(RintToInt(rb.y1) - erode, 0, view_h);
        if (x2 - x1 < 8 || y2c - y1c < 8) {
            continue;
        }
        std::vector<double> inside;
        inside.reserve(keypoints.size());
        for (std::size_t i = 0; i < keypoints.size(); ++i) {
            if (!point_valid[i]) {
                continue;
            }
            const double px = static_cast<double>(keypoints[i].first);
            const double py = static_cast<double>(keypoints[i].second);
            if (px >= static_cast<double>(x1) && px < static_cast<double>(x2) &&
                py >= static_cast<double>(y1c) &&
                py < static_cast<double>(y2c)) {
                inside.push_back(disp[i]);
            }
        }
        if (static_cast<int>(inside.size()) < cfg.min_points) {
            continue;
        }
        std::sort(inside.begin(), inside.end());
        const double d_near = PercentileSorted(inside, cfg.near_pct);
        std::vector<double> cluster;
        cluster.reserve(inside.size());
        for (double d : inside) {
            if (d >= d_near - cfg.near_tol_px) {
                cluster.push_back(d);
            }
        }
        if (static_cast<int>(cluster.size()) < cfg.min_points) {
            cluster = inside;  // 近端簇太小：目标可能充满 bbox，退回全体
        }
        const double d_med = MedianSorted(cluster);
        if (!(d_med > 0.0)) {
            continue;
        }
        const double z_raw = fxb / d_med;
        const double u = 0.5 * (static_cast<double>(x1) + static_cast<double>(x2));
        const double v = 0.5 * (static_cast<double>(y1c) + static_cast<double>(y2c));
        const double x_raw = (u - cx) * z_raw / fx;
        const double y_raw = (v - cy) * z_raw / fy;
        const double iqr = PercentileSorted(cluster, 75.0) -
                           PercentileSorted(cluster, 25.0);

        // EMA（键：类别 + 原始框 x1/y1 的 40px 粗格；截断取整同参考实现）
        double z = z_raw;
        if (cfg.smooth_alpha > 0.0) {
            const int gx = static_cast<int>(boxes[bi].bbox_x) / 40;
            const int gy = static_cast<int>(boxes[bi].bbox_y) / 40;
            const std::uint64_t key =
                (static_cast<std::uint64_t>(boxes[bi].class_id & 0xFFu) << 40) ^
                (static_cast<std::uint64_t>(gx & 0xFFFFF) << 20) ^
                static_cast<std::uint64_t>(gy & 0xFFFFF);
            auto it = impl_->ema.find(key);
            if (it == impl_->ema.end()) {
                impl_->ema.emplace(key, z_raw);
            } else {
                z = cfg.smooth_alpha * z_raw + (1.0 - cfg.smooth_alpha) * it->second;
                it->second = z;
            }
        }
        const double scale = z_raw > 0.0 ? z / z_raw : 1.0;
        const double x = x_raw * scale;
        const double y = y_raw * scale;

        out.available = true;
        out.clipped = d_med >= cfg.disp_max_px - 2.0;
        out.z_m = z;
        out.x_m = x;
        out.y_m = y;
        out.euclid_m = std::sqrt(x * x + y * y + z * z);
        out.disparity_px = d_med;
        out.iqr_px = iqr;
        out.cluster_points = static_cast<int>(cluster.size());
        out.center_x_rect = u;
        out.center_y_rect = v;
        ++impl_->available_count;
        if (out.clipped) {
            ++impl_->clipped_count;
        }
    }

    const auto t_total1 = std::chrono::steady_clock::now();
    impl_->last_total_ms =
        std::chrono::duration<double, std::milli>(t_total1 - t_total0).count();
    impl_->stage_aggregate.Add(std::chrono::duration<double, std::milli>(
        t_total1 - t_agg0).count());
    return results;
}

}  // namespace drone::perception
