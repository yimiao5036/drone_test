/**
 * @file stereo_rectifier.cpp
 * @brief 立体校正数学与 remap 实现
 */
#include "stereo_depth/stereo_rectifier.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace stereo_depth {
namespace {

using Vec3 = std::array<double, 3>;

double Norm(const Vec3& v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

Vec3 Normalized(const Vec3& v) {
    double n = Norm(v);
    if (n < 1e-12) return {0.0, 0.0, 0.0};
    return {v[0] / n, v[1] / n, v[2] / n};
}

Vec3 Cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

/// C = A·B（3×3 行主序）。
Matrix3 Mul(const Matrix3& A, const Matrix3& B) {
    Matrix3 C{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) {
                s += A[static_cast<std::size_t>(r * 3 + k)] *
                     B[static_cast<std::size_t>(k * 3 + c)];
            }
            C[static_cast<std::size_t>(r * 3 + c)] = s;
        }
    }
    return C;
}

Matrix3 Transpose(const Matrix3& A) {
    Matrix3 T{};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            T[static_cast<std::size_t>(r * 3 + c)] = A[static_cast<std::size_t>(c * 3 + r)];
    return T;
}

/// y = M·v。
Vec3 MulVec(const Matrix3& M, const Vec3& v) {
    Vec3 y{};
    for (int r = 0; r < 3; ++r) {
        y[static_cast<std::size_t>(r)] =
            M[static_cast<std::size_t>(r * 3 + 0)] * v[0] +
            M[static_cast<std::size_t>(r * 3 + 1)] * v[1] +
            M[static_cast<std::size_t>(r * 3 + 2)] * v[2];
    }
    return y;
}

/// 施加 5 项畸变（dist=[k1,k2,p1,p2,k3]）：归一化坐标 (x,y) → (xd,yd)。
void ApplyDistortion(const std::array<double, 5>& d, double x, double y,
                     double* xd, double* yd) {
    const double k1 = d[0], k2 = d[1], p1 = d[2], p2 = d[3], k3 = d[4];
    double r2 = x * x + y * y;
    double radial = 1.0 + ((k3 * r2 + k2) * r2 + k1) * r2;
    *xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    *yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
}

/// 畸变逆解（定点迭代，同 OpenCV undistortPoints）：畸变归一化 (xd,yd) → (x,y)。
void RemoveDistortion(const std::array<double, 5>& d, double xd, double yd,
                      double* x, double* y) {
    const double k1 = d[0], k2 = d[1], p1 = d[2], p2 = d[3], k3 = d[4];
    double xx = xd, yy = yd;
    for (int i = 0; i < 10; ++i) {
        double r2 = xx * xx + yy * yy;
        double icdist = 1.0 / (1.0 + ((k3 * r2 + k2) * r2 + k1) * r2);
        double dx = 2.0 * p1 * xx * yy + p2 * (r2 + 2.0 * xx * xx);
        double dy = p1 * (r2 + 2.0 * yy * yy) + 2.0 * p2 * xx * yy;
        xx = (xd - dx) * icdist;
        yy = (yd - dy) * icdist;
    }
    *x = xx;
    *y = yy;
}

}  // namespace

void StereoRectifier::Build(const StereoCalibration& cal, std::uint32_t rect_width,
                            std::uint32_t rect_height) {
    if (!cal.Valid()) throw std::invalid_argument("标定数据不完整，无法构建校正");
    if (rect_width == 0 || rect_height == 0)
        throw std::invalid_argument("rect_width/height 必须为正");

    baseline_ = cal.baseline_m;
    const Vec3 T = {cal.T_m[0], cal.T_m[1], cal.T_m[2]};

    // ---- 校正基（Hartley-Sturm）----
    const Vec3 e_x = Normalized(T);
    Vec3 tmp = {-T[0] * T[2], -T[1] * T[2], T[0] * T[0] + T[1] * T[1]};
    if (Norm(tmp) < 1e-9) {
        // 退化：T 几乎沿光轴（非法立体基线），无法校正。
        throw std::invalid_argument("基线沿光轴方向，无法立体校正");
    }
    const Vec3 e_z = Normalized(tmp);
    const Vec3 e_y = Normalized(Cross(e_z, e_x));

    // R1 行 = [e_x; e_y; e_z]
    R1_ = {e_x[0], e_x[1], e_x[2],
           e_y[0], e_y[1], e_y[2],
           e_z[0], e_z[1], e_z[2]};
    const Matrix3 R2 = Mul(R1_, cal.R);

    // 校验：R1·T 应为 [baseline, 0, 0]
    const Vec3 rectT = MulVec(R1_, T);
    if (std::abs(rectT[1]) > 1e-3 * baseline_ || std::abs(rectT[2]) > 1e-3 * baseline_) {
        throw std::logic_error("校正后平移非纯 x 方向，构造有误");
    }

    // ---- 共用新内参 K_new ----
    focal_ = 0.25 * (cal.left.Fx() + cal.left.Fy() + cal.right.Fx() + cal.right.Fy());
    cx_new_ = 0.5 * (static_cast<double>(rect_width) - 1.0);
    cy_new_ = 0.5 * (static_cast<double>(rect_height) - 1.0);

    rect_width_ = rect_width;
    rect_height_ = rect_height;
    cal_left_ = cal.left;

    // ---- 构建正向 remap LUT ----
    const std::size_t npix = static_cast<std::size_t>(rect_width) * rect_height;
    left_.width = right_.width = rect_width;
    left_.height = right_.height = rect_height;
    left_.map_x.resize(npix);
    left_.map_y.resize(npix);
    right_.map_x.resize(npix);
    right_.map_y.resize(npix);

    const Matrix3 R1t = Transpose(R1_);   // rectified → cam1
    const Matrix3 R2t = Transpose(R2);    // rectified → cam2

    for (std::uint32_t v = 0; v < rect_height; ++v) {
        const double y_n = (static_cast<double>(v) - cy_new_) / focal_;
        for (std::uint32_t u = 0; u < rect_width; ++u) {
            const double x_n = (static_cast<double>(u) - cx_new_) / focal_;
            const std::size_t idx = static_cast<std::size_t>(v) * rect_width + u;

            // 左目
            const Vec3 c1 = MulVec(R1t, {x_n, y_n, 1.0});
            double a = c1[0] / c1[2], b = c1[1] / c1[2];
            double ad, bd;
            ApplyDistortion(cal.left.dist_coeffs, a, b, &ad, &bd);
            left_.map_x[idx] = static_cast<float>(cal.left.Fx() * ad + cal.left.Cx());
            left_.map_y[idx] = static_cast<float>(cal.left.Fy() * bd + cal.left.Cy());

            // 右目
            const Vec3 c2 = MulVec(R2t, {x_n, y_n, 1.0});
            double a2 = c2[0] / c2[2], b2 = c2[1] / c2[2];
            double ad2, bd2;
            ApplyDistortion(cal.right.dist_coeffs, a2, b2, &ad2, &bd2);
            right_.map_x[idx] = static_cast<float>(cal.right.Fx() * ad2 + cal.right.Cx());
            right_.map_y[idx] = static_cast<float>(cal.right.Fy() * bd2 + cal.right.Cy());
        }
    }
    ready_ = true;
}

void StereoRectifier::RemapBilinear(const GrayImage& src, const RemapLut& lut,
                                    std::uint8_t* out) const {
    const std::int64_t sw = static_cast<std::int64_t>(src.width);
    const std::int64_t sh = static_cast<std::int64_t>(src.height);
    for (std::uint32_t v = 0; v < lut.height; ++v) {
        for (std::uint32_t u = 0; u < lut.width; ++u) {
            const std::size_t idx = static_cast<std::size_t>(v) * lut.width + u;
            const double sx = lut.map_x[idx];
            const double sy = lut.map_y[idx];
            const std::int64_t x0 = static_cast<std::int64_t>(std::floor(sx));
            const std::int64_t y0 = static_cast<std::int64_t>(std::floor(sy));
            if (x0 < 0 || y0 < 0 || x0 + 1 >= sw || y0 + 1 >= sh) {
                out[idx] = 0;  // 采样越界填 0（rectified 边缘无效区）
                continue;
            }
            const double fx = sx - static_cast<double>(x0);
            const double fy = sy - static_cast<double>(y0);
            const std::uint8_t* p = src.data;
            const double p00 = p[static_cast<std::size_t>(y0) * src.stride + x0];
            const double p01 = p[static_cast<std::size_t>(y0) * src.stride + x0 + 1];
            const double p10 = p[static_cast<std::size_t>(y0 + 1) * src.stride + x0];
            const double p11 = p[static_cast<std::size_t>(y0 + 1) * src.stride + x0 + 1];
            const double top = p00 + (p01 - p00) * fx;
            const double bot = p10 + (p11 - p10) * fx;
            out[idx] = static_cast<std::uint8_t>(
                std::clamp(top + (bot - top) * fy, 0.0, 255.0));
        }
    }
}

void StereoRectifier::RectifyLeft(const GrayImage& src, std::uint8_t* out) const {
    if (!ready_) throw std::logic_error("StereoRectifier 未构建");
    RemapBilinear(src, left_, out);
}

void StereoRectifier::RectifyRight(const GrayImage& src, std::uint8_t* out) const {
    if (!ready_) throw std::logic_error("StereoRectifier 未构建");
    RemapBilinear(src, right_, out);
}

void StereoRectifier::SourceToRectifiedLeft(double src_x, double src_y,
                                            double* rect_u, double* rect_v) const {
    if (!ready_) throw std::logic_error("StereoRectifier 未构建");
    // 反投影到左目归一化坐标
    const double a_d = (src_x - cal_left_.Cx()) / cal_left_.Fx();
    const double b_d = (src_y - cal_left_.Cy()) / cal_left_.Fy();
    // 畸变逆解
    double a, b;
    RemoveDistortion(cal_left_.dist_coeffs, a_d, b_d, &a, &b);
    // 旋转到 rectified 系
    const Vec3 r = MulVec(R1_, {a, b, 1.0});
    *rect_u = focal_ * (r[0] / r[2]) + cx_new_;
    *rect_v = focal_ * (r[1] / r[2]) + cy_new_;
}

double StereoRectifier::DepthFromDisparity(double disp_px) const {
    if (!(disp_px > 0.0)) return NAN;
    return focal_ * baseline_ / disp_px;
}

}  // namespace stereo_depth
