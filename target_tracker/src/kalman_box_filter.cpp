// =============================================================================
// kalman_box_filter.cpp —— 单目标 8 维恒定速度卡尔曼滤波器实现
//
// 对应规格：docs/视觉追踪技术路线.md §4（算法细节）、§8.2（内部常量）、
// §9.3（数值计算：定长数组手写矩阵乘法，4×4 高斯-约当求逆 + 1e-9 正则化）。
// 关键公式逐行注释并标注对照文档小节号。
// =============================================================================

#include "target_tracker/kalman_box_filter.h"

#include <cmath>
#include <utility>

namespace drone {
namespace tracker {
namespace {

// ---- 定长矩阵基础运算（§9.3：无需引入 Eigen/OpenCV） ----

/// 8×8 矩阵乘法：C = A·B
std::array<std::array<double, 8>, 8> MatMul8(const std::array<std::array<double, 8>, 8>& a,
                                               const std::array<std::array<double, 8>, 8>& b) {
    std::array<std::array<double, 8>, 8> c{};
    for (int i = 0; i < 8; ++i) {
        for (int k = 0; k < 8; ++k) {
            if (a[i][k] == 0.0) continue;  // 稀疏常量矩阵加速，结果不变
            for (int j = 0; j < 8; ++j) {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return c;
}

/// 8×8 矩阵转置：R = Aᵀ
std::array<std::array<double, 8>, 8> MatTranspose8(
    const std::array<std::array<double, 8>, 8>& a) {
    std::array<std::array<double, 8>, 8> r{};
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            r[i][j] = a[j][i];
        }
    }
    return r;
}

/// 4×4 矩阵求逆：高斯-约当消元 + 部分主元（§9.3）。
/// 输入矩阵先对角加 1e-9 正则化防奇异（§9.3 建议）。
/// 返回的 bool 表示是否求逆成功（主元退化时返回 false）。
bool Invert4x4(const std::array<std::array<double, 4>, 4>& m,
               std::array<std::array<double, 4>, 4>& out) {
    constexpr double kRegularization = 1e-9;  // 正则化极小量（§9.3）

    // 增广矩阵 [M + εI | I]，消元后左半变为单位阵、右半即为逆矩阵
    std::array<std::array<double, 8>, 4> aug{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            aug[i][j] = m[i][j] + (i == j ? kRegularization : 0.0);
            aug[i][j + 4] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (int col = 0; col < 4; ++col) {
        // 部分主元：选取当前列绝对值最大行交换，提高数值稳定性
        int pivot = col;
        for (int r = col + 1; r < 4; ++r) {
            if (std::abs(aug[r][col]) > std::abs(aug[pivot][col])) {
                pivot = r;
            }
        }
        if (std::abs(aug[pivot][col]) < 1e-12) {
            return false;  // 主元退化（正则化后理论上不可达）
        }
        if (pivot != col) {
            std::swap(aug[pivot], aug[col]);
        }
        // 主元行归一化
        const double d = aug[col][col];
        for (int j = 0; j < 8; ++j) {
            aug[col][j] /= d;
        }
        // 消去其余行（高斯-约当：一次消成对角单位阵）
        for (int r = 0; r < 4; ++r) {
            if (r == col) continue;
            const double f = aug[r][col];
            if (f == 0.0) continue;
            for (int j = 0; j < 8; ++j) {
                aug[r][j] -= f * aug[col][j];
            }
        }
    }

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            out[i][j] = aug[i][j + 4];
        }
    }
    return true;
}

}  // namespace

void KalmanBoxFilter::Initiate(const BoxCXCYWH& z) {
    // §4.3 初始化：状态 = [cx, cy, w, h, 0, 0, 0, 0]（速度置零）
    x_ = {{z.cx, z.cy, z.w, z.h, 0.0, 0.0, 0.0, 0.0}};

    // §4.3 协方差：P = diag(2,2,2,2,100,100,100,100)
    // （位置置信度中等，速度置信度低，允许前几帧快速收敛速度估计）
    for (auto& row : P_) row.fill(0.0);
    for (int i = 0; i < 4; ++i) {
        P_[i][i] = kIniPos;       // INI_POS = 2.0（§4.2）
        P_[i + 4][i + 4] = kIniVel;  // INI_VEL = 100.0（§4.2）
    }
    initialized_ = true;
}

std::optional<BoxCXCYWH> KalmanBoxFilter::Predict() {
    // §4.4：未初始化返回空
    if (!initialized_) {
        return std::nullopt;
    }

    // §4.2 动态过程噪声：Q = diag(σ_pos², σ_vel²)，σ 按状态中 w/h 缩放
    //   σ_pos = [w/50, h/50, w/50, h/50]
    //   σ_vel = [w/200, h/200, w/200, h/200]
    // （Q 直接使用状态值，不做 abs，与规格 §4.2 注记保持一致）
    const double w = x_[2];
    const double h = x_[3];
    const double std_pos[4] = {w * kStdPos, h * kStdPos, w * kStdPos, h * kStdPos};
    const double std_vel[4] = {w * kStdVel, h * kStdVel, w * kStdVel, h * kStdVel};
    std::array<std::array<double, 8>, 8> q{};
    for (int i = 0; i < 4; ++i) {
        q[i][i] = std_pos[i] * std_pos[i];       // 前 4 对角元来自 σ_pos（各分量先平方）
        q[i + 4][i + 4] = std_vel[i] * std_vel[i];  // 后 4 对角元来自 σ_vel
    }

    // §4.4 状态转移：x ← F·x（恒定速度模型：位置/尺寸各自加上对应速度）
    std::array<double, 8> x_new{};
    for (int i = 0; i < 8; ++i) {
        double sum = 0.0;
        for (int j = 0; j < 8; ++j) {
            sum += kF[i][j] * x_[j];
        }
        x_new[i] = sum;
    }
    x_ = x_new;

    // §4.4 协方差传播：P ← F·P·Fᵀ + Q
    P_ = MatMul8(MatMul8(kF, P_), MatTranspose8(kF));
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            P_[i][j] += q[i][j];
        }
    }

    // §4.4 返回预测的位置/尺寸（状态向量前 4 维的副本）
    return BoxCXCYWH{x_[0], x_[1], x_[2], x_[3]};
}

void KalmanBoxFilter::Update(const BoxCXCYWH& z) {
    // §4.5：未初始化直接返回
    if (!initialized_) {
        return;
    }

    // §4.2 动态观测噪声：R = diag(2(w/50)², 2(h/50)², 2(w/50)², 2(h/50)²)
    // （R 使用 abs(w)/abs(h) 防御负尺寸，并额外乘 2 增强平滑；
    //  该差异与规格 §4.2 注记保持一致）
    const double w = std::abs(x_[2]);
    const double h = std::abs(x_[3]);
    std::array<std::array<double, 4>, 4> r{};
    const double rw = kRScale * (w * kStdPos) * (w * kStdPos);
    const double rh = kRScale * (h * kStdPos) * (h * kStdPos);
    r[0][0] = rw;
    r[1][1] = rh;
    r[2][2] = rw;
    r[3][3] = rh;

    // §4.5 新息协方差：S = H·P·Hᵀ + R（4×4）
    // H = [I4 | 0]，故 H·P·Hᵀ 即 P 的左上 4×4 子块
    std::array<std::array<double, 4>, 4> s{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            s[i][j] = P_[i][j] + r[i][j];
        }
    }

    // §4.5 / §9.3：S⁻¹（4×4 高斯-约当求逆 + 对角加 1e-9 正则化防奇异）
    std::array<std::array<double, 4>, 4> s_inv{};
    if (!Invert4x4(s, s_inv)) {
        return;  // 正则化后理论不可达；防御性跳过本次更新
    }

    // §4.5 卡尔曼增益：K = P·Hᵀ·S⁻¹（8×4）
    // H = [I4 | 0]，故 P·Hᵀ 即 P 的前 4 列（8×4）
    std::array<std::array<double, 4>, 8> k{};
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int l = 0; l < 4; ++l) {
                sum += P_[i][l] * s_inv[l][j];
            }
            k[i][j] = sum;
        }
    }

    // §4.5 残差（innovation）：y = z − H·x（H·x 即状态前 4 维）
    const double z_vec[4] = {z.cx, z.cy, z.w, z.h};
    double y[4];
    for (int i = 0; i < 4; ++i) {
        y[i] = z_vec[i] - x_[i];
    }

    // §4.5 状态更新：x ← x + K·y
    for (int i = 0; i < 8; ++i) {
        double sum = 0.0;
        for (int j = 0; j < 4; ++j) {
            sum += k[i][j] * y[j];
        }
        x_[i] += sum;
    }

    // §4.5 协方差更新：P ← (I8 − K·H)·P
    // K·H（8×8）= K 的前 4 列铺到前 4 列上（H 选通前 4 维）
    std::array<std::array<double, 8>, 8> ikh{};
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            ikh[i][j] = (i == j ? 1.0 : 0.0) - (j < 4 ? k[i][j] : 0.0);
        }
    }
    P_ = MatMul8(ikh, P_);
}

}  // namespace tracker
}  // namespace drone
