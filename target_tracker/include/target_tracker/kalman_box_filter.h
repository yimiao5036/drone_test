#pragma once

// =============================================================================
// kalman_box_filter.h —— 单目标 8 维恒定速度卡尔曼滤波器
//
// 对应规格：docs/视觉追踪技术路线.md §4（卡尔曼滤波算法细节）、§8.2（内部常量）
//   状态向量 x[8] = [cx, cy, w, h, vx, vy, vw, vh]（§4.1）
//   观测向量 z[4] = [cx, cy, w, h]（§4.1）
//   时间步长 dt = 1 帧，与帧率无关（§1.3）
//
// 实现约束（§9.3）：矩阵规模极小（8×8 / 4×8 / 4×4），手写定长矩阵运算，
// 全程 double 精度，不引入 Eigen/OpenCV 等线性代数库。
// =============================================================================

#include <array>
#include <optional>

namespace drone {
namespace tracker {

/// 轴对齐边界框（中心点 + 宽高表示，卡尔曼观测空间）
struct BoxCXCYWH {
    double cx = 0.0;  // 框中心 x（像素）
    double cy = 0.0;  // 框中心 y（像素）
    double w = 0.0;   // 框宽（像素）
    double h = 0.0;   // 框高（像素）
};

/// 单目标恒定速度卡尔曼滤波器（§3.1 / §4）
class KalmanBoxFilter {
public:
    KalmanBoxFilter() = default;

    /// 是否已用首次观测初始化（§3.1 initialized 字段）
    bool Initialized() const { return initialized_; }

    /// 用首次观测 [cx, cy, w, h] 初始化（§4.3）：
    /// 状态速度置零；P = diag(2,2,2,2,100,100,100,100)。
    void Initiate(const BoxCXCYWH& z);

    /// 预测推进一帧（§4.4）：未初始化返回 std::nullopt；
    /// 否则 x ← F·x，P ← F·P·Fᵀ + Q，返回状态前 4 维 [cx,cy,w,h] 副本。
    std::optional<BoxCXCYWH> Predict();

    /// 观测更新（§4.5）：未初始化时直接返回；
    /// S = H·P·Hᵀ + R，K = P·Hᵀ·S⁻¹，x ← x + K(z−Hx)，P ← (I−KH)P。
    void Update(const BoxCXCYWH& z);

    /// 状态向量只读访问（[cx, cy, w, h, vx, vy, vw, vh]，测试与排查用）
    const std::array<double, 8>& State() const { return x_; }

    // ---- 内部常量（§4.2 / §8.2，经验值，针对 640×480 画面调优） ----
    static constexpr double kStdPos = 1.0 / 50.0;   // STD_POS：位置噪声比例系数
    static constexpr double kStdVel = 1.0 / 200.0;  // STD_VEL：速度噪声比例系数
    static constexpr double kIniPos = 2.0;          // INI_POS：初始位置协方差
    static constexpr double kIniVel = 100.0;        // INI_VEL：初始速度协方差
    static constexpr double kRScale = 2.0;          // R 额外缩放因子（增强平滑，§8.2）

private:
    std::array<double, 8> x_{};                // 状态向量（§3.1）
    std::array<std::array<double, 8>, 8> P_{}; // 状态协方差 8×8（§3.1）
    bool initialized_ = false;                 // 是否已初始化（§3.1）

    // 状态转移矩阵 F（常量，§4.1）：I4 + 右上 I4，即 F[i][i]=1、F[i][i+4]=1
    static constexpr std::array<std::array<double, 8>, 8> kF = {{
        {{1, 0, 0, 0, 1, 0, 0, 0}},
        {{0, 1, 0, 0, 0, 1, 0, 0}},
        {{0, 0, 1, 0, 0, 0, 1, 0}},
        {{0, 0, 0, 1, 0, 0, 0, 1}},
        {{0, 0, 0, 0, 1, 0, 0, 0}},
        {{0, 0, 0, 0, 0, 1, 0, 0}},
        {{0, 0, 0, 0, 0, 0, 1, 0}},
        {{0, 0, 0, 0, 0, 0, 0, 1}},
    }};

    // 观测矩阵 H（常量，§4.1）：[I4 | 0]（4 行 × 8 列），
    // 直接观测位置与尺寸、不观测速度
    static constexpr std::array<std::array<double, 8>, 4> kH = {{
        {{1, 0, 0, 0, 0, 0, 0, 0}},
        {{0, 1, 0, 0, 0, 0, 0, 0}},
        {{0, 0, 1, 0, 0, 0, 0, 0}},
        {{0, 0, 0, 1, 0, 0, 0, 0}},
    }};
};

}  // namespace tracker
}  // namespace drone
