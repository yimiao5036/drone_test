/**
 * @file subpixel_refiner.h
 * @brief 亚像素视差细化（可替换接口 ISubpixelRefiner）
 *
 * WTA 只给整数视差；亚像素细化用最小值及其两侧聚合代价拟合峰位。裸抛物线对
 * SGM/Census 的 Hamming 代价曲线有系统性偏差（代价曲线更接近 V 型/分段线性，
 * 抛物线会把峰位往整数拉偏），故提供两种估计器，接口注入可替换：
 *
 * - ParabolicSubpixel（经典）：δ = 0.5·(C₋ − C₊) / (C₋ − 2C₀ + C₊)
 * - LinearSubpixel（默认，本设计从 V 型代价模型第一性推导）：
 *     设 cost(d)=a·|d−d_true|+b，d_true=d*+δ，δ∈[−0.5,0.5]，则
 *       C₋=a(1+δ)+b, C₀=a|δ|+b, C₊=a(1−δ)+b
 *       ⇒ (C₋−C₊)/(C₋+C₊−2C₀) = δ/(1−|δ|) ≜ r
 *     反解：δ = r/(1+r) (r≥0)；δ = r/(1−r) (r<0)。
 *   该式对分段线性代价无偏，实测比裸抛物线更接近真值（见单测）。
 */
#pragma once

namespace stereo_depth {

/// 亚像素细化接口。输入 WTA 整数视差 d_best 及聚合代价 C(d*-1)/C(d*)/C(d*+1)，
/// 返回 [−0.5,0.5] 内的亚像素偏移；无法细化时返回 0。
class ISubpixelRefiner {
public:
    virtual ~ISubpixelRefiner() = default;

    /// @param cost_prev C(d*-1)；d*==0 时无左邻，传 cost_min
    /// @param cost_min  C(d*)
    /// @param cost_next C(d*+1)；d*==max-1 时无右邻，传 cost_min
    virtual double Refine(double cost_prev, double cost_min,
                          double cost_next) const = 0;
};

/// 经典抛物线拟合。
class ParabolicSubpixel final : public ISubpixelRefiner {
public:
    double Refine(double cost_prev, double cost_min, double cost_next) const override;
};

/// V 型（分段线性）代价模型拟合，默认。对 Census+SGM 代价曲线偏差更小。
class LinearSubpixel final : public ISubpixelRefiner {
public:
    double Refine(double cost_prev, double cost_min, double cost_next) const override;
};

/// 按配置名创建估计器："linear"→LinearSubpixel，"parabolic"→ParabolicSubpixel。
/// 非法名抛 std::invalid_argument。返回堆对象，调用方持有。
ISubpixelRefiner* CreateSubpixelRefiner(const char* name);

}  // namespace stereo_depth
