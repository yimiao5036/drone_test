/**
 * @file subpixel_refiner.cpp
 * @brief 亚像素细化实现
 */
#include "stereo_depth/subpixel_refiner.h"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace stereo_depth {
namespace {
constexpr double kEps = 1e-9;
constexpr double kHalf = 0.5;
}  // namespace

double ParabolicSubpixel::Refine(double cost_prev, double cost_min,
                                 double cost_next) const {
    const double denom = cost_prev - 2.0 * cost_min + cost_next;
    if (denom <= kEps) return 0.0;  // 无明确极小
    const double delta = 0.5 * (cost_prev - cost_next) / denom;
    return std::clamp(delta, -kHalf, kHalf);
}

double LinearSubpixel::Refine(double cost_prev, double cost_min,
                              double cost_next) const {
    const double num = cost_prev - cost_next;
    const double den = cost_prev + cost_next - 2.0 * cost_min;
    if (den <= kEps) return 0.0;  // 退化（V 型无曲率）
    const double r = num / den;   // = δ/(1-|δ|)
    const double delta = (r >= 0.0) ? r / (1.0 + r) : r / (1.0 - r);
    return std::clamp(delta, -kHalf, kHalf);
}

ISubpixelRefiner* CreateSubpixelRefiner(const char* name) {
    const std::string n = (name != nullptr) ? std::string(name) : std::string();
    if (n == "linear") return new LinearSubpixel();
    if (n == "parabolic") return new ParabolicSubpixel();
    throw std::invalid_argument("未知亚像素估计器: " + n);
}

}  // namespace stereo_depth
