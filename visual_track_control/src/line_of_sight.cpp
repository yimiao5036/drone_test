// =============================================================================
// line_of_sight.cpp —— 像素偏移 → 视线角实现（全程针孔模型）
//
// 对应规格：docs/物理追踪思路.md §4.1（水平 β）、§5.1（垂直 α）：
//   针孔模型：angle = atan( offset · 2·tan(θ/2) / size )
//
// 原设计的线性近似双路切换已取消：线性式与针孔式在切换边界两侧不连续
// （θ=60°、边界比例 0.2 处约 8% 角度台阶），会向控制环路注入突变；
// atan 代价可忽略，全程采用精确针孔式。
//
// 构造期预计算针孔系数，Convert 热路径仅一次乘加 + atan。
// =============================================================================

#include "visual_track_control/line_of_sight.h"

#include <cmath>
#include <stdexcept>

namespace drone::vtc {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

PixelToAngle::PixelToAngle(double size_px, double fov_rad) {
    // 构造期校验：非法参数直接抛异常（初始化抛异常规范）
    if (!(size_px > 0.0)) {
        throw std::invalid_argument("PixelToAngle: size_px 必须为正");
    }
    if (!(fov_rad > 0.0 && fov_rad < kPi)) {
        throw std::invalid_argument("PixelToAngle: fov_rad 必须在 (0, π) 内");
    }

    // 预计算针孔系数（§4.1 公式）
    pinhole_scale_ = 2.0 * std::tan(fov_rad / 2.0) / size_px;  // 2·tan(θ/2) / size
}

double PixelToAngle::Convert(double offset_px) const {
    return std::atan(offset_px * pinhole_scale_);  // 针孔模型，全偏移范围精确
}

}  // namespace drone::vtc
