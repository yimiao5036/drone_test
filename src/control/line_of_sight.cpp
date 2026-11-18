// =============================================================================
// line_of_sight.cpp —— 像素坐标 → 视线角实现（内参针孔模型）
//
// 对应规格：docs/物理追踪思路.md §4.1（水平 β）、§5.1（垂直 α）：
//   内参式：angle = atan( (pixel − c) / f )
//
// 主点偏移基准为标定 cx/cy（2026-09-23 标定：cy 偏离画面中心约 9.7°，
// 画面中心假设不可用）；畸变不参与（见 line_of_sight.h 移植说明）。
// =============================================================================

#include "control/line_of_sight.h"

#include <cmath>
#include <stdexcept>

namespace drone::control {

PixelToAngle::PixelToAngle(double f_px, double c_px) : f_px_(f_px), c_px_(c_px) {
    if (!(f_px > 0.0)) {
        throw std::invalid_argument("PixelToAngle: f_px 必须为正");
    }
    if (!(c_px >= 0.0)) {
        throw std::invalid_argument("PixelToAngle: c_px 不得为负");
    }
}

double PixelToAngle::Convert(double pixel_px) const {
    return std::atan((pixel_px - c_px_) / f_px_);
}

}  // namespace drone::control
