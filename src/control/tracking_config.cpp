// =============================================================================
// tracking_config.cpp —— 视觉跟踪控制律配置校验
//
// 字段解析在 config.cpp（主工程约定：配置段缺席用默认值）；本文件只做
// 跨字段/取值校验，非法抛 std::invalid_argument 并携带字段名。
// =============================================================================

#include "control/tracking_config.h"

#include <stdexcept>

namespace drone::control {

void TrackingConfig::Validate() const {
    if (!(control.frequency_hz > 0.0)) {
        throw std::invalid_argument("visual_tracking.control.frequency_hz 必须为正");
    }
    if (control.visual_stale_ms <= 0 || control.distance_stale_ms <= 0 ||
        control.attitude_stale_ms <= 0) {
        throw std::invalid_argument("visual_tracking.control.*_stale_ms 必须为正");
    }
    if (camera.image_width <= 0 || camera.image_height <= 0) {
        throw std::invalid_argument("visual_tracking.camera.image_width/height 必须为正");
    }
    if (!(camera.fx_px > 0.0) || !(camera.fy_px > 0.0)) {
        throw std::invalid_argument("visual_tracking.camera.fx_px/fy_px 必须为正");
    }
    // 主点可以偏离画面中心（2026-09-23 标定 cy=532 vs 360），但必须在画面内
    if (camera.cx_px < 0.0 || camera.cx_px > camera.image_width ||
        camera.cy_px < 0.0 || camera.cy_px > camera.image_height) {
        throw std::invalid_argument(
            "visual_tracking.camera.cx_px/cy_px 必须落在画面范围内");
    }
    if (!(heading.gain >= 0.0)) {
        throw std::invalid_argument("visual_tracking.heading.gain 不得为负");
    }
    if (!(heading.yaw_rate_limit_dps > 0.0) || !(heading.yaw_slew_limit_dps2 > 0.0)) {
        throw std::invalid_argument(
            "visual_tracking.heading.yaw_rate_limit_dps/yaw_slew_limit_dps2 必须为正");
    }
    if (!(vertical.vz_gain_mps_per_deg >= 0.0) || !(vertical.vz_limit_mps > 0.0)) {
        throw std::invalid_argument(
            "visual_tracking.vertical.vz_gain_mps_per_deg 不得为负、vz_limit_mps 必须为正");
    }
    if (!(distance.d_exp_m > 0.0)) {
        throw std::invalid_argument("visual_tracking.distance.d_exp_m 必须为正");
    }
    if (distance.kp < 0.0 || distance.ki < 0.0 || distance.kd < 0.0 ||
        distance.integral_limit < 0.0) {
        throw std::invalid_argument(
            "visual_tracking.distance.kp/ki/kd/integral_limit 不得为负");
    }
    if (distance.derivative_filter_coef < 0.0 ||
        distance.derivative_filter_coef >= 1.0) {
        throw std::invalid_argument(
            "visual_tracking.distance.derivative_filter_coef 必须在 [0,1) 内");
    }
    if (!(distance.approach_velocity_limit_mps > 0.0) ||
        !(distance.retreat_velocity_limit_mps > 0.0)) {
        throw std::invalid_argument(
            "visual_tracking.distance.approach/retreat_velocity_limit_mps 必须为正");
    }
    if (distance.no_distance_approach_limit_mps < 0.0 ||
        distance.no_distance_exit_grace_ms < 0) {
        throw std::invalid_argument(
            "visual_tracking.distance.no_distance_approach_limit_mps/exit_grace_ms 不得为负");
    }
    if (!(accel_limit.ax_mps2 > 0.0) || !(accel_limit.ay_mps2 > 0.0) ||
        !(accel_limit.az_mps2 > 0.0)) {
        throw std::invalid_argument("visual_tracking.accel_limit.* 必须为正");
    }
}

}  // namespace drone::control
