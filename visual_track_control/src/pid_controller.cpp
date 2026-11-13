// =============================================================================
// pid_controller.cpp —— 位置式 PID 控制器实现
//
// 对应规格：docs/物理追踪思路.md
//   §6.2 前向速度指令 = PID(Δd)
//   §6.4 Coast 时积分项清空（ResetIntegral）
//
// anti-windup 策略（条件积分）：输出达到饱和限幅且本拍积分增量仍在
// 推深饱和时，冻结本步积分累加（不采纳候选积分值），防止积分饱和；
// 误差反号后增量反向、允许积分回卷，避免饱和残留。
// =============================================================================

#include "visual_track_control/pid_controller.h"

#include <cmath>

#include "visual_track_control/rate_limiter.h"

namespace drone::vtc {

PidController::PidController(const PidParams& params) : params_(params) {}

double PidController::Update(double error, double dt_s) {
    if (dt_s <= 0.0) {
        return last_output_;  // 异常节拍保护：保持上次输出
    }

    // ---- 积分：候选值先钳位（积分限幅） ----
    double integral_candidate = integral_ + error * dt_s;
    if (params_.integral_limit > 0.0) {
        integral_candidate =
            Clamp(integral_candidate, -params_.integral_limit, params_.integral_limit);
    }

    // ---- 微分：一阶低通滤波（系数 ∈ [0,1)，0 = 不滤波） ----
    double derivative = 0.0;
    if (has_prev_error_) {
        derivative = (error - prev_error_) / dt_s;
        const double coef = params_.derivative_filter_coef;
        derivative = coef * filtered_derivative_ + (1.0 - coef) * derivative;
    }
    filtered_derivative_ = derivative;
    prev_error_ = error;
    has_prev_error_ = true;

    // ---- 位置式合成 ----
    const double integral_term = params_.ki * integral_candidate;
    const double raw = params_.kp * error + integral_term + params_.kd * derivative;

    // ---- anti-windup（条件积分，C1 修复） ----
    // 冻结判据基于"本拍积分增量方向"（error·dt）：输出饱和且本拍增量仍
    // 在推深饱和时才冻结。不采用旧实现"积分项与输出同向"判据（误差反号
    // 后无法回卷），也不用 raw·error（大 kp 项会掩盖饱和方向、拖延回卷）。
    const bool saturated = params_.output_limit > 0.0 && std::fabs(raw) > params_.output_limit;
    const bool increment_pushes_deeper = (error * dt_s) * raw > 0.0;
    if (!(saturated && increment_pushes_deeper)) {
        integral_ = integral_candidate;
    }

    // ---- 输出限幅 ----
    if (params_.output_limit > 0.0) {
        last_output_ = Clamp(raw, -params_.output_limit, params_.output_limit);
    } else {
        last_output_ = raw;
    }
    return last_output_;
}

void PidController::ResetIntegral() {
    integral_ = 0.0;  // §6.4：Coast 清积分，防积分饱和
}

void PidController::Reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    filtered_derivative_ = 0.0;
    last_output_ = 0.0;
    has_prev_error_ = false;
}

}  // namespace drone::vtc
