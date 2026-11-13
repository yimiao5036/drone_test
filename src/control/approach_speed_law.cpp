#include "control/approach_speed_law.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>

namespace drone::control {
namespace {

constexpr double kMinDecel = 1e-6;
constexpr double kMinDistance = 1e-6;
constexpr double kMaxTargetAccel = 20.0;
constexpr double kScheduleBandLowM = 8.0;
constexpr double kScheduleBandWidthM = 4.0;
constexpr double kScheduleRatePerS = 0.8;
constexpr double kTargetAccelTauS = 0.12;
constexpr double kTargetBrakeReactionS = 0.1;  // 采样+传输+一拍的近似余量
constexpr double kPenetrationM = 0.5;

double Clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

/// 匀减速制动距离（J=0）：u²/(2b)。
double StoppingDistance(double speed, double decel) {
    const double u = std::max(0.0, speed);
    const double b = std::max(kMinDecel, decel);
    return u * u / (2.0 * b);
}

/// 制动曲线允许的接近速度：一拍反应距离 + 匀减速距离，离散化留 1.3 余量
/// 由调用方对 decel 预除。
double BrakingSpeed(double distance, double decel, double cap, double dt) {
    if (!(distance > 0.0) || !(decel > kMinDecel)) {
        return 0.0;
    }
    const double v = std::sqrt((decel * dt) * (decel * dt) +
                               2.0 * decel * distance) -
                     decel * dt;
    return std::max(0.0, std::min(cap, v));
}

/// 目标急停预留：额外计入反应延迟与目标内向加速度。
double GuardedBrakingSpeed(double distance, double decel, double cap,
                           double delay, double reaction_accel) {
    if (!(distance > 0.0) || !(decel > kMinDecel)) {
        return 0.0;
    }
    double lo = 0.0;
    double hi = std::max(0.0, cap);
    for (int i = 0; i < 26; ++i) {
        const double u = (lo + hi) * 0.5;
        const double after = u + reaction_accel * delay;
        const double need = u * delay +
                            0.5 * reaction_accel * delay * delay +
                            StoppingDistance(after, decel);
        if (need <= distance) {
            lo = u;
        } else {
            hi = u;
        }
    }
    return lo;
}

/// 二维速度预算：总速度 ≤ cap 时，沿视线还能给多少接近速度。
double VelocityBudget(double cap, double along, double cross, bool match) {
    if (!match) {
        return cap;
    }
    if (std::abs(cross) > cap) {
        return 0.0;
    }
    return std::max(0.0, std::sqrt(std::max(0.0, cap * cap - cross * cross)) -
                             along);
}

struct RelativeBudget {
    double up = 0.0;
    double down = 0.0;
    bool feasible = false;
};

/// 加速度圆里先扣侧向需求，剩余才给径向（沿视线）加减速。
RelativeBudget ComputeRelativeBudget(double up, double down, double target_along,
                                     double side_need) {
    const auto radial = [](double cap, double side) {
        return std::sqrt(std::max(0.0, cap * cap - side * side));
    };
    const double radial_up = radial(up, side_need);
    const double radial_down = radial(down, side_need);
    RelativeBudget budget;
    budget.up = std::max(0.0, radial_up - target_along);
    budget.down = std::max(0.0, radial_down + target_along);
    budget.feasible = std::abs(side_need) <= std::min(up, down) &&
                      target_along <= radial_up &&
                      target_along >= -radial_down;
    return budget;
}

}  // namespace

const char* ApproachPhaseName(ApproachPhase phase) {
    switch (phase) {
        case ApproachPhase::kIdle:
            return "idle";
        case ApproachPhase::kAccel:
            return "accel";
        case ApproachPhase::kCruise:
            return "cruise";
        case ApproachPhase::kDecel:
            return "decel";
        case ApproachPhase::kMatch:
            return "match";
    }
    return "unknown";
}

void ApproachSpeedLawConfig::Validate() const {
    const auto require = [](bool condition, const char* message) {
        if (!condition) {
            throw std::invalid_argument(message);
        }
    };
    require(std::isfinite(standoff_m) && standoff_m > 0.0,
            "approach.speed_law.standoff_m 必须为有限正数");
    require(std::isfinite(accel_plan_mps2) && accel_plan_mps2 > 0.0,
            "approach.speed_law.accel_plan_mps2 必须为有限正数");
    require(std::isfinite(decel_plan_mps2) && decel_plan_mps2 > 0.0,
            "approach.speed_law.decel_plan_mps2 必须为有限正数");
    require(std::isfinite(accel_phys_mps2) && accel_phys_mps2 > 0.0,
            "approach.speed_law.accel_phys_mps2 必须为有限正数");
    require(std::isfinite(accel_term_mps2) && accel_term_mps2 > 0.0,
            "approach.speed_law.accel_term_mps2 必须为有限正数");
    require(std::isfinite(slew_mps2) && slew_mps2 >= 0.0,
            "approach.speed_law.slew_mps2 必须为有限非负数");
    require(std::isfinite(brake_safety) && brake_safety > 1.0,
            "approach.speed_law.brake_safety 必须大于 1");
    require(std::isfinite(terminal_gain) && terminal_gain > 0.0,
            "approach.speed_law.terminal_gain 必须为有限正数");
    require(std::isfinite(lead_scaled_k) && lead_scaled_k >= 0.0,
            "approach.speed_law.lead_scaled_k 必须为有限非负数");
    require(std::isfinite(lead_scaled_max_m) && lead_scaled_max_m >= 0.0,
            "approach.speed_law.lead_scaled_max_m 必须为有限非负数");
    require(std::isfinite(lead_extra_m) && lead_extra_m >= 0.0,
            "approach.speed_law.lead_extra_m 必须为有限非负数");
    require(std::isfinite(t_lead_max_s) && t_lead_max_s > 0.0,
            "approach.speed_law.t_lead_max_s 必须为有限正数");
    require(std::isfinite(target_brake_guard_mps2) &&
                target_brake_guard_mps2 >= 0.0,
            "approach.speed_law.target_brake_guard_mps2 必须为有限非负数");
    require(std::isfinite(boost_delay_s) && boost_delay_s >= 0.0,
            "approach.speed_law.boost_delay_s 必须为有限非负数");
    require(std::isfinite(boost_rate_mps2_per_s) && boost_rate_mps2_per_s >= 0.0,
            "approach.speed_law.boost_rate_mps2_per_s 必须为有限非负数");
    require(std::isfinite(ref_lead_s) && ref_lead_s >= 0.0,
            "approach.speed_law.ref_lead_s 必须为有限非负数");
    require(std::isfinite(ref_lead_max_m) && ref_lead_max_m >= 0.0,
            "approach.speed_law.ref_lead_max_m 必须为有限非负数");
    require(std::isfinite(vertical_speed_cap_mps) && vertical_speed_cap_mps > 0.0,
            "approach.speed_law.vertical_speed_cap_mps 必须为有限正数");
    require(std::isfinite(vertical_accel_cap_mps2) &&
                vertical_accel_cap_mps2 > 0.0,
            "approach.speed_law.vertical_accel_cap_mps2 必须为有限正数");
    require(std::isfinite(vertical_margin_m) && vertical_margin_m >= 0.0,
            "approach.speed_law.vertical_margin_m 必须为有限非负数");
}

ApproachSpeedLaw::ApproachSpeedLaw(const ApproachSpeedLawConfig& config,
                                   double max_speed_mps)
    : cfg_(config), max_speed_mps_(max_speed_mps) {
    cfg_.Validate();
    if (!std::isfinite(max_speed_mps_) || max_speed_mps_ <= 0.0) {
        throw std::invalid_argument("ApproachSpeedLaw max_speed_mps 必须为有限正数");
    }
}

void ApproachSpeedLaw::Reset() {
    initialized_ = false;
    last_now_ms_ = 0;
    last_target_sample_ms_ = 0;
    last_target_vn_mps_ = 0.0;
    last_target_ve_mps_ = 0.0;
    target_acc_n_mps2_ = 0.0;
    target_acc_e_mps2_ = 0.0;
    target_need_hold_ = 0.0;
    schedule_w_ = 1.0;
    cmd_vn_mps_ = 0.0;
    cmd_ve_mps_ = 0.0;
    cmd_vd_mps_ = 0.0;
    best_distance_m_ = 0.0;
    last_progress_ms_ = 0;
    boost_time_s_ = 0.0;
    accel_boost_mps2_ = 0.0;
    penetrated_ = false;
    capability_limited_ = false;
}

ApproachOutput ApproachSpeedLaw::Update(const ApproachInput& input,
                                        std::int64_t now_ms) {
    ApproachOutput out;
    if (!input.own_valid || !input.target_valid) {
        return out;
    }

    double dt = 0.1;  // 10 Hz 决策拍兜底；异常节拍由上下限夹住
    if (last_now_ms_ > 0 && now_ms > last_now_ms_) {
        dt = Clamp(static_cast<double>(now_ms - last_now_ms_) / 1000.0, 0.02,
                   0.25);
    }
    last_now_ms_ = now_ms;

    // ---- 目标加速度：相邻速度样本差分 + 一阶低通（不读真值） ----
    if (input.target_sample_ms > 0 &&
        input.target_sample_ms != last_target_sample_ms_) {
        if (last_target_sample_ms_ > 0) {
            const double h = static_cast<double>(input.target_sample_ms -
                                                 last_target_sample_ms_) /
                             1000.0;
            if (h > 1e-3) {
                double an = (input.target_vn_mps - last_target_vn_mps_) / h;
                double ae = (input.target_ve_mps - last_target_ve_mps_) / h;
                const double magnitude = std::hypot(an, ae);
                if (magnitude > kMaxTargetAccel) {
                    const double scale = kMaxTargetAccel / magnitude;
                    an *= scale;
                    ae *= scale;
                }
                const double alpha = 1.0 - std::exp(-h / kTargetAccelTauS);
                target_acc_n_mps2_ +=
                    alpha * (an - target_acc_n_mps2_);
                target_acc_e_mps2_ +=
                    alpha * (ae - target_acc_e_mps2_);
            }
        }
        last_target_sample_ms_ = input.target_sample_ms;
        last_target_vn_mps_ = input.target_vn_mps;
        last_target_ve_mps_ = input.target_ve_mps;
    }

    if (!initialized_) {
        initialized_ = true;
        cmd_vn_mps_ = input.own_vn_mps;
        cmd_ve_mps_ = input.own_ve_mps;
        cmd_vd_mps_ = input.own_vd_mps;
        best_distance_m_ = std::hypot(input.target_n_m - input.own_n_m,
                                      input.target_e_m - input.own_e_m);
        last_progress_ms_ = now_ms;
    }

    const double vn = input.target_vn_mps;
    const double ve = input.target_ve_mps;
    const double vt_abs = std::hypot(vn, ve);

    // ---- 当前可见几何与视线单位向量（北/东） ----
    const double dx = input.target_n_m - input.own_n_m;
    const double dy = input.target_e_m - input.own_e_m;
    const double distance = std::hypot(dx, dy);
    const double nx = distance > kMinDistance ? dx / distance : 1.0;
    const double ny = distance > kMinDistance ? dy / distance : 0.0;

    // ---- 时延补偿 + 距离自适应提前量 ----
    const double age =
        (input.target_sample_ms > 0 && now_ms > input.target_sample_ms)
            ? std::min(0.6, static_cast<double>(now_ms - input.target_sample_ms) /
                                1000.0)
            : 0.0;
    const double lead_adaptive =
        std::min(cfg_.lead_scaled_k * std::max(0.0, distance - cfg_.standoff_m),
                 cfg_.lead_scaled_max_m);
    const double lead_total = lead_adaptive + cfg_.lead_extra_m;
    const double ahead = age + lead_total / std::max(0.5, vt_abs);
    const double seen_n = input.target_n_m + vn * ahead;
    const double seen_e = input.target_e_m + ve * ahead;

    // ---- 瞄准点：速度匹配时不做拦截提前（提前量已由 seen 补时延） ----
    double t_go = 0.0;
    if (!cfg_.speed_match && distance > kMinDistance) {
        const double rx = seen_n - input.own_n_m;
        const double ry = seen_e - input.own_e_m;
        const double rv = rx * vn + ry * ve;
        const double r2 = rx * rx + ry * ry;
        const double qa = max_speed_mps_ * max_speed_mps_ - vt_abs * vt_abs;
        const double qb = -2.0 * rv;
        const double qc = -r2;
        double t = std::numeric_limits<double>::quiet_NaN();
        if (std::abs(qa) <= 1e-9) {
            if (std::abs(qb) > 1e-9) {
                t = -qc / qb;
            }
        } else {
            const double disc = qb * qb - 4.0 * qa * qc;
            if (disc >= 0.0) {
                const double root = std::sqrt(disc);
                const double t1 = (-qb - root) / (2.0 * qa);
                const double t2 = (-qb + root) / (2.0 * qa);
                const double candidates[2] = {t1, t2};
                for (double candidate : candidates) {
                    if (std::isfinite(candidate) && candidate > 0.0 &&
                        (!std::isfinite(t) || candidate < t)) {
                        t = candidate;
                    }
                }
            }
        }
        if (!std::isfinite(t) || t <= 0.0) {
            t = distance / std::max(1.0, max_speed_mps_);
        }
        t = Clamp(t, 0.0, 4.0);
        const double lead_want =
            std::min({vt_abs * t, 0.5 * distance,
                      max_speed_mps_ * cfg_.t_lead_max_s});
        t_go = vt_abs > 1e-6 ? lead_want / vt_abs : 0.0;
    }
    const double aim_n = seen_n + vn * t_go;
    const double aim_e = seen_e + ve * t_go;

    // ---- 以瞄准点方向为接近方向 ----
    const double ax = aim_n - input.own_n_m;
    const double ay = aim_e - input.own_e_m;
    const double aim_distance = std::hypot(ax, ay);
    const double ux = aim_distance > kMinDistance ? ax / aim_distance : 1.0;
    const double uy = aim_distance > kMinDistance ? ay / aim_distance : 0.0;
    const double vt_along = vn * ux + ve * uy;
    const double vt_cross = std::abs(-uy * vn + ux * ve);
    const double vd_along = input.own_vn_mps * ux + input.own_ve_mps * uy;
    const double closing = vd_along - vt_along;

    // ---- 目标机动需求与峰值保持 ----
    const double need_now = std::hypot(target_acc_n_mps2_, target_acc_e_mps2_);
    if (cfg_.target_need_hold) {
        target_need_hold_ = std::max(target_need_hold_, need_now);
    } else {
        target_need_hold_ = need_now;
    }
    const double need_peak = std::max(need_now, target_need_hold_);

    // ---- 停滞自动提能力（用上一拍的能力受限/穿透状态计时） ----
    const bool stalled =
        now_ms - last_progress_ms_ >= 4000 && distance > cfg_.standoff_m + 1.0;
    if (distance < best_distance_m_ - 1.0) {
        best_distance_m_ = distance;
        last_progress_ms_ = now_ms;
    }
    if (distance < kPenetrationM) {
        penetrated_ = true;
    }
    if (cfg_.auto_boost) {
        if (stalled || capability_limited_ || penetrated_) {
            boost_time_s_ += dt;
        } else {
            boost_time_s_ = std::max(0.0, boost_time_s_ - 0.5 * dt);
        }
        const double want =
            boost_time_s_ > cfg_.boost_delay_s
                ? std::max(0.0, cfg_.accel_phys_mps2 -
                                    std::min(cfg_.accel_plan_mps2,
                                             cfg_.decel_plan_mps2))
                : 0.0;
        const double limit = cfg_.boost_rate_mps2_per_s * dt;
        accel_boost_mps2_ +=
            Clamp(want - accel_boost_mps2_, -limit, limit);
    } else {
        boost_time_s_ = 0.0;
        accel_boost_mps2_ = 0.0;
    }

    // ---- 分相位加速度调度：12 m 外 a_plan、8 m 内 a_term_eff、之间过渡 ----
    const double a_term_eff =
        std::max(cfg_.accel_term_mps2 + accel_boost_mps2_,
                 std::min(need_peak + 0.5, cfg_.accel_phys_mps2));
    const double w_target =
        Clamp((distance - kScheduleBandLowM) / kScheduleBandWidthM, 0.0, 1.0);
    const double w_step = kScheduleRatePerS * dt;
    schedule_w_ += Clamp(w_target - schedule_w_, -w_step, w_step);
    const double plan_up = a_term_eff +
                           (cfg_.accel_plan_mps2 + accel_boost_mps2_ -
                            a_term_eff) *
                               schedule_w_;
    const double plan_down = a_term_eff +
                             (cfg_.decel_plan_mps2 + accel_boost_mps2_ -
                              a_term_eff) *
                                 schedule_w_;
    const double slew_eff =
        cfg_.slew_mps2 > 0.0
            ? std::max(cfg_.slew_mps2,
                       accel_boost_mps2_ > 0.0 ? std::max(plan_up, plan_down)
                                               : 0.0)
            : std::numeric_limits<double>::infinity();
    const double up = std::min({plan_up, cfg_.accel_phys_mps2, slew_eff});
    const double down = std::min({plan_down, cfg_.accel_phys_mps2, slew_eff});
    const double brake_down = std::min(down, a_term_eff);

    // ---- 侧向预算：目标法向机动 + 跟住视线旋转所需 ----
    const double target_along =
        target_acc_n_mps2_ * nx + target_acc_e_mps2_ * ny;
    const double target_side =
        -ny * target_acc_n_mps2_ + nx * target_acc_e_mps2_;
    const double cross_relative =
        -(input.own_vn_mps - vn) * ny + (input.own_ve_mps - ve) * nx;
    const double los_rate = -cross_relative / std::max(0.1, distance);
    const double side_need = target_side + closing * los_rate;
    RelativeBudget budget =
        ComputeRelativeBudget(up, brake_down, target_along, side_need);

    const double reserve = cfg_.target_brake_guard_mps2;
    if (reserve > 0.0 && vt_abs > 0.05) {
        const RelativeBudget guarded = ComputeRelativeBudget(
            up, brake_down, std::min(target_along, -reserve), side_need);
        budget.down = std::min(budget.down, guarded.down);
    }
    capability_limited_ =
        cfg_.speed_match && (!budget.feasible || budget.down <= 1e-6 ||
                             budget.up <= 1e-6);

    const double decel_eff =
        std::min(brake_down, budget.down) / cfg_.brake_safety;
    const double radial_acc = -target_along -
                              cross_relative * cross_relative /
                                  std::max(0.1, distance);

    // ---- 制动包络 ----
    const double cap_env = max_speed_mps_ + vt_abs + 1.0;
    double brake_bound =
        BrakingSpeed(std::max(0.0, distance - cfg_.standoff_m), decel_eff,
                     cap_env, dt);
    if (reserve > 0.0 && vt_abs > 0.05) {
        const double reaction_t = dt + age + kTargetBrakeReactionS;
        const double worst_target_brake = std::max(reserve, -target_along);
        const double reaction_accel = std::max(0.0, 0.0) + worst_target_brake;
        brake_bound = std::min(
            brake_bound,
            GuardedBrakingSpeed(distance + cfg_.standoff_m - 0.5, decel_eff,
                                cap_env, reaction_t, reaction_accel));
    }
    const double u_env =
        std::min(brake_bound, cfg_.terminal_gain *
                                  std::max(0.0, distance - cfg_.standoff_m));

    // ---- 速度预算 + 转弯约束 + 末端匹配 ----
    const double plateau = max_speed_mps_;
    const double v_catch = VelocityBudget(plateau, vt_along, vt_cross,
                                          cfg_.speed_match);
    const double ff_n = cfg_.speed_match ? vn : 0.0;
    const double ff_e = cfg_.speed_match ? ve : 0.0;
    const double omega_los = std::abs(cross_relative) / std::max(1e-6, distance);
    const double side_room =
        std::max(0.0, 0.8 * std::min(up, down) - std::abs(target_side));
    const double u_turn = omega_los > 1e-4
                              ? side_room / omega_los
                              : std::numeric_limits<double>::infinity();
    const double u_close =
        std::max(0.0, std::min({v_catch, u_env, u_turn}));

    double desired_n = ff_n + u_close * ux;
    double desired_e = ff_e + u_close * uy;
    ApproachPhase phase = ApproachPhase::kIdle;
    if (distance < cfg_.standoff_m) {
        // 已进入控制间距：有符号比例回退，恢复到 R 并压住相对速度。
        const double close =
            -std::min(cfg_.terminal_gain * (cfg_.standoff_m - distance),
                      BrakingSpeed(cfg_.standoff_m - distance,
                                   budget.up / cfg_.brake_safety, cap_env, dt));
        desired_n = ff_n + close * nx;
        desired_e = ff_e + close * ny;
        phase = ApproachPhase::kMatch;
        if (u_close > 1e-3) {
            phase = ApproachPhase::kDecel;
        }
    } else if (u_close > 1e-3) {
        phase = u_env < v_catch ? ApproachPhase::kDecel : ApproachPhase::kAccel;
    }

    // 机体总速度硬上限（先夹紧，再做一阶 slew）。
    const double desired_mag = std::hypot(desired_n, desired_e);
    if (desired_mag > plateau && desired_mag > 1e-9) {
        desired_n *= plateau / desired_mag;
        desired_e *= plateau / desired_mag;
    }
    if (cfg_.slew_mps2 > 0.0) {
        const double dv_n = desired_n - cmd_vn_mps_;
        const double dv_e = desired_e - cmd_ve_mps_;
        const double dv_mag = std::hypot(dv_n, dv_e);
        const double max_dv = slew_eff * dt;
        if (dv_mag > max_dv && dv_mag > 1e-9) {
            cmd_vn_mps_ += dv_n * (max_dv / dv_mag);
            cmd_ve_mps_ += dv_e * (max_dv / dv_mag);
        } else {
            cmd_vn_mps_ = desired_n;
            cmd_ve_mps_ = desired_e;
        }
    } else {
        cmd_vn_mps_ = desired_n;
        cmd_ve_mps_ = desired_e;
    }

    // ---- 垂直通道（与水平解耦；NED z 向下为正） ----
    const double dz_up = -(input.target_d_m - input.own_d_m);
    const double vz_match_up = cfg_.speed_match ? -input.target_vd_mps : 0.0;
    const double vz_allow =
        std::sqrt(2.0 * (cfg_.vertical_accel_cap_mps2 / cfg_.brake_safety) *
                  std::max(0.0, std::abs(dz_up) - cfg_.vertical_margin_m));
    const double sign = dz_up >= 0.0 ? 1.0 : -1.0;
    double vz_up = vz_match_up + sign * std::min(cfg_.vertical_speed_cap_mps,
                                                 vz_allow);
    vz_up = Clamp(vz_up, -cfg_.vertical_speed_cap_mps,
                  cfg_.vertical_speed_cap_mps);
    const double vz_des_ned = -vz_up;
    const double dvz = Clamp(vz_des_ned - cmd_vd_mps_,
                             -cfg_.vertical_accel_cap_mps2 * dt,
                             cfg_.vertical_accel_cap_mps2 * dt);
    if (cfg_.slew_mps2 > 0.0) {
        cmd_vd_mps_ += dvz;
    } else {
        cmd_vd_mps_ = vz_des_ned;
    }

    out.valid = true;
    out.v_n_mps = cmd_vn_mps_;
    out.v_e_mps = cmd_ve_mps_;
    out.v_d_mps = cmd_vd_mps_;
    // 参考点 = 自机 + v_ref × ref_lead_s（沿指令方向，上限 ref_lead_max_m）。
    // 位置环只追这个“速度锚点”；目标点本身不再作为位置目标，避免制动段
    // 位置误差额外加速（2026-11-11 SITL 实测冲进 0.65m）。
    const double ref_speed = std::hypot(cmd_vn_mps_, cmd_ve_mps_);
    const double ref_lead =
        std::min(cfg_.ref_lead_max_m, ref_speed * cfg_.ref_lead_s);
    const double ref_dir_n = ref_speed > 1e-6 ? cmd_vn_mps_ / ref_speed : 1.0;
    const double ref_dir_e = ref_speed > 1e-6 ? cmd_ve_mps_ / ref_speed : 0.0;
    out.ref_n_m = input.own_n_m + ref_dir_n * ref_lead;
    out.ref_e_m = input.own_e_m + ref_dir_e * ref_lead;
    out.ref_d_m = input.own_d_m + cmd_vd_mps_ * cfg_.ref_lead_s;
    out.distance_m = distance;
    out.closing_mps = closing;
    out.phase = phase;
    out.capability_limited = capability_limited_;
    out.brake_infeasible = decel_eff < 0.05;
    out.penetrated = penetrated_;
    return out;
}

}  // namespace drone::control
