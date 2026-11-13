/**
 * @file return_speed_law.cpp
 * @brief S12 返航水平速度剖面实现（分段恒定加速度，无 jerk）
 */
#include "control/return_speed_law.h"

#include <algorithm>
#include <cmath>

namespace drone::control {
namespace {

/// 斜坡步长下限保护：dt<=0（时钟回退/首拍）时不推进斜坡，也不做跳变。
constexpr double kMinDt = 1e-6;
/// 顶速比较的死区：低于该速率的差异不触发加速/减速分级，避免在巡航顶速
/// 附近反复 accel/decel 抖动（0121 SITL 实测 PX4 地速噪声 ±0.04m/s，
/// 1e-2 会让 `剖面=cruise/decel/accel` 每秒翻转；5e-2 足够吸收且不影响斜坡）。
constexpr double kSpeedDeadbandMps = 5e-2;

}  // namespace

const char* ReturnSpeedPhaseName(ReturnSpeedPhase phase) {
    switch (phase) {
        case ReturnSpeedPhase::kIdle:
            return "idle";
        case ReturnSpeedPhase::kAccel:
            return "accel";
        case ReturnSpeedPhase::kCruise:
            return "cruise";
        case ReturnSpeedPhase::kDecel:
            return "decel";
        case ReturnSpeedPhase::kBrake:
            return "brake";
        case ReturnSpeedPhase::kFinal:
            return "final";
    }
    return "unknown";
}

void ResetReturnSpeedState(ReturnSpeedState& state) {
    state.prev_speed_mps = 0.0;
    state.initialized = false;
}

ReturnSpeedOutput ComputeReturnSpeed(const ReturnSpeedLawConfig& config,
                                     const ReturnSpeedInput& input,
                                     ReturnSpeedState& state) {
    ReturnSpeedOutput out;
    if (!input.valid) {
        return out;
    }
    const double cruise = std::max(0.0, config.cruise_mps);
    const double final_speed = std::clamp(config.final_speed_mps, 0.0, cruise);
    const double brake_safety =
        config.brake_safety > 1.0 ? config.brake_safety : 1.0;
    const double remaining = std::max(0.0, input.remaining_m);
    const double measured = std::max(0.0, input.ground_speed_mps);
    const double dt = input.dt_s > kMinDt ? input.dt_s : 0.0;

    // ① 顶速：巡航段用 cruise_mps；进入末端低速区降为 final_speed_mps。
    const bool in_final_zone =
        config.final_zone_m > 0.0 && remaining <= config.final_zone_m;
    double ceiling = in_final_zone ? final_speed : cruise;

    // ② 末端制动包络：v ≤ √(2·(decel/brake_safety)·(d − arrival_radius))。
    // decel=0 表示不做减速规划，此时包络也不启用（包络本身就是减速能力）。
    double envelope = ceiling;
    bool brake_limited = false;
    if (config.decel_mps2 > 0.0) {
        const double usable =
            std::max(0.0, remaining - std::max(0.0, config.arrival_radius_m));
        const double decel_eff = config.decel_mps2 / brake_safety;
        envelope = std::sqrt(2.0 * decel_eff * usable);
        if (envelope < ceiling) {
            ceiling = envelope;
            brake_limited = true;
        }
    }

    // ③ 分段恒定加速度斜坡，起点是**实测地速**而不是上一拍指令：
    //    起步段机体速度会被风阻/前馈丢失吃掉，若按指令积分会"以为自己已经
    //    在巡航"，永远给不出加速段（正是"目标静止时飞机会停住不动"的成因）。
    //    减速段同理必须从实测出发，否则超速进入返航时指令会一直高于实况。
    double speed = ceiling;
    ReturnSpeedPhase phase;
    // dt<=0（首拍/时钟回退）不推进斜坡、也不跳变：保持实测速度；只有显式把
    // accel/decel 配成 0 才表示"该方向不做斜坡"（直通 ceiling）。
    const bool have_dt = dt > 0.0;
    if (ceiling > measured + kSpeedDeadbandMps) {
        const double step = config.accel_mps2 * dt;
        if (config.accel_mps2 <= 0.0) {
            speed = ceiling;  // 显式关闭加速斜坡：直通
        } else {
            speed = have_dt ? std::min(ceiling, measured + step) : measured;
        }
        phase = ReturnSpeedPhase::kAccel;
    } else if (ceiling < measured - kSpeedDeadbandMps) {
        const double step = config.decel_mps2 * dt;
        if (config.decel_mps2 <= 0.0) {
            speed = ceiling;  // 显式关闭减速斜坡：直通
        } else {
            speed = have_dt ? std::max(ceiling, measured - step) : measured;
        }
        phase = brake_limited ? ReturnSpeedPhase::kBrake
                              : ReturnSpeedPhase::kDecel;
    } else {
        speed = ceiling;
        if (brake_limited) {
            phase = ReturnSpeedPhase::kBrake;
        } else if (in_final_zone) {
            phase = ReturnSpeedPhase::kFinal;
        } else {
            phase = ReturnSpeedPhase::kCruise;
        }
    }

    // ④ 输出钳制。上界取 max(cruise, 实测)：减速段必须允许指令**暂时高于
    // cruise**（那是正在衰减的现有速度，钳到 cruise 会让超速回落变成瞬跳、
    // 斜坡失效）；加速/巡航段则恒 ≤ cruise，返航绝不会跑得比巡航顶速快。
    // 下界取 min(kReturnSpeedMinCapMps, cruise)：0 在本工程里是"未填相位
    // 上限"的语义（控制器回退硬上限），不能让末端包络收敛到 0 时把限速整个
    // 解除——那样最后几米会被 PX4 位置环重新加速（2026-11-12 复审发现）。
    speed = std::clamp(speed, std::min(kReturnSpeedMinCapMps, cruise),
                       std::max(cruise, measured));
    state.prev_speed_mps = speed;  // 仅诊断：本拍指令速率
    state.initialized = true;

    out.valid = true;
    out.speed_mps = speed;
    out.phase = phase;
    out.brake_limited = brake_limited;
    return out;
}

}  // namespace drone::control
