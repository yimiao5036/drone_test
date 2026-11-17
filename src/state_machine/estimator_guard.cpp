/**
 * @file estimator_guard.cpp
 * @brief 估计器健康判据实现（判据阈值与极性说明见 estimator_guard.h）
 */
#include "state_machine/estimator_guard.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

namespace drone::state_machine {
namespace {

// MAV_SYS_STATUS_SENSOR_*（数值与 MAVLink common.xml 一致）。这里不引 mavlink 头，
// 避免状态机库把 third_party 的 SYSTEM include 传播到所有依赖方。
constexpr uint32_t kSensor3dGyro = 1u << 0;   ///< MAV_SYS_STATUS_SENSOR_3D_GYRO
constexpr uint32_t kSensor3dAccel = 1u << 1;  ///< MAV_SYS_STATUS_SENSOR_3D_ACCEL
constexpr uint32_t kSensor3dMag = 1u << 2;    ///< MAV_SYS_STATUS_SENSOR_3D_MAG
constexpr uint32_t kSensorAhrs = 1u << 21;    ///< MAV_SYS_STATUS_AHRS
/// 默认关键传感器：陀螺/加速度计/磁罗盘/AHRS。
/// 不含 GPS：GNSS 丢失时 EKF 会转惯导，属于"精度下降"而不是"估计已坏"，
/// 直接切 POSCTL 反而打断任务；GPS 相关处置由既有健康管理负责。
constexpr uint32_t kCriticalSensorMask =
    kSensor3dGyro | kSensor3dAccel | kSensor3dMag | kSensorAhrs;

// MAV_SEVERITY_WARNING = 4；0~4 表示 WARNING 及以上。
constexpr uint8_t kSeverityWarning = 4;

/// 文本里是否出现估计器相关关键字（大小写不敏感）。
/// 只匹配 EKF/estimator：`Arming denied: ...` 这类解锁拒绝属于另一条链路，
/// 交给 STATUSTEXT 日志与起飞前门禁，不在飞行中触发"估计器降级"。
bool HasEstimatorKeyword(const std::string& text) {
    std::string lower;
    lower.reserve(text.size());
    for (char ch : text) {
        lower.push_back(static_cast<char>(
            std::tolower(static_cast<unsigned char>(ch))));
    }
    return lower.find("ekf") != std::string::npos ||
           lower.find("estimator") != std::string::npos;
}

/// 非有限值（NaN/Inf）视为"该路没有有效数据"，不参与比较。
double FiniteOr(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

}  // namespace

const char* EstimatorIssueLabel(EstimatorIssue issue) {
    switch (issue) {
        case EstimatorIssue::kNone:
            return "正常";
        case EstimatorIssue::kVerticalJump:
            return "EKF 高度单拍跳变";
        case EstimatorIssue::kHorizontalJump:
            return "EKF 水平位置单拍跳变";
        case EstimatorIssue::kEstimatorFlag:
            return "估计器状态位异常";
        case EstimatorIssue::kInnovationRatio:
            return "新息检验比超限";
        case EstimatorIssue::kSensorUnhealthy:
            return "关键传感器不健康";
        case EstimatorIssue::kBaroDivergence:
            return "气压高度与 EKF 高度不一致";
        case EstimatorIssue::kStatustext:
            return "PX4 估计器告警文本";
        case EstimatorIssue::kLocalPositionStale:
            return "局部位置遥测超时";
    }
    return "未知判据";
}

EstimatorGuard::EstimatorGuard(EstimatorGuardThresholds thresholds)
    : thresholds_(thresholds) {
    if (thresholds_.critical_sensor_mask == 0) {
        thresholds_.critical_sensor_mask = kCriticalSensorMask;
    }
}

void EstimatorGuard::Reset() {
    const EstimatorGuardThresholds thresholds = thresholds_;
    *this = EstimatorGuard(thresholds);
}

bool EstimatorGuard::Hold(uint64_t now_ms, uint64_t hold_ms, uint64_t* since_ms) {
    if (*since_ms == 0) {
        *since_ms = now_ms;
        return hold_ms == 0;
    }
    if (now_ms < *since_ms) {  // 单调时钟回绕/重启：重新计时
        *since_ms = now_ms;
        return false;
    }
    return now_ms - *since_ms >= hold_ms;
}

void EstimatorGuard::RememberLocalPosition(
    const common::FlightStateSnapshot& state) {
    has_prev_local_ = state.local_position_valid;
    prev_local_stamp_ms_ = state.local_position_time_ms;
    prev_local_x_ = static_cast<double>(state.local_x_m);
    prev_local_y_ = static_cast<double>(state.local_y_m);
    prev_local_z_ = static_cast<double>(state.local_z_m);
}

EstimatorGuardResult EstimatorGuard::Hit(EstimatorIssue issue, uint64_t now_ms,
                                         double magnitude) {
    if (!result_.degraded) {
        result_.degraded = true;
        result_.issue = issue;
        result_.since_ms = now_ms;
        result_.magnitude = magnitude;
    }
    return result_;
}

EstimatorGuardResult EstimatorGuard::Update(
    const common::FlightStateSnapshot& state, uint64_t now_ms, bool armed) {
    // 已锁存：一律返回首次命中的结果，不因后续帧好转而自动恢复。
    if (result_.degraded) {
        return result_;
    }

    // 未上锁：只维护基线。地面上的 EKF 重置无害，上电阶段的对齐抖动也不该报警。
    if (!armed) {
        RememberLocalPosition(state);
        ratio_since_ms_ = 0;
        baro_since_ms_ = 0;
        sensor_since_ms_ = 0;
        statustext_checked_ms_ = state.statustext_time_ms;
        return result_;
    }

    // 1) 局部位置新鲜度：断链/串口丢帧本身就是"不可用"，不再往下用陈旧值判断。
    if (!state.local_position_time_valid ||
        state.local_position_time_ms == 0 ||
        now_ms < state.local_position_time_ms ||
        now_ms - state.local_position_time_ms >
            thresholds_.local_position_stale_ms) {
        return Hit(EstimatorIssue::kLocalPositionStale, now_ms,
                   static_cast<double>(state.local_position_time_ms));
    }

    // 2) 单拍跳变：只在收到新的一帧时比较，且要扣掉速度本身带来的位移。
    if (has_prev_local_ &&
        state.local_position_time_ms != prev_local_stamp_ms_) {
        const double dt_s = static_cast<double>(
                                state.local_position_time_ms -
                                prev_local_stamp_ms_) /
                            1000.0;
        const double dz =
            std::fabs(static_cast<double>(state.local_z_m) - prev_local_z_);
        const double vz = std::fabs(static_cast<double>(state.vz_mps));
        const double allowed_z = vz * dt_s + thresholds_.z_jump_speed_margin_m;
        if (dz > thresholds_.z_jump_m && dz > allowed_z) {
            return Hit(EstimatorIssue::kVerticalJump, now_ms, dz);
        }
        const double dx =
            static_cast<double>(state.local_x_m) - prev_local_x_;
        const double dy =
            static_cast<double>(state.local_y_m) - prev_local_y_;
        const double dxy = std::hypot(dx, dy);
        const double ground_speed = std::hypot(
            static_cast<double>(state.vx_mps), static_cast<double>(state.vy_mps));
        const double allowed_xy =
            ground_speed * dt_s + thresholds_.xy_jump_speed_margin_m;
        if (dxy > thresholds_.xy_jump_m && dxy > allowed_xy) {
            return Hit(EstimatorIssue::kHorizontalJump, now_ms, dxy);
        }
    }
    RememberLocalPosition(state);

    // 3) PX4 文本告警（WARN 及以上里出现 EKF/estimator）。
    // 先判文本：EKF 故障通常先发 STATUSTEXT，再出现位置跳变。
    if (state.statustext_valid &&
        state.statustext_time_ms > statustext_checked_ms_) {
        statustext_checked_ms_ = state.statustext_time_ms;
        if (state.statustext_severity <= kSeverityWarning &&
            HasEstimatorKeyword(state.statustext_text)) {
            return Hit(EstimatorIssue::kStatustext, now_ms,
                       static_cast<double>(state.statustext_severity));
        }
    }

    // 4) ESTIMATOR_STATUS.flags：位语义按固件方言而定，默认掩码 0（关闭）。
    if (thresholds_.bad_estimator_flag_mask != 0 &&
        (state.estimator_flags & thresholds_.bad_estimator_flag_mask) != 0) {
        return Hit(EstimatorIssue::kEstimatorFlag, now_ms,
                   static_cast<double>(state.estimator_flags));
    }

    // 5) 新息检验比：>1 表示该路观测与状态不一致，持续 1s 才判（单帧尖峰不判）。
    const bool ratio_bad =
        state.estimator_status_valid &&
        (FiniteOr(static_cast<double>(state.estimator_pos_vert_ratio), 0.0) >
             thresholds_.ratio_limit ||
         FiniteOr(static_cast<double>(state.estimator_pos_horiz_ratio), 0.0) >
             thresholds_.ratio_limit ||
         FiniteOr(static_cast<double>(state.estimator_vel_ratio), 0.0) >
             thresholds_.ratio_limit);
    if (!ratio_bad) {
        ratio_since_ms_ = 0;
    } else if (Hold(now_ms, thresholds_.ratio_hold_ms, &ratio_since_ms_)) {
        return Hit(EstimatorIssue::kInnovationRatio, now_ms,
                   std::max({FiniteOr(static_cast<double>(
                                 state.estimator_pos_vert_ratio), 0.0),
                             FiniteOr(static_cast<double>(
                                 state.estimator_pos_horiz_ratio), 0.0),
                             FiniteOr(static_cast<double>(
                                 state.estimator_vel_ratio), 0.0)}));
    }

    // 6) 关键传感器健康位：present 里被置位但 health 里被清掉 = 不健康。
    const uint32_t critical_present =
        state.sensors_present & thresholds_.critical_sensor_mask;
    const uint32_t unhealthy =
        critical_present & ~state.sensors_health;
    const bool sensor_bad =
        state.sensors_status_valid && critical_present != 0 && unhealthy != 0;
    if (!sensor_bad) {
        sensor_since_ms_ = 0;
    } else if (Hold(now_ms, thresholds_.sensor_hold_ms, &sensor_since_ms_)) {
        return Hit(EstimatorIssue::kSensorUnhealthy, now_ms,
                   static_cast<double>(unhealthy));
    }

    // 7) 气压 AGL 与 EKF 相对高度交叉校验（两个不同物理路径）。
    const auto baro_agl = common::BaroAboveTakeoffM(state);
    const auto base_z = common::RelativeAltitudeBaseZ(state);
    bool baro_bad = false;
    double divergence = 0.0;
    if (baro_agl && base_z && state.local_position_valid) {
        const double ekf_relative =
            *base_z - static_cast<double>(state.local_z_m);
        divergence = std::fabs(*baro_agl - ekf_relative);
        baro_bad = divergence > thresholds_.baro_divergence_m;
    }
    if (!baro_bad) {
        baro_since_ms_ = 0;
    } else if (Hold(now_ms, thresholds_.baro_hold_ms, &baro_since_ms_)) {
        return Hit(EstimatorIssue::kBaroDivergence, now_ms, divergence);
    }

    return result_;
}

}  // namespace drone::state_machine
