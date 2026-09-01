// =============================================================================
// tracking_control_law.cpp —— 视觉跟踪主控律实现
//
// 对应规格：docs/物理追踪思路.md
//   §4.1/§4.2 水平航向控制：像素偏移 → β → 增益 → slew → 角速度限幅，
//             位置式输出 yaw_deg / 速率式输出 yaw_rate_dps（按配置）
//   §5.1/§5.2 垂直控制：统一速度支路 α → 垂直速度设定值（原"小误差姿态
//             微调/大误差垂直速度"双支路已合并：姿态微调会把输出切成姿态
//             消息，丢弃同周期距离通道前向速度导致逼近停滞，详见
//             vtc_types.h ControlMode 注释）
//   §6.2/§6.3 距离通道：Δd = d − d_exp → PID → 前向速度（限幅）；
//             未关联/超龄 → 按配置降级动作（hold / slow_approach / exit）
//   §6.4      Coast（is_predicted）清航向/距离积分；判丢（!tracked）零速刹车
//   §7.1      超龄数据只触发降级：姿态过期 → 禁止控制；视觉过期 → 降级保持
//   §8        所有输出统一限幅；首条指令经变化率限幅（S6）
//   §2.3      输出端随机体航向把机体系速度旋转为局部 NED
//
// 日志纪律：仅状态切换点（刹车进出、降级切换）打印；热路径零日志、零分配。
// =============================================================================

#include "visual_track_control/tracking_control_law.h"

#include <algorithm>
#include <cmath>

#include <spdlog/spdlog.h>

#include "visual_track_control/frame_transform.h"
#include "visual_track_control/staleness_checker.h"

namespace drone::vtc {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kMaxDtS = 0.5;  // 单步 dt 上限（秒）：异常节拍保护

/// 角度回绕到 [-180, 180)（位置式绝对航向合成用）
double WrapDeg180(double deg) {
    double d = std::fmod(deg + 180.0, 360.0);
    if (d < 0.0) {
        d += 360.0;
    }
    return d - 180.0;
}

/// 控制模式可读名（仅状态切换日志使用）
const char* ModeName(ControlMode mode) {
    switch (mode) {
        case ControlMode::kVelocityHeading: return "速度航向";
        case ControlMode::kBrakeHover:      return "零速刹车";
        case ControlMode::kDegradedHold:    return "降级保持";
    }
    return "未知";
}

/// 由距离配置构造 PID 参数（§6.2）；输出限幅取接近/后退限幅的较大者
PidParams BuildDistancePidParams(const TrackingConfig& cfg) {
    PidParams p;
    p.kp = cfg.distance.kp;
    p.ki = cfg.distance.ki;
    p.kd = cfg.distance.kd;
    p.integral_limit = cfg.distance.integral_limit;
    p.derivative_filter_coef = cfg.distance.derivative_filter_coef;
    p.output_limit = std::max(cfg.distance.approach_velocity_limit_mps,
                              cfg.distance.retreat_velocity_limit_mps);
    return p;
}

}  // namespace

TrackingControlLaw::TrackingControlLaw(const TrackingConfig& cfg)
    : cfg_(cfg),
      pixel_to_beta_(cfg.camera.image_width, cfg.camera.fov_h_deg * kDegToRad),
      pixel_to_alpha_(cfg.camera.image_height, cfg.camera.fov_v_deg * kDegToRad),
      distance_pid_(BuildDistancePidParams(cfg)),
      yaw_deg_limiter_(cfg.heading.yaw_slew_limit_dps2),
      yaw_rate_limiter_(cfg.heading.yaw_slew_limit_dps2),
      vx_limiter_(cfg.accel_limit.ax_mps2),
      vz_limiter_(cfg.accel_limit.az_mps2) {}

ControlOutput TrackingControlLaw::Update(const ControlSnapshot& snapshot,
                                         std::int64_t now_ms) {
    // ---- 节拍推算（§7.1 固定周期；异常节拍用配置频率兜底） ----
    // 时钟回拨或 now_ms 未前进（同值）时不信任差值，回退配置频率：
    // 单拍误差由限幅器与超龄判定兜底，不引入额外状态。
    double dt_s = 1.0 / cfg_.control.frequency_hz;
    if (last_now_ms_ > 0 && now_ms > last_now_ms_) {
        dt_s = std::min(static_cast<double>(now_ms - last_now_ms_) / 1000.0, kMaxDtS);
    }
    last_now_ms_ = now_ms;

    ControlOutput out;
    out.mode = ControlMode::kVelocityHeading;  // 正常路径默认模式，分支可覆盖

    // 状态切换日志辅助：仅模式变化时打印（刹车进出、降级切换等）
    auto FinishAndLog = [&](const char* reason) {
        if (out.mode != last_mode_) {
            spdlog::info("[ControlLaw] 状态切换: {} -> {} ({})",
                         ModeName(last_mode_), ModeName(out.mode), reason);
            last_mode_ = out.mode;
        }
        return out;
    };

    // ---- ① 超龄判定（§7.1 / §9.2） ----
    // 姿态过期或缺失：禁止空间转换和控制，立即退出正常控制（§9.2）
    if (!snapshot.attitude_present ||
        StalenessChecker::IsStale(snapshot.attitude_time_ms, now_ms,
                                  cfg_.control.attitude_stale_ms)) {
        out.mode = ControlMode::kDegradedHold;
        out.degraded_reason = DegradedReason::kAttitudeStale;
        return FinishAndLog("姿态过期/缺失，禁止控制");
    }
    // 视觉追踪结果超龄：不得继续生成正常指令（§7.1）
    if (StalenessChecker::IsStale(snapshot.track_time_ms, now_ms,
                                  cfg_.control.visual_stale_ms)) {
        out.mode = ControlMode::kDegradedHold;
        out.degraded_reason = DegradedReason::kVisionStale;
        return FinishAndLog("视觉追踪结果超龄");
    }

    // ---- ② 判丢 → 零速刹车（§6.4：tracked == false 执行 kBrakeHover） ----
    if (!snapshot.track.tracked) {
        if (last_mode_ != ControlMode::kBrakeHover) {
            // 进入刹车：清积分与限幅器历史，恢复跟踪后指令从零渐变（S6 首条限幅）
            distance_pid_.Reset();
            yaw_deg_limiter_.Reset();
            yaw_rate_limiter_.Reset();
            vx_limiter_.Reset();
            vz_limiter_.Reset();
        }
        out.mode = ControlMode::kBrakeHover;  // 速度分量保持默认全零
        out.valid = true;
        return FinishAndLog("判丢，零速刹车");
    }

    // ---- ③ Coast：is_predicted == true → 清积分防饱和（§6.4） ----
    if (snapshot.track.is_predicted) {
        distance_pid_.ResetIntegral();
        // 航向通道为纯比例增益映射（§4.2），无积分项，无需清理
        out.coast_active = true;
    }

    // ---- ④ 像素偏移 → 视线角（§4.1 水平 β / §5.1 垂直 α） ----
    // x_offset = center.first − W/2（右正），y_offset = center.second − H/2（下正）
    const double x_offset =
        snapshot.track.center.first - cfg_.camera.image_width * 0.5;
    const double y_offset =
        snapshot.track.center.second - cfg_.camera.image_height * 0.5;
    const double beta_deg = pixel_to_beta_.Convert(x_offset) * kRadToDeg;
    const double alpha_deg = pixel_to_alpha_.Convert(y_offset) * kRadToDeg;

    // ---- ⑤ 水平航向通道（§4.2：增益 → slew → 角速度限幅） ----
    if (cfg_.heading.mode == HeadingMode::kRate) {
        // 速率式：β → 偏航角速度指令，先变化率限幅再角速度限幅（FR-033）
        const double target_rate = cfg_.heading.gain * beta_deg;
        const double slewed = yaw_rate_limiter_.Limit(target_rate, dt_s);
        out.yaw_rate_dps = Clamp(slewed, -cfg_.heading.yaw_rate_limit_dps,
                                 cfg_.heading.yaw_rate_limit_dps);
    } else {
        // 位置式：β → 航向修正量，变化率限幅（方向变化率上限，FR-033）后
        // 与当前航向合成为绝对 NED 航向——ID 84 的 yaw 字段是绝对值，
        // 直接发修正量会让机体转向"修正角"方向（A2 修复）。姿态已在 ①
        // 通过超龄校验，yaw_rad 可用。
        const double target_yaw = cfg_.heading.gain * beta_deg;
        const double correction_deg = yaw_deg_limiter_.Limit(target_yaw, dt_s);
        out.yaw_deg = WrapDeg180(snapshot.yaw_rad * kRadToDeg + correction_deg);
    }

    // ---- ⑥ 垂直速度通道（§5：统一速度支路） ----
    // 符号约定：α 下正（目标偏低）→ NED 下正 → vz 为正（下降趋近目标高度）。
    // 小误差自然映射为小垂直速度：无需单独的切换阈值与姿态微调支路，
    // 输出模式保持 kVelocityHeading，距离通道前向速度不会被丢弃（A1 修复）。
    {
        const double vz_target = cfg_.vertical.vz_gain_mps_per_deg * alpha_deg;
        const double slewed = vz_limiter_.Limit(vz_target, dt_s);  // 加速度限幅（§8）
        out.vz_mps = Clamp(slewed, -cfg_.vertical.vz_limit_mps,
                           cfg_.vertical.vz_limit_mps);
    }

    // ---- ⑦ 距离通道（§6.2 / §6.3） ----
    // 雷达距离不得直接当目标距离：仅关联成立且未超龄才可用（§6.3、FR-041）
    const bool distance_available =
        snapshot.associated_to_target && snapshot.radar_distance_m > 0.0 &&
        !StalenessChecker::IsStale(snapshot.radar_time_ms, now_ms,
                                   cfg_.control.radar_stale_ms);
    if (distance_available) {
        // Δd = d − d_exp：d > d_exp 前向接近，d < d_exp 减速/后退（§6.2）
        const double delta_d = snapshot.radar_distance_m - cfg_.distance.d_exp_m;
        const double vx_raw = distance_pid_.Update(delta_d, dt_s);
        const double slewed = vx_limiter_.Limit(vx_raw, dt_s);  // 加速度限幅（§8）
        out.vx_mps = Clamp(slewed, -cfg_.distance.retreat_velocity_limit_mps,
                           cfg_.distance.approach_velocity_limit_mps);
    } else {
        // 无可用目标距离：清积分防饱和，按配置降级动作（§6.3）
        distance_pid_.ResetIntegral();
        switch (cfg_.distance.no_distance_action) {
            case NoDistanceAction::kHold:
                // 保持：前向指令清零（安全占位语义；接入主工程后可改为保持当前速度）
                out.vx_mps = 0.0;
                vx_limiter_.Reset(0.0);
                break;
            case NoDistanceAction::kSlowApproach:
                // 限速接近：缓速前向，同样经加速度限幅
                out.vx_mps = vx_limiter_.Limit(
                    cfg_.distance.no_distance_approach_limit_mps, dt_s);
                break;
            case NoDistanceAction::kExit:
                // 退出近距操作：整体降级保持
                out.mode = ControlMode::kDegradedHold;
                out.degraded_reason = DegradedReason::kNoDistanceExit;
                return FinishAndLog("距离未关联，退出近距操作");
        }
    }

    // ---- ⑧ 机体系 → 局部 NED 旋转（§2.3，A4 修复） ----
    // 到此为止各通道限幅均在机体系完成（旋转为正交变换，合速度不变，
    // 限幅约束不被破坏）；输出端随快照航向旋转为北/东/地，与网关的
    // MAV_FRAME_LOCAL_NED 一致。姿态已在 ① 通过超龄校验，yaw_rad 可用。
    RotateBodyToNed(out.vx_mps, out.vy_mps, out.vz_mps, snapshot.yaw_rad);

    out.valid = true;
    return FinishAndLog(out.coast_active ? "Coast 预测跟踪" : "正常跟踪");
}

void TrackingControlLaw::Reset() {
    // 停止视觉闭环时清全部状态（§6.4）
    distance_pid_.Reset();
    yaw_deg_limiter_.Reset();
    yaw_rate_limiter_.Reset();
    vx_limiter_.Reset();
    vz_limiter_.Reset();
    last_now_ms_ = 0;
    last_mode_ = ControlMode::kDegradedHold;
}

}  // namespace drone::vtc
