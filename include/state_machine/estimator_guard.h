/**
 * @file estimator_guard.h
 * @brief 估计器健康判据（降落与估计器保护 P1）
 *
 * 背景：2026-11-17 10:24 现场 EKF 全状态重置（delta_z≈9m）+ fs_bad_acc_vertical，
 * AUTO.LAND 过程中姿态设定值外扩到 45° 后翻滚。当时机载对估计器状态**零可见性**：
 * 没有 EKF 重置检测、没有传感器健康判据、也没有 PX4 文本告警（"Arming denied: ..."）
 * 的通道，因此"7 秒的止损窗口"里我们什么也做不了。
 *
 * 本模块只做**判据**：输入一帧 PX4 快照，输出"是否已判为降级"与首个命中的原因。
 * 不切模式、不发指令、不持有 Topic —— 处置（P3：切 POSCTL 悬停 + 告警）由状态机决定。
 * 单独成模块是为了可被单测覆盖：这些阈值和极性错了就是漏报或误报，必须能回归。
 *
 * 判据分三类（见 docs/降落与估计器保护设计-2026-11-17.md §3.3）：
 * 1. **瞬时跳变**：EKF local z/xy 单拍跳变（本次事故的 9m 就属这类，一帧命中）；
 * 2. **持续不一致**：气压 AGL 与 EKF 相对高度差、新息检验比超限、关键传感器健康位掉线；
 * 3. **兜底文本**：STATUSTEXT 里出现 EKF/estimator 关键字（WARN 及以上）。
 *
 * 关于 ESTIMATOR_STATUS.flags：不同 MAVLink 方言的位号不同（本项目 third_party 里是
 * "置位=该路可用"的旧语义，而部分固件的 ESTIMATOR_STATUS 用另一套位定义）。**位掩码
 * 默认全 0（关闭）**，等实机确认 PX4 1.17 的实际位语义后再由 config 打开；在此之前
 * 靠数值判据，避免"按错的极性判据报警"这种更危险的情况。
 */
#pragma once

#include <cstdint>

#include "common/types.h"

namespace drone::state_machine {

/// 判据原因（首个命中项，锁存后不再变化）。
enum class EstimatorIssue : uint8_t {
    kNone = 0,
    kVerticalJump,        ///< EKF local z 单拍跳变
    kHorizontalJump,      ///< EKF local xy 单拍跳变
    kEstimatorFlag,       ///< ESTIMATOR_STATUS.flags 命中配置的掩码
    kInnovationRatio,     ///< 新息检验比持续 > 门限
    kSensorUnhealthy,     ///< SYS_STATUS 关键传感器健康位持续掉线
    kBaroDivergence,      ///< 气压 AGL 与 EKF 相对高度持续不一致
    kStatustext,          ///< STATUSTEXT 出现 EKF/估计器关键字（WARN 及以上）
    kLocalPositionStale,  ///< LOCAL_POSITION_NED 长时间未更新
};

/// 判据中文文案（日志与事件用，禁止在别处另写一份）。
const char* EstimatorIssueLabel(EstimatorIssue issue);

/// 判据阈值。P1 用头文件默认值；P2 接 config.landing 节时外置（见设计文档 §4.7）。
struct EstimatorGuardThresholds {
    /// 单拍 z 跳变门限。本次现场是约 9m；0.5m 对 EKF 重置足够灵敏。
    double z_jump_m = 0.5;
    /// |vz|·dt 之外的余量：高速下降/爬升本身就有单拍位移，不能当跳变。
    double z_jump_speed_margin_m = 0.1;
    double xy_jump_m = 1.0;
    double xy_jump_speed_margin_m = 0.3;
    /// 归一化新息检验比门限（PX4 口径 >1 视为该路不可信）。
    double ratio_limit = 1.0;
    uint64_t ratio_hold_ms = 1000;
    /// 气压 AGL 与 EKF 相对高度允许的偏差，以及需要持续多久。
    double baro_divergence_m = 1.5;
    uint64_t baro_hold_ms = 2000;
    /// 关键传感器健康位需要持续掉线多久才判降级（防 PX4 健康位抖动）。
    uint64_t sensor_hold_ms = 1000;
    /// LOCAL_POSITION_NED 新鲜度门限。
    uint64_t local_position_stale_ms = 500;
    /// 关键传感器掩码（MAV_SYS_STATUS_SENSOR_*）。0 表示关闭该判据；
    /// 默认由构造函数填 GYRO|ACCEL|MAG|AHRS。
    uint32_t critical_sensor_mask = 0;
    /// ESTIMATOR_STATUS.flags 掩码：置位的位视为退化。0 = 关闭（见文件头说明）。
    uint16_t bad_estimator_flag_mask = 0;
};

/// 判据结果。
struct EstimatorGuardResult {
    bool degraded = false;
    EstimatorIssue issue = EstimatorIssue::kNone;
    uint64_t since_ms = 0;     ///< 首次命中时刻（单调时钟）
    double magnitude = 0.0;    ///< 触发时的关键数值（跳变量/比值/偏差），供日志与告警
};

class EstimatorGuard {
public:
    explicit EstimatorGuard(EstimatorGuardThresholds thresholds = {});

    /// 喂一帧快照。只有 `armed` 为真时判据才生效：
    /// 地面上的 EKF 重置无害，且上电阶段的对齐抖动会被误报成降级。
    /// 未上锁时只刷新基线（上一拍位置），不置位。
    ///
    /// 判据一旦置位即**锁存**，不自动恢复——避免抖动下反复切模式；
    /// 清理由"重置任务状态"或程序重启完成。
    EstimatorGuardResult Update(const common::FlightStateSnapshot& state,
                               uint64_t now_ms, bool armed);

    bool degraded() const { return result_.degraded; }
    const EstimatorGuardResult& result() const { return result_; }
    void Reset();

private:
    /// 记录一次命中并锁存；已锁存时直接返回缓存结果。
    EstimatorGuardResult Hit(EstimatorIssue issue, uint64_t now_ms, double magnitude);
    /// 持续型判据的滞回：首次满足记时刻，满足超过 hold 才返回 true。
    bool Hold(uint64_t now_ms, uint64_t hold_ms, uint64_t* since_ms);
    void RememberLocalPosition(const common::FlightStateSnapshot& state);

    EstimatorGuardThresholds thresholds_;
    EstimatorGuardResult result_;
    bool has_prev_local_ = false;
    uint64_t prev_local_stamp_ms_ = 0;
    double prev_local_x_ = 0.0;
    double prev_local_y_ = 0.0;
    double prev_local_z_ = 0.0;
    uint64_t ratio_since_ms_ = 0;
    uint64_t baro_since_ms_ = 0;
    uint64_t sensor_since_ms_ = 0;
    uint64_t statustext_checked_ms_ = 0;
};

}  // namespace drone::state_machine
