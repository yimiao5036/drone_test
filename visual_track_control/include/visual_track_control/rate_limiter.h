#pragma once

// =============================================================================
// rate_limiter.h —— 限幅与变化率限制（纯算法构件，header-only）
//
// 对应规格：docs/物理追踪思路.md
//   §8   所有阈值集中配置：速度/加速度/方向变化率必须限幅（FR-033）
//   S6   首条视觉速度设定值必须经过限幅，避免位置→速度切换产生突变
//        （状态机设计.md S6）
//
// 特性（热路径零分配、无虚函数、无锁）：
//   - Clamp()：静态限幅；
//   - SlewRateLimiter：保存上次输出，按配置变化率限幅；初值置 0 时
//     天然实现"首条指令突变抑制"（S6）。
// =============================================================================

#include <algorithm>

namespace drone::vtc {

/// 限幅：将 v 钳制在 [lo, hi]（调用方保证 lo <= hi）
inline double Clamp(double v, double lo, double hi) {
    return std::min(std::max(v, lo), hi);
}

/// 变化率限幅器：限制输出每秒最大变化量（单位由调用方决定，
/// 如 度/秒^2 用于航向、米/秒^2 用于速度）
class SlewRateLimiter {
public:
    /// rate_limit：每秒最大变化量（<=0 表示冻结输出）；initial：初始输出值
    explicit SlewRateLimiter(double rate_limit, double initial = 0.0)
        : rate_limit_(rate_limit), output_(initial) {}

    /// 朝 target 推进一个节拍：单步变化不超过 rate_limit·dt_s。
    /// dt_s <= 0 或 rate_limit <= 0 时保持当前输出（异常节拍保护）。
    double Limit(double target, double dt_s) {
        if (dt_s <= 0.0 || rate_limit_ <= 0.0) {
            return output_;
        }
        const double max_step = rate_limit_ * dt_s;
        output_ = Clamp(target, output_ - max_step, output_ + max_step);
        return output_;
    }

    /// 复位输出值（刹车/重置时归零，保证恢复跟踪后指令从零渐变）
    void Reset(double value = 0.0) { output_ = value; }

    /// 上次输出（只读）
    double LastOutput() const { return output_; }

private:
    double rate_limit_;  // 每秒最大变化量
    double output_;      // 当前输出
};

}  // namespace drone::vtc
