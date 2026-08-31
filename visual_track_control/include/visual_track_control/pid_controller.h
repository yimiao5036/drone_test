#pragma once

// =============================================================================
// pid_controller.h —— 位置式 PID 控制器（纯算法构件）
//
// 对应规格：docs/物理追踪思路.md
//   §6.2 距离控制律：前向速度指令 = PID(Δd)，Δd = d − d_exp
//   §8   PID 参数（Kp/Ki/Kd、积分限幅、微分滤波）全部待定/配置化
//
// 特性（热路径零分配、无虚函数、无锁）：
//   - 位置式输出：out = Kp·e + Ki·∫e + Kd·d(e)/dt；
//   - 积分钳位：积分项限幅在 ±integral_limit；
//   - anti-windup（条件积分）：输出饱和且本拍积分增量仍推深饱和时冻结积分，
//     误差反号后允许积分回卷，防止积分饱和与残留；
//   - 微分一阶低通：derivative_filter_coef ∈ [0,1)，0 = 不滤波；
//   - ResetIntegral() / Reset() 均 O(1)（Coast 清积分，§6.4）。
// =============================================================================

namespace drone::vtc {

/// PID 参数（全部来自配置，代码不写死任何数值）
struct PidParams {
    double kp = 0.0;  // 比例增益
    double ki = 0.0;  // 积分增益
    double kd = 0.0;  // 微分增益
    double integral_limit = 0.0;        // 积分项限幅（<=0 表示不限幅）
    double derivative_filter_coef = 0.0;  // 微分一阶低通系数 ∈ [0,1)：越大滤波越强
    double output_limit = 0.0;          // 输出限幅（<=0 表示不限幅）
};

/// 位置式 PID 控制器
class PidController {
public:
    explicit PidController(const PidParams& params);

    /// 单步更新：输入误差与采样间隔（秒），返回限幅后的输出。
    /// dt_s <= 0 时保持上次输出（异常节拍保护）。
    double Update(double error, double dt_s);

    /// 清空积分项（§6.4：Coast 时防止积分饱和）；保留微分历史与上次输出
    void ResetIntegral();

    /// 清空全部内部状态（积分、微分历史、上次输出）
    void Reset();

    /// 上次输出（只读）
    double LastOutput() const { return last_output_; }

private:
    PidParams params_;

    double integral_ = 0.0;            // 积分累计（已钳位）
    double prev_error_ = 0.0;          // 上次误差（微分用）
    double filtered_derivative_ = 0.0; // 低通后的微分值
    double last_output_ = 0.0;         // 上次输出
    bool has_prev_error_ = false;      // 是否存在有效微分历史
};

}  // namespace drone::vtc
