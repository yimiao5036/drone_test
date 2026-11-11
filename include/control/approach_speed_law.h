#pragma once

// =============================================================================
// approach_speed_law.h —— S5 接近段速度律（ApproachSpeedLaw）
//
// 来源：docs/接近段速度律-数学分析-2026-11-09.md §6 + 演示页
//       docs/示意图/追踪梯形曲线-交互演示-2026-11-05.html（stepSim 的控制部分）。
// 2026-11-11 起落到机载：S5 不再只下发“目标点 + 固定提前”，而是每拍按
// 相对系制动包络生成 (p_ref, v_ref)，由状态机映射为位置意图 + 速度前馈。
//
// 边界：
//   - 只做水平制导（NED 北/东）与垂直速度通道；姿态/限幅最终仍由
//     FlightController 与 PX4 兜底；
//   - 不读取目标真值：目标位置/速度来自 TargetState，目标加速度由相邻速度
//     样本差分 + 一阶低通得到（与演示页同口径）；
//   - 不包含演示页的 UI/评价量（穿透计数、加速度图、碰撞三角形提前量的
//     stop 模式等），`speed_match=true` 时提前量只补时延 + 距离自适应；
//   - jerk 二阶平滑未移植（演示页默认 J=0），指令限幅为一阶 slew。
//
// 线程模型：单线程调用（状态机决策线程），成员无锁、无堆对象。
// =============================================================================

#include <cstdint>

namespace drone::control {

/// 接近段速度律配置（approach.speed_law 节；缺席时用默认值）。
struct ApproachSpeedLawConfig {
    /// 控制间距 R（米）：目标到本机的保持距离，默认 3.0 − 0.1 = 2.9。
    double standoff_m = 2.9;
    /// 远段规划加/减速度（m/s²）；真实下发受 slew 与 a_phys 封顶。
    double accel_plan_mps2 = 3.0;
    double decel_plan_mps2 = 3.0;
    /// 机体物理加速度上限（m/s²，PX4 45° 倾角参考≈9.8），只封顶。
    double accel_phys_mps2 = 9.8;
    /// 近段/末端规划加速度下限（8 m 内生效，8~12 m 过渡）。
    double accel_term_mps2 = 1.5;
    /// 指令加速度上限（m/s²，下发力约束）；0=不限制。
    double slew_mps2 = 3.0;
    /// 机体总速度上限（m/s）。
    double speed_cap_mps = 10.0;
    /// 制动安全系数：减速度先除以该值，吸收离散化越界。
    double brake_safety = 1.3;
    /// 末端比例收敛增益（1/s）。
    double terminal_gain = 1.5;
    /// 距离自适应提前量：lead = min(k·(d−R), max) + extra。
    double lead_scaled_k = 0.06;
    double lead_scaled_max_m = 2.0;
    double lead_extra_m = 0.0;
    /// 额外提前量上限（秒）：碰撞三角形路径单次最多提前量。
    double t_lead_max_s = 0.5;
    /// 目标急停预留（m/s²）：0=不预留；>0 时按目标最大内向加速度收紧制动预算。
    double target_brake_guard_mps2 = 0.0;
    /// 末端速度匹配：true=末端相对速度收敛到 0（跟住目标）；false=末端停车。
    bool speed_match = true;
    /// 参考点前瞻：p_ref = 自机 + v_ref × ref_lead_s（上限 ref_lead_max_m）。
    /// 位置环只做“速度指令的兜底锚点”，不再把目标点本身当成位置目标——
    /// 否则位置误差会在制动段额外加速，导致冲进控制间距（2026-11-11 实测）。
    double ref_lead_s = 0.3;
    double ref_lead_max_m = 6.0;
    /// 停滞自动提能力（演示页默认开）：停滞/能力受限/穿透持续 boost_delay 后，
    /// 规划加速度按 boost_rate 抬向 a_phys。
    bool auto_boost = true;
    double boost_delay_s = 4.0;
    double boost_rate_mps2_per_s = 0.5;
    /// 目标机动需求峰值保持（演示页恒开）：近段下限不低于 |â_t|+0.5。
    bool target_need_hold = true;
    /// 垂直通道：速度/加速度上限与末端余量（米）。
    double vertical_speed_cap_mps = 2.0;
    double vertical_accel_cap_mps2 = 2.0;
    double vertical_margin_m = 0.2;

    /// 跨字段/取值校验；非法抛 std::invalid_argument（启动期 fail-fast）。
    void Validate() const;
};

/// 控制律输入（局部 NED；z 向下为正）。
struct ApproachInput {
    bool own_valid = false;
    bool target_valid = false;
    double own_n_m = 0.0;
    double own_e_m = 0.0;
    double own_d_m = 0.0;
    double own_vn_mps = 0.0;
    double own_ve_mps = 0.0;
    double own_vd_mps = 0.0;
    double target_n_m = 0.0;
    double target_e_m = 0.0;
    double target_d_m = 0.0;
    double target_vn_mps = 0.0;
    double target_ve_mps = 0.0;
    double target_vd_mps = 0.0;
    /// 目标样本时刻（单调毫秒）；用于时延补偿与目标加速度差分。
    std::int64_t target_sample_ms = 0;
};

/// 接近阶段（日志/诊断用，不参与控制分支）。
enum class ApproachPhase : std::uint8_t {
    kIdle = 0,
    kAccel,
    kCruise,
    kDecel,
    kMatch,
};

const char* ApproachPhaseName(ApproachPhase phase);

/// 控制律输出。
struct ApproachOutput {
    bool valid = false;
    /// 期望速度（局部 NED，m/s）：直接作为位置意图的速度前馈下发。
    double v_n_mps = 0.0;
    double v_e_mps = 0.0;
    double v_d_mps = 0.0;
    /// 参考点（局部 NED，m）：目标沿视线后退 standoff_m 的保持点，
    /// 作为位置意图的落点；速度前馈与它同源。
    double ref_n_m = 0.0;
    double ref_e_m = 0.0;
    double ref_d_m = 0.0;
    /// 水平可见距离（米）与当前接近速度（m/s，正=在接近）。
    double distance_m = 0.0;
    double closing_mps = 0.0;
    ApproachPhase phase = ApproachPhase::kIdle;
    /// 诊断：目标机动/侧向预算把能力吃满；制动预算不可行；已进入穿透区。
    bool capability_limited = false;
    bool brake_infeasible = false;
    bool penetrated = false;
};

/// S5 接近段速度律。
class ApproachSpeedLaw {
public:
    explicit ApproachSpeedLaw(const ApproachSpeedLawConfig& config);

    /// 单周期计算；now_ms 为单调时钟毫秒（用于 dt 与目标时延）。
    ApproachOutput Update(const ApproachInput& input, std::int64_t now_ms);

    /// 清全部状态（交接/任务重置时调用）。
    void Reset();

private:
    ApproachSpeedLawConfig cfg_;
    bool initialized_ = false;
    std::int64_t last_now_ms_ = 0;
    // 上一拍目标速度样本（差分目标加速度）
    std::int64_t last_target_sample_ms_ = 0;
    double last_target_vn_mps_ = 0.0;
    double last_target_ve_mps_ = 0.0;
    double target_acc_n_mps2_ = 0.0;
    double target_acc_e_mps2_ = 0.0;
    // 目标机动需求峰值保持
    double target_need_hold_ = 0.0;
    // 分相位过渡权重（0=近段 a_term，1=远段 a_plan）
    double schedule_w_ = 1.0;
    // 指令速度（一阶 slew 状态）
    double cmd_vn_mps_ = 0.0;
    double cmd_ve_mps_ = 0.0;
    double cmd_vd_mps_ = 0.0;
    // 停滞/提能力/穿透
    double best_distance_m_ = 0.0;
    std::int64_t last_progress_ms_ = 0;
    double boost_time_s_ = 0.0;
    double accel_boost_mps2_ = 0.0;
    bool penetrated_ = false;
    bool capability_limited_ = false;
};

}  // namespace drone::control
