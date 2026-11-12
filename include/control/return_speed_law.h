/**
 * @file return_speed_law.h
 * @brief S12 返航水平速度剖面（纯函数，无状态、无日志、无线程）
 *
 * 属于 drone/control 模块。职责：按“分段恒定加速度”生成返航水平速度指令，
 * 覆盖三种必须区分的情形（2026-11-12 维护者裁决）：
 *
 *   1. **起步加速**：进入返航时飞机可能静止（目标未运动/目标已掉落，飞机
 *      已经停下），需要从当前速度斜坡加速到巡航速度；
 *   2. **超速减速**：始终追不上目标而转入返航时，飞机的当前速度可能**高于**
 *      巡航速度，必须先把速度斜坡降回巡航速度，不能直接按巡航指令下发
 *      （那会让 PX4 用自身限幅去吸收，表现为急停）；
 *   3. **末端制动**：接近目标时按制动包络 `v ≤ √(2·a·(d−R))` 收敛，并在
 *      `final_zone_m` 内把顶速降为 `final_speed_mps`，让到达判据自然满足。
 *
 * 明确**不做**的事：
 *   - 不做 jerk（二阶）平滑——只做分段恒定加速度（维护者：不需要变加速度）；
 *   - 不做“追不上就持续加速”的能力提升（`ApproachSpeedLaw::auto_boost` 是
 *     为追运动目标设计的；返航目标是固定点，永久可达，任何加速补偿都是逻辑
 *     错误）。停滞只由状态机打 WARN，不改速度剖面；
 *   - 不读配置、不读时间：所有参数由调用方给出，纯函数便于单测。
 *
 * 坐标系：局部 NED 水平面内的**标量速率**（m/s，正=在靠近目标）。
 * 实现文档见同目录 return_speed_law.md。
 */
#pragma once

#include <cstdint>

namespace drone::control {

/// 输出速度下限（m/s）。位置引导把 `max_speed_mps==0` 解释为"未填相位
/// 上限"并回退控制器硬上限，所以有效输出**恒大于 0**：末端制动包络收敛到 0
/// 时仍给该下限，避免最后几米突然解除限速、被 PX4 位置环一冲（2026-11-12 复审）。
constexpr double kReturnSpeedMinCapMps = 0.1;

/// 返航速度剖面参数（由 `return_home` 配置节映射，状态机注入）。
struct ReturnSpeedLawConfig {
    /// 巡航顶速（m/s）：水平返航段的最大速度，独立于追踪速度。
    double cruise_mps = 4.0;
    /// 规划加速度（m/s²）：起步/恢复时向巡航顶速逼近的斜率。
    /// 0 = 不做加速斜坡（直通巡航顶速）。
    double accel_mps2 = 1.5;
    /// 规划减速度（m/s²）：超速时回落到巡航顶速、以及末端制动包络的斜率。
    /// 0 = 不做减速斜坡（直通，仅保留末端包络）。
    double decel_mps2 = 2.0;
    /// 末端制动包络的安全系数（>1）：包络内减速先除以该值，吸收离散化。
    double brake_safety = 1.3;
    /// 末端低速档（m/s）：进入 `final_zone_m` 后顶速降为 max(final_speed_mps,
    /// 制动包络)。
    double final_speed_mps = 1.0;
    /// 末端低速档的作用距离（m）：距目标小于该值才启用低速档。
    double final_zone_m = 10.0;
    /// 到达半径（m）：制动包络以 `d − arrival_radius_m` 为剩余可用距离，
    /// 使速度在进入到达半径时自然收敛到 0 附近。
    double arrival_radius_m = 2.0;
};

/// 单步速度剖面状态（跨周期保持的仅"上一拍指令速率"。
/// 其余全部来自实时测量，避免积分漂移）。
struct ReturnSpeedState {
    /// 上一拍下发的水平速率（m/s）；0 且 `initialized=false` 表示尚未起步。
    double prev_speed_mps = 0.0;
    bool initialized = false;
};

/// 剖面分级（日志/诊断用，不参与控制分支）。
enum class ReturnSpeedPhase : std::uint8_t {
    kIdle = 0,     ///< 未启用（无有效输入）
    kAccel,        ///< 起步加速：向巡航顶速逼近
    kCruise,       ///< 巡航：已到顶速，剩余距离充足
    kDecel,        ///< 超速回落：当前速度高于本拍允许速度
    kBrake,        ///< 末端制动包络：剩余距离不足，按 √(2ad) 收敛
    kFinal,        ///< 末端低速档
};

const char* ReturnSpeedPhaseName(ReturnSpeedPhase phase);

/// 单步输入（全部为实时测量，非累积量）。
struct ReturnSpeedInput {
    /// 是否有有效输入（无局部位置/无目标时必须为 false）。
    bool valid = false;
    /// 距返航目标的水平剩余距离（m，>=0）。
    double remaining_m = 0.0;
    /// 当前实测水平地速（m/s，>=0）。起步/超速判定都用它，而不是上一拍指令：
    /// 机体响应滞后不会让剖面"以为自己已经在巡航"。
    double ground_speed_mps = 0.0;
    /// 控制周期（秒）：斜坡步长 = accel/decel × dt。
    double dt_s = 0.05;
};

/// 单步输出。
struct ReturnSpeedOutput {
    bool valid = false;
    /// 本拍允许的水平速率上限（m/s）：交给控制器作为位置引导限速。
    /// 有效输出恒 ≥ `kReturnSpeedMinCapMps`（0 在控制器侧表示"未填上限"）。
    double speed_mps = 0.0;
    ReturnSpeedPhase phase = ReturnSpeedPhase::kIdle;
    /// 诊断：本拍是否被制动包络（含末端低速档）压住。
    bool brake_limited = false;
};

/// 推进一拍返航速度剖面。
///
/// 语义（按此顺序，任一都可能进一步压低本拍指令）：
///   ① 顶速 = cruise_mps；进入 final_zone_m 后顶速 = final_speed_mps。
///   ② 制动包络 = max(0, √(2·(decel/brake_safety)·max(0, d−arrival_radius)))；
///      顶速取 min(顶速, 包络)。
///   ③ 斜坡：从**当前实测地速**出发，按 accel/decel 向②的顶速收敛：
///      - 目标顶速 > 当前地速：以 accel_mps2 上升（起步加速）；
///      - 目标顶速 < 当前地速：以 decel_mps2 下降（超速回落/末端制动）。
///   ④ 输出 speed_mps 上界为 max(cruise_mps, 实测地速)：加速/巡航段恒不高于
///      巡航顶速，减速段允许指令暂时高于 cruise（那是正在衰减的速度，钳到
///      cruise 会让超速回落变成瞬跳、斜坡失效）。下界为
///      min(kReturnSpeedMinCapMps, cruise_mps)：0 在控制器侧是"未填相位上限"
///      并回退硬上限，不能让末端包络把限速整个解除。
///
/// `accel_mps2=0` 或 `decel_mps2=0` 时对应方向不设斜坡（直通）；
/// `dt_s<=0`（首拍/时钟回退）不推进斜坡、不跳变，保持实测地速。
/// 输入无效（valid=false）返回 valid=false 且不改变 state（下拍可续）。
ReturnSpeedOutput ComputeReturnSpeed(const ReturnSpeedLawConfig& config,
                                     const ReturnSpeedInput& input,
                                     ReturnSpeedState& state);

/// 复位剖面状态（进入/重启 S12 时调用）。
void ResetReturnSpeedState(ReturnSpeedState& state);

}  // namespace drone::control
