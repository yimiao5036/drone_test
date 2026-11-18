#pragma once

// =============================================================================
// interception_device.h —— 通用拦截设备接口（IInterceptionDevice）
//
// 2026-11-11 由 net_actuator.h 抽象而来：状态机与 GcsNetFire 不再依赖
// “捕网执行器”这一具体形态，而是依赖通用拦截设备接口。设备形态用
// InterceptMode 表达：
//   - kPulse：一次性动作（捕网发射）——执行 → 保持 → 复位到待机；
//   - kHold：持续作用（干扰）——开始 → 持续作用 → 停止；
//   - kNone：无执行器（撞击）——末端由飞控直接执行，设备层不产生命令。
// 当前只实现 kPulse（single_pwm 捕网枪），kHold/kNone 为后续预留；新增实现
// 在工厂按 actuator.type 接入，状态机 S9/S10/S11 流程不需要改动。
//
// 返回语义：Execute()/Stop() 返回 true=命令已成功入队（不代表机构已动作）；
// false=链路未就绪/命令队列满/状态不允许，调用方保留 pending 并在下一拍重试。
// =============================================================================

#include <cstdint>
#include <memory>

#include "common/types.h"

namespace drone {

namespace communication {
class IPx4CommandSender;
}

namespace config {
struct ActuatorConfig;
}

namespace state_machine {

/// 拦截动作形态：决定 S9/S10 的时序语义，不绑定具体执行机构。
enum class InterceptMode : std::uint8_t {
    kPulse = 0,  ///< 一次性动作：执行一次后按内部时序复位（捕网）
    kHold = 1,   ///< 持续作用：开始后保持到显式停止（干扰，预留）
    kNone = 2,   ///< 无执行器：末端由飞控执行（撞击，预留）
};

const char* InterceptModeName(InterceptMode mode);

/// 设备每拍反馈：PX4 连接状态、命令 ACK 快照与调用方时钟。
///
/// 由状态机在决策拍组装并调用 `IInterceptionDevice::Update()`；设备内部时序
/// （触发确认保持 → 复位 → 复位稳定）全部由设备自己推进，调用方只负责转发
/// 反馈并观察 `Ready()/Triggered()/Faulted()`。
struct InterceptFeedback {
    bool px4_connected = false;      ///< PX4 链路当前是否在线
    common::CommandAckStatus ack;    ///< 命令窄接口的 ACK 快照
    uint64_t now_ms = 0;             ///< 调用方单调时钟（与 ACK 快照同一时基）
};

/// 通用拦截设备接口。
///
/// 状态机只依赖“发起一次动作/停止/推进时序”的通用语义，不关心底层是单发
/// PWM 舵机、多发转盘、干扰机开关，还是无执行器的撞击。新增实现时通过
/// `actuator.type` 在工厂中选择。
class IInterceptionDevice {
public:
    virtual ~IInterceptionDevice() = default;

    /// 动作形态（kPulse/kHold/kNone）；状态机据此选择 S10 完成语义。
    virtual InterceptMode Mode() const = 0;

    /// 发起一次拦截动作。仅在 Ready 且仍有剩余次数时成功；kPulse 实现发送
    /// 触发行程，kHold 实现发送开始指令。true=命令已入队。
    virtual bool Execute() = 0;

    /// 停止/复位到待机。kPulse 实现发送复位行程，kHold 实现发送停止指令。
    /// 维护/测试用；状态机正常流程只在 S16 故障处置时停止转发反馈，
    /// 不补发复位（故障不自动清除）。Faulted 时返回 false。
    virtual bool Stop() = 0;

    /// 每拍推进内部时序（ACK 确认、保持计时、复位、稳定计时、故障判定）。
    /// 必须由调用方按周期调用；Ready/Faulted 时为空操作。
    virtual void Update(const InterceptFeedback& feedback) = 0;

    /// 是否已完成动作-复位周期、可接受下一次动作。
    virtual bool Ready() const = 0;

    /// 动作是否已成功发起过（kPulse=已入队触发；kHold=正在作用）。
    /// 用于 S10→S11 完成判据与防重复触发。
    virtual bool Triggered() const = 0;

    /// 剩余可用次数（kPulse 单发：初始 1，触发入队后 0，复位不恢复；
    /// kHold 可返回 UINT32_MAX 表示不限）。
    virtual uint32_t RemainingUses() const = 0;

    /// 是否进入终态故障（需人工/维护处置，不复位、不重试）。
    virtual bool Faulted() const = 0;

    /// 故障原因（中文，供日志/状态回传）；未故障为空串。
    virtual const char* FaultReason() const = 0;

    /// 实现类型（与 `actuator.type` 同源，供日志/诊断）。
    virtual const char* Type() const = 0;
};

/// 按 `actuator.type` 创建设备实现。当前支持 `single_pwm`（kPulse 捕网）；
/// 未来多发/干扰/无执行器实现在同一工厂按类型接入，装配层无需改动状态机。
/// @param config 已通过 Validate 的执行器配置
/// @param sender PX4 命令窄接口；为空时返回 nullptr（影子/未开控制）
/// @throws std::invalid_argument 配置类型未知或参数非法
std::unique_ptr<IInterceptionDevice> CreateInterceptionDevice(
    const config::ActuatorConfig& config,
    communication::IPx4CommandSender* sender);

}  // namespace state_machine
}  // namespace drone
