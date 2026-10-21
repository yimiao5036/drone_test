#pragma once

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

/// 执行器每拍反馈：PX4 连接状态、命令 ACK 快照与调用方时钟。
///
/// 由状态机在决策拍组装并调用 `INetActuator::Update()`；执行器内部时序
/// （触发确认保持 → 复位 → 复位稳定）全部由执行器自己推进，状态机只负责
/// 转发反馈与观察 `Ready()/Fired()/Faulted()`。
struct ActuatorFeedback {
    bool px4_connected = false;      ///< PX4 链路当前是否在线
    common::CommandAckStatus ack;    ///< 命令窄接口的 ACK 快照
    uint64_t now_ms = 0;             ///< 调用方单调时钟（与 ACK 快照同一时基）
};

/// 捕网执行器抽象接口。
///
/// 状态机只依赖本接口的“触发一次/复位到待机”语义，不关心底层是单发 PWM
/// 舵机、后续多发转盘/多通道机构，还是台架假件。新增实现时通过
/// `actuator.type` 在工厂中选择，状态机与 S9/S10/S11 流程不需要改动。
///
/// 返回语义：true=命令已成功入队（不代表机构已动作）；false=链路未就绪/
/// 命令队列满，调用方保留 pending 标志并在下一拍重试。
///
/// 时序语义（当前 single_pwm）：触发命令 ACK ACCEPTED 后保持 `reset_delay`，
/// 再发复位行程；复位 ACK ACCEPTED 后等待 `reset_settle` 才回到 Ready。
/// 单发剩余发数 1→0，手动 `Reset()` 只归位、不恢复发数。
class INetActuator {
public:
    virtual ~INetActuator() = default;

    /// 触发一次发射/下一发。仅在 Ready 且仍有剩余发数时成功；单发实现
    /// 发送触发行程。true=命令已入队。
    virtual bool Trigger() = 0;

    /// 手动复位到待机/待命行程（离开拦截序列或故障处置前的归位）。
    /// 取消进行中的触发保持/复位时序，但不恢复剩余发数。Faulted 时返回 false。
    virtual bool Reset() = 0;

    /// 每拍推进内部时序（ACK 确认、保持计时、复位、稳定计时、故障判定）。
    /// 必须由状态机在决策拍周期调用；Ready/Faulted 时为空操作。
    virtual void Update(const ActuatorFeedback& feedback) = 0;

    /// 是否已完成触发-复位周期、可接受下一次触发。
    virtual bool Ready() const = 0;

    /// 剩余可用发数（单发：初始 1，触发入队后 0，复位不恢复）。
    virtual uint32_t RemainingShots() const = 0;

    /// 是否已成功入队过触发命令（诊断/状态回传）。
    virtual bool Fired() const = 0;

    /// 是否进入终态故障（需人工/维护处置，不复位、不重试）。
    virtual bool Faulted() const = 0;

    /// 故障原因（中文，供日志/状态回传）；未故障为空串。
    virtual const char* FaultReason() const = 0;

    /// 实现类型（与 `actuator.type` 同源，供日志/诊断）。
    virtual const char* Type() const = 0;
};

/// 按 `actuator.type` 创建执行器实现。当前支持 `single_pwm`；未来多发实现
/// 在同一工厂中按类型接入，装配层无需改动状态机。
/// @param config 已通过 Validate 的执行器配置
/// @param sender PX4 命令窄接口；为空时返回 nullptr（影子/未开控制）
/// @throws std::invalid_argument 配置类型未知或参数非法
std::unique_ptr<INetActuator> CreateNetActuator(
    const config::ActuatorConfig& config,
    communication::IPx4CommandSender* sender);

}  // namespace state_machine
}  // namespace drone
