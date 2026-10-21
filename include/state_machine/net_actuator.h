#pragma once

#include <memory>

namespace drone {

namespace communication {
class IPx4CommandSender;
}

namespace config {
struct ActuatorConfig;
}

namespace state_machine {

/// 捕网执行器抽象接口。
///
/// 状态机只依赖本接口的“触发一次/复位到待机”语义，不关心底层是单发 PWM
/// 舵机、后续多发转盘/多通道机构，还是台架假件。新增实现时通过
/// `actuator.type` 在工厂中选择，状态机与 S9/S10/S11 流程不需要改动。
///
/// 返回语义：true=命令已成功入队（不代表机构已动作）；false=链路未就绪/
/// 命令队列满，调用方保留 pending 标志并在下一拍重试。
class INetActuator {
public:
    virtual ~INetActuator() = default;

    /// 触发一次发射/下一发。单发实现发送触发行程；多发实现可自行推进弹仓/
    /// 通道选择，但必须保持“返回 true 才认为本发已入队”的语义。
    virtual bool Trigger() = 0;

    /// 复位到待机/待命行程。用于离开拦截序列或机构归位。
    virtual bool Reset() = 0;

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
