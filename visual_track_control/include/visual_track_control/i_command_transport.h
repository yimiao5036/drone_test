#pragma once

// =============================================================================
// i_command_transport.h —— 命令传输层抽象接口
//
// 设计目标（双重解耦）：
//   - 传输层只吃字节流：串口 / 电台 / 回环打印均可注入实现，
//     后续重写传输只换实现，不动打包与控制律；
//   - 打包封装（MavlinkCommandGateway）与本接口组合，不直接依赖具体链路。
//
// 控制律不直接写飞控（物理追踪思路 §2.2 控制单出口约束的独立库映射）：
// 网关打包后交由实现方决定如何送达。
// =============================================================================

#include <cstddef>
#include <cstdint>

namespace drone::vtc {

/// 命令传输接口：只吃字节流
class ICommandTransport {
public:
    virtual ~ICommandTransport() = default;

    /// 发送一帧已打包的命令字节流。
    /// 返回 true = 已交付底层链路；false = 发送失败（调用方可计数/告警）。
    /// 实现方保证：不长期持有 data 指针（同步拷贝或同步写出）。
    virtual bool Send(const std::uint8_t* data, std::size_t len) = 0;
};

}  // namespace drone::vtc
