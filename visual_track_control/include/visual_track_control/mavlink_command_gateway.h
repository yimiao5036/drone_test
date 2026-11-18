#pragma once

// =============================================================================
// mavlink_command_gateway.h —— MAVLink 命令打包网关（对外声明）
//
// 对应规格：docs/物理追踪思路.md §2.2
//   - kVelocityHeading → SET_POSITION_TARGET_LOCAL_NED（ID 84，速度模式）
//   - kBrakeHover      → ID 84 全零速度
//   （原 kAttitudeTrim → ID 82 已随姿态微调支路一并移除，见 vtc_types.h）
//
// 关键构建约束（务必遵守）：
//   - MAVLink 头（common/mavlink.h）只允许被本库的
//     src/mavlink_command_gateway.cpp 一个 TU 包含：
//     MAVLINK_HELPER 默认空宏，多 TU 包含会多重定义；
//   - 因此本头文件不出现任何 mavlink_* 类型，打包/发送缓冲用
//     固定尺寸字节存储承载（尺寸在 .cpp 中 static_assert 校验）。
//
// 线程与资源：无线程、无 IO、公共方法非虚；成员固定缓冲，热路径零堆分配。
// =============================================================================

#include <cstddef>
#include <cstdint>

#include "visual_track_control/i_command_transport.h"
#include "visual_track_control/vtc_types.h"

namespace drone::vtc {

/// MAVLink 命令打包网关：控制意图 → MAVLink 消息字节流 → 传输接口
class MavlinkCommandGateway {
public:
    /// transport：字节流传输实现（注入，生命周期由调用方保证）；
    /// system_id/component_id：本机 MAVLink 身份；
    /// target_system/target_component：飞控端身份。
    MavlinkCommandGateway(ICommandTransport& transport, std::uint8_t system_id,
                          std::uint8_t component_id, std::uint8_t target_system,
                          std::uint8_t target_component);

    /// 速度与航向意图 → SET_POSITION_TARGET_LOCAL_NED（ID 84，速度模式 type_mask）。
    /// 掩码选择：`yaw_rate_dps` 非零 → 角速率掩码；`yaw_deg` 非零 → 航向角掩码；
    /// 两者均为零 → 角速率掩码配零角速率（保持当前航向，避免误发绝对航向 0）。
    bool SendVelocityHeading(const ControlOutput& output, std::uint32_t time_boot_ms);

    /// 零速刹车 → ID 84 全零速度（判丢等场景，§6.4）。
    bool SendBrakeHover(std::uint32_t time_boot_ms);

    // 固定缓冲尺寸（.cpp 中与实际类型大小做 static_assert 校验）：
    static constexpr std::size_t kMsgBufferSize = 512;  // ≥ sizeof(mavlink_message_t)
    static constexpr std::size_t kTxBufferSize = 512;   // ≥ MAVLINK_MAX_PACKET_LEN

private:
    ICommandTransport& transport_;  // 字节流传输实现（注入）

    // MAVLink 身份
    std::uint8_t system_id_;
    std::uint8_t component_id_;
    std::uint8_t target_system_;
    std::uint8_t target_component_;

    // 固定缓冲：热路径零堆分配（对齐 8 满足 mavlink_message_t 对齐要求）
    alignas(8) std::uint8_t msg_storage_[kMsgBufferSize];  // mavlink_message_t 承载
    std::uint8_t tx_buffer_[kTxBufferSize];                // 发送字节缓冲
};

}  // namespace drone::vtc
