// =============================================================================
// mavlink_command_gateway.cpp —— MAVLink 命令打包网关实现
//
// 对应规格：docs/物理追踪思路.md §2.2
//   kVelocityHeading → SET_POSITION_TARGET_LOCAL_NED（ID 84，速度模式）
//   kBrakeHover      → ID 84 全零速度
//   （原 kAttitudeTrim → ID 82 已随姿态微调支路一并移除，见 vtc_types.h）
//
// 关键构建约束：本文件是全工程唯一包含 common/mavlink.h 的 TU。
// MAVLINK_HELPER 默认空宏（mavlink_helpers.h），多 TU 包含会多重定义。
//
// 打包策略：
//   - 使用 mavlink_msg_*_pack（无通道状态、纯静态内联、可重入），
//     不使用 _pack_chan 通道 API；
//   - mavlink_msg_to_send_buffer 输出字节流 → ICommandTransport::Send；
//   - 消息/发送缓冲为成员固定持有，热路径零堆分配。
//
// 坐标系与语义说明（待实测确认）：
//   - coordinate_frame 取 MAV_FRAME_LOCAL_NED；速度分量由主控律在输出端
//     随机体航向旋转为局部 NED（北/东/地，物理追踪思路 §2.3），本文件
//     直接填充不再做坐标变换；
//   - 速度模式 type_mask 需以 PX4 1.11.3 Offboard 实测为准
//     （物理追踪思路 §10.1）。
// =============================================================================

#include "visual_track_control/mavlink_command_gateway.h"

#include <cmath>

#include "common/mavlink.h"  // 全工程唯一包含点（见文件头构建约束）

namespace drone::vtc {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// ---- SET_POSITION_TARGET_LOCAL_NED（ID 84）type_mask 常量 ----
// 掩码位语义：置位 = 忽略该字段。
// 基础位：忽略位置 x/y/z（bit0-2）与加速度 afx/afy/afz（bit6-8）= 0x01C7。
constexpr std::uint16_t kMaskVelocityBase = 0x01C7;
// 速度 + 航向角模式：忽略 yaw_rate（bit11）
constexpr std::uint16_t kMaskVelocityYaw = kMaskVelocityBase | 0x0800;
// 速度 + 偏航角速率模式：忽略 yaw（bit10）
constexpr std::uint16_t kMaskVelocityYawRate = kMaskVelocityBase | 0x0400;
// 刹车模式：速度全零，yaw 与 yaw_rate 均忽略（bit10、bit11）
constexpr std::uint16_t kMaskBrake = kMaskVelocityBase | 0x0C00;

/// 固定存储解释为消息结构（打包函数只写入字节，不读取旧值）
inline mavlink_message_t* AsMessage(std::uint8_t* storage) {
    return reinterpret_cast<mavlink_message_t*>(storage);
}

}  // namespace

// 固定缓冲尺寸校验：编译期保证存储足够（头文件不暴露 mavlink 类型）
static_assert(sizeof(mavlink_message_t) <= MavlinkCommandGateway::kMsgBufferSize,
              "msg_storage_ 不足以容纳 mavlink_message_t");
static_assert(MAVLINK_MAX_PACKET_LEN <= MavlinkCommandGateway::kTxBufferSize,
              "tx_buffer_ 不足以容纳最大 MAVLink 数据包");

MavlinkCommandGateway::MavlinkCommandGateway(ICommandTransport& transport,
                                             std::uint8_t system_id,
                                             std::uint8_t component_id,
                                             std::uint8_t target_system,
                                             std::uint8_t target_component)
    : transport_(transport),
      system_id_(system_id),
      component_id_(component_id),
      target_system_(target_system),
      target_component_(target_component) {}

bool MavlinkCommandGateway::SendVelocityHeading(const ControlOutput& output,
                                                std::uint32_t time_boot_ms) {
    auto* msg = AsMessage(msg_storage_);

    // 掩码三分支（A2 修复）：
    //   角速率非零 → 速率掩码（速率式控制律常态，§4.2）；
    //   航向角非零 → 航向掩码（位置式控制律）；
    //   两者均零   → 速率掩码配零角速率：保持当前航向。
    // 旧实现把"零角速率"落到航向掩码，等于误发绝对航向 0（指向正北）。
    const bool has_yaw_rate = std::fabs(output.yaw_rate_dps) > 1.0e-9;
    const bool has_yaw = std::fabs(output.yaw_deg) > 1.0e-9;
    const std::uint16_t type_mask =
        (!has_yaw_rate && has_yaw) ? kMaskVelocityYaw : kMaskVelocityYawRate;

    mavlink_msg_set_position_target_local_ned_pack(
        system_id_, component_id_, msg,
        time_boot_ms, target_system_, target_component_,
        MAV_FRAME_LOCAL_NED, type_mask,
        0.0F, 0.0F, 0.0F,  // 位置字段：忽略
        static_cast<float>(output.vx_mps), static_cast<float>(output.vy_mps),
        static_cast<float>(output.vz_mps),
        0.0F, 0.0F, 0.0F,  // 加速度字段：忽略
        static_cast<float>(output.yaw_deg * kDegToRad),
        static_cast<float>(output.yaw_rate_dps * kDegToRad));

    const std::uint16_t len = mavlink_msg_to_send_buffer(tx_buffer_, msg);
    return transport_.Send(tx_buffer_, len);
}

bool MavlinkCommandGateway::SendBrakeHover(std::uint32_t time_boot_ms) {
    auto* msg = AsMessage(msg_storage_);

    // 判丢零速刹车（§6.4）：速度全零，位置/加速度/航向字段全部忽略
    mavlink_msg_set_position_target_local_ned_pack(
        system_id_, component_id_, msg,
        time_boot_ms, target_system_, target_component_,
        MAV_FRAME_LOCAL_NED, kMaskBrake,
        0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F,
        0.0F, 0.0F);

    const std::uint16_t len = mavlink_msg_to_send_buffer(tx_buffer_, msg);
    return transport_.Send(tx_buffer_, len);
}

}  // namespace drone::vtc
