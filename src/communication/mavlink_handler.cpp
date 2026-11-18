#include "communication/mavlink_handler.h"

#include <cstring>
#include <stdexcept>

namespace drone::communication {

// ============================================================================
// MavlinkHandler —— 单条物理链路独占的 MAVLink 字节流解析/序列化基础层。
//
// 职责：
//   - 增量接收任意长度字节流，处理逐字节/半包/粘包/连续多帧；
//   - 校验 MAVLink CRC 与签名状态，坏帧后继续寻找下一帧；
//   - 同时解码 MAVLink 1/2，接收端无需预先选择版本；
//   - 输出帧版本由 output_version_ 控制，发送序号保存在实例私有 tx_status_，
//     避免与其它链路共享生成库的全局 channel 状态。
//
// 边界：不打开串口/网络、不创建线程、不解释业务字段、不关联 ACK；
//       业务语义由 Px4Link / GroundStationLink 负责。
//
// 数据流：
//   物理字节 → Feed() → mavlink_message_t → 业务 Link 解码 → 项目 Topic
//   项目消息 → 业务 Link 调用 *_pack_status → Encode() → 完整 MAVLink 帧字节
// ============================================================================

// 构造：校验输出版本并写入发送状态的 OUT_MAVLINK1 标志。
MavlinkHandler::MavlinkHandler(MavlinkVersion output_version)
    : output_version_(output_version) {
    if (output_version_ != MavlinkVersion::kV1 &&
        output_version_ != MavlinkVersion::kV2) {
        throw std::invalid_argument("MAVLink 输出版本必须是 1 或 2");
    }
    ApplyOutputVersion();
}

// 增量喂入任意长度字节流，逐字节驱动生成库状态机完成增量解析。
// 外部数据可能来自串口一次 Read 或 UDP 数据报，内部含半包/多帧粘包，
// 这里统一按字节推进，保证任意切分方式都能正确还原完整帧。
std::size_t MavlinkHandler::Feed(const uint8_t* data, std::size_t size,
                                 const MessageCallback& callback) {
    if (data == nullptr || size == 0) {
        return 0;
    }

    received_byte_count_ += size;
    std::size_t decoded_count = 0;
    for (std::size_t index = 0; index < size; ++index) {
        mavlink_message_t decoded_message{};
        mavlink_status_t decoded_status{};
        // 生成库的增量解析器：返回 MAVLINK_FRAMING_OK 表示收到完整帧，
        // 其它返回码表示仍在解析中或帧校验失败。
        const uint8_t framing = mavlink_frame_char_buffer(
            &rx_message_buffer_, &rx_status_, data[index],
            &decoded_message, &decoded_status);

        // 每次喂入都同步生成库内部的序号丢包计数，用于诊断链路带宽/重传。
        UpdateDroppedPacketCount();
        if (framing == MAVLINK_FRAMING_OK) {
            ++received_message_count_;
            ++decoded_count;
            if (callback) {
                callback(decoded_message);
            }
        } else if (framing == MAVLINK_FRAMING_BAD_CRC ||
                   framing == MAVLINK_FRAMING_BAD_SIGNATURE) {
            // 校验失败：坏帧不回调，计数后继续解析下一帧（生成库会自动重新同步）。
            ++parse_error_count_;
        }
    }
    return decoded_count;
}

// 切换后续输出帧版本；只影响发送，不影响接收端同时解析 MAVLink 1/2。
void MavlinkHandler::SetOutputVersion(MavlinkVersion version) {
    if (version != MavlinkVersion::kV1 && version != MavlinkVersion::kV2) {
        throw std::invalid_argument("MAVLink 输出版本必须是 1 或 2");
    }
    output_version_ = version;
    ApplyOutputVersion();
}

MavlinkVersion MavlinkHandler::OutputVersion() const {
    return output_version_;
}

// 物理链路重连后清空半包缓冲、收发序号与统计，使解析器和发送状态回到初始。
// 保留 output_version_，最后重新应用到新的发送状态。
void MavlinkHandler::Reset() {
    std::memset(&rx_message_buffer_, 0, sizeof(rx_message_buffer_));
    std::memset(&rx_status_, 0, sizeof(rx_status_));
    std::memset(&tx_status_, 0, sizeof(tx_status_));
    last_library_drop_count_ = 0;
    received_byte_count_ = 0;
    received_message_count_ = 0;
    sent_byte_count_ = 0;
    sent_message_count_ = 0;
    parse_error_count_ = 0;
    dropped_packet_count_ = 0;
    ApplyOutputVersion();
}

// 依据配置的输出版本，在发送状态里置位/清除 OUT_MAVLINK1 标志。
// 该标志决定 *_pack_status 生成的帧头是 v1 还是 v2。
void MavlinkHandler::ApplyOutputVersion() {
    if (output_version_ == MavlinkVersion::kV1) {
        tx_status_.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    } else {
        tx_status_.flags &= static_cast<uint8_t>(~MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    }
}

// 累积生成库报告的序号丢包计数。packet_rx_drop_count 是 16 位，
// 这里记录上次值并做差值（uint16_t 减法自动回绕取模），只累加增量，
// 避免把库内部回绕误当成一次大幅丢包。
void MavlinkHandler::UpdateDroppedPacketCount() {
    const uint16_t current = rx_status_.packet_rx_drop_count;
    const uint16_t delta = static_cast<uint16_t>(current - last_library_drop_count_);
    dropped_packet_count_ += delta;
    last_library_drop_count_ = current;
}

}  // namespace drone::communication
