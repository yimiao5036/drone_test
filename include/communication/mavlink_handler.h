#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include "common/mavlink.h"

namespace drone::communication {

/// MAVLink 输出协议版本。
enum class MavlinkVersion : uint8_t {
    kV1 = 1,
    kV2 = 2,
};

/// 单条物理链路独占的 MAVLink 字节流解析与序列化器。
///
/// 每个 PX4/地面站链路必须创建独立实例，禁止跨链路共享解析状态。
/// 本类不解释具体业务消息，只负责半包/粘包解析、CRC 校验和帧序列化。
/// 线程模型为单线程独占，调用方负责保证 Feed/Encode 不并发执行。
class MavlinkHandler final {
public:
    using MessageCallback = std::function<void(const mavlink_message_t&)>;

    explicit MavlinkHandler(MavlinkVersion output_version = MavlinkVersion::kV2);

    MavlinkHandler(const MavlinkHandler&) = delete;
    MavlinkHandler& operator=(const MavlinkHandler&) = delete;
    MavlinkHandler(MavlinkHandler&&) = delete;
    MavlinkHandler& operator=(MavlinkHandler&&) = delete;

    /// 输入任意长度的串口/网络字节流，支持逐字节、半包和多帧粘包。
    /// @return 本次成功解析并回调的完整 MAVLink 消息数。
    std::size_t Feed(const uint8_t* data, std::size_t size,
                     const MessageCallback& callback);

    /// 使用生成库的 *_pack_status 函数构造并序列化消息。
    ///
    /// Packer 签名应为：uint16_t(mavlink_status_t*, mavlink_message_t*)。
    /// 使用实例私有 tx_status_ 维护发送序号，避免多条链路共享全局 channel 状态。
    template <typename Packer>
    std::vector<uint8_t> Encode(Packer&& packer) {
        mavlink_message_t message{};
        const uint16_t packed_length =
            std::forward<Packer>(packer)(&tx_status_, &message);
        if (packed_length == 0) {
            return {};
        }

        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
        const uint16_t frame_length =
            mavlink_msg_to_send_buffer(buffer.data(), &message);
        if (frame_length == 0) {
            return {};
        }

        ++sent_message_count_;
        sent_byte_count_ += frame_length;
        return std::vector<uint8_t>(buffer.begin(), buffer.begin() + frame_length);
    }

    /// 设置后续输出帧版本；不影响接收端同时解析 MAVLink 1/2。
    void SetOutputVersion(MavlinkVersion version);
    [[nodiscard]] MavlinkVersion OutputVersion() const;

    /// 清空半包、收发序号和全部统计，用于物理链路重连后重新同步。
    void Reset();

    [[nodiscard]] uint64_t ReceivedByteCount() const { return received_byte_count_; }
    [[nodiscard]] uint64_t ReceivedMessageCount() const { return received_message_count_; }
    [[nodiscard]] uint64_t SentByteCount() const { return sent_byte_count_; }
    [[nodiscard]] uint64_t SentMessageCount() const { return sent_message_count_; }
    [[nodiscard]] uint64_t ParseErrorCount() const { return parse_error_count_; }
    [[nodiscard]] uint64_t DroppedPacketCount() const { return dropped_packet_count_; }

private:
    void ApplyOutputVersion();
    void UpdateDroppedPacketCount();

    MavlinkVersion output_version_ = MavlinkVersion::kV2;
    mavlink_message_t rx_message_buffer_{};
    mavlink_status_t rx_status_{};
    mavlink_status_t tx_status_{};
    uint16_t last_library_drop_count_ = 0;

    uint64_t received_byte_count_ = 0;
    uint64_t received_message_count_ = 0;
    uint64_t sent_byte_count_ = 0;
    uint64_t sent_message_count_ = 0;
    uint64_t parse_error_count_ = 0;
    uint64_t dropped_packet_count_ = 0;
};

}  // namespace drone::communication
