#include "communication/mavlink_handler.h"

#include <cstring>
#include <stdexcept>

namespace drone::communication {

MavlinkHandler::MavlinkHandler(MavlinkVersion output_version)
    : output_version_(output_version) {
    if (output_version_ != MavlinkVersion::kV1 &&
        output_version_ != MavlinkVersion::kV2) {
        throw std::invalid_argument("MAVLink 输出版本必须是 1 或 2");
    }
    ApplyOutputVersion();
}

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
        const uint8_t framing = mavlink_frame_char_buffer(
            &rx_message_buffer_, &rx_status_, data[index],
            &decoded_message, &decoded_status);

        UpdateDroppedPacketCount();
        if (framing == MAVLINK_FRAMING_OK) {
            ++received_message_count_;
            ++decoded_count;
            if (callback) {
                callback(decoded_message);
            }
        } else if (framing == MAVLINK_FRAMING_BAD_CRC ||
                   framing == MAVLINK_FRAMING_BAD_SIGNATURE) {
            ++parse_error_count_;
        }
    }
    return decoded_count;
}

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

void MavlinkHandler::ApplyOutputVersion() {
    if (output_version_ == MavlinkVersion::kV1) {
        tx_status_.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    } else {
        tx_status_.flags &= static_cast<uint8_t>(~MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    }
}

void MavlinkHandler::UpdateDroppedPacketCount() {
    const uint16_t current = rx_status_.packet_rx_drop_count;
    const uint16_t delta = static_cast<uint16_t>(current - last_library_drop_count_);
    dropped_packet_count_ += delta;
    last_library_drop_count_ = current;
}

}  // namespace drone::communication
