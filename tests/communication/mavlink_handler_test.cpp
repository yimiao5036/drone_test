#include "communication/mavlink_handler.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace drone::communication {
namespace {

std::vector<uint8_t> EncodeHeartbeat(MavlinkHandler& handler, uint8_t system_id,
                                     uint32_t custom_mode = 0) {
    return handler.Encode(
        [system_id, custom_mode](mavlink_status_t* status, mavlink_message_t* message) {
            return mavlink_msg_heartbeat_pack_status(
                system_id, MAV_COMP_ID_ONBOARD_COMPUTER, status, message,
                MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0,
                custom_mode, MAV_STATE_ACTIVE);
        });
}

TEST(MavlinkHandlerTest, EncodesAndDecodesMavlink2Heartbeat) {
    MavlinkHandler encoder(MavlinkVersion::kV2);
    MavlinkHandler decoder;
    const auto frame = EncodeHeartbeat(encoder, 42, 0x12345678u);

    ASSERT_FALSE(frame.empty());
    EXPECT_EQ(frame.front(), MAVLINK_STX);

    std::size_t callback_count = 0;
    const auto decoded_count = decoder.Feed(
        frame.data(), frame.size(),
        [&callback_count](const mavlink_message_t& message) {
            ++callback_count;
            EXPECT_EQ(message.msgid, MAVLINK_MSG_ID_HEARTBEAT);
            EXPECT_EQ(message.sysid, 42);
            EXPECT_EQ(message.compid, MAV_COMP_ID_ONBOARD_COMPUTER);

            mavlink_heartbeat_t heartbeat{};
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            EXPECT_EQ(heartbeat.custom_mode, 0x12345678u);
            EXPECT_EQ(heartbeat.type, MAV_TYPE_ONBOARD_CONTROLLER);
        });

    EXPECT_EQ(decoded_count, 1u);
    EXPECT_EQ(callback_count, 1u);
    EXPECT_EQ(decoder.ReceivedMessageCount(), 1u);
    EXPECT_EQ(decoder.ReceivedByteCount(), frame.size());
    EXPECT_EQ(decoder.ParseErrorCount(), 0u);
    EXPECT_EQ(encoder.SentMessageCount(), 1u);
    EXPECT_EQ(encoder.SentByteCount(), frame.size());
}

TEST(MavlinkHandlerTest, SupportsMavlink1OutputAndInput) {
    MavlinkHandler encoder(MavlinkVersion::kV1);
    MavlinkHandler decoder;
    const auto frame = EncodeHeartbeat(encoder, 7);

    ASSERT_FALSE(frame.empty());
    EXPECT_EQ(frame.front(), MAVLINK_STX_MAVLINK1);
    EXPECT_EQ(decoder.Feed(frame.data(), frame.size(), nullptr), 1u);
    EXPECT_EQ(decoder.ReceivedMessageCount(), 1u);
}

TEST(MavlinkHandlerTest, ParsesFrameProvidedInFragments) {
    MavlinkHandler encoder;
    MavlinkHandler decoder;
    const auto frame = EncodeHeartbeat(encoder, 9);
    const std::size_t split = frame.size() / 2;

    EXPECT_EQ(decoder.Feed(frame.data(), split, nullptr), 0u);
    EXPECT_EQ(decoder.ReceivedMessageCount(), 0u);

    uint8_t decoded_system_id = 0;
    EXPECT_EQ(decoder.Feed(
                  frame.data() + split, frame.size() - split,
                  [&decoded_system_id](const mavlink_message_t& message) {
                      decoded_system_id = message.sysid;
                  }),
              1u);
    EXPECT_EQ(decoded_system_id, 9);
}

TEST(MavlinkHandlerTest, ParsesMultipleFramesFromSingleBuffer) {
    MavlinkHandler encoder;
    MavlinkHandler decoder;
    const auto first = EncodeHeartbeat(encoder, 10, 1);
    const auto second = EncodeHeartbeat(encoder, 10, 2);

    std::vector<uint8_t> combined = first;
    combined.insert(combined.end(), second.begin(), second.end());

    std::vector<uint32_t> custom_modes;
    EXPECT_EQ(decoder.Feed(
                  combined.data(), combined.size(),
                  [&custom_modes](const mavlink_message_t& message) {
                      mavlink_heartbeat_t heartbeat{};
                      mavlink_msg_heartbeat_decode(&message, &heartbeat);
                      custom_modes.push_back(heartbeat.custom_mode);
                  }),
              2u);
    EXPECT_EQ(custom_modes, (std::vector<uint32_t>{1, 2}));
}

TEST(MavlinkHandlerTest, RejectsBadCrcAndRecoversForFollowingFrame) {
    MavlinkHandler encoder;
    MavlinkHandler decoder;
    auto damaged = EncodeHeartbeat(encoder, 11, 3);
    const auto valid = EncodeHeartbeat(encoder, 11, 4);
    ASSERT_GT(damaged.size(), 12u);
    damaged[10] ^= 0x5Au;  // MAVLink 2 头之后的第一个 payload 字节

    damaged.insert(damaged.end(), valid.begin(), valid.end());
    uint32_t received_custom_mode = 0;
    EXPECT_EQ(decoder.Feed(
                  damaged.data(), damaged.size(),
                  [&received_custom_mode](const mavlink_message_t& message) {
                      mavlink_heartbeat_t heartbeat{};
                      mavlink_msg_heartbeat_decode(&message, &heartbeat);
                      received_custom_mode = heartbeat.custom_mode;
                  }),
              1u);
    EXPECT_EQ(received_custom_mode, 4u);
    EXPECT_GE(decoder.ParseErrorCount(), 1u);
}

TEST(MavlinkHandlerTest, IndependentInstancesDoNotSharePartialFrames) {
    MavlinkHandler encoder;
    MavlinkHandler first_decoder;
    MavlinkHandler second_decoder;
    const auto frame = EncodeHeartbeat(encoder, 12);
    const std::size_t split = frame.size() / 2;

    EXPECT_EQ(first_decoder.Feed(frame.data(), split, nullptr), 0u);
    EXPECT_EQ(second_decoder.Feed(frame.data() + split, frame.size() - split, nullptr), 0u);
    EXPECT_EQ(first_decoder.ReceivedMessageCount(), 0u);
    EXPECT_EQ(second_decoder.ReceivedMessageCount(), 0u);
}

TEST(MavlinkHandlerTest, ResetClearsParserSequenceAndStatistics) {
    MavlinkHandler encoder(MavlinkVersion::kV1);
    MavlinkHandler decoder(MavlinkVersion::kV1);
    const auto frame = EncodeHeartbeat(encoder, 13);

    ASSERT_EQ(decoder.Feed(frame.data(), frame.size(), nullptr), 1u);
    decoder.Reset();

    EXPECT_EQ(decoder.OutputVersion(), MavlinkVersion::kV1);
    EXPECT_EQ(decoder.ReceivedByteCount(), 0u);
    EXPECT_EQ(decoder.ReceivedMessageCount(), 0u);
    EXPECT_EQ(decoder.SentByteCount(), 0u);
    EXPECT_EQ(decoder.SentMessageCount(), 0u);
    EXPECT_EQ(decoder.ParseErrorCount(), 0u);
    EXPECT_EQ(decoder.DroppedPacketCount(), 0u);
}

TEST(MavlinkHandlerTest, EmptyInputIsIgnored) {
    MavlinkHandler handler;
    EXPECT_EQ(handler.Feed(nullptr, 0, nullptr), 0u);
    EXPECT_EQ(handler.ReceivedByteCount(), 0u);
}

}  // namespace
}  // namespace drone::communication
