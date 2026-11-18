#include "communication/communication_transport.h"

#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace drone::communication {
namespace {

TEST(CommunicationTransportTest, UdpTransportSendsAndReceivesDatagram) {
    const uint16_t base = static_cast<uint16_t>(20000 + (::getpid() % 10000));

    UdpTransportConfig first_config;
    first_config.bind_port = base;
    first_config.remote_address = "127.0.0.1";
    first_config.remote_port = static_cast<uint16_t>(base + 1);
    first_config.read_timeout = std::chrono::milliseconds(100);

    UdpTransportConfig second_config;
    second_config.bind_port = static_cast<uint16_t>(base + 1);
    second_config.remote_address = "127.0.0.1";
    second_config.remote_port = base;
    second_config.read_timeout = std::chrono::milliseconds(100);

    auto first = CreateUdpTransport(first_config);
    auto second = CreateUdpTransport(second_config);
    first->Open();
    second->Open();

    const std::array<uint8_t, 5> expected{1, 2, 3, 4, 5};
    ASSERT_TRUE(first->Write(expected.data(), expected.size()));

    std::array<uint8_t, 32> actual{};
    const auto count = second->Read(actual.data(), actual.size());
    ASSERT_EQ(count, static_cast<std::ptrdiff_t>(expected.size()));
    EXPECT_TRUE(std::equal(expected.begin(), expected.end(), actual.begin()));
    EXPECT_EQ(first->ErrorCount(), 0u);
    EXPECT_EQ(second->ErrorCount(), 0u);

    first->Close();
    second->Close();
}

TEST(CommunicationTransportTest, UdpTransportLearnsPeerFromFirstDatagram) {
    const uint16_t base = static_cast<uint16_t>(40000 + (::getpid() % 10000));

    UdpTransportConfig learner_config;
    learner_config.bind_port = base;
    learner_config.read_timeout = std::chrono::milliseconds(100);

    UdpTransportConfig peer_config;
    peer_config.bind_port = static_cast<uint16_t>(base + 1);
    peer_config.remote_address = "127.0.0.1";
    peer_config.remote_port = base;
    peer_config.read_timeout = std::chrono::milliseconds(100);

    auto learner = CreateUdpTransport(learner_config);
    auto peer = CreateUdpTransport(peer_config);
    learner->Open();
    peer->Open();

    const std::array<uint8_t, 2> hello{9, 8};
    ASSERT_TRUE(peer->Write(hello.data(), hello.size()));
    std::array<uint8_t, 8> buffer{};
    ASSERT_EQ(learner->Read(buffer.data(), buffer.size()), 2);

    const std::array<uint8_t, 3> reply{7, 6, 5};
    ASSERT_TRUE(learner->Write(reply.data(), reply.size()));
    ASSERT_EQ(peer->Read(buffer.data(), buffer.size()), 3);
    EXPECT_EQ(buffer[0], 7);
    EXPECT_EQ(buffer[1], 6);
    EXPECT_EQ(buffer[2], 5);
}

TEST(CommunicationTransportTest, UdpConfigRejectsPartialRemote) {
    UdpTransportConfig config;
    config.remote_address = "127.0.0.1";
    config.remote_port = 0;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
}

}  // namespace
}  // namespace drone::communication
