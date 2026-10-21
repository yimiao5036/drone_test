#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>

#include "../common/log_capture.h"
#include "../communication/fake_command_sender.h"
#include "config/config.h"
#include "state_machine/net_actuator.h"

namespace {
using drone::common::CommandAckStatus;
using drone::state_machine::ActuatorFeedback;
using drone::test::FakeCommandSender;
using namespace std::chrono_literals;

constexpr uint16_t kDoSetActuator = 187;
constexpr uint8_t kAccepted = 0;
constexpr uint8_t kDenied = 2;
constexpr uint8_t kInProgress = 5;

drone::config::ActuatorConfig MakeConfig(int64_t reset_delay_ms = 1500,
                                         int64_t reset_settle_ms = 0) {
    drone::config::ActuatorConfig config;
    config.net_channel = 1;
    config.reset_delay = std::chrono::milliseconds(reset_delay_ms);
    config.reset_settle = std::chrono::milliseconds(reset_settle_ms);
    return config;
}

ActuatorFeedback Feedback(const FakeCommandSender& sender, bool connected,
                          uint64_t now_ms) {
    return {connected, sender.AckStatus(), now_ms};
}

TEST(NetActuatorTest, InitialStateReadyWithOneShot) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(), &sender);
    ASSERT_NE(actuator, nullptr);
    EXPECT_TRUE(actuator->Ready());
    EXPECT_EQ(actuator->RemainingShots(), 1u);
    EXPECT_FALSE(actuator->Fired());
    EXPECT_FALSE(actuator->Faulted());
}

TEST(NetActuatorTest, TriggerSendsFireAndConsumesShot) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(), &sender);
    ASSERT_TRUE(actuator->Trigger());
    EXPECT_TRUE(actuator->Fired());
    EXPECT_EQ(actuator->RemainingShots(), 0u);
    EXPECT_FALSE(actuator->Ready());
    ASSERT_EQ(sender.CallCount(), 1u);
    EXPECT_EQ(sender.LastCommand(), kDoSetActuator);
    EXPECT_FLOAT_EQ(sender.LastParams()[0], -0.7F);  // Set 1 → p1
    EXPECT_TRUE(std::isnan(sender.LastParams()[1]));
    EXPECT_FLOAT_EQ(sender.LastParams()[6], 0.0F);
    EXPECT_FALSE(actuator->Trigger());
}

TEST(NetActuatorTest, FireAckStartsHoldThenSendsReset) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(1500, 0), &sender);
    ASSERT_TRUE(actuator->Trigger());

    sender.SimulateAck(kDoSetActuator, kAccepted, 50);
    actuator->Update(Feedback(sender, true, 50));
    ASSERT_EQ(sender.CallCount(), 1u);  // 尚未到保持截止

    actuator->Update(Feedback(sender, true, 1549));
    EXPECT_EQ(sender.CallCount(), 1u);

    actuator->Update(Feedback(sender, true, 1550));
    ASSERT_EQ(sender.CallCount(), 2u);
    EXPECT_FLOAT_EQ(sender.LastParams()[0], 0.0F);  // 复位/待机
}

TEST(NetActuatorTest, ResetAckAndSettleMakesReady) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(0, 200), &sender);
    ASSERT_TRUE(actuator->Trigger());

    sender.SimulateAck(kDoSetActuator, kAccepted, 10);
    actuator->Update(Feedback(sender, true, 10));
    ASSERT_EQ(sender.CallCount(), 2u);  // reset_delay=0 → 同拍复位

    sender.SimulateAck(kDoSetActuator, kAccepted, 20);
    actuator->Update(Feedback(sender, true, 20));
    EXPECT_FALSE(actuator->Ready());

    actuator->Update(Feedback(sender, true, 219));
    EXPECT_FALSE(actuator->Ready());

    actuator->Update(Feedback(sender, true, 220));
    EXPECT_TRUE(actuator->Ready());
    EXPECT_EQ(actuator->RemainingShots(), 0u);
}

TEST(NetActuatorTest, InProgressKeepsWaiting) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(0, 0), &sender);
    ASSERT_TRUE(actuator->Trigger());

    sender.SimulateAck(kDoSetActuator, kInProgress, 10);
    actuator->Update(Feedback(sender, true, 10));
    EXPECT_EQ(sender.CallCount(), 1u);

    sender.SimulateAck(kDoSetActuator, kAccepted, 20);
    actuator->Update(Feedback(sender, true, 20));
    EXPECT_EQ(sender.CallCount(), 2u);
}

TEST(NetActuatorTest, RejectedAckFaultsWithoutReset) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(0, 0), &sender);
    ASSERT_TRUE(actuator->Trigger());

    sender.SimulateAck(kDoSetActuator, kDenied, 10);
    actuator->Update(Feedback(sender, true, 10));
    EXPECT_TRUE(actuator->Faulted());
    EXPECT_EQ(sender.CallCount(), 1u);
    EXPECT_FALSE(actuator->Reset());
    actuator->Update(Feedback(sender, true, 1000));
    EXPECT_EQ(sender.CallCount(), 1u);
}

TEST(NetActuatorTest, FireAckTimeoutFaultsAndLateAckDoesNotAdvance) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(), &sender);
    ASSERT_TRUE(actuator->Trigger());

    sender.SimulateTimeout(kDoSetActuator);
    actuator->Update(Feedback(sender, true, 1000));
    EXPECT_TRUE(actuator->Faulted());
    EXPECT_EQ(sender.CallCount(), 1u);

    // 超时后迟到的 ACK 不得让状态机复活或补发复位。
    sender.SimulateAck(kDoSetActuator, kAccepted, 1100);
    actuator->Update(Feedback(sender, true, 1100));
    EXPECT_TRUE(actuator->Faulted());
    EXPECT_EQ(sender.CallCount(), 1u);
}

TEST(NetActuatorTest, DisconnectWhileHoldingFaults) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(1500, 0), &sender);
    ASSERT_TRUE(actuator->Trigger());

    sender.SimulateAck(kDoSetActuator, kAccepted, 10);
    actuator->Update(Feedback(sender, true, 10));
    ASSERT_EQ(sender.CallCount(), 1u);  // 已进入保持，尚未复位

    actuator->Update(Feedback(sender, false, 100));
    EXPECT_TRUE(actuator->Faulted());
    EXPECT_EQ(sender.CallCount(), 1u);  // 不得补发复位

    actuator->Update(Feedback(sender, true, 2000));
    EXPECT_EQ(sender.CallCount(), 1u);
}

TEST(NetActuatorTest, ResetEnqueueFailureRetriesWithoutFire) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(0, 0), &sender);
    ASSERT_TRUE(actuator->Trigger());

    sender.FailNextCalls(2);
    sender.SimulateAck(kDoSetActuator, kAccepted, 10);
    actuator->Update(Feedback(sender, true, 10));   // 复位第 1 次失败
    actuator->Update(Feedback(sender, true, 110));  // 复位第 2 次失败
    actuator->Update(Feedback(sender, true, 210));  // 复位成功
    ASSERT_EQ(sender.CallCount(), 4u);

    const auto history = sender.History();
    EXPECT_FLOAT_EQ(history[0].params[0], -0.7F);
    for (std::size_t i = 1; i < history.size(); ++i) {
        EXPECT_FLOAT_EQ(history[i].params[0], 0.0F)
            << "复位重试期间不得混入触发行程 index=" << i;
    }
}

TEST(NetActuatorTest, DisconnectWhileWaitingFaults) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(), &sender);
    ASSERT_TRUE(actuator->Trigger());

    actuator->Update(Feedback(sender, false, 100));
    EXPECT_TRUE(actuator->Faulted());
    EXPECT_EQ(sender.CallCount(), 1u);
}

TEST(NetActuatorTest, UnrelatedAckDoesNotAdvanceOrFault) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(0, 0), &sender);
    ASSERT_TRUE(actuator->Trigger());

    sender.SimulateAck(176, kAccepted, 10);  // AUTO.LAND
    actuator->Update(Feedback(sender, true, 10));
    EXPECT_EQ(sender.CallCount(), 1u);

    sender.SimulateAck(kDoSetActuator, kAccepted, 20);
    actuator->Update(Feedback(sender, true, 20));
    EXPECT_EQ(sender.CallCount(), 2u);
}

TEST(NetActuatorTest, EnqueueFailureRetriesWithoutConsumingShot) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(), &sender);
    sender.FailNextCalls(1);
    EXPECT_FALSE(actuator->Trigger());
    EXPECT_FALSE(actuator->Fired());
    EXPECT_EQ(actuator->RemainingShots(), 1u);

    EXPECT_TRUE(actuator->Trigger());
    EXPECT_TRUE(actuator->Fired());
    EXPECT_EQ(actuator->RemainingShots(), 0u);
}

TEST(NetActuatorTest, WarnsWhenResetDelayOutsideManualRange) {
    drone::test::LogCapture logs;
    FakeCommandSender sender;
    (void)drone::state_machine::CreateNetActuator(MakeConfig(500, 0), &sender);
    EXPECT_EQ(logs.Count("reset_delay_ms"), 1);
}

}  // namespace
