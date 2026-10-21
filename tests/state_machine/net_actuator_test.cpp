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

/// 模拟 PX4 worker 在 SendCommand() 返回前就完成命令往返：ACK 已写入
/// 快照。用于验证执行器的 ACK 基线必须在入队前抓取。
class AckDuringSendSender final : public FakeCommandSender {
public:
    explicit AckDuringSendSender(uint64_t ack_time_ms)
        : ack_time_ms_(ack_time_ms) {}

    bool SendCommand(uint16_t mavlink_command, float param1, float param2,
                     float param3, float param4, float param5, float param6,
                     float param7) override {
        const bool queued = FakeCommandSender::SendCommand(
            mavlink_command, param1, param2, param3, param4, param5, param6,
            param7);
        SimulateAck(mavlink_command, kAccepted, ack_time_ms_);
        return queued;
    }

private:
    uint64_t ack_time_ms_;
};

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

// 回归：ACK 在 SendCommand() 返回前到达时，基线若在入队后才抓取会把该
// ACK 当成旧 ACK 吞掉，执行器永久停在 FireQueued/ResetQueued。
TEST(NetActuatorTest, AckArrivingBeforeSendReturnsStillAdvances) {
    AckDuringSendSender sender(50);
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(0, 0), &sender);
    ASSERT_TRUE(actuator->Trigger());  // 触发 ACK 在 SendCommand 内到达

    actuator->Update(Feedback(sender, true, 50));
    EXPECT_EQ(sender.CallCount(), 2u) << "入队前到达的 ACK 不得被基线吞掉";
    EXPECT_FALSE(actuator->Faulted());

    // 复位 ACK 同样在 SendCommand 内到达：下一拍应完成稳定并回到 Ready。
    actuator->Update(Feedback(sender, true, 50));
    EXPECT_TRUE(actuator->Ready());
    EXPECT_EQ(sender.CallCount(), 2u);
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

// 回归（最终评审 Important 1）：复位等待跨越 S12~S14 时，同一决策拍内
// AUTO.LAND(176)/DISARM(400) 的 ACK 不得覆盖 187 专用终态；否则执行器会
// 停在 ResetQueued 且不故障（静默挂死）。
TEST(NetActuatorTest, InterleavedAckDoesNotSwallow187) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(0, 0), &sender);
    ASSERT_TRUE(actuator->Trigger());

    sender.SimulateAck(kDoSetActuator, kAccepted, 10);
    sender.SimulateAck(176, kAccepted, 11);  // AUTO.LAND，与 187 同拍到达
    actuator->Update(Feedback(sender, true, 11));

    ASSERT_EQ(sender.CallCount(), 2u);
    EXPECT_FLOAT_EQ(sender.LastParams()[0], 0.0F);  // 复位行程已发出
    EXPECT_FALSE(actuator->Faulted());
}

// 回归（最终评审 Important 1）：无关命令超时只更新全局 last_timeout_command，
// 不得让执行器 Faulted；随后真正的 187 ACK 仍能推进。
TEST(NetActuatorTest, UnrelatedCommandTimeoutDoesNotFault) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(0, 0), &sender);
    ASSERT_TRUE(actuator->Trigger());

    sender.SimulateTimeout(176);  // AUTO.LAND ACK 超时
    actuator->Update(Feedback(sender, true, 10));
    EXPECT_FALSE(actuator->Faulted());
    EXPECT_EQ(sender.CallCount(), 1u);  // 不得补发复位

    sender.SimulateAck(kDoSetActuator, kAccepted, 20);
    actuator->Update(Feedback(sender, true, 20));
    ASSERT_EQ(sender.CallCount(), 2u);
    EXPECT_FLOAT_EQ(sender.LastParams()[0], 0.0F);
}

// 回归（最终评审 Important 3）：手动 Reset() 取消保持/稳定计时、回到 Ready，
// 但不恢复发数，且此后不得再补发命令。
TEST(NetActuatorTest, ManualResetCancelsCycleKeepsShotsConsumed) {
    FakeCommandSender sender;
    auto actuator =
        drone::state_machine::CreateNetActuator(MakeConfig(1500, 0), &sender);
    ASSERT_TRUE(actuator->Trigger());

    sender.SimulateAck(kDoSetActuator, kAccepted, 10);
    actuator->Update(Feedback(sender, true, 10));
    ASSERT_EQ(sender.CallCount(), 1u);  // 保持中，尚未复位

    ASSERT_TRUE(actuator->Reset());
    ASSERT_EQ(sender.CallCount(), 2u);
    EXPECT_FLOAT_EQ(sender.LastParams()[0], 0.0F);
    EXPECT_TRUE(actuator->Ready());
    EXPECT_EQ(actuator->RemainingShots(), 0u);

    // 手动复位后保持/稳定计时已取消，不再产生任何命令。
    actuator->Update(Feedback(sender, true, 5000));
    EXPECT_EQ(sender.CallCount(), 2u);
    EXPECT_FALSE(actuator->Trigger());  // 发数已扣，不能再次触发
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
