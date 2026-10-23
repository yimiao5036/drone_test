// 地面站直控发射服务（GcsNetFire）单元测试：门禁拒绝路径、受理→完成回执、
// 幂等（同一 command_id 只触发一次）与冷却。执行器用共享 FakeCommandSender，
// 服务在独立线程按 20ms 节拍推进，因此断言一律带超时轮询，不依赖固定睡眠。

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>

#include "../communication/fake_command_sender.h"
#include "common/topic.h"
#include "common/types.h"
#include "config/config.h"
#include "state_machine/gcs_net_fire.h"

namespace {

using drone::common::FlightStateSnapshot;
using drone::common::NetFireAck;
using drone::common::NetFireCommand;
using drone::common::NetFireReason;
using drone::common::NetFireResult;
using drone::config::ActuatorConfig;
using drone::state_machine::GcsNetFire;
using drone::test::FakeCommandSender;
using drone::test::kDoSetActuatorCommand;
using namespace std::chrono_literals;

uint64_t NowMs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

ActuatorConfig MakeConfig(int64_t cooldown_ms = 3000) {
    ActuatorConfig config;
    config.type = "single_pwm";
    config.net_channel = 1;
    config.reset_delay = std::chrono::milliseconds(100);
    config.reset_settle = std::chrono::milliseconds(0);
    config.gcs_fire_enabled = true;
    config.gcs_fire_cooldown = std::chrono::milliseconds(cooldown_ms);
    return config;
}

/// 测试夹具：绑定 Topic、启动服务、按 command_id 等待 65017 回执。
class GcsNetFireTest : public ::testing::Test {
protected:
    void SetUp() override {
        service_ = std::make_unique<GcsNetFire>(MakeConfig(cooldown_ms_), &sender_);
        service_->SetFlightStateInput(flight_state_topic_);
        service_->SetCommandInput(command_topic_);
        ack_subscription_ = service_->AckOutput().Subscribe(
            16, drone::common::Topic<NetFireAck>::OverflowPolicy::kDropOldest);
        ASSERT_TRUE(service_->Start());
    }

    void TearDown() override {
        if (service_ != nullptr) {
            service_->Stop();
        }
    }

    void SetCooldown(int64_t cooldown_ms) { cooldown_ms_ = cooldown_ms; }
    /// 发布飞行状态快照；默认为落地、未解锁、状态有效。
    void PublishState(bool landed = true, bool armed = false,
                      bool landed_state_valid = true) {
        FlightStateSnapshot state;
        state.header.receive_time_ms = NowMs();
        state.header.valid_for_ms = 1000;
        state.connected = true;
        state.landed = landed;
        state.armed = armed;
        state.landed_state_valid = landed_state_valid;
        flight_state_topic_.Publish(
            std::make_shared<const FlightStateSnapshot>(state));
    }

    void PublishCommand(uint32_t command_id, uint8_t flags = 0x01,
                        uint8_t version = 1) {
        NetFireCommand command;
        command.header.receive_time_ms = NowMs();
        command.header.valid_for_ms = 2000;
        command.protocol_version = version;
        command.command_id = command_id;
        command.flags = flags;
        command.received_ms = NowMs();
        command_topic_.Publish(std::make_shared<const NetFireCommand>(command));
    }

    /// 等待指定 command_id 的回执（超时返回 nullopt）。
    std::optional<NetFireAck> WaitAck(uint32_t command_id,
                                      std::chrono::milliseconds timeout = 2000ms) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            auto ack = ack_subscription_.WaitTakeFor(remaining);
            if (!ack.has_value()) {
                return std::nullopt;
            }
            if ((*ack)->command_id == command_id) {
                return **ack;
            }
        }
        return std::nullopt;
    }

    /// 轮询等待执行器收到第 count 次命令（触发/复位）。
    bool WaitSenderCalls(uint64_t count, std::chrono::milliseconds timeout = 2000ms) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (sender_.CallCount() >= count) {
                return true;
            }
            std::this_thread::sleep_for(5ms);
        }
        return false;
    }

    FakeCommandSender sender_;
    drone::common::Topic<FlightStateSnapshot> flight_state_topic_;
    drone::common::Topic<NetFireCommand> command_topic_;
    std::unique_ptr<GcsNetFire> service_;
    drone::common::Topic<NetFireAck>::Subscription ack_subscription_;
    int64_t cooldown_ms_ = 3000;
};

TEST_F(GcsNetFireTest, RejectsWithoutFreshFlightState) {
    PublishCommand(1);
    const auto ack = WaitAck(1);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(ack->result, NetFireResult::kRejected);
    EXPECT_EQ(ack->reason, NetFireReason::kFlightStateStale);
    EXPECT_EQ(sender_.CallCount(), 0u);
}

TEST_F(GcsNetFireTest, RejectsWhenArmed) {
    PublishState(true, true);
    PublishCommand(2);
    const auto ack = WaitAck(2);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(ack->reason, NetFireReason::kArmed);
    EXPECT_EQ(sender_.CallCount(), 0u);
}

TEST_F(GcsNetFireTest, RejectsWhenNotLanded) {
    PublishState(false, false);
    PublishCommand(3);
    const auto ack = WaitAck(3);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(ack->reason, NetFireReason::kNotLanded);
    EXPECT_EQ(sender_.CallCount(), 0u);
}

TEST_F(GcsNetFireTest, RejectsWithoutConfirmationFlag) {
    PublishState();
    PublishCommand(4, /*flags=*/0x00);
    const auto ack = WaitAck(4);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(ack->reason, NetFireReason::kConfirmationMissing);
    EXPECT_EQ(sender_.CallCount(), 0u);
}

TEST_F(GcsNetFireTest, RejectsUnsupportedVersion) {
    PublishState();
    PublishCommand(5, /*flags=*/0x01, /*version=*/2);
    const auto ack = WaitAck(5);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(ack->reason, NetFireReason::kUnsupportedVersion);
    EXPECT_EQ(sender_.CallCount(), 0u);
}

TEST_F(GcsNetFireTest, AcceptsLandedDisarmedAndPublishesCompleted) {
    PublishState();
    PublishCommand(10);
    const auto accepted = WaitAck(10);
    ASSERT_TRUE(accepted.has_value());
    ASSERT_EQ(accepted->result, NetFireResult::kAccepted);
    EXPECT_EQ(accepted->reason, NetFireReason::kNone);
    EXPECT_TRUE(WaitSenderCalls(1));

    // 触发 ACK → 保持 reset_delay 后发复位。
    sender_.SimulateAck(kDoSetActuatorCommand, 0, NowMs());
    ASSERT_TRUE(WaitSenderCalls(2)) << "保持结束后应发出复位命令";
    sender_.SimulateAck(kDoSetActuatorCommand, 0, NowMs());

    const auto completed = WaitAck(10);
    ASSERT_TRUE(completed.has_value());
    EXPECT_EQ(completed->result, NetFireResult::kCompleted);
    EXPECT_EQ(completed->actuator_state, 0);  // Ready
}

TEST_F(GcsNetFireTest, DuplicateCommandIdDoesNotFireTwice) {
    PublishState();
    PublishCommand(20);
    ASSERT_TRUE(WaitAck(20).has_value());
    EXPECT_TRUE(WaitSenderCalls(1));

    // 同一 command_id 重发：回执标记 DUPLICATE_COMMAND，且不会再发触发命令。
    PublishCommand(20);
    const auto duplicate = WaitAck(20);
    ASSERT_TRUE(duplicate.has_value());
    EXPECT_EQ(duplicate->reason, NetFireReason::kDuplicateCommand);
    EXPECT_EQ(sender_.CallCount(), 1u);
}

// 单发执行器触发后剩余发数归零；复燃请求应报 NO_REMAINING_SHOTS（冷却门禁
// 位于发数检查之后，单发实现下不可达，留给多发实现）。
TEST_F(GcsNetFireTest, RejectsSecondCommandAfterSingleShotUsedUp) {
    service_->Stop();
    service_ = std::make_unique<GcsNetFire>(MakeConfig(60000), &sender_);
    service_->SetFlightStateInput(flight_state_topic_);
    service_->SetCommandInput(command_topic_);
    ack_subscription_ = service_->AckOutput().Subscribe(
        16, drone::common::Topic<NetFireAck>::OverflowPolicy::kDropOldest);
    ASSERT_TRUE(service_->Start());

    PublishState();
    PublishCommand(30);
    ASSERT_TRUE(WaitAck(30).has_value());
    ASSERT_TRUE(WaitSenderCalls(1));
    sender_.SimulateAck(kDoSetActuatorCommand, 0, NowMs());
    ASSERT_TRUE(WaitSenderCalls(2));
    sender_.SimulateAck(kDoSetActuatorCommand, 0, NowMs());
    const auto completed = WaitAck(30);
    ASSERT_TRUE(completed.has_value());
    ASSERT_EQ(completed->result, NetFireResult::kCompleted);

    // 执行器已回到 Ready，但单发发数已耗尽：拒绝且不再发触发命令。
    PublishCommand(31);
    const auto ack = WaitAck(31);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(ack->reason, NetFireReason::kNoRemainingShots);
    EXPECT_EQ(sender_.CallCount(), 2u);
}

}  // namespace
