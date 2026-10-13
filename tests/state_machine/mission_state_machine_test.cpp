#include <chrono>
#include <memory>
#include <optional>
#include <thread>

#include <gtest/gtest.h>

#include "state_machine/mission_state_machine.h"

namespace {

using namespace drone;
using namespace std::chrono_literals;

uint64_t TestMonotonicMs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

common::FlightStateSnapshot MakeNavigationReadyFlight() {
    common::FlightStateSnapshot flight;
    flight.header.receive_time_ms = TestMonotonicMs();
    flight.header.valid_for_ms = 1000;
    flight.connected = true;
    flight.gps_state_valid = true;
    flight.gps_fix = true;
    flight.gps_fix_type = 3;
    flight.global_position_valid = true;
    flight.latitude_1e7 = 231230000;
    flight.longitude_1e7 = 1131230000;
    flight.altitude_mm = 120000;
    flight.home_valid = true;
    flight.home_lat_1e7 = 231200000;
    flight.home_lon_1e7 = 1131200000;
    flight.home_altitude_mm = 100000;
    return flight;
}

common::GroundStationTarget MakeFreshGroundTarget() {
    common::GroundStationTarget target;
    target.header.sequence = 1;
    target.header.source_time_ms = TestMonotonicMs();
    target.header.receive_time_ms = TestMonotonicMs();
    target.header.valid_for_ms = 1000;
    target.header.source_id = 255;
    target.header.health = 1;
    target.header.frame_id = 1;
    target.ground_station_boot_id = 42;
    target.update_seq = 1;
    target.target_id = 7;
    target.latitude_1e7 = 231230000;
    target.longitude_1e7 = 1131230000;
    target.altitude_mm = 120000;
    target.alt_reference = 1;
    target.protocol_version = 1;
    target.transport_age_ms = 50;
    return target;
}

// 局部 NED 坐标系编码（frame_id=3，见 types.h 文件头约定）。
constexpr uint8_t kFrameLocalNed = 3;

/// 有效目标估计快照（TargetEstimator 影子输出语义：valid=true 且剩余有效期充足）。
common::TargetState MakeValidEstimatedTarget() {
    common::TargetState state;
    state.header.sequence = 1;
    state.header.source_time_ms = TestMonotonicMs();
    state.header.receive_time_ms = TestMonotonicMs();
    state.header.valid_for_ms = 1000;
    state.header.source_id = 0;
    state.header.health = 1;
    state.header.frame_id = kFrameLocalNed;
    state.target_id = 7;
    state.pos_x_m = 12.5f;
    state.pos_y_m = -3.0f;
    state.pos_z_m = -8.0f;
    state.valid = true;
    return state;
}

/// 估计失效快照（valid=false，例如地面站目标过期后 TargetEstimator 发布的过期态）。
common::TargetState MakeInvalidEstimatedTarget() {
    common::TargetState state = MakeValidEstimatedTarget();
    state.header.sequence = 2;
    state.header.receive_time_ms = TestMonotonicMs();
    state.header.valid_for_ms = 0;
    state.header.health = 2;
    state.valid = false;
    return state;
}

std::optional<common::MissionStatus> WaitForState(
    common::Topic<common::MissionStatus>::Subscription& subscription,
    common::MissionState expected,
    std::chrono::milliseconds timeout = 1500ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = subscription.WaitTakeFor(50ms);
        if (!message) {
            continue;
        }
        if ((*message)->state == expected) {
            return **message;
        }
    }
    return std::nullopt;
}

TEST(MissionStateMachineTest, StartFailsBeforeInputsAreBound) {
    state_machine::MissionStateMachine machine;

    EXPECT_FALSE(machine.Start());
    EXPECT_FALSE(machine.IsRunning());
    EXPECT_GT(machine.ErrorCount(), 0U);
}

TEST(MissionStateMachineTest, PublishesBootAndSelfCheckOnStart) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    state_machine::MissionStateMachine machine;
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    auto status_subscription = machine.StatusOutput().Subscribe(8);

    ASSERT_TRUE(machine.Start());
    EXPECT_TRUE(WaitForState(status_subscription, common::MissionState::kBoot).has_value());
    EXPECT_TRUE(WaitForState(status_subscription, common::MissionState::kSelfCheck).has_value());

    machine.Stop();
}

TEST(MissionStateMachineTest, NavigationReadyFlightMovesToReady) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    state_machine::MissionStateMachine machine;
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    auto status_subscription = machine.StatusOutput().Subscribe(8);

    ASSERT_TRUE(machine.Start());
    (void)flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(MakeNavigationReadyFlight()));

    const auto ready_status = WaitForState(status_subscription, common::MissionState::kReady);
    ASSERT_TRUE(ready_status.has_value());
    EXPECT_EQ(machine.CurrentState(), common::MissionState::kReady);

    machine.Stop();
}

TEST(MissionStateMachineTest, FreshGroundTargetMovesToGpsApproachShadowState) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    state_machine::MissionStateMachine machine;
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    auto intent_subscription = machine.IntentOutput().Subscribe(2);

    ASSERT_TRUE(machine.Start());
    (void)flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(MakeNavigationReadyFlight()));
    ASSERT_TRUE(WaitForState(status_subscription, common::MissionState::kReady).has_value());

    (void)ground_target_topic.Publish(
        std::make_shared<const common::GroundStationTarget>(MakeFreshGroundTarget()));

    const auto gps_status =
        WaitForState(status_subscription, common::MissionState::kGpsApproach);
    ASSERT_TRUE(gps_status.has_value());
    EXPECT_EQ(gps_status->control_source, 0U);
    EXPECT_FALSE(intent_subscription.WaitTakeFor(150ms).has_value());

    machine.Stop();
}

TEST(MissionStateMachineTest, TargetWithoutNavigationDoesNotLeaveSelfCheck) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    state_machine::MissionStateMachine machine;
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    auto status_subscription = machine.StatusOutput().Subscribe(8);

    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForState(status_subscription, common::MissionState::kSelfCheck).has_value());
    (void)ground_target_topic.Publish(
        std::make_shared<const common::GroundStationTarget>(MakeFreshGroundTarget()));

    EXPECT_FALSE(WaitForState(status_subscription, common::MissionState::kGpsApproach, 300ms)
                     .has_value());
    EXPECT_EQ(machine.CurrentState(), common::MissionState::kSelfCheck);

    machine.Stop();
}

/// 等待一条控制意图；超时返回 nullopt。
std::optional<common::ControlIntent> WaitForIntent(
    common::Topic<common::ControlIntent>::Subscription& subscription,
    std::chrono::milliseconds timeout = 1500ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = subscription.WaitTakeFor(50ms);
        if (message) {
            return **message;
        }
    }
    return std::nullopt;
}

/// 驱动状态机进入 GPS_APPROACH（导航就绪 + 新鲜地面站目标）。
void DriveToGpsApproach(
    common::Topic<common::FlightStateSnapshot>& flight_topic,
    common::Topic<common::GroundStationTarget>& ground_target_topic,
    common::Topic<common::MissionStatus>::Subscription& status_subscription) {
    (void)flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(MakeNavigationReadyFlight()));
    ASSERT_TRUE(WaitForState(status_subscription, common::MissionState::kReady).has_value());
    (void)ground_target_topic.Publish(
        std::make_shared<const common::GroundStationTarget>(MakeFreshGroundTarget()));
    ASSERT_TRUE(
        WaitForState(status_subscription, common::MissionState::kGpsApproach).has_value());
}

// 阶段四 Task 7：绑定估计/视觉/测距输入后，S5 发布位置引导意图——指向估计目标、
// control_source=0（位置链）、reason_code 带 bit31 控制权请求位、有效期来自配置。
TEST(MissionStateMachineTest, GpsApproachPublishesPositionIntent) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    common::Topic<common::TargetState> estimated_topic;
    common::Topic<common::VisualTargetStatus> visual_status_topic;
    common::Topic<common::StereoTargetDistance> distance_topic;
    state_machine::MissionStateMachine machine;
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    machine.SetEstimationInputs(estimated_topic, visual_status_topic, distance_topic);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    auto intent_subscription = machine.IntentOutput().Subscribe(8);

    ASSERT_TRUE(machine.Start());
    DriveToGpsApproach(flight_topic, ground_target_topic, status_subscription);
    (void)estimated_topic.Publish(
        std::make_shared<const common::TargetState>(MakeValidEstimatedTarget()));

    const auto intent = WaitForIntent(intent_subscription);
    ASSERT_TRUE(intent.has_value());
    EXPECT_EQ(intent->type, common::ControlIntentType::kPositionGuide);
    EXPECT_EQ(intent->control_source, 0U);
    EXPECT_FLOAT_EQ(intent->target_x, 12.5f);
    EXPECT_FLOAT_EQ(intent->target_y, -3.0f);
    EXPECT_FLOAT_EQ(intent->target_z, -8.0f);
    EXPECT_EQ(intent->header.frame_id, kFrameLocalNed);
    EXPECT_NE(intent->reason_code & common::kControlIntentReasonRequestControl, 0U);
    EXPECT_EQ(intent->header.valid_for_ms, 300U);
    EXPECT_EQ(intent->valid_until_ms,
              intent->header.receive_time_ms + intent->header.valid_for_ms);

    machine.Stop();
}

// 意图有效期必须来自注入的配置（与 controller.intent_timeout 对齐），不写死字面量。
TEST(MissionStateMachineTest, IntentValidityComesFromConfig) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    common::Topic<common::TargetState> estimated_topic;
    common::Topic<common::VisualTargetStatus> visual_status_topic;
    common::Topic<common::StereoTargetDistance> distance_topic;
    state_machine::MissionStateMachineConfig config;
    config.intent_valid_for = 450ms;
    state_machine::MissionStateMachine machine(config);
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    machine.SetEstimationInputs(estimated_topic, visual_status_topic, distance_topic);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    auto intent_subscription = machine.IntentOutput().Subscribe(8);

    ASSERT_TRUE(machine.Start());
    DriveToGpsApproach(flight_topic, ground_target_topic, status_subscription);
    (void)estimated_topic.Publish(
        std::make_shared<const common::TargetState>(MakeValidEstimatedTarget()));

    const auto intent = WaitForIntent(intent_subscription);
    ASSERT_TRUE(intent.has_value());
    EXPECT_EQ(intent->header.valid_for_ms, 450U);
    EXPECT_EQ(intent->valid_until_ms, intent->header.receive_time_ms + 450U);

    machine.Stop();
}

// 估计失效（TargetState.valid=false，如地面站目标过期）后停止发布位置引导意图；
// 超龄收口由 FlightController 意图超时门禁接管。
TEST(MissionStateMachineTest, IntentStopsWhenEstimateInvalid) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    common::Topic<common::TargetState> estimated_topic;
    common::Topic<common::VisualTargetStatus> visual_status_topic;
    common::Topic<common::StereoTargetDistance> distance_topic;
    state_machine::MissionStateMachine machine;
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    machine.SetEstimationInputs(estimated_topic, visual_status_topic, distance_topic);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    auto intent_subscription = machine.IntentOutput().Subscribe(8);

    ASSERT_TRUE(machine.Start());
    DriveToGpsApproach(flight_topic, ground_target_topic, status_subscription);
    (void)estimated_topic.Publish(
        std::make_shared<const common::TargetState>(MakeValidEstimatedTarget()));
    ASSERT_TRUE(WaitForIntent(intent_subscription).has_value());

    (void)estimated_topic.Publish(
        std::make_shared<const common::TargetState>(MakeInvalidEstimatedTarget()));
    // 等待至少两个决策周期，确保状态机已观测到失效估计并停发。
    std::this_thread::sleep_for(250ms);
    while (intent_subscription.TryTake().has_value()) {
    }
    EXPECT_FALSE(intent_subscription.WaitTakeFor(300ms).has_value());

    machine.Stop();
}

}  // namespace
