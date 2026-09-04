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

}  // namespace
