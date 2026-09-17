#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "common/topic.h"
#include "common/types.h"
#include "perception/geodetic_converter.h"
#include "perception/linear_target_kalman_filter.h"
#include "perception/target_estimator.h"

namespace drone::perception {
namespace {

uint64_t NowMs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

common::FlightStateSnapshot MakeHome() {
    common::FlightStateSnapshot state;
    state.home_valid = true;
    state.home_lat_1e7 = 300000000;
    state.home_lon_1e7 = 1200000000;
    state.home_altitude_mm = 100000;
    state.header.receive_time_ms = NowMs();
    return state;
}

common::GroundStationTarget MakeTarget(uint32_t sequence = 1) {
    common::GroundStationTarget target;
    target.header.sequence = sequence;
    target.header.receive_time_ms = NowMs();
    target.header.valid_for_ms = 1000;
    target.ground_station_boot_id = 7;
    target.update_seq = sequence;
    target.target_id = 42;
    target.latitude_1e7 = 300010000;   // 约向北111米
    target.longitude_1e7 = 1200000000;
    target.altitude_mm = 120000;
    target.alt_reference = 1;
    target.horizontal_accuracy_m = 2.0F;
    target.vertical_accuracy_m = 3.0F;
    target.validity_flags = (1U << 0U) | (1U << 1U) | (1U << 2U) |
                            (1U << 5U) | (1U << 6U);
    target.velocity_north_mps = 4.0F;
    target.velocity_east_mps = -1.0F;
    return target;
}

TEST(GeodeticConverterTest, SameCoordinateIsOrigin) {
    const GeodeticCoordinate origin{30.0, 120.0, 100.0};
    const auto ned = GeodeticToLocalNed(origin, origin);
    ASSERT_TRUE(ned.has_value());
    EXPECT_NEAR(ned->north_m, 0.0, 1e-9);
    EXPECT_NEAR(ned->east_m, 0.0, 1e-9);
    EXPECT_NEAR(ned->down_m, 0.0, 1e-9);
}

TEST(GeodeticConverterTest, ConvertsNorthEastAndDownSigns) {
    const auto ned = GeodeticToLocalNed(
        GeodeticCoordinate{30.0, 120.0, 100.0},
        GeodeticCoordinate{30.001, 120.001, 120.0});
    ASSERT_TRUE(ned.has_value());
    EXPECT_GT(ned->north_m, 110.0);
    EXPECT_LT(ned->north_m, 112.0);
    EXPECT_GT(ned->east_m, 95.0);
    EXPECT_LT(ned->east_m, 97.5);
    EXPECT_NEAR(ned->down_m, -20.0, 1e-9);
}

TEST(GeodeticConverterTest, RejectsInvalidLatitude) {
    EXPECT_FALSE(GeodeticToLocalNed(
        GeodeticCoordinate{91.0, 120.0, 0.0},
        GeodeticCoordinate{30.0, 120.0, 0.0}).has_value());
}

TEST(LinearTargetKalmanFilterTest, InitializesAndPredictsVelocity) {
    LinearTargetKalmanFilter filter;
    LinearTargetMeasurement measurement;
    measurement.north_m = 10.0;
    measurement.east_m = 20.0;
    measurement.horizontal_position_std_m = 2.0;
    measurement.velocity_north_valid = true;
    measurement.velocity_east_valid = true;
    measurement.velocity_north_mps = 3.0;
    measurement.velocity_east_mps = -2.0;

    ASSERT_TRUE(filter.Initialize(measurement));
    ASSERT_TRUE(filter.Predict(2.0));
    const auto state = filter.State();
    EXPECT_TRUE(state.horizontal_valid);
    EXPECT_FALSE(state.vertical_valid);
    EXPECT_NEAR(state.north_m, 16.0, 1e-9);
    EXPECT_NEAR(state.east_m, 16.0, 1e-9);
}

TEST(LinearTargetKalmanFilterTest, LaterAltitudeObservationInitializesVerticalAxis) {
    LinearTargetKalmanFilter filter;
    LinearTargetMeasurement first;
    first.horizontal_position_std_m = 1.0;
    ASSERT_TRUE(filter.Initialize(first));
    EXPECT_FALSE(filter.State().vertical_valid);

    LinearTargetMeasurement second = first;
    second.down_valid = true;
    second.down_m = -30.0;
    second.vertical_position_std_m = 2.0;
    ASSERT_TRUE(filter.Update(second));
    EXPECT_TRUE(filter.State().vertical_valid);
    EXPECT_NEAR(filter.State().down_m, -30.0, 1e-9);
}

TEST(LinearTargetKalmanFilterTest, PositionUpdatesInferVelocity) {
    LinearTargetKalmanFilter filter;
    LinearTargetMeasurement first;
    first.horizontal_position_std_m = 1.0;
    ASSERT_TRUE(filter.Initialize(first));
    ASSERT_TRUE(filter.Predict(1.0));

    LinearTargetMeasurement second = first;
    second.north_m = 10.0;
    ASSERT_TRUE(filter.Update(second));
    const auto state = filter.State();
    EXPECT_GT(state.north_m, 9.0);
    EXPECT_GT(state.velocity_north_mps, 5.0);
}

TEST(LinearTargetKalmanFilterTest, RejectsInvalidPredictionStep) {
    LinearTargetKalmanFilter filter;
    LinearTargetMeasurement measurement;
    measurement.horizontal_position_std_m = 1.0;
    ASSERT_TRUE(filter.Initialize(measurement));
    EXPECT_FALSE(filter.Predict(-0.1));
    EXPECT_FALSE(filter.Predict(10.1));
    EXPECT_FALSE(filter.Predict(std::numeric_limits<double>::quiet_NaN()));
}

TEST(TargetEstimatorTest, RequiresBothInputsBeforeStart) {
    TargetEstimator estimator;
    EXPECT_FALSE(estimator.Start());
    EXPECT_FALSE(estimator.IsRunning());
    EXPECT_EQ(estimator.ErrorCount(), 1U);
}

TEST(TargetEstimatorTest, PublishesGroundTargetAsLocalNedShadowState) {
    TargetEstimatorConfig config;
    config.publish_interval = std::chrono::milliseconds(10);
    TargetEstimator estimator(config);
    common::Topic<common::GroundStationTarget> ground;
    common::Topic<common::FlightStateSnapshot> flight;
    estimator.SetGroundTargetInput(ground);
    estimator.SetFlightStateInput(flight);
    auto output = estimator.EstimatedOutput().Subscribe(8);
    ASSERT_TRUE(estimator.Start());

    ASSERT_TRUE(flight.Publish(
        std::make_shared<const common::FlightStateSnapshot>(MakeHome())).accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ASSERT_TRUE(ground.Publish(
        std::make_shared<const common::GroundStationTarget>(MakeTarget())).accepted);

    std::optional<std::shared_ptr<const common::TargetState>> received;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        received = output.WaitTakeFor(std::chrono::milliseconds(30));
        if (received.has_value() && (*received)->valid) {
            break;
        }
    }
    estimator.Stop();

    ASSERT_TRUE(received.has_value());
    const auto& state = **received;
    EXPECT_TRUE(state.valid);
    EXPECT_EQ(state.header.frame_id, 3U);
    EXPECT_EQ(state.target_id, 42U);
    EXPECT_GT(state.pos_x_m, 110.0F);
    EXPECT_LT(state.pos_x_m, 112.0F);
    EXPECT_NEAR(state.pos_y_m, 0.0F, 0.2F);
    EXPECT_NEAR(state.pos_z_m, -20.0F, 0.1F);
    EXPECT_NEAR(state.vel_x_mps, 4.0F, 0.1F);
    EXPECT_NEAR(state.vel_y_mps, -1.0F, 0.1F);
    EXPECT_EQ(estimator.UpdateCount(), 1U);
}

TEST(TargetEstimatorTest, MissingAltitudePublishesHorizontalStateWithNanVertical) {
    TargetEstimatorConfig config;
    config.publish_interval = std::chrono::milliseconds(10);
    TargetEstimator estimator(config);
    common::Topic<common::GroundStationTarget> ground;
    common::Topic<common::FlightStateSnapshot> flight;
    estimator.SetGroundTargetInput(ground);
    estimator.SetFlightStateInput(flight);
    auto output = estimator.EstimatedOutput().Subscribe(4);
    ASSERT_TRUE(estimator.Start());

    ASSERT_TRUE(flight.Publish(
        std::make_shared<const common::FlightStateSnapshot>(MakeHome())).accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    auto target = MakeTarget();
    target.validity_flags &= ~(1U << 0U);
    target.alt_reference = 0;
    ASSERT_TRUE(ground.Publish(
        std::make_shared<const common::GroundStationTarget>(target)).accepted);

    const auto received = output.WaitTakeFor(std::chrono::milliseconds(500));
    estimator.Stop();
    ASSERT_TRUE(received.has_value());
    EXPECT_TRUE((*received)->valid);
    EXPECT_TRUE(std::isnan((*received)->pos_z_m));
    EXPECT_TRUE(std::isnan((*received)->vel_z_mps));
}

TEST(TargetEstimatorTest, RejectsAlreadyExpiredGroundTarget) {
    TargetEstimatorConfig config;
    config.publish_interval = std::chrono::milliseconds(10);
    TargetEstimator estimator(config);
    common::Topic<common::GroundStationTarget> ground;
    common::Topic<common::FlightStateSnapshot> flight;
    estimator.SetGroundTargetInput(ground);
    estimator.SetFlightStateInput(flight);
    auto output = estimator.EstimatedOutput().Subscribe(4);
    ASSERT_TRUE(estimator.Start());

    ASSERT_TRUE(flight.Publish(
        std::make_shared<const common::FlightStateSnapshot>(MakeHome())).accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    auto target = MakeTarget();
    target.header.receive_time_ms = NowMs() - 100;
    target.header.valid_for_ms = 50;
    ASSERT_TRUE(ground.Publish(
        std::make_shared<const common::GroundStationTarget>(target)).accepted);

    EXPECT_FALSE(output.WaitTakeFor(std::chrono::milliseconds(100)).has_value());
    estimator.Stop();
    EXPECT_EQ(estimator.UpdateCount(), 0U);
    EXPECT_EQ(estimator.ErrorCount(), 1U);
}

TEST(TargetEstimatorTest, PublishesOneInvalidStateWhenTargetExpires) {
    TargetEstimatorConfig config;
    config.publish_interval = std::chrono::milliseconds(10);
    TargetEstimator estimator(config);
    common::Topic<common::GroundStationTarget> ground;
    common::Topic<common::FlightStateSnapshot> flight;
    estimator.SetGroundTargetInput(ground);
    estimator.SetFlightStateInput(flight);
    auto output = estimator.EstimatedOutput().Subscribe(16);
    ASSERT_TRUE(estimator.Start());

    ASSERT_TRUE(flight.Publish(
        std::make_shared<const common::FlightStateSnapshot>(MakeHome())).accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    auto target = MakeTarget();
    target.header.valid_for_ms = 80;
    ASSERT_TRUE(ground.Publish(
        std::make_shared<const common::GroundStationTarget>(target)).accepted);

    bool saw_valid = false;
    bool saw_invalid = false;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto item = output.WaitTakeFor(std::chrono::milliseconds(30));
        if (!item.has_value()) {
            continue;
        }
        saw_valid = saw_valid || (*item)->valid;
        saw_invalid = saw_invalid || !(*item)->valid;
        if (saw_valid && saw_invalid) {
            break;
        }
    }
    estimator.Stop();
    EXPECT_TRUE(saw_valid);
    EXPECT_TRUE(saw_invalid);
}

TEST(TargetEstimatorTest, TargetIdChangeReinitializesFilter) {
    TargetEstimatorConfig config;
    config.publish_interval = std::chrono::milliseconds(10);
    TargetEstimator estimator(config);
    common::Topic<common::GroundStationTarget> ground;
    common::Topic<common::FlightStateSnapshot> flight;
    estimator.SetGroundTargetInput(ground);
    estimator.SetFlightStateInput(flight);
    auto output = estimator.EstimatedOutput().Subscribe(8);
    ASSERT_TRUE(estimator.Start());

    ASSERT_TRUE(flight.Publish(
        std::make_shared<const common::FlightStateSnapshot>(MakeHome())).accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ASSERT_TRUE(ground.Publish(
        std::make_shared<const common::GroundStationTarget>(MakeTarget(1))).accepted);
    ASSERT_TRUE(output.WaitTakeFor(std::chrono::milliseconds(500)).has_value());

    auto second = MakeTarget(2);
    second.target_id = 99;
    second.latitude_1e7 = 300020000;
    ASSERT_TRUE(ground.Publish(
        std::make_shared<const common::GroundStationTarget>(second)).accepted);

    std::shared_ptr<const common::TargetState> latest;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto item = output.WaitTakeFor(std::chrono::milliseconds(30));
        if (item.has_value() && (*item)->target_id == 99U) {
            latest = *item;
            break;
        }
    }
    estimator.Stop();
    ASSERT_NE(latest, nullptr);
    EXPECT_GT(latest->pos_x_m, 220.0F);
    EXPECT_LT(latest->pos_x_m, 223.5F);
    EXPECT_EQ(estimator.UpdateCount(), 2U);
}

}  // namespace
}  // namespace drone::perception
