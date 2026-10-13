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

// ---------------------------------------------------------------------------
// 第二阶段：双目视觉观测融合（docs/物理追踪思路.md §6.3）
// 语义：视觉只细化、不初始化、不延展有效期；过期政策归状态机。
// ---------------------------------------------------------------------------

/// 完整快照：Home + 零姿态 + 原点局部位置（视觉转换所需全部字段有效）。
common::FlightStateSnapshot MakeFlightFull() {
    common::FlightStateSnapshot state = MakeHome();
    state.attitude_valid = true;
    state.roll_rad = 0.0F;
    state.pitch_rad = 0.0F;
    state.yaw_rad = 0.0F;
    state.local_position_valid = true;
    state.local_x_m = 0.0F;
    state.local_y_m = 0.0F;
    state.local_z_m = 0.0F;
    return state;
}

/// 视觉观测：准直安装 + 零姿态 + 自身原点下，cam(x,y,z) → NED(z, x, y)。
common::StereoTargetDistance MakeVisual(uint64_t frame_sequence, float z_m,
                                        float x_m = 0.0F, float y_m = 0.0F,
                                        uint64_t source_time_ms = 0) {
    common::StereoTargetDistance msg;
    msg.header.sequence = frame_sequence;
    msg.header.source_time_ms =
        source_time_ms == 0 ? NowMs() : source_time_ms;
    msg.header.receive_time_ms = NowMs();
    msg.frame_sequence = frame_sequence;
    msg.valid = true;
    msg.confidence = 0.9F;
    msg.forward_distance_m = z_m;
    msg.camera_x_m = x_m;
    msg.camera_y_m = y_m;
    return msg;
}

TargetEstimatorConfig MakeFusionConfig() {
    TargetEstimatorConfig config;
    config.publish_interval = std::chrono::milliseconds(10);
    config.visual.enabled = true;
    config.visual.fxb_m_px = 63.5665;
    // 放宽时间门禁，测试时序不被调度抖动影响（门禁本身有专门用例覆盖）
    config.visual.max_observation_age_ms = std::chrono::milliseconds(1000);
    config.visual.flight_state_max_age_ms = std::chrono::milliseconds(2000);
    return config;
}

/// 融合测试装配：估计器 + 三路 Topic；fusion_on=false 时视觉不接线
///（对应 runtime 开关关闭的装配行为）。
struct FusionFixture {
    explicit FusionFixture(bool fusion_on = true)
        : config(MakeFusionConfig()), estimator(config) {
        config.visual.enabled = fusion_on;
        estimator.SetGroundTargetInput(ground);
        estimator.SetFlightStateInput(flight);
        if (fusion_on) {
            estimator.SetVisualDistanceInput(visual);
        }
    }

    void PublishFlight() {
        ASSERT_TRUE(flight.Publish(
            std::make_shared<const common::FlightStateSnapshot>(
                MakeFlightFull()))
                .accepted);
    }

    /// 地面站初始化滤波器（原点目标：N≈0/E≈0，无速度/高度观测）。
    void InitByGroundTarget(uint32_t sequence = 1) {
        PublishFlight();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        auto target = MakeTarget(sequence);
        target.latitude_1e7 = 300000000;  // 与 Home 同点 → NED 原点
        target.altitude_mm = 100000;
        target.validity_flags = 0;  // 无高度/速度/精度 → 默认 std
        target.horizontal_accuracy_m = 0.0F;
        target.vertical_accuracy_m = 0.0F;
        ASSERT_TRUE(ground.Publish(
            std::make_shared<const common::GroundStationTarget>(target))
                .accepted);
    }

    void PublishVisual(const common::StereoTargetDistance& msg) {
        ASSERT_TRUE(visual.Publish(
            std::make_shared<const common::StereoTargetDistance>(msg))
                .accepted);
    }

    /// 轮询直到地面站更新计数达到 expected（或超时）。
    bool WaitGroundUpdates(uint64_t expected, int timeout_ms = 1000) {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            if (estimator.UpdateCount() >= expected) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
    }

    /// 轮询直到视觉更新计数达到 expected（或超时）。
    bool WaitVisualUpdates(uint64_t expected, int timeout_ms = 1000) {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            if (estimator.VisualUpdateCount() >= expected) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
    }

    TargetEstimatorConfig config;
    common::Topic<common::GroundStationTarget> ground;
    common::Topic<common::FlightStateSnapshot> flight;
    common::Topic<common::StereoTargetDistance> visual;
    TargetEstimator estimator;
};

TEST(TargetEstimatorFusionTest, EnabledWithoutVisualInputFailsStart) {
    TargetEstimator estimator(MakeFusionConfig());
    common::Topic<common::GroundStationTarget> ground;
    common::Topic<common::FlightStateSnapshot> flight;
    estimator.SetGroundTargetInput(ground);
    estimator.SetFlightStateInput(flight);
    // 融合启用但视觉未接线：fail-fast（与未绑地面站/PX4 一致）
    EXPECT_FALSE(estimator.Start());
    EXPECT_EQ(estimator.ErrorCount(), 1U);
}

TEST(TargetEstimatorFusionTest, DisabledIgnoresVisualMessages) {
    TargetEstimatorConfig config = MakeFusionConfig();
    config.visual.enabled = false;
    TargetEstimator estimator(config);
    common::Topic<common::GroundStationTarget> ground;
    common::Topic<common::FlightStateSnapshot> flight;
    estimator.SetGroundTargetInput(ground);
    estimator.SetFlightStateInput(flight);
    // 开关关闭时装配层不接线（drone_application 行为）；这里直接构造估计器
    // 也不接线，验证组件在开关关闭时无任何视觉行为
    ASSERT_TRUE(estimator.Start());
    ASSERT_TRUE(flight.Publish(
        std::make_shared<const common::FlightStateSnapshot>(MakeFlightFull()))
            .accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    estimator.Stop();
    EXPECT_EQ(estimator.VisualUpdateCount(), 0U);
    EXPECT_EQ(estimator.ErrorCount(), 0U);
}

TEST(TargetEstimatorFusionTest, VisualDoesNotInitializeFilter) {
    FusionFixture f;
    ASSERT_TRUE(f.estimator.Start());
    auto output = f.estimator.EstimatedOutput().Subscribe(4);
    f.PublishFlight();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    // 无地面站目标：视觉观测不造目标、不计错误、不产生输出
    f.PublishVisual(MakeVisual(1, 10.0F));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    f.estimator.Stop();
    EXPECT_EQ(f.estimator.VisualUpdateCount(), 0U);
    EXPECT_EQ(f.estimator.ErrorCount(), 0U);
    EXPECT_FALSE(f.estimator.LastState().valid);
}

TEST(TargetEstimatorFusionTest, VisualUpdatesPullEstimateTowardObservation) {
    FusionFixture f;
    ASSERT_TRUE(f.estimator.Start());
    f.InitByGroundTarget();  // 初始化于 NED 原点（默认 std 10m，拉力弱）
    // 视觉观测：前方 10m（cam z=10 → NED north=10）；σ≈0.79m 拉力强
    for (uint64_t i = 1; i <= 5; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
        f.PublishFlight();
        f.PublishVisual(MakeVisual(i, 10.0F));
    }
    ASSERT_TRUE(f.WaitVisualUpdates(5));
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    f.estimator.Stop();
    const auto state = f.estimator.LastState();
    EXPECT_TRUE(state.valid);
    EXPECT_GT(state.pos_x_m, 5.0F);   // 从原点被显著拉向 north=10
    EXPECT_NEAR(state.pos_y_m, 0.0F, 1.0F);
}

TEST(TargetEstimatorFusionTest, VisualOlderThanStateRejected) {
    FusionFixture f;
    ASSERT_TRUE(f.estimator.Start());
    f.InitByGroundTarget();
    ASSERT_TRUE(f.WaitGroundUpdates(1));  // 等初始化落地
    const uint64_t errors_before = f.estimator.ErrorCount();
    // 观测时刻早于滤波状态（但不超龄）：视觉直接拒绝
    f.PublishVisual(MakeVisual(1, 10.0F, 0.0F, 0.0F, NowMs() - 150));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    f.estimator.Stop();
    EXPECT_EQ(f.estimator.VisualUpdateCount(), 0U);
    EXPECT_GT(f.estimator.ErrorCount(), errors_before);
}

TEST(TargetEstimatorFusionTest, LateGroundTargetDegradedNotStarved) {
    FusionFixture f;
    ASSERT_TRUE(f.estimator.Start());
    f.InitByGroundTarget(1);
    // 视觉推前状态时刻
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    f.PublishFlight();
    f.PublishVisual(MakeVisual(1, 10.0F));
    ASSERT_TRUE(f.WaitVisualUpdates(1));
    // 地面站晚到观测（transport_age 使 obs_time 早于滤波状态）：
    // 开关开 → 原地降级更新，不拒绝不饿死
    auto late = MakeTarget(2);
    late.transport_age_ms = 300;
    ASSERT_TRUE(f.ground.Publish(
        std::make_shared<const common::GroundStationTarget>(late)).accepted);
    ASSERT_TRUE(f.WaitGroundUpdates(2));
    f.estimator.Stop();
    EXPECT_EQ(f.estimator.UpdateCount(), 2U);  // 两条地面站观测都接受
}

TEST(TargetEstimatorFusionTest, LateGroundTargetRejectedWhenFusionOff) {
    // 回归：开关关闭时保持原有拒绝行为
    TargetEstimatorConfig config;
    config.publish_interval = std::chrono::milliseconds(10);
    TargetEstimator estimator(config);
    common::Topic<common::GroundStationTarget> ground;
    common::Topic<common::FlightStateSnapshot> flight;
    estimator.SetGroundTargetInput(ground);
    estimator.SetFlightStateInput(flight);
    ASSERT_TRUE(estimator.Start());
    ASSERT_TRUE(flight.Publish(
        std::make_shared<const common::FlightStateSnapshot>(MakeHome()))
            .accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ASSERT_TRUE(ground.Publish(
        std::make_shared<const common::GroundStationTarget>(MakeTarget(1)))
            .accepted);
    auto late = MakeTarget(2);
    late.transport_age_ms = 300;  // obs 早于状态 → 拒绝
    ASSERT_TRUE(ground.Publish(
        std::make_shared<const common::GroundStationTarget>(late)).accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    estimator.Stop();
    EXPECT_EQ(estimator.UpdateCount(), 1U);
    EXPECT_GE(estimator.ErrorCount(), 1U);
}

TEST(TargetEstimatorFusionTest, RejectsInvalidOrUntimelyVisualMessages) {
    FusionFixture f;
    ASSERT_TRUE(f.estimator.Start());
    f.PublishFlight();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    auto invalid = MakeVisual(1, 10.0F);
    invalid.valid = false;
    f.PublishVisual(invalid);  // valid=false

    auto nan_msg = MakeVisual(2, std::numeric_limits<float>::quiet_NaN());
    f.PublishVisual(nan_msg);  // NaN 距离

    auto zero_time = MakeVisual(3, 10.0F);
    zero_time.header.source_time_ms = 0;
    f.PublishVisual(zero_time);  // 时刻为 0

    f.PublishVisual(MakeVisual(4, 10.0F, 0.0F, 0.0F, NowMs() + 60000));  // 未来

    f.PublishVisual(MakeVisual(5, 10.0F, 0.0F, 0.0F,
                               NowMs() - 60000));  // 超龄

    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    f.estimator.Stop();
    EXPECT_EQ(f.estimator.VisualUpdateCount(), 0U);
    EXPECT_EQ(f.estimator.ErrorCount(), 5U);
}

TEST(TargetEstimatorFusionTest, RejectsSecondBoxOfSameFrame) {
    FusionFixture f;
    ASSERT_TRUE(f.estimator.Start());
    f.InitByGroundTarget();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    f.PublishFlight();
    f.PublishVisual(MakeVisual(100, 10.0F));
    ASSERT_TRUE(f.WaitVisualUpdates(1));
    const uint64_t errors_before = f.estimator.ErrorCount();
    // 同一左目帧的第二个框：多框歧义防线拒绝
    f.PublishVisual(MakeVisual(100, 12.0F));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    f.estimator.Stop();
    EXPECT_EQ(f.estimator.VisualUpdateCount(), 1U);
    EXPECT_GT(f.estimator.ErrorCount(), errors_before);
}

TEST(TargetEstimatorFusionTest, VisualRejectedWhenAttitudeOrPositionInvalid) {
    FusionFixture f;
    ASSERT_TRUE(f.estimator.Start());
    f.InitByGroundTarget();
    const uint64_t errors_before = f.estimator.ErrorCount();

    // 姿态无效
    auto no_attitude = MakeFlightFull();
    no_attitude.attitude_valid = false;
    ASSERT_TRUE(f.flight.Publish(
        std::make_shared<const common::FlightStateSnapshot>(no_attitude))
            .accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    f.PublishVisual(MakeVisual(1, 10.0F));
    std::this_thread::sleep_for(std::chrono::milliseconds(40));

    // 位置无效（姿态恢复有效）
    auto no_position = MakeFlightFull();
    no_position.local_position_valid = false;
    ASSERT_TRUE(f.flight.Publish(
        std::make_shared<const common::FlightStateSnapshot>(no_position))
            .accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    f.PublishVisual(MakeVisual(2, 10.0F));
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    f.estimator.Stop();
    EXPECT_EQ(f.estimator.VisualUpdateCount(), 0U);
    EXPECT_EQ(f.estimator.ErrorCount(), errors_before + 2);
}

TEST(TargetEstimatorFusionTest, VisualRejectedWhenFlightSnapshotStale) {
    // 窄超龄门限（50ms）：快照超龄后视觉拒绝
    TargetEstimatorConfig config = MakeFusionConfig();
    config.visual.flight_state_max_age_ms = std::chrono::milliseconds(50);
    TargetEstimator estimator(config);
    common::Topic<common::GroundStationTarget> ground;
    common::Topic<common::FlightStateSnapshot> flight;
    common::Topic<common::StereoTargetDistance> visual;
    estimator.SetGroundTargetInput(ground);
    estimator.SetFlightStateInput(flight);
    estimator.SetVisualDistanceInput(visual);
    ASSERT_TRUE(estimator.Start());
    ASSERT_TRUE(flight.Publish(
        std::make_shared<const common::FlightStateSnapshot>(MakeFlightFull()))
            .accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    auto target = MakeTarget(1);
    target.latitude_1e7 = 300000000;
    target.altitude_mm = 100000;
    target.validity_flags = 0;
    ASSERT_TRUE(ground.Publish(
        std::make_shared<const common::GroundStationTarget>(target)).accepted);
    // 快照超龄（>50ms 未刷新）后视觉拒绝
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    ASSERT_TRUE(visual.Publish(
        std::make_shared<const common::StereoTargetDistance>(
            MakeVisual(1, 10.0F)))
            .accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    estimator.Stop();
    EXPECT_EQ(estimator.VisualUpdateCount(), 0U);
    EXPECT_GE(estimator.ErrorCount(), 1U);
}

TEST(TargetEstimatorFusionTest, VisualSkippedAfterGroundExpiry) {
    FusionFixture f;
    ASSERT_TRUE(f.estimator.Start());
    f.InitByGroundTarget();
    // 视觉在有效期内正常更新
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    f.PublishFlight();
    f.PublishVisual(MakeVisual(1, 10.0F));
    ASSERT_TRUE(f.WaitVisualUpdates(1));
    // 等地面站有效期（1000ms）过后再发视觉：不延展、不计错误
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    const uint64_t errors_before = f.estimator.ErrorCount();
    f.PublishFlight();
    f.PublishVisual(MakeVisual(2, 10.0F));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    f.estimator.Stop();
    EXPECT_EQ(f.estimator.VisualUpdateCount(), 1U);  // 过期后未再更新
    EXPECT_EQ(f.estimator.ErrorCount(), errors_before);
    EXPECT_FALSE(f.estimator.LastState().valid);  // 过期语义不变
}

TEST(TargetEstimatorFusionTest, HomeChangeResetsVisualBookkeeping) {
    FusionFixture f;
    ASSERT_TRUE(f.estimator.Start());
    f.InitByGroundTarget();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    f.PublishFlight();
    f.PublishVisual(MakeVisual(1, 10.0F));
    ASSERT_TRUE(f.WaitVisualUpdates(1));

    // Home 变化 → 滤波器重置；此后视觉在无新地面站目标时不再更新
    auto moved = MakeFlightFull();
    moved.home_lat_1e7 = 300100000;
    ASSERT_TRUE(f.flight.Publish(
        std::make_shared<const common::FlightStateSnapshot>(moved)).accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    // 滤波器重置同时清空视觉创新诊断值（不得残留上一原点的创新）
    EXPECT_TRUE(std::isnan(f.estimator.LastVisualInnovationM()));
    const uint64_t errors_before = f.estimator.ErrorCount();
    f.PublishVisual(MakeVisual(2, 10.0F));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    f.estimator.Stop();
    EXPECT_EQ(f.estimator.VisualUpdateCount(), 1U);
    EXPECT_EQ(f.estimator.ErrorCount(), errors_before);
}

TEST(TargetEstimatorFusionTest, MountYawRotatesVisualObservation) {
    FusionFixture f;
    f.estimator.Stop();  // 重建以替换配置
    TargetEstimatorConfig config = MakeFusionConfig();
    config.visual.mount.yaw_deg = 45.0;  // 相机光轴相对机体右偏 45°
    TargetEstimator estimator(config);
    common::Topic<common::GroundStationTarget> ground;
    common::Topic<common::FlightStateSnapshot> flight;
    common::Topic<common::StereoTargetDistance> visual;
    estimator.SetGroundTargetInput(ground);
    estimator.SetFlightStateInput(flight);
    estimator.SetVisualDistanceInput(visual);
    ASSERT_TRUE(estimator.Start());
    ASSERT_TRUE(flight.Publish(
        std::make_shared<const common::FlightStateSnapshot>(MakeFlightFull()))
            .accepted);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    auto target = MakeTarget(1);
    target.latitude_1e7 = 300000000;  // 原点初始化
    target.altitude_mm = 100000;
    target.validity_flags = 0;
    ASSERT_TRUE(ground.Publish(
        std::make_shared<const common::GroundStationTarget>(target)).accepted);
    // 相机正前方 10m，光轴右偏 45° → NED (10·cos45, 10·sin45, 0)
    for (uint64_t i = 1; i <= 6; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
        ASSERT_TRUE(flight.Publish(
            std::make_shared<const common::FlightStateSnapshot>(
                MakeFlightFull()))
                .accepted);
        ASSERT_TRUE(visual.Publish(
            std::make_shared<const common::StereoTargetDistance>(
                MakeVisual(i, 10.0F)))
                .accepted);
    }
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        if (estimator.VisualUpdateCount() >= 6) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    estimator.Stop();
    const auto state = estimator.LastState();
    ASSERT_TRUE(state.valid);
    // 东向偏移显著为正且与北向同量级（45° 旋转生效）
    EXPECT_GT(state.pos_y_m, 2.0F);
    EXPECT_NEAR(state.pos_x_m, state.pos_y_m, 2.0F);
}

TEST(TargetEstimatorFusionTest, InnovationDiagnosticsReflectsObservationOffset) {
    // 交接创新门限（阶段四 Task 8）的数据来源：最近一次接受的视觉观测与
    // 卡尔曼预测的水平距离，须在观测-预测差距大时显著、收敛后显著减小；
    // 无任何视觉更新时为 NaN（表示无值，不被门限误当作"一致"）。
    FusionFixture f;
    ASSERT_TRUE(f.estimator.Start());
    f.InitByGroundTarget();  // 原点初始化：预测在 NED 原点
    ASSERT_TRUE(f.WaitGroundUpdates(1));
    EXPECT_TRUE(std::isnan(f.estimator.LastVisualInnovationM()));

    // 视觉观测在前方 10m（cam z=10 → NED north=10）：创新≈10m
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    f.PublishFlight();
    f.PublishVisual(MakeVisual(1, 10.0F));
    ASSERT_TRUE(f.WaitVisualUpdates(1));
    const double first = f.estimator.LastVisualInnovationM();
    EXPECT_NEAR(first, 10.0, 0.5);

    // 同位置第二条观测（滤波器已被第一条拉近）：创新显著减小但不为负/NaN
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    f.PublishFlight();
    f.PublishVisual(MakeVisual(2, 10.0F));
    ASSERT_TRUE(f.WaitVisualUpdates(2));
    const double second = f.estimator.LastVisualInnovationM();
    f.estimator.Stop();
    EXPECT_GT(second, 0.0);
    EXPECT_LT(second, first);
}

TEST(TargetEstimatorFusionTest, FarObservationPullsWeakerThanNear) {
    // σ_z = z²·σ_d/fxb 随距离平方增长：同样的更新次数下远距观测拉力弱
    auto run_scenario = [](float z_m) {
        FusionFixture f;
        EXPECT_TRUE(f.estimator.Start());
        f.InitByGroundTarget();  // 原点初始化
        for (uint64_t i = 1; i <= 5; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            f.PublishFlight();
            f.PublishVisual(MakeVisual(i, z_m));  // 正前方 z_m → NED north=z
        }
        EXPECT_TRUE(f.WaitVisualUpdates(5));
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        f.estimator.Stop();
        return static_cast<double>(f.estimator.LastState().pos_x_m);
    };
    const double near_north = run_scenario(20.0F);  // σ≈3.1m
    const double far_north = run_scenario(60.0F);   // σ≈28.3m
    // 归一化收敛度：近距显著更接近各自目标点
    EXPECT_GT(near_north / 20.0, 0.3);
    EXPECT_GT(near_north / 20.0, 2.0 * (far_north / 60.0));
}

}  // namespace
}  // namespace drone::perception
