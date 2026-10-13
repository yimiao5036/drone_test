#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "communication/px4_link.h"
#include "config/config.h"
#include "perception/target_estimator.h"
#include "state_machine/mission_state_machine.h"

#include "../common/log_capture.h"

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

// ---------------------------------------------------------------------------
// 阶段四 Task 8：交接判据（S5→S6→S7 进入/退出迟滞）
//
// 进入（S5→S6）需同时满足：视觉 LOCKED 连续帧达 handover.lock_frames、
// 双目距离有效且未超龄、视线角距视场边缘有余量（handover.fow_margin_deg）、
// 视觉观测与卡尔曼预测创新在门限内（handover.innovation_gate_m）。
// S6 驻留超时（handover.timeout_ms）回 S5；S7 退出需 LOST 连续帧达
// handover.exit_lost_frames（进入/退出门限不同构成迟滞）。
// 本层计数与 VisualTargetMonitor 内部 lock_frames 判据相互独立。
// ---------------------------------------------------------------------------

/// 视觉稳定性状态快照。默认中心 (640, 532) 位于标定内参下视场余量内：
/// 标定主点 cy=532.07 偏离画面中心，画面中心 (640, 360) 反而贴近视场边缘。
common::VisualTargetStatus MakeVisualStatus(common::VisualTargetState state,
                                            uint64_t frame_sequence,
                                            float center_x = 640.0F,
                                            float center_y = 532.0F) {
    common::VisualTargetStatus status;
    status.header.sequence = frame_sequence;
    status.header.source_time_ms = TestMonotonicMs();
    status.header.receive_time_ms = TestMonotonicMs();
    status.header.source_id = 0;
    status.header.health = 1;
    status.state = state;
    status.frame_sequence = frame_sequence;
    status.last_seen_ms = TestMonotonicMs();
    status.consecutive_detected_frames = static_cast<int>(frame_sequence);
    status.confidence = 0.9F;
    status.center_pixel_x = center_x;
    status.center_pixel_y = center_y;
    status.frame_width = 1280;
    status.frame_height = 720;
    status.valid = state == common::VisualTargetState::kLocked;
    return status;
}

/// 双目测距快照（StereoTargetDistance 头不含 valid_for_ms，
/// 新鲜度由状态机按注入的超龄阈值判定）。valid=false 时距离字段为 NaN。
common::StereoTargetDistance MakeFreshDistance(uint64_t frame_sequence,
                                               bool valid = true,
                                               float forward_m = 10.0F) {
    common::StereoTargetDistance msg;
    msg.header.sequence = frame_sequence;
    msg.header.source_time_ms = TestMonotonicMs();
    msg.header.receive_time_ms = TestMonotonicMs();
    msg.frame_sequence = frame_sequence;
    msg.valid = valid;
    msg.confidence = 0.9F;
    if (valid) {
        msg.forward_distance_m = forward_m;
        msg.camera_x_m = 0.0F;
        msg.camera_y_m = 0.0F;
    }
    return msg;
}

/// 交接测试用估计快照：有效期足够长，覆盖用例全程。
common::TargetState MakeHandoverEstimate() {
    common::TargetState state = MakeValidEstimatedTarget();
    state.header.valid_for_ms = 5000;
    state.header.receive_time_ms = TestMonotonicMs();
    return state;
}

/// 估计器诊断假件：只提供交接创新门限所需的值（其余接口占位）。
/// 用假件而不是真估计器，使创新值可精确注入，状态机测试不依赖感知内部时序。
class FakeEstimator final : public perception::ITargetEstimator {
public:
    bool Start() override { return true; }
    void Stop() override {}
    bool IsRunning() const override { return true; }
    void SetGroundTargetInput(
        common::Topic<common::GroundStationTarget>& /*ground*/) override {}
    void SetFlightStateInput(
        common::Topic<common::FlightStateSnapshot>& /*flight*/) override {}
    void SetVisualDistanceInput(
        common::Topic<common::StereoTargetDistance>& /*visual*/) override {}
    common::Topic<common::TargetState>& EstimatedOutput() override {
        return output_;
    }
    common::TargetState LastState() const override { return {}; }
    uint64_t UpdateCount() const override { return 0; }
    uint64_t VisualUpdateCount() const override { return 0; }
    uint64_t ErrorCount() const override { return 0; }
    double LastVisualInnovationM() const override { return innovation_m_; }

    void SetInnovation(double value) { innovation_m_ = value; }

private:
    common::Topic<common::TargetState> output_;
    double innovation_m_ = std::numeric_limits<double>::quiet_NaN();
};

/// 交接测试默认配置：交接阈值取小值（lock=3、exit=6、超时 2s、测距失效 3s）
/// 压缩用例时长；视觉/测距超龄阈值默认放宽（5s），避免用例时序被调度抖动影响，
/// 需要验证超龄门禁的用例单独收紧。
state_machine::MissionStateMachineConfig MakeHandoverTestConfig(
    std::chrono::milliseconds handover_timeout = 2000ms,
    std::chrono::milliseconds distance_lost = 3000ms,
    std::chrono::milliseconds visual_stale = 5000ms,
    std::chrono::milliseconds distance_stale = 5000ms) {
    state_machine::MissionStateMachineConfig config;
    config.visual_stale = visual_stale;
    config.distance_stale = distance_stale;
    config.handover.lock_frames = 3;
    config.handover.exit_lost_frames = 6;
    config.handover.fow_margin_deg = 5.0;
    config.handover.innovation_gate_m = 3.0;
    config.handover.timeout = handover_timeout;
    config.handover.distance_lost_ms = distance_lost;
    return config;
}

/// 交接判据测试夹具：状态机 + 四路输入 + 假估计器诊断。
struct HandoverFixture {
    explicit HandoverFixture(
        const state_machine::MissionStateMachineConfig& custom_config,
        bool inject_diagnostics = true)
        : config(custom_config) {
        estimator.SetInnovation(0.5);  // 默认创新在门限内
        machine = std::make_unique<state_machine::MissionStateMachine>(config);
        machine->SetInputs(ground, flight, health);
        machine->SetEstimationInputs(estimated, visual, distance);
        if (inject_diagnostics) {
            machine->SetEstimationDiagnostics(&estimator);
        }
        status_subscription = machine->StatusOutput().Subscribe(64);
        started = machine->Start();
    }

    explicit HandoverFixture(bool inject_diagnostics = true)
        : HandoverFixture(MakeHandoverTestConfig(), inject_diagnostics) {}

    ~HandoverFixture() { machine->Stop(); }

    /// 驱动到 S5（导航就绪 → 新鲜地面站目标 → 估计/测距有效）。
    void DriveToGpsApproach() {
        (void)flight.Publish(std::make_shared<const common::FlightStateSnapshot>(
            MakeNavigationReadyFlight()));
        ASSERT_TRUE(
            WaitForState(status_subscription, common::MissionState::kReady).has_value());
        (void)ground.Publish(std::make_shared<const common::GroundStationTarget>(
            MakeFreshGroundTarget()));
        ASSERT_TRUE(WaitForState(status_subscription,
                                 common::MissionState::kGpsApproach)
                        .has_value());
        PublishEstimateAndDistance();
    }

    void PublishEstimateAndDistance() {
        (void)estimated.Publish(
            std::make_shared<const common::TargetState>(MakeHandoverEstimate()));
        (void)distance.Publish(std::make_shared<const common::StereoTargetDistance>(
            MakeFreshDistance(++distance_sequence)));
    }

    /// 发布一条视觉状态并等待决策周期消费（决策周期 100ms，留余量）。
    void PublishVisual(const common::VisualTargetStatus& status) {
        (void)visual.Publish(
            std::make_shared<const common::VisualTargetStatus>(status));
        std::this_thread::sleep_for(120ms);
    }

    /// 发布一条测距快照（valid=false 为测距失效）并等待决策周期消费。
    void PublishDistance(bool valid) {
        (void)distance.Publish(std::make_shared<const common::StereoTargetDistance>(
            MakeFreshDistance(++distance_sequence, valid)));
        std::this_thread::sleep_for(120ms);
    }

    /// 驱动进入 S7：S5 → 连续 3 帧 LOCKED → S6 → 再推进一帧新 LOCKED → S7。
    void DriveToVisualTracking() {
        DriveToGpsApproach();
        for (uint64_t frame = 1; frame <= 3; ++frame) {
            PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, frame));
        }
        ASSERT_TRUE(WaitForState(status_subscription,
                                 common::MissionState::kVisualHandover)
                        .has_value());
        PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, 4));
        ASSERT_TRUE(WaitForState(status_subscription,
                                 common::MissionState::kVisualTracking)
                        .has_value());
    }

    state_machine::MissionStateMachineConfig config;
    common::Topic<common::GroundStationTarget> ground;
    common::Topic<common::FlightStateSnapshot> flight;
    common::Topic<common::HealthStatus> health;
    common::Topic<common::TargetState> estimated;
    common::Topic<common::VisualTargetStatus> visual;
    common::Topic<common::StereoTargetDistance> distance;
    common::Topic<common::MissionStatus>::Subscription status_subscription;
    FakeEstimator estimator;
    std::unique_ptr<state_machine::MissionStateMachine> machine;
    bool started = false;
    uint64_t distance_sequence = 0;
};

// LOCKED 断续不累计：N-1 条连续 LOCKED 被中断后不进入 S6；再补齐连续 N 条才进 S6。
TEST(MissionStateMachineTest, HandoverRequiresConsecutiveLockFrames) {
    HandoverFixture f;
    ASSERT_TRUE(f.started);
    f.DriveToGpsApproach();

    // LOCKED(1) LOCKED(2) LOST(3) LOCKED(4) LOCKED(5)：任一刻连续锁定 < 3
    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, 1));
    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, 2));
    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLost, 3));
    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, 4));
    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, 5));
    EXPECT_FALSE(WaitForState(f.status_subscription,
                              common::MissionState::kVisualHandover, 300ms)
                     .has_value());
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kGpsApproach);

    // 第 3 条连续 LOCKED（断续累计是第 5 条，必须不生效）→ 进入 S6
    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, 6));
    EXPECT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kVisualHandover)
                    .has_value());
}

// 创新门限：无有效创新（NaN）或观测-预测水平距离超门限均不放行交接（防错锁伪目标）。
TEST(MissionStateMachineTest, HandoverRejectsInnovationBeyondGate) {
    HandoverFixture f;
    ASSERT_TRUE(f.started);
    f.DriveToGpsApproach();

    f.estimator.SetInnovation(std::numeric_limits<double>::quiet_NaN());
    for (uint64_t frame = 1; frame <= 4; ++frame) {
        f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, frame));
    }
    // 连续锁定帧数已超门限，但创新无值 → 不放行
    EXPECT_FALSE(WaitForState(f.status_subscription,
                              common::MissionState::kVisualHandover, 300ms)
                     .has_value());

    f.estimator.SetInnovation(5.0);  // 超 innovation_gate_m=3.0
    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, 5));
    EXPECT_FALSE(WaitForState(f.status_subscription,
                              common::MissionState::kVisualHandover, 300ms)
                     .has_value());
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kGpsApproach);

    f.estimator.SetInnovation(0.5);  // 回到门限内 → 连续锁定条件已满足，进入 S6
    EXPECT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kVisualHandover)
                    .has_value());
}

// 视场余量：目标视线角距视场边缘不足 fow_margin_deg 时不进入 S6。
TEST(MissionStateMachineTest, HandoverRejectsTargetNearFovEdge) {
    HandoverFixture f;
    ASSERT_TRUE(f.started);
    f.DriveToGpsApproach();

    // 像素 x=1240 → 视线方位角约 31.6°，标定内参下视场半角约 33.2°，
    // 余量不足 5° → 不放行
    for (uint64_t frame = 1; frame <= 4; ++frame) {
        f.PublishVisual(
            MakeVisualStatus(common::VisualTargetState::kLocked, frame, 1240.0F));
    }
    EXPECT_FALSE(WaitForState(f.status_subscription,
                              common::MissionState::kVisualHandover, 300ms)
                     .has_value());
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kGpsApproach);

    // 目标回到画面中部（余量充足）→ 进入 S6
    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, 5));
    EXPECT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kVisualHandover)
                    .has_value());
}

// S6 驻留超时：进入 S6 后视觉链未再推进新帧（首条视觉指令未生效）→ 回 S5。
TEST(MissionStateMachineTest, HandoverTimeoutReturnsToGpsApproach) {
    // 超时 300ms 先于视觉/测距超龄（5s）发生，确保走超时路径而非超龄撤销路径
    HandoverFixture f(MakeHandoverTestConfig(/*handover_timeout=*/300ms));
    ASSERT_TRUE(f.started);
    f.DriveToGpsApproach();

    for (uint64_t frame = 1; frame <= 3; ++frame) {
        f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, frame));
    }
    ASSERT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kVisualHandover)
                    .has_value());
    // 驻留期：进入条件仍有效（输入未超龄），仅因超时回 S5
    EXPECT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kGpsApproach, 1500ms)
                    .has_value());
    // 超时回退后不因旧锁定帧立刻回跳 S6
    EXPECT_FALSE(WaitForState(f.status_subscription,
                              common::MissionState::kVisualHandover, 400ms)
                     .has_value());
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kGpsApproach);
}

// S7 退出迟滞：LOST 连续帧超过进入门限但未达 exit_lost_frames 保持 S7；
// 达到 exit_lost_frames 才回退（进入/退出门限不同防止两状态抖动）。
TEST(MissionStateMachineTest, ExitRequiresMoreLostFramesThanEntry) {
    HandoverFixture f;
    ASSERT_TRUE(f.started);
    f.DriveToVisualTracking();

    // LOST 4 条（> lock_frames=3 但 < exit_lost_frames=6）→ 保持 S7
    for (uint64_t frame = 5; frame <= 8; ++frame) {
        f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLost, frame));
    }
    EXPECT_FALSE(WaitForState(f.status_subscription,
                              common::MissionState::kGpsApproach, 300ms)
                     .has_value());
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kVisualTracking);

    // 补足 6 条连续 LOST → 回退 S5
    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLost, 9));
    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLost, 10));
    EXPECT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kGpsApproach)
                    .has_value());
}

// S6 撤销：驻留期进入条件失效（视觉丢失）→ 立即撤销回 S5。
TEST(MissionStateMachineTest, HandoverAbortsWhenLockLostInHandover) {
    HandoverFixture f;
    ASSERT_TRUE(f.started);
    f.DriveToGpsApproach();

    for (uint64_t frame = 1; frame <= 3; ++frame) {
        f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, frame));
    }
    ASSERT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kVisualHandover)
                    .has_value());

    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLost, 4));
    EXPECT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kGpsApproach)
                    .has_value());
}

// 未注入估计器诊断接口时不放行交接：创新门限无法核验，保守保持 S5（并记 ERROR）。
TEST(MissionStateMachineTest, HandoverRequiresEstimatorDiagnostics) {
    HandoverFixture f(/*inject_diagnostics=*/false);
    ASSERT_TRUE(f.started);
    EXPECT_GT(f.machine->ErrorCount(), 0U);
    f.DriveToGpsApproach();

    for (uint64_t frame = 1; frame <= 5; ++frame) {
        f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, frame));
    }
    EXPECT_FALSE(WaitForState(f.status_subscription,
                              common::MissionState::kVisualHandover, 300ms)
                     .has_value());
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kGpsApproach);
}

// S7 测距失效退出：双目测距失效（invalid 或超龄）连续达 handover.distance_lost_ms
// → 退出 S7 回退 S5；期间一旦恢复有效则计时清零（不得误退）。
TEST(MissionStateMachineTest, ExitWhenDistanceLostBeyondThreshold) {
    HandoverFixture f(MakeHandoverTestConfig(/*handover_timeout=*/2000ms,
                                             /*distance_lost=*/300ms,
                                             /*visual_stale=*/5000ms,
                                             /*distance_stale=*/2000ms));
    ASSERT_TRUE(f.started);
    f.DriveToVisualTracking();

    // 距离失效（invalid）持续约 250ms（未达 300ms 阈值）后恢复有效：
    // 计时必须清零——若未清零，恢复后再等 400ms（累计 > 阈值）就会退出。
    f.PublishDistance(false);
    std::this_thread::sleep_for(120ms);
    f.PublishDistance(true);
    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kVisualTracking);

    // 距离持续失效（仍按期发布，排除超龄路径，纯 invalid 路径）→ 达阈值退出
    const auto deadline = std::chrono::steady_clock::now() + 2000ms;
    while (std::chrono::steady_clock::now() < deadline &&
           f.machine->CurrentState() == common::MissionState::kVisualTracking) {
        f.PublishDistance(false);
    }
    EXPECT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kGpsApproach, 1000ms)
                    .has_value());
    // 测距失效推出后停在 S5（锁定计数已复位，不因旧帧回跳 S6）
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kGpsApproach);
}

// S7 视觉断流退出：监视器线程死亡/链路中断时无 LOST 状态消息可计数、帧序冻结，
// 状态消息接收年龄超过 S6 进入判据所用的同一视觉新鲜度阈值即按断流退出。
TEST(MissionStateMachineTest, ExitWhenVisualStreamStalls) {
    HandoverFixture f(MakeHandoverTestConfig(/*handover_timeout=*/2000ms,
                                             /*distance_lost=*/3000ms,
                                             /*visual_stale=*/400ms,
                                             /*distance_stale=*/5000ms));
    ASSERT_TRUE(f.started);
    f.DriveToVisualTracking();

    // 不再发布任何视觉状态（等价监视器线程死亡）：未到超龄阈值时不退出
    EXPECT_FALSE(WaitForState(f.status_subscription,
                              common::MissionState::kGpsApproach, 150ms)
                     .has_value());
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kVisualTracking);
    // 超龄（visual_stale=400ms）→ 按断流退出回 S5
    EXPECT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kGpsApproach, 1500ms)
                    .has_value());
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kGpsApproach);
}

// ---------------------------------------------------------------------------
// 阶段四 Task 9：S7 视觉伺服意图（TrackingControlLaw 迁入状态机）+ coast +
// 热备份核对
//
// S7（VISUAL_TRACKING）期间状态机是唯一的视觉意图生产者：以 VisualTargetStatus
// （LOCKED→tracked）、StereoTargetDistance 与 FlightStateSnapshot 姿态组装
// ControlSnapshot，驱动 TrackingControlLaw（含机体系→NED 旋转），把输出映射为
// control_source=1 的 ControlIntent（带 bit31 控制权请求位）。
// 主链路无预测坐标（is_predicted 恒 false）：短期 LOST 按控制律判丢语义映射为
// 零速刹车意图（kBrakeHover）；控制律降级保持（姿态/视觉超龄）时不发布意图
// （安全语义：超龄收口交 FlightController 意图超时门禁刹停）。
// S6 驻留期由状态机续发进入时刻冻结的最后位置目标（valid_until 逐拍滚动，
// 设计 §2.1/§2.5 无扰衔接：避免交接点意图超龄刹车再起步），首条视觉速度
// 意图发出即停。
// ---------------------------------------------------------------------------

constexpr double kPi = 3.14159265358979323846;

/// 默认跟踪配置（visual_tracking 节默认值）下的伺服期望输出基准：
///   航向 β = atan((px − cx)/fx)，速率式 yaw_rate = gain·β（gain 默认 1.0）
///   垂直 α = atan((py − cy)/fy)，vz = vz_gain·α（vz_gain 默认 0.1）
///   距离 Δd = 15 − d_exp，vx = kp·Δd（kp 用例注入 0.1，占位默认 0 无法验证映射）
constexpr double kDefaultFx = 1007.82;
constexpr double kDefaultFy = 1008.02;
constexpr double kDefaultCx = 620.73;
constexpr double kDefaultCy = 532.07;
constexpr double kDefaultVzGain = 0.1;
constexpr double kServoDistanceKp = 0.1;
constexpr double kDefaultDExpM = 10.0;
constexpr double kServoDistanceM = 15.0;
constexpr double kServoVisualX = 640.0;
constexpr double kServoVisualY = 532.0;

/// 期望航向角速度（度/秒）：β 由标定内参算出，约 1.0955°/s。
double ServoExpectedBetaDeg() {
    return std::atan((kServoVisualX - kDefaultCx) / kDefaultFx) * 180.0 / kPi;
}

/// 期望垂直速度（米/秒）：α≈−0.004°（像素 y=532 略高于主点 cy=532.07）。
double ServoExpectedVz() {
    const double alpha_deg =
        std::atan((kServoVisualY - kDefaultCy) / kDefaultFy) * 180.0 / kPi;
    return kDefaultVzGain * alpha_deg;
}

/// 期望前向速度（米/秒）：Δd=15−10=5m、kp=0.1 → 0.5m/s（加速度限幅收敛后）。
double ServoExpectedVx() {
    return kServoDistanceKp * (kServoDistanceM - kDefaultDExpM);
}

/// 导航就绪 + 新鲜姿态的飞行快照（S7 伺服要求姿态在控制律超龄阈值内，
/// 用例需按期重发刷新接收时间）。
common::FlightStateSnapshot MakeFlightWithAttitude(double yaw_rad) {
    common::FlightStateSnapshot flight = MakeNavigationReadyFlight();
    flight.header.receive_time_ms = TestMonotonicMs();
    flight.attitude_valid = true;
    flight.roll_rad = 0.0F;
    flight.pitch_rad = 0.0F;
    flight.yaw_rad = static_cast<float>(yaw_rad);
    return flight;
}

/// 收集一段时间内的全部意图（按发布顺序）。
std::vector<common::ControlIntent> CollectIntents(
    common::Topic<common::ControlIntent>::Subscription& subscription,
    std::chrono::milliseconds duration) {
    std::vector<common::ControlIntent> intents;
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = subscription.WaitTakeFor(50ms);
        if (message) {
            intents.push_back(**message);
        }
    }
    return intents;
}

/// 返回序列中最后一条指定类型的意图；不存在返回 nullptr。
const common::ControlIntent* LastIntentOfType(
    const std::vector<common::ControlIntent>& intents,
    common::ControlIntentType type) {
    const common::ControlIntent* found = nullptr;
    for (const auto& intent : intents) {
        if (intent.type == type) {
            found = &intent;
        }
    }
    return found;
}

/// 等待一条带指定告警位的任务状态（超时返回 nullopt）。
std::optional<common::MissionStatus> WaitForWarningBit(
    common::Topic<common::MissionStatus>::Subscription& subscription,
    uint32_t mask,
    std::chrono::milliseconds timeout = 3000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = subscription.WaitTakeFor(50ms);
        if (message && ((*message)->active_warning_bits & mask) != 0U) {
            return **message;
        }
    }
    return std::nullopt;
}

/// 视觉伺服用例配置：交接阈值同 HandoverFixture（取小值压缩用例时长）；
/// 距离环比例增益取非零——占位默认 kp=0 会让前向速度恒为零，无法验证
/// "距离通道输出 → 意图映射"。
state_machine::MissionStateMachineConfig MakeServoTestConfig() {
    auto config = MakeHandoverTestConfig();
    config.tracking.distance.kp = kServoDistanceKp;
    return config;
}

/// 视觉伺服测试夹具：交接驱动复用 HandoverFixture，附加 S7 输入泵与意图订阅。
/// 控制律内部超龄阈值（视觉 200ms/姿态 300ms/测距 500ms，跟踪配置默认值）
/// 要求用例按期刷新三路输入，否则控制律降级保持、停止发布意图。
struct VisualServoFixture : HandoverFixture {
    explicit VisualServoFixture(
        double yaw_rad = 0.0,
        const state_machine::MissionStateMachineConfig& custom = MakeServoTestConfig())
        : HandoverFixture(custom), yaw_rad_(yaw_rad) {
        intent_subscription = machine->IntentOutput().Subscribe(64);
    }

    /// 发布一轮伺服输入：新鲜姿态 + 指定视觉状态 + 测距 15m（射程 10m 之外）。
    /// with_attitude=false 时不发布飞行快照（最新快照保持无姿态）——用于构造
    /// "控制律姿态超龄持续降级、伺服永不启动"的场景。
    void PublishServoInputs(
        uint64_t frame,
        common::VisualTargetState visual_state = common::VisualTargetState::kLocked,
        bool with_attitude = true) {
        if (with_attitude) {
            (void)flight.Publish(std::make_shared<const common::FlightStateSnapshot>(
                MakeFlightWithAttitude(yaw_rad_)));
        }
        (void)visual.Publish(std::make_shared<const common::VisualTargetStatus>(
            MakeVisualStatus(visual_state, frame)));
        (void)distance.Publish(std::make_shared<const common::StereoTargetDistance>(
            MakeFreshDistance(frame, /*valid=*/true, /*forward_m=*/kServoDistanceM)));
    }

    /// 按期泵输入并收集期间的意图（每轮约 120ms）。
    std::vector<common::ControlIntent> PumpAndCollect(int rounds) {
        std::vector<common::ControlIntent> intents;
        for (int round = 0; round < rounds; ++round) {
            PublishServoInputs(++frame_);
            auto batch = CollectIntents(intent_subscription, 120ms);
            intents.insert(intents.end(), batch.begin(), batch.end());
        }
        return intents;
    }

    /// 清空意图队列（跳过交接驱动期已发出的意图）。
    void DrainIntents() {
        while (intent_subscription.TryTake().has_value()) {
        }
    }

    double yaw_rad_ = 0.0;
    uint64_t frame_ = 100;  ///< 帧序号：避开交接驱动帧 1..4 的迟滞去重记忆
    common::Topic<common::ControlIntent>::Subscription intent_subscription;
};

// S7：视觉 LOCKED + 测距有效 + 姿态新鲜 → 发布 control_source=1 的速度航向意图，
// 速度分量与 TrackingControlLaw 输出一致（注入已知几何，对拍控制律单测期望值）。
TEST(MissionStateMachineTest, VisualTrackingPublishesVelocityIntent) {
    VisualServoFixture f;
    ASSERT_TRUE(f.started);
    f.DriveToVisualTracking();
    f.DrainIntents();

    const auto intents = f.PumpAndCollect(/*rounds=*/10);
    const common::ControlIntent* last = LastIntentOfType(
        intents, common::ControlIntentType::kVelocityHeading);
    ASSERT_NE(last, nullptr);

    EXPECT_EQ(last->control_source, 1U);  // 1=视觉
    EXPECT_NE(last->reason_code & common::kControlIntentReasonRequestControl, 0U);
    EXPECT_EQ(last->reason_code & 0xFFU, 100U);  // 视觉跟踪：正常跟踪
    EXPECT_EQ(last->header.frame_id, kFrameLocalNed);
    EXPECT_EQ(last->header.valid_for_ms, 300U);
    EXPECT_EQ(last->valid_until_ms, last->header.receive_time_ms + 300U);

    // 距离通道：Δd = 15−10 = 5m、kp=0.1 → 前向 0.5m/s（加速度限幅收敛后）
    EXPECT_NEAR(last->target_x, ServoExpectedVx(), 1e-3);
    // 姿态 yaw=0：机体系侧向恒 0，NED 东向 0
    EXPECT_NEAR(last->target_y, 0.0, 1e-6);
    // 垂直通道：α 由标定内参算出
    EXPECT_NEAR(last->target_z, ServoExpectedVz(), 1e-6);
    // 航向通道（速率式）：增益×视线角
    EXPECT_NEAR(last->yaw_rate_dps, ServoExpectedBetaDeg(), 1e-3);
    EXPECT_FLOAT_EQ(last->yaw_deg, 0.0F);  // 速率式不使用 yaw_deg
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kVisualTracking);
}

// 意图速度已是局部 NED：姿态 yaw=90°（机头朝东）时，机体前向速度映射为 NED
// 东向速度、北向≈0 —— 状态机内完成机体系→NED 旋转，FlightController 不做
// 二次旋转（控制边界见 flight_controller.md）。
TEST(MissionStateMachineTest, VisualTrackingIntentIsLocalNed) {
    VisualServoFixture f(kPi / 2.0);
    ASSERT_TRUE(f.started);
    f.DriveToVisualTracking();
    f.DrainIntents();

    const auto intents = f.PumpAndCollect(/*rounds=*/10);
    const common::ControlIntent* last = LastIntentOfType(
        intents, common::ControlIntentType::kVelocityHeading);
    ASSERT_NE(last, nullptr);

    EXPECT_NEAR(last->target_x, 0.0, 1e-6);                // 北向≈0
    EXPECT_NEAR(last->target_y, ServoExpectedVx(), 1e-3);  // 东向=机体前向
    EXPECT_NEAR(last->target_z, ServoExpectedVz(), 1e-6);
    // 速率式航向通道与自机航向无关：仍为增益×视线角
    EXPECT_NEAR(last->yaw_rate_dps, ServoExpectedBetaDeg(), 1e-3);
}

// S7 视觉短期 LOST：主链路无预测坐标（is_predicted 恒 false），控制律按判丢
// 语义输出零速刹车（valid=true）→ 意图映射为零速 kBrakeHover 安全意图
//（图像空间 coast 在本链路的退化形态，见实现文档）；LOST 连续帧达
// exit_lost_frames 才退出 S7 回退 S5，回退后位置引导链接管。
TEST(MissionStateMachineTest, VisualLostBrakesThenExitsToGpsApproach) {
    VisualServoFixture f;
    ASSERT_TRUE(f.started);
    f.DriveToVisualTracking();
    f.DrainIntents();

    // 先确认伺服已启动（S7 期收到过速度意图）
    const auto servo = f.PumpAndCollect(/*rounds=*/4);
    ASSERT_NE(LastIntentOfType(servo, common::ControlIntentType::kVelocityHeading),
              nullptr);

    // LOST 4 条（> lock_frames=3、< exit_lost_frames=6）：保持 S7，意图转零速刹车
    std::vector<common::ControlIntent> lost_intents;
    for (uint64_t frame = 200; frame <= 203; ++frame) {
        f.PublishServoInputs(frame, common::VisualTargetState::kLost);
        auto batch = CollectIntents(f.intent_subscription, 120ms);
        lost_intents.insert(lost_intents.end(), batch.begin(), batch.end());
    }
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kVisualTracking);
    const common::ControlIntent* brake =
        LastIntentOfType(lost_intents, common::ControlIntentType::kBrakeHover);
    ASSERT_NE(brake, nullptr);
    EXPECT_EQ(brake->control_source, 1U);
    EXPECT_NE(brake->reason_code & common::kControlIntentReasonRequestControl, 0U);
    EXPECT_EQ(brake->reason_code & 0xFFU, 102U);  // 判丢零速刹车
    EXPECT_FLOAT_EQ(brake->target_x, 0.0F);
    EXPECT_FLOAT_EQ(brake->target_y, 0.0F);
    EXPECT_FLOAT_EQ(brake->target_z, 0.0F);

    // 补足 6 条连续 LOST → 退出 S7 回退 S5
    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLost, 204));
    f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLost, 205));
    ASSERT_TRUE(
        WaitForState(f.status_subscription, common::MissionState::kGpsApproach)
            .has_value());
    // 回退后位置引导链接管（估计仍新鲜）
    const auto after_exit = CollectIntents(f.intent_subscription, 400ms);
    const common::ControlIntent* position =
        LastIntentOfType(after_exit, common::ControlIntentType::kPositionGuide);
    ASSERT_NE(position, nullptr);
    EXPECT_EQ(position->control_source, 0U);
}

// S7 伺服永不启动（控制律持续降级，如姿态超龄）时，冻结位置续发不得无限
// 续期：S6 有 handover.timeout_ms 封顶，S7 需等价封顶——驻留超时仍未见
// 首条有效视觉意图 → 收口回退 S5（复用既有退出机制，中文 reason 区分路径）。
TEST(MissionStateMachineTest, S7ServoNotStartedFallsBackAfterTimeout) {
    drone::test::LogCapture logs;  // 必须在组件启动前挂载（见 log_capture.h 约束）
    VisualServoFixture f;
    ASSERT_TRUE(f.started);
    f.DriveToVisualTracking();  // 飞行快照无姿态 → 控制律姿态超龄降级
    f.DrainIntents();
    logs.Clear();

    // S7 驻留：按期刷新视觉/测距（不提供姿态），冻结位置续发持续滚动续期
    std::vector<common::ControlIntent> intents;
    uint64_t frame = 300;
    for (int round = 0; round < 14; ++round) {
        f.PublishServoInputs(++frame, common::VisualTargetState::kLocked,
                             /*with_attitude=*/false);
        auto batch = CollectIntents(f.intent_subscription, 200ms);
        intents.insert(intents.end(), batch.begin(), batch.end());
        if (f.machine->CurrentState() != common::MissionState::kVisualTracking) {
            break;
        }
    }
    // 回退前伺服从未启动：无速度/刹车意图；冻结位置续发在生效
    EXPECT_EQ(LastIntentOfType(intents, common::ControlIntentType::kVelocityHeading),
              nullptr);
    EXPECT_EQ(LastIntentOfType(intents, common::ControlIntentType::kBrakeHover),
              nullptr);
    EXPECT_NE(LastIntentOfType(intents, common::ControlIntentType::kPositionGuide),
              nullptr);
    // 超时收口回 S5（reason 中文可查，区别于 S6 超时与其它三条 S7 退出路径）
    ASSERT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kGpsApproach)
                    .has_value());
    EXPECT_EQ(logs.Count("S7伺服未启动超时"), 1);
}

// S7 报告状态与实际控制输出背离的可观测位（T9 评审裁决 I2，Task 12）：
// S7 期间本拍未发出视觉伺服意图——伺服尚未启动（仍在续发冻结位置意图）
// 或控制律降级停发（姿态/视觉超龄）——MissionStatus 告警位 bit3 置位；
// 伺服意图正常发出即清除（同一 S7 驻留内）。
TEST(MissionStateMachineTest, ServoSuspendedBitTracksVisualServoOutput) {
    auto config = MakeServoTestConfig();
    config.handover.timeout = 8000ms;  // 放宽 S7 伺服未启动超时，专测告警位
    VisualServoFixture f(/*yaw_rad=*/0.0, config);
    ASSERT_TRUE(f.started);
    f.DriveToVisualTracking();  // 飞行快照无姿态 → 控制律姿态超龄降级、伺服未启动
    constexpr uint32_t kWarningServoSuspended = 1U << 3U;  // 见 mission_state_machine.md

    // 伺服未启动：S7 无视觉意图输出（冻结位置意图在续发）→ 背离位置位
    const auto suspended =
        WaitForWarningBit(f.status_subscription, kWarningServoSuspended, 3000ms);
    ASSERT_TRUE(suspended.has_value());
    EXPECT_EQ(suspended->state, common::MissionState::kVisualTracking);

    // 姿态恢复新鲜 + 持续伺服输入 → 首条视觉意图发出 → 位清除（仍在 S7）
    bool cleared = false;
    uint64_t frame = 400;  // 避开交接驱动帧 1..4 与既有用例帧段
    const auto deadline = std::chrono::steady_clock::now() + 3000ms;
    while (std::chrono::steady_clock::now() < deadline) {
        f.PublishServoInputs(++frame);
        auto message = f.status_subscription.WaitTakeFor(100ms);
        if (message && (*message)->state == common::MissionState::kVisualTracking &&
            ((*message)->active_warning_bits & kWarningServoSuspended) == 0U) {
            cleared = true;
            break;
        }
    }
    EXPECT_TRUE(cleared);
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kVisualTracking);
}

// S7 热备份核对（设计 §2.4）：每秒核对卡尔曼状态年龄（TargetState 接收时刻 +
// 自报有效期窗），断粮 → 节流 WARN「热备份断粮」+ MissionStatus 告警位 bit2，
// 不退出 S7（停 S7 由四条收口路径决定，热备份告警只提示"假备份"风险）。
TEST(MissionStateMachineTest, HotBackupKalmanStarvedWarnsInS7) {
    drone::test::LogCapture logs;  // 必须在组件启动前挂载（见 log_capture.h 约束）
    VisualServoFixture f;
    ASSERT_TRUE(f.started);
    f.DriveToVisualTracking();
    logs.Clear();

    // 卡尔曼断粮：估计失效快照（valid=false、有效期为 0）
    (void)f.estimated.Publish(
        std::make_shared<const common::TargetState>(MakeInvalidEstimatedTarget()));

    // 核对周期 1s：等待告警位置位（MissionStatus 500ms 周期回传）
    constexpr uint32_t kWarningHotBackupStarved = 1U << 2U;  // 见 mission_state_machine.md
    const auto starved =
        WaitForWarningBit(f.status_subscription, kWarningHotBackupStarved, 3000ms);
    ASSERT_TRUE(starved.has_value());
    EXPECT_EQ(f.machine->CurrentState(), common::MissionState::kVisualTracking);
    EXPECT_GE(logs.Count("热备份断粮"), 1);
}

// 设计 §2.1「进入 S6 后冻结最后位置目标」+ §2.5 无扰衔接（评审裁决 (a)）：
// S6 驻留期由状态机逐拍续发进入时刻冻结的位置意图（valid_until 滚动续期），
// 避免意图超龄 → 控制器 COAST 刹车 → 再起步的"交接点先刹停"；冻结坐标取
// 进入时刻的 TargetState（此后估计更新不影响冻结值）；首条视觉速度意图
// 发出（S7）后冻结续发停止。
TEST(MissionStateMachineTest, HandoverPublishesFrozenPositionUntilServoStarts) {
    VisualServoFixture f;
    ASSERT_TRUE(f.started);
    f.DriveToGpsApproach();
    for (uint64_t frame = 1; frame <= 3; ++frame) {
        f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, frame));
    }
    ASSERT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kVisualHandover)
                    .has_value());
    f.DrainIntents();

    // S6 驻留期估计更新为新位置（仍新鲜有效）——冻结续发必须仍指向进入时刻的值
    common::TargetState updated = MakeHandoverEstimate();
    updated.header.sequence += 1;
    updated.pos_x_m = 99.0F;
    updated.pos_y_m = 98.0F;
    updated.pos_z_m = -7.0F;
    (void)f.estimated.Publish(std::make_shared<const common::TargetState>(updated));

    std::vector<common::ControlIntent> frozen;
    for (int round = 0; round < 3; ++round) {
        // 重复同一锁定帧：不推进锁定计数（停留 S6），只刷新状态新鲜度
        f.PublishVisual(MakeVisualStatus(common::VisualTargetState::kLocked, 3));
        auto batch = CollectIntents(f.intent_subscription, 120ms);
        frozen.insert(frozen.end(), batch.begin(), batch.end());
    }
    ASSERT_EQ(f.machine->CurrentState(), common::MissionState::kVisualHandover);
    ASSERT_GE(frozen.size(), 2U);
    for (const auto& intent : frozen) {
        EXPECT_EQ(intent.type, common::ControlIntentType::kPositionGuide);
        EXPECT_EQ(intent.control_source, 0U);
        EXPECT_FLOAT_EQ(intent.target_x, 12.5F);  // 冻结值（非更新后的 99）
        EXPECT_FLOAT_EQ(intent.target_y, -3.0F);
        EXPECT_FLOAT_EQ(intent.target_z, -8.0F);
        // 滚动续期：valid_until 按 300ms 窗逐拍推进
        EXPECT_EQ(intent.valid_until_ms, intent.header.receive_time_ms + 300U);
    }
    // 续期确实发生：至少两条意图的截止时刻不同（逐拍滚动，非一次性发布）
    bool renewed = false;
    for (size_t index = 1; index < frozen.size(); ++index) {
        if (frozen[index].valid_until_ms != frozen.front().valid_until_ms) {
            renewed = true;
        }
    }
    EXPECT_TRUE(renewed);

    // 视觉链推进一条新 LOCKED 帧（附姿态/测距输入）→ S7 首条视觉速度意图发出
    f.PublishServoInputs(4);
    ASSERT_TRUE(WaitForState(f.status_subscription,
                             common::MissionState::kVisualTracking)
                    .has_value());
    const auto servo = f.PumpAndCollect(/*rounds=*/4);
    const common::ControlIntent* first_velocity = nullptr;
    for (const auto& intent : servo) {
        if (intent.type == common::ControlIntentType::kVelocityHeading) {
            first_velocity = &intent;
            break;
        }
    }
    ASSERT_NE(first_velocity, nullptr);
    // 冻结续发停止：位置意图不得出现在首条视觉速度意图之后
    //（迁移当拍允许同拍续发一条，控制器保留最新=视觉意图）
    for (const auto& intent : servo) {
        if (intent.type == common::ControlIntentType::kPositionGuide) {
            EXPECT_LE(intent.header.receive_time_ms,
                      first_velocity->header.receive_time_ms);
        }
    }
}

// ---------------------------------------------------------------------------
// 阶段四 Task 10：捕网执行器命令（MAV_CMD_DO_SET_ACTUATOR=187）
//
// 状态机在进入 S10 INTERCEPT_EXECUTING 时经命令窄接口 IPx4CommandSender 发
// 一次触发命令（net_fired_ 防重复；入队失败不置位、下拍重试）；离开拦截序列
// （S9/S10 → 回退/返航/POST_INTERCEPT）时发复位行程并清除触发标志。
// 通道号/行程来自 actuator 配置节；PWM 微秒按 PX4 输出映射区间换算为
// MAV_CMD_DO_SET_ACTUATOR 的归一化参数（-1..1），未用参数位 NaN（规范语义
// 为"忽略"，0 会误动其他执行器组）；p7=执行器组序号（PX4 仅处理 0）。
// 未注入命令接口（enable_control=false 装配语义）→ 零调用（物理缺席）。
//
// 拦截序列（S9/S10/S11）当前尚无生产入口——入口判据（授权策略/射程门限）
// 属后续任务，本任务不发明；用例经 ForceStateForTest 测试入口驱动状态迁移，
// 与生产路径共用同一 TransitionTo/决策线程挂点，触发/复位语义等同生产。
// ---------------------------------------------------------------------------

/// 触发行程 1100µs、复位行程 1900µs、输出映射区间 1000..2000µs 的归一化期望值：
/// norm = 2·(pwm − 1000)/1000 − 1。
constexpr float kNetFireNormalized = -0.8F;
constexpr float kNetIdleNormalized = 0.8F;

/// 命令窄接口假件：记录 SendCommand 调用（顺序/参数/入队结果）并可脚本化
/// 前 N 次入队失败。
class FakeCommandSender final : public communication::IPx4CommandSender {
public:
    /// 单条命令调用记录：供触发/复位次序与各次入队结果断言。
    struct CallRecord {
        uint16_t command = 0;
        std::array<float, 7> params{};
        bool queued = false;  // SendCommand 返回值（false=链路未就绪/队列满）
    };

    bool SendCommand(uint16_t mavlink_command, float param1, float param2,
                     float param3, float param4, float param5, float param6,
                     float param7) override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++call_count_;
        last_command_ = mavlink_command;
        last_params_ = {param1, param2, param3, param4, param5, param6, param7};
        bool queued = true;
        if (fail_calls_remaining_ > 0) {
            --fail_calls_remaining_;
            queued = false;
        }
        history_.push_back({mavlink_command, last_params_, queued});
        return queued;
    }

    uint64_t AckMatchCount() const override { return 0; }
    uint64_t AckTimeoutCount() const override { return 0; }

    uint64_t CallCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return call_count_;
    }

    uint16_t LastCommand() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_command_;
    }

    std::array<float, 7> LastParams() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_params_;
    }

    /// 全部调用记录（按时间顺序的拷贝）。
    std::vector<CallRecord> History() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return history_;
    }

    /// 令接下来 count 次 SendCommand 返回失败（模拟 PX4 未连接/队列满）。
    void FailNextCalls(uint64_t count) {
        std::lock_guard<std::mutex> lock(mutex_);
        fail_calls_remaining_ = count;
    }

private:
    mutable std::mutex mutex_;
    uint64_t call_count_ = 0;
    uint16_t last_command_ = 0;
    std::array<float, 7> last_params_{};
    std::vector<CallRecord> history_;
    uint64_t fail_calls_remaining_ = 0;
};

/// 捕网执行器用例配置：通道 3（验证通道号落在 p3 而非 p1），行程取默认值；
/// 显式开启测试钩子（拦截序列无生产入口，用例经 `ForceStateForTest` 驱动）。
state_machine::MissionStateMachineConfig MakeNetActuatorTestConfig() {
    state_machine::MissionStateMachineConfig config;
    config.actuator.net_channel = 3;
    config.test_hooks_enabled = true;
    return config;
}

/// 等待发送器累计调用数达到期望值（决策周期 100ms，留足余量）。
bool WaitForCallCount(const FakeCommandSender& sender, uint64_t expected,
                      std::chrono::milliseconds timeout = 1500ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (sender.CallCount() >= expected) {
            return true;
        }
        std::this_thread::sleep_for(20ms);
    }
    return sender.CallCount() >= expected;
}

/// 等待状态机完成启动首个迁移（BOOT→SELF_CHECK）再强制状态；否则工作线程
/// 的初始迁移会把测试入口强制写入的状态覆盖掉。
bool WaitForSelfCheck(const state_machine::MissionStateMachine& machine,
                      std::chrono::milliseconds timeout = 1000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (machine.CurrentState() == common::MissionState::kSelfCheck) {
            return true;
        }
        std::this_thread::sleep_for(10ms);
    }
    return machine.CurrentState() == common::MissionState::kSelfCheck;
}

// 进入 S10 → 恰好一次触发命令；停留不重复；离开拦截序列 → 复位行程；
// 再次进入 → 新一次触发（触发标志随离开清除，防重复仅限同一拦截段）。
TEST(MissionStateMachineTest, InterceptExecutingFiresNetActuator) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeNetActuatorTestConfig());
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForCallCount(sender, 1)) << "进入 S10 应发出触发命令";
    EXPECT_EQ(sender.LastCommand(), 187U);  // MAV_CMD_DO_SET_ACTUATOR
    {
        const auto params = sender.LastParams();
        EXPECT_FLOAT_EQ(params[2], kNetFireNormalized);  // p3=通道 3（触发行程）
        EXPECT_TRUE(std::isnan(params[0]));              // p1 未用 → NaN（忽略）
        EXPECT_TRUE(std::isnan(params[1]));
        EXPECT_TRUE(std::isnan(params[3]));
        EXPECT_TRUE(std::isnan(params[4]));
        EXPECT_TRUE(std::isnan(params[5]));
        EXPECT_FLOAT_EQ(params[6], 0.0F);                // p7=执行器组序号 0
    }

    // 停留 S10 多拍：不重复触发
    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(sender.CallCount(), 1U);
    EXPECT_EQ(machine.CurrentState(), common::MissionState::kInterceptExecuting);

    // 离开拦截序列（S10→S11 POST_INTERCEPT）：发复位行程（待命行程），只发一次
    machine.ForceStateForTest(common::MissionState::kPostIntercept);
    ASSERT_TRUE(WaitForCallCount(sender, 2)) << "离开拦截序列应发复位行程";
    EXPECT_EQ(sender.LastCommand(), 187U);
    EXPECT_FLOAT_EQ(sender.LastParams()[2], kNetIdleNormalized);
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(sender.CallCount(), 2U);

    // 再次进入 S10：新一次拦截触发（net_fired_ 已随离开清除）
    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForCallCount(sender, 3));
    EXPECT_FLOAT_EQ(sender.LastParams()[2], kNetFireNormalized);

    machine.Stop();
}

// 入队失败不置位、下拍重试：PX4 未连接/命令队列满（SendCommand 返回 false）
// 时触发命令不得丢失，也不得因"已尝试"而停产。
TEST(MissionStateMachineTest, NetActuatorFiringRetriesWhenEnqueueFails) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeNetActuatorTestConfig());
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    sender.FailNextCalls(2);  // 前两次入队失败
    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForCallCount(sender, 3)) << "入队失败应下拍重试直至成功";
    {
        const auto params = sender.LastParams();
        EXPECT_FLOAT_EQ(params[2], kNetFireNormalized);  // 重试用同一条命令
        EXPECT_FLOAT_EQ(params[6], 0.0F);
    }
    // 成功入队后不再补发
    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(sender.CallCount(), 3U);

    machine.Stop();
}

// 复位入队失败同样下拍重试：复位是安全动作（机构归位到待命行程），链路瞬时
// 不可用（PX4 未连接/命令队列满）不得让其静默丢失——重试至成功，且重试期间
// 只反复位行程、不得混入触发命令。
TEST(MissionStateMachineTest, NetActuatorResetRetriesWhenEnqueueFails) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeNetActuatorTestConfig());
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForCallCount(sender, 1)) << "进入 S10 应发出触发命令";

    sender.FailNextCalls(2);  // 复位前两次入队失败
    machine.ForceStateForTest(common::MissionState::kPostIntercept);
    ASSERT_TRUE(WaitForCallCount(sender, 4)) << "复位入队失败应下拍重试直至成功";

    // 成功入队后不再补发（含不补发触发）
    std::this_thread::sleep_for(400ms);
    ASSERT_EQ(sender.CallCount(), 4U);
    const auto history = sender.History();
    ASSERT_EQ(history.size(), 4U);
    // 次序：1=触发成功；2/3=复位失败（重试内容一致）；4=复位成功
    EXPECT_EQ(history[0].command, 187U);
    EXPECT_FLOAT_EQ(history[0].params[2], kNetFireNormalized);
    EXPECT_TRUE(history[0].queued);
    for (std::size_t index = 1; index < history.size(); ++index) {
        EXPECT_EQ(history[index].command, 187U);
        EXPECT_FLOAT_EQ(history[index].params[2], kNetIdleNormalized)
            << "复位重试期间不得混入触发行程 index=" << index;
        EXPECT_EQ(history[index].queued, index == 3U)
            << "入队结果序列不符（前两次复位应失败、第三次成功）index=" << index;
    }

    machine.Stop();
}

// 复位未入队前不发触发：离开拦截序列后复位尚在重试（未成功入队）期间重进
// S10，触发命令必须等复位成功入队后才发——"先复位、后触发"的次序在异常重试
// 下仍保持，机构不会带着上一次拦截的残留行程直接触发。
TEST(MissionStateMachineTest, NetActuatorTriggerWaitsForResetEnqueued) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeNetActuatorTestConfig());
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForCallCount(sender, 1)) << "进入 S10 应发出触发命令";

    sender.FailNextCalls(3);  // 复位连续三次入队失败
    machine.ForceStateForTest(common::MissionState::kPostIntercept);
    // 等复位首拍落地，确保决策线程已消费"离开拦截序列"（复位挂起中）
    ASSERT_TRUE(WaitForCallCount(sender, 2)) << "离开拦截序列应发复位行程";

    // 复位仍挂起（未成功入队）时重进 S10：不得抢发触发
    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForCallCount(sender, 6)) << "复位重试成功后应恢复触发";
    std::this_thread::sleep_for(400ms);
    ASSERT_EQ(sender.CallCount(), 6U);

    const auto history = sender.History();
    ASSERT_EQ(history.size(), 6U);
    // 期望次序：1=触发成功；2~4=复位重试全失败；5=复位成功；6=新触发
    EXPECT_FLOAT_EQ(history[0].params[2], kNetFireNormalized);
    EXPECT_TRUE(history[0].queued);
    for (std::size_t index = 1; index <= 3; ++index) {
        EXPECT_FLOAT_EQ(history[index].params[2], kNetIdleNormalized)
            << "复位未入队前不得发触发 index=" << index;
        EXPECT_FALSE(history[index].queued);
    }
    EXPECT_FLOAT_EQ(history[4].params[2], kNetIdleNormalized);
    EXPECT_TRUE(history[4].queued) << "第 5 条应为复位成功入队";
    // 触发排在复位成功之后
    EXPECT_FLOAT_EQ(history[5].params[2], kNetFireNormalized);
    EXPECT_TRUE(history[5].queued);

    machine.Stop();
}

// 离开拦截序列发复位行程不依赖是否触发过：S9 进入后未进 S10（如授权拒绝
// 返航/条件失效回退）即离开，同样发复位行程让机构归位到待命行程。
TEST(MissionStateMachineTest, LeavingInterceptSequenceSendsResetStroke) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeNetActuatorTestConfig());
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    // S9（INTERCEPT_READY）不触发：只在 S10 执行态触发
    machine.ForceStateForTest(common::MissionState::kInterceptReady);
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(sender.CallCount(), 0U);

    // 返航离开拦截序列 → 复位行程
    machine.ForceStateForTest(common::MissionState::kReturnHome);
    ASSERT_TRUE(WaitForCallCount(sender, 1));
    EXPECT_FLOAT_EQ(sender.LastParams()[2], kNetIdleNormalized);
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(sender.CallCount(), 1U);

    machine.Stop();
}

// 未注入命令接口（enable_control=false 装配语义）：即使进入拦截态也零调用，
// 本功能物理缺席（命令发不出去）。
TEST(MissionStateMachineTest, NoCommandSenderNeverFires) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;  // 不注入
    state_machine::MissionStateMachine machine(MakeNetActuatorTestConfig());
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    machine.ForceStateForTest(common::MissionState::kInterceptReady);
    std::this_thread::sleep_for(150ms);
    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    std::this_thread::sleep_for(300ms);
    machine.ForceStateForTest(common::MissionState::kPostIntercept);
    std::this_thread::sleep_for(300ms);

    EXPECT_EQ(sender.CallCount(), 0U);

    machine.Stop();
}

// 测试入口门控（2026-10-14 终审）：`ForceStateForTest` 在
// `MissionStateMachineConfig::test_hooks_enabled=false`（默认＝生产装配语义，
// DroneApplication 不置位）时必须拒绝并记 ERROR——强推到 S10 会在
// enable_control=true 装配下经命令窄接口发出**不可逆的捕网触发命令**，
// 生产路径不得可达；而拦截序列当前无生产入口，该入口是唯一的误触路径。
TEST(MissionStateMachineTest, ForceStateForTestRejectedWithoutTestHooks) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;  // 已注入命令接口：验证"拒绝发生在任何命令之前"
    auto config = MakeNetActuatorTestConfig();
    config.test_hooks_enabled = false;  // 生产装配默认值
    state_machine::MissionStateMachine machine(config);
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    const uint64_t errors_before = machine.ErrorCount();
    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    std::this_thread::sleep_for(300ms);  // 留出决策线程消费（若放行会触发捕网）

    EXPECT_GT(machine.ErrorCount(), errors_before);  // 拒绝并记 ERROR
    EXPECT_NE(machine.CurrentState(), common::MissionState::kInterceptExecuting);
    EXPECT_EQ(sender.CallCount(), 0U);  // 任何命令都不得发出

    machine.Stop();
}

}  // namespace
