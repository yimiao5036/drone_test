#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <thread>

#include <gtest/gtest.h>

#include "config/config.h"
#include "perception/target_estimator.h"
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
                                               bool valid = true) {
    common::StereoTargetDistance msg;
    msg.header.sequence = frame_sequence;
    msg.header.source_time_ms = TestMonotonicMs();
    msg.header.receive_time_ms = TestMonotonicMs();
    msg.frame_sequence = frame_sequence;
    msg.valid = valid;
    msg.confidence = 0.9F;
    if (valid) {
        msg.forward_distance_m = 10.0F;
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

}  // namespace
