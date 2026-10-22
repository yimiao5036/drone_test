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

#include "communication/ground_station_link.h"
#include "communication/px4_link.h"
#include "config/config.h"
#include "perception/target_estimator.h"
#include "state_machine/mission_state_machine.h"

#include "../common/log_capture.h"
#include "../communication/fake_command_sender.h"

namespace {

using namespace drone;
using namespace std::chrono_literals;
using drone::test::FakeCommandSender;

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
    // 健康 PX4 电池基线（Task 8）：BAT_LOW_THR=15%、COM_LOW_BAT_ACT=0、
    // 遥测有效且剩余 80%。飞行中电池守卫对所有派生夹具生效，避免旧用例
    // 因“遥测缺失”被误判为电池失效。
    flight.battery_valid = true;
    flight.battery_remaining_valid = true;
    flight.battery_remaining_pct = 80.f;
    flight.battery_low_threshold_valid = true;
    flight.battery_low_threshold_pct = 15.f;
    flight.low_battery_action_valid = true;
    flight.low_battery_action = 0;
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
    auto estimated = MakeValidEstimatedTarget();
    estimated.vel_x_mps = 2.5f;
    estimated.vel_y_mps = -1.5f;
    estimated.vel_z_mps = 0.5f;
    estimated.velocity_valid_mask = common::kVelocityValidAll;
    (void)estimated_topic.Publish(
        std::make_shared<const common::TargetState>(estimated));

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
    // P3：S5 位置意图携带逐轴速度前馈。
    EXPECT_EQ(intent->feedforward_valid_mask, common::kVelocityValidAll);
    EXPECT_FLOAT_EQ(intent->feedforward_vx_mps, 2.5f);
    EXPECT_FLOAT_EQ(intent->feedforward_vy_mps, -1.5f);
    EXPECT_FLOAT_EQ(intent->feedforward_vz_mps, 0.5f);

    machine.Stop();
}

// P3：未置位的速度轴必须清零，避免无效 KF 值污染意图；位置照发。
TEST(MissionStateMachineTest, GpsApproachNormalizesUnsetFeedforwardAxes) {
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
    auto estimated = MakeValidEstimatedTarget();
    estimated.velocity_valid_mask = common::kVelocityValidNorth;  // 仅 N 有效
    estimated.vel_x_mps = 3.0f;
    estimated.vel_y_mps = 9.0f;  // E 未置位：必须被清零
    estimated.vel_z_mps = 9.0f;  // D 未置位：必须被清零
    (void)estimated_topic.Publish(
        std::make_shared<const common::TargetState>(estimated));

    const auto intent = WaitForIntent(intent_subscription);
    ASSERT_TRUE(intent.has_value());
    EXPECT_EQ(intent->feedforward_valid_mask, common::kVelocityValidNorth);
    EXPECT_FLOAT_EQ(intent->feedforward_vx_mps, 3.0f);
    EXPECT_FLOAT_EQ(intent->feedforward_vy_mps, 0.0f);
    EXPECT_FLOAT_EQ(intent->feedforward_vz_mps, 0.0f);

    machine.Stop();
}

// P3：位置不变、仅速度/有效位变化时立即重发（不等待 10Hz 节流窗口）。
TEST(MissionStateMachineTest, GpsApproachRepublishesOnFeedforwardChange) {
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
    auto estimated = MakeValidEstimatedTarget();
    estimated.velocity_valid_mask = common::kVelocityValidNorth;
    estimated.vel_x_mps = 1.0f;
    (void)estimated_topic.Publish(
        std::make_shared<const common::TargetState>(estimated));
    ASSERT_TRUE(WaitForIntent(intent_subscription).has_value());

    // 位置不变，仅改变前馈速度；决策下一拍应立即发布新内容。
    estimated.header.sequence += 1;
    estimated.header.receive_time_ms = TestMonotonicMs();
    estimated.vel_x_mps = 2.0f;
    (void)estimated_topic.Publish(
        std::make_shared<const common::TargetState>(estimated));

    std::optional<common::ControlIntent> updated;
    const auto deadline = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < deadline) {
        auto intent = intent_subscription.WaitTakeFor(50ms);
        if (!intent) {
            continue;
        }
        if (std::fabs((*intent)->feedforward_vx_mps - 2.0f) < 1e-6f) {
            updated = **intent;
            break;
        }
    }
    ASSERT_TRUE(updated.has_value());
    EXPECT_FLOAT_EQ(updated->target_x, 12.5f);  // 位置未变
    EXPECT_EQ(updated->feedforward_valid_mask, common::kVelocityValidNorth);

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
    // P3：交接测试带速度前馈，用于证明 S6 冻结段显式丢弃前馈。
    state.vel_x_mps = 1.5f;
    state.vel_y_mps = -0.5f;
    state.vel_z_mps = 0.0f;
    state.velocity_valid_mask = common::kVelocityValidAll;
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
        // P3：S6 冻结段显式无前馈（即使进入时估计带速度位）。
        EXPECT_EQ(intent.feedforward_valid_mask, 0U);
        EXPECT_FLOAT_EQ(intent.feedforward_vx_mps, 0.0F);
        EXPECT_FLOAT_EQ(intent.feedforward_vy_mps, 0.0F);
        EXPECT_FLOAT_EQ(intent.feedforward_vz_mps, 0.0F);
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
// 一次触发命令（执行器 `Fired()` 防重复；入队失败下拍重试）。Task 5 起复位
// 由执行器在触发 ACK 后按 reset_delay 自行发起（不再由离开拦截序列触发），
// ACK 超时/断连/入队超时 → S16 故障。
// 通道号/行程来自 actuator 配置节；PWM 微秒按 PX4 输出映射区间换算为
// MAV_CMD_DO_SET_ACTUATOR 的归一化参数（-1..1），未用参数位 NaN（规范语义
// 为"忽略"，0 会误动其他执行器组）；p7=执行器组序号（PX4 仅处理 0）。
// 未注入命令接口（enable_control=false 装配语义）→ 零调用（物理缺席）。
//
// 拦截序列（S9/S10/S11）当前尚无生产入口——入口判据（授权策略/射程门限）
// 属后续任务，本任务不发明；用例经 ForceStateForTest 测试入口驱动状态迁移，
// 与生产路径共用同一 TransitionTo/决策线程挂点，触发/复位语义等同生产。
// ---------------------------------------------------------------------------

/// 单发捕网触发行程 1150µs、待机/复位 1500µs、输出映射区间 1000..2000µs
/// 的归一化期望值：norm = 2·(pwm − 1000)/1000 − 1。
constexpr float kNetFireNormalized = -0.7F;
constexpr float kNetIdleNormalized = 0.0F;

/// 捕网执行器用例配置：通道 3（验证通道号落在 p3 而非 p1），行程取默认值；
/// 显式开启测试钩子（拦截序列无生产入口，用例经 `ForceStateForTest` 驱动）。
state_machine::MissionStateMachineConfig MakeNetActuatorTestConfig() {
    state_machine::MissionStateMachineConfig config;
    config.actuator.net_channel = 3;
    config.test_hooks_enabled = true;
    // 本组用例只验证捕网执行器触发/复位语义；关闭单发自动返航，
    // 保持 S10 停留/离开由测试显式强制。
    config.return_home.enabled = false;
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

// 进入 S10 → 恰好一次触发命令；停留多拍不重复触发。触发前置是 PX4 在线
// （Task 5 起状态机要求连接证据，见 MaybeTriggerNetActuator）。
TEST(MissionStateMachineTest, InterceptExecutingFiresOnce) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeNetActuatorTestConfig());
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));
    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(
            MakeNavigationReadyFlight())).accepted);
    std::this_thread::sleep_for(150ms);

    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForCallCount(sender, 1));
    EXPECT_EQ(sender.LastCommand(), 187U);  // MAV_CMD_DO_SET_ACTUATOR
    EXPECT_FLOAT_EQ(sender.LastParams()[2], kNetFireNormalized);

    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(sender.CallCount(), 1U);  // 单发不重复触发
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
    // Task 5 起无连接证据不触发：先发布在线快照（PX4 connected）。
    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(
            MakeNavigationReadyFlight())).accepted);
    std::this_thread::sleep_for(150ms);

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

// 未触发即离开拦截序列：不发任何命令。Task 5 起复位由执行器在触发 ACK 后
// 按自身计时推进（不再由“离开 S9/S10”触发），未发射过的机构无需归位行程。
TEST(MissionStateMachineTest, LeavingInterceptWithoutFireSendsNoCommand) {
    common::Topic<common::GroundStationTarget> ground_target_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeNetActuatorTestConfig());
    machine.SetInputs(ground_target_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));
    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(
            MakeNavigationReadyFlight())).accepted);
    std::this_thread::sleep_for(150ms);

    machine.ForceStateForTest(common::MissionState::kInterceptReady);
    std::this_thread::sleep_for(300ms);
    machine.ForceStateForTest(common::MissionState::kReturnHome);
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(sender.CallCount(), 0U);  // 未发射 → 无复位命令
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

// ---------------------------------------------------------------------------
// S11~S14 单发拦截后返航（2026-10-19 V1 口径）
// 强制 S10 模拟“发射”，走：S10→S11→S12 算力板位置返航→S13 AUTO.LAND→
// S14 DISARMED。Home 局部 NED 由 Home 经纬度 + EKF 锚点换算（Home=锚点时
// Home 局部坐标应为 0,0,0）。
// ---------------------------------------------------------------------------

common::FlightStateSnapshot MakeReturnHomeFlight(float x_m, float y_m, float z_m,
                                                 float vx_mps = 0.0f,
                                                 float vy_mps = 0.0f,
                                                 float vz_mps = 0.0f,
                                                 bool armed = true,
                                                 bool landed = false,
                                                 uint8_t main_mode = 6,
                                                 uint8_t sub_mode = 0,
                                                 bool home_valid = true,
                                                 bool ned_ref_valid = true) {
    common::FlightStateSnapshot flight = MakeNavigationReadyFlight();
    flight.armed = armed;
    flight.landed = landed;
    flight.landed_state_valid = true;
    flight.flight_mode = main_mode;
    flight.flight_sub_mode = sub_mode;
    flight.attitude_valid = true;
    flight.local_position_valid = true;
    flight.local_x_m = x_m;
    flight.local_y_m = y_m;
    flight.local_z_m = z_m;
    flight.vx_mps = vx_mps;
    flight.vy_mps = vy_mps;
    flight.vz_mps = vz_mps;
    flight.home_valid = home_valid;
    flight.ned_ref_valid = ned_ref_valid;
    flight.ned_ref_lat_1e7 = flight.home_lat_1e7;
    flight.ned_ref_lon_1e7 = flight.home_lon_1e7;
    flight.ned_ref_alt_mm = flight.home_altitude_mm;
    return flight;
}

state_machine::MissionStateMachineConfig MakeReturnHomeTestConfig() {
    state_machine::MissionStateMachineConfig config;
    config.test_hooks_enabled = true;
    // 默认 return_home.enabled=true；缩短稳定/重试时间便于测试。
    config.return_home.arrival_hold = 300ms;
    config.return_home.command_retry = 100ms;
    config.return_home.fire_timeout = 1000ms;
    return config;
}

bool WaitForCommand(const FakeCommandSender& sender, uint16_t command,
                    std::chrono::milliseconds timeout = 2000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        for (const auto& call : sender.History()) {
            if (call.command == command) {
                return true;
            }
        }
        std::this_thread::sleep_for(20ms);
    }
    return false;
}

std::optional<common::ControlIntent> WaitForReturnIntent(
    common::Topic<common::ControlIntent>::Subscription& subscription,
    std::chrono::milliseconds timeout = 2000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = subscription.WaitTakeFor(50ms);
        if (!message) {
            continue;
        }
        const auto& intent = **message;
        if (intent.type == common::ControlIntentType::kPositionGuide &&
            (intent.reason_code & 0xFFU) == 8U) {
            return intent;
        }
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// Task 5：执行器 ACK 驱动复位 / 故障 S16（2026-10-21）
// 复位不再由“离开拦截序列”触发：触发 ACK 后执行器按 reset_delay 自身计时
// 发复位，复位 ACK 后再稳定 reset_settle 回到 Ready；等待 ACK 期间超时、
// 非 ACCEPTED、PX4 断连均进入故障 S16（停发一切控制意图、取消待发命令、
// 不重试/不补发复位）。
// ---------------------------------------------------------------------------

TEST(MissionStateMachineTest, ResetRunsAfterFireAckWithoutLeavingS10) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    auto config = MakeNetActuatorTestConfig();
    config.actuator.reset_delay = 0ms;
    config.actuator.reset_settle = 0ms;
    state_machine::MissionStateMachine machine(config);
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));
    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(
            MakeReturnHomeFlight(10.f, 5.f, -10.f))).accepted);
    std::this_thread::sleep_for(150ms);

    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForCallCount(sender, 1));
    EXPECT_EQ(sender.LastCommand(), 187U);

    sender.SimulateAck(187U, 0U, TestMonotonicMs());
    ASSERT_TRUE(WaitForCallCount(sender, 2));
    EXPECT_FLOAT_EQ(sender.LastParams()[2], kNetIdleNormalized);
    EXPECT_EQ(machine.CurrentState(),
              common::MissionState::kInterceptExecuting);
    machine.Stop();
}

TEST(MissionStateMachineTest, FireAckTimeoutEntersFailsafeAndCancelsCommands) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    auto config = MakeNetActuatorTestConfig();
    config.actuator.reset_delay = 0ms;
    config.actuator.reset_settle = 0ms;
    state_machine::MissionStateMachine machine(config);
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));
    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(
            MakeReturnHomeFlight(10.f, 5.f, -10.f))).accepted);
    std::this_thread::sleep_for(150ms);

    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForCallCount(sender, 1));

    sender.SimulateTimeout(187U);
    const auto failsafe = WaitForState(
        status_subscription, common::MissionState::kFailsafe, 2000ms);
    ASSERT_TRUE(failsafe.has_value());
    EXPECT_EQ(sender.CancelCount(), 1U);
    // 故障后不得补发复位
    EXPECT_EQ(sender.CallCount(), 1U);
    EXPECT_NE(failsafe->active_warning_bits & (1U << 4U), 0U);
    machine.Stop();
}

TEST(MissionStateMachineTest, DisconnectDuringFireWaitFaults) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    auto config = MakeNetActuatorTestConfig();
    config.actuator.reset_delay = 0ms;
    config.actuator.reset_settle = 0ms;
    state_machine::MissionStateMachine machine(config);
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    const auto connected = MakeReturnHomeFlight(10.f, 5.f, -10.f);
    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(connected))
                    .accepted);
    std::this_thread::sleep_for(150ms);
    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForCallCount(sender, 1));  // 触发已入队，等待 ACK

    auto disconnected = connected;
    disconnected.connected = false;
    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(disconnected))
                    .accepted);
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kFailsafe, 2000ms)
                    .has_value());
    EXPECT_EQ(sender.CancelCount(), 1U);
    EXPECT_EQ(sender.CallCount(), 1U);  // 不得补发复位
    machine.Stop();
}

TEST(MissionStateMachineTest, FireEnqueueTimeoutFaults) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    auto config = MakeNetActuatorTestConfig();
    config.return_home.fire_timeout = 300ms;
    state_machine::MissionStateMachine machine(config);
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(
            MakeReturnHomeFlight(10.f, 5.f, -10.f))).accepted);
    std::this_thread::sleep_for(100ms);
    sender.FailNextCalls(1000);
    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kFailsafe, 2000ms)
                    .has_value());
    EXPECT_EQ(sender.CancelCount(), 1U);
    machine.Stop();
}

TEST(MissionStateMachineTest, ResetRunsDuringReturnAfterFireAck) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    auto config = MakeReturnHomeTestConfig();
    config.actuator.reset_delay = 0ms;
    config.actuator.reset_settle = 0ms;
    state_machine::MissionStateMachine machine(config);
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(
            MakeReturnHomeFlight(10.f, 5.f, -10.f))).accepted);
    std::this_thread::sleep_for(150ms);

    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kReturnHome, 2500ms)
                    .has_value());
    ASSERT_EQ(sender.CallCount(), 1u);  // 触发已入队

    // 已在 S12 返航；ACK 到达后执行器仍按计时发复位（通道 1 → p1）。
    sender.SimulateAck(187U, 0U, TestMonotonicMs());
    ASSERT_TRUE(WaitForCallCount(sender, 2));
    EXPECT_FLOAT_EQ(sender.LastParams()[0], kNetIdleNormalized);
    machine.Stop();
}

// 回归（最终评审 Important 3）：S16 故障后不得再发布任何控制意图。
// 故障前必须先真正进入会持续出意图的状态（S12 返航），否则“无意图”断言无意义。
TEST(MissionStateMachineTest, NoIntentsAfterActuatorFault) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeReturnHomeTestConfig());
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    auto status_subscription = machine.StatusOutput().Subscribe(32);
    auto intent_subscription = machine.IntentOutput().Subscribe(32);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(
            MakeReturnHomeFlight(10.f, 5.f, -10.f))).accepted);
    std::this_thread::sleep_for(150ms);

    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForCallCount(sender, 1));  // 触发已入队，等待 ACK

    // 先进入 S12 并确认返航意图在持续发布，再制造执行器 ACK 超时。
    machine.ForceStateForTest(common::MissionState::kReturnHome);
    ASSERT_TRUE(WaitForReturnIntent(intent_subscription).has_value());

    sender.SimulateTimeout(187U);
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kFailsafe, 2000ms)
                    .has_value());
    EXPECT_EQ(sender.CancelCount(), 1U);

    // 清空故障前已排队的意图，之后 300ms 内不得再出现新意图。
    while (intent_subscription.WaitTakeFor(20ms).has_value()) {
    }
    std::this_thread::sleep_for(300ms);
    EXPECT_FALSE(intent_subscription.WaitTakeFor(100ms).has_value());
    machine.Stop();
}

TEST(MissionStateMachineTest, InterceptionFiresThenReturnsHomeAndLands) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeReturnHomeTestConfig());
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    auto intent_subscription = machine.IntentOutput().Subscribe(16);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    // 当前在 Home 北 10m/东 5m、高 10m（≥5m → 保持当前高度返航）。
    ASSERT_TRUE(flight_topic.Publish(std::make_shared<const common::FlightStateSnapshot>(
                                         MakeReturnHomeFlight(10.0f, 5.0f, -10.0f)))
                    .accepted);
    std::this_thread::sleep_for(150ms);

    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kReturnHome, 2500ms)
                    .has_value());

    const auto return_intent = WaitForReturnIntent(intent_subscription);
    ASSERT_TRUE(return_intent.has_value());
    EXPECT_EQ(return_intent->control_source, 0U);
    EXPECT_NE(return_intent->reason_code & common::kControlIntentReasonRequestControl,
              0U);
    EXPECT_NEAR(return_intent->target_x, 0.0f, 0.05f);   // Home 局部 NED x
    EXPECT_NEAR(return_intent->target_y, 0.0f, 0.05f);
    EXPECT_NEAR(return_intent->target_z, -10.0f, 0.05f); // 保持当前高度
    EXPECT_EQ(return_intent->feedforward_valid_mask, 0U);

    // 到达 Home 附近并稳定 300ms → S13；发 AUTO.LAND（176，4/6）。
    for (int round = 0; round < 8; ++round) {
        ASSERT_TRUE(flight_topic.Publish(
            std::make_shared<const common::FlightStateSnapshot>(
                MakeReturnHomeFlight(0.5f, 0.0f, -10.0f))).accepted);
        std::this_thread::sleep_for(80ms);
    }
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kLanding, 2500ms)
                    .has_value());
    ASSERT_TRUE(WaitForCommand(sender, 176U));
    {
        const auto history = sender.History();
        bool found = false;
        for (const auto& call : history) {
            if (call.command == 176U) {
                found = true;
                EXPECT_FLOAT_EQ(call.params[0], 1.0f);  // custom mode enabled
                EXPECT_FLOAT_EQ(call.params[1], 4.0f);  // AUTO
                EXPECT_FLOAT_EQ(call.params[2], 6.0f);  // LAND
            }
        }
        EXPECT_TRUE(found);
    }

    // 落地但仍 armed → DISARM（400，param1=0）。
    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(
            MakeReturnHomeFlight(0.0f, 0.0f, 0.0f, 0,0,0, /*armed=*/true,
                                 /*landed=*/true, 4, 6))).accepted);
    ASSERT_TRUE(WaitForCommand(sender, 400U));

    // 确认 disarmed → S14。
    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(
            MakeReturnHomeFlight(0.0f, 0.0f, 0.0f, 0,0,0, /*armed=*/false,
                                 /*landed=*/true, 4, 6))).accepted);
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kDisarmed, 2500ms)
                    .has_value());
    machine.Stop();
}

// S5 无目标停发后 PX4 已自行 RTL 落地/上锁、模式又恢复 Offboard：
// S12 不得继续发返航设定值（会卡到返航超时），直接 S13→S14 收口。
TEST(MissionStateMachineTest, ReturnHomeAlreadyLandedDisarmedCloses) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeReturnHomeTestConfig());
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    auto intent_subscription = machine.IntentOutput().Subscribe(16);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    ASSERT_TRUE(flight_topic.Publish(std::make_shared<const common::FlightStateSnapshot>(
                                         MakeReturnHomeFlight(
                                             0.0f, 0.0f, 0.0f, 0, 0, 0,
                                             /*armed=*/false, /*landed=*/true)))
                    .accepted);
    std::this_thread::sleep_for(150ms);

    machine.ForceStateForTest(common::MissionState::kReturnHome);
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kLanding, 2500ms)
                    .has_value());
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kDisarmed, 2500ms)
                    .has_value());
    EXPECT_FALSE(intent_subscription.WaitTakeFor(100ms).has_value())
        << "已落地/上锁不得继续发布返航位置意图";
    machine.Stop();
}

TEST(MissionStateMachineTest, ReturnHomeClimbsToFiveMetersBeforeCruise) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeReturnHomeTestConfig());
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    auto intent_subscription = machine.IntentOutput().Subscribe(16);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    // 相对 Home 仅 2m：先原地爬到 5m，再水平返航。
    ASSERT_TRUE(flight_topic.Publish(std::make_shared<const common::FlightStateSnapshot>(
                                         MakeReturnHomeFlight(10.0f, 5.0f, -2.0f)))
                    .accepted);
    std::this_thread::sleep_for(150ms);
    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kReturnHome, 2500ms)
                    .has_value());

    const auto climb_intent = WaitForReturnIntent(intent_subscription);
    ASSERT_TRUE(climb_intent.has_value());
    EXPECT_NEAR(climb_intent->target_x, 10.0f, 0.05f);  // 原地爬升
    EXPECT_NEAR(climb_intent->target_y, 5.0f, 0.05f);
    EXPECT_NEAR(climb_intent->target_z, -5.0f, 0.05f);  // Home 上方 5m

    // 到达 5m、垂速归零 → 进入水平巡航（目标 Home x/y）。
    for (int round = 0; round < 8; ++round) {
        ASSERT_TRUE(flight_topic.Publish(
            std::make_shared<const common::FlightStateSnapshot>(
                MakeReturnHomeFlight(10.0f, 5.0f, -5.0f))).accepted);
        std::this_thread::sleep_for(80ms);
    }
    std::optional<common::ControlIntent> cruise_intent;
    const auto deadline = std::chrono::steady_clock::now() + 2000ms;
    while (std::chrono::steady_clock::now() < deadline) {
        auto intent = WaitForReturnIntent(intent_subscription, 100ms);
        if (!intent) {
            continue;
        }
        if (std::fabs(intent->target_x) < 0.05f &&
            std::fabs(intent->target_y) < 0.05f) {
            cruise_intent = intent;
            break;
        }
    }
    ASSERT_TRUE(cruise_intent.has_value());
    EXPECT_NEAR(cruise_intent->target_z, -5.0f, 0.05f);
    machine.Stop();
}

TEST(MissionStateMachineTest, ReturnHomeHoldsWhenHomeInvalid) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeReturnHomeTestConfig());
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    auto intent_subscription = machine.IntentOutput().Subscribe(16);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(
            MakeReturnHomeFlight(10.0f, 5.0f, -10.0f, 0,0,0, true, false, 6, 0,
                                 /*home_valid=*/false))).accepted);
    std::this_thread::sleep_for(150ms);
    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kReturnHome, 2500ms)
                    .has_value());

    const auto hold_intent = WaitForReturnIntent(intent_subscription);
    ASSERT_TRUE(hold_intent.has_value());
    EXPECT_NEAR(hold_intent->target_x, 10.0f, 0.05f);
    EXPECT_NEAR(hold_intent->target_y, 5.0f, 0.05f);
    EXPECT_NEAR(hold_intent->target_z, -10.0f, 0.05f);
    EXPECT_EQ(machine.CurrentState(), common::MissionState::kReturnHome);
    machine.Stop();
}

TEST(MissionStateMachineTest, ReturnHomeYieldsWhenPx4LeavesOffboard) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeReturnHomeTestConfig());
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetCommandSender(&sender);
    auto status_subscription = machine.StatusOutput().Subscribe(16);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));

    ASSERT_TRUE(flight_topic.Publish(std::make_shared<const common::FlightStateSnapshot>(
                                         MakeReturnHomeFlight(10.0f, 5.0f, -10.0f)))
                    .accepted);
    std::this_thread::sleep_for(150ms);
    machine.ForceStateForTest(common::MissionState::kInterceptExecuting);
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kReturnHome, 2500ms)
                    .has_value());

    // PX4 自主切到 AUTO.RTL（4/5）：状态机停止返航设定值，转 S13 仅监控。
    ASSERT_TRUE(flight_topic.Publish(
        std::make_shared<const common::FlightStateSnapshot>(
            MakeReturnHomeFlight(10.0f, 5.0f, -10.0f, 0,0,0, true, false, 4, 5)))
                    .accepted);
    ASSERT_TRUE(WaitForState(status_subscription,
                             common::MissionState::kLanding, 2500ms)
                    .has_value());
    // PX4 接管时不得再发 AUTO.LAND（176）。
    std::this_thread::sleep_for(300ms);
    for (const auto& call : sender.History()) {
        EXPECT_NE(call.command, 176U);
    }
    machine.Stop();
}

// ---------------------------------------------------------------------------
// S9/S10 生产入口（2026-10-19）：LOCKED 连续帧 + 测距窗口 + Offboard/armed/
// 姿态/位置有效 + 命令接口就绪 → S9 → 自动授权 → S10 发射。
// ---------------------------------------------------------------------------

state_machine::MissionStateMachineConfig MakeInterceptEntryTestConfig() {
    state_machine::MissionStateMachineConfig config;
    config.test_hooks_enabled = true;
    config.interception.enabled = true;
    config.interception.auto_authorize = true;
    config.interception.lock_frames = 3;
    config.interception.min_range_m = 2.0;
    config.interception.max_range_m = 12.0;
    config.return_home.arrival_hold = 300ms;
    config.return_home.command_retry = 100ms;
    return config;
}

TEST(MissionStateMachineTest, InterceptEntryLockAndRangeThenFiresAndReturns) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    common::Topic<common::TargetState> estimated_topic;
    common::Topic<common::VisualTargetStatus> visual_topic;
    common::Topic<common::StereoTargetDistance> distance_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeInterceptEntryTestConfig());
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetEstimationInputs(estimated_topic, visual_topic, distance_topic);
    machine.SetCommandSender(&sender);
    auto status_subscription = machine.StatusOutput().Subscribe(32);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));
    machine.ForceStateForTest(common::MissionState::kVisualTracking);

    bool saw_ready = false;
    bool saw_executing = false;
    bool saw_return = false;
    for (uint64_t frame = 1; frame <= 20 && !saw_return; ++frame) {
        ASSERT_TRUE(flight_topic.Publish(
            std::make_shared<const common::FlightStateSnapshot>(
                MakeReturnHomeFlight(10.0f, 5.0f, -10.0f))).accepted);
        ASSERT_TRUE(visual_topic.Publish(
            std::make_shared<const common::VisualTargetStatus>(
                MakeVisualStatus(common::VisualTargetState::kLocked, frame)))
                        .accepted);
        ASSERT_TRUE(distance_topic.Publish(
            std::make_shared<const common::StereoTargetDistance>(
                MakeFreshDistance(frame, true, 8.0F))).accepted);
        std::this_thread::sleep_for(60ms);
        while (auto status = status_subscription.TryTake()) {
            switch ((*status)->state) {
                case common::MissionState::kInterceptReady: saw_ready = true; break;
                case common::MissionState::kInterceptExecuting:
                    saw_executing = true;
                    break;
                case common::MissionState::kReturnHome: saw_return = true; break;
                default: break;
            }
        }
    }
    EXPECT_TRUE(saw_ready) << "锁定+测距窗口满足后应进入 S9";
    EXPECT_TRUE(saw_executing) << "自动授权后应进入 S10";
    EXPECT_TRUE(saw_return) << "单发发射后应进入 S12 返航";
    EXPECT_TRUE(WaitForCommand(sender, 187U)) << "S10 应发捕网触发命令";
    machine.Stop();
}

TEST(MissionStateMachineTest, InterceptEntryRejectsDistanceOutsideWindow) {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    common::Topic<common::TargetState> estimated_topic;
    common::Topic<common::VisualTargetStatus> visual_topic;
    common::Topic<common::StereoTargetDistance> distance_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine(MakeInterceptEntryTestConfig());
    machine.SetInputs(ground_topic, flight_topic, health_topic);
    machine.SetEstimationInputs(estimated_topic, visual_topic, distance_topic);
    machine.SetCommandSender(&sender);
    ASSERT_TRUE(machine.Start());
    ASSERT_TRUE(WaitForSelfCheck(machine));
    machine.ForceStateForTest(common::MissionState::kVisualTracking);

    // 测距 20m 超出 2~12m 窗口：LOCKED 再多也不得进入 S9/发射。
    for (uint64_t frame = 1; frame <= 15; ++frame) {
        ASSERT_TRUE(flight_topic.Publish(
            std::make_shared<const common::FlightStateSnapshot>(
                MakeReturnHomeFlight(10.0f, 5.0f, -10.0f))).accepted);
        ASSERT_TRUE(visual_topic.Publish(
            std::make_shared<const common::VisualTargetStatus>(
                MakeVisualStatus(common::VisualTargetState::kLocked, frame)))
                        .accepted);
        ASSERT_TRUE(distance_topic.Publish(
            std::make_shared<const common::StereoTargetDistance>(
                MakeFreshDistance(frame, true, 20.0F))).accepted);
        std::this_thread::sleep_for(60ms);
    }
    EXPECT_EQ(machine.CurrentState(), common::MissionState::kVisualTracking);
    EXPECT_EQ(sender.CallCount(), 0U);
    machine.Stop();
}

// ---------------------------------------------------------------------------
// 多机任务分配：65014 输入、S2 预检与 65015 回执（2026-10-22，Task 4）
// ---------------------------------------------------------------------------

/// 地面待飞快照：导航就绪 + 落地状态有效且已落地 + 未 armed + 局部位置/姿态
/// 有效。S2 分配预检要求这些前置条件全部成立。
common::FlightStateSnapshot MakeGroundedFlight() {
    common::FlightStateSnapshot flight = MakeNavigationReadyFlight();
    flight.landed_state_valid = true;
    flight.landed = true;
    flight.armed = false;
    flight.local_position_valid = true;
    flight.local_x_m = 1.5f;
    flight.local_y_m = -2.5f;
    flight.local_z_m = 0.1f;
    flight.attitude_valid = true;
    // 电池门禁默认满足（Task 8）：PX4 BAT_LOW_THR=15%、COM_LOW_BAT_ACT=0、
    // 遥测有效且剩余 80%。用例只按需覆写单项字段。
    flight.battery_valid = true;
    flight.battery_remaining_valid = true;
    flight.battery_remaining_pct = 80.f;
    flight.battery_low_threshold_valid = true;
    flight.battery_low_threshold_pct = 15.f;
    flight.low_battery_action_valid = true;
    flight.low_battery_action = 0;
    return flight;
}

/// 65014 分配消息（默认字段全部合法：授权、目标 7、捕网 25、有效期 5s）。
common::MissionAssignment MakeAssignment(uint32_t assignment_id) {
    common::MissionAssignment assignment;
    assignment.header.sequence = assignment_id;
    assignment.header.source_time_ms = TestMonotonicMs();
    assignment.header.receive_time_ms = TestMonotonicMs();
    assignment.header.valid_for_ms = 5000;
    assignment.header.source_id = 255;
    assignment.header.health = 1;
    assignment.protocol_version = 1;
    assignment.assignment_id = assignment_id;
    assignment.target_id = 7;
    assignment.interceptor_type = 25;
    assignment.flags = 1;  // bit0=授权自动起飞
    assignment.valid_for_ms = 5000;
    assignment.received_ms = TestMonotonicMs();
    return assignment;
}

/// 65014 取消当前分配（flags.bit1；取消不要求 bit0 授权位）。
common::MissionAssignment MakeCancel(uint32_t assignment_id) {
    common::MissionAssignment assignment = MakeAssignment(assignment_id);
    assignment.flags = communication::kMissionAssignFlagCancel;
    return assignment;
}

std::optional<common::MissionAssignmentAck> WaitForAssignmentAck(
    common::Topic<common::MissionAssignmentAck>::Subscription& subscription,
    std::chrono::milliseconds timeout = 1500ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = subscription.WaitTakeFor(50ms);
        if (!message) {
            continue;
        }
        return **message;
    }
    return std::nullopt;
}

/// 任务分配用例默认起飞时序：bootstrap/mode/arm 超时放宽到远大于 Task 4
/// 用例时长，保证只验证“分配入口与 S2 预检”的用例不会被 S3 超时中止。
/// S3/S4 用例显式传入 `MakeTakeoffTestConfig()`（缩短时序）。
state_machine::MissionStateMachineConfig MakeAssignmentHarnessConfig() {
    state_machine::MissionStateMachineConfig config;
    config.takeoff.bootstrap_lead = 5000ms;
    config.takeoff.mode_timeout = 5000ms;
    config.takeoff.arm_timeout = 5000ms;
    config.takeoff.bootstrap_timeout = 16000ms;
    return config;
}

/// S3/S4 用例配置：缩短各超时以便在单测内驱动完整时序。
/// bootstrap_timeout 必须严格大于 lead+mode+arm（TakeoffConfig::Validate）。
state_machine::MissionStateMachineConfig MakeTakeoffTestConfig(
    std::chrono::milliseconds lead = 100ms,
    std::chrono::milliseconds mode_timeout = 1000ms,
    std::chrono::milliseconds arm_timeout = 1000ms,
    std::chrono::milliseconds height_timeout = 3000ms,
    double height_m = 0.5, double climb_rate_mps = 0.5) {
    state_machine::MissionStateMachineConfig config;
    auto& takeoff = config.takeoff;
    takeoff.bootstrap_lead = lead;
    takeoff.mode_timeout = mode_timeout;
    takeoff.arm_timeout = arm_timeout;
    takeoff.bootstrap_timeout =
        lead + mode_timeout + arm_timeout + 1000ms;
    takeoff.height_timeout = height_timeout;
    takeoff.height_m = height_m;
    takeoff.climb_rate_mps = climb_rate_mps;
    takeoff.stable_time = 100ms;
    takeoff.z_tolerance_m = 0.2;
    takeoff.xy_tolerance_m = 0.5;
    takeoff.max_vz_mps = 0.3;
    return config;
}

/// 等待指定结果的 65015（跳过队列里更早的回执，如 accepted）。
std::optional<common::MissionAssignmentAck> WaitForAckWithResult(
    common::Topic<common::MissionAssignmentAck>::Subscription& subscription,
    common::MissionAssignmentResult result,
    std::chrono::milliseconds timeout = 3000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = subscription.WaitTakeFor(50ms);
        if (!message) {
            continue;
        }
        if ((*message)->result == result) {
            return **message;
        }
    }
    return std::nullopt;
}

/// 等待满足条件的控制意图（跳过队列里更早的意图）。
template <typename Predicate>
std::optional<common::ControlIntent> WaitForIntent(
    common::Topic<common::ControlIntent>::Subscription& subscription,
    Predicate predicate, std::chrono::milliseconds timeout = 1500ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = subscription.WaitTakeFor(50ms);
        if (!message) {
            continue;
        }
        if (predicate(**message)) {
            return **message;
        }
    }
    return std::nullopt;
}

/// 清空意图订阅积压（中止后检查“无残留意图”前先丢掉中止前的意图）。
void DrainIntents(
    common::Topic<common::ControlIntent>::Subscription& subscription) {
    while (auto message = subscription.TryTake()) {
    }
}

bool PublishFlight(common::Topic<common::FlightStateSnapshot>& topic,
                   const common::FlightStateSnapshot& flight) {
    return topic
        .Publish(std::make_shared<const common::FlightStateSnapshot>(flight))
        .accepted;
}

/// Offboard(6/0) 快照：armed/landed 由参数指定（地面待飞或空中）。
common::FlightStateSnapshot MakeOffboardFlight(bool armed, bool landed) {
    common::FlightStateSnapshot flight = MakeGroundedFlight();
    flight.armed = armed;
    flight.landed = landed;
    flight.landed_state_valid = true;
    flight.flight_mode = 6;
    flight.flight_sub_mode = 0;
    return flight;
}

/// 起飞原点为 (1.5,-2.5,0.1)（MakeGroundedFlight），height=0.5 时目标 z=-0.4。
constexpr float kTakeoffOriginX = 1.5f;
constexpr float kTakeoffOriginY = -2.5f;
constexpr float kTakeoffOriginZ = 0.1f;

/// 分配用例夹具：统一装配输入/分配 Topic、命令接口与估计输入，并驱动到 S2。
struct AssignmentHarness {
    common::Topic<common::GroundStationTarget> ground_topic;
    common::Topic<common::FlightStateSnapshot> flight_topic;
    common::Topic<common::HealthStatus> health_topic;
    common::Topic<common::TargetState> estimated_topic;
    common::Topic<common::VisualTargetStatus> visual_topic;
    common::Topic<common::StereoTargetDistance> distance_topic;
    common::Topic<common::MissionAssignment> assignment_topic;
    FakeCommandSender sender;
    state_machine::MissionStateMachine machine;
    common::Topic<common::MissionStatus>::Subscription status_subscription;
    common::Topic<common::MissionAssignmentAck>::Subscription ack_subscription;
    common::Topic<common::ControlIntent>::Subscription intent_subscription;

    AssignmentHarness() : AssignmentHarness(MakeAssignmentHarnessConfig()) {}

    explicit AssignmentHarness(state_machine::MissionStateMachineConfig config)
        : machine(config) {
        machine.SetInputs(ground_topic, flight_topic, health_topic);
        machine.SetAssignmentInput(assignment_topic);
        status_subscription = machine.StatusOutput().Subscribe(32);
        ack_subscription = machine.AssignmentAckOutput().Subscribe(32);
        intent_subscription = machine.IntentOutput().Subscribe(64);
    }

    /// 装配并启动；with_control=false 模拟影子装配（不注入命令接口）。
    bool Start(bool with_control, bool with_estimation,
               common::FlightStateSnapshot flight = MakeGroundedFlight()) {
        if (with_estimation) {
            machine.SetEstimationInputs(estimated_topic, visual_topic,
                                        distance_topic);
        }
        if (with_control) {
            machine.SetCommandSender(&sender);
        }
        if (!machine.Start()) {
            return false;
        }
        if (!flight_topic
                 .Publish(std::make_shared<const common::FlightStateSnapshot>(
                     flight))
                 .accepted) {
            return false;
        }
        return WaitForState(status_subscription, common::MissionState::kReady)
            .has_value();
    }

    bool Send(const common::MissionAssignment& assignment) {
        return assignment_topic
            .Publish(std::make_shared<const common::MissionAssignment>(
                assignment))
            .accepted;
    }
};

/// 清空 65015 订阅积压（检查“无重复 aborted”前先丢掉中止前的回执）。
void DrainAcks(
    common::Topic<common::MissionAssignmentAck>::Subscription& subscription) {
    while (auto message = subscription.TryTake()) {
    }
}

/// 驱动夹具从 S2 经 S3/S4 到 S5（GPS_APPROACH）。要求 harness 使用
/// MakeTakeoffTestConfig（height=0.5，原点 z=0.1 → 目标 z=-0.4）。
bool DriveToGpsApproach(AssignmentHarness& harness, uint32_t assignment_id) {
    if (!harness.Send(MakeAssignment(assignment_id))) {
        return false;
    }
    if (!WaitForState(harness.status_subscription,
                      common::MissionState::kArming)
             .has_value()) {
        return false;
    }
    if (!WaitForCallCount(harness.sender, 1)) {  // 176
        return false;
    }
    if (!PublishFlight(harness.flight_topic,
                       MakeOffboardFlight(/*armed=*/false, /*landed=*/true))) {
        return false;
    }
    if (!WaitForCallCount(harness.sender, 2)) {  // 400
        return false;
    }
    if (!PublishFlight(harness.flight_topic,
                       MakeOffboardFlight(/*armed=*/true, /*landed=*/false))) {
        return false;
    }
    if (!WaitForState(harness.status_subscription,
                      common::MissionState::kTakeoff)
             .has_value()) {
        return false;
    }
    auto arrived = MakeOffboardFlight(/*armed=*/true, /*landed=*/false);
    arrived.local_x_m = kTakeoffOriginX;
    arrived.local_y_m = kTakeoffOriginY;
    arrived.local_z_m = kTakeoffOriginZ - 0.5f;
    arrived.vz_mps = 0.f;
    if (!PublishFlight(harness.flight_topic, arrived)) {
        return false;
    }
    return WaitForState(harness.status_subscription,
                        common::MissionState::kGpsApproach, 4000ms)
        .has_value();
}

// 未注入命令接口（enable_control=false 影子装配）：分配必须拒绝
// control_disabled，状态保持 S2，availability=unavailable（不可被调度）。
TEST(MissionStateMachineTest, AssignmentRejectedWithoutCommandSender) {
    AssignmentHarness harness;
    ASSERT_TRUE(harness.Start(/*with_control=*/false,
                              /*with_estimation=*/true));

    ASSERT_TRUE(harness.Send(MakeAssignment(1)));
    const auto ack = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(ack->assignment_id, 1U);
    EXPECT_EQ(ack->target_id, 7U);
    EXPECT_EQ(ack->result, common::MissionAssignmentResult::kRejected);
    EXPECT_EQ(ack->reason, common::MissionAssignmentReason::kControlDisabled);
    EXPECT_EQ(ack->mission_state, common::MissionState::kReady);
    EXPECT_EQ(ack->availability, common::MissionAvailability::kUnavailable);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
    EXPECT_EQ(harness.sender.CallCount(), 0U);
}

// 有命令接口但未绑定估计输入：控制链不完整，拒绝 preflight_failed，保持 S2。
TEST(MissionStateMachineTest, AssignmentRejectedWithoutEstimationInputs) {
    AssignmentHarness harness;
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/false));

    ASSERT_TRUE(harness.Send(MakeAssignment(2)));
    const auto ack = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(ack->result, common::MissionAssignmentResult::kRejected);
    EXPECT_EQ(ack->reason, common::MissionAssignmentReason::kPreflightFailed);
    EXPECT_EQ(ack->availability, common::MissionAvailability::kReady);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
}

// 字段级校验：版本/未知 flags/未授权/target 0/类型不匹配/有效期非法或过期
// 全部 rejected，且保持 S2（预检拒绝不消耗任务机会）。
TEST(MissionStateMachineTest, AssignmentInvalidFieldsRejected) {
    AssignmentHarness harness;
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));

    struct Case {
        common::MissionAssignment assignment;
        common::MissionAssignmentReason expected_reason;
    };
    std::vector<Case> cases;
    {
        auto assignment = MakeAssignment(10);
        assignment.protocol_version = 2;
        cases.push_back(
            {assignment, common::MissionAssignmentReason::kUnsupportedVersion});
    }
    {
        auto assignment = MakeAssignment(11);
        assignment.flags = 1U | 0x80U;  // 未知位
        cases.push_back(
            {assignment, common::MissionAssignmentReason::kInvalidMessage});
    }
    {
        auto assignment = MakeAssignment(12);
        assignment.flags = 0;  // 未授权 bit0
        cases.push_back(
            {assignment, common::MissionAssignmentReason::kInvalidMessage});
    }
    {
        auto assignment = MakeAssignment(13);
        assignment.target_id = 0;
        cases.push_back(
            {assignment, common::MissionAssignmentReason::kInvalidMessage});
    }
    {
        auto assignment = MakeAssignment(14);
        assignment.interceptor_type = 26;  // 火箭类，与本机捕网 25 不匹配
        cases.push_back({assignment,
                         common::MissionAssignmentReason::kUnsupportedInterceptorType});
    }
    {
        auto assignment = MakeAssignment(15);
        assignment.valid_for_ms = 0;
        cases.push_back(
            {assignment, common::MissionAssignmentReason::kInvalidMessage});
    }
    {
        auto assignment = MakeAssignment(16);
        assignment.received_ms = TestMonotonicMs() - 5000;  // 已过期
        assignment.valid_for_ms = 1000;
        cases.push_back(
            {assignment, common::MissionAssignmentReason::kInvalidMessage});
    }

    for (const auto& test_case : cases) {
        ASSERT_TRUE(harness.Send(test_case.assignment));
        const auto ack = WaitForAssignmentAck(harness.ack_subscription);
        ASSERT_TRUE(ack.has_value());
        EXPECT_EQ(ack->assignment_id, test_case.assignment.assignment_id);
        EXPECT_EQ(ack->result, common::MissionAssignmentResult::kRejected);
        EXPECT_EQ(ack->reason, test_case.expected_reason);
        EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
    }
}

// 合法分配 + 地面/未 armed/导航有效 → accepted、65015 availability=assigned、
// S2→S3；65013 写入 assignment_id/target_id/availability/interceptor_type。
// S3 入口动作（Task 5）：立即发布 kTakeoff 意图；176 必须等 bootstrap_lead。
TEST(MissionStateMachineTest, AssignmentAcceptedEntersArming) {
    AssignmentHarness harness;
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    const uint64_t transitions_before = harness.machine.TransitionCount();

    ASSERT_TRUE(harness.Send(MakeAssignment(42)));
    const auto ack = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(ack->assignment_id, 42U);
    EXPECT_EQ(ack->target_id, 7U);
    EXPECT_EQ(ack->result, common::MissionAssignmentResult::kAccepted);
    EXPECT_EQ(ack->reason, common::MissionAssignmentReason::kNone);
    EXPECT_EQ(ack->mission_state, common::MissionState::kReady);
    EXPECT_EQ(ack->availability, common::MissionAvailability::kAssigned);

    const auto status = WaitForState(harness.status_subscription,
                                     common::MissionState::kArming);
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->assignment_id, 42U);
    EXPECT_EQ(status->target_id, 7U);
    EXPECT_EQ(status->availability, common::MissionAvailability::kAssigned);
    EXPECT_EQ(status->interceptor_type, 25U);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kArming);
    EXPECT_EQ(harness.machine.TransitionCount(), transitions_before + 1);
    // S3 每拍发布起飞意图（原点、bit31）；默认用例 bootstrap_lead=5s，
    // 断言窗口内不得提前发 176/400。
    const auto intent = WaitForIntent(
        harness.intent_subscription, [](const common::ControlIntent& value) {
            return value.type == common::ControlIntentType::kTakeoff;
        });
    ASSERT_TRUE(intent.has_value());
    EXPECT_FLOAT_EQ(intent->target_x, kTakeoffOriginX);
    EXPECT_FLOAT_EQ(intent->target_y, kTakeoffOriginY);
    EXPECT_FLOAT_EQ(intent->target_z, kTakeoffOriginZ);
    EXPECT_EQ(harness.sender.CallCount(), 0U)
        << "bootstrap_lead 内不得提前发 176/400";
}

// 重复 assignment_id：只重发原 65015，不再预检/迁移。先用“已 armed”
// 让首次分配被拒绝（保持 S2），恢复地面状态后重发同一 id 仍必须重发原
// 拒绝回执且不进入 S3；换新 id 才允许接受。
// 已 accepted 后、S3 执行期间再次收到同一 assignment_id：必须重发 accepted
// 回执，不重新预检、不重复迁移；新的 assignment_id 则拒绝。
TEST(MissionStateMachineTest, AcceptedAssignmentDuplicateWhileArmingResendsAck) {
    AssignmentHarness harness;
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));

    ASSERT_TRUE(harness.Send(MakeAssignment(43)));
    const auto first = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->result, common::MissionAssignmentResult::kAccepted);
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    const uint64_t transitions_after_accept = harness.machine.TransitionCount();

    ASSERT_TRUE(harness.Send(MakeAssignment(43)));
    const auto duplicate = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(duplicate.has_value());
    EXPECT_EQ(duplicate->assignment_id, 43U);
    EXPECT_EQ(duplicate->result, common::MissionAssignmentResult::kAccepted);
    EXPECT_EQ(duplicate->availability, common::MissionAvailability::kAssigned);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kArming);
    EXPECT_EQ(harness.machine.TransitionCount(), transitions_after_accept);

    // 任务执行中的新分配必须拒绝，不打断当前任务。
    ASSERT_TRUE(harness.Send(MakeAssignment(44)));
    const auto rejected = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(rejected.has_value());
    EXPECT_EQ(rejected->assignment_id, 44U);
    EXPECT_EQ(rejected->result, common::MissionAssignmentResult::kRejected);
    EXPECT_EQ(rejected->reason,
              common::MissionAssignmentReason::kAlreadyAssigned);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kArming);
}

TEST(MissionStateMachineTest, DuplicateAssignmentResendsAckWithoutReentry) {
    AssignmentHarness harness;
    auto armed_flight = MakeGroundedFlight();
    armed_flight.armed = true;  // 地面未上锁前置不满足
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true, armed_flight));

    ASSERT_TRUE(harness.Send(MakeAssignment(7)));
    auto first = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->result, common::MissionAssignmentResult::kRejected);
    EXPECT_EQ(first->reason, common::MissionAssignmentReason::kPreflightFailed);
    ASSERT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);

    // 重复 id：同一条 65015 重发，不重复预检。
    ASSERT_TRUE(harness.Send(MakeAssignment(7)));
    auto second = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->result, common::MissionAssignmentResult::kRejected);
    EXPECT_EQ(second->reason, common::MissionAssignmentReason::kPreflightFailed);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);

    // 恢复地面状态后重发同一 id：WasHandled 命中，仍只重发原回执。
    ASSERT_TRUE(harness.flight_topic
                    .Publish(std::make_shared<const common::FlightStateSnapshot>(
                        MakeGroundedFlight()))
                    .accepted);
    std::this_thread::sleep_for(300ms);
    ASSERT_TRUE(harness.Send(MakeAssignment(7)));
    auto third = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(third.has_value());
    EXPECT_EQ(third->result, common::MissionAssignmentResult::kRejected);
    EXPECT_EQ(third->reason, common::MissionAssignmentReason::kPreflightFailed);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady)
        << "重复 assignment_id 不得重新进入 S3";

    // 新 id 才允许接受（预检拒绝不消耗任务机会）。
    ASSERT_TRUE(harness.Send(MakeAssignment(8)));
    auto fourth = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(fourth.has_value());
    EXPECT_EQ(fourth->result, common::MissionAssignmentResult::kAccepted);
    // 65015 先于状态迁移发布：以状态 Topic 的 S3 为准（避免测试线程抢在
    // TransitionTo 之前读取 CurrentState 的竞态）。
    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
}

// 注入命令接口后，65010 地面站目标不得单独把 S2 推到 S5；必须由 65014
// 分配进入 S3（生产控制路径语义）。
TEST(MissionStateMachineTest, GroundTargetDoesNotEnterGpsApproachWithControl) {
    AssignmentHarness harness;
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    const uint64_t transitions_before = harness.machine.TransitionCount();

    ASSERT_TRUE(harness.ground_topic
                    .Publish(std::make_shared<const common::GroundStationTarget>(
                        MakeFreshGroundTarget()))
                    .accepted);
    std::this_thread::sleep_for(500ms);

    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady)
        << "有控制路径时 65010 目标不得单独进入 GPS_APPROACH";
    EXPECT_EQ(harness.machine.TransitionCount(), transitions_before);
}

// 影子兼容：绑定分配输入但未注入命令接口时，既有 S2→S5 影子路径原样保留。
TEST(MissionStateMachineTest, ShadowGroundTargetStillEntersGpsApproach) {
    AssignmentHarness harness;
    ASSERT_TRUE(harness.Start(/*with_control=*/false,
                              /*with_estimation=*/false));

    ASSERT_TRUE(harness.ground_topic
                    .Publish(std::make_shared<const common::GroundStationTarget>(
                        MakeFreshGroundTarget()))
                    .accepted);
    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kGpsApproach)
                    .has_value());
}

// ---------------------------------------------------------------------------
// S3 解锁/Offboard 与 S4 自动爬升（2026-10-22，Task 5）
// ---------------------------------------------------------------------------

// S3 入口每拍发布 kTakeoff 意图（原点、bit31、局部 NED）；bootstrap_lead
// 之前不得发 176，满足后恰好发一次 176（p1=1/p2=6/p3=0），停留不重复。
TEST(MissionStateMachineTest, TakeoffArmingPublishesIntentAndSendsModeOnce) {
    AssignmentHarness harness(
        MakeTakeoffTestConfig(/*lead=*/500ms, 2000ms, 2000ms, 3000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(101)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());

    const auto intent = WaitForIntent(
        harness.intent_subscription, [](const common::ControlIntent& value) {
            return value.type == common::ControlIntentType::kTakeoff;
        });
    ASSERT_TRUE(intent.has_value());
    EXPECT_FLOAT_EQ(intent->target_x, kTakeoffOriginX);
    EXPECT_FLOAT_EQ(intent->target_y, kTakeoffOriginY);
    EXPECT_FLOAT_EQ(intent->target_z, kTakeoffOriginZ);
    EXPECT_EQ(intent->header.frame_id, kFrameLocalNed);
    EXPECT_NE(intent->reason_code & common::kControlIntentReasonRequestControl,
              0U);
    EXPECT_GT(intent->valid_until_ms, intent->header.receive_time_ms);

    // bootstrap_lead 内不得提前发 176（进入 S3 约 100ms）。
    std::this_thread::sleep_for(100ms);
    EXPECT_EQ(harness.sender.CallCount(), 0U);

    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));
    auto history = harness.sender.History();
    ASSERT_EQ(history.size(), 1U);
    EXPECT_EQ(history[0].command, 176U);
    EXPECT_FLOAT_EQ(history[0].params[0], 1.0f);
    EXPECT_FLOAT_EQ(history[0].params[1], 6.0f);
    EXPECT_FLOAT_EQ(history[0].params[2], 0.0f);

    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(harness.sender.CallCount(), 1U) << "176 只发一次，不重发";
}

// 实际 6/0 之前不发 400；实际进入 Offboard 后恰好发一次 400（p1=1）；
// armed 且 Offboard → S4；到达任务高度后 → S5（availability=engaged）。
TEST(MissionStateMachineTest, TakeoffOffboardThenArmEntersS4AndReachesS5) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 2000ms, 2000ms,
                                                    4000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(102)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));

    // 未到 6/0：不得发 400。
    std::this_thread::sleep_for(150ms);
    EXPECT_EQ(harness.sender.CallCount(), 1U);

    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/false,
                                                 /*landed=*/true)));
    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));
    auto history = harness.sender.History();
    ASSERT_GE(history.size(), 2U);
    EXPECT_EQ(history[1].command, 400U);
    EXPECT_FLOAT_EQ(history[1].params[0], 1.0f);

    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(harness.sender.CallCount(), 2U) << "400 只发一次，不重发";

    // 实际 armed 且 Offboard → S4。
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/true,
                                                 /*landed=*/false)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kTakeoff)
                    .has_value());

    // 到达任务高度（z0 - height = -0.4，水平保持原点）→ S5。
    auto arrived = MakeOffboardFlight(/*armed=*/true, /*landed=*/false);
    arrived.local_z_m = kTakeoffOriginZ - 0.5f;
    arrived.vz_mps = 0.f;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, arrived));
    const auto approach = WaitForState(harness.status_subscription,
                                       common::MissionState::kGpsApproach,
                                       4000ms);
    ASSERT_TRUE(approach.has_value());
    EXPECT_EQ(approach->availability, common::MissionAvailability::kEngaged);
}

// 176 等待 Offboard 超时：65015 aborted mode_timeout、回 S2 READY、
// 不重发 176、不留残余起飞/位置意图；重复 65014 重发最新 aborted 结果。
TEST(MissionStateMachineTest, TakeoffModeTimeoutAbortsWithoutRetry) {
    AssignmentHarness harness(
        MakeTakeoffTestConfig(/*lead=*/100ms, /*mode=*/300ms,
                              /*arm=*/2000ms, /*height=*/3000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(103)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kCancelledOrAborted);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->assignment_id, 103U);
    EXPECT_EQ(aborted->target_id, 7U);
    EXPECT_EQ(aborted->reason, common::MissionAssignmentReason::kModeTimeout);
    EXPECT_EQ(aborted->mission_state, common::MissionState::kReady);
    EXPECT_EQ(aborted->availability, common::MissionAvailability::kReady);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
    EXPECT_EQ(harness.sender.CallCount(), 1U) << "失败不重发 176/400";

    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(harness.sender.CallCount(), 1U);

    // 中止后不得残留 takeoff/position 意图。
    DrainIntents(harness.intent_subscription);
    std::this_thread::sleep_for(250ms);
    EXPECT_FALSE(WaitForIntent(harness.intent_subscription,
                               [](const common::ControlIntent&) { return true; },
                               50ms)
                     .has_value())
        << "中止后不得继续发布控制意图";

    // 重复 assignment_id：重发最新结果（aborted，原因不变）。
    ASSERT_TRUE(harness.Send(MakeAssignment(103)));
    const auto duplicate = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kCancelledOrAborted);
    ASSERT_TRUE(duplicate.has_value());
    EXPECT_EQ(duplicate->assignment_id, 103U);
    EXPECT_EQ(duplicate->reason, common::MissionAssignmentReason::kModeTimeout);
}

// 400 等待实际 armed 超时：65015 aborted arm_timeout、回 READY；未 armed
// 不在地面发 DISARM；不重发 400。
TEST(MissionStateMachineTest, TakeoffArmTimeoutAbortsWithoutRetry) {
    AssignmentHarness harness(
        MakeTakeoffTestConfig(/*lead=*/100ms, /*mode=*/2000ms,
                              /*arm=*/300ms, /*height=*/3000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(104)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/false,
                                                 /*landed=*/true)));
    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kCancelledOrAborted);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason, common::MissionAssignmentReason::kArmTimeout);
    EXPECT_EQ(aborted->availability, common::MissionAvailability::kReady);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
    EXPECT_EQ(harness.sender.CallCount(), 2U);

    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(harness.sender.CallCount(), 2U) << "失败不重发 400";
    for (const auto& call : harness.sender.History()) {
        if (call.command == 400U) {
            EXPECT_FLOAT_EQ(call.params[0], 1.0f)
                << "未 armed 时地面中止不得发 DISARM";
        }
    }
}

// 地面中止时若 PX4 仍 armed+landed：只发一次 DISARM（param1=0），不重试。
TEST(MissionStateMachineTest, TakeoffGroundArmedAbortSendsDisarmOnce) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 2000ms, 2000ms,
                                                    3000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(105)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));

    // 地面 armed（未进入 Offboard），176 明确失败 → 中止。
    auto armed_ground = MakeGroundedFlight();
    armed_ground.armed = true;  // landed=true，模式保持 0/0
    ASSERT_TRUE(PublishFlight(harness.flight_topic, armed_ground));
    harness.sender.SimulateAck(176U, 4U, TestMonotonicMs());  // MAV_RESULT_FAILED

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kCancelledOrAborted);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason, common::MissionAssignmentReason::kModeTimeout);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);

    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));
    auto history = harness.sender.History();
    ASSERT_EQ(history.size(), 2U);
    EXPECT_EQ(history[1].command, 400U);
    EXPECT_FLOAT_EQ(history[1].params[0], 0.0f) << "地面中止发 DISARM";

    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(harness.sender.CallCount(), 2U) << "DISARM 只发一次";
}

// 400 ACK 已 accepted、armed 快照尚未刷新时地面取消：仍补发一次 DISARM，
// 避免飞机被留在 armed（SITL 2026-10-22 实测竞态）。
TEST(MissionStateMachineTest, TakeoffGroundCancelAfterArmAckSendsDisarm) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 2000ms, 2000ms,
                                                    3000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(150)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/false,
                                                 /*landed=*/true)));
    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));  // ARM 已入队
    harness.sender.SimulateAck(400U, 0U, TestMonotonicMs());  // ACCEPTED

    ASSERT_TRUE(harness.Send(MakeCancel(150)));
    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kCancelledOrAborted);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason,
              common::MissionAssignmentReason::kCancelledByGcs);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);

    ASSERT_TRUE(WaitForCallCount(harness.sender, 3));
    const auto history = harness.sender.History();
    ASSERT_FALSE(history.empty());
    EXPECT_EQ(history.back().command, 400U);
    EXPECT_FLOAT_EQ(history.back().params[0], 0.0f)
        << "ARM ACK accepted 后地面取消必须补发 DISARM";
}

// 已离地中止（PX4 断连）：不空中 disarm，tracker AbortAirborne → S12；
// 状态不确定 → availability=unavailable（不错误回到 ready）。
TEST(MissionStateMachineTest, TakeoffAirborneAbortReturnsHomeWithoutDisarm) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 2000ms, 2000ms,
                                                    4000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(106)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/false,
                                                 /*landed=*/true)));
    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/true,
                                                 /*landed=*/false)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kTakeoff)
                    .has_value());

    auto disconnected = MakeOffboardFlight(/*armed=*/true, /*landed=*/false);
    disconnected.connected = false;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, disconnected));

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kCancelledOrAborted);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason,
              common::MissionAssignmentReason::kPx4Disconnected);
    EXPECT_EQ(aborted->availability, common::MissionAvailability::kUnavailable);
    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kReturnHome)
                    .has_value());

    // 已离地：不得发 DISARM（历史里只有 176 与 ARM 400）。
    std::this_thread::sleep_for(400ms);
    for (const auto& call : harness.sender.History()) {
        if (call.command == 400U) {
            EXPECT_FLOAT_EQ(call.params[0], 1.0f) << "空中中止不得 disarm";
        }
    }
}

// S4 按 climb_rate 逐拍推进 z 目标（不阶跃）；到达判据满足并稳定后 S5。
TEST(MissionStateMachineTest, TakeoffClimbRampsZTargetAndReachesS5) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 2000ms, 2000ms,
                                                    5000ms, /*height=*/0.5,
                                                    /*climb=*/0.5));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(107)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/false,
                                                 /*landed=*/true)));
    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/true,
                                                 /*landed=*/false)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kTakeoff)
                    .has_value());

    const float goal_z = kTakeoffOriginZ - 0.5f;
    bool saw_intermediate = false;
    float last_z = kTakeoffOriginZ;
    const auto deadline =
        std::chrono::steady_clock::now() + 700ms;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = harness.intent_subscription.WaitTakeFor(50ms);
        if (!message) {
            continue;
        }
        const auto& value = **message;
        if (value.type != common::ControlIntentType::kTakeoff) {
            continue;
        }
        EXPECT_LE(value.target_z, last_z + 1e-4f)
            << "爬升目标不得反向/阶跃上升";
        if (value.target_z < kTakeoffOriginZ - 1e-3f &&
            value.target_z > goal_z + 1e-3f) {
            saw_intermediate = true;
        }
        last_z = value.target_z;
    }
    EXPECT_TRUE(saw_intermediate) << "z 目标必须逐拍推进而非一步到目标";

    auto arrived = MakeOffboardFlight(/*armed=*/true, /*landed=*/false);
    arrived.local_x_m = kTakeoffOriginX;
    arrived.local_y_m = kTakeoffOriginY;
    arrived.local_z_m = goal_z;
    arrived.vz_mps = 0.f;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, arrived));
    const auto approach = WaitForState(harness.status_subscription,
                                       common::MissionState::kGpsApproach,
                                       4000ms);
    ASSERT_TRUE(approach.has_value());
    EXPECT_EQ(approach->availability, common::MissionAvailability::kEngaged);
}

// S4 地面超时：停流；armed+landed 只发一次 DISARM；回 S2 READY。
TEST(MissionStateMachineTest, TakeoffClimbGroundTimeoutDisarmsAndReturnsReady) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 2000ms, 2000ms,
                                                    /*height_timeout=*/400ms,
                                                    /*height=*/5.0));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(108)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/false,
                                                 /*landed=*/true)));
    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));
    // 地面 armed + Offboard → 进入 S4；高度目标不可达 → 地面超时。
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/true,
                                                 /*landed=*/true)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kTakeoff)
                    .has_value());

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kCancelledOrAborted);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason,
              common::MissionAssignmentReason::kTakeoffTimeout);
    EXPECT_EQ(aborted->mission_state, common::MissionState::kReady);
    EXPECT_EQ(aborted->availability, common::MissionAvailability::kReady);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);

    ASSERT_TRUE(WaitForCallCount(harness.sender, 3));
    auto history = harness.sender.History();
    ASSERT_EQ(history.size(), 3U);
    EXPECT_EQ(history[2].command, 400U);
    EXPECT_FLOAT_EQ(history[2].params[0], 0.0f);
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(harness.sender.CallCount(), 3U) << "DISARM 只发一次";
}

// S4 空中超时且 PX4 仍在 Offboard：S13 由现有降落链发 AUTO.LAND(4/6)。
TEST(MissionStateMachineTest, TakeoffClimbAirTimeoutLandsInPlace) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 2000ms, 2000ms,
                                                    /*height_timeout=*/400ms,
                                                    /*height=*/5.0));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(109)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/false,
                                                 /*landed=*/true)));
    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));
    // 空中 armed + Offboard → S4；高度目标不可达 → 空中超时。
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/true,
                                                 /*landed=*/false)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kTakeoff)
                    .has_value());

    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kLanding, 3000ms)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 3));
    auto history = harness.sender.History();
    ASSERT_EQ(history.size(), 3U);
    EXPECT_EQ(history[2].command, 176U);
    EXPECT_FLOAT_EQ(history[2].params[1], 4.0f);  // AUTO
    EXPECT_FLOAT_EQ(history[2].params[2], 6.0f);  // AUTO.LAND

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kCancelledOrAborted);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason,
              common::MissionAssignmentReason::kTakeoffTimeout);
}

// S4 爬升中 PX4 离开 Offboard（非等到 height_timeout）：当拍停发让权，
// 转 S13 仅监控，不发 AUTO.LAND，65015 aborted(mode_timeout)。
TEST(MissionStateMachineTest, TakeoffClimbOffboardLossYieldsImmediately) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 2000ms, 2000ms,
                                                    /*height_timeout=*/3000ms,
                                                    /*height=*/5.0));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(111)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/false,
                                                 /*landed=*/true)));
    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/true,
                                                 /*landed=*/false)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kTakeoff)
                    .has_value());

    // PX4 人工/failsafe 接管：离开 Offboard，但仍在空中。
    auto takeover = MakeOffboardFlight(/*armed=*/true, /*landed=*/false);
    takeover.flight_mode = 4;
    takeover.flight_sub_mode = 5;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, takeover));

    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kLanding, 1500ms)
                    .has_value());
    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kCancelledOrAborted);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason,
              common::MissionAssignmentReason::kModeTimeout);
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(harness.sender.CallCount(), 2U)
        << "离开 Offboard 后不得争控发 AUTO.LAND";
}

// S4 空中超时且 PX4 已离开 Offboard：停发让权，转 S13 仅监控，不发 AUTO.LAND。
TEST(MissionStateMachineTest, TakeoffClimbAirTimeoutYieldsWhenOffboardLost) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 2000ms, 2000ms,
                                                    /*height_timeout=*/400ms,
                                                    /*height=*/5.0));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(110)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/false,
                                                 /*landed=*/true)));
    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/true,
                                                 /*landed=*/false)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kTakeoff)
                    .has_value());

    // PX4 已被接管（4/5），S4 超时后只监控。
    auto takeover = MakeOffboardFlight(/*armed=*/true, /*landed=*/false);
    takeover.flight_mode = 4;
    takeover.flight_sub_mode = 5;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, takeover));

    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kLanding, 3000ms)
                    .has_value());
    std::this_thread::sleep_for(500ms);
    EXPECT_EQ(harness.sender.CallCount(), 2U)
        << "PX4 已离开 Offboard 时不得争控发 AUTO.LAND";
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kLanding);
}

// 中止时离地状态不明（landed_state_valid=false）：即使回到 S2 也锁存
// unavailable，新分配回 not_ready，不错误回到 ready。
TEST(MissionStateMachineTest, TakeoffUncertainStateLatchesUnavailable) {
    AssignmentHarness harness(
        MakeTakeoffTestConfig(/*lead=*/100ms, /*mode=*/300ms,
                              /*arm=*/2000ms, /*height=*/3000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(112)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));

    // 离地状态不明（未 armed、landed_state_valid=false）→ 模式超时中止后
    // 不可判定是否仍在地面，置 per-boot 不可调度标志。
    auto uncertain = MakeGroundedFlight();
    uncertain.landed_state_valid = false;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, uncertain));

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kCancelledOrAborted);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason, common::MissionAssignmentReason::kModeTimeout);
    EXPECT_EQ(aborted->mission_state, common::MissionState::kReady);
    EXPECT_EQ(aborted->availability, common::MissionAvailability::kUnavailable);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
    EXPECT_EQ(harness.sender.CallCount(), 1U) << "状态不明不得发 DISARM";

    // 新分配被拒 not_ready（不是 ready），availability 仍 unavailable。
    ASSERT_TRUE(harness.Send(MakeAssignment(113)));
    const auto rejected = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kRejected);
    ASSERT_TRUE(rejected.has_value());
    EXPECT_EQ(rejected->reason, common::MissionAssignmentReason::kNotReady);
    EXPECT_EQ(rejected->availability, common::MissionAvailability::kUnavailable);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
    EXPECT_EQ(harness.sender.CallCount(), 1U);
}

// 176 ACK IN_PROGRESS(5) 不中止；明确失败(4) 立即中止（不等 mode_timeout）。
TEST(MissionStateMachineTest, TakeoffAckFailureAbortsImmediately) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 3000ms, 3000ms,
                                                    3000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(111)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    const auto accepted = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kAccepted);
    ASSERT_TRUE(accepted.has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));

    // IN_PROGRESS 只作过程反馈，不中止。
    harness.sender.SimulateAck(176U, 5U, TestMonotonicMs());
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kArming);
    EXPECT_FALSE(WaitForAckWithResult(harness.ack_subscription,
                                      common::MissionAssignmentResult::
                                          kCancelledOrAborted,
                                      100ms)
                     .has_value());

    // 明确失败立即中止（mode_timeout 原因、无重发）。
    harness.sender.SimulateAck(176U, 4U, TestMonotonicMs());
    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription, common::MissionAssignmentResult::kCancelledOrAborted);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason, common::MissionAssignmentReason::kModeTimeout);
    EXPECT_EQ(harness.sender.CallCount(), 1U);
}

// ---------------------------------------------------------------------------
// 电池守卫：S2 预检、地面 availability、飞行中低电/遥测失效 S12（2026-10-22，Task 8）
// ---------------------------------------------------------------------------

// PX4 低电参数不可用（阈值/动作无效、动作非 0、阈值 0 或 100）→ S2 拒绝
// battery_config_unknown，保持 READY。
TEST(MissionStateMachineTest, BatteryPreflightRejectsInvalidParameters) {
    AssignmentHarness harness;
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));

    struct Case {
        bool threshold_valid;
        float threshold_pct;
        bool action_valid;
        uint8_t action;
    };
    const Case cases[] = {
        {false, 15.f, true, 0},   // BAT_LOW_THR 未读到
        {true, 15.f, false, 0},   // COM_LOW_BAT_ACT 未读到
        {true, 15.f, true, 1},    // 动作非 0（非 Warning Only）
        {true, 0.f, true, 0},     // 阈值越界（0）
        {true, 100.f, true, 0},   // 阈值越界（100）
    };
    uint32_t assignment_id = 201;
    for (const auto& item : cases) {
        auto flight = MakeGroundedFlight();
        flight.battery_low_threshold_valid = item.threshold_valid;
        flight.battery_low_threshold_pct = item.threshold_pct;
        flight.low_battery_action_valid = item.action_valid;
        flight.low_battery_action = item.action;
        ASSERT_TRUE(PublishFlight(harness.flight_topic, flight));
        std::this_thread::sleep_for(150ms);

        ASSERT_TRUE(harness.Send(MakeAssignment(assignment_id++)));
        const auto ack = WaitForAssignmentAck(harness.ack_subscription);
        ASSERT_TRUE(ack.has_value());
        EXPECT_EQ(ack->result, common::MissionAssignmentResult::kRejected);
        EXPECT_EQ(ack->reason,
                  common::MissionAssignmentReason::kBatteryConfigUnknown);
        EXPECT_EQ(ack->availability,
                  common::MissionAvailability::kUnavailable);
        EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
    }
}

// 电池遥测无效（battery_valid=false / remaining_valid=false / 百分比非有限）
// → S2 拒绝 battery_telemetry_invalid。
TEST(MissionStateMachineTest, BatteryPreflightRejectsInvalidTelemetry) {
    AssignmentHarness harness;
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));

    std::vector<common::FlightStateSnapshot> cases;
    {
        auto flight = MakeGroundedFlight();
        flight.battery_valid = false;
        cases.push_back(flight);
    }
    {
        auto flight = MakeGroundedFlight();
        flight.battery_remaining_valid = false;
        cases.push_back(flight);
    }
    {
        auto flight = MakeGroundedFlight();
        flight.battery_remaining_pct =
            std::numeric_limits<float>::quiet_NaN();
        cases.push_back(flight);
    }
    uint32_t assignment_id = 211;
    for (const auto& flight : cases) {
        ASSERT_TRUE(PublishFlight(harness.flight_topic, flight));
        std::this_thread::sleep_for(150ms);
        ASSERT_TRUE(harness.Send(MakeAssignment(assignment_id++)));
        const auto ack = WaitForAssignmentAck(harness.ack_subscription);
        ASSERT_TRUE(ack.has_value());
        EXPECT_EQ(ack->result, common::MissionAssignmentResult::kRejected);
        EXPECT_EQ(ack->reason,
                  common::MissionAssignmentReason::kBatteryTelemetryInvalid);
        EXPECT_EQ(ack->availability,
                  common::MissionAvailability::kUnavailable);
        EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
    }
}

// 地面电量不高于阈值（pct == BAT_LOW_THR 也算低电）→ rejected battery_low、
// availability=unavailable；恢复且未执行空中任务后自然回到 ready，可接受
// 新分配（不锁存为 terminal）。
TEST(MissionStateMachineTest, BatteryLowOnGroundUnavailableThenRecovers) {
    AssignmentHarness harness;
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));

    auto low = MakeGroundedFlight();
    low.battery_remaining_pct = 15.f;  // == BAT_LOW_THR，pct <= 阈值即低电
    ASSERT_TRUE(PublishFlight(harness.flight_topic, low));
    std::this_thread::sleep_for(150ms);

    ASSERT_TRUE(harness.Send(MakeAssignment(221)));
    const auto rejected = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(rejected.has_value());
    EXPECT_EQ(rejected->result, common::MissionAssignmentResult::kRejected);
    EXPECT_EQ(rejected->reason, common::MissionAssignmentReason::kBatteryLow);
    EXPECT_EQ(rejected->availability,
              common::MissionAvailability::kUnavailable);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
    EXPECT_EQ(harness.sender.CallCount(), 0U);

    // 地面恢复（未执行空中任务）：availability 自然回 ready，可接受新分配。
    ASSERT_TRUE(PublishFlight(harness.flight_topic, MakeGroundedFlight()));
    std::this_thread::sleep_for(150ms);
    ASSERT_TRUE(harness.Send(MakeAssignment(222)));
    const auto accepted = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(accepted.has_value());
    EXPECT_EQ(accepted->result, common::MissionAssignmentResult::kAccepted);
    EXPECT_EQ(accepted->availability,
              common::MissionAvailability::kAssigned);
    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
}

// S3 地面低电（持续防抖）→ 中止，65015 aborted battery_low，回 READY；
// 不进入 S9/S10 发射（无 187），也不发 176/400（bootstrap_lead 未到）。
TEST(MissionStateMachineTest, BatteryLowInArmingAbortsOnGround) {
    auto config = MakeTakeoffTestConfig(/*lead=*/5000ms, 5000ms, 5000ms);
    config.takeoff.battery_low_debounce = 200ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(231)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());

    auto low = MakeGroundedFlight();
    low.battery_remaining_pct = 10.f;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, low));

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 3000ms);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason, common::MissionAssignmentReason::kBatteryLow);
    EXPECT_EQ(aborted->mission_state, common::MissionState::kReady);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
    EXPECT_EQ(harness.sender.CallCount(), 0U)
        << "低电中止不得再发 176/400";
    for (const auto& call : harness.sender.History()) {
        EXPECT_NE(call.command, 187U) << "低电不得触发发射";
    }
}

// S4 已离地低电（持续防抖）→ 65015 aborted battery_low → S12 机载位置返航，
// 不空中 disarm、不走 PX4 RTL、不触发 187。
TEST(MissionStateMachineTest, BatteryLowAirborneInTakeoffReturnsHome) {
    auto config = MakeTakeoffTestConfig(100ms, 2000ms, 2000ms, 4000ms);
    config.takeoff.battery_low_debounce = 200ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(241)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/false,
                                                 /*landed=*/true)));
    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/true,
                                                 /*landed=*/false)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kTakeoff)
                    .has_value());

    auto low = MakeOffboardFlight(/*armed=*/true, /*landed=*/false);
    low.battery_remaining_pct = 10.f;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, low));

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 3000ms);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason, common::MissionAssignmentReason::kBatteryLow);
    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kReturnHome)
                    .has_value());

    // 空中中止不 disarm（400 只有 param1=1 的解锁命令），不触发 187。
    std::this_thread::sleep_for(300ms);
    for (const auto& call : harness.sender.History()) {
        if (call.command == 400U) {
            EXPECT_FLOAT_EQ(call.params[0], 1.0f) << "空中中止不得 disarm";
        }
        EXPECT_NE(call.command, 187U) << "低电不得触发发射";
    }
}

// S5 低电：不足防抖不触发；持续超过防抖 → 65015 aborted battery_low → S12；
// 之后不再重复 aborted（一次任务只中止一次），且无 187。
TEST(MissionStateMachineTest, BatteryLowInGpsApproachDebouncesThenReturnsHome) {
    auto config = MakeTakeoffTestConfig(100ms, 2000ms, 2000ms, 5000ms);
    config.takeoff.battery_low_debounce = 800ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 251));
    ASSERT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach);

    auto low = MakeOffboardFlight(/*armed=*/true, /*landed=*/false);
    low.battery_remaining_pct = 10.f;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, low));
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach)
        << "不足 battery_low_debounce 不得中止";

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 3000ms);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason, common::MissionAssignmentReason::kBatteryLow);
    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kReturnHome)
                    .has_value());

    // S12 起不再重复触发/重复 65015。
    DrainAcks(harness.ack_subscription);
    ASSERT_TRUE(PublishFlight(harness.flight_topic, low));
    std::this_thread::sleep_for(500ms);
    EXPECT_FALSE(WaitForAckWithResult(
                     harness.ack_subscription,
                     common::MissionAssignmentResult::kCancelledOrAborted,
                     100ms)
                     .has_value())
        << "S12 不得重复 aborted";
    for (const auto& call : harness.sender.History()) {
        EXPECT_NE(call.command, 187U) << "低电不得触发发射";
        if (call.command == 400U) {
            EXPECT_FLOAT_EQ(call.params[0], 1.0f) << "空中中止不得 disarm";
        }
    }
}

// S5 电池遥测失效 → 立即 65015 aborted battery_telemetry_invalid → S12，
// 不等防抖、不触发 187。
TEST(MissionStateMachineTest, BatteryTelemetryLossInGpsApproachReturnsImmediately) {
    auto config = MakeTakeoffTestConfig(100ms, 2000ms, 2000ms, 5000ms);
    config.takeoff.battery_low_debounce = 5000ms;  // 遥测失效不等防抖
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 261));
    ASSERT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach);

    auto lost = MakeOffboardFlight(/*armed=*/true, /*landed=*/false);
    lost.battery_valid = false;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, lost));

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 1500ms);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason,
              common::MissionAssignmentReason::kBatteryTelemetryInvalid);
    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kReturnHome)
                    .has_value());
    for (const auto& call : harness.sender.History()) {
        EXPECT_NE(call.command, 187U) << "遥测失效不得触发发射";
    }
}

// 电池空中中止后落地：S12→S13→S14 沿用现有链路，tracker 进入 terminal，
// 65013 availability=terminal，本次上电不再分配（新 65014 被拒）。
TEST(MissionStateMachineTest, BatteryAbortLandsThenTerminal) {
    auto config = MakeTakeoffTestConfig(100ms, 2000ms, 2000ms, 5000ms);
    config.takeoff.battery_low_debounce = 200ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 271));

    auto low = MakeOffboardFlight(/*armed=*/true, /*landed=*/false);
    low.battery_remaining_pct = 10.f;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, low));
    ASSERT_TRUE(WaitForAckWithResult(
                    harness.ack_subscription,
                    common::MissionAssignmentResult::kCancelledOrAborted, 3000ms)
                    .has_value());
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kReturnHome)
                    .has_value());

    // PX4 接管（离开 Offboard）→ S13；落地且 disarmed → S14。
    auto takeover = MakeOffboardFlight(/*armed=*/true, /*landed=*/false);
    takeover.flight_mode = 4;
    takeover.flight_sub_mode = 5;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, takeover));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kLanding)
                    .has_value());

    auto landed = MakeOffboardFlight(/*armed=*/false, /*landed=*/true);
    landed.flight_mode = 4;
    landed.flight_sub_mode = 5;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, landed));
    const auto disarmed = WaitForState(harness.status_subscription,
                                       common::MissionState::kDisarmed,
                                       2000ms);
    ASSERT_TRUE(disarmed.has_value());
    EXPECT_EQ(disarmed->availability,
              common::MissionAvailability::kTerminal);

    // terminal：新分配不再接受（状态非 READY 与 tracker terminal 双重拒绝）。
    ASSERT_TRUE(harness.Send(MakeAssignment(272)));
    const auto rejected = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(rejected.has_value());
    EXPECT_EQ(rejected->result, common::MissionAssignmentResult::kRejected);
    EXPECT_EQ(rejected->reason,
              common::MissionAssignmentReason::kAlreadyAssigned);
    EXPECT_EQ(rejected->availability,
              common::MissionAvailability::kTerminal);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kDisarmed);
}

// ---------------------------------------------------------------------------
// 65014 取消、S5 target_id 过滤与目标等待/丢失收口（2026-10-22，Task 10）
// ---------------------------------------------------------------------------

// 地面 S3（未 armed）取消：65015 cancelled(cancelled_by_gcs)、回 READY、
// 不产生 176/400（bootstrap_lead 未到）、无 187；重复取消重发同一结果。
TEST(MissionStateMachineTest, CancelOnGroundInArmingReturnsReady) {
    AssignmentHarness harness(MakeTakeoffTestConfig(/*lead=*/5000ms, 5000ms,
                                                    5000ms, 5000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(301)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());

    ASSERT_TRUE(harness.Send(MakeCancel(301)));
    const auto cancelled = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 2000ms);
    ASSERT_TRUE(cancelled.has_value());
    EXPECT_EQ(cancelled->assignment_id, 301U);
    EXPECT_EQ(cancelled->target_id, 7U);
    EXPECT_EQ(cancelled->reason,
              common::MissionAssignmentReason::kCancelledByGcs);
    EXPECT_EQ(cancelled->mission_state, common::MissionState::kReady);
    EXPECT_EQ(cancelled->availability, common::MissionAvailability::kReady);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
    EXPECT_EQ(harness.sender.CallCount(), 0U)
        << "bootstrap_lead 内地面取消不得发 176/400";
    for (const auto& call : harness.sender.History()) {
        EXPECT_NE(call.command, 187U);
    }

    // 重复取消：同一 assignment_id 重发最近 cancelled 结果，不再迁移/发命令。
    DrainAcks(harness.ack_subscription);
    ASSERT_TRUE(harness.Send(MakeCancel(301)));
    const auto duplicate = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 2000ms);
    ASSERT_TRUE(duplicate.has_value());
    EXPECT_EQ(duplicate->assignment_id, 301U);
    EXPECT_EQ(duplicate->reason,
              common::MissionAssignmentReason::kCancelledByGcs);
    EXPECT_EQ(duplicate->availability, common::MissionAvailability::kReady);
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(harness.sender.CallCount(), 0U);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
}

// 地面取消且 PX4 仍 armed+landed：只发一次 DISARM(param1=0)，不重试；
// 重复取消不重复下发命令。
TEST(MissionStateMachineTest, CancelGroundArmedSendsDisarmOnce) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 3000ms, 3000ms,
                                                    3000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(302)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));  // 176

    // 地面 armed（landed=true，模式仍 0/0）→ 取消按地面中止处置。
    auto armed_ground = MakeGroundedFlight();
    armed_ground.armed = true;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, armed_ground));

    ASSERT_TRUE(harness.Send(MakeCancel(302)));
    const auto cancelled = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 2000ms);
    ASSERT_TRUE(cancelled.has_value());
    EXPECT_EQ(cancelled->reason,
              common::MissionAssignmentReason::kCancelledByGcs);
    EXPECT_EQ(cancelled->mission_state, common::MissionState::kReady);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);

    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));
    auto history = harness.sender.History();
    ASSERT_GE(history.size(), 2U);
    EXPECT_EQ(history[1].command, 400U);
    EXPECT_FLOAT_EQ(history[1].params[0], 0.0f) << "地面取消发 DISARM";

    // 重复取消：只重发结果，不再发 DISARM。
    DrainAcks(harness.ack_subscription);
    ASSERT_TRUE(harness.Send(MakeCancel(302)));
    ASSERT_TRUE(WaitForAckWithResult(harness.ack_subscription,
                                     common::MissionAssignmentResult::
                                         kCancelledOrAborted,
                                     2000ms)
                    .has_value());
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(harness.sender.CallCount(), 2U) << "DISARM 只发一次";
}

// 已离地（S4）取消：tracker AbortAirborne → S12，65015 cancelled；不空中
// disarm、不触发 187。
TEST(MissionStateMachineTest, CancelAirborneInTakeoffReturnsHomeWithoutDisarm) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 2000ms, 2000ms,
                                                    5000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(harness.Send(MakeAssignment(303)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kArming)
                    .has_value());
    ASSERT_TRUE(WaitForCallCount(harness.sender, 1));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/false,
                                                 /*landed=*/true)));
    ASSERT_TRUE(WaitForCallCount(harness.sender, 2));
    ASSERT_TRUE(PublishFlight(harness.flight_topic,
                              MakeOffboardFlight(/*armed=*/true,
                                                 /*landed=*/false)));
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kTakeoff)
                    .has_value());

    ASSERT_TRUE(harness.Send(MakeCancel(303)));
    const auto cancelled = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 2000ms);
    ASSERT_TRUE(cancelled.has_value());
    EXPECT_EQ(cancelled->assignment_id, 303U);
    EXPECT_EQ(cancelled->reason,
              common::MissionAssignmentReason::kCancelledByGcs);
    EXPECT_EQ(cancelled->mission_state, common::MissionState::kReturnHome);
    EXPECT_EQ(cancelled->availability,
              common::MissionAvailability::kReturning);
    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kReturnHome)
                    .has_value());

    std::this_thread::sleep_for(300ms);
    for (const auto& call : harness.sender.History()) {
        if (call.command == 400U) {
            EXPECT_FLOAT_EQ(call.params[0], 1.0f) << "空中取消不得 disarm";
        }
        EXPECT_NE(call.command, 187U) << "取消不得触发发射";
    }
}

// 已离地（S5）取消：中止分配 → S12；停发任务位置引导，无 disarm/187。
TEST(MissionStateMachineTest, CancelAirborneInGpsApproachReturnsHome) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 2000ms, 2000ms,
                                                    5000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 304));
    ASSERT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach);

    ASSERT_TRUE(harness.Send(MakeCancel(304)));
    const auto cancelled = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 2000ms);
    ASSERT_TRUE(cancelled.has_value());
    EXPECT_EQ(cancelled->reason,
              common::MissionAssignmentReason::kCancelledByGcs);
    EXPECT_EQ(cancelled->mission_state, common::MissionState::kReturnHome);
    EXPECT_EQ(cancelled->availability,
              common::MissionAvailability::kReturning);
    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kReturnHome)
                    .has_value());

    std::this_thread::sleep_for(300ms);
    for (const auto& call : harness.sender.History()) {
        if (call.command == 400U) {
            EXPECT_FLOAT_EQ(call.params[0], 1.0f) << "空中取消不得 disarm";
        }
        EXPECT_NE(call.command, 187U) << "取消不得触发发射";
    }
}

// 无活动分配且 assignment_id 从未处理：取消回 rejected(not_ready)，保持 S2。
TEST(MissionStateMachineTest, CancelWithoutActiveAssignmentRejected) {
    AssignmentHarness harness;
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));

    ASSERT_TRUE(harness.Send(MakeCancel(401)));
    const auto rejected = WaitForAssignmentAck(harness.ack_subscription);
    ASSERT_TRUE(rejected.has_value());
    EXPECT_EQ(rejected->assignment_id, 401U);
    EXPECT_EQ(rejected->result, common::MissionAssignmentResult::kRejected);
    EXPECT_EQ(rejected->reason, common::MissionAssignmentReason::kNotReady);
    EXPECT_EQ(rejected->mission_state, common::MissionState::kReady);
    EXPECT_EQ(harness.machine.CurrentState(), common::MissionState::kReady);
    EXPECT_EQ(harness.sender.CallCount(), 0U);
}

// 控制任务 S5：非本分配 target_id 的 65010/估计不得更新位置引导，也不进入
// 交接；匹配 target_id 后才发布位置意图。
TEST(MissionStateMachineTest, S5TargetIdFilterIgnoresNonMatchingTargets) {
    AssignmentHarness harness(MakeTakeoffTestConfig(100ms, 2000ms, 2000ms,
                                                    5000ms));
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 305));
    ASSERT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach);
    DrainIntents(harness.intent_subscription);

    // 非匹配 65010（target_id=99）与估计：不得进入引导（不能飞向别人的目标）；
    // 控制任务下目标不可用按“原地保持”续流（原因码 9），不是停发。
    auto foreign_ground = MakeFreshGroundTarget();
    foreign_ground.target_id = 99;
    ASSERT_TRUE(harness.ground_topic
                    .Publish(std::make_shared<const common::GroundStationTarget>(
                        foreign_ground))
                    .accepted);
    auto foreign_estimate = MakeValidEstimatedTarget();
    foreign_estimate.target_id = 99;
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        foreign_estimate))
                    .accepted);
    const auto hold = WaitForIntent(
        harness.intent_subscription, [](const common::ControlIntent& value) {
            return value.type == common::ControlIntentType::kPositionGuide &&
                   (value.reason_code & 0xFFU) == 9U;
        });
    ASSERT_TRUE(hold.has_value())
        << "非匹配 target_id 不得进入引导，但应按原地保持续流";
    EXPECT_FLOAT_EQ(hold->target_x, kTakeoffOriginX);
    EXPECT_FLOAT_EQ(hold->target_y, kTakeoffOriginY);
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach);

    // 匹配 target_id=7 的估计 → 连续 3 拍恢复后回到 S5 位置引导（原因码 7）。
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeValidEstimatedTarget()))
                    .accepted);
    const auto intent = WaitForIntent(
        harness.intent_subscription, [](const common::ControlIntent& value) {
            return value.type == common::ControlIntentType::kPositionGuide &&
                   (value.reason_code & 0xFFU) == 7U;
        });
    ASSERT_TRUE(intent.has_value());
    EXPECT_FLOAT_EQ(intent->target_x, 12.5f);
    EXPECT_FLOAT_EQ(intent->target_y, -3.0f);
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach);
}

// 控制任务 S5 无匹配目标：超过 target_wait_timeout → 65015
// aborted(target_wait_timeout)，tracker 转 returning，S12 返航；无 187。
TEST(MissionStateMachineTest, S5TargetWaitTimeoutReturnsHome) {
    auto config = MakeTakeoffTestConfig(100ms, 2000ms, 2000ms, 5000ms);
    config.takeoff.target_wait_timeout = 300ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 306));

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 3000ms);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->assignment_id, 306U);
    EXPECT_EQ(aborted->reason,
              common::MissionAssignmentReason::kTargetWaitTimeout);
    EXPECT_EQ(aborted->mission_state, common::MissionState::kReturnHome);
    EXPECT_EQ(aborted->availability,
              common::MissionAvailability::kReturning);
    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kReturnHome)
                    .has_value());
    for (const auto& call : harness.sender.History()) {
        EXPECT_NE(call.command, 187U) << "目标等待超时不得触发发射";
        if (call.command == 400U) {
            EXPECT_FLOAT_EQ(call.params[0], 1.0f) << "空中中止不得 disarm";
        }
    }

    // 一次任务只中止一次：S12 后不再重复 65015 aborted。
    DrainAcks(harness.ack_subscription);
    std::this_thread::sleep_for(500ms);
    EXPECT_FALSE(WaitForAckWithResult(
                     harness.ack_subscription,
                     common::MissionAssignmentResult::kCancelledOrAborted,
                     100ms)
                     .has_value())
        << "S12 不得重复 aborted";
}

// 控制任务 S5：已获得匹配目标后失效，短暂丢失不触发；恢复后计时清零；
// 再次持续失效超过 target_lost_timeout → 65015 aborted(target_lost) + S12。
TEST(MissionStateMachineTest, S5TargetLostTimeoutWithRecovery) {
    auto config = MakeTakeoffTestConfig(100ms, 2000ms, 2000ms, 5000ms);
    config.takeoff.target_wait_timeout = 5000ms;
    config.takeoff.target_lost_timeout = 400ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 307));

    // 先获得匹配目标（位置意图出现即说明已进入引导）。
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeValidEstimatedTarget()))
                    .accepted);
    ASSERT_TRUE(WaitForIntent(
                    harness.intent_subscription,
                    [](const common::ControlIntent& value) {
                        return value.type ==
                               common::ControlIntentType::kPositionGuide;
                    })
                    .has_value());

    // 短暂丢失（200ms < 400ms）后恢复：不得中止，丢失计时清零。
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeInvalidEstimatedTarget()))
                    .accepted);
    std::this_thread::sleep_for(200ms);
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeValidEstimatedTarget()))
                    .accepted);
    std::this_thread::sleep_for(500ms);
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach)
        << "短暂丢失并恢复不得中止任务";

    // 再次持续失效超过 400ms → 中止返航。
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeInvalidEstimatedTarget()))
                    .accepted);
    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 3000ms);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason, common::MissionAssignmentReason::kTargetLost);
    EXPECT_EQ(aborted->availability,
              common::MissionAvailability::kReturning);
    EXPECT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kReturnHome)
                    .has_value());
    for (const auto& call : harness.sender.History()) {
        EXPECT_NE(call.command, 187U) << "目标丢失中止不得触发发射";
    }
}

// 影子兼容：未注入命令接口时 S5 保持既有行为——不按 target_id 过滤，
// target_wait_timeout 不生效（不自动中止）。
TEST(MissionStateMachineTest, ShadowS5IgnoresFilterAndWaitTimeout) {
    auto config = MakeAssignmentHarnessConfig();
    config.takeoff.target_wait_timeout = 200ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/false,
                              /*with_estimation=*/true));

    ASSERT_TRUE(harness.ground_topic
                    .Publish(std::make_shared<const common::GroundStationTarget>(
                        MakeFreshGroundTarget()))
                    .accepted);
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kGpsApproach)
                    .has_value());
    DrainIntents(harness.intent_subscription);

    // 影子不过滤：target_id=99 的估计仍发布位置意图（既有行为）。
    auto foreign_estimate = MakeValidEstimatedTarget();
    foreign_estimate.target_id = 99;
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        foreign_estimate))
                    .accepted);
    EXPECT_TRUE(WaitForIntent(
                    harness.intent_subscription,
                    [](const common::ControlIntent& value) {
                        return value.type ==
                               common::ControlIntentType::kPositionGuide;
                    })
                    .has_value())
        << "影子模式不得按 target_id 过滤";

    // 影子不启用目标看门狗：超过 200ms 仍停在 S5。
    std::this_thread::sleep_for(500ms);
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach);
}

// ---------------------------------------------------------------------------
// S5 目标失效“原地保持”（2026-10-22）
// 目标失效不再直接停发：控制任务以 PX4 当前局部位置续流（原因码 9、无前馈、
// 10Hz），保证控制器与 PX4 Offboard 不断流，10s 看门狗到点后由 S12 无缝接管。
// ---------------------------------------------------------------------------

namespace {

bool IsPositionIntent(const common::ControlIntent& value) {
    return value.type == common::ControlIntentType::kPositionGuide;
}

/// 低位原因码（bit31 为控制权请求位，需剥离）。
uint32_t LowReasonCode(const common::ControlIntent& value) {
    return value.reason_code & 0xFFU;
}

constexpr uint32_t kLowReasonGroundTask = 7U;
constexpr uint32_t kLowReasonTargetHold = 9U;

}  // namespace

// 目标失效：保持意图以 PX4 当前位置为目标（绝不用过期目标）、无前馈、
// 10Hz 续流，65013 置 bit5；保持期间不中止任务。
TEST(MissionStateMachineTest, S5TargetLossHoldsCurrentPosition) {
    auto config = MakeTakeoffTestConfig(100ms, 2000ms, 2000ms, 5000ms);
    config.takeoff.target_wait_timeout = 3000ms;
    config.takeoff.target_lost_timeout = 3000ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 308));

    // 先获得匹配目标：估计引导（原因码 7）。
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeValidEstimatedTarget()))
                    .accepted);
    const auto guided = WaitForIntent(
        harness.intent_subscription, [](const common::ControlIntent& value) {
            return IsPositionIntent(value) &&
                   LowReasonCode(value) == kLowReasonGroundTask;
        });
    ASSERT_TRUE(guided.has_value());
    DrainIntents(harness.intent_subscription);

    // 目标失效：保持意图（原因码 9），目标=PX4 当前局部位置、无前馈。
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeInvalidEstimatedTarget()))
                    .accepted);
    const auto hold = WaitForIntent(
        harness.intent_subscription, [](const common::ControlIntent& value) {
            return IsPositionIntent(value) &&
                   LowReasonCode(value) == kLowReasonTargetHold;
        });
    ASSERT_TRUE(hold.has_value());
    EXPECT_EQ(hold->header.frame_id, kFrameLocalNed);
    EXPECT_FLOAT_EQ(hold->target_x, kTakeoffOriginX);
    EXPECT_FLOAT_EQ(hold->target_y, kTakeoffOriginY);
    EXPECT_FLOAT_EQ(hold->target_z, kTakeoffOriginZ - 0.5f);
    EXPECT_EQ(hold->feedforward_valid_mask, 0U)
        << "保持意图不得携带目标速度前馈";
    EXPECT_NE(hold->reason_code & common::kControlIntentReasonRequestControl, 0U)
        << "保持意图必须继续携带控制权请求位";

    // 10Hz 续流：500ms 内应出现多条保持意图（而不是停发）。
    DrainIntents(harness.intent_subscription);
    std::this_thread::sleep_for(500ms);
    int hold_count = 0;
    while (auto message = harness.intent_subscription.TryTake()) {
        if (IsPositionIntent(**message) &&
            LowReasonCode(**message) == kLowReasonTargetHold) {
            ++hold_count;
        }
    }
    EXPECT_GE(hold_count, 3) << "目标失效期间必须持续发送保持意图";
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach)
        << "保持期间不得中止任务";

    // 65013 告警位 bit5：目标保持中（状态周期 500ms，留 1.5s）。
    bool hold_warning_seen = false;
    const auto deadline = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < deadline) {
        auto status = harness.status_subscription.WaitTakeFor(50ms);
        if (status && ((*status)->active_warning_bits & (1U << 5U)) != 0U) {
            hold_warning_seen = true;
            break;
        }
    }
    EXPECT_TRUE(hold_warning_seen) << "65013 应置位目标保持告警 bit5";
}

// 恢复迟滞：不足 3 拍的新鲜估计不退出保持；持续新鲜到第 3 拍才回到引导。
TEST(MissionStateMachineTest, S5HoldReleasesAfterThreeFreshFrames) {
    auto config = MakeTakeoffTestConfig(100ms, 2000ms, 2000ms, 5000ms);
    config.takeoff.target_wait_timeout = 4000ms;
    config.takeoff.target_lost_timeout = 4000ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 309));

    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeValidEstimatedTarget()))
                    .accepted);
    ASSERT_TRUE(WaitForIntent(
                    harness.intent_subscription,
                    [](const common::ControlIntent& value) {
                        return IsPositionIntent(value) &&
                               LowReasonCode(value) == kLowReasonGroundTask;
                    })
                    .has_value());
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeInvalidEstimatedTarget()))
                    .accepted);
    ASSERT_TRUE(WaitForIntent(
                    harness.intent_subscription,
                    [](const common::ControlIntent& value) {
                        return IsPositionIntent(value) &&
                               LowReasonCode(value) == kLowReasonTargetHold;
                    })
                    .has_value());
    DrainIntents(harness.intent_subscription);

    // 只新鲜 150ms（< 3 个决策拍）：仍保持，不得立刻回到引导。
    auto brief = MakeValidEstimatedTarget();
    brief.header.valid_for_ms = 150;
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(brief))
                    .accepted);
    std::this_thread::sleep_for(250ms);
    int guided_during_brief = 0;
    while (auto message = harness.intent_subscription.TryTake()) {
        if (IsPositionIntent(**message) &&
            LowReasonCode(**message) == kLowReasonGroundTask) {
            ++guided_during_brief;
        }
    }
    EXPECT_EQ(guided_during_brief, 0)
        << "不足 3 拍的新鲜估计不得退出保持";
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach);

    // 持续新鲜：第一拍仍是保持（恢复确认），第 3 拍后回到引导。
    auto persistent = MakeValidEstimatedTarget();
    persistent.header.valid_for_ms = 2000;
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        persistent))
                    .accepted);
    const auto first = WaitForIntent(
        harness.intent_subscription, IsPositionIntent);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(LowReasonCode(*first), kLowReasonTargetHold)
        << "恢复确认的第一拍仍应为保持";
    const auto resumed = WaitForIntent(
        harness.intent_subscription, [](const common::ControlIntent& value) {
            return IsPositionIntent(value) &&
                   LowReasonCode(value) == kLowReasonGroundTask;
        });
    ASSERT_TRUE(resumed.has_value()) << "连续 3 拍新鲜后必须回到估计引导";
    EXPECT_FLOAT_EQ(resumed->target_x, 12.5f);
}

// 影子兼容：未注入命令接口时不发保持意图（保持仅控制任务语义）。
TEST(MissionStateMachineTest, ShadowS5DoesNotHoldOnTargetLoss) {
    auto config = MakeAssignmentHarnessConfig();
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/false,
                              /*with_estimation=*/true));

    ASSERT_TRUE(harness.ground_topic
                    .Publish(std::make_shared<const common::GroundStationTarget>(
                        MakeFreshGroundTarget()))
                    .accepted);
    ASSERT_TRUE(WaitForState(harness.status_subscription,
                             common::MissionState::kGpsApproach)
                    .has_value());
    DrainIntents(harness.intent_subscription);

    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeInvalidEstimatedTarget()))
                    .accepted);
    std::this_thread::sleep_for(400ms);
    EXPECT_FALSE(WaitForIntent(harness.intent_subscription, IsPositionIntent,
                               200ms)
                     .has_value())
        << "影子模式不得发保持意图";
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach);
}

// 保持能力门禁：PX4 局部位置无效时不得发保持意图（停发交兜底链），
// 且任务在看门狗到点前不中止。
TEST(MissionStateMachineTest, S5HoldRequiresValidLocalPosition) {
    auto config = MakeTakeoffTestConfig(100ms, 2000ms, 2000ms, 5000ms);
    config.takeoff.target_wait_timeout = 3000ms;
    config.takeoff.target_lost_timeout = 3000ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 310));

    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeValidEstimatedTarget()))
                    .accepted);
    ASSERT_TRUE(WaitForIntent(
                    harness.intent_subscription,
                    [](const common::ControlIntent& value) {
                        return IsPositionIntent(value) &&
                               LowReasonCode(value) == kLowReasonGroundTask;
                    })
                    .has_value());
    DrainIntents(harness.intent_subscription);

    // 局部位置失效（连接/armed/Offboard 仍在）。
    auto no_position = MakeOffboardFlight(/*armed=*/true, /*landed=*/false);
    no_position.local_position_valid = false;
    ASSERT_TRUE(PublishFlight(harness.flight_topic, no_position));
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeInvalidEstimatedTarget()))
                    .accepted);
    std::this_thread::sleep_for(400ms);
    EXPECT_FALSE(WaitForIntent(harness.intent_subscription, IsPositionIntent,
                               200ms)
                     .has_value())
        << "局部位置无效时不得发保持意图";
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach);
}

// 累计保持上限（已获得目标）：丢失计时被抖动清零也不得无限悬停，
// 累计超 target_hold_max → 65015 aborted(target_lost) + S12。
TEST(MissionStateMachineTest, S5HoldMaxAbortsAfterTargetAcquired) {
    auto config = MakeTakeoffTestConfig(100ms, 2000ms, 2000ms, 5000ms);
    config.takeoff.target_wait_timeout = 5000ms;
    config.takeoff.target_lost_timeout = 5000ms;
    config.takeoff.target_hold_max = 300ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 311));

    // 先获得目标（assigned_target_acquired_ 置位），再持续失效。
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeValidEstimatedTarget()))
                    .accepted);
    ASSERT_TRUE(WaitForIntent(
                    harness.intent_subscription,
                    [](const common::ControlIntent& value) {
                        return IsPositionIntent(value) &&
                               LowReasonCode(value) == kLowReasonGroundTask;
                    })
                    .has_value());
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeInvalidEstimatedTarget()))
                    .accepted);

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 3000ms);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason, common::MissionAssignmentReason::kTargetLost)
        << "已获得目标后超时按目标丢失回执";
    EXPECT_EQ(aborted->availability,
              common::MissionAvailability::kReturning);
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kReturnHome);
    for (const auto& call : harness.sender.History()) {
        EXPECT_NE(call.command, 187U) << "目标保持超时不得触发发射";
    }
}

// 累计保持上限（从未获得目标）：早于 target_wait_timeout 收口，
// 回执按等待超时（未获得目标）语义。
TEST(MissionStateMachineTest, S5HoldMaxAbortsBeforeTargetWaitTimeout) {
    auto config = MakeTakeoffTestConfig(100ms, 2000ms, 2000ms, 5000ms);
    config.takeoff.target_wait_timeout = 5000ms;
    config.takeoff.target_lost_timeout = 5000ms;
    config.takeoff.target_hold_max = 300ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 312));

    const auto aborted = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 3000ms);
    ASSERT_TRUE(aborted.has_value());
    EXPECT_EQ(aborted->reason,
              common::MissionAssignmentReason::kTargetWaitTimeout)
        << "从未获得目标时按等待超时回执";
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kReturnHome);
}

// 累计上限只在“目标当前不可用”时收口：短暂丢失累计未超限后恢复引导，
// 不得因历史累计立刻中止；再次失效时按累计上限收口。
TEST(MissionStateMachineTest, S5HoldMaxDoesNotAbortWhileGuiding) {
    auto config = MakeTakeoffTestConfig(100ms, 2000ms, 2000ms, 5000ms);
    config.takeoff.target_wait_timeout = 5000ms;
    config.takeoff.target_lost_timeout = 5000ms;
    config.takeoff.target_hold_max = 1000ms;
    AssignmentHarness harness(config);
    ASSERT_TRUE(harness.Start(/*with_control=*/true,
                              /*with_estimation=*/true));
    ASSERT_TRUE(DriveToGpsApproach(harness, 313));

    auto persistent = MakeValidEstimatedTarget();
    persistent.header.valid_for_ms = 2000;
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        persistent))
                    .accepted);
    ASSERT_TRUE(WaitForIntent(
                    harness.intent_subscription,
                    [](const common::ControlIntent& value) {
                        return IsPositionIntent(value) &&
                               LowReasonCode(value) == kLowReasonGroundTask;
                    })
                    .has_value());

    // 短暂丢失 400ms（< 1000ms 上限）→ 保持，随后恢复引导。
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeInvalidEstimatedTarget()))
                    .accepted);
    ASSERT_TRUE(WaitForIntent(
                    harness.intent_subscription,
                    [](const common::ControlIntent& value) {
                        return IsPositionIntent(value) &&
                               LowReasonCode(value) == kLowReasonTargetHold;
                    })
                    .has_value());
    std::this_thread::sleep_for(400ms);
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        persistent))
                    .accepted);
    ASSERT_TRUE(WaitForIntent(
                    harness.intent_subscription,
                    [](const common::ControlIntent& value) {
                        return IsPositionIntent(value) &&
                               LowReasonCode(value) == kLowReasonGroundTask;
                    })
                    .has_value())
        << "目标恢复后必须回到引导";

    // 引导期间不得因历史累计中止。
    DrainAcks(harness.ack_subscription);
    std::this_thread::sleep_for(800ms);
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kGpsApproach)
        << "目标恢复引导中不得因累计保持中止";
    EXPECT_FALSE(WaitForAckWithResult(
                     harness.ack_subscription,
                     common::MissionAssignmentResult::kCancelledOrAborted,
                     100ms)
                     .has_value())
        << "引导期间不得回 aborted";

    // 再次失效：累计（约 400ms + 本次）到 1000ms 后按上限收口。
    ASSERT_TRUE(harness.estimated_topic
                    .Publish(std::make_shared<const common::TargetState>(
                        MakeInvalidEstimatedTarget()))
                    .accepted);
    const auto aborted2 = WaitForAckWithResult(
        harness.ack_subscription,
        common::MissionAssignmentResult::kCancelledOrAborted, 3000ms);
    ASSERT_TRUE(aborted2.has_value());
    EXPECT_EQ(aborted2->reason,
              common::MissionAssignmentReason::kTargetLost);
    EXPECT_EQ(harness.machine.CurrentState(),
              common::MissionState::kReturnHome);
}

}  // namespace
