#include "control/flight_controller.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "common/topic.h"
#include "common/types.h"
#include "communication/px4_link.h"
#include "config/config.h"

namespace drone::control {
namespace {

using namespace std::chrono_literals;

/// 返回当前进程单调时钟毫秒（与实现侧同源，参照 mission_state_machine.cpp）。
uint64_t MonotonicMs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

/// 轮询等待谓词成立（参照 tests/communication/px4_link_test.cpp）。
bool WaitUntil(const std::function<bool()>& predicate,
               std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(5ms);
    }
    return predicate();
}

drone::config::FlightControllerConfig MakeControllerConfig() {
    drone::config::FlightControllerConfig config;
    config.period = 20ms;
    config.intent_timeout = 100ms;
    config.coast_timeout = 400ms;
    return config;
}

common::ControlIntent MakeVelocityIntent(double vx, double vy, double vz) {
    common::ControlIntent intent;
    intent.type = common::ControlIntentType::kVelocityHeading;
    intent.target_x = static_cast<float>(vx);
    intent.target_y = static_cast<float>(vy);
    intent.target_z = static_cast<float>(vz);
    intent.valid_until_ms = MonotonicMs() + 5000;
    return intent;
}

/// 只读门禁 Mock：LatestState() 返回注入快照，其余接口为占位空实现
/// （替代 PTY，直接驱动控制权门禁的判定输入）。
class MockPx4Link final : public drone::communication::IPx4Link {
public:
    bool Start() override { return true; }
    void Stop() override {}
    bool IsRunning() const override { return true; }
    bool IsConnected() const override { return state_.connected; }
    void SetInput(common::Topic<common::Px4Setpoint>& /*setpoint*/) override {}
    common::Topic<common::FlightStateSnapshot>& StateOutput() override {
        return state_output_;
    }
    common::FlightStateSnapshot LatestState() const override { return state_; }
    bool SendCommand(uint16_t /*command*/, float /*p1*/, float /*p2*/, float /*p3*/,
                     float /*p4*/, float /*p5*/, float /*p6*/, float /*p7*/) override {
        return false;
    }
    uint64_t SetpointSendCount() const override { return 0; }
    uint64_t ReceiveCount() const override { return 0; }
    uint64_t AckMatchCount() const override { return 0; }
    uint64_t AckTimeoutCount() const override { return 0; }
    common::CommandAckStatus AckStatus() const override { return {}; }
    void CancelPendingCommands() override {}
    uint64_t ErrorCount() const override { return 0; }

    void SetState(const common::FlightStateSnapshot& state) { state_ = state; }

private:
    common::FlightStateSnapshot state_{};
    common::Topic<common::FlightStateSnapshot> state_output_;
};

/// PX4 1.17.0 模式编码（与 px4_link 解码侧一致）：Offboard = flight_mode 6 /
/// sub_mode 0；Position = 3/0；AUTO.LOITER = 4/3。
common::FlightStateSnapshot MakeOffboardSnapshot() {
    common::FlightStateSnapshot state;
    state.connected = true;
    state.armed = true;
    state.flight_mode = 6;
    state.flight_sub_mode = 0;
    state.home_valid = true;
    state.local_position_valid = true;
    return state;
}

common::FlightStateSnapshot MakePositionModeSnapshot() {
    auto state = MakeOffboardSnapshot();
    state.flight_mode = 3;
    return state;
}

}  // namespace

TEST(FlightControllerTest, ConstructorRejectsNonPositivePeriod) {
    // 本地防御：period<=0 会让周期线程 sleep_until 立即返回、退化为忙等空转，
    // 构造期 fail-fast 抛 std::invalid_argument（不依赖 drone_config 的 Validate）。
    auto config = MakeControllerConfig();
    config.period = 0ms;
    EXPECT_THROW(FlightController controller(config), std::invalid_argument);
}

TEST(FlightControllerTest, IdlePublishesNothingWithoutAuthority) {
    // 未获授权（默认 IDLE）：即使有意图也不发布任何 setpoint
    FlightController controller(MakeControllerConfig());
    common::Topic<common::ControlIntent> intents;
    controller.SetInput(intents);
    auto subscription = controller.SetpointOutput().Subscribe(4);
    ASSERT_TRUE(controller.Start());
    (void)intents.Publish(std::make_shared<const common::ControlIntent>(
        MakeVelocityIntent(1.0, 0.0, 0.0)));
    std::this_thread::sleep_for(100ms);
    EXPECT_FALSE(subscription.TryTake().has_value());
    EXPECT_EQ(controller.SetpointCount(), 0u);
    controller.Stop();
}

TEST(FlightControllerTest, RejectsNanIntent) {
    // Review Focus #1：NaN/Inf 意图字段 → 拒绝计数，不产生输出
    FlightController controller(MakeControllerConfig());
    common::Topic<common::ControlIntent> intents;
    controller.SetInput(intents);
    ASSERT_TRUE(controller.Start());
    auto intent = MakeVelocityIntent(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    (void)intents.Publish(std::make_shared<const common::ControlIntent>(intent));
    ASSERT_TRUE(WaitUntil([&] { return controller.RejectedIntentCount() > 0; }, 500ms));
    controller.Stop();
}

TEST(FlightControllerTest, RejectsAlreadyStaleIntent) {
    // Review Focus #2：到达即超龄（valid_until_ms < now）→ 拒绝
    FlightController controller(MakeControllerConfig());
    common::Topic<common::ControlIntent> intents;
    controller.SetInput(intents);
    ASSERT_TRUE(controller.Start());
    auto intent = MakeVelocityIntent(1.0, 0.0, 0.0);
    intent.valid_until_ms = 1;  // 早已过期
    (void)intents.Publish(std::make_shared<const common::ControlIntent>(intent));
    ASSERT_TRUE(WaitUntil([&] { return controller.RejectedIntentCount() > 0; }, 500ms));
    controller.Stop();
}

// =============================================================================
// 限幅流水线用例（Task 4）：合成时间轴 + TestTick 同步驱动，确定性无睡眠。
// 拉长 intent_timeout 使单条意图可跨多拍复用（门禁/超龄语义由上面用例覆盖，
// 此处聚焦限幅/无扰衔接流水线本身）。
// =============================================================================

drone::config::FlightControllerConfig MakePipelineConfig() {
    auto config = MakeControllerConfig();
    config.intent_timeout = 60000ms;
    config.coast_timeout = 120000ms;
    return config;
}

class FlightControllerPipelineTest : public ::testing::Test {
protected:
    FlightControllerPipelineTest() : controller_(MakePipelineConfig()) {
        controller_.SetInput(intents_);
        // 控制权门禁（Task 5）：注入 Offboard+armed 快照使授权链可通，
        // 意图经 PublishIntent 携带状态机显式请求位。
        px4_link_.SetState(MakeOffboardSnapshot());
        controller_.SetPx4LinkSource(&px4_link_);
        output_ = controller_.SetpointOutput().Subscribe(kOutputCapacity);
        // 测试友好授权入口：强制 ACTIVE 放行发布路径（授权链每拍重估，
        // 快照需保持 Offboard+armed 才能维持 ACTIVE）
        controller_.TestForceAuthority(ControlAuthority::kActive);
    }

    /// 发布一条意图（receive_time 为 0，由控制器在排干时补当前合成时刻；
    /// 携带控制权显式请求位，模拟状态机正式请求）。
    void PublishIntent(common::ControlIntent intent) {
        intent.valid_until_ms = now_ms_ + kValidForMs;
        intent.reason_code |= common::kControlIntentReasonRequestControl;
        (void)intents_.Publish(
            std::make_shared<const common::ControlIntent>(intent));
    }

    /// 以当前合成时刻同步执行一拍，随后推进一个控制周期。
    void Tick() {
        controller_.TestTick(now_ms_);
        now_ms_ += kPeriodMs;
    }

    /// 取下一条输出（无输出返回 nullopt）。
    std::optional<common::Px4Setpoint> TakeOutput() {
        auto message = output_.TryTake();
        if (!message.has_value()) {
            return std::nullopt;
        }
        return **message;
    }

    static constexpr uint64_t kStartMs = 1000000;
    static constexpr uint64_t kPeriodMs = 20;
    static constexpr uint64_t kValidForMs = 60000;
    static constexpr std::size_t kOutputCapacity = 512;
    /// max_accel_mps2(2.0) * period(0.02s)：每拍速度最大推进量
    static constexpr double kMaxStep = 2.0 * 0.02;

    uint64_t now_ms_ = kStartMs;
    common::Topic<common::ControlIntent> intents_;
    MockPx4Link px4_link_;
    FlightController controller_;
    common::Topic<common::Px4Setpoint>::Subscription output_;
};

TEST_F(FlightControllerPipelineTest, SpeedMagnitudeClamped) {
    // 合速度 |(3,4,0)|=5m/s 超过 max_speed_mps=2 → 等比钳制到 2，方向不变
    auto intent = MakeVelocityIntent(3.0, 4.0, 0.0);
    intent.control_source = 1;
    PublishIntent(intent);
    for (int i = 0; i < 200; ++i) {
        Tick();
    }
    std::optional<common::Px4Setpoint> last;
    double max_magnitude = 0.0;
    while (auto out = TakeOutput()) {
        const double magnitude = std::sqrt(
            static_cast<double>(out->x) * out->x +
            static_cast<double>(out->y) * out->y +
            static_cast<double>(out->z) * out->z);
        max_magnitude = std::max(max_magnitude, magnitude);
        last = out;
    }
    ASSERT_TRUE(last.has_value());
    // 全程不超上限；收敛后幅值=2 且方向比 3:4 不变（等比钳制）
    EXPECT_LE(max_magnitude, 2.0 + 1e-4);
    EXPECT_EQ(last->type, common::SetpointType::kVelocity);
    EXPECT_TRUE(last->valid);
    EXPECT_NEAR(last->x, 1.2, 1e-3);
    EXPECT_NEAR(last->y, 1.6, 1e-3);
    EXPECT_NEAR(last->z, 0.0, 1e-6);
    EXPECT_GT(controller_.ClampedCount(), 0u);  // 限幅触发计数（供 G2 验收核对）
    // 坐标系契约：输出必须标注局部 NED(frame_id=3)，否则 Px4Link 结构校验拒发
    EXPECT_EQ(last->header.frame_id, 3);
}

TEST_F(FlightControllerPipelineTest, AccelerationSlewLimitsStepResponse) {
    // 阶跃 0→2m/s：相邻周期输出差 <= max_accel_mps2 * period
    auto intent = MakeVelocityIntent(2.0, 0.0, 0.0);
    intent.control_source = 1;
    PublishIntent(intent);
    for (int i = 0; i < 100; ++i) {
        Tick();
    }
    std::vector<common::Px4Setpoint> outputs;
    while (auto out = TakeOutput()) {
        outputs.push_back(*out);
    }
    ASSERT_EQ(outputs.size(), 100u);
    // 首拍从 0 起步只推进一拍（首条指令突变抑制，S6）
    EXPECT_NEAR(outputs.front().x, kMaxStep, 1e-5);
    for (std::size_t i = 1; i < outputs.size(); ++i) {
        EXPECT_LE(std::fabs(static_cast<double>(outputs[i].x) - outputs[i - 1].x),
                  kMaxStep + 1e-6)
            << "第 " << i << " 拍相邻差超 slew 上限";
    }
    EXPECT_NEAR(outputs.back().x, 2.0, 1e-3);  // 最终收敛到目标
}

TEST_F(FlightControllerPipelineTest, BumplessTransitionStartsFromLastOutput) {
    // control_source 从 0(位置) 切到 1(视觉)：新意图首条输出斜坡起点=上一周期
    // 输出，不回零（输出序列相邻差满足 slew 上限且不为 0 跳变）
    auto phase1 = MakeVelocityIntent(1.0, 0.0, 0.0);
    phase1.control_source = 0;
    PublishIntent(phase1);
    for (int i = 0; i < 60; ++i) {
        Tick();  // 收敛到 1.0
    }
    while (TakeOutput().has_value()) {
    }  // 排干阶段一输出

    auto phase2 = MakeVelocityIntent(2.0, 0.0, 0.0);
    phase2.control_source = 1;
    PublishIntent(phase2);
    Tick();  // 切换后首拍
    auto first = TakeOutput();
    ASSERT_TRUE(first.has_value());
    // 斜坡起点=上一周期输出 1.0：首拍=1.0+一拍步长，不回零
    EXPECT_NEAR(first->x, 1.0 + kMaxStep, 1e-5);

    for (int i = 0; i < 60; ++i) {
        Tick();
    }
    double previous = first->x;
    while (auto out = TakeOutput()) {
        EXPECT_LE(std::fabs(static_cast<double>(out->x) - previous), kMaxStep + 1e-6);
        previous = out->x;
    }
    EXPECT_NEAR(previous, 2.0, 1e-3);  // 平滑收敛到新目标
}

TEST_F(FlightControllerPipelineTest, ClockRegressionFreezesOutput) {
    // Review Focus #5：now 回退时 dt<=0，输出保持上一值
    auto intent = MakeVelocityIntent(2.0, 0.0, 0.0);
    intent.control_source = 1;
    PublishIntent(intent);
    controller_.TestTick(now_ms_);                  // t0
    auto out1 = TakeOutput();
    controller_.TestTick(now_ms_ + kPeriodMs);      // t0+20：正常推进
    auto out2 = TakeOutput();
    controller_.TestTick(now_ms_ + kPeriodMs / 2);  // 回退到 t0+10
    auto out3 = TakeOutput();
    ASSERT_TRUE(out1.has_value());
    ASSERT_TRUE(out2.has_value());
    ASSERT_TRUE(out3.has_value());
    EXPECT_GT(out2->x, out1->x);         // 正常推进
    EXPECT_FLOAT_EQ(out3->x, out2->x);   // 回退拍冻结：保持上一值
    controller_.TestTick(now_ms_ + 2 * kPeriodMs);  // 恢复前进
    auto out4 = TakeOutput();
    ASSERT_TRUE(out4.has_value());
    EXPECT_GT(out4->x, out3->x);
}

TEST_F(FlightControllerPipelineTest, UnknownControlSourceTreatedAsSafe) {
    // Review Focus #3：未定义 control_source（如 99）→ 按安全降级处理 +
    // 节流 WARN，不崩溃不直通；仅刹停类意图生效
    auto bad = MakeVelocityIntent(1.0, 0.0, 0.0);
    bad.control_source = 99;
    PublishIntent(bad);
    for (int i = 0; i < 5; ++i) {
        Tick();
    }
    EXPECT_FALSE(TakeOutput().has_value());  // 速度意图被抑制，不直通
    EXPECT_GT(controller_.ErrorCount(), 0u);

    // 同来源刹停意图放行（安全语义：只允许刹停类意图生效）
    common::ControlIntent brake;
    brake.type = common::ControlIntentType::kBrakeHover;
    brake.control_source = 99;
    PublishIntent(brake);
    Tick();
    auto out = TakeOutput();
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(out->type, common::SetpointType::kVelocity);
    EXPECT_TRUE(out->valid);
    EXPECT_FLOAT_EQ(out->x, 0.f);
    EXPECT_FLOAT_EQ(out->y, 0.f);
    EXPECT_FLOAT_EQ(out->z, 0.f);
}

TEST_F(FlightControllerPipelineTest, PositionGuidePassthrough) {
    // 位置式（S5 远端引导）：限幅透传、不 slew、类型映射正确；
    // 完整联调测试归 Task 7
    common::ControlIntent intent;
    intent.type = common::ControlIntentType::kPositionGuide;
    intent.control_source = 0;
    intent.target_x = 10.f;
    intent.target_y = 20.f;
    intent.target_z = -5.f;
    intent.yaw_deg = 30.f;
    PublishIntent(intent);
    Tick();
    auto out = TakeOutput();
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(out->type, common::SetpointType::kPosition);
    EXPECT_TRUE(out->valid);
    EXPECT_FLOAT_EQ(out->x, 10.f);  // 首拍即目标值（位置不做 slew）
    EXPECT_FLOAT_EQ(out->y, 20.f);
    EXPECT_FLOAT_EQ(out->z, -5.f);
    EXPECT_TRUE(out->use_yaw);
    EXPECT_FLOAT_EQ(out->yaw_deg, 30.f);
    EXPECT_EQ(controller_.ClampedCount(), 0u);
    // P3 默认开关关闭：不携带前馈，行为与旧版一致。
    EXPECT_EQ(out->feedforward_valid_mask, 0U);
    EXPECT_FLOAT_EQ(out->feedforward_vx_mps, 0.f);
    EXPECT_FLOAT_EQ(out->feedforward_vy_mps, 0.f);
    EXPECT_FLOAT_EQ(out->feedforward_vz_mps, 0.f);
    // 坐标系契约：位置式输出同样标注局部 NED(frame_id=3)
    EXPECT_EQ(out->header.frame_id, 3);
}

TEST_F(FlightControllerPipelineTest, PositionFeedforwardIgnoredWhenSwitchOff) {
    // 开关关闭：即使意图误带前馈，控制器也完全忽略（掩码/值清零）。
    common::ControlIntent intent;
    intent.type = common::ControlIntentType::kPositionGuide;
    intent.control_source = 0;
    intent.target_x = 10.f;
    intent.target_y = 20.f;
    intent.target_z = -5.f;
    intent.feedforward_vx_mps = 2.0f;
    intent.feedforward_vy_mps = -1.0f;
    intent.feedforward_vz_mps = 0.5f;
    intent.feedforward_valid_mask = common::kVelocityValidAll;
    PublishIntent(intent);
    Tick();
    auto out = TakeOutput();
    ASSERT_TRUE(out.has_value());
    EXPECT_FLOAT_EQ(out->x, 10.f);
    EXPECT_EQ(out->feedforward_valid_mask, 0U);
    EXPECT_FLOAT_EQ(out->feedforward_vx_mps, 0.f);
    EXPECT_FLOAT_EQ(out->feedforward_vy_mps, 0.f);
    EXPECT_FLOAT_EQ(out->feedforward_vz_mps, 0.f);
    EXPECT_EQ(controller_.ClampedCount(), 0u);
    EXPECT_EQ(controller_.FeedforwardDroppedCount(), 0u);
}

TEST_F(FlightControllerPipelineTest, PositionToVelocitySlewStartsFromZero) {
    // 位置→速度类型切换：上一输出是位置量纲，速度斜坡须从 0 起步——
    // 若误以位置值作斜坡初值，首拍速度会跳变到位置量级（危险）
    common::ControlIntent position;
    position.type = common::ControlIntentType::kPositionGuide;
    position.control_source = 0;
    position.target_x = 10.f;
    PublishIntent(position);
    Tick();
    (void)TakeOutput();

    auto velocity = MakeVelocityIntent(2.0, 0.0, 0.0);
    velocity.control_source = 1;
    PublishIntent(velocity);
    Tick();
    auto first = TakeOutput();
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->type, common::SetpointType::kVelocity);
    EXPECT_NEAR(first->x, kMaxStep, 1e-5);  // 从 0 起步推进一拍，而非位置值 10
}

// =============================================================================
// 控制权三态用例（Task 5）：MockPx4Link 注入快照 + 合成时间轴 TestTick 驱动。
// 配置：period=20ms、intent_timeout=100ms、coast_timeout=400ms
// （MakeControllerConfig），围栏 max_radius_m=100、max_altitude_m=30（默认）。
// =============================================================================

class FlightControllerAuthorityTest : public ::testing::Test {
protected:
    FlightControllerAuthorityTest() : controller_(MakeControllerConfig()) {
        controller_.SetInput(intents_);
        controller_.SetPx4LinkSource(&px4_link_);
        output_ = controller_.SetpointOutput().Subscribe(kOutputCapacity);
    }

    /// 发布一条带控制权显式请求位的意图（receive_time 由控制器补合成时刻）。
    void PublishIntent(common::ControlIntent intent) {
        intent.valid_until_ms = now_ms_ + kValidForMs;
        intent.reason_code |= common::kControlIntentReasonRequestControl;
        (void)intents_.Publish(
            std::make_shared<const common::ControlIntent>(intent));
    }

    /// 以当前合成时刻同步执行一拍，随后推进一个控制周期。
    void Tick() {
        controller_.TestTick(now_ms_);
        now_ms_ += kPeriodMs;
    }

    /// 取下一条输出（无输出返回 nullopt）。
    std::optional<common::Px4Setpoint> TakeOutput() {
        auto message = output_.TryTake();
        if (!message.has_value()) {
            return std::nullopt;
        }
        return **message;
    }

    /// 排干输出队列。
    void DrainOutputs() {
        while (TakeOutput().has_value()) {
        }
    }

    static constexpr uint64_t kStartMs = 1000000;
    static constexpr uint64_t kPeriodMs = 20;
    static constexpr uint64_t kValidForMs = 60000;
    static constexpr std::size_t kOutputCapacity = 512;

    uint64_t now_ms_ = kStartMs;
    common::Topic<common::ControlIntent> intents_;
    MockPx4Link px4_link_;
    FlightController controller_;
    common::Topic<common::Px4Setpoint>::Subscription output_;
};

TEST_F(FlightControllerAuthorityTest, ActiveOnlyInOffboardAndArmed) {
    // 未 armed：授权链不通，保持 IDLE 不输出（即使意图带显式请求位）
    auto disarmed = MakeOffboardSnapshot();
    disarmed.armed = false;
    px4_link_.SetState(disarmed);
    PublishIntent(MakeVelocityIntent(1.0, 0.0, 0.0));
    Tick();
    EXPECT_EQ(controller_.Authority(), ControlAuthority::kIdle);
    EXPECT_FALSE(TakeOutput().has_value());

    // 快照 flight_mode=6/0(Offboard) 且 armed → 授权链通，转 ACTIVE 产生输出
    px4_link_.SetState(MakeOffboardSnapshot());
    Tick();
    EXPECT_EQ(controller_.Authority(), ControlAuthority::kActive);
    auto out = TakeOutput();
    ASSERT_TRUE(out.has_value());
    EXPECT_TRUE(out->valid);
    EXPECT_EQ(out->type, common::SetpointType::kVelocity);

    // 模式变为 4/3(AUTO/LOITER) → 下一周期立即停发（valid=false）并转 IDLE
    auto loiter = MakeOffboardSnapshot();
    loiter.flight_mode = 4;
    loiter.flight_sub_mode = 3;
    px4_link_.SetState(loiter);
    Tick();
    EXPECT_EQ(controller_.Authority(), ControlAuthority::kIdle);
    auto stop = TakeOutput();
    ASSERT_TRUE(stop.has_value());
    EXPECT_FALSE(stop->valid);
}

TEST_F(FlightControllerAuthorityTest, ModeFlapDoesNotChatterAuthority) {
    // Review Focus #4：Offboard↔Position 快速抖动 → 每次离开都转 IDLE，
    // 回 ACTIVE 必须重新满足授权链（含状态机显式请求标志）
    px4_link_.SetState(MakeOffboardSnapshot());
    PublishIntent(MakeVelocityIntent(1.0, 0.0, 0.0));
    Tick();
    ASSERT_EQ(controller_.Authority(), ControlAuthority::kActive);

    // 抖到 Position：立即停发转 IDLE（显式请求标志同时清零）
    px4_link_.SetState(MakePositionModeSnapshot());
    Tick();
    EXPECT_EQ(controller_.Authority(), ControlAuthority::kIdle);
    DrainOutputs();

    // 抖回 Offboard 但无新意图：授权链缺"状态机显式请求"，保持 IDLE 不恢复
    px4_link_.SetState(MakeOffboardSnapshot());
    Tick();
    EXPECT_EQ(controller_.Authority(), ControlAuthority::kIdle);
    EXPECT_FALSE(TakeOutput().has_value());

    // 状态机重新显式请求（新意图带请求位）→ 授权链重新全通回 ACTIVE
    PublishIntent(MakeVelocityIntent(1.0, 0.0, 0.0));
    Tick();
    EXPECT_EQ(controller_.Authority(), ControlAuthority::kActive);
    auto out = TakeOutput();
    ASSERT_TRUE(out.has_value());
    EXPECT_TRUE(out->valid);

    // 再次抖离 → 再次立即停发转 IDLE（离开即撤，无迟滞不抖动）
    px4_link_.SetState(MakePositionModeSnapshot());
    Tick();
    EXPECT_EQ(controller_.Authority(), ControlAuthority::kIdle);
    auto stop = TakeOutput();
    ASSERT_TRUE(stop.has_value());
    EXPECT_FALSE(stop->valid);
}

TEST_F(FlightControllerAuthorityTest, CoastBrakesThenIdles) {
    // ACTIVE 中意图超龄：先发零速刹车（COAST），coast_timeout 后停发转 IDLE
    px4_link_.SetState(MakeOffboardSnapshot());
    PublishIntent(MakeVelocityIntent(1.0, 0.0, 0.0));
    Tick();  // t0：转 ACTIVE，正常输出
    ASSERT_EQ(controller_.Authority(), ControlAuthority::kActive);
    DrainOutputs();

    // 停止发意图：receive_time=t0，intent_timeout=100ms → 年龄 <=100ms 的
    // 5 拍保持 ACTIVE，第 6 拍（t0+120，年龄 120ms）持有期超龄转 COAST
    for (int i = 0; i < 5; ++i) {
        Tick();
        ASSERT_EQ(controller_.Authority(), ControlAuthority::kActive);
        DrainOutputs();
    }
    Tick();  // t0+120：超龄 → COAST + 零速刹车
    ASSERT_EQ(controller_.Authority(), ControlAuthority::kCoast);
    auto brake = TakeOutput();
    ASSERT_TRUE(brake.has_value());
    EXPECT_TRUE(brake->valid);
    EXPECT_EQ(brake->type, common::SetpointType::kVelocity);
    EXPECT_FLOAT_EQ(brake->x, 0.f);
    EXPECT_FLOAT_EQ(brake->y, 0.f);
    EXPECT_FLOAT_EQ(brake->z, 0.f);

    // COAST 期间意图恢复 → 回 ACTIVE（恢复路径）
    PublishIntent(MakeVelocityIntent(1.0, 0.0, 0.0));
    Tick();  // t0+140：新意图 receive=t0+140
    ASSERT_EQ(controller_.Authority(), ControlAuthority::kActive);
    auto resumed = TakeOutput();
    ASSERT_TRUE(resumed.has_value());
    EXPECT_TRUE(resumed->valid);

    // 再次断粮：年龄 20..100ms 的 5 拍保持 ACTIVE，第 6 拍（t0+260）转 COAST
    for (int i = 0; i < 6; ++i) {
        Tick();
    }
    ASSERT_EQ(controller_.Authority(), ControlAuthority::kCoast);
    DrainOutputs();

    // coast_timeout=400ms：20 拍内（now-coast_since=20..400ms）保持 COAST
    // 刹车输出，第 21 拍（420ms > 400ms）超时转 IDLE 并停发
    for (int i = 0; i < 20; ++i) {
        Tick();
        ASSERT_EQ(controller_.Authority(), ControlAuthority::kCoast)
            << "COAST 第 " << i << " 拍";
        auto coast_out = TakeOutput();
        ASSERT_TRUE(coast_out.has_value());
        EXPECT_TRUE(coast_out->valid);
        EXPECT_FLOAT_EQ(coast_out->x, 0.f);
    }
    Tick();  // 超时 → IDLE + valid=false 停发
    EXPECT_EQ(controller_.Authority(), ControlAuthority::kIdle);
    auto stop = TakeOutput();
    ASSERT_TRUE(stop.has_value());
    EXPECT_FALSE(stop->valid);
}

TEST_F(FlightControllerAuthorityTest, StaleRequestDoesNotActivateAuthority) {
    // 评审修复（fix round 2）：请求位意图到达后长期 IDLE（无新意图），
    // 授权链链路/模式条件事后就绪时，陈旧请求不得激活 ACTIVE——IDLE→ACTIVE
    // 须同时持有新鲜请求位意图
    px4_link_.SetState(MakePositionModeSnapshot());  // 非 Offboard：保持 IDLE
    PublishIntent(MakeVelocityIntent(1.0, 0.0, 0.0));
    Tick();  // t0：请求位意图被排干（锁存=true），门禁不通 → IDLE
    EXPECT_EQ(controller_.Authority(), ControlAuthority::kIdle);

    // 长期 IDLE：推进远超 intent_timeout(100ms)，锁存意图陈旧
    for (int i = 0; i < 10; ++i) {  // t0+20 .. t0+200
        Tick();
        EXPECT_EQ(controller_.Authority(), ControlAuthority::kIdle);
    }

    // Offboard 就绪：链路/模式条件满足，但请求位意图已超龄 → 不得转 ACTIVE
    px4_link_.SetState(MakeOffboardSnapshot());
    Tick();
    EXPECT_EQ(controller_.Authority(), ControlAuthority::kIdle);
    EXPECT_FALSE(TakeOutput().has_value());

    // 对照：补一条新鲜请求位意图 → 授权链全通，转 ACTIVE
    PublishIntent(MakeVelocityIntent(1.0, 0.0, 0.0));
    Tick();
    EXPECT_EQ(controller_.Authority(), ControlAuthority::kActive);
    auto out = TakeOutput();
    ASSERT_TRUE(out.has_value());
    EXPECT_TRUE(out->valid);
}

TEST_F(FlightControllerAuthorityTest, FenceClampsPositionIntent) {
    // 位置意图超过 max_radius_m / max_altitude_m（相对 Home）→ 钳制到边界 +
    // 节流 WARN（ClampedCount 计数）
    px4_link_.SetState(MakeOffboardSnapshot());
    common::ControlIntent intent;
    intent.type = common::ControlIntentType::kPositionGuide;
    intent.control_source = 0;
    intent.target_x = 150.f;  // 水平半径 150m > max_radius_m=100
    intent.target_y = 0.f;
    intent.target_z = -50.f;  // 相对高度 50m（NED z 向下为负）> max_altitude_m=30
    PublishIntent(intent);
    Tick();
    ASSERT_EQ(controller_.Authority(), ControlAuthority::kActive);
    auto out = TakeOutput();
    ASSERT_TRUE(out.has_value());
    EXPECT_TRUE(out->valid);
    EXPECT_EQ(out->type, common::SetpointType::kPosition);
    EXPECT_NEAR(out->x, 100.f, 1e-3);  // 水平等比钳制到半径边界
    EXPECT_NEAR(out->y, 0.f, 1e-6);
    EXPECT_FLOAT_EQ(out->z, -30.f);  // 高度钳制到上限
    EXPECT_EQ(controller_.ClampedCount(), 2u);  // 水平+高度各钳制一次

    // home_valid=false：无基准不可围栏 → 位置式意图拒绝输出并计数
    auto no_home = MakeOffboardSnapshot();
    no_home.home_valid = false;
    px4_link_.SetState(no_home);
    common::ControlIntent no_home_intent;
    no_home_intent.type = common::ControlIntentType::kPositionGuide;
    no_home_intent.control_source = 0;
    no_home_intent.target_x = 10.f;
    no_home_intent.target_z = -5.f;
    PublishIntent(no_home_intent);
    Tick();
    EXPECT_FALSE(TakeOutput().has_value());
    EXPECT_GT(controller_.RejectedIntentCount(), 0u);
}

// =============================================================================
// P3 位置速度前馈用例（2026-10-19）：开关显式打开，合成时间轴 + TestTick。
// 配置：max_feedforward_mps=1.0（前馈独立上限，与 max_speed_mps 无关）。
// =============================================================================

drone::config::FlightControllerConfig MakeFeedforwardConfig() {
    auto config = MakePipelineConfig();
    config.enable_position_velocity_feedforward = true;
    config.max_feedforward_mps = 1.0;
    return config;
}

class FlightControllerFeedforwardTest : public ::testing::Test {
protected:
    FlightControllerFeedforwardTest() : controller_(MakeFeedforwardConfig()) {
        controller_.SetInput(intents_);
        px4_link_.SetState(MakeOffboardSnapshot());
        controller_.SetPx4LinkSource(&px4_link_);
        output_ = controller_.SetpointOutput().Subscribe(kOutputCapacity);
        controller_.TestForceAuthority(ControlAuthority::kActive);
    }

    void PublishIntent(common::ControlIntent intent) {
        intent.valid_until_ms = now_ms_ + kValidForMs;
        intent.reason_code |= common::kControlIntentReasonRequestControl;
        (void)intents_.Publish(
            std::make_shared<const common::ControlIntent>(intent));
    }

    void Tick() {
        controller_.TestTick(now_ms_);
        now_ms_ += kPeriodMs;
    }

    std::optional<common::Px4Setpoint> TakeOutput() {
        auto message = output_.TryTake();
        if (!message.has_value()) {
            return std::nullopt;
        }
        return **message;
    }

    common::ControlIntent MakePositionIntent(float x, float y, float z) {
        common::ControlIntent intent;
        intent.type = common::ControlIntentType::kPositionGuide;
        intent.control_source = 0;
        intent.target_x = x;
        intent.target_y = y;
        intent.target_z = z;
        return intent;
    }

    static constexpr uint64_t kStartMs = 1000000;
    static constexpr uint64_t kPeriodMs = 20;
    static constexpr uint64_t kValidForMs = 60000;
    static constexpr std::size_t kOutputCapacity = 512;
    static constexpr double kMaxStep = 2.0 * 0.02;

    uint64_t now_ms_ = kStartMs;
    common::Topic<common::ControlIntent> intents_;
    MockPx4Link px4_link_;
    FlightController controller_;
    common::Topic<common::Px4Setpoint>::Subscription output_;
};

TEST_F(FlightControllerFeedforwardTest, PositionFeedforwardPassthrough) {
    auto intent = MakePositionIntent(10.f, 20.f, -5.f);
    intent.feedforward_vx_mps = 0.5f;
    intent.feedforward_vy_mps = -0.25f;
    intent.feedforward_vz_mps = 0.25f;
    intent.feedforward_valid_mask = common::kVelocityValidAll;
    PublishIntent(intent);
    Tick();
    auto out = TakeOutput();
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(out->type, common::SetpointType::kPosition);
    EXPECT_FLOAT_EQ(out->x, 10.f);  // 位置仍透传
    EXPECT_FLOAT_EQ(out->y, 20.f);
    EXPECT_FLOAT_EQ(out->z, -5.f);
    EXPECT_EQ(out->feedforward_valid_mask, common::kVelocityValidAll);
    EXPECT_FLOAT_EQ(out->feedforward_vx_mps, 0.5f);
    EXPECT_FLOAT_EQ(out->feedforward_vy_mps, -0.25f);
    EXPECT_FLOAT_EQ(out->feedforward_vz_mps, 0.25f);
    EXPECT_EQ(controller_.ClampedCount(), 0u);
    EXPECT_EQ(controller_.FeedforwardDroppedCount(), 0u);
}

TEST_F(FlightControllerFeedforwardTest, HorizontalAndVerticalClampedIndependently) {
    auto intent = MakePositionIntent(0.f, 0.f, -1.f);
    intent.feedforward_vx_mps = 3.f;   // 水平模长 5 → 等比缩放到 1
    intent.feedforward_vy_mps = 4.f;
    intent.feedforward_vz_mps = -5.f;  // 垂直独立钳制到 -1
    intent.feedforward_valid_mask = common::kVelocityValidAll;
    PublishIntent(intent);
    Tick();
    auto out = TakeOutput();
    ASSERT_TRUE(out.has_value());
    ASSERT_EQ(out->feedforward_valid_mask, common::kVelocityValidAll);
    EXPECT_NEAR(out->feedforward_vx_mps, 0.6f, 1e-5f);
    EXPECT_NEAR(out->feedforward_vy_mps, 0.8f, 1e-5f);
    EXPECT_FLOAT_EQ(out->feedforward_vz_mps, -1.f);
    EXPECT_EQ(controller_.ClampedCount(), 2u);  // 水平 + 垂直各一次
}

TEST_F(FlightControllerFeedforwardTest, PerAxisFiniteDegradationKeepsPosition) {
    auto intent = MakePositionIntent(1.f, 2.f, -3.f);
    intent.feedforward_vx_mps = 1.f;
    intent.feedforward_vy_mps = std::numeric_limits<float>::quiet_NaN();
    intent.feedforward_vz_mps = std::numeric_limits<float>::infinity();
    intent.feedforward_valid_mask = common::kVelocityValidAll;
    PublishIntent(intent);
    Tick();
    auto out = TakeOutput();
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(out->feedforward_valid_mask, common::kVelocityValidNorth);
    EXPECT_FLOAT_EQ(out->feedforward_vx_mps, 1.f);
    EXPECT_FLOAT_EQ(out->feedforward_vy_mps, 0.f);
    EXPECT_FLOAT_EQ(out->feedforward_vz_mps, 0.f);
    EXPECT_EQ(controller_.FeedforwardDroppedCount(), 2u);
    // 位置照常发布（坏前馈不拒绝整条意图）。
    EXPECT_FLOAT_EQ(out->x, 1.f);
    EXPECT_FLOAT_EQ(out->y, 2.f);
    EXPECT_FLOAT_EQ(out->z, -3.f);

    // 三轴全部非有限：前馈全丢，位置仍发布。
    auto all_bad = MakePositionIntent(4.f, 5.f, -6.f);
    all_bad.feedforward_vx_mps = std::numeric_limits<float>::quiet_NaN();
    all_bad.feedforward_vy_mps = std::numeric_limits<float>::infinity();
    all_bad.feedforward_vz_mps = std::numeric_limits<float>::quiet_NaN();
    all_bad.feedforward_valid_mask = common::kVelocityValidAll;
    PublishIntent(all_bad);
    Tick();
    auto second = TakeOutput();
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->feedforward_valid_mask, 0U);
    EXPECT_EQ(controller_.FeedforwardDroppedCount(), 5u);
    EXPECT_FLOAT_EQ(second->x, 4.f);
}

TEST_F(FlightControllerFeedforwardTest, PositionToVelocitySlewStartsFromZero) {
    auto position = MakePositionIntent(10.f, 0.f, 0.f);
    position.feedforward_vx_mps = 1.f;
    position.feedforward_valid_mask = common::kVelocityValidNorth;
    PublishIntent(position);
    Tick();
    (void)TakeOutput();

    auto velocity = MakeVelocityIntent(2.0, 0.0, 0.0);
    velocity.control_source = 1;
    PublishIntent(velocity);
    Tick();
    auto first = TakeOutput();
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->type, common::SetpointType::kVelocity);
    // 前馈不参与速度式无扰衔接：仍从 0 起步推进一拍。
    EXPECT_NEAR(first->x, kMaxStep, 1e-5);
    EXPECT_EQ(first->feedforward_valid_mask, 0U);
}

TEST_F(FlightControllerFeedforwardTest, VelocityIntentIgnoresFeedforwardFields) {
    auto intent = MakeVelocityIntent(1.5, 0.0, 0.0);
    intent.control_source = 1;
    intent.feedforward_vx_mps = 9.f;
    intent.feedforward_valid_mask = common::kVelocityValidNorth;
    PublishIntent(intent);
    Tick();
    auto out = TakeOutput();
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(out->type, common::SetpointType::kVelocity);
    EXPECT_NEAR(out->x, kMaxStep, 1e-5);  // 速度意图走自身斜坡，忽略前馈字段
    EXPECT_EQ(out->feedforward_valid_mask, 0U);
}

}  // namespace drone::control
