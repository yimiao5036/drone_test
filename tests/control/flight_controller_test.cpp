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
        output_ = controller_.SetpointOutput().Subscribe(kOutputCapacity);
        // 测试友好授权入口：控制权三态归 Task 5，此处强制 ACTIVE 放行发布路径
        controller_.TestForceAuthority(ControlAuthority::kActive);
    }

    /// 发布一条意图（receive_time 为 0，由控制器在排干时补当前合成时刻）。
    void PublishIntent(common::ControlIntent intent) {
        intent.valid_until_ms = now_ms_ + kValidForMs;
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

}  // namespace drone::control
