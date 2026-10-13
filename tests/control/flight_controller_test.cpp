#include "control/flight_controller.h"

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>

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

}  // namespace drone::control
