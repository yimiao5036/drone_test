#include "health/health_manager.h"

#include <chrono>
#include <cstdint>
#include <thread>

#include <gtest/gtest.h>

namespace {

std::uint64_t SteadyNowMs() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

bool WaitForSnapshot(drone::common::Topic<drone::common::HealthStatus>::Subscription& subscription,
                     std::uint32_t expected_freshness_bits,
                     std::uint32_t expected_link_bits,
                     std::uint32_t expected_device_bits,
                     std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (const auto message = subscription.TryTake(); message.has_value()) {
            if ((*message)->data_freshness_bits == expected_freshness_bits &&
                (*message)->link_health_bits == expected_link_bits &&
                (*message)->device_health_bits == expected_device_bits) {
                return true;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    return false;
}

}  // namespace

TEST(HealthManagerTest, RegistersSourcesAndPublishesHealthySnapshot) {
    drone::health::HealthManager manager;
    ASSERT_TRUE(manager.RegisterSource(drone::health::source_names::kPx4, 500));
    ASSERT_TRUE(manager.RegisterSource(drone::health::source_names::kYolo, 500));

    auto subscription = manager.Output().Subscribe(8);
    manager.ReportData(drone::health::source_names::kPx4, SteadyNowMs());
    manager.ReportData(drone::health::source_names::kYolo, SteadyNowMs());
    ASSERT_TRUE(manager.Start());

    ASSERT_TRUE(WaitForSnapshot(subscription,
                                0,
                                1U << 1U,
                                1U << 1U,
                                std::chrono::milliseconds(300)));
    EXPECT_EQ(manager.ErrorCount(), 0U);
}

TEST(HealthManagerTest, ReportsMissingDataAsTimeoutAndRecovers) {
    drone::health::HealthManager manager;
    ASSERT_TRUE(manager.RegisterSource(drone::health::source_names::kCamera, 20));

    auto subscription = manager.Output().Subscribe(16);
    ASSERT_TRUE(manager.Start());

    ASSERT_TRUE(WaitForSnapshot(subscription,
                                1U << 0U,
                                0,
                                0,
                                std::chrono::milliseconds(500)));
    EXPECT_EQ(manager.TimeoutEventCount(), 1U);

    manager.ReportData(drone::health::source_names::kCamera, SteadyNowMs());
    ASSERT_TRUE(WaitForSnapshot(subscription,
                                0,
                                1U << 0U,
                                0,
                                std::chrono::milliseconds(300)));
}

TEST(HealthManagerTest, RejectsUnknownDuplicateAndRuntimeRegistration) {
    drone::health::HealthManager manager;
    EXPECT_FALSE(manager.RegisterSource("unknown", 100));
    EXPECT_FALSE(manager.RegisterSource(drone::health::source_names::kPx4, 0));
    ASSERT_TRUE(manager.RegisterSource(drone::health::source_names::kPx4, 100));
    EXPECT_FALSE(manager.RegisterSource(drone::health::source_names::kPx4, 100));
    EXPECT_GE(manager.ErrorCount(), 3U);

    ASSERT_TRUE(manager.Start());
    EXPECT_FALSE(manager.RegisterSource(drone::health::source_names::kYolo, 100));
    EXPECT_GE(manager.ErrorCount(), 4U);
}

TEST(HealthManagerTest, RejectsUnregisteredAndFutureReports) {
    drone::health::HealthManager manager;
    ASSERT_TRUE(manager.RegisterSource(drone::health::source_names::kPx4, 100));

    manager.ReportData("unknown", SteadyNowMs());
    manager.ReportData(drone::health::source_names::kPx4, SteadyNowMs() + 1000);
    EXPECT_EQ(manager.ErrorCount(), 2U);
}

TEST(HealthManagerTest, StartStopAndRestartAreIdempotent) {
    drone::health::HealthManager manager;
    ASSERT_TRUE(manager.RegisterSource(drone::health::source_names::kVideo, 100));
    ASSERT_TRUE(manager.Start());
    EXPECT_TRUE(manager.Start());
    EXPECT_TRUE(manager.IsRunning());

    manager.Stop();
    manager.Stop();
    EXPECT_FALSE(manager.IsRunning());

    EXPECT_TRUE(manager.Start());
    EXPECT_TRUE(manager.IsRunning());
    manager.Stop();
}
