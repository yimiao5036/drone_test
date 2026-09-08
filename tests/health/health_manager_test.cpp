#include "health/health_manager.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace {

class SequenceCpuLoadProvider final : public drone::health::ICpuLoadProvider {
public:
    explicit SequenceCpuLoadProvider(std::vector<drone::health::CpuLoadSample> samples)
        : samples_(std::move(samples)) {}

    drone::health::CpuLoadSample Sample() override {
        if (index_ >= samples_.size()) {
            return samples_.back();
        }
        return samples_[index_++];
    }

    void Reset() override { index_ = 0; }

private:
    std::vector<drone::health::CpuLoadSample> samples_;
    std::size_t index_ = 0;
};

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

TEST(HealthManagerTest, ProcStatProviderCalculatesCpuLoadFromCounterDelta) {
    const auto path = std::filesystem::temp_directory_path() / "drone_proc_stat_test.txt";
    {
        std::ofstream output(path);
        output << "cpu 100 0 100 800 0 0 0 0 0 0\n";
    }
    drone::health::ProcStatCpuLoadProvider provider(path.string());
    EXPECT_EQ(provider.Sample().status, drone::health::CpuLoadSampleStatus::kWarmup);

    {
        std::ofstream output(path);
        output << "cpu 150 0 150 900 0 0 0 0 0 0\n";
    }
    const auto sample = provider.Sample();
    EXPECT_EQ(sample.status, drone::health::CpuLoadSampleStatus::kValid);
    EXPECT_FLOAT_EQ(sample.load_pct, 50.f);
    std::filesystem::remove(path);
}

TEST(HealthManagerTest, PublishesInjectedCpuLoadAndMarksWarmupUnknown) {
    auto provider = std::make_unique<SequenceCpuLoadProvider>(
        std::vector<drone::health::CpuLoadSample>{
            {drone::health::CpuLoadSampleStatus::kWarmup, 0.f},
            {drone::health::CpuLoadSampleStatus::kValid, 37.5f}});
    drone::health::HealthManager manager(std::move(provider),
                                         std::chrono::milliseconds(10));
    auto subscription = manager.Output().Subscribe(16);
    ASSERT_TRUE(manager.Start());

    bool saw_unknown = false;
    bool saw_valid = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline && !saw_valid) {
        if (const auto message = subscription.TryTake(); message.has_value()) {
            saw_unknown |= std::isnan((*message)->cpu_load_pct);
            saw_valid |= std::fabs((*message)->cpu_load_pct - 37.5f) < 0.01f;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    EXPECT_TRUE(saw_unknown);
    EXPECT_TRUE(saw_valid);
}

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

TEST(HealthManagerTest, ActiveErrorSetsBitAndNewerDataClearsIt) {
    drone::health::HealthManager manager;
    ASSERT_TRUE(manager.RegisterSource(drone::health::source_names::kCamera, 500));
    auto subscription = manager.Output().Subscribe(16);
    manager.ReportData(drone::health::source_names::kCamera, SteadyNowMs());
    ASSERT_TRUE(manager.Start());

    const auto wait_error_bits = [&](std::uint32_t expected) {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(500);
        while (std::chrono::steady_clock::now() < deadline) {
            if (const auto message = subscription.TryTake(); message.has_value()) {
                if ((*message)->error_bits == expected) {
                    return true;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        return false;
    };

    const std::uint64_t error_time = SteadyNowMs();
    manager.ReportError(drone::health::source_names::kCamera, error_time);
    ASSERT_TRUE(wait_error_bits(drone::health::error_bits::kCamera));

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    manager.ReportData(drone::health::source_names::kCamera, SteadyNowMs());
    EXPECT_TRUE(wait_error_bits(0));
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
