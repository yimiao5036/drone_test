#include "common/latency_statistics.h"

#include <gtest/gtest.h>

TEST(LatencyStatisticsTest, EmptySnapshotIsZero) {
    drone::common::LatencyStatistics stats(4);
    const auto summary = stats.Snapshot();
    EXPECT_EQ(summary.total_count, 0U);
    EXPECT_EQ(summary.window_count, 0U);
}

TEST(LatencyStatisticsTest, CalculatesAverageAndPercentiles) {
    drone::common::LatencyStatistics stats(8);
    for (double value = 1.0; value <= 5.0; value += 1.0) {
        stats.Add(value);
    }
    const auto summary = stats.Snapshot();
    EXPECT_EQ(summary.total_count, 5U);
    EXPECT_EQ(summary.window_count, 5U);
    EXPECT_DOUBLE_EQ(summary.minimum_ms, 1.0);
    EXPECT_DOUBLE_EQ(summary.average_ms, 3.0);
    EXPECT_DOUBLE_EQ(summary.p50_ms, 3.0);
    EXPECT_NEAR(summary.p95_ms, 4.8, 1e-9);
    EXPECT_DOUBLE_EQ(summary.maximum_ms, 5.0);
}

TEST(LatencyStatisticsTest, KeepsOnlyLatestWindowAndIgnoresInvalidSamples) {
    drone::common::LatencyStatistics stats(3);
    stats.Add(-1.0);
    stats.Add(1.0);
    stats.Add(2.0);
    stats.Add(3.0);
    stats.Add(4.0);
    const auto summary = stats.Snapshot();
    EXPECT_EQ(summary.total_count, 4U);
    EXPECT_EQ(summary.window_count, 3U);
    EXPECT_DOUBLE_EQ(summary.minimum_ms, 2.0);
    EXPECT_DOUBLE_EQ(summary.maximum_ms, 4.0);

    stats.Reset();
    EXPECT_EQ(stats.Snapshot().total_count, 0U);
}
