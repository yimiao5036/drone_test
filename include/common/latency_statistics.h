#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace drone::common {

/// 固定窗口延迟统计快照。所有数值单位均为毫秒。
struct LatencySummary {
    std::uint64_t total_count = 0;
    std::size_t window_count = 0;
    double average_ms = 0.0;
    double minimum_ms = 0.0;
    double p50_ms = 0.0;
    double p95_ms = 0.0;
    double p99_ms = 0.0;
    double maximum_ms = 0.0;
};

/// 固定容量滑动窗口延迟统计器。Add热路径在初始化后不分配内存；Snapshot才复制并排序。
class LatencyStatistics final {
public:
    explicit LatencyStatistics(std::size_t capacity = 2048)
        : samples_(capacity, 0.0) {
        if (capacity == 0) {
            throw std::invalid_argument("延迟统计窗口容量必须大于0");
        }
    }

    void Add(double latency_ms) {
        if (!std::isfinite(latency_ms) || latency_ms < 0.0) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        samples_[next_index_] = latency_ms;
        next_index_ = (next_index_ + 1) % samples_.size();
        if (window_count_ < samples_.size()) {
            ++window_count_;
        }
        ++total_count_;
    }

    [[nodiscard]] LatencySummary Snapshot() const {
        std::vector<double> values;
        std::uint64_t total_count = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            total_count = total_count_;
            values.assign(samples_.begin(), samples_.begin() + window_count_);
        }

        LatencySummary result;
        result.total_count = total_count;
        result.window_count = values.size();
        if (values.empty()) {
            return result;
        }
        std::sort(values.begin(), values.end());
        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        result.average_ms = sum / static_cast<double>(values.size());
        result.minimum_ms = values.front();
        result.p50_ms = Percentile(values, 0.50);
        result.p95_ms = Percentile(values, 0.95);
        result.p99_ms = Percentile(values, 0.99);
        result.maximum_ms = values.back();
        return result;
    }

    void Reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        next_index_ = 0;
        window_count_ = 0;
        total_count_ = 0;
    }

private:
    static double Percentile(const std::vector<double>& sorted, double fraction) {
        const double position = fraction * static_cast<double>(sorted.size() - 1);
        const auto lower = static_cast<std::size_t>(position);
        const auto upper = std::min(lower + 1, sorted.size() - 1);
        const double weight = position - static_cast<double>(lower);
        return sorted[lower] * (1.0 - weight) + sorted[upper] * weight;
    }

    mutable std::mutex mutex_;
    std::vector<double> samples_;
    std::size_t next_index_ = 0;
    std::size_t window_count_ = 0;
    std::uint64_t total_count_ = 0;
};

}  // namespace drone::common
