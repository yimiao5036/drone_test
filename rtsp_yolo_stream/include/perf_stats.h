#ifndef RTSP_YOLO_STREAM_PERF_STATS_H
#define RTSP_YOLO_STREAM_PERF_STATS_H

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <limits>

// 简单的时间统计累加器（头文件实现，无额外依赖）
struct PerfStats {
    const char* name = nullptr;
    int64_t sum_us = 0;
    int64_t max_us = 0;
    int64_t min_us = std::numeric_limits<int64_t>::max();
    int count = 0;

    void Record(int64_t elapsed_us) {
        sum_us += elapsed_us;
        max_us = std::max(max_us, elapsed_us);
        min_us = std::min(min_us, elapsed_us);
        count++;
    }

    double avg_ms() const { return count > 0 ? (sum_us / 1000.0) / count : 0.0; }
    double max_ms() const { return max_us / 1000.0; }
    double min_ms() const { return min_us / 1000.0; }
    double total_ms() const { return sum_us / 1000.0; }
};

// 重连记录
struct ReconnectStats {
    const char* type = nullptr;
    int count = 0;
    int64_t first_ms = 0;  // 首次重连时间戳（steady_clock ms）
    int64_t last_ms = 0;   // 末次重连时间戳

    void Record(int64_t now_ms) {
        if (count == 0) first_ms = now_ms;
        last_ms = now_ms;
        count++;
    }
};

#endif // RTSP_YOLO_STREAM_PERF_STATS_H