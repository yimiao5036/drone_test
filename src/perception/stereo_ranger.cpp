// =============================================================================
// stereo_ranger.cpp —— 双目测距组件实现
//
// 配对：按 source_time_ms 分组为帧；左帧在挂起右帧中找 |Δt|≤pair_window
// 的最近者配对；左帧超龄（now−t>pair_window）按空右帧处理（valid=false
// 照常发布方位）。每个输出消息在发布时填 header.sequence（本地单调计数器）
// 与 header.receive_time_ms。
//
// 摘要（summary_log_interval 节流）：配对率/有效域内占比/|Δy| P95/测距
// 耗时。|Δy| P95 连续 3 个摘要周期超 tau_y_px → WARN（不 remap 的代价
// 兜底，提示评估升级 remap v1.1）。
//
// 线程模型：独立消费线程以 10ms 节拍阻塞等待左目消息驱动 Pump；Stop()
// 置停止标志 → Reset 双订阅唤醒阻塞等待 → join → core.ResetFilter()。
// 热路径零日志；INFO=生命周期，WARN=降级，ERROR=逻辑缺陷（未接线启动）。
// =============================================================================

#include "perception/stereo_ranger.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::perception {
namespace {

std::int64_t SteadyNowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

/// 一帧的全部检测（按 source_time_ms 聚合）。
struct FrameDetections {
    std::uint64_t source_time_ms = 0;
    std::uint64_t frame_sequence = 0;
    std::vector<common::DetectionResult> detections;
};

}  // namespace

struct StereoRanger::Impl {
    explicit Impl(StereoRangerConfig value) : config(std::move(value)), core(config) {}

    StereoRangerConfig config;
    StereoRangerCore core;

    common::Topic<common::DetectionResult>* left_topic = nullptr;
    common::Topic<common::DetectionResult>* right_topic = nullptr;
    common::Topic<common::DetectionResult>::Subscription left_sub;
    common::Topic<common::DetectionResult>::Subscription right_sub;
    common::Topic<common::StereoTargetDistance> distance_output;

    std::atomic<bool> running{false};
    std::thread thread;

    std::map<std::uint64_t, FrameDetections> pending_left;   // key=source_time_ms
    std::map<std::uint64_t, FrameDetections> pending_right;

    std::atomic<std::uint64_t> processed_pair_count{0};
    std::atomic<std::uint64_t> error_count{0};

    // 以下状态为消费线程独占，无需加锁。
    std::uint64_t publish_sequence = 0;  // 来源内单调递增，跨重启不重置
    double range_time_total_ms = 0.0;
    double range_time_max_ms = 0.0;
    std::uint64_t range_call_count = 0;
    std::int64_t last_summary_ms = 0;
    int consecutive_dy_over = 0;

    /// 把一条检测并入对应帧分组。
    static void AppendTo(std::map<std::uint64_t, FrameDetections>& pending,
                         const common::DetectionResult& d) {
        auto& frame = pending[d.header.source_time_ms];
        frame.source_time_ms = d.header.source_time_ms;
        frame.frame_sequence = d.frame_sequence;
        frame.detections.push_back(d);
    }

    /// 排空订阅队列并入挂起表。
    void DrainInputs() {
        while (auto msg = left_sub.TryTake()) {
            AppendTo(pending_left, **msg);
        }
        while (auto msg = right_sub.TryTake()) {
            AppendTo(pending_right, **msg);
        }
    }

    /// 挂起右帧中找 |Δt|≤pair_window 的最近帧；找到则取出返回。
    std::optional<FrameDetections> TakeNearestRight(std::uint64_t left_time) {
        const std::int64_t window = config.pair_window.count();
        auto best = pending_right.end();
        std::int64_t best_dt = window + 1;
        for (auto it = pending_right.begin(); it != pending_right.end(); ++it) {
            const std::int64_t dt =
                static_cast<std::int64_t>(it->first) -
                static_cast<std::int64_t>(left_time);
            const std::int64_t abs_dt = dt < 0 ? -dt : dt;
            if (abs_dt <= window && abs_dt < best_dt) {
                best_dt = abs_dt;
                best = it;
            }
        }
        if (best == pending_right.end()) {
            return std::nullopt;
        }
        FrameDetections frame = std::move(best->second);
        pending_right.erase(best);
        return frame;
    }

    void ProcessFrame(const FrameDetections& left,
                      const std::vector<common::DetectionResult>& right) {
        const auto start = std::chrono::steady_clock::now();
        auto results = core.Process(left.detections, right,
                                    left.frame_sequence, left.source_time_ms);
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start)
                .count();
        range_time_total_ms += elapsed_ms;
        range_time_max_ms = std::max(range_time_max_ms, elapsed_ms);
        ++range_call_count;

        for (auto& msg : results) {
            msg.header.sequence = ++publish_sequence;
            msg.header.receive_time_ms =
                static_cast<std::uint64_t>(SteadyNowMs());
            (void)distance_output.Emplace(std::move(msg));
        }
        processed_pair_count.fetch_add(1);
    }

    /// 配对 + 左帧超龄处理。
    void Pump() {
        DrainInputs();
        const std::int64_t now_ms = SteadyNowMs();
        const std::int64_t window = config.pair_window.count();
        while (!pending_left.empty()) {
            const auto it = pending_left.begin();
            const std::int64_t age =
                now_ms - static_cast<std::int64_t>(it->first);
            auto right = TakeNearestRight(it->first);
            if (right.has_value()) {
                FrameDetections left = std::move(it->second);
                pending_left.erase(it);
                ProcessFrame(left, right->detections);
            } else if (age > window) {
                FrameDetections left = std::move(it->second);
                pending_left.erase(it);
                ProcessFrame(left, {});  // 超龄未配对：valid=false 照常发布
            } else {
                break;  // 最早的左帧仍在窗内等右目，更新的帧不可能更急
            }
        }
        // 右帧无配对需求（左目无检测帧不发布），超龄右帧直接丢弃
        while (!pending_right.empty() &&
               now_ms - static_cast<std::int64_t>(
                            pending_right.begin()->first) > 4 * window) {
            pending_right.erase(pending_right.begin());
        }
    }

    void MaybeLogSummary(std::int64_t now_ms) {
        if (now_ms - last_summary_ms < config.summary_log_interval.count()) {
            return;
        }
        last_summary_ms = now_ms;
        const std::uint64_t processed = processed_pair_count.load();
        const std::uint64_t matched = core.MatchedPairCount();
        const std::uint64_t valid = core.ValidDistanceCount();
        const double pair_rate =
            processed > 0
                ? 100.0 * static_cast<double>(matched) /
                      static_cast<double>(processed)
                : 0.0;
        const double valid_rate =
            matched > 0
                ? 100.0 * static_cast<double>(valid) /
                      static_cast<double>(matched)
                : 0.0;
        const double avg_ms =
            range_call_count > 0 ? range_time_total_ms / range_call_count : 0.0;
        const double p95 = core.DeltaYP95Px();
        SPDLOG_INFO("双目测距摘要: 左帧={} 匹配对={} 配对率={:.1f}% 有效={} "
                    "有效域内占比={:.1f}% |Δy|P95={:.2f}px "
                    "测距avg={:.2f}ms max={:.2f}ms",
                    processed, matched, pair_rate, valid, valid_rate, p95,
                    avg_ms, range_time_max_ms);
        range_time_total_ms = 0.0;
        range_time_max_ms = 0.0;
        range_call_count = 0;
        // 不 remap 的代价兜底：P95 持续超门限提示评估升级 remap（v1.1）
        if (!std::isnan(p95) && p95 > config.tau_y_px) {
            ++consecutive_dy_over;
            if (consecutive_dy_over >= 3) {
                SPDLOG_WARN("双目测距|Δy| P95={:.2f}px 连续{}个周期超门限"
                            "tau_y={:.1f}px，未remap校正质量不足，"
                            "需评估升级remap（v1.1）",
                            p95, consecutive_dy_over, config.tau_y_px);
            }
        } else {
            consecutive_dy_over = 0;
        }
    }

    void Run() {
        while (running.load()) {
            // 10ms 节拍：阻塞等待左目消息驱动 Pump，同时保证超龄帧及时发布。
            // 取到的消息必须先并入分组，再由 Pump 内 DrainInputs 统一排空，
            // 否则该条消息会丢失分组。
            if (auto msg = left_sub.WaitTakeFor(std::chrono::milliseconds(10))) {
                AppendTo(pending_left, **msg);
            }
            Pump();
            MaybeLogSummary(SteadyNowMs());
        }
    }

    void Stop() {
        if (!running.exchange(false)) {
            return;
        }
        left_sub.Reset();
        right_sub.Reset();
        if (thread.joinable()) {
            thread.join();
        }
    }
};

StereoRanger::StereoRanger(StereoRangerConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
    impl_->config.Validate();
    SPDLOG_INFO("双目测距创建: shadow=true control_output=false "
                "pair_window={}ms 有效域=[{:.1f},{:.1f}]m",
                impl_->config.pair_window.count(),
                impl_->config.distance_min_m, impl_->config.distance_max_m);
}

StereoRanger::~StereoRanger() {
    Stop();
}

bool StereoRanger::Start() {
    if (impl_->running.load()) {
        return false;  // 幂等：状态不变，按接口约定返回 false
    }
    if (impl_->left_topic == nullptr || impl_->right_topic == nullptr) {
        ++impl_->error_count;
        SPDLOG_ERROR("双目测距启动失败: 左右目输入未接线 left={} right={}",
                     impl_->left_topic != nullptr,
                     impl_->right_topic != nullptr);
        return false;
    }
    impl_->left_sub = impl_->left_topic->Subscribe(
        impl_->config.input_queue_capacity,
        common::Topic<common::DetectionResult>::OverflowPolicy::kDropOldest);
    impl_->right_sub = impl_->right_topic->Subscribe(
        impl_->config.input_queue_capacity,
        common::Topic<common::DetectionResult>::OverflowPolicy::kDropOldest);
    impl_->pending_left.clear();
    impl_->pending_right.clear();
    impl_->range_time_total_ms = 0.0;
    impl_->range_time_max_ms = 0.0;
    impl_->range_call_count = 0;
    impl_->last_summary_ms = SteadyNowMs();
    impl_->consecutive_dy_over = 0;
    impl_->running = true;
    impl_->thread = std::thread(&Impl::Run, impl_.get());
    SPDLOG_INFO("双目测距启动: shadow=true pair_window={}ms 队列容量={} "
                "有效域=[{:.1f},{:.1f}]m",
                impl_->config.pair_window.count(),
                impl_->config.input_queue_capacity,
                impl_->config.distance_min_m, impl_->config.distance_max_m);
    return true;
}

void StereoRanger::Stop() {
    const bool was_running = impl_->running.load();
    impl_->Stop();
    if (was_running) {
        impl_->core.ResetFilter();
        SPDLOG_INFO("双目测距停止: 累计左帧={} 匹配对={} 有效={} 错误={}",
                    impl_->processed_pair_count.load(),
                    impl_->core.MatchedPairCount(),
                    impl_->core.ValidDistanceCount(),
                    impl_->error_count.load());
    }
}

bool StereoRanger::IsRunning() const {
    return impl_->running.load();
}

void StereoRanger::SetLeftInput(
    common::Topic<common::DetectionResult>& input) {
    impl_->left_topic = &input;
}

void StereoRanger::SetRightInput(
    common::Topic<common::DetectionResult>& input) {
    impl_->right_topic = &input;
}

common::Topic<common::StereoTargetDistance>& StereoRanger::DistanceOutput() {
    return impl_->distance_output;
}

std::uint64_t StereoRanger::ProcessedPairCount() const {
    return impl_->processed_pair_count.load();
}

std::uint64_t StereoRanger::MatchedPairCount() const {
    return impl_->core.MatchedPairCount();
}

std::uint64_t StereoRanger::ValidDistanceCount() const {
    return impl_->core.ValidDistanceCount();
}

std::uint64_t StereoRanger::ErrorCount() const {
    return impl_->error_count.load();
}

double StereoRanger::DeltaYP95Px() const {
    return impl_->core.DeltaYP95Px();
}

}  // namespace drone::perception
