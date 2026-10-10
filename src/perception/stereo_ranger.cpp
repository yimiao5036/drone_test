// =============================================================================
// stereo_ranger.cpp —— 双目测距组件实现（v2：稀疏 Census 路线）
//
// 帧/批次配对：
//   - 左右帧进环形缓冲（深度 frame_buffer_depth，持有 Topic 消息指针以占用
//     拆分器内存池槽位，队满挤掉最旧），按 |Δt|≤pair_window 时间邻接配对；
//   - 检测按 frame_sequence 成批：新序号首条到达或 settle 超时即闭合；
//   - 闭合批次按 FIFO 处理，等帧上限 kBatchGiveUpMs；帧缺失/被挤出时该批次
//     每个框仍发布一条 valid=false 消息（保证下游每目标有节拍）。
//
// 摘要（summary_log_interval 节流）：帧数/匹配帧/有效占比/丢帧/截断/耗时。
// 事件日志节流：第 1 次与每满 100 次（丢帧 WARN、截断 WARN）。
//
// 线程模型：独立消费线程以 10ms 节拍阻塞等待左目检测驱动 Pump；Stop()
// 置停止标志 → Reset 三路订阅唤醒阻塞等待 → join → core.Reset()。
// 热路径零日志；INFO=生命周期，WARN=降级，ERROR=逻辑缺陷（未接线启动）。
// =============================================================================

#include "perception/stereo_ranger.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
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

/// 批次等待配对帧的放弃时限：超过即按无效发布，避免队首长期阻塞。
constexpr std::int64_t kBatchGiveUpMs = 200;

/// 事件日志节流：第 1 次与每满 100 次。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

}  // namespace

struct StereoRanger::Impl {
    Impl(StereoRangerConfig value, StereoRectifyMatrices value_matrices)
        : config(std::move(value)),
          matrices(value_matrices),
          core(config, matrices) {}

    StereoRangerConfig config;
    StereoRectifyMatrices matrices;
    StereoRangerCore core;

    common::Topic<common::DetectionResult>* detection_topic = nullptr;
    common::Topic<video::FrameHandle>* left_topic = nullptr;
    common::Topic<video::FrameHandle>* right_topic = nullptr;
    common::Topic<common::DetectionResult>::Subscription detection_sub;
    common::Topic<video::FrameHandle>::Subscription left_sub;
    common::Topic<video::FrameHandle>::Subscription right_sub;
    common::Topic<common::StereoTargetDistance> distance_output;

    /// 帧条目：持有 Topic 消息指针（shared_ptr<const FrameHandle>），
    /// 从而在缓冲期内占用拆分器内存池槽位；条目移除即归还槽位。
    using FrameMessage = common::Topic<video::FrameHandle>::MessagePtr;
    struct FrameEntry {
        FrameMessage message;
        std::uint64_t sequence = 0;
        std::int64_t timestamp_ms = 0;
    };
    /// 检测批次：同一 frame_sequence 的全部左目检测。
    struct Batch {
        std::uint64_t sequence = 0;
        std::int64_t deadline_ms = 0;  ///< 超时闭合时刻（steady ms）
        std::vector<common::DetectionResult> boxes;
    };

    std::atomic<bool> running{false};
    std::thread thread;

    std::atomic<std::uint64_t> batch_count{0};
    std::atomic<std::uint64_t> published_count{0};
    std::atomic<std::uint64_t> frame_miss_count{0};
    std::atomic<std::uint64_t> error_count{0};

    // 以下状态为消费线程独占，无需加锁。
    std::deque<FrameEntry> left_frames;
    std::deque<FrameEntry> right_frames;
    std::optional<Batch> open_batch;
    std::deque<Batch> ready_batches;
    std::uint64_t publish_sequence = 0;
    double match_time_total_ms = 0.0;
    double match_time_max_ms = 0.0;
    std::uint64_t match_call_count = 0;
    std::int64_t last_summary_ms = 0;
    std::uint64_t clipped_log_count = 0;
    // 摘要窗口内的有效距离统计（板上卷尺验收判读依据）
    double distance_sum_m = 0.0;
    double distance_min_m = 0.0;
    double distance_max_m = 0.0;
    std::uint64_t distance_sample_count = 0;

    // ---- 帧缓冲 -------------------------------------------------------------

    void PushFrame(std::deque<FrameEntry>& buffer, FrameMessage message) {
        const video::VideoFrameInfo& info = message->Info();
        FrameEntry entry;
        entry.sequence = info.sequence;
        entry.timestamp_ms = info.timestamp_ms;
        entry.message = std::move(message);
        buffer.push_back(std::move(entry));
        while (buffer.size() > config.frame_buffer_depth) {
            buffer.pop_front();  // 挤掉最旧：释放其内存池槽位
        }
    }

    void DrainFrames() {
        while (auto msg = left_sub.TryTake()) {
            PushFrame(left_frames, std::move(*msg));
        }
        while (auto msg = right_sub.TryTake()) {
            PushFrame(right_frames, std::move(*msg));
        }
    }

    using FrameIt = std::deque<FrameEntry>::iterator;

    FrameIt FindLeftFrame(std::uint64_t sequence) {
        for (auto it = left_frames.begin(); it != left_frames.end(); ++it) {
            if (it->sequence == sequence) {
                return it;
            }
        }
        return left_frames.end();
    }

    /// 右帧中找 |Δt|≤pair_window 的最近帧。
    FrameIt FindRightFrame(std::int64_t left_timestamp_ms) {
        const std::int64_t window = config.pair_window.count();
        auto best = right_frames.end();
        std::int64_t best_dt = window + 1;
        for (auto it = right_frames.begin(); it != right_frames.end(); ++it) {
            const std::int64_t dt = it->timestamp_ms - left_timestamp_ms;
            const std::int64_t abs_dt = dt < 0 ? -dt : dt;
            if (abs_dt <= window && abs_dt < best_dt) {
                best_dt = abs_dt;
                best = it;
            }
        }
        return best;
    }

    // ---- 检测批次 -----------------------------------------------------------

    void AddDetection(const common::DetectionResult& detection) {
        const std::int64_t now = SteadyNowMs();
        if (!open_batch || open_batch->sequence != detection.frame_sequence) {
            if (open_batch) {
                ready_batches.push_back(std::move(*open_batch));  // 新序号 ⇒ 上批闭合
                open_batch.reset();
            }
            open_batch = Batch{};
            open_batch->sequence = detection.frame_sequence;
        }
        open_batch->deadline_ms = now + config.settle.count();
        open_batch->boxes.push_back(detection);
    }

    /// 批次闭合（新序号已在 AddDetection 处理；此处只处理超时）。
    void CloseTimedOutBatch(std::int64_t now) {
        if (open_batch && now >= open_batch->deadline_ms) {
            ready_batches.push_back(std::move(*open_batch));
            open_batch.reset();
        }
    }

    // ---- 发布 ---------------------------------------------------------------

    void PublishMessages(const Batch& batch,
                         const std::vector<StereoRangeResult>& results,
                         std::int64_t source_time_ms) {
        const std::uint64_t now =
            static_cast<std::uint64_t>(SteadyNowMs());
        for (std::size_t i = 0; i < batch.boxes.size(); ++i) {
            const common::DetectionResult& box = batch.boxes[i];
            const StereoRangeResult& range = results[i];
            common::StereoTargetDistance msg;
            msg.header.sequence = ++publish_sequence;
            msg.header.source_time_ms = static_cast<std::uint64_t>(source_time_ms);
            msg.header.receive_time_ms = now;
            msg.frame_sequence = batch.sequence;
            msg.class_id = box.class_id;
            msg.track_id = 0;  // v2 单/少目标约定，多目标关联留后续
            msg.confidence = box.confidence;
            msg.bbox_x = box.bbox_x;
            msg.bbox_y = box.bbox_y;
            msg.bbox_w = box.bbox_w;
            msg.bbox_h = box.bbox_h;
            msg.center_pixel_x_mid = box.bbox_x + 0.5f * box.bbox_w;
            if (range.available) {
                msg.forward_distance_m = static_cast<float>(range.z_m);
                msg.slant_range_m = static_cast<float>(range.euclid_m);
                msg.disparity_px = static_cast<float>(range.disparity_px);
                msg.center_pixel_x_mid = static_cast<float>(range.center_x_rect);
                msg.valid = true;
                if (distance_sample_count == 0) {
                    distance_min_m = range.z_m;
                    distance_max_m = range.z_m;
                } else {
                    distance_min_m = std::min(distance_min_m, range.z_m);
                    distance_max_m = std::max(distance_max_m, range.z_m);
                }
                distance_sum_m += range.z_m;
                ++distance_sample_count;
                if (range.clipped) {
                    ++clipped_log_count;
                    if (ShouldLogThrottled(clipped_log_count)) {
                        SPDLOG_WARN(
                            "双目测距视差顶到搜索上限(截断): d={:.1f}px "
                            "上限={:.0f}px，距离可能更近（累计 {}）",
                            range.disparity_px, config.disp_max_px,
                            clipped_log_count);
                    }
                }
            }
            (void)distance_output.Emplace(std::move(msg));
            ++published_count;
        }
    }

    /// 帧缺失批次：每个框发布一条 valid=false（NaN 距离），保持节拍。
    void PublishUnavailable(const Batch& batch, const char* reason) {
        const std::vector<StereoRangeResult> empty(batch.boxes.size());
        PublishMessages(batch, empty, 0);
        const std::uint64_t count = frame_miss_count.fetch_add(1) + 1;
        if (ShouldLogThrottled(count)) {
            SPDLOG_WARN("双目测距批次未配对到帧({}): seq={} 框数={}（累计 {}）",
                        reason, batch.sequence, batch.boxes.size(), count);
        }
    }

    // ---- 主流程 -------------------------------------------------------------

    void ProcessBatch(const Batch& batch, const FrameEntry& left,
                      const FrameEntry& right) {
        const video::VideoFrameInfo& left_info = left.message->Info();
        const video::VideoFrameInfo& right_info = right.message->Info();
        const GrayImageView ref{
            reinterpret_cast<const std::uint8_t*>(left.message->Data()),
            static_cast<int>(left_info.width), static_cast<int>(left_info.height),
            static_cast<int>(left_info.hor_stride)};
        const GrayImageView aux{
            reinterpret_cast<const std::uint8_t*>(right.message->Data()),
            static_cast<int>(right_info.width), static_cast<int>(right_info.height),
            static_cast<int>(right_info.hor_stride)};

        const auto start = std::chrono::steady_clock::now();
        const std::vector<StereoRangeResult> results =
            core.Process(ref, aux, batch.boxes);
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start)
                .count();
        match_time_total_ms += elapsed_ms;
        match_time_max_ms = std::max(match_time_max_ms, elapsed_ms);
        ++match_call_count;

        PublishMessages(batch, results,
                        static_cast<std::int64_t>(left_info.timestamp_ms));
        batch_count.fetch_add(1);
    }

    void Pump() {
        DrainFrames();
        const std::int64_t now = SteadyNowMs();
        CloseTimedOutBatch(now);

        while (!ready_batches.empty()) {
            Batch& batch = ready_batches.front();
            const FrameIt left = FindLeftFrame(batch.sequence);
            if (left == left_frames.end()) {
                // 左帧被挤出（缓冲最旧序号已超过）或等待超时 ⇒ 放弃该批次
                const bool evicted =
                    !left_frames.empty() && left_frames.front().sequence > batch.sequence;
                if (evicted || now - batch.deadline_ms > kBatchGiveUpMs) {
                    PublishUnavailable(batch, "左帧缺失/被挤出");
                    ready_batches.pop_front();
                    continue;
                }
                break;  // 等左帧
            }
            const FrameIt right = FindRightFrame(left->timestamp_ms);
            if (right == right_frames.end()) {
                // 右帧不会再来：右缓冲最新帧已超过左帧时间 + 配对窗
                const bool missing =
                    !right_frames.empty() &&
                    right_frames.back().timestamp_ms >
                        left->timestamp_ms + config.pair_window.count();
                if (missing || now - batch.deadline_ms > kBatchGiveUpMs) {
                    PublishUnavailable(batch, "右帧缺失/被挤出");
                    left_frames.erase(left);
                    ready_batches.pop_front();
                    continue;
                }
                break;  // 等右帧
            }
            ProcessBatch(batch, *left, *right);
            left_frames.erase(left);
            right_frames.erase(right);
            ready_batches.pop_front();
        }
    }

    void MaybeLogSummary(std::int64_t now_ms) {
        if (now_ms - last_summary_ms < config.summary_log_interval.count()) {
            return;
        }
        last_summary_ms = now_ms;
        const std::uint64_t frames = core.ProcessedFrameCount();
        const std::uint64_t matched = core.MatchedFrameCount();
        const std::uint64_t available = core.AvailableCount();
        const std::uint64_t published = published_count.load();
        const std::uint64_t skipped = core.SkippedFrameCount();
        const std::uint64_t clipped = core.ClippedCount();
        const std::uint64_t misses = frame_miss_count.load();
        const double valid_rate = published > 0
                                      ? 100.0 * static_cast<double>(available) /
                                            static_cast<double>(published)
                                      : 0.0;
        const double avg_ms =
            match_call_count > 0 ? match_time_total_ms / match_call_count : 0.0;
        if (distance_sample_count > 0) {
            SPDLOG_INFO("双目测距摘要: 帧={} 匹配帧={} 批次={} 发布={} 有效测距={} "
                        "有效占比={:.1f}% 距离 min/avg/max={:.3f}/{:.3f}/{:.3f}m "
                        "跳过={} 帧未配={} 截断={} 处理avg={:.2f}ms max={:.2f}ms",
                        frames, matched, batch_count.load(), published, available,
                        valid_rate, distance_min_m,
                        distance_sum_m / static_cast<double>(distance_sample_count),
                        distance_max_m, skipped, misses, clipped, avg_ms,
                        match_time_max_ms);
        } else {
            SPDLOG_INFO("双目测距摘要: 帧={} 匹配帧={} 批次={} 发布={} 有效测距={} "
                        "有效占比={:.1f}% 跳过={} 帧未配={} 截断={} "
                        "处理avg={:.2f}ms max={:.2f}ms",
                        frames, matched, batch_count.load(), published, available,
                        valid_rate, skipped, misses, clipped, avg_ms,
                        match_time_max_ms);
        }
        // 阶段耗时细分（滚动窗口 avg/p95，开发机基准与板上定位热点用）
        const StereoStageLatency stage = core.StageLatency();
        if (stage.remap.window_count > 0) {
            SPDLOG_INFO(
                "双目测距阶段耗时 avg/p95(ms,n={}): remap={:.1f}/{:.1f} "
                "census={:.1f}/{:.1f} 提点={:.1f}/{:.1f} 匹配={:.1f}/{:.1f} "
                "聚合={:.1f}/{:.1f}",
                stage.remap.window_count, stage.remap.average_ms,
                stage.remap.p95_ms, stage.census.average_ms, stage.census.p95_ms,
                stage.keypoints.average_ms, stage.keypoints.p95_ms,
                stage.match.average_ms, stage.match.p95_ms,
                stage.aggregate.average_ms, stage.aggregate.p95_ms);
        }
        match_time_total_ms = 0.0;
        match_time_max_ms = 0.0;
        match_call_count = 0;
        distance_sum_m = 0.0;
        distance_sample_count = 0;
    }

    void Run() {
        while (running.load()) {
            // 10ms 节拍：阻塞等待左目检测驱动；同时保证帧排空/批次超时闭合及时。
            if (auto msg = detection_sub.WaitTakeFor(std::chrono::milliseconds(10))) {
                AddDetection(**msg);
            }
            Pump();
            MaybeLogSummary(SteadyNowMs());
        }
    }

    void Stop() {
        if (!running.exchange(false)) {
            return;
        }
        detection_sub.Reset();
        left_sub.Reset();
        right_sub.Reset();
        if (thread.joinable()) {
            thread.join();
        }
    }
};

StereoRanger::StereoRanger(StereoRangerConfig config,
                           StereoRectifyMatrices matrices)
    : impl_(std::make_unique<Impl>(std::move(config), matrices)) {
    SPDLOG_INFO("双目测距v2创建: shadow=true 基准图=SBS右半(物理左目) "
                "fx_rect={:.2f}px 基线={:.3f}mm fxB={:.2f}m·px 视差=[{:.0f},{:.0f}]px",
                impl_->matrices.fx_rect_px, impl_->matrices.baseline_m * 1000.0,
                impl_->matrices.fxb_m_px, impl_->config.disp_min_px,
                impl_->config.disp_max_px);
}

StereoRanger::~StereoRanger() { Stop(); }

bool StereoRanger::Start() {
    if (impl_->running.load()) {
        return false;  // 幂等：状态不变，按接口约定返回 false
    }
    if (impl_->detection_topic == nullptr || impl_->left_topic == nullptr ||
        impl_->right_topic == nullptr) {
        impl_->error_count.fetch_add(1);
        SPDLOG_ERROR("双目测距启动失败: 输入未接线 detection={} left={} right={}",
                     impl_->detection_topic != nullptr,
                     impl_->left_topic != nullptr,
                     impl_->right_topic != nullptr);
        return false;
    }
    impl_->detection_sub = impl_->detection_topic->Subscribe(
        impl_->config.input_queue_capacity,
        common::Topic<common::DetectionResult>::OverflowPolicy::kDropOldest);
    impl_->left_sub = impl_->left_topic->Subscribe(
        impl_->config.input_queue_capacity,
        common::Topic<video::FrameHandle>::OverflowPolicy::kDropOldest);
    impl_->right_sub = impl_->right_topic->Subscribe(
        impl_->config.input_queue_capacity,
        common::Topic<video::FrameHandle>::OverflowPolicy::kDropOldest);
    impl_->left_frames.clear();
    impl_->right_frames.clear();
    impl_->open_batch.reset();
    impl_->ready_batches.clear();
    impl_->match_time_total_ms = 0.0;
    impl_->match_time_max_ms = 0.0;
    impl_->match_call_count = 0;
    impl_->clipped_log_count = 0;
    impl_->last_summary_ms = SteadyNowMs();
    impl_->running = true;
    impl_->thread = std::thread(&Impl::Run, impl_.get());
    SPDLOG_INFO("双目测距启动: shadow=true 配对窗={}ms 批次闭合={}ms 帧缓冲={} "
                "队列容量={}",
                impl_->config.pair_window.count(), impl_->config.settle.count(),
                impl_->config.frame_buffer_depth,
                impl_->config.input_queue_capacity);
    return true;
}

void StereoRanger::Stop() {
    const bool was_running = impl_->running.load();
    impl_->Stop();
    if (was_running) {
        impl_->core.Reset();
        SPDLOG_INFO("双目测距停止: 批次={} 发布={} 有效={} 帧未配={} 截断={}",
                    impl_->batch_count.load(), impl_->published_count.load(),
                    impl_->core.AvailableCount(), impl_->frame_miss_count.load(),
                    impl_->core.ClippedCount());
    }
}

bool StereoRanger::IsRunning() const { return impl_->running.load(); }

void StereoRanger::SetDetectionInput(
    common::Topic<common::DetectionResult>& input) {
    impl_->detection_topic = &input;
}

void StereoRanger::SetLeftFrameInput(common::Topic<video::FrameHandle>& input) {
    impl_->left_topic = &input;
}

void StereoRanger::SetRightFrameInput(common::Topic<video::FrameHandle>& input) {
    impl_->right_topic = &input;
}

common::Topic<common::StereoTargetDistance>& StereoRanger::DistanceOutput() {
    return impl_->distance_output;
}

std::uint64_t StereoRanger::ProcessedFrameCount() const {
    return impl_->core.ProcessedFrameCount();
}
std::uint64_t StereoRanger::MatchedFrameCount() const {
    return impl_->core.MatchedFrameCount();
}
std::uint64_t StereoRanger::AvailableCount() const {
    return impl_->core.AvailableCount();
}
std::uint64_t StereoRanger::ClippedCount() const {
    return impl_->core.ClippedCount();
}
std::uint64_t StereoRanger::SkippedFrameCount() const {
    return impl_->core.SkippedFrameCount();
}
std::uint64_t StereoRanger::BatchCount() const {
    return impl_->batch_count.load();
}
std::uint64_t StereoRanger::FrameMissCount() const {
    return impl_->frame_miss_count.load();
}
std::uint64_t StereoRanger::ErrorCount() const {
    return impl_->error_count.load();
}
double StereoRanger::LastMatchMs() const { return impl_->core.LastMatchMs(); }

}  // namespace drone::perception
