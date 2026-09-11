// =============================================================================
// visual_target_monitor.cpp —— 单目标视觉稳定性判定实现
//
// 状态机（每收到一条观测推进一次，另按周期检查观测超时）：
//
//   收到 detected=true 观测：
//     consecutive_detected++ , consecutive_missed=0
//     consecutive_detected >= lock_frames  → LOCKED
//     否则                                  → ACQUIRING
//
//   收到 detected=false 观测：
//     consecutive_detected=0 , consecutive_missed++
//     consecutive_missed >= lost_frames     → LOST
//     否则若当前不是 LOCKED                  → SEARCHING
//     否则（LOCKED 且未超丢失门限）           → 保持 LOCKED（容忍短暂漏检）
//
//   超过 observation_timeout 未收到任何观测：
//     → LOST（数据中断，区别于“本帧无目标”）
//
// 该状态机只描述“视觉是否稳定看到目标”，不代表目标已被击落或离开。
// =============================================================================

#include "perception/visual_target_monitor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

namespace drone::perception {
namespace {

uint64_t SteadyNowMs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

uint64_t SaturatingAddMs(uint64_t base_ms, uint64_t delta_ms) {
    const uint64_t maximum = std::numeric_limits<uint64_t>::max();
    return delta_ms > maximum - base_ms ? maximum : base_ms + delta_ms;
}

/// 异常日志节流：第 1 次与每满 100 次才打印，避免高频异常刷屏。
bool ShouldLogThrottled(uint64_t count) {
    return count == 1 || count % 100 == 0;
}

}  // namespace

void VisualTargetMonitorConfig::Validate() const {
    if (input_queue_capacity == 0) {
        throw std::invalid_argument("视觉稳定性判定订阅队列容量必须为正数");
    }
    if (publish_interval.count() <= 0) {
        throw std::invalid_argument("视觉稳定性判定发布周期必须为正数");
    }
    if (observation_timeout.count() <= 0) {
        throw std::invalid_argument("视觉稳定性判定观测超时必须为正数");
    }
    if (lock_frames <= 0) {
        throw std::invalid_argument("视觉稳定性判定锁定帧数必须为正数");
    }
    if (lost_frames <= 0) {
        throw std::invalid_argument("视觉稳定性判定丢失帧数必须为正数");
    }
    if (!(smoothing_alpha > 0.0) || smoothing_alpha > 1.0) {
        throw std::invalid_argument("视觉稳定性判定平滑系数必须在(0,1]");
    }
}

struct VisualTargetMonitor::Impl {
    explicit Impl(VisualTargetMonitorConfig value) : config(std::move(value)) {}

    VisualTargetMonitorConfig config;
    common::Topic<common::VisualTargetObservation>* input_topic = nullptr;
    common::Topic<common::VisualTargetObservation>::Subscription input_sub;
    common::Topic<common::VisualTargetStatus> status_output;

    std::atomic<bool> running{false};
    std::thread thread;

    std::atomic<uint64_t> observation_count{0};
    std::atomic<uint64_t> error_count{0};
    uint64_t status_sequence = 0;

    // 以下状态为消费线程独占，无需加锁。
    common::VisualTargetState state = common::VisualTargetState::kSearching;
    int consecutive_detected = 0;
    int consecutive_missed = 0;
    bool smoothing_initialized = false;
    double smoothed_cx = 0.0;
    double smoothed_cy = 0.0;
    double smoothed_confidence = 0.0;
    uint64_t last_seen_ms = 0;             // 最近一次 detected=true 的时刻
    uint64_t last_observation_ms = 0;      // 最近一次任意观测到达时刻
    uint64_t frame_sequence = 0;
    uint32_t frame_width = 0;
    uint32_t frame_height = 0;

    mutable std::mutex last_status_mutex;
    common::VisualTargetStatus last_status;

    void ResetState() {
        state = common::VisualTargetState::kSearching;
        consecutive_detected = 0;
        consecutive_missed = 0;
        smoothing_initialized = false;
        smoothed_cx = 0.0;
        smoothed_cy = 0.0;
        smoothed_confidence = 0.0;
        last_seen_ms = 0;
        last_observation_ms = 0;
        frame_sequence = 0;
        frame_width = 0;
        frame_height = 0;
    }

    /// 平滑目标中心与置信度。首帧直接采用观测值，避免从0开始收敛。
    void UpdateSmoothing(double cx, double cy, double confidence) {
        const double alpha = config.smoothing_alpha;
        if (!smoothing_initialized) {
            smoothed_cx = cx;
            smoothed_cy = cy;
            smoothed_confidence = confidence;
            smoothing_initialized = true;
            return;
        }
        smoothed_cx = smoothed_cx * (1.0 - alpha) + cx * alpha;
        smoothed_cy = smoothed_cy * (1.0 - alpha) + cy * alpha;
        smoothed_confidence =
            smoothed_confidence * (1.0 - alpha) + confidence * alpha;
    }

    void HandleObservation(const common::VisualTargetObservation& observation) {
        const uint64_t now_ms = SteadyNowMs();
        ++observation_count;
        last_observation_ms = now_ms;
        frame_sequence = observation.frame_sequence;
        frame_width = observation.frame_width;
        frame_height = observation.frame_height;

        // 多候选只作诊断：当前按单目标约定已在上游取置信度最高者。
        if (observation.candidate_count > 1) {
            const uint64_t count = ++error_count;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN(
                    "视觉稳定性判定收到多目标候选，按置信度最高者处理: candidates={} count={}",
                    observation.candidate_count, count);
            }
        }

        if (observation.detected) {
            consecutive_detected += 1;
            consecutive_missed = 0;
            UpdateSmoothing(observation.center_pixel_x, observation.center_pixel_y,
                            observation.confidence);
            last_seen_ms = now_ms;
            state = consecutive_detected >= config.lock_frames
                        ? common::VisualTargetState::kLocked
                        : common::VisualTargetState::kAcquiring;
            return;
        }

        // 本帧推理完成但没有目标：这是有效的丢失证据，与数据中断不同。
        consecutive_detected = 0;
        consecutive_missed += 1;
        if (consecutive_missed >= config.lost_frames) {
            state = common::VisualTargetState::kLost;
        } else if (state != common::VisualTargetState::kLocked) {
            // 未锁定时的漏检直接退回搜索；已锁定则容忍到丢失门限。
            state = common::VisualTargetState::kSearching;
        }
    }

    /// 观测超时检查：YOLO线程停止或卡死时，本消息流会整体中断。
    void CheckObservationTimeout(uint64_t now_ms) {
        if (last_observation_ms == 0) {
            return;  // 尚未收到任何观测，保持初始 SEARCHING
        }
        if (now_ms - last_observation_ms <
            static_cast<uint64_t>(config.observation_timeout.count())) {
            return;
        }
        if (state != common::VisualTargetState::kLost) {
            state = common::VisualTargetState::kLost;
            consecutive_detected = 0;
            const uint64_t count = ++error_count;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN(
                    "视觉观测超时，判定目标丢失: last_age_ms={} count={}",
                    now_ms - last_observation_ms, count);
            }
        }
    }

    common::VisualTargetStatus BuildStatus(uint64_t now_ms) const {
        common::VisualTargetStatus status;
        status.header.sequence = status_sequence + 1;
        status.header.source_time_ms = now_ms;
        status.header.receive_time_ms = now_ms;
        status.header.source_id = 0;
        status.header.health = 1;
        status.header.frame_id = 0;  // 像素坐标，不属于WGS84/NED等空间坐标系
        status.state = state;
        status.frame_sequence = frame_sequence;
        status.last_seen_ms = last_seen_ms;
        status.consecutive_detected_frames = consecutive_detected;
        status.consecutive_missed_frames = consecutive_missed;
        status.confidence = static_cast<float>(smoothed_confidence);
        status.center_pixel_x = static_cast<float>(smoothed_cx);
        status.center_pixel_y = static_cast<float>(smoothed_cy);
        status.frame_width = frame_width;
        status.frame_height = frame_height;
        status.valid = state == common::VisualTargetState::kLocked;
        return status;
    }

    void PublishStatus(uint64_t now_ms) {
        common::VisualTargetStatus status = BuildStatus(now_ms);
        ++status_sequence;
        {
            std::lock_guard<std::mutex> lock(last_status_mutex);
            last_status = status;
        }
        (void)status_output.Publish(
            std::make_shared<const common::VisualTargetStatus>(std::move(status)));
    }

    void Run() {
        // 周期取值同时作为订阅等待上限：既保证发布节奏，也保证停止标志能被及时检查。
        const auto wait_span = std::min(
            config.publish_interval, std::chrono::milliseconds(20));
        uint64_t next_publish_ms = SteadyNowMs();
        while (running.load()) {
            const auto message = input_sub.WaitTakeFor(wait_span);
            if (message) {
                HandleObservation(**message);
            }
            const uint64_t now_ms = SteadyNowMs();
            CheckObservationTimeout(now_ms);
            if (now_ms >= next_publish_ms) {
                PublishStatus(now_ms);
                next_publish_ms = SaturatingAddMs(
                    now_ms,
                    static_cast<uint64_t>(config.publish_interval.count()));
            }
        }
    }

    void Stop() {
        if (!running.exchange(false)) {
            return;
        }
        input_sub.Reset();
        if (thread.joinable()) {
            thread.join();
        }
    }
};

VisualTargetMonitor::VisualTargetMonitor(VisualTargetMonitorConfig config) {
    config.Validate();
    impl_ = std::make_unique<Impl>(std::move(config));
    SPDLOG_INFO("视觉稳定性判定创建: shadow=true control_output=false lock_frames={} lost_frames={}",
                impl_->config.lock_frames, impl_->config.lost_frames);
}

VisualTargetMonitor::~VisualTargetMonitor() {
    impl_->Stop();
    SPDLOG_INFO("视觉稳定性判定销毁");
}

bool VisualTargetMonitor::Start() {
    if (impl_->running.load()) {
        return true;
    }
    if (impl_->input_topic == nullptr) {
        ++impl_->error_count;
        SPDLOG_ERROR("视觉稳定性判定启动失败: 观测输入未绑定");
        return false;
    }
    if (!impl_->input_sub.IsOpen()) {
        impl_->input_sub = impl_->input_topic->Subscribe(
            impl_->config.input_queue_capacity);
    }
    impl_->ResetState();
    {
        std::lock_guard<std::mutex> lock(impl_->last_status_mutex);
        impl_->last_status = common::VisualTargetStatus{};
    }
    impl_->running = true;
    impl_->thread = std::thread(&Impl::Run, impl_.get());
    SPDLOG_INFO("视觉稳定性判定启动: shadow=true");
    return true;
}

void VisualTargetMonitor::Stop() {
    const bool was_running = impl_->running.load();
    impl_->Stop();
    if (was_running) {
        SPDLOG_INFO("视觉稳定性判定停止");
    }
}

bool VisualTargetMonitor::IsRunning() const {
    return impl_->running.load();
}

void VisualTargetMonitor::SetInput(
    common::Topic<common::VisualTargetObservation>& input) {
    impl_->input_topic = &input;
    impl_->input_sub = input.Subscribe(impl_->config.input_queue_capacity);
}

common::Topic<common::VisualTargetStatus>& VisualTargetMonitor::StatusOutput() {
    return impl_->status_output;
}

common::VisualTargetStatus VisualTargetMonitor::LastStatus() const {
    std::lock_guard<std::mutex> lock(impl_->last_status_mutex);
    return impl_->last_status;
}

uint64_t VisualTargetMonitor::ObservationCount() const {
    return impl_->observation_count.load();
}

uint64_t VisualTargetMonitor::ErrorCount() const {
    return impl_->error_count.load();
}

}  // namespace drone::perception
