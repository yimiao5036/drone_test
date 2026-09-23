// =============================================================================
// visual_tracking_shadow.cpp —— 视觉跟踪控制律影子适配层实现
//
// 节拍循环（固定周期，§7.1）：
//   ① 排空两个输入订阅队列取最新（容量 1 + 丢最旧，只需最新状态）；
//   ② 组装 ControlSnapshot：kLocked → tracked=true 并取平滑中心；
//      姿态缺失/超龄时按配置以零姿态兜底（虚拟姿态影子，日志标注）；
//   ③ TrackingControlLaw::Update；
//   ④ ControlOutput → ControlIntent 映射发布（见头文件 reason_code/模式约定）；
//   ⑤ 节流摘要日志（默认 10s）。
//
// 日志纪律：启动/停止/虚拟姿态启用为生命周期 INFO；摘要节流；热路径零日志。
// =============================================================================

#include "control/visual_tracking_shadow.h"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <spdlog/spdlog.h>

namespace drone::control {
namespace {

std::int64_t SteadyNowMs() {
    return static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

// reason_code 影子段（避开骨架期 0-7，见 数据接口文档.md §3.5）
constexpr std::uint32_t kReasonTracking = 100;      // 正常跟踪
constexpr std::uint32_t kReasonCoast = 101;         // Coast 预测跟踪
constexpr std::uint32_t kReasonBrakeHover = 102;    // 判丢刹车
constexpr std::uint32_t kReasonAttitudeStale = 103; // 姿态超龄降级
constexpr std::uint32_t kReasonVisionStale = 104;   // 视觉超龄降级
constexpr std::uint32_t kReasonNoDistanceExit = 105;// 持续无距离退出降级

const char* ModeName(ControlMode mode) {
    switch (mode) {
        case ControlMode::kVelocityHeading: return "速度航向";
        case ControlMode::kBrakeHover:      return "零速刹车";
        case ControlMode::kDegradedHold:    return "降级保持";
    }
    return "未知";
}

}  // namespace

void VisualTrackingShadowConfig::Validate() const {
    if (summary_log_interval.count() <= 0) {
        throw std::invalid_argument("视觉跟踪影子摘要日志间隔必须为正数");
    }
}

struct VisualTrackingShadow::Impl {
    Impl(TrackingConfig tracking, VisualTrackingShadowConfig shadow)
        : tracking_config(std::move(tracking)),
          shadow_config(shadow),
          law(tracking_config) {}

    TrackingConfig tracking_config;
    VisualTrackingShadowConfig shadow_config;
    TrackingControlLaw law;

    common::Topic<common::VisualTargetStatus>* visual_topic = nullptr;
    common::Topic<common::VisualTargetStatus>::Subscription visual_sub;
    common::Topic<common::FlightStateSnapshot>* flight_topic = nullptr;
    common::Topic<common::FlightStateSnapshot>::Subscription flight_sub;
    common::Topic<common::ControlIntent> intent_output;

    std::atomic<bool> running{false};
    std::thread thread;

    std::atomic<std::uint64_t> intent_count{0};
    std::atomic<std::uint64_t> tick_count{0};
    std::atomic<bool> last_virtual_attitude{false};

    // 最新输入快照（节拍线程独占）
    common::VisualTargetStatus last_status;
    bool has_status = false;
    common::FlightStateSnapshot last_flight;
    bool has_flight = false;
    bool virtual_attitude_logged = false;  // 虚拟姿态启用只提示一次

    /// 排空订阅队列取最新消息（容量 1 + 丢最旧，语义上只需最新）。
    template <typename T>
    void DrainLatest(typename common::Topic<T>::Subscription& sub, T& out,
                     bool& has_flag) {
        while (auto msg = sub.TryTake()) {
            out = **msg;
            has_flag = true;
        }
    }

    ControlSnapshot BuildSnapshot(std::int64_t now_ms) {
        ControlSnapshot snapshot;
        // 视觉：kLocked 才算稳定跟踪目标；时间戳取状态消息的接收时刻
        if (has_status) {
            snapshot.track.tracked =
                last_status.state == common::VisualTargetState::kLocked;
            snapshot.track.is_predicted = false;  // 主链路无预测
            snapshot.track.center_x = last_status.center_pixel_x;
            snapshot.track.center_y = last_status.center_pixel_y;
            snapshot.track_time_ms =
                static_cast<std::int64_t>(last_status.header.receive_time_ms);
        }
        // 距离：双目测距未实现，恒无效（走配置的 no_distance_action 支路）
        snapshot.distance_valid = false;
        snapshot.distance_time_ms = 0;
        // 姿态：真实优先；缺失/超龄且配置允许时零姿态兜底（仅供观察）
        const bool attitude_fresh =
            has_flight && last_flight.attitude_valid &&
            last_flight.header.receive_time_ms > 0 &&
            (now_ms - static_cast<std::int64_t>(last_flight.header.receive_time_ms)) <=
                tracking_config.control.attitude_stale_ms;
        if (attitude_fresh) {
            snapshot.attitude_present = true;
            snapshot.attitude_time_ms =
                static_cast<std::int64_t>(last_flight.header.receive_time_ms);
            snapshot.roll_rad = last_flight.roll_rad;
            snapshot.pitch_rad = last_flight.pitch_rad;
            snapshot.yaw_rad = last_flight.yaw_rad;
        } else if (shadow_config.virtual_attitude_when_absent) {
            snapshot.attitude_present = true;
            snapshot.attitude_time_ms = now_ms;
            snapshot.roll_rad = 0.0;
            snapshot.pitch_rad = 0.0;
            snapshot.yaw_rad = 0.0;
            last_virtual_attitude.store(true);
            if (!virtual_attitude_logged) {
                virtual_attitude_logged = true;
                SPDLOG_INFO("视觉跟踪影子：姿态缺失/超龄，启用虚拟零姿态"
                            "（仅供观察控制律行为，不产生真实控制输出）");
            }
            return snapshot;
        }
        last_virtual_attitude.store(false);
        return snapshot;
    }

    /// ControlOutput → ControlIntent 映射（约定见头文件注释）。
    common::ControlIntent MapIntent(const ControlOutput& out, std::int64_t now_ms,
                                    std::uint64_t sequence) {
        common::ControlIntent intent;
        intent.header.sequence = sequence;
        intent.header.source_time_ms = static_cast<std::uint64_t>(now_ms);
        intent.header.receive_time_ms = static_cast<std::uint64_t>(now_ms);
        intent.header.valid_for_ms = static_cast<std::uint64_t>(
            2000.0 / tracking_config.control.frequency_hz);  // 两个控制周期
        intent.header.source_id = 0;
        intent.header.health = 1;
        intent.header.frame_id = 0;
        intent.control_source = 1;  // 1=视觉
        intent.priority = 0;

        switch (out.mode) {
            case ControlMode::kVelocityHeading:
                intent.type = common::ControlIntentType::kVelocityHeading;
                intent.target_x = static_cast<float>(out.vx_mps);
                intent.target_y = static_cast<float>(out.vy_mps);
                intent.target_z = static_cast<float>(out.vz_mps);
                intent.yaw_deg = static_cast<float>(out.yaw_deg);
                intent.yaw_rate_dps = static_cast<float>(out.yaw_rate_dps);
                intent.reason_code =
                    out.coast_active ? kReasonCoast : kReasonTracking;
                break;
            case ControlMode::kBrakeHover:
                intent.type = common::ControlIntentType::kBrakeHover;
                intent.reason_code = kReasonBrakeHover;
                break;
            case ControlMode::kDegradedHold:
                intent.type = common::ControlIntentType::kNone;
                switch (out.degraded_reason) {
                    case DegradedReason::kAttitudeStale:
                        intent.reason_code = kReasonAttitudeStale;
                        break;
                    case DegradedReason::kVisionStale:
                        intent.reason_code = kReasonVisionStale;
                        break;
                    case DegradedReason::kNoDistanceExit:
                        intent.reason_code = kReasonNoDistanceExit;
                        break;
                    default:
                        intent.reason_code = 0;
                        break;
                }
                break;
        }
        return intent;
    }

    void Run() {
        const auto period = std::chrono::duration<double>(
            1.0 / tracking_config.control.frequency_hz);
        auto next_tick = std::chrono::steady_clock::now();
        std::int64_t last_summary_ms = 0;
        ControlOutput last_out;

        while (running.load()) {
            next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                period);
            const std::int64_t now_ms = SteadyNowMs();

            DrainLatest(visual_sub, last_status, has_status);
            DrainLatest(flight_sub, last_flight, has_flight);

            const ControlSnapshot snapshot = BuildSnapshot(now_ms);
            last_out = law.Update(snapshot, now_ms);
            tick_count.fetch_add(1);

            auto intent = std::make_shared<common::ControlIntent>(
                MapIntent(last_out, now_ms, intent_count.load() + 1));
            (void)intent_output.Publish(std::move(intent));
            intent_count.fetch_add(1);

            if (now_ms - last_summary_ms >= shadow_config.summary_log_interval.count()) {
                last_summary_ms = now_ms;
                SPDLOG_INFO("视觉跟踪影子摘要: 模式={} 有效={} vx={:.2f} vy={:.2f} "
                            "vz={:.2f} yaw_rate={:.1f}dps coast={} 虚拟姿态={}",
                            ModeName(last_out.mode), last_out.valid,
                            last_out.vx_mps, last_out.vy_mps, last_out.vz_mps,
                            last_out.yaw_rate_dps, last_out.coast_active,
                            last_virtual_attitude.load());
            }

            std::this_thread::sleep_until(next_tick);
            // 严重滞后（睡眠错过多个周期）时重置节拍基准，避免追帧空转
            if (std::chrono::steady_clock::now() - next_tick >
                std::chrono::milliseconds(500)) {
                next_tick = std::chrono::steady_clock::now();
            }
        }
    }
};

VisualTrackingShadow::VisualTrackingShadow(TrackingConfig tracking_config,
                                           VisualTrackingShadowConfig shadow_config)
    : impl_(std::make_unique<Impl>(std::move(tracking_config), shadow_config)) {
    // 构造期校验（初始化抛异常规范）；Impl 构造完成后这里复核一份副本语义
    impl_->tracking_config.Validate();
    impl_->shadow_config.Validate();
}

VisualTrackingShadow::~VisualTrackingShadow() { Stop(); }

bool VisualTrackingShadow::Start() {
    if (impl_->running.load()) {
        return true;  // 幂等
    }
    if (impl_->visual_topic == nullptr) {
        SPDLOG_ERROR("视觉跟踪影子启动失败：视觉状态输入未接线");
        return false;
    }
    // 容量 1 + 丢最旧：节拍循环只消费最新状态
    impl_->visual_sub = impl_->visual_topic->Subscribe(
        1, common::Topic<common::VisualTargetStatus>::OverflowPolicy::kDropOldest);
    if (impl_->flight_topic != nullptr) {
        impl_->flight_sub = impl_->flight_topic->Subscribe(
            1, common::Topic<common::FlightStateSnapshot>::OverflowPolicy::kDropOldest);
    }
    impl_->running.store(true);
    impl_->thread = std::thread([this] { impl_->Run(); });
    SPDLOG_INFO("视觉跟踪影子启动: 频率={}Hz 虚拟姿态={} 姿态输入={}",
                impl_->tracking_config.control.frequency_hz,
                impl_->shadow_config.virtual_attitude_when_absent,
                impl_->flight_topic != nullptr);
    return true;
}

void VisualTrackingShadow::Stop() {
    if (!impl_->running.exchange(false)) {
        return;  // 幂等
    }
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
    impl_->law.Reset();
    SPDLOG_INFO("视觉跟踪影子停止: 节拍={} 意图={}", impl_->tick_count.load(),
                impl_->intent_count.load());
}

bool VisualTrackingShadow::IsRunning() const { return impl_->running.load(); }

void VisualTrackingShadow::SetVisualInput(
    common::Topic<common::VisualTargetStatus>& input) {
    impl_->visual_topic = &input;
}

void VisualTrackingShadow::SetFlightInput(
    common::Topic<common::FlightStateSnapshot>& input) {
    impl_->flight_topic = &input;
}

common::Topic<common::ControlIntent>& VisualTrackingShadow::IntentOutput() {
    return impl_->intent_output;
}

std::uint64_t VisualTrackingShadow::IntentCount() const {
    return impl_->intent_count.load();
}

std::uint64_t VisualTrackingShadow::TickCount() const {
    return impl_->tick_count.load();
}

bool VisualTrackingShadow::LastUsedVirtualAttitude() const {
    return impl_->last_virtual_attitude.load();
}

}  // namespace drone::control
