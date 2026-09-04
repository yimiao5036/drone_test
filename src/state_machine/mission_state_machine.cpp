/**
 * @file mission_state_machine.cpp
 * @brief MissionStateMachine 安全影子实现
 *
 * 当前阶段只接入地面站目标与飞行状态，发布 MissionStatus 供链路验证；
 * 不发布真实 ControlIntent，不连接 PX4 控制输出。
 */
#include "state_machine/mission_state_machine.h"

#include <atomic>
#include <chrono>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

namespace drone::state_machine {
namespace {

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

/// 状态机工作周期。当前影子阶段只做状态/新鲜度判断，100ms足够且不会造成CPU压力。
constexpr auto kDecisionPeriod = 100ms;
/// MissionStatus周期回传间隔。状态切换会立即发布，周期发布用于地面站持续显示。
constexpr uint64_t kStatusPublishIntervalMs = 500;
/// MissionStatus有效期，地面站可据此判断状态回传是否过期。
constexpr uint64_t kStatusValidForMs = 1000;
/// 告警位：尚未收到满足GPS引导要求的PX4导航快照。
constexpr uint32_t kWarningNavigationUnavailable = 1U << 0U;
/// 告警位：已经进入GPS_APPROACH影子状态，但最新目标超过剩余有效期。
constexpr uint32_t kWarningGroundTargetStale = 1U << 1U;

/// 返回当前进程单调时钟毫秒，用于内部Topic时间戳和新鲜度判断。
uint64_t MonotonicMs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now().time_since_epoch())
            .count());
}

/// 异常/诊断日志节流规则：第1次与每满100次打印一次，避免周期线程刷屏。
bool ShouldLogThrottled(uint64_t count) {
    return count == 1 || count % 100 == 0;
}

/// 任务状态枚举转中文可读名称，供状态转移日志定位。
const char* MissionStateName(common::MissionState state) {
    switch (state) {
        case common::MissionState::kBoot:
            return "BOOT";
        case common::MissionState::kSelfCheck:
            return "SELF_CHECK";
        case common::MissionState::kReady:
            return "READY";
        case common::MissionState::kArming:
            return "ARMING";
        case common::MissionState::kTakeoff:
            return "TAKEOFF";
        case common::MissionState::kGpsApproach:
            return "GPS_APPROACH";
        case common::MissionState::kVisualHandover:
            return "VISUAL_HANDOVER";
        case common::MissionState::kVisualTracking:
            return "VISUAL_TRACKING";
        case common::MissionState::kObstacleHold:
            return "OBSTACLE_HOLD";
        case common::MissionState::kInterceptReady:
            return "INTERCEPT_READY";
        case common::MissionState::kInterceptExecuting:
            return "INTERCEPT_EXECUTING";
        case common::MissionState::kPostIntercept:
            return "POST_INTERCEPT";
        case common::MissionState::kReturnHome:
            return "RETURN_HOME";
        case common::MissionState::kLanding:
            return "LANDING";
        case common::MissionState::kDisarmed:
            return "DISARMED";
        case common::MissionState::kManualOverride:
            return "MANUAL_OVERRIDE";
        case common::MissionState::kFailsafe:
            return "FAILSAFE";
        case common::MissionState::kCount:
            return "COUNT";
    }
    return "UNKNOWN";
}

/// 把任务状态映射为简洁阶段号，便于地面站还没有完整枚举表时显示。
uint8_t TaskPhase(common::MissionState state) {
    return static_cast<uint8_t>(state);
}

}  // namespace

/// MissionStateMachine的实际状态与线程实现，隐藏在PIMPL中保持头文件稳定。
class MissionStateMachine::Impl final {
public:
    Impl() { SPDLOG_INFO("任务状态机创建: shadow=true control_output=false"); }

    ~Impl() {
        Stop();
        SPDLOG_INFO("任务状态机销毁");
    }

    /// 启动状态机工作线程。输入Topic必须已绑定，避免运行期出现空订阅。
    bool Start() {
        if (running_.load(std::memory_order_acquire)) {
            return true;
        }
        if (ground_target_topic_ == nullptr || flight_topic_ == nullptr ||
            health_topic_ == nullptr) {
            RecordError("任务状态机启动失败: 输入Topic未绑定");
            return false;
        }

        try {
            ground_target_subscription_ = ground_target_topic_->Subscribe(4);
            flight_subscription_ = flight_topic_->Subscribe(2);
            health_subscription_ = health_topic_->Subscribe(2);
        } catch (const std::exception& error) {
            RecordError(std::string("任务状态机订阅输入失败: ") + error.what());
            return false;
        }

        const uint64_t now_ms = MonotonicMs();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_ = common::MissionState::kBoot;
            state_entered_ms_ = now_ms;
            transition_count_ = 0;
            active_warning_bits_ = 0;
        }
        PublishStatus(now_ms);

        running_.store(true, std::memory_order_release);
        worker_ = std::thread([this] { WorkerLoop(); });
        SPDLOG_INFO("任务状态机启动: shadow=true");
        return true;
    }

    /// 停止工作线程并释放订阅。幂等，不关闭输出Topic，便于主程序按统一顺序停机。
    void Stop() {
        if (!running_.exchange(false, std::memory_order_acq_rel)) {
            return;
        }
        ground_target_subscription_.Reset();
        flight_subscription_.Reset();
        health_subscription_.Reset();
        if (worker_.joinable()) {
            worker_.join();
        }
        SPDLOG_INFO("任务状态机停止");
    }

    bool IsRunning() const { return running_.load(std::memory_order_acquire); }

    /// 绑定输入Topic。必须在Start前调用；运行中重新绑定会破坏订阅一致性，因此拒绝。
    void SetInputs(common::Topic<common::GroundStationTarget>& ground_target,
                   common::Topic<common::FlightStateSnapshot>& flight,
                   common::Topic<common::HealthStatus>& health) {
        if (running_.load(std::memory_order_acquire)) {
            RecordError("任务状态机运行中不允许重新绑定输入Topic");
            return;
        }
        ground_target_topic_ = &ground_target;
        flight_topic_ = &flight;
        health_topic_ = &health;
    }

    common::Topic<common::ControlIntent>& IntentOutput() { return intent_output_; }
    common::Topic<common::MissionStatus>& StatusOutput() { return status_output_; }

    common::MissionState CurrentState() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_;
    }

    uint64_t TransitionCount() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return transition_count_;
    }

    uint64_t ErrorCount() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return error_count_;
    }

private:
    /// 状态机主循环：先进入SELF_CHECK，再按周期抽取最新输入并执行影子状态推进。
    void WorkerLoop() {
        TransitionTo(common::MissionState::kSelfCheck, "初始化完成");
        while (running_.load(std::memory_order_acquire)) {
            DrainInputs();
            Evaluate(MonotonicMs());
            std::this_thread::sleep_for(kDecisionPeriod);
        }
    }

    /// 抽干各输入队列，只保留最新快照。状态判断必须基于同一周期的不可变快照。
    void DrainInputs() {
        while (auto message = ground_target_subscription_.TryTake()) {
            latest_ground_target_ = **message;
            const uint64_t count = ++target_receive_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_INFO("状态机收到地面站目标: boot={} seq={} target_id={} valid_for_ms={} age_ms={} count={}",
                            latest_ground_target_->ground_station_boot_id,
                            latest_ground_target_->update_seq,
                            latest_ground_target_->target_id,
                            latest_ground_target_->header.valid_for_ms,
                            latest_ground_target_->transport_age_ms, count);
            }
        }
        while (auto message = flight_subscription_.TryTake()) {
            latest_flight_ = **message;
        }
        while (auto message = health_subscription_.TryTake()) {
            latest_health_ = **message;
        }
    }

    /// 根据当前状态、飞行快照和目标新鲜度推进影子状态，并周期发布MissionStatus。
    void Evaluate(uint64_t now_ms) {
        uint32_t warnings = 0;
        if (!HasNavigationForGps()) {
            warnings |= kWarningNavigationUnavailable;
        }

        const bool target_fresh = HasFreshGroundTarget(now_ms);
        const auto state = CurrentState();
        if (state == common::MissionState::kSelfCheck && HasNavigationForGps()) {
            TransitionTo(common::MissionState::kReady, "PX4导航快照满足GPS引导前置条件");
        } else if (state == common::MissionState::kReady && target_fresh &&
                   HasNavigationForGps()) {
            // 安全影子阶段只验证GPS目标进入状态链路；不执行ARMING/TAKEOFF，不输出控制。
            TransitionTo(common::MissionState::kGpsApproach,
                         "收到有效地面站目标，进入GPS_APPROACH影子验证");
        } else if (state == common::MissionState::kGpsApproach && !target_fresh) {
            warnings |= kWarningGroundTargetStale;
            const uint64_t count = ++target_stale_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("GPS_APPROACH影子状态目标过期: count={}", count);
            }
        }

        SetWarnings(warnings);
        if (now_ms - last_status_publish_ms_ >= kStatusPublishIntervalMs) {
            PublishStatus(now_ms);
        }
    }

    /// GPS位置引导的最低导航前置条件。后续接入控制前还需增加模式/高度/电池等门禁。
    bool HasNavigationForGps() const {
        if (!latest_flight_) {
            return false;
        }
        const auto& flight = *latest_flight_;
        return flight.connected && flight.gps_state_valid && flight.gps_fix &&
               flight.global_position_valid && flight.home_valid;
    }

    /// 判断地面站目标是否仍在飞机端计算出的剩余有效期内。
    bool HasFreshGroundTarget(uint64_t now_ms) const {
        if (!latest_ground_target_) {
            return false;
        }
        const auto& target = *latest_ground_target_;
        if (target.header.receive_time_ms == 0 || target.header.valid_for_ms == 0) {
            return false;
        }
        return now_ms <= target.header.receive_time_ms + target.header.valid_for_ms;
    }

    /// 执行状态转移并立即发布状态。所有转移统一经过此函数，便于计数和日志审计。
    void TransitionTo(common::MissionState next_state, const char* reason) {
        const uint64_t now_ms = MonotonicMs();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (state_ == next_state) {
                return;
            }
            SPDLOG_INFO("任务状态转移: {} -> {} reason={}", MissionStateName(state_),
                        MissionStateName(next_state), reason);
            state_ = next_state;
            state_entered_ms_ = now_ms;
            ++transition_count_;
        }
        PublishStatus(now_ms);
    }

    /// 更新当前告警位。状态发布线程读取同一份告警快照。
    void SetWarnings(uint32_t warnings) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        active_warning_bits_ = warnings;
    }

    /// 发布任务状态回传。影子阶段control_source固定为0，表示未取得真实控制权。
    void PublishStatus(uint64_t now_ms) {
        common::MissionStatus status;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status.header.sequence = ++status_sequence_;
            status.state = state_;
            status.state_entered_ms = state_entered_ms_;
            status.task_phase = TaskPhase(state_);
            status.active_warning_bits = active_warning_bits_;
        }
        status.header.source_time_ms = now_ms;
        status.header.receive_time_ms = now_ms;
        status.header.valid_for_ms = kStatusValidForMs;
        status.header.source_id = 1;
        status.header.health = 1;
        status.control_source = 0;
        status.interception_authorized = false;
        (void)status_output_.Publish(std::make_shared<const common::MissionStatus>(status));
        last_status_publish_ms_ = now_ms;
    }

    /// 记录状态机错误并按节流规则打印，避免启动失败或运行期错误刷屏。
    void RecordError(const std::string& message) {
        uint64_t count = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            count = ++error_count_;
        }
        if (ShouldLogThrottled(count)) {
            SPDLOG_ERROR("{} count={}", message, count);
        }
    }

    common::Topic<common::GroundStationTarget>* ground_target_topic_ = nullptr;
    common::Topic<common::FlightStateSnapshot>* flight_topic_ = nullptr;
    common::Topic<common::HealthStatus>* health_topic_ = nullptr;

    common::Topic<common::GroundStationTarget>::Subscription ground_target_subscription_;
    common::Topic<common::FlightStateSnapshot>::Subscription flight_subscription_;
    common::Topic<common::HealthStatus>::Subscription health_subscription_;

    std::optional<common::GroundStationTarget> latest_ground_target_;
    std::optional<common::FlightStateSnapshot> latest_flight_;
    std::optional<common::HealthStatus> latest_health_;

    std::atomic<bool> running_{false};
    std::thread worker_;
    mutable std::mutex state_mutex_;
    common::MissionState state_ = common::MissionState::kBoot;
    uint64_t state_entered_ms_ = 0;
    uint64_t transition_count_ = 0;
    uint64_t error_count_ = 0;
    uint64_t status_sequence_ = 0;
    uint64_t last_status_publish_ms_ = 0;
    uint32_t active_warning_bits_ = 0;
    uint64_t target_receive_count_ = 0;
    uint64_t target_stale_count_ = 0;

    common::Topic<common::ControlIntent> intent_output_;
    common::Topic<common::MissionStatus> status_output_;
};

MissionStateMachine::MissionStateMachine() : impl_(std::make_unique<Impl>()) {}
MissionStateMachine::~MissionStateMachine() = default;
bool MissionStateMachine::Start() { return impl_->Start(); }
void MissionStateMachine::Stop() { impl_->Stop(); }
bool MissionStateMachine::IsRunning() const { return impl_->IsRunning(); }
void MissionStateMachine::SetInputs(common::Topic<common::GroundStationTarget>& ground_target,
                                    common::Topic<common::FlightStateSnapshot>& flight,
                                    common::Topic<common::HealthStatus>& health) {
    impl_->SetInputs(ground_target, flight, health);
}
common::Topic<common::ControlIntent>& MissionStateMachine::IntentOutput() {
    return impl_->IntentOutput();
}
common::Topic<common::MissionStatus>& MissionStateMachine::StatusOutput() {
    return impl_->StatusOutput();
}
common::MissionState MissionStateMachine::CurrentState() const {
    return impl_->CurrentState();
}
uint64_t MissionStateMachine::TransitionCount() const {
    return impl_->TransitionCount();
}
uint64_t MissionStateMachine::ErrorCount() const { return impl_->ErrorCount(); }

MissionStateMachineStub::MissionStateMachineStub() {
    SPDLOG_INFO("任务状态机部件骨架创建");
}

MissionStateMachineStub::~MissionStateMachineStub() {
    SPDLOG_INFO("任务状态机部件骨架销毁");
}

bool MissionStateMachineStub::Start() {
    running_ = true;
    SPDLOG_INFO("任务状态机部件骨架启动");
    return true;
}

void MissionStateMachineStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("任务状态机部件骨架停止");
}

bool MissionStateMachineStub::IsRunning() const {
    return running_;
}

void MissionStateMachineStub::SetInputs(
    common::Topic<common::GroundStationTarget>& /*ground_target*/,
    common::Topic<common::FlightStateSnapshot>& /*flight*/,
    common::Topic<common::HealthStatus>& /*health*/) {
    // 骨架实现仅保留接口兼容性；正式逻辑见 MissionStateMachine。
}

common::Topic<common::ControlIntent>& MissionStateMachineStub::IntentOutput() {
    return intent_output_;
}

common::Topic<common::MissionStatus>& MissionStateMachineStub::StatusOutput() {
    return status_output_;
}

common::MissionState MissionStateMachineStub::CurrentState() const {
    return state_;
}

uint64_t MissionStateMachineStub::TransitionCount() const {
    return transition_count_;
}

uint64_t MissionStateMachineStub::ErrorCount() const {
    return error_count_;
}

}  // namespace drone::state_machine
