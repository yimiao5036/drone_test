/**
 * @file mission_state_machine.cpp
 * @brief MissionStateMachine 安全影子实现（阶段四起支持 S5 位置引导意图）
 *
 * 默认只接入地面站目标与飞行状态，发布 MissionStatus 供链路验证；
 * 经 SetEstimationInputs 绑定估计/视觉/测距输入后，GPS_APPROACH 状态发布
 * kPositionGuide 控制意图（带 bit31 控制权请求位），是否真正输出由
 * FlightController 授权链与 enable_control 装配门禁决定。
 */
#include "state_machine/mission_state_machine.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

#include "communication/px4_link.h"

namespace drone::state_machine {
namespace {

// 当前实现默认是安全影子状态机：只消费目标/飞行/健康快照并发布 MissionStatus，
// 不生成真实 ControlIntent，也不连接 PX4 控制输出。所有决策运行在独立线程中。
// 绑定估计/视觉/测距输入后，S5 开始发布位置引导意图（见 PublishGpsApproachIntent）。

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
/// 意图重发上限（10Hz）：意图是周期语义，FlightController 50ms 周期消费；
/// 内容无变化时按该间隔重发，内容变化立即发。
constexpr uint64_t kIntentRepublishIntervalMs = 100;
/// 位置引导意图低位原因码：地面站任务下发（docs/数据接口文档.md §3.5 编码 7），
/// 与 bit31 控制权请求位正交叠加。
constexpr uint32_t kReasonGroundStationTask = 7;

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

/// 浮点内容比较：NaN 与 NaN 视为相等（高度无效时 pos_z_m 恒为 NaN，
/// 不得因此被当成"内容变化"每拍立即重发）。
bool FloatContentEqual(float a, float b) {
    return a == b || (std::isnan(a) && std::isnan(b));
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
    explicit Impl(const MissionStateMachineConfig& config)
        : intent_valid_for_ms_(
              static_cast<uint64_t>(config.intent_valid_for.count())) {
        SPDLOG_INFO("任务状态机创建: shadow=true control_output=false intent_valid_for_ms={}",
                    intent_valid_for_ms_);
    }

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

        // 估计/视觉/测距三个输入是一组：部分绑定视为装配错误，按未绑定处理
        // （保持影子行为），避免 S5 意图只依据部分观测链。
        const bool any_estimation_bound = estimated_topic_ != nullptr ||
                                          visual_status_topic_ != nullptr ||
                                          distance_topic_ != nullptr;
        const bool all_estimation_bound = estimated_topic_ != nullptr &&
                                          visual_status_topic_ != nullptr &&
                                          distance_topic_ != nullptr;
        if (any_estimation_bound && !all_estimation_bound) {
            RecordError("估计/视觉/测距输入必须三者同时绑定；本次按未绑定处理（保持影子）");
        }

        try {
            ground_target_subscription_ = ground_target_topic_->Subscribe(4);
            flight_subscription_ = flight_topic_->Subscribe(2);
            health_subscription_ = health_topic_->Subscribe(2);
            if (all_estimation_bound) {
                estimated_subscription_ = estimated_topic_->Subscribe(2);
                visual_status_subscription_ = visual_status_topic_->Subscribe(2);
                distance_subscription_ = distance_topic_->Subscribe(2);
                estimation_inputs_active_ = true;
            }
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
        estimated_subscription_.Reset();
        visual_status_subscription_.Reset();
        distance_subscription_.Reset();
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

    /// 绑定估计/视觉/测距输入（三者一组）。必须在Start前调用；未绑定时
    /// GPS_APPROACH 不出控制意图，保持影子行为。
    void SetEstimationInputs(common::Topic<common::TargetState>& estimated,
                             common::Topic<common::VisualTargetStatus>& visual_status,
                             common::Topic<common::StereoTargetDistance>& distance) {
        if (running_.load(std::memory_order_acquire)) {
            RecordError("任务状态机运行中不允许重新绑定估计输入Topic");
            return;
        }
        estimated_topic_ = &estimated;
        visual_status_topic_ = &visual_status;
        distance_topic_ = &distance;
    }

    /// 注入 PX4 命令窄接口。仅 enable_control=true 装配期调用；当前阶段只持有，
    /// 供后续 ARM/执行机构命令使用，不参与意图发布判定。
    void SetCommandSender(communication::IPx4CommandSender* sender) {
        if (running_.load(std::memory_order_acquire)) {
            RecordError("任务状态机运行中不允许注入 PX4 命令接口");
            return;
        }
        command_sender_ = sender;
        SPDLOG_INFO("任务状态机 PX4 命令接口已注入: sender={}", static_cast<const void*>(sender));
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
    // 工作线程启动后先离开 BOOT 进入 SELF_CHECK；随后每个周期使用最新输入做一次决策。
    void WorkerLoop() {
        TransitionTo(common::MissionState::kSelfCheck, "初始化完成");
        while (running_.load(std::memory_order_acquire)) {
            DrainInputs();
            Evaluate(MonotonicMs());
            std::this_thread::sleep_for(kDecisionPeriod);
        }
    }

    /// 抽干各输入队列，只保留最新快照。状态判断必须基于同一周期的不可变快照。
    // 每个决策周期先抽干输入队列，只保留最新快照，避免旧目标或旧飞行状态积压。
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
        while (auto message = estimated_subscription_.TryTake()) {
            latest_estimated_ = **message;
        }
        while (auto message = visual_status_subscription_.TryTake()) {
            latest_visual_status_ = **message;
        }
        while (auto message = distance_subscription_.TryTake()) {
            latest_distance_ = **message;
        }
    }

    /// 根据当前状态、飞行快照和目标新鲜度推进影子状态，并周期发布MissionStatus。
    // 影子决策：SELF_CHECK 等待导航就绪，READY 等待新鲜目标，GPS_APPROACH 只做告警维护。
    // 这里刻意不生成控制意图，真实 ARM/TAKEOFF/导航流程需经过后续安全门禁。
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

        EvaluateGpsApproachIntent(now_ms);

        SetWarnings(warnings);
        if (now_ms - last_status_publish_ms_ >= kStatusPublishIntervalMs) {
            PublishStatus(now_ms);
        }
    }

    /// GPS_APPROACH 位置引导意图（阶段四）。仅在估计输入已绑定且处于 S5 时工作：
    /// 估计新鲜有效→发布/重发 kPositionGuide 意图；估计失效→停发，
    /// 超龄收口由 FlightController 意图超时门禁（controller.intent_timeout）接管。
    void EvaluateGpsApproachIntent(uint64_t now_ms) {
        if (!estimation_inputs_active_) {
            return;
        }
        if (CurrentState() != common::MissionState::kGpsApproach) {
            if (position_intent_active_) {
                position_intent_active_ = false;
                last_intent_target_.reset();
                SPDLOG_INFO("GPS_APPROACH 位置引导意图停止: 离开GPS_APPROACH状态");
            }
            return;
        }
        if (HasFreshEstimate(now_ms)) {
            PublishGpsApproachIntent(*latest_estimated_, now_ms);
        } else if (position_intent_active_) {
            // 估计失效或超龄：停止正常意图。FlightController 在意图超时后自行收口，
            // 这里不再补发安全类意图（刹停语义归后续状态迁移任务）。
            position_intent_active_ = false;
            last_intent_target_.reset();
            const uint64_t count = ++intent_stale_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("GPS_APPROACH 位置引导意图停止: 估计无效或超龄 count={}", count);
            }
        }
    }

    /// 估计目标是否新鲜有效（valid 标志 + 剩余有效期，与 TargetEstimator 输出语义一致）。
    bool HasFreshEstimate(uint64_t now_ms) const {
        if (!latest_estimated_) {
            return false;
        }
        const auto& target = *latest_estimated_;
        if (!target.valid) {
            return false;
        }
        if (target.header.receive_time_ms == 0 || target.header.valid_for_ms == 0) {
            return false;
        }
        return now_ms <= target.header.receive_time_ms + target.header.valid_for_ms;
    }

    /// 发布 S5 位置引导意图。节流纪律：内容无变化按 10Hz 上限重发
    /// （意图是周期语义，FlightController 50ms 周期消费）；内容变化立即发。
    void PublishGpsApproachIntent(const common::TargetState& target, uint64_t now_ms) {
        const bool content_changed = !last_intent_target_.has_value() ||
            !FloatContentEqual(last_intent_target_->pos_x_m, target.pos_x_m) ||
            !FloatContentEqual(last_intent_target_->pos_y_m, target.pos_y_m) ||
            !FloatContentEqual(last_intent_target_->pos_z_m, target.pos_z_m) ||
            last_intent_target_->valid != target.valid;
        if (!content_changed &&
            now_ms - last_intent_publish_ms_ < kIntentRepublishIntervalMs) {
            return;
        }
        if (!position_intent_active_) {
            SPDLOG_INFO("GPS_APPROACH 位置引导意图启动: target_id={} pos=({:.2f},{:.2f},{:.2f})",
                        target.target_id, static_cast<double>(target.pos_x_m),
                        static_cast<double>(target.pos_y_m),
                        static_cast<double>(target.pos_z_m));
        }
        common::ControlIntent intent;
        intent.header.sequence = ++intent_sequence_;
        intent.header.source_time_ms = now_ms;
        intent.header.receive_time_ms = now_ms;
        intent.header.valid_for_ms = intent_valid_for_ms_;
        intent.header.source_id = 1;
        intent.header.health = 1;
        intent.header.frame_id = target.header.frame_id;
        intent.type = common::ControlIntentType::kPositionGuide;
        intent.control_source = 0;  // 位置链
        intent.target_x = target.pos_x_m;
        intent.target_y = target.pos_y_m;
        // 高度无效时 pos_z_m 为 NaN（合法语义）：原样透传，
        // 由 FlightController 有限性门禁拒绝，状态机侧不得改成 0。
        intent.target_z = target.pos_z_m;
        // bit31 显式请求控制权（FlightController 授权链必要条件）+
        // 低位 7=地面站任务下发。
        intent.reason_code =
            common::kControlIntentReasonRequestControl | kReasonGroundStationTask;
        intent.valid_until_ms = now_ms + intent_valid_for_ms_;
        (void)intent_output_.Publish(
            std::make_shared<const common::ControlIntent>(intent));
        last_intent_publish_ms_ = now_ms;
        last_intent_target_ = target;
        position_intent_active_ = true;
    }

    /// GPS位置引导的最低导航前置条件。后续接入控制前还需增加模式/高度/电池等门禁。
    // GPS 引导最低门禁：连接、3D fix、全局位置和 Home 必须同时有效。
    bool HasNavigationForGps() const {
        if (!latest_flight_) {
            return false;
        }
        const auto& flight = *latest_flight_;
        return flight.connected && flight.gps_state_valid && flight.gps_fix &&
               flight.global_position_valid && flight.home_valid;
    }

    /// 判断地面站目标是否仍在飞机端计算出的剩余有效期内。
    // 目标有效期使用 GroundStationLink 校正后的 receive_time + remaining valid_for 判断。
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
    // 发布不可变任务状态快照；影子阶段固定 control_source=0 且禁止拦截授权。
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
    common::Topic<common::TargetState>* estimated_topic_ = nullptr;
    common::Topic<common::VisualTargetStatus>* visual_status_topic_ = nullptr;
    common::Topic<common::StereoTargetDistance>* distance_topic_ = nullptr;
    /// PX4 命令窄接口（仅 enable_control=true 装配期注入；当前仅持有备用）。
    communication::IPx4CommandSender* command_sender_ = nullptr;

    common::Topic<common::GroundStationTarget>::Subscription ground_target_subscription_;
    common::Topic<common::FlightStateSnapshot>::Subscription flight_subscription_;
    common::Topic<common::HealthStatus>::Subscription health_subscription_;
    common::Topic<common::TargetState>::Subscription estimated_subscription_;
    common::Topic<common::VisualTargetStatus>::Subscription visual_status_subscription_;
    common::Topic<common::StereoTargetDistance>::Subscription distance_subscription_;

    std::optional<common::GroundStationTarget> latest_ground_target_;
    std::optional<common::FlightStateSnapshot> latest_flight_;
    std::optional<common::HealthStatus> latest_health_;
    std::optional<common::TargetState> latest_estimated_;
    std::optional<common::VisualTargetStatus> latest_visual_status_;
    std::optional<common::StereoTargetDistance> latest_distance_;

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

    /// 控制意图有效期（构造注入，与 controller.intent_timeout 对齐）。
    const uint64_t intent_valid_for_ms_;
    bool estimation_inputs_active_ = false;  ///< 估计/视觉/测距输入组已成功订阅
    bool position_intent_active_ = false;    ///< S5 位置引导意图发布中
    uint64_t intent_sequence_ = 0;
    uint64_t last_intent_publish_ms_ = 0;
    uint64_t intent_stale_count_ = 0;
    std::optional<common::TargetState> last_intent_target_;  ///< 上次发布意图对应的目标内容

    common::Topic<common::ControlIntent> intent_output_;
    common::Topic<common::MissionStatus> status_output_;
};

MissionStateMachine::MissionStateMachine(MissionStateMachineConfig config)
    : impl_(std::make_unique<Impl>(config)) {}
MissionStateMachine::~MissionStateMachine() = default;
bool MissionStateMachine::Start() { return impl_->Start(); }
void MissionStateMachine::Stop() { impl_->Stop(); }
bool MissionStateMachine::IsRunning() const { return impl_->IsRunning(); }
void MissionStateMachine::SetInputs(common::Topic<common::GroundStationTarget>& ground_target,
                                    common::Topic<common::FlightStateSnapshot>& flight,
                                    common::Topic<common::HealthStatus>& health) {
    impl_->SetInputs(ground_target, flight, health);
}
void MissionStateMachine::SetEstimationInputs(
    common::Topic<common::TargetState>& estimated,
    common::Topic<common::VisualTargetStatus>& visual_status,
    common::Topic<common::StereoTargetDistance>& distance) {
    impl_->SetEstimationInputs(estimated, visual_status, distance);
}
void MissionStateMachine::SetCommandSender(communication::IPx4CommandSender* sender) {
    impl_->SetCommandSender(sender);
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

void MissionStateMachineStub::SetEstimationInputs(
    common::Topic<common::TargetState>& /*estimated*/,
    common::Topic<common::VisualTargetStatus>& /*visual_status*/,
    common::Topic<common::StereoTargetDistance>& /*distance*/) {
    // 骨架实现仅保留接口兼容性；正式逻辑见 MissionStateMachine。
}

void MissionStateMachineStub::SetCommandSender(
    communication::IPx4CommandSender* /*sender*/) {
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
