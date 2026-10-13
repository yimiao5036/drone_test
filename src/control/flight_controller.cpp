/**
 * @file flight_controller.cpp
 * @brief IFlightController 骨架占位实现 + FlightController 正式实现（骨架阶段）
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（控制意图 → PX4
 * 设定值转换、控制算法）在实现期接入。
 *
 * FlightController（阶段四控制接入 Task 3）：固定周期线程消费
 * ControlIntent，门禁链（有限性/超龄/控制权）通过后发布 Px4Setpoint。
 * 本任务控制权恒为 kIdle（不发布任何 setpoint）；限幅（Task 4）、
 * 控制权三态与 PX4 门禁（Task 5）后续接入。
 */
#include "control/flight_controller.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <optional>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

#include "communication/px4_link.h"
#include "config/config.h"
#include "control/staleness_checker.h"

namespace drone::control {
namespace {

using Clock = std::chrono::steady_clock;

/// 返回当前进程单调时钟毫秒，用于超龄判定与输出时间戳。
uint64_t MonotonicMs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now().time_since_epoch())
            .count());
}

/// 拒绝/异常日志节流规则：第1次与每满100次打印一次，避免周期线程刷屏。
bool ShouldLogThrottled(uint64_t count) {
    return count == 1 || count % 100 == 0;
}

/// 意图字段有限性检查：数值通道全部 std::isfinite，NaN/Inf 直接拒绝。
bool IsFiniteIntent(const common::ControlIntent& intent) {
    return std::isfinite(intent.target_x) && std::isfinite(intent.target_y) &&
           std::isfinite(intent.target_z) && std::isfinite(intent.yaw_deg) &&
           std::isfinite(intent.yaw_rate_dps) &&
           std::isfinite(intent.max_speed_mps);
}

}  // namespace

// 数据流：ControlIntent Topic → 控制算法 → Px4Setpoint Topic。
// Stub 仅暴露生命周期与主题接口，用于装配验证；真实控制算法在实现期接入。

FlightControllerStub::FlightControllerStub() {
    SPDLOG_INFO("飞行控制器部件骨架创建");
}

FlightControllerStub::~FlightControllerStub() {
    SPDLOG_INFO("飞行控制器部件骨架销毁");
}

// 启动：骨架无真实后端，仅置位运行标志（幂等语义由调用方 Start 前置判断保证）。
bool FlightControllerStub::Start() {
    running_ = true;
    SPDLOG_INFO("飞行控制器部件骨架启动");
    return true;
}

// 停止：幂等，仅在已启动时置位，未启动直接返回。
void FlightControllerStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("飞行控制器部件骨架停止");
}

bool FlightControllerStub::IsRunning() const {
    return running_;
}

// 骨架期忽略输入绑定；实现期保存订阅并启动控制周期。
void FlightControllerStub::SetInput(common::Topic<common::ControlIntent>& /*intent*/) {
}

common::Topic<common::Px4Setpoint>& FlightControllerStub::SetpointOutput() {
    return setpoint_output_;
}

uint64_t FlightControllerStub::SetpointCount() const {
    return setpoint_count_;
}

uint64_t FlightControllerStub::ErrorCount() const {
    return error_count_;
}

// =============================================================================
// FlightController —— 正式实现（骨架阶段）
//
// 线程模型：单周期线程按 config.period 执行 Tick()；意图经 Topic 有界订阅
// 队列传入，Tick 开头排干只留最新。诊断计数用原子量，跨线程只读安全。
// 日志纪律：控制周期热路径不打日志；拒绝事件节流（第1次与每满100次）。
// =============================================================================

class FlightController::Impl {
public:
    explicit Impl(const config::FlightControllerConfig& config) : config_(config) {
        // 配置在 LoadAppConfig 加载期已 Validate() 校验（fail-fast）；
        // 此处不再重复校验，避免 drone_control_core → drone_config 的链接依赖反转。
    }

    ~Impl() { Stop(); }

    bool Start() {
        bool expected = false;
        if (!running_.compare_exchange_strong(expected, true)) {
            return true;  // 幂等：已启动直接成功
        }
        worker_ = std::thread([this] { Run(); });
        SPDLOG_INFO("飞行控制器启动: period={}ms intent_timeout={}ms coast_timeout={}ms",
                    config_.period.count(), config_.intent_timeout.count(),
                    config_.coast_timeout.count());
        return true;
    }

    void Stop() {
        if (!running_.exchange(false)) {
            return;  // 幂等：未启动直接返回
        }
        if (worker_.joinable()) {
            worker_.join();
        }
        SPDLOG_INFO("飞行控制器停止: setpoint_count={} rejected_intent_count={}",
                    setpoint_count_.load(), rejected_intent_count_.load());
    }

    bool IsRunning() const { return running_.load(); }

    void SetInput(common::Topic<common::ControlIntent>& intent) {
        // 队列有界、优先最新：周期线程排干时只保留最后一条意图。
        intent_subscription_ = intent.Subscribe(kIntentQueueCapacity);
    }

    common::Topic<common::Px4Setpoint>& SetpointOutput() { return setpoint_output_; }

    uint64_t SetpointCount() const { return setpoint_count_.load(); }
    uint64_t ErrorCount() const { return error_count_.load(); }
    uint64_t RejectedIntentCount() const { return rejected_intent_count_.load(); }

    ControlAuthority Authority() const {
        return authority_.load(std::memory_order_acquire);
    }

    void SetPx4LinkSource(const communication::IPx4Link* link) {
        // Task 5 接入 PX4 快照门禁；本任务仅保存只读源指针。
        px4_link_ = link;
    }

private:
    /// 意图订阅队列容量：周期消费，4 条足够吸收突发，溢出丢旧保新。
    static constexpr std::size_t kIntentQueueCapacity = 4;

    /// 固定周期线程：每 period 执行一次 Tick()。
    void Run() {
        auto next = Clock::now();
        while (running_.load(std::memory_order_acquire)) {
            next += config_.period;
            Tick(MonotonicMs());
            std::this_thread::sleep_until(next);
        }
    }

    void Tick(uint64_t now_ms) {
        // 1) 排干意图队列，只留最新（入口门禁在排干时执行，拒绝即计数）
        DrainIntents(now_ms);
        // 2) 门禁链：IDLE 直接返回（本任务默认 IDLE，Task 5 才接入授权）
        if (authority_.load(std::memory_order_acquire) == ControlAuthority::kIdle) {
            return;
        }
        if (!latest_intent_.has_value()) {
            return;
        }
        // ① 持有期再超龄（意图在消费间隔内过期）→ 拒绝并刹停降级
        if (now_ms >= latest_intent_->valid_until_ms ||
            StalenessChecker::IsStale(
                static_cast<int64_t>(latest_intent_->header.receive_time_ms),
                static_cast<int64_t>(now_ms), config_.intent_timeout.count())) {
            RejectIntent("意图持有期超龄");
            PublishBrake(now_ms);  // EnterCoastOrIdle 由 Task 5 完整实现
            return;
        }
        PublishFromIntent(*latest_intent_, now_ms);  // 限幅由 Task 4 接入；本任务直通
    }

    /// 排干意图队列，只保留通过入口门禁的最新一条。
    void DrainIntents(uint64_t now_ms) {
        while (auto message = intent_subscription_.TryTake()) {
            common::ControlIntent intent = **message;
            // 控制器是意图的接收边界：上游未打戳时补 receive_time_ms，
            // 保证超龄判定有可信取样时间。
            if (intent.header.receive_time_ms == 0) {
                intent.header.receive_time_ms = now_ms;
            }
            // ③ 有限性（NaN/Inf 意图字段直接拒绝）
            if (!IsFiniteIntent(intent)) {
                RejectIntent("意图字段非有限(NaN/Inf)");
                continue;
            }
            // ① 超龄（含到达即超龄）
            if (now_ms >= intent.valid_until_ms ||
                StalenessChecker::IsStale(
                    static_cast<int64_t>(intent.header.receive_time_ms),
                    static_cast<int64_t>(now_ms), config_.intent_timeout.count())) {
                RejectIntent("意图超龄");
                continue;
            }
            latest_intent_ = std::move(intent);
        }
    }

    /// 拒绝计数 + 节流日志（第1次与每满100次）。
    void RejectIntent(const char* reason) {
        const uint64_t count = ++rejected_intent_count_;
        if (ShouldLogThrottled(count)) {
            SPDLOG_WARN("飞行控制器拒绝意图({}): 累计={}", reason, count);
        }
    }

    /// 发布零速刹停设定值：速度三分量为 0、valid=true。
    void PublishBrake(uint64_t now_ms) {
        common::Px4Setpoint setpoint;
        setpoint.type = common::SetpointType::kVelocity;
        setpoint.x = 0.f;
        setpoint.y = 0.f;
        setpoint.z = 0.f;
        setpoint.valid = true;
        Publish(std::move(setpoint), now_ms);
    }

    /// 意图直通为设定值（Task 4 在此之前接入限幅与无扰衔接）。
    void PublishFromIntent(const common::ControlIntent& intent, uint64_t now_ms) {
        common::Px4Setpoint setpoint;
        switch (intent.type) {
            case common::ControlIntentType::kVelocityHeading:
                setpoint.type = common::SetpointType::kVelocity;
                setpoint.x = intent.target_x;
                setpoint.y = intent.target_y;
                setpoint.z = intent.target_z;
                setpoint.yaw_deg = intent.yaw_deg;
                setpoint.use_yaw = true;
                break;
            case common::ControlIntentType::kBrakeHover:
                PublishBrake(now_ms);
                return;
            case common::ControlIntentType::kPositionGuide:
                setpoint.type = common::SetpointType::kPosition;
                setpoint.x = intent.target_x;
                setpoint.y = intent.target_y;
                setpoint.z = intent.target_z;
                setpoint.yaw_deg = intent.yaw_deg;
                setpoint.use_yaw = true;
                break;
            default: {
                // 起飞/降落/返航/人工接管等类型骨架期不支持，拒绝计数。
                const uint64_t count = ++error_count_;
                if (ShouldLogThrottled(count)) {
                    SPDLOG_WARN("飞行控制器暂不支持的意图类型: type={} 累计={}",
                                static_cast<int>(intent.type), count);
                }
                return;
            }
        }
        setpoint.valid = true;
        Publish(std::move(setpoint), now_ms);
    }

    void Publish(common::Px4Setpoint setpoint, uint64_t now_ms) {
        setpoint.header.sequence = ++setpoint_sequence_;
        setpoint.header.receive_time_ms = now_ms;
        setpoint.header.valid_for_ms =
            static_cast<uint64_t>(config_.period.count()) * 2;
        (void)setpoint_output_.Publish(
            std::make_shared<const common::Px4Setpoint>(std::move(setpoint)));
        ++setpoint_count_;
    }

    const config::FlightControllerConfig config_;
    std::atomic<bool> running_{false};
    std::thread worker_;
    common::Topic<common::ControlIntent>::Subscription intent_subscription_;
    common::Topic<common::Px4Setpoint> setpoint_output_;
    const communication::IPx4Link* px4_link_ = nullptr;  // 只读门禁源（Task 5 启用）

    // 以下仅周期线程访问（诊断接口经原子量暴露）。
    std::optional<common::ControlIntent> latest_intent_;
    uint64_t setpoint_sequence_ = 0;
    std::atomic<ControlAuthority> authority_{ControlAuthority::kIdle};
    std::atomic<uint64_t> setpoint_count_{0};
    std::atomic<uint64_t> rejected_intent_count_{0};
    std::atomic<uint64_t> error_count_{0};
};

FlightController::FlightController(const config::FlightControllerConfig& config)
    : impl_(std::make_unique<Impl>(config)) {
    SPDLOG_INFO("飞行控制器创建（骨架：默认 IDLE，不发布设定值）");
}

FlightController::~FlightController() = default;

bool FlightController::Start() { return impl_->Start(); }

void FlightController::Stop() { impl_->Stop(); }

bool FlightController::IsRunning() const { return impl_->IsRunning(); }

void FlightController::SetInput(common::Topic<common::ControlIntent>& intent) {
    impl_->SetInput(intent);
}

common::Topic<common::Px4Setpoint>& FlightController::SetpointOutput() {
    return impl_->SetpointOutput();
}

uint64_t FlightController::SetpointCount() const { return impl_->SetpointCount(); }

uint64_t FlightController::ErrorCount() const { return impl_->ErrorCount(); }

ControlAuthority FlightController::Authority() const { return impl_->Authority(); }

uint64_t FlightController::RejectedIntentCount() const {
    return impl_->RejectedIntentCount();
}

void FlightController::SetPx4LinkSource(const communication::IPx4Link* link) {
    impl_->SetPx4LinkSource(link);
}

}  // namespace drone::control
