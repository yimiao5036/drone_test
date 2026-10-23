#include "state_machine/gcs_net_fire.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

#include "common/logger.h"
#include "communication/ground_station_link.h"
#include "communication/px4_link.h"

namespace drone::state_machine {
namespace {

// 与 GroundStationLink 的协议常量保持一致；不一致会破坏 65016/65017 语义。
static_assert(kGcsNetFireProtocolVersion == communication::kNetFireProtocolVersion,
              "GcsNetFire 协议版本常量与 communication 层不一致");
static_assert(kGcsNetFireFlagConfirmed == communication::kNetFireFlagConfirmed,
              "GcsNetFire 确认位常量与 communication 层不一致");

/// 工作节拍：与状态机决策拍同量级，足以覆盖执行器 50ms 级时序。
constexpr std::chrono::milliseconds kLoopPeriod{20};
/// 飞行状态新鲜度上限：超时则无法确认落地/未解锁，拒绝发射。
constexpr uint64_t kFlightStateTimeoutMs = 1000;
/// 指令在队列中滞留上限（指令未携带 valid_for_ms 时的兜底）。
constexpr uint64_t kDefaultCommandValidMs = 2000;

uint64_t MonotonicMs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

}  // namespace

GcsNetFire::GcsNetFire(config::ActuatorConfig config,
                       communication::IPx4CommandSender* sender)
    : config_(std::move(config)), sender_(sender) {
    if (!config_.gcs_fire_enabled) {
        // 装配层只在 gcs_fire_enabled=true 时构造本服务；走到这里说明配置与装配
        // 不一致，保持"无执行器"状态，所有请求按 kActuatorUnavailable 拒绝。
        SPDLOG_WARN("GcsNetFire: gcs_fire_enabled=false，捕网执行器不创建");
        return;
    }
    actuator_ = CreateNetActuator(config_, sender_);
}

GcsNetFire::~GcsNetFire() { Stop(); }

void GcsNetFire::SetFlightStateInput(
    common::Topic<common::FlightStateSnapshot>& state) {
    flight_state_topic_ = &state;
}

void GcsNetFire::SetCommandInput(common::Topic<common::NetFireCommand>& command) {
    command_topic_ = &command;
}

bool GcsNetFire::Start() {
    if (running_.load(std::memory_order_acquire)) {
        return true;
    }
    if (flight_state_topic_ == nullptr || command_topic_ == nullptr) {
        SPDLOG_ERROR("GcsNetFire: 未绑定飞行状态或指令输入，拒绝启动");
        return false;
    }
    if (actuator_ == nullptr) {
        SPDLOG_ERROR("GcsNetFire: 执行器未创建（未注入 PX4 命令接口），拒绝启动");
        return false;
    }

    // 指令队列保留最近 8 条即可：发射是人工低频操作，积压说明链路异常，
    // 宁可丢最早帧也不阻塞状态机线程。
    command_subscription_ = command_topic_->Subscribe(
        8, common::Topic<common::NetFireCommand>::OverflowPolicy::kDropOldest);
    flight_state_subscription_ = flight_state_topic_->Subscribe(
        2,
        common::Topic<common::FlightStateSnapshot>::OverflowPolicy::kDropOldest);

    running_.store(true, std::memory_order_release);
    worker_ = std::thread([this]() { Loop(); });
    SPDLOG_INFO("GcsNetFire: 已启动（type={} channel={} cooldown={}ms）",
                config_.type, config_.net_channel,
                static_cast<unsigned long long>(config_.gcs_fire_cooldown.count()));
    return true;
}

void GcsNetFire::Stop() {
    if (!running_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    command_subscription_ = CommandSubscription{};
    flight_state_subscription_ = FlightSubscription{};
    latest_state_.reset();
    SPDLOG_INFO("GcsNetFire: 已停止（accepted={} rejected={}）",
                static_cast<unsigned long long>(accepted_.load()),
                static_cast<unsigned long long>(rejected_.load()));
}

void GcsNetFire::Loop() {
    while (running_.load(std::memory_order_acquire)) {
        // 先刷新飞行状态：判定"落地且未解锁"必须用最新快照。
        if (flight_state_subscription_.IsOpen()) {
            while (auto state = flight_state_subscription_.TryTake()) {
                latest_state_ = **state;
            }
        }

        const uint64_t now_ms = MonotonicMs();
        if (command_subscription_.IsOpen()) {
            while (auto command = command_subscription_.TryTake()) {
                HandleCommand(**command, now_ms);
            }
        }

        UpdateActuator(now_ms);

        std::this_thread::sleep_for(kLoopPeriod);
    }
}

void GcsNetFire::UpdateActuator(uint64_t now_ms) {
    if (actuator_ == nullptr) {
        return;
    }

    ActuatorFeedback feedback;
    feedback.now_ms = now_ms;
    feedback.px4_connected = latest_state_.has_value() && latest_state_->connected;
    if (sender_ != nullptr) {
        feedback.ack = sender_->AckStatus();
    }
    actuator_->Update(feedback);

    if (actuator_->Faulted() && !fault_logged_) {
        fault_logged_ = true;
        SPDLOG_ERROR("GcsNetFire: 捕网执行器进入终态故障（{}），需人工维护",
                     actuator_->FaultReason());
        if (pending_completion_) {
            // 已受理的触发在执行器时序中失败：补发拒绝回执，地面站可见失败原因。
            common::NetFireCommand command;
            command.command_id = pending_command_id_;
            PublishAck(command, common::NetFireResult::kRejected,
                       common::NetFireReason::kActuatorFaulted);
            pending_completion_ = false;
        }
        return;
    }

    // 触发-复位周期走完：执行器已可接受下一发，补发 kCompleted。
    if (pending_completion_ && actuator_->Fired() && actuator_->Ready()) {
        common::NetFireCommand command;
        command.command_id = pending_command_id_;
        PublishAck(command, common::NetFireResult::kCompleted,
                   common::NetFireReason::kNone);
        pending_completion_ = false;
        SPDLOG_INFO("GcsNetFire: 发射周期完成 command_id={}",
                    static_cast<unsigned long long>(pending_command_id_));
    }
}

void GcsNetFire::HandleCommand(const common::NetFireCommand& command,
                               uint64_t now_ms) {
    const auto reject = [this, &command](common::NetFireReason reason) {
        rejected_.fetch_add(1, std::memory_order_acq_rel);
        last_result_ = common::NetFireResult::kRejected;
        last_reason_ = reason;
        has_last_command_ = true;
        last_command_id_ = command.command_id;
        SPDLOG_WARN("GcsNetFire: 拒绝发射 command_id={} reason={}",
                    static_cast<unsigned long long>(command.command_id),
                    static_cast<int>(reason));
        PublishAck(command, common::NetFireResult::kRejected, reason);
    };

    if (command.protocol_version != kGcsNetFireProtocolVersion) {
        reject(common::NetFireReason::kUnsupportedVersion);
        return;
    }

    // 同一 command_id 只执行一次：重传帧回放上一次结论，绝不重复触发。
    if (has_last_command_ && command.command_id == last_command_id_) {
        rejected_.fetch_add(1, std::memory_order_acq_rel);
        SPDLOG_INFO("GcsNetFire: 重复指令 command_id={}，回放上次结论",
                    static_cast<unsigned long long>(command.command_id));
        PublishAck(command, last_result_, common::NetFireReason::kDuplicateCommand);
        return;
    }

    if ((command.flags & kGcsNetFireFlagConfirmed) == 0) {
        reject(common::NetFireReason::kConfirmationMissing);
        return;
    }

    const uint64_t valid_for_ms =
        command.header.valid_for_ms == 0 ? kDefaultCommandValidMs
                                         : command.header.valid_for_ms;
    if (command.received_ms == 0 || now_ms < command.received_ms ||
        now_ms - command.received_ms > valid_for_ms) {
        reject(common::NetFireReason::kCommandStale);
        return;
    }

    if (!latest_state_.has_value()) {
        reject(common::NetFireReason::kFlightStateStale);
        return;
    }
    const common::FlightStateSnapshot& state = *latest_state_;
    const uint64_t state_age_ms =
        now_ms > state.header.receive_time_ms
            ? now_ms - state.header.receive_time_ms
            : 0;
    if (state_age_ms > kFlightStateTimeoutMs) {
        reject(common::NetFireReason::kFlightStateStale);
        return;
    }
    if (!state.landed_state_valid) {
        // 落地状态未知时不能确认"地面安全"，按飞行状态不可用拒绝。
        reject(common::NetFireReason::kFlightStateStale);
        return;
    }
    if (state.armed) {
        reject(common::NetFireReason::kArmed);
        return;
    }
    if (!state.landed) {
        reject(common::NetFireReason::kNotLanded);
        return;
    }

    if (actuator_ == nullptr) {
        reject(common::NetFireReason::kActuatorUnavailable);
        return;
    }
    if (actuator_->Faulted()) {
        reject(common::NetFireReason::kActuatorFaulted);
        return;
    }
    if (!actuator_->Ready()) {
        reject(common::NetFireReason::kActuatorBusy);
        return;
    }
    if (actuator_->RemainingShots() == 0) {
        reject(common::NetFireReason::kNoRemainingShots);
        return;
    }
    if (last_fire_ms_ != 0 &&
        now_ms - last_fire_ms_ < static_cast<uint64_t>(config_.gcs_fire_cooldown.count())) {
        reject(common::NetFireReason::kCooldown);
        return;
    }

    if (!actuator_->Trigger()) {
        reject(common::NetFireReason::kActuatorUnavailable);
        return;
    }

    // 受理成功：记录状态并把"完成回执"交给 UpdateActuator 在时序走完后补发。
    accepted_.fetch_add(1, std::memory_order_acq_rel);
    last_fire_ms_ = now_ms;
    last_result_ = common::NetFireResult::kAccepted;
    last_reason_ = common::NetFireReason::kNone;
    has_last_command_ = true;
    last_command_id_ = command.command_id;
    pending_completion_ = true;
    pending_command_id_ = command.command_id;
    SPDLOG_WARN("GcsNetFire: 地面站直控发射已入队 command_id={}（落地未解锁）",
                static_cast<unsigned long long>(command.command_id));
    PublishAck(command, common::NetFireResult::kAccepted,
               common::NetFireReason::kNone);
}

void GcsNetFire::PublishAck(const common::NetFireCommand& command,
                            common::NetFireResult result,
                            common::NetFireReason reason) {
    common::NetFireAck ack;
    ack.header.sequence = ++ack_seq_;
    ack.header.receive_time_ms = MonotonicMs();
    ack.header.source_id = command.header.source_id;
    ack.header.health = 1;
    ack.command_id = command.command_id;
    ack.result = result;
    ack.reason = reason;
    ack.actuator_state = ActuatorStateCode();
    (void)ack_output_.Publish(
        std::make_shared<const common::NetFireAck>(ack));
}

uint8_t GcsNetFire::ActuatorStateCode() const {
    if (actuator_ == nullptr || actuator_->Faulted()) {
        return 2;
    }
    return actuator_->Ready() ? 0 : 1;
}

}  // namespace drone::state_machine
