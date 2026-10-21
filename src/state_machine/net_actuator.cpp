#include "state_machine/net_actuator.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <spdlog/spdlog.h>

#include "communication/px4_link.h"
#include "config/config.h"

namespace drone::state_machine {

namespace {

/// MAV_CMD_DO_SET_ACTUATOR（MAVLink common.xml #187）。
/// p1..p6 = Actuator Set 1..6 归一化值（-1..1，NaN=忽略）；p7=执行器组序号。
constexpr uint16_t kMavCmdDoSetActuator = 187;

constexpr uint8_t kMavResultAccepted = 0;    // MAV_RESULT_ACCEPTED
constexpr uint8_t kMavResultInProgress = 5;  // MAV_RESULT_IN_PROGRESS

/// 说明书要求发射后保持 1~2s 再复位；超出范围只告警，不阻止启动
/// （实际行程仍以 G4-3 台架实测冻结）。
void WarnIfResetDelayOutsideManualRange(const config::ActuatorConfig& config) {
    if (config.type != "single_pwm") {
        return;
    }
    if (config.reset_delay < std::chrono::milliseconds(1000) ||
        config.reset_delay > std::chrono::milliseconds(2000)) {
        SPDLOG_WARN("当前捕网枪说明书要求发射保持 1~2s: reset_delay_ms={}，"
                    "请以 G4-3 实测行程冻结",
                    config.reset_delay.count());
    }
}

/// 单发舵机式执行器：PX4 侧把物理输出的 Output Function 配为
/// `Peripheral via Actuator Set N`，并把 -1..1 映射到该输出通道的
/// PWM_MAIN_MINn/MAXn（AUX 输出为 PWM_AUX_*，标准 1000..2000µs）；
/// 机载只发送归一化控制量。当前装机（2026-10-21）：MAIN5 = Actuator Set 1。
///
/// 内部时序（状态机只转发 ActuatorFeedback，计时全部在本类内推进）：
///   Ready --Trigger--> FireQueued
///   FireQueued --187 ACK ACCEPTED--> FireAcked
///   FireAcked --(同一拍)--> Holding
///   Holding --now>=ACK 时刻+reset_delay--> ResetQueued（发复位行程）
///   ResetQueued --187 ACK ACCEPTED--> ResetAcked
///   ResetAcked --(同一拍)--> Settling
///   Settling --now>=ACK 时刻+reset_settle--> Ready
/// 任意等待态 187 ACK 超时 / 非 ACCEPTED / IN_PROGRESS 之外的非法结果 /
/// PX4 断连 → Faulted（终态，不自动恢复、不重试、不补发复位）。
/// ACK 关联只使用 CommandAckStatus 的 187 专用计数/终态字段：复位等待跨
/// S12~S14，同期 AUTO.LAND(176)/DISARM(400) 的 ACK 不得覆盖或推进本时序。
class SinglePwmNetActuator final : public INetActuator {
public:
    SinglePwmNetActuator(config::ActuatorConfig config,
                         communication::IPx4CommandSender* sender)
        : config_(std::move(config)), sender_(sender) {}

    bool Trigger() override {
        if (state_ != State::kReady || remaining_shots_ == 0) {
            return false;
        }
        // 基线必须在入队前抓取：PX4 worker 可能在 SendCommand() 返回前就把
        // 该命令的 ACK 写入快照；入队后再抓会把这个 ACK 当成旧 ACK 吞掉，
        // 执行器将永久停在等待态（既不推进也不故障）。
        CaptureAckBaseline();
        if (!SendNorm(config_.net_fire_norm, "触发")) {
            return false;
        }
        remaining_shots_ = 0;
        fired_ = true;
        state_ = State::kFireQueued;
        return true;
    }

    bool Reset() override {
        if (Faulted()) {
            return false;
        }
        if (!SendNorm(config_.net_idle_norm, "复位行程")) {
            return false;
        }
        state_ = State::kReady;
        return true;
    }

    void Update(const ActuatorFeedback& feedback) override {
        if (state_ == State::kFaulted || state_ == State::kReady) {
            return;
        }
        if (ProcessAck(feedback)) {
            return;
        }
        if (!feedback.px4_connected &&
            (state_ == State::kFireQueued || state_ == State::kHolding ||
             state_ == State::kResetQueued)) {
            EnterFault("等待 PX4 ACK 期间连接断开");
            return;
        }
        if (state_ == State::kFireAcked) {
            state_ = State::kHolding;
        }
        if (state_ == State::kHolding && feedback.now_ms >= hold_deadline_ms_) {
            // 同 Trigger：复位命令的 ACK 也可能在入队返回前到达，先抓基线。
            CaptureAckBaseline();
            if (!SendNorm(config_.net_idle_norm, "复位行程")) {
                return;  // 连接正常但队列满：下拍重试
            }
            state_ = State::kResetQueued;
            return;
        }
        if (state_ == State::kResetAcked) {
            state_ = State::kSettling;
        }
        if (state_ == State::kSettling &&
            feedback.now_ms >= settle_deadline_ms_) {
            state_ = State::kReady;
        }
    }

    bool Ready() const override { return state_ == State::kReady; }
    uint32_t RemainingShots() const override { return remaining_shots_; }
    bool Fired() const override { return fired_; }
    bool Faulted() const override { return state_ == State::kFaulted; }
    const char* FaultReason() const override { return fault_reason_; }
    const char* Type() const override { return "single_pwm"; }

private:
    enum class State {
        kReady,
        kFireQueued,
        kFireAcked,
        kHolding,
        kResetQueued,
        kResetAcked,
        kSettling,
        kFaulted,
    };

    /// 抓取 187 专用 ACK/超时计数基线，之后只处理“新到达”的 187 ACK。
    /// 必须在对应命令入队（SendNorm）之前调用，见 Trigger/Update 中的说明。
    void CaptureAckBaseline() {
        const auto ack = sender_->AckStatus();
        ack_baseline_ = ack.do_set_actuator_ack_count;
        timeout_baseline_ = ack.do_set_actuator_timeout_count;
    }

    /// 处理本拍新到达的 187 ACK/超时。返回 true 表示已判故障、本拍不再推进。
    bool ProcessAck(const ActuatorFeedback& feedback) {
        const auto& ack = feedback.ack;
        // 只认 187 专用计数：176/400 等命令即使在同一拍到达，也不会推进
        // 本状态机，更不会覆盖 187 的终态。
        const bool matched_new =
            ack.do_set_actuator_ack_count != ack_baseline_;
        const bool timeout_new =
            ack.do_set_actuator_timeout_count != timeout_baseline_;
        ack_baseline_ = ack.do_set_actuator_ack_count;
        timeout_baseline_ = ack.do_set_actuator_timeout_count;

        if (timeout_new) {
            EnterFault("等待 187 ACK 超时");
            return true;
        }
        if (!matched_new) {
            return false;  // 无新 187 ACK：无关命令的 ACK/超时已由专用计数隔离
        }
        if (ack.do_set_actuator_last_result == kMavResultInProgress) {
            return false;  // 仍在执行：保持等待，等最终结果
        }
        if (ack.do_set_actuator_last_result != kMavResultAccepted) {
            EnterFault("187 ACK 结果非 ACCEPTED");
            return true;
        }

        // 计时起点优先用 PX4 ACK 到达时间；缺失时回退本拍时钟。
        const uint64_t ack_time = ack.do_set_actuator_last_time_ms == 0
                                      ? feedback.now_ms
                                      : ack.do_set_actuator_last_time_ms;
        if (state_ == State::kFireQueued) {
            state_ = State::kFireAcked;
            hold_deadline_ms_ =
                ack_time + static_cast<uint64_t>(config_.reset_delay.count());
        } else if (state_ == State::kResetQueued) {
            state_ = State::kResetAcked;
            settle_deadline_ms_ =
                ack_time + static_cast<uint64_t>(config_.reset_settle.count());
        }
        return false;
    }

    bool SendNorm(double normalized, const char* action) {
        if (sender_ == nullptr) {
            return false;
        }
        const int32_t channel = config_.net_channel;
        if (channel < 1 || channel > 6) {
            EnterFault("执行器通道越界");
            return false;
        }
        std::array<float, 7> params;
        params.fill(std::numeric_limits<float>::quiet_NaN());
        // 只有本 Set 对应参数位填值；其余 NaN=忽略（不得填 0，0 为中位行程）。
        params[static_cast<std::size_t>(channel - 1)] =
            static_cast<float>(normalized);
        params[6] = 0.0F;  // PX4 仅处理执行器组 0
        const bool queued = sender_->SendCommand(
            kMavCmdDoSetActuator, params[0], params[1], params[2], params[3],
            params[4], params[5], params[6]);
        if (queued) {
            SPDLOG_WARN("捕网执行器{}命令已入队: type={} set={} norm={:.2f}",
                        action, Type(), channel, normalized);
            return true;
        }
        const uint64_t count = ++failure_count_;
        if (count == 1 || count % 100 == 0) {
            SPDLOG_WARN("捕网执行器{}命令入队失败（下拍重试）: type={} set={} "
                        "count={}",
                        action, Type(), channel, count);
        }
        return false;
    }

    void EnterFault(const char* reason) {
        if (state_ == State::kFaulted) {
            return;
        }
        state_ = State::kFaulted;
        fault_reason_ = reason;
        SPDLOG_ERROR("捕网执行器故障: type={} set={} reason={}", Type(),
                     config_.net_channel, reason);
    }

    const config::ActuatorConfig config_;
    communication::IPx4CommandSender* const sender_;
    State state_ = State::kReady;
    uint32_t remaining_shots_ = 1;
    bool fired_ = false;
    uint64_t ack_baseline_ = 0;
    uint64_t timeout_baseline_ = 0;
    uint64_t hold_deadline_ms_ = 0;
    uint64_t settle_deadline_ms_ = 0;
    uint64_t failure_count_ = 0;
    const char* fault_reason_ = "";
};

}  // namespace

std::unique_ptr<INetActuator> CreateNetActuator(
    const config::ActuatorConfig& config,
    communication::IPx4CommandSender* sender) {
    if (sender == nullptr) {
        return nullptr;  // 影子/未开控制：执行器物理缺席
    }
    config.Validate();
    WarnIfResetDelayOutsideManualRange(config);
    if (config.type == "single_pwm") {
        return std::make_unique<SinglePwmNetActuator>(config, sender);
    }
    throw std::invalid_argument(
        "不支持的 actuator.type: " + config.type +
        "（当前支持 single_pwm；多发实现待接入）");
}

}  // namespace drone::state_machine
