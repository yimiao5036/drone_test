#include "state_machine/net_actuator.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <spdlog/spdlog.h>

#include "communication/px4_link.h"
#include "config/config.h"

namespace drone::state_machine {

namespace {

/// MAV_CMD_DO_SET_ACTUATOR（MAVLink common.xml #187）。
/// p1..p6 = Actuator Set 1..6 归一化值（-1..1，NaN=忽略）；p7=执行器组序号。
constexpr uint16_t kMavCmdDoSetActuator = 187;

/// 单发舵机式执行器：PX4 侧把物理输出的 Output Function 配为
/// `Peripheral via Actuator Set N`，并把 -1..1 映射到该输出通道的
/// PWM_MAIN_MINn/MAXn（AUX 输出为 PWM_AUX_*，标准 1000..2000µs）；
/// 机载只发送归一化控制量。当前装机（2026-10-21）：MAIN5 = Actuator Set 1。
/// 后续多发实现应实现同一个 INetActuator，并在 CreateNetActuator 中按
/// actuator.type 接入；状态机不感知通道数/弹仓结构。
class SinglePwmNetActuator final : public INetActuator {
public:
    SinglePwmNetActuator(config::ActuatorConfig config,
                         communication::IPx4CommandSender* sender)
        : config_(config), sender_(sender) {}

    bool Trigger() override { return SendNorm(config_.net_fire_norm, "触发"); }

    bool Reset() override { return SendNorm(config_.net_idle_norm, "复位行程"); }

    const char* Type() const override { return "single_pwm"; }

private:
    bool SendNorm(double normalized, const char* action) {
        if (sender_ == nullptr) {
            return false;
        }
        const int32_t channel = config_.net_channel;
        if (channel < 1 || channel > 6) {
            SPDLOG_ERROR("捕网执行器通道越界: type={} channel={}", Type(), channel);
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

    const config::ActuatorConfig config_;
    communication::IPx4CommandSender* const sender_;
    uint64_t failure_count_ = 0;
};

}  // namespace

std::unique_ptr<INetActuator> CreateNetActuator(
    const config::ActuatorConfig& config,
    communication::IPx4CommandSender* sender) {
    if (sender == nullptr) {
        return nullptr;  // 影子/未开控制：执行器物理缺席
    }
    config.Validate();
    if (config.type == "single_pwm") {
        return std::make_unique<SinglePwmNetActuator>(config, sender);
    }
    throw std::invalid_argument(
        "不支持的 actuator.type: " + config.type +
        "（当前支持 single_pwm；多发实现待接入）");
}

}  // namespace drone::state_machine
