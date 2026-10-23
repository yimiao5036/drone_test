#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>

#include "common/topic.h"
#include "common/types.h"
#include "config/config.h"
#include "state_machine/net_actuator.h"

namespace drone {

namespace communication {
class IPx4CommandSender;
}

namespace state_machine {

/// 65016/65017 协议版本与确认位。必须与
/// `communication/ground_station_link.h` 的 kNetFireProtocolVersion /
/// kNetFireFlagConfirmed 一致（gcs_net_fire.cpp 内有 static_assert 守护）。
constexpr uint8_t kGcsNetFireProtocolVersion = 1;
constexpr uint8_t kGcsNetFireFlagConfirmed = 1U << 0U;

/// 地面站直控发射服务（65016 指令 → 捕网执行器，65017 回执）。
///
/// 职责边界：
///   - 只在 `actuator.gcs_fire_enabled=true` 时被 DroneApplication 装配；
///     与 `interception.enabled` 互斥（配置加载期 fail-fast），避免自动拦截
///     链与地面站直控各自持有执行器实例重复点火。
///   - 本服务不改变任务状态机：不推进 MissionState、不发布 ControlIntent、
///     不发 ARM/起飞命令，只占用执行器通道。
///
/// 触发门禁（全部满足才真正入队触发；否则立刻回 65017 拒绝回执）：
///   1. 指令新鲜：`received_ms` 距今不超过 `header.valid_for_ms`（链路层置 2000ms）；
///   2. 飞行状态新鲜（1s 内）且 `landed_state_valid && landed` 为真、`armed` 为假；
///   3. 执行器存在、未进入终态故障、`Ready()`、`RemainingShots()>0`；
///   4. 距上一次成功触发不小于 `actuator.gcs_fire_cooldown`（默认 3000ms）；
///   5. 同一 `command_id` 只执行一次，重复帧回放上一次结论（不重复触发）。
///
/// 单发（single_pwm）语义：触发一次后发数归零，执行器保持"已发射"直到人工
/// 维护复位；本服务不自动补发、不恢复发数。
class GcsNetFire final {
public:
    /// @param config 已通过 Validate 的 actuator 配置
    /// @param sender PX4 命令窄接口；为空时服务仍装配但所有发射请求按
    ///               `kActuatorUnavailable` 拒绝（保持影子/无控制装配的物理缺席）
    GcsNetFire(config::ActuatorConfig config,
               communication::IPx4CommandSender* sender);
    ~GcsNetFire();

    GcsNetFire(const GcsNetFire&) = delete;
    GcsNetFire& operator=(const GcsNetFire&) = delete;

    /// 绑定飞行状态输入（Px4Link::StateOutput）。必须在 Start 前调用。
    void SetFlightStateInput(common::Topic<common::FlightStateSnapshot>& state);
    /// 绑定 65016 指令输入（GroundStationLink::NetFireCommandOutput）。
    /// 必须在 Start 前调用。
    void SetCommandInput(common::Topic<common::NetFireCommand>& command);

    /// 65017 回执输出（接 GroundStationLink::SetNetFireAckInput）。
    common::Topic<common::NetFireAck>& AckOutput() { return ack_output_; }

    /// 启动工作线程（20ms 一拍：消费指令 + 推进执行器时序 + 完成回执）。
    bool Start();
    /// 停止并等待工作线程退出；幂等。
    void Stop();
    bool IsRunning() const { return running_.load(std::memory_order_acquire); }

    /// 已入队触发次数（诊断）。
    uint64_t AcceptedCount() const {
        return accepted_.load(std::memory_order_acquire);
    }
    /// 被拒绝次数（诊断，含重复帧回执）。
    uint64_t RejectedCount() const {
        return rejected_.load(std::memory_order_acquire);
    }

private:
    using FlightSubscription =
        common::Topic<common::FlightStateSnapshot>::Subscription;
    using CommandSubscription =
        common::Topic<common::NetFireCommand>::Subscription;

    void Loop();
    /// 处理一条 65016 指令：门禁 → 触发或拒绝 → 发布 65017。
    void HandleCommand(const common::NetFireCommand& command, uint64_t now_ms);
    /// 推进执行器时序；完成一次触发-复位周期后补发 kCompleted 回执。
    void UpdateActuator(uint64_t now_ms);
    void PublishAck(const common::NetFireCommand& command,
                    common::NetFireResult result, common::NetFireReason reason);
    uint8_t ActuatorStateCode() const;

    config::ActuatorConfig config_;
    communication::IPx4CommandSender* sender_ = nullptr;
    std::unique_ptr<INetActuator> actuator_;

    common::Topic<common::FlightStateSnapshot>* flight_state_topic_ = nullptr;
    common::Topic<common::NetFireCommand>* command_topic_ = nullptr;
    common::Topic<common::NetFireAck> ack_output_;

    FlightSubscription flight_state_subscription_;
    CommandSubscription command_subscription_;
    std::optional<common::FlightStateSnapshot> latest_state_;

    std::atomic<bool> running_{false};
    std::thread worker_;
    std::atomic<uint64_t> accepted_{0};
    std::atomic<uint64_t> rejected_{0};

    // 以下状态只在工作线程内读写。
    bool has_last_command_ = false;
    uint32_t last_command_id_ = 0;
    common::NetFireResult last_result_ = common::NetFireResult::kRejected;
    common::NetFireReason last_reason_ = common::NetFireReason::kNone;
    uint32_t ack_seq_ = 0;
    uint64_t last_fire_ms_ = 0;
    bool pending_completion_ = false;
    uint32_t pending_command_id_ = 0;
    bool fault_logged_ = false;
};

}  // namespace state_machine
}  // namespace drone
