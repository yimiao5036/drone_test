/**
 * @file mission_state_machine.h
 * @brief 任务状态机部件接口（IMissionStateMachine）
 *
 * 属于 drone/state_machine 模块。职责：消费目标、飞行状态与健康快照，
 * 按任务状态机（S0~S16，见 docs/状态机设计.md）决策，输出控制意图
 * （ControlIntent）与任务状态回传（MissionStatus）。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，状态转移规则在实现期按状态机设计文档接入。
 * - MissionStateMachineStub 为骨架占位实现：生命周期可运行，业务方法
 *   记录"未实现"节流日志并返回默认值。
 *
 * 数据流：
 *   common::Topic<TargetState> ─┐
 *   common::Topic<FlightStateSnapshot> ─┼──► IMissionStateMachine ──┬─► common::Topic<ControlIntent>
 *   common::Topic<HealthStatus> ─┘                                   └─► common::Topic<MissionStatus>
 * 可替换边界：不依赖具体设备。
 */
#pragma once

#include <cstdint>

#include "common/topic.h"
#include "common/types.h"

namespace drone::state_machine {

/// 任务状态机部件抽象接口。
class IMissionStateMachine {
public:
    virtual ~IMissionStateMachine() = default;

    // ---- 生命周期 ----
    /// 启动状态机（启动决策线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止状态机；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;

    // ---- 输入 ----
    /// 绑定目标、飞行状态与健康快照输入主题。
    virtual void SetInputs(common::Topic<common::TargetState>& target,
                           common::Topic<common::FlightStateSnapshot>& flight,
                           common::Topic<common::HealthStatus>& health) = 0;

    // ---- 输出 ----
    /// 控制意图输出主题：common::ControlIntent（供 IFlightController 消费）。
    virtual common::Topic<common::ControlIntent>& IntentOutput() = 0;
    /// 任务状态回传输出主题：common::MissionStatus（供地面站回传）。
    virtual common::Topic<common::MissionStatus>& StatusOutput() = 0;

    // ---- 状态查询 ----
    /// 当前任务状态（S0~S16，见 common::MissionState）。
    virtual common::MissionState CurrentState() const = 0;
    /// 累计状态转移次数。
    virtual uint64_t TransitionCount() const = 0;
    /// 累计错误次数。
    virtual uint64_t ErrorCount() const = 0;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class MissionStateMachineStub final : public IMissionStateMachine {
public:
    MissionStateMachineStub();
    ~MissionStateMachineStub() override;

    MissionStateMachineStub(const MissionStateMachineStub&) = delete;
    MissionStateMachineStub& operator=(const MissionStateMachineStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInputs(common::Topic<common::TargetState>& target,
                   common::Topic<common::FlightStateSnapshot>& flight,
                   common::Topic<common::HealthStatus>& health) override;

    common::Topic<common::ControlIntent>& IntentOutput() override;
    common::Topic<common::MissionStatus>& StatusOutput() override;

    common::MissionState CurrentState() const override;
    uint64_t TransitionCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    common::MissionState state_ = common::MissionState::kBoot;
    uint64_t transition_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::ControlIntent> intent_output_;
    common::Topic<common::MissionStatus> status_output_;
};

}  // namespace drone::state_machine
