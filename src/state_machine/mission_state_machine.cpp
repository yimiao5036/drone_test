/**
 * @file mission_state_machine.cpp
 * @brief IMissionStateMachine 骨架占位实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（S0~S16 状态转移规则、
 * 控制意图生成）在实现期按状态机设计文档接入。
 */
#include "state_machine/mission_state_machine.h"

#include <spdlog/spdlog.h>

namespace drone::state_machine {

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

void MissionStateMachineStub::SetInputs(common::Topic<common::TargetState>& /*target*/,
                                        common::Topic<common::FlightStateSnapshot>& /*flight*/,
                                        common::Topic<common::HealthStatus>& /*health*/) {
    // 骨架期忽略输入绑定；实现期保存订阅并启动决策周期
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
