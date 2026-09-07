/**
 * @file flight_controller.cpp
 * @brief IFlightController 骨架占位实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（控制意图 → PX4
 * 设定值转换、控制算法）在实现期接入。
 */
#include "control/flight_controller.h"

#include <spdlog/spdlog.h>

namespace drone::control {

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

}  // namespace drone::control
