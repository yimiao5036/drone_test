/**
 * @file px4_link.cpp
 * @brief IPx4Link 骨架占位实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（MAVLink 解析、串口
 * 收发、ACK 关联、飞行状态快照维护）在实现期接入。
 */
#include "communication/px4_link.h"

#include <cstdint>

#include <spdlog/spdlog.h>

namespace drone::communication {

namespace {

/// 异常日志节流：第 1 次与每满 100 次才打印，避免高频异常刷屏。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

}  // namespace

Px4LinkStub::Px4LinkStub() {
    SPDLOG_INFO("PX4 通信部件骨架创建");
}

Px4LinkStub::~Px4LinkStub() {
    SPDLOG_INFO("PX4 通信部件骨架销毁");
}

bool Px4LinkStub::Start() {
    running_ = true;
    SPDLOG_INFO("PX4 通信部件骨架启动");
    return true;
}

void Px4LinkStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("PX4 通信部件骨架停止");
}

bool Px4LinkStub::IsRunning() const {
    return running_;
}

bool Px4LinkStub::IsConnected() const {
    // 骨架期未实现 MAVLink 心跳解析，恒为 false
    return false;
}

void Px4LinkStub::SetInput(common::Topic<common::Px4Setpoint>& /*setpoint*/) {
    // 骨架期忽略输入绑定；实现期保存订阅并启动发送
}

common::Topic<common::FlightStateSnapshot>& Px4LinkStub::StateOutput() {
    return state_output_;
}

bool Px4LinkStub::SendCommand(uint16_t /*mavlink_command*/, float /*param1*/,
                              float /*param2*/, float /*param3*/, float /*param4*/,
                              float /*param5*/, float /*param6*/, float /*param7*/) {
    // 骨架期未实现命令发送，返回失败并节流记录
    ++error_count_;
    if (ShouldLogThrottled(error_count_)) {
        SPDLOG_WARN("PX4 通信部件 SendCommand 未实现（骨架占位），累计调用 {}", error_count_);
    }
    return false;
}

uint64_t Px4LinkStub::SetpointSendCount() const {
    return setpoint_send_count_;
}

uint64_t Px4LinkStub::ReceiveCount() const {
    return receive_count_;
}

uint64_t Px4LinkStub::AckMatchCount() const {
    return ack_match_count_;
}

uint64_t Px4LinkStub::AckTimeoutCount() const {
    return ack_timeout_count_;
}

uint64_t Px4LinkStub::ErrorCount() const {
    return error_count_;
}

}  // namespace drone::communication
