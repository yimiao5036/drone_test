/**
 * @file ground_station_link.cpp
 * @brief IGroundStationLink 骨架占位实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（地面站 MAVLink
 * dialect 解析、任务输入与状态回传）在实现期接入。
 */
#include "communication/ground_station_link.h"

#include <spdlog/spdlog.h>

namespace drone::communication {

GroundStationLinkStub::GroundStationLinkStub() {
    SPDLOG_INFO("地面站通信部件骨架创建");
}

GroundStationLinkStub::~GroundStationLinkStub() {
    SPDLOG_INFO("地面站通信部件骨架销毁");
}

bool GroundStationLinkStub::Start() {
    running_ = true;
    SPDLOG_INFO("地面站通信部件骨架启动");
    return true;
}

void GroundStationLinkStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("地面站通信部件骨架停止");
}

bool GroundStationLinkStub::IsRunning() const {
    return running_;
}

bool GroundStationLinkStub::IsConnected() const {
    // 骨架期未实现地面站心跳解析，恒为 false
    return false;
}

common::Topic<common::GroundStationTarget>& GroundStationLinkStub::TargetOutput() {
    return target_output_;
}

void GroundStationLinkStub::SetInput(common::Topic<common::MissionStatus>& /*status*/) {
    // 骨架期忽略输入绑定；实现期保存订阅并启动回传
}

uint64_t GroundStationLinkStub::SendCount() const {
    return send_count_;
}

uint64_t GroundStationLinkStub::ReceiveCount() const {
    return receive_count_;
}

uint64_t GroundStationLinkStub::ErrorCount() const {
    return error_count_;
}

}  // namespace drone::communication
