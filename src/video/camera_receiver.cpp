/**
 * @file camera_receiver.cpp
 * @brief ICameraReceiver 骨架占位实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（TCP 建连、码流接收、
 * 断线重连）在实现期按摄像头厂商协议接入。
 */
#include "video/camera_receiver.h"

#include <spdlog/spdlog.h>

namespace drone::video {

CameraReceiverStub::CameraReceiverStub() {
    SPDLOG_INFO("摄像头接收部件骨架创建");
}

CameraReceiverStub::~CameraReceiverStub() {
    SPDLOG_INFO("摄像头接收部件骨架销毁");
}

bool CameraReceiverStub::Start() {
    running_ = true;
    SPDLOG_INFO("摄像头接收部件骨架启动");
    return true;
}

void CameraReceiverStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("摄像头接收部件骨架停止");
}

bool CameraReceiverStub::IsRunning() const {
    return running_;
}

bool CameraReceiverStub::IsConnected() const {
    // 骨架期未实现建连，恒为 false
    return false;
}

uint64_t CameraReceiverStub::ConnectCount() const {
    return connect_count_;
}

uint64_t CameraReceiverStub::ReceivedBytes() const {
    return received_bytes_;
}

uint64_t CameraReceiverStub::ErrorCount() const {
    return error_count_;
}

common::Topic<common::EncodedFrame>& CameraReceiverStub::StreamOutput() {
    return stream_output_;
}

}  // namespace drone::video
