/**
 * @file camera_receiver_stub.cpp
 * @brief ICameraReceiver 骨架占位实现（CameraReceiverStub）
 *
 * 骨架期占位：生命周期可运行、输出主题可发布，业务逻辑（真实建连/接收）
 * 由 CameraReceiver（同目录 camera_receiver.cpp）实现。保留 Stub 用于
 * 装配联调与测试。
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
