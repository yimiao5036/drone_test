/**
 * @file video_sender.cpp
 * @brief IVideoSender 骨架占位实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（编码后端、图传协议、
 * 拥塞丢帧策略）在实现期接入。
 */
#include "video_transmission/video_sender.h"

#include <spdlog/spdlog.h>

namespace drone::video_transmission {

VideoSenderStub::VideoSenderStub() {
    SPDLOG_INFO("图传发送部件骨架创建");
}

VideoSenderStub::~VideoSenderStub() {
    SPDLOG_INFO("图传发送部件骨架销毁");
}

bool VideoSenderStub::Start() {
    running_ = true;
    SPDLOG_INFO("图传发送部件骨架启动");
    return true;
}

void VideoSenderStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("图传发送部件骨架停止");
}

bool VideoSenderStub::IsRunning() const {
    return running_;
}

void VideoSenderStub::SetInput(common::Topic<video::FrameHandle>& /*input*/) {
    // 骨架期忽略输入绑定；实现期保存订阅并启动消费
}

uint64_t VideoSenderStub::SentFrameCount() const {
    return sent_count_;
}

uint64_t VideoSenderStub::DroppedFrameCount() const {
    return dropped_count_;
}

uint64_t VideoSenderStub::ErrorCount() const {
    return error_count_;
}

}  // namespace drone::video_transmission
