/**
 * @file video_decoder.cpp
 * @brief IVideoDecoder 骨架占位实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（MPP 硬件解码、
 * 帧时间标记、内存池接入）在实现期接入。
 */
#include "video/video_decoder.h"

#include <spdlog/spdlog.h>

namespace drone::video {

VideoDecoderStub::VideoDecoderStub() {
    SPDLOG_INFO("视频解码部件骨架创建");
}

VideoDecoderStub::~VideoDecoderStub() {
    SPDLOG_INFO("视频解码部件骨架销毁");
}

bool VideoDecoderStub::Start() {
    running_ = true;
    SPDLOG_INFO("视频解码部件骨架启动");
    return true;
}

void VideoDecoderStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("视频解码部件骨架停止");
}

bool VideoDecoderStub::IsRunning() const {
    return running_;
}

void VideoDecoderStub::SetInput(common::Topic<common::EncodedFrame>& /*input*/) {
    // 骨架期忽略输入绑定；实现期保存订阅并启动消费
}

common::Topic<FrameHandle>& VideoDecoderStub::FrameOutput() {
    return frame_output_;
}

uint64_t VideoDecoderStub::DecodedFrameCount() const {
    return decoded_count_;
}

uint64_t VideoDecoderStub::DroppedFrameCount() const {
    return dropped_count_;
}

uint64_t VideoDecoderStub::ErrorCount() const {
    return error_count_;
}

}  // namespace drone::video
