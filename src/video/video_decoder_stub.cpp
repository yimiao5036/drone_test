/**
 * @file video_decoder_stub.cpp
 * @brief IVideoDecoder 骨架占位实现（VideoDecoderStub）
 *
 * 骨架期占位：生命周期可运行、输出主题可发布，业务逻辑（真实解码）
 * 由 VideoDecoder（同目录 video_decoder.cpp）实现。保留 Stub 用于
 * 装配联调与测试。
 */
#include "video/video_decoder.h"

#include <spdlog/spdlog.h>

namespace drone::video {

// 数据流：EncodedFrame Topic → 解码 → FrameHandle Topic。
// Stub 只暴露生命周期与输出主题，真实解码逻辑由 VideoDecoder 实现。

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

// 骨架期忽略输入绑定；实现期保存订阅并启动消费。
void VideoDecoderStub::SetInput(common::Topic<common::EncodedFrame>& /*input*/) {
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
