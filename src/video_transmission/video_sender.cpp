/**
 * @file video_sender.cpp
 * @brief IVideoSender 真实实现（VideoSender）与骨架占位（VideoSenderStub）
 *
 * VideoSender：订阅 FrameCompositor 产出的标注帧（kAnnotatedFrame，
 * NV12 FrameHandle），经 IVideoEncoderBackend 编码并 RTSP 推流给图传。
 * - 图传拥塞/失败只丢图传帧：订阅队列使用 kDropOldest，编码失败直接跳过该帧，
 *   不阻塞、不反压上游感知链路。
 * - 编码后端经工厂 CreateVideoEncoderBackend 创建（rkmpp 硬编优先 / 软编回退），
 *   便于替换图传协议或编码设备。
 * - 消费线程与编码后端同一线程串行推进（单消费者订阅句柄），无需额外锁。
 *
 * VideoSenderStub：保留供骨架冒烟测试与装配联调。
 */
#include "video_transmission/video_sender.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <spdlog/spdlog.h>

namespace drone::video_transmission {

/// VideoSender 实现细节（PIMPL）：消费线程、编码后端与计数。
struct VideoSender::Impl {
    explicit Impl(VideoSenderConfig cfg) : config(std::move(cfg)) {
        if (config.input_queue == 0) {
            throw std::invalid_argument("图传订阅队列容量必须大于 0");
        }
    }

    ~Impl() { Stop(); }

    VideoSenderConfig config;
    std::atomic<bool> stop_requested{false};
    std::thread thread;
    std::unique_ptr<IVideoEncoderBackend> backend;

    common::Topic<video::FrameHandle>::Subscription input_sub;
    common::Topic<video::FrameHandle>* input_topic = nullptr;

    std::atomic<uint64_t> sent_count{0};
    std::atomic<uint64_t> dropped_count{0};
    std::atomic<uint64_t> error_count{0};

    /// 消费线程主循环：取标注帧 → 编码推流；失败即丢该帧，不反压。
    void SendLoop() {
        while (!stop_requested.load()) {
            auto message = input_sub.WaitTakeFor(std::chrono::milliseconds(100));
            if (!message) {
                continue;  // 超时或主题关闭；循环顶检查停止标志
            }
            const auto& handle = **message;
            if (!handle.Valid()) {
                ++error_count;
                ++dropped_count;
                continue;
            }
            if (backend != nullptr && !backend->EncodeFrame(handle)) {
                ++dropped_count;   // 编码/推流失败：丢图传帧，不反压
                continue;
            }
            if (backend != nullptr) {
                ++sent_count;
            }
        }
    }

    void Stop() {
        if (!thread.joinable()) {
            return;
        }
        stop_requested = true;
        input_sub.Reset();  // 唤醒 WaitTakeFor 中的等待
        thread.join();
        if (backend != nullptr) {
            backend->Stop();
        }
    }
};

VideoSender::VideoSender(VideoSenderConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
    SPDLOG_INFO("图传发送器创建: 编码={} 地址={} 分辨率={}x{}@{}fps",
                impl_->config.encode.codec, impl_->config.encode.url,
                impl_->config.encode.width, impl_->config.encode.height,
                impl_->config.encode.fps);
}

VideoSender::~VideoSender() {
    if (impl_ != nullptr) {
        impl_->Stop();
    }
    SPDLOG_INFO("图传发送器销毁");
}

bool VideoSender::Start() {
    if (impl_->thread.joinable()) {
        return true;  // 已启动，幂等
    }
    impl_->stop_requested = false;
    // 重启时复用已建后端，不重新走工厂（避免后端/RTSP 会话重建抖动）
    if (impl_->backend == nullptr) {
        if (impl_->config.backend_factory) {
            impl_->backend = impl_->config.backend_factory();
        } else {
            impl_->backend = CreateVideoEncoderBackend(impl_->config.encode);
        }
    }
    if (impl_->backend == nullptr) {
        impl_->error_count.fetch_add(1);
        SPDLOG_ERROR("图传发送器创建编码后端失败");
        return false;
    }
    if (!impl_->backend->Start()) {
        impl_->error_count.fetch_add(1);
        SPDLOG_ERROR("图传发送器后端启动失败（推流地址不可达或无可用编码器）");
        return false;
    }
    // Stop 会 Reset 订阅，重启时重新订阅（幂等：已打开则不重复）
    if (!impl_->input_sub.IsOpen() && impl_->input_topic != nullptr) {
        impl_->input_sub =
            impl_->input_topic->Subscribe(impl_->config.input_queue);
    }
    impl_->thread = std::thread(&Impl::SendLoop, impl_.get());
    SPDLOG_INFO("图传发送器启动");
    return true;
}

void VideoSender::Stop() {
    impl_->Stop();
    SPDLOG_INFO("图传发送器停止");
}

bool VideoSender::IsRunning() const {
    return impl_->thread.joinable();
}

void VideoSender::SetInput(common::Topic<video::FrameHandle>& input) {
    // 队列容量小（默认 2）+ kDropOldest：图传落后只丢图传帧，不反压感知
    impl_->input_topic = &input;
    impl_->input_sub = input.Subscribe(impl_->config.input_queue);
}

uint64_t VideoSender::SentFrameCount() const {
    return impl_->sent_count.load();
}

uint64_t VideoSender::DroppedFrameCount() const {
    return impl_->dropped_count.load();
}

uint64_t VideoSender::ErrorCount() const {
    return impl_->error_count.load();
}

// ---------------------------------------------------------------------------
// 骨架占位实现（保留供冒烟测试与装配联调）
// ---------------------------------------------------------------------------

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
    // 骨架期忽略输入绑定
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
