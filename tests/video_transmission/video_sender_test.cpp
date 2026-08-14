/**
 * @file video_sender_test.cpp
 * @brief 图传发送器（VideoSender）单元测试
 *
 * 通过注入 Mock 编码后端（backend_factory）验证不依赖真实 RTSP 目标的
 * 线程/发布/计数逻辑：
 * - 生命周期：Start/Stop 幂等
 * - 发送：注入有效帧 → 后端收到并计数 SentFrameCount
 * - 后端失败丢帧：EncodeFrame 返回 false → 计入 DroppedFrameCount
 * - 停机和重启
 *
 * 真实编码 + RTSP 推流链路（rkmpp 硬编/软件编码）依赖 FFmpeg 编码器与
 * 推流目标，留待香橙派实机 / RTSP 接收端联调验证。
 */
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "video/video_frame_pool.h"
#include "video_transmission/video_sender.h"

namespace drone::video_transmission {
namespace {

using testing::Test;

/// Mock 编码后端：记录调用，可配置失败语义。
class MockBackend final : public IVideoEncoderBackend {
public:
    bool start_result = true;
    bool encode_result = true;
    std::uint64_t start_count = 0;
    std::uint64_t encode_count = 0;
    std::uint64_t stop_count = 0;
    std::uint64_t error_override = 0;

    bool Start() override {
        ++start_count;
        return start_result;
    }
    void Stop() override { ++stop_count; }
    bool IsRunning() const override { return start_count > 0 && stop_count == 0; }
    bool EncodeFrame(const video::FrameHandle&) override {
        ++encode_count;
        return encode_result;
    }
    std::uint64_t SentFrameCount() const override { return encode_count; }
    std::uint64_t ErrorCount() const override { return start_result ? error_override : 1; }
};

/// 轮询等待条件满足（带超时）。
bool WaitFor(const std::function<bool()>& condition, int timeout_ms = 3000) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return condition();
}

class VideoSenderTest : public Test {
protected:
    void SetUp() override {
        frame_tmpl.width = 64;
        frame_tmpl.height = 64;
        frame_tmpl.hor_stride = 64;
        frame_tmpl.ver_stride = 64;
        frame_tmpl.format = video::PixelFormat::kYuv420SpNv12;
        pool_ = std::make_shared<video::VideoFramePool>(4, frame_tmpl);
    }

    /// 构建 VideoSender，注入 Mock 后端；返回裸指针供测试读取。
    /// Mock 所有权在 Start() 时转移给 VideoSender（由 sender 销毁）。
    std::unique_ptr<VideoSender> MakeSender(MockBackend** mock_out,
                                            std::size_t input_queue = 2) {
        VideoSenderConfig config;
        config.encode.url = "rtsp://127.0.0.1:8554/drone_out";
        config.encode.width = 64;
        config.encode.height = 64;
        config.encode.fps = 25;
        config.input_queue = input_queue;

        auto mock = std::make_unique<MockBackend>();
        MockBackend* raw = mock.get();
        if (mock_out) {
            *mock_out = raw;
        }
        // 工厂返回所有权；Start 时 VideoSender 接手并最终销毁 mock。
        // 测试通过 raw 裸指针只读状态，不参与所有权。
        config.backend_factory = [raw]() -> std::unique_ptr<IVideoEncoderBackend> {
            return std::unique_ptr<IVideoEncoderBackend>(raw);
        };
        (void)mock.release();  // 所有权交给 lambda：Start 时由 VideoSender 管理

        auto sender = std::make_unique<VideoSender>(std::move(config));
        sender->SetInput(frame_topic_);
        return sender;
    }

    /// 发布一帧有效 NV12 帧。
    void PublishFrame() {
        auto handle = pool_->Acquire();
        ASSERT_TRUE(handle.Valid());
        std::memset(handle.Data(), 128, pool_->SlotSize());
        (void)frame_topic_.Emplace(std::move(handle));
    }

    void TearDown() override {
        // Mock 所有权由 Start 后 VideoSender 管理；局部 sender 出作用域即析构
    }

    std::shared_ptr<video::VideoFramePool> pool_;
    common::Topic<video::FrameHandle> frame_topic_;
    video::VideoFrameInfo frame_tmpl;
};

TEST_F(VideoSenderTest, StartStopLifecycle) {
    MockBackend* mock = nullptr;
    auto sender = MakeSender(&mock);
    EXPECT_FALSE(sender->IsRunning());
    EXPECT_TRUE(sender->Start());
    EXPECT_TRUE(sender->IsRunning());
    EXPECT_TRUE(sender->Start());  // 幂等
    sender->Stop();
    EXPECT_FALSE(sender->IsRunning());
    sender->Stop();
}

TEST_F(VideoSenderTest, SendsFramesToBackend) {
    MockBackend* mock = nullptr;
    auto sender = MakeSender(&mock);
    ASSERT_TRUE(sender->Start());

    PublishFrame();
    EXPECT_TRUE(WaitFor([mock] { return mock->encode_count == 1; }));
    EXPECT_EQ(sender->SentFrameCount(), 1u);
    EXPECT_EQ(sender->DroppedFrameCount(), 0u);

    sender->Stop();
}

TEST_F(VideoSenderTest, BackendFailureDropsFrameNoBackpressure) {
    MockBackend* mock = nullptr;
    auto sender = MakeSender(&mock);
    mock->encode_result = false;
    ASSERT_TRUE(sender->Start());

    PublishFrame();
    EXPECT_TRUE(WaitFor([&] { return sender->DroppedFrameCount() == 1; }));
    EXPECT_EQ(mock->encode_count, 1u);
    EXPECT_EQ(sender->SentFrameCount(), 0u);
    // 链路不中断：后端一直报失败会持续丢帧计数而非崩溃
    PublishFrame();
    EXPECT_TRUE(WaitFor([&] { return sender->DroppedFrameCount() == 2; }));
    sender->Stop();
}

TEST_F(VideoSenderTest, StopStopsSending) {
    MockBackend* mock = nullptr;
    auto sender = MakeSender(&mock);
    ASSERT_TRUE(sender->Start());

    PublishFrame();
    EXPECT_TRUE(WaitFor([mock] { return mock->encode_count == 1; }));

    sender->Stop();
    PublishFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    EXPECT_EQ(mock->encode_count, 1u);
    EXPECT_FALSE(sender->IsRunning());
}

TEST_F(VideoSenderTest, RestartAfterStop) {
    MockBackend* mock = nullptr;
    auto sender = MakeSender(&mock);
    ASSERT_TRUE(sender->Start());
    sender->Stop();

    EXPECT_TRUE(sender->Start());
    EXPECT_TRUE(sender->IsRunning());
    PublishFrame();
    EXPECT_TRUE(WaitFor([mock] { return mock->encode_count == 1; }));
    sender->Stop();
}

TEST_F(VideoSenderTest, StartFailsWhenBackendFails) {
    MockBackend* mock = nullptr;
    auto sender = MakeSender(&mock);
    mock->start_result = false;
    EXPECT_FALSE(sender->Start());
    EXPECT_GT(sender->ErrorCount(), 0u);
    EXPECT_FALSE(sender->IsRunning());
}

}  // namespace
}  // namespace drone::video_transmission
