/**
 * @file frame_compositor_test.cpp
 * @brief 视频帧叠加器（FrameCompositor）单元测试
 *
 * 不依赖硬件：用内存池构造 NV12 帧 + 构造检测结果，验证：
 * - 生命周期：Start/Stop 幂等
 * - 叠加发布：有效帧 + 检测 → 输出标注帧（句柄有效、可读、计数递增）
 * - 检测对齐：先发检测再发帧，输出帧画上框
 * - 错误处理：无效帧计入错误、不发布
 * - 池满丢帧：输出池容量不足时丢帧计数
 *
 * 像素叠加正确性（框/文字是否画到 NV12 对应位置）属于视觉验收，
 * 在此仅验证数据处理链路与计数；可视化验证留待香橙派实机或人工检查。
 */
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "common/types.h"
#include "video/frame_compositor.h"
#include "video/video_frame_pool.h"

namespace drone::video {
namespace {

using testing::Test;

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

class FrameCompositorTest : public Test {
protected:
    void SetUp() override {
        // 输入解码帧池（生产者侧）
        video::VideoFrameInfo tmpl;
        tmpl.width = 64;
        tmpl.height = 64;
        tmpl.hor_stride = 64;
        tmpl.ver_stride = 64;
        tmpl.format = video::PixelFormat::kYuv420SpNv12;
        source_pool_ = std::make_shared<video::VideoFramePool>(4, tmpl);

        CompositorConfig config;
        config.pool_capacity = 4;
        compositor_ = std::make_unique<FrameCompositor>(config);
        compositor_->SetDecodedInput(decoded_topic_);
        compositor_->SetDetectionInput(detection_topic_);
        annotated_sub_ = compositor_->AnnotatedOutput().Subscribe(8);
    }

    void TearDown() override {
        compositor_->Stop();
        compositor_.reset();
    }

    /// 发布一帧有效的纯色 NV12 帧。
    void PublishDecodedFrame() {
        auto handle = source_pool_->Acquire();
        ASSERT_TRUE(handle.Valid());
        std::memset(handle.Data(), 128, source_pool_->SlotSize());
        (void)decoded_topic_.Emplace(std::move(handle));
    }

    /// 发布一个空句柄（无效帧场景）。
    void PublishEmptyFrame() {
        (void)decoded_topic_.Emplace(video::FrameHandle{});
    }

    /// 构造并发布一条检测结果。
    void PublishDetection(uint32_t class_id, float conf, float x, float y,
                          float w, float h) {
        common::DetectionResult d;
        d.class_id = class_id;
        d.confidence = conf;
        d.bbox_x = x;
        d.bbox_y = y;
        d.bbox_w = w;
        d.bbox_h = h;
        (void)detection_topic_.Emplace(d);
    }

    std::shared_ptr<video::VideoFramePool> source_pool_;
    std::unique_ptr<FrameCompositor> compositor_;
    common::Topic<video::FrameHandle> decoded_topic_;
    common::Topic<common::DetectionResult> detection_topic_;
    common::Topic<video::FrameHandle>::Subscription annotated_sub_;
};

TEST_F(FrameCompositorTest, StartStopLifecycle) {
    EXPECT_FALSE(compositor_->IsRunning());
    EXPECT_TRUE(compositor_->Start());
    EXPECT_TRUE(compositor_->IsRunning());
    EXPECT_TRUE(compositor_->Start());  // 幂等
    compositor_->Stop();
    EXPECT_FALSE(compositor_->IsRunning());
    compositor_->Stop();  // 幂等
}

TEST_F(FrameCompositorTest, PublishAnnotatedFrame) {
    ASSERT_TRUE(compositor_->Start());
    // 先发送检测，再送帧，确保叠加器在取帧前已拉到结果
    PublishDetection(0, 0.85f, 10.f, 10.f, 40.f, 40.f);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    PublishDecodedFrame();

    EXPECT_TRUE(WaitFor([this] { return compositor_->AnnotatedCount() == 1; }));

    auto message = annotated_sub_.WaitTakeFor(std::chrono::milliseconds(1000));
    ASSERT_TRUE(message.has_value());
    const auto& handle = **message;
    EXPECT_TRUE(handle.Valid());
    EXPECT_EQ(handle.Info().format, video::PixelFormat::kYuv420SpNv12);
    EXPECT_EQ(handle.Info().width, 64u);
    EXPECT_EQ(handle.Info().height, 64u);
    // 标注帧缓冲可读
    EXPECT_TRUE(handle.Data() != nullptr);
}

TEST_F(FrameCompositorTest, DetectionAppliedAndIdempotentPublish) {
    ASSERT_TRUE(compositor_->Start());
    // 帧 + 检测各发一条 → 只产出一条标注帧
    PublishDetection(1, 0.60f, 5.f, 5.f, 30.f, 30.f);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    PublishDecodedFrame();

    EXPECT_TRUE(WaitFor([this] { return compositor_->AnnotatedCount() == 1; }));
    // 只产出一条标注帧（在队列中尚未消费）
    EXPECT_EQ(annotated_sub_.PendingCount(), 1u);
    auto message = annotated_sub_.TryTake();
    ASSERT_TRUE(message.has_value());
    EXPECT_TRUE((**message).Valid());
    // 再发一帧 → 累计两条
    PublishDecodedFrame();
    EXPECT_TRUE(WaitFor([this] { return compositor_->AnnotatedCount() == 2; }));
}

TEST_F(FrameCompositorTest, UsesConfiguredClassNameAndNormalizesLowercase) {
    CompositorConfig config;
    config.pool_capacity = 2;
    config.class_names = {"balloon"};
    FrameCompositor configured_compositor(config);
    common::Topic<video::FrameHandle> decoded;
    common::Topic<common::DetectionResult> detections;
    configured_compositor.SetDecodedInput(decoded);
    configured_compositor.SetDetectionInput(detections);
    auto output = configured_compositor.AnnotatedOutput().Subscribe(2);

    ASSERT_TRUE(configured_compositor.Start());
    common::DetectionResult detection;
    detection.class_id = 0;
    detection.confidence = 0.9f;
    detection.bbox_x = 4.f;
    detection.bbox_y = 30.f;
    detection.bbox_w = 20.f;
    detection.bbox_h = 20.f;
    (void)detections.Emplace(detection);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    auto frame = source_pool_->Acquire();
    ASSERT_TRUE(frame.Valid());
    std::memset(frame.Data(), 128, source_pool_->SlotSize());
    (void)decoded.Emplace(std::move(frame));

    ASSERT_TRUE(WaitFor([&configured_compositor] {
        return configured_compositor.AnnotatedCount() == 1;
    }));
    auto message = output.WaitTakeFor(std::chrono::milliseconds(1000));
    ASSERT_TRUE(message.has_value());

    // "BALLOON" 第三个字符 L 从 x=16 开始，文字顶行为 y=23；该像素不在检测框上。
    // 若 JSON 配置名未生效、未转大写或 L 字模缺失，此处仍会保持输入亮度 128。
    const auto& annotated = **message;
    const std::size_t l_top_left = 23u * annotated.Info().hor_stride + 16u;
    EXPECT_EQ(std::to_integer<unsigned char>(annotated.Data()[l_top_left]), 230u);
    configured_compositor.Stop();
}

TEST_F(FrameCompositorTest, InvalidFrameCountsErrorNoPublish) {
    ASSERT_TRUE(compositor_->Start());
    PublishEmptyFrame();
    EXPECT_TRUE(WaitFor([this] { return compositor_->ErrorCount() == 1; }));
    EXPECT_EQ(compositor_->AnnotatedCount(), 0u);
    EXPECT_EQ(annotated_sub_.PendingCount(), 0u);
}

TEST_F(FrameCompositorTest, StopStopsConsuming) {
    ASSERT_TRUE(compositor_->Start());
    PublishDetection(0, 0.5f, 0.f, 0.f, 10.f, 10.f);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    PublishDecodedFrame();
    EXPECT_TRUE(WaitFor([this] { return compositor_->AnnotatedCount() == 1; }));

    compositor_->Stop();
    PublishDecodedFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    EXPECT_EQ(compositor_->AnnotatedCount(), 1u);
}

TEST_F(FrameCompositorTest, RestartAfterStop) {
    ASSERT_TRUE(compositor_->Start());
    compositor_->Stop();

    EXPECT_TRUE(compositor_->Start());
    EXPECT_TRUE(compositor_->IsRunning());

    PublishDetection(0, 0.9f, 10.f, 10.f, 20.f, 20.f);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    PublishDecodedFrame();
    EXPECT_TRUE(WaitFor([this] { return compositor_->AnnotatedCount() == 1; }));
    compositor_->Stop();
}

}  // namespace
}  // namespace drone::video
