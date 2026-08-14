/**
 * @file yolo_detector_test.cpp
 * @brief YOLO 检测器（YoloDetector）单元测试
 *
 * 通过注入 Mock 后端验证不依赖硬件的线程/发布/统计逻辑：
 * - 生命周期：Start/Stop 幂等、无后端启动失败
 * - 发布：检测结果字段（类别/置信度/框/中心/帧序号/推理耗时）、
 *   一帧多目标、无目标不发布、多帧帧序号关联
 * - 错误处理：后端抛异常计入 ErrorCount 且链路继续、无效帧跳过
 * - 停机：Stop 后不再消费新帧
 *
 * RKNN 真实推理（RGA + NPU）依赖香橙派硬件，在香橙派实机验证
 * （配合 rknn_detection_backend，DRONE_HAVE_RKNN 编译）。
 */
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "perception/detection_backend.h"
#include "perception/yolo_detector.h"
#include "video/video_frame_pool.h"

namespace drone::perception {
namespace {

using testing::Test;

/// Mock 推理后端：记录调用，返回可配置检测结果。
class MockBackend final : public IDetectionBackend {
public:
    bool load_result = true;               ///< Load() 返回
    std::vector<BackendDetection> detections;  ///< Detect() 返回
    bool throw_on_detect = false;          ///< Detect() 抛异常（模拟后端故障）
    std::uint64_t load_count = 0;
    std::uint64_t detect_count = 0;
    bool loaded = false;

    bool Load() override {
        ++load_count;
        loaded = load_result;
        return load_result;
    }
    void Unload() noexcept override { loaded = false; }
    bool IsLoaded() const override { return loaded; }
    std::vector<BackendDetection> Detect(const video::FrameHandle&) override {
        ++detect_count;
        if (throw_on_detect) {
            throw std::runtime_error("mock detect failure");
        }
        return detections;
    }
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

class YoloDetectorTest : public Test {
protected:
    void SetUp() override {
        video::VideoFrameInfo frame_template;
        frame_template.width = 64;
        frame_template.height = 64;
        frame_template.hor_stride = 64;
        frame_template.ver_stride = 64;
        frame_template.format = video::PixelFormat::kYuv420SpNv12;
        pool_ = std::make_shared<video::VideoFramePool>(4, frame_template);

        // YoloDetector 独占 Mock 后端所有权；裸指针仅用于测试读取状态
        mock_ = new MockBackend();
        // YoloDetector 独占 Mock 后端所有权；裸指针仅用于测试读取状态
        std::unique_ptr<IDetectionBackend> backend(mock_);
        YoloDetectorConfig config;
        config.input_queue_capacity = 2;
        detector_ = std::make_unique<YoloDetector>(config, std::move(backend));
        detector_->SetInput(frame_topic_);
        result_sub_ = detector_->DetectionOutput().Subscribe(8);
    }

    void TearDown() override {
        detector_->Stop();
        detector_.reset();  // 释放 YoloDetector（连带 Mock 后端）
    }

    /// 发布一帧有效 NV12 帧（数据全 0，内容不影响 Mock 后端）。
    void PublishFrame() {
        auto handle = pool_->Acquire();
        ASSERT_TRUE(handle.Valid());
        std::memset(handle.Data(), 0, pool_->SlotSize());
        (void)frame_topic_.Emplace(std::move(handle));
    }

    /// 发布一个空句柄（无效帧场景）。
    void PublishEmptyFrame() {
        (void)frame_topic_.Emplace(video::FrameHandle{});
    }

    /// 阻塞等待一条检测结果。
    bool WaitResult(common::DetectionResult* out, int timeout_ms = 3000) {
        auto message = result_sub_.WaitTakeFor(std::chrono::milliseconds(timeout_ms));
        if (message) {
            *out = **message;
            return true;
        }
        return false;
    }

    std::shared_ptr<video::VideoFramePool> pool_;
    MockBackend* mock_ = nullptr;  // 所有权归 YoloDetector，裸指针仅测试读取
    std::unique_ptr<YoloDetector> detector_;
    common::Topic<video::FrameHandle> frame_topic_;
    common::Topic<common::DetectionResult>::Subscription result_sub_;
};

TEST_F(YoloDetectorTest, StartWithoutBackendFails) {
    // 无后端（开发机未编译 RKNN 且未注入）→ Start 失败并计入错误
    YoloDetectorConfig config;
    YoloDetector detector(config);
    EXPECT_FALSE(detector.Start());
    EXPECT_GT(detector.ErrorCount(), 0u);
    EXPECT_FALSE(detector.IsRunning());
    EXPECT_EQ(detector.ProcessedFrameCount(), 0u);
}

TEST_F(YoloDetectorTest, StartStopLifecycle) {
    EXPECT_FALSE(detector_->IsRunning());
    EXPECT_TRUE(detector_->Start());
    EXPECT_TRUE(detector_->IsRunning());
    EXPECT_EQ(mock_->load_count, 1u);

    // 已启动再次 Start 幂等
    EXPECT_TRUE(detector_->Start());
    EXPECT_EQ(mock_->load_count, 1u);

    detector_->Stop();
    EXPECT_FALSE(detector_->IsRunning());
    detector_->Stop();  // 幂等
    EXPECT_EQ(mock_->detect_count, 0u);
}

TEST_F(YoloDetectorTest, StartWithLoadFailureFails) {
    mock_->load_result = false;
    EXPECT_FALSE(detector_->Start());
    EXPECT_GT(detector_->ErrorCount(), 0u);
    EXPECT_FALSE(detector_->IsRunning());
    EXPECT_EQ(mock_->load_count, 1u);

    // 加载失败后不重复尝试（load_failed 防刷屏）
    EXPECT_FALSE(detector_->Start());
    EXPECT_EQ(mock_->load_count, 1u);
}

TEST_F(YoloDetectorTest, DetectPublishesResults) {
    mock_->detections = {
        {0, 0.9f, 10.f, 20.f, 110.f, 120.f},
        {1, 0.7f, 200.f, 200.f, 300.f, 300.f},
    };
    ASSERT_TRUE(detector_->Start());

    PublishFrame();

    common::DetectionResult r1;
    common::DetectionResult r2;
    ASSERT_TRUE(WaitResult(&r1));
    ASSERT_TRUE(WaitResult(&r2));

    // 第一个目标字段完整
    EXPECT_EQ(r1.class_id, 0u);
    EXPECT_NEAR(r1.confidence, 0.9f, 1e-6f);
    EXPECT_NEAR(r1.bbox_x, 10.f, 1e-6f);
    EXPECT_NEAR(r1.bbox_y, 20.f, 1e-6f);
    EXPECT_NEAR(r1.bbox_w, 100.f, 1e-6f);
    EXPECT_NEAR(r1.bbox_h, 100.f, 1e-6f);
    EXPECT_NEAR(r1.center_pixel_x, 60.f, 1e-6f);
    EXPECT_NEAR(r1.center_pixel_y, 70.f, 1e-6f);
    EXPECT_EQ(r1.frame_sequence, 0u);  // 第一帧
    EXPECT_GT(r1.inference_time_ms, 0.f);
    EXPECT_GT(r1.header.sequence, 0u);
    EXPECT_GT(r1.header.source_time_ms, 0u);

    // 第二个目标：同一帧，序号递增
    EXPECT_EQ(r2.class_id, 1u);
    EXPECT_NEAR(r2.confidence, 0.7f, 1e-6f);
    EXPECT_EQ(r2.frame_sequence, r1.frame_sequence);
    EXPECT_GT(r2.header.sequence, r1.header.sequence);

    // 统计
    EXPECT_TRUE(WaitFor([this] { return detector_->ProcessedFrameCount() == 1; }));
    EXPECT_GT(detector_->InferenceTimeMsAvg(), 0.f);
    EXPECT_EQ(mock_->detect_count, 1u);
}

TEST_F(YoloDetectorTest, NoDetectionsNoPublish) {
    mock_->detections = {};
    ASSERT_TRUE(detector_->Start());

    PublishFrame();

    EXPECT_TRUE(WaitFor([this] { return detector_->ProcessedFrameCount() == 1; }));
    EXPECT_EQ(result_sub_.PendingCount(), 0u);
    EXPECT_TRUE(result_sub_.TryTake() == std::nullopt);
}

TEST_F(YoloDetectorTest, BackendThrowCountsErrorAndContinues) {
    ASSERT_TRUE(detector_->Start());

    // 后端故障：抛异常 → 计入错误，帧不计入处理
    mock_->throw_on_detect = true;
    PublishFrame();
    EXPECT_TRUE(WaitFor([this] { return detector_->ErrorCount() == 1; }));
    EXPECT_EQ(detector_->ProcessedFrameCount(), 0u);

    // 恢复后链路继续
    mock_->throw_on_detect = false;
    mock_->detections = {{0, 0.5f, 0.f, 0.f, 10.f, 10.f}};
    PublishFrame();
    common::DetectionResult r;
    ASSERT_TRUE(WaitResult(&r));
    EXPECT_EQ(detector_->ProcessedFrameCount(), 1u);
    EXPECT_EQ(detector_->ErrorCount(), 1u);
}

TEST_F(YoloDetectorTest, InvalidFrameSkipped) {
    ASSERT_TRUE(detector_->Start());

    PublishEmptyFrame();

    EXPECT_TRUE(WaitFor([this] { return detector_->ErrorCount() == 1; }));
    EXPECT_EQ(detector_->ProcessedFrameCount(), 0u);
    EXPECT_EQ(result_sub_.PendingCount(), 0u);
}

TEST_F(YoloDetectorTest, MultipleFramesSequencing) {
    mock_->detections = {{0, 0.5f, 0.f, 0.f, 10.f, 10.f}};
    ASSERT_TRUE(detector_->Start());

    PublishFrame();
    PublishFrame();

    common::DetectionResult r1;
    common::DetectionResult r2;
    ASSERT_TRUE(WaitResult(&r1));
    ASSERT_TRUE(WaitResult(&r2));
    EXPECT_EQ(r1.frame_sequence, 0u);
    EXPECT_EQ(r2.frame_sequence, 1u);
    EXPECT_TRUE(WaitFor([this] { return detector_->ProcessedFrameCount() == 2; }));
    EXPECT_EQ(mock_->detect_count, 2u);
}

TEST_F(YoloDetectorTest, StopStopsConsuming) {
    mock_->detections = {{0, 0.5f, 0.f, 0.f, 10.f, 10.f}};
    ASSERT_TRUE(detector_->Start());

    PublishFrame();
    EXPECT_TRUE(WaitFor([this] { return detector_->ProcessedFrameCount() == 1; }));

    detector_->Stop();
    PublishFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    EXPECT_FALSE(detector_->IsRunning());
    EXPECT_EQ(detector_->ProcessedFrameCount(), 1u);
    EXPECT_EQ(mock_->detect_count, 1u);
}

TEST_F(YoloDetectorTest, RestartAfterStop) {
    mock_->detections = {{0, 0.5f, 0.f, 0.f, 10.f, 10.f}};
    ASSERT_TRUE(detector_->Start());
    detector_->Stop();

    // 停止后重新启动：重新加载后端并恢复消费
    EXPECT_TRUE(detector_->Start());
    EXPECT_TRUE(detector_->IsRunning());
    EXPECT_EQ(mock_->load_count, 2u);

    PublishFrame();
    common::DetectionResult r;
    ASSERT_TRUE(WaitResult(&r));
    EXPECT_EQ(detector_->ProcessedFrameCount(), 1u);
    detector_->Stop();
}

}  // namespace
}  // namespace drone::perception
