/**
 * @file stereo_frame_splitter_test.cpp
 * @brief 双目 SBS 帧拆分器（StereoFrameSplitter）单元测试
 *
 * 不依赖硬件：用内存池构造左右半幅不同图案的全幅 NV12 假帧，验证：
 * - 生命周期：Start/Stop 幂等
 * - 拆分正确性：左右输出各为单目尺寸，内容与对应半幅逐字节一致
 *   （prefer_rga=false/true 各一遍；x86 无 RGA 时 true 自动走 memcpy）
 * - 帧元数据：pipeline_ingress_time_ms 透传、timestamp_ms 为拆分完成时刻
 * - 错误处理：非法输入（空句柄/尺寸不符）计入错误、不发布
 * - 池满丢帧：输出池容量不足时按左右分别计数
 * - 配置校验：奇数宽/0 尺寸/0 池容量构造抛出
 */
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "common/topic.h"
#include "video/stereo_frame_splitter.h"
#include "video/video_frame_pool.h"

namespace drone::video {
namespace {

using testing::Test;

constexpr std::uint32_t kFullWidth = 64;    // 全幅 SBS 宽（左右各 32）
constexpr std::uint32_t kFullHeight = 64;
constexpr std::uint8_t kLeftY = 0x10;
constexpr std::uint8_t kRightY = 0x80;
constexpr std::uint8_t kLeftUv = 0x40;
constexpr std::uint8_t kRightUv = 0xC0;

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

class StereoFrameSplitterTest : public Test {
protected:
    void SetUp() override {
        video::VideoFrameInfo tmpl;
        tmpl.width = kFullWidth;
        tmpl.height = kFullHeight;
        tmpl.hor_stride = kFullWidth;
        tmpl.ver_stride = kFullHeight;
        tmpl.format = video::PixelFormat::kYuv420SpNv12;
        source_pool_ = std::make_shared<video::VideoFramePool>(8, tmpl);
    }

    void TearDown() override {
        if (splitter_ != nullptr) {
            splitter_->Stop();
            splitter_.reset();
        }
    }

    /// 创建并接线拆分器（每个用例按需选择 RGA 开关）。
    void BuildSplitter(bool prefer_rga, std::size_t pool_capacity = 4) {
        StereoFrameSplitterConfig config;
        config.width = kFullWidth;
        config.height = kFullHeight;
        config.pool_capacity = pool_capacity;
        config.prefer_rga = prefer_rga;
        splitter_ = std::make_unique<StereoFrameSplitter>(config);
        splitter_->SetInput(decoded_topic_);
        left_sub_ = splitter_->LeftOutput().Subscribe(8);
        right_sub_ = splitter_->RightOutput().Subscribe(8);
    }

    /// 发布一帧左半 kLeftY/kLeftUv、右半 kRightY/kRightUv 的全幅 NV12 帧。
    /// 注意：实证 SBS 左半幅=物理右目，拆分器已交换 x_offset——
    /// 左目输出承载右半幅内容（kRight*），右目输出承载左半幅内容（kLeft*）。
    void PublishSplitPatternFrame(std::int64_t ingress_ms = 0) {
        auto handle = source_pool_->Acquire(ingress_ms);
        ASSERT_TRUE(handle.Valid());
        const auto& info = handle.Info();
        std::uint8_t* base = reinterpret_cast<std::uint8_t*>(handle.Data());
        // Y 平面：左半/右半按行分别填充
        for (std::uint32_t row = 0; row < info.height; ++row) {
            std::uint8_t* line = base + static_cast<std::size_t>(row) * info.hor_stride;
            std::memset(line, kLeftY, info.width / 2);
            std::memset(line + info.width / 2, kRightY, info.width / 2);
        }
        // UV 半平面：布局同 Y（每行左半/右半各 width/2 字节）
        std::uint8_t* uv_base =
            base + static_cast<std::size_t>(info.hor_stride) * info.ver_stride;
        for (std::uint32_t row = 0; row < info.height / 2; ++row) {
            std::uint8_t* line =
                uv_base + static_cast<std::size_t>(row) * info.hor_stride;
            std::memset(line, kLeftUv, info.width / 2);
            std::memset(line + info.width / 2, kRightUv, info.width / 2);
        }
        (void)decoded_topic_.Emplace(std::move(handle));
    }

    /// 校验一目输出：尺寸为单目、NV12、整面为期望值、ingress 透传。
    static void ExpectEyeContent(const FrameHandle& handle, std::uint8_t expect_y,
                                 std::uint8_t expect_uv, std::int64_t ingress_ms) {
        ASSERT_TRUE(handle.Valid());
        const auto& info = handle.Info();
        EXPECT_TRUE(info.Valid());
        EXPECT_EQ(info.width, kFullWidth / 2);
        EXPECT_EQ(info.height, kFullHeight);
        EXPECT_EQ(info.format, video::PixelFormat::kYuv420SpNv12);
        EXPECT_EQ(info.pipeline_ingress_time_ms, ingress_ms);
        EXPECT_GT(info.timestamp_ms, 0);  // 拆分完成时刻（单调时钟）
        ASSERT_NE(handle.Data(), nullptr);
        const std::uint8_t* base =
            reinterpret_cast<const std::uint8_t*>(handle.Data());
        for (std::uint32_t row = 0; row < info.height; ++row) {
            for (std::uint32_t col = 0; col < info.width; ++col) {
                EXPECT_EQ(base[static_cast<std::size_t>(row) * info.hor_stride + col],
                          expect_y)
                    << "Y 平面第 " << row << " 行第 " << col << " 列";
            }
        }
        const std::uint8_t* uv_base =
            base + static_cast<std::size_t>(info.hor_stride) * info.ver_stride;
        for (std::uint32_t row = 0; row < info.height / 2; ++row) {
            for (std::uint32_t col = 0; col < info.width; ++col) {
                EXPECT_EQ(uv_base[static_cast<std::size_t>(row) * info.hor_stride + col],
                          expect_uv)
                    << "UV 平面第 " << row << " 行第 " << col << " 列";
            }
        }
    }

    std::shared_ptr<video::VideoFramePool> source_pool_;
    std::unique_ptr<StereoFrameSplitter> splitter_;
    common::Topic<video::FrameHandle> decoded_topic_;
    common::Topic<video::FrameHandle>::Subscription left_sub_;
    common::Topic<video::FrameHandle>::Subscription right_sub_;
};

TEST_F(StereoFrameSplitterTest, StartStopLifecycle) {
    BuildSplitter(false);
    EXPECT_FALSE(splitter_->IsRunning());
    EXPECT_TRUE(splitter_->Start());
    EXPECT_TRUE(splitter_->IsRunning());
    EXPECT_TRUE(splitter_->Start());  // 幂等
    splitter_->Stop();
    EXPECT_FALSE(splitter_->IsRunning());
    splitter_->Stop();  // 幂等
}

TEST_F(StereoFrameSplitterTest, SplitsLeftAndRightEyesByMemcpy) {
    BuildSplitter(false);  // 强制 memcpy 路径
    ASSERT_TRUE(splitter_->Start());
    constexpr std::int64_t kIngress = 12345;
    PublishSplitPatternFrame(kIngress);

    ASSERT_TRUE(WaitFor([this] {
        return splitter_->LeftFrameCount() >= 1 && splitter_->RightFrameCount() >= 1;
    }));
    auto left = left_sub_.TryTake();
    auto right = right_sub_.TryTake();
    ASSERT_TRUE(left.has_value());
    ASSERT_TRUE(right.has_value());
    ExpectEyeContent(**left, kRightY, kRightUv, kIngress);
    ExpectEyeContent(**right, kLeftY, kLeftUv, kIngress);
    // 序号由左右各自内存池独立分配：首帧均为各自池的第 0 号
    EXPECT_EQ((*left)->Info().sequence, 0u);
    EXPECT_EQ((*right)->Info().sequence, 0u);
    EXPECT_EQ(splitter_->ErrorCount(), 0u);
    EXPECT_EQ(splitter_->LeftDroppedCount(), 0u);
    EXPECT_EQ(splitter_->RightDroppedCount(), 0u);
}

TEST_F(StereoFrameSplitterTest, SplitsWithPreferRgaEnabled) {
    BuildSplitter(true);  // x86 无 RGA 时自动走 memcpy；香橙派上走 RGA
    ASSERT_TRUE(splitter_->Start());
    PublishSplitPatternFrame();

    ASSERT_TRUE(WaitFor([this] {
        return splitter_->LeftFrameCount() >= 1 && splitter_->RightFrameCount() >= 1;
    }));
    auto left = left_sub_.TryTake();
    auto right = right_sub_.TryTake();
    ASSERT_TRUE(left.has_value());
    ASSERT_TRUE(right.has_value());
    ExpectEyeContent(**left, kRightY, kRightUv, 0);
    ExpectEyeContent(**right, kLeftY, kLeftUv, 0);
    EXPECT_EQ(splitter_->ErrorCount(), 0u);
}

TEST_F(StereoFrameSplitterTest, CountsInvalidInputWithoutPublishing) {
    BuildSplitter(false);
    ASSERT_TRUE(splitter_->Start());
    (void)decoded_topic_.Emplace(video::FrameHandle{});  // 空句柄
    ASSERT_TRUE(WaitFor([this] { return splitter_->ErrorCount() >= 1; }));
    EXPECT_EQ(splitter_->LeftFrameCount(), 0u);
    EXPECT_EQ(splitter_->RightFrameCount(), 0u);
}

TEST_F(StereoFrameSplitterTest, DropsWhenOutputPoolExhausted) {
    BuildSplitter(false, /*pool_capacity=*/2);
    ASSERT_TRUE(splitter_->Start());
    // 输出订阅不消费：左右池各 2 槽在途占满后，后续帧只能丢
    for (int index = 0; index < 6; ++index) {
        PublishSplitPatternFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(WaitFor([this] {
        return splitter_->LeftFrameCount() + splitter_->LeftDroppedCount() >= 6 &&
               splitter_->RightFrameCount() + splitter_->RightDroppedCount() >= 6;
    }));
    EXPECT_EQ(splitter_->LeftFrameCount(), 2u);
    EXPECT_EQ(splitter_->RightFrameCount(), 2u);
    EXPECT_EQ(splitter_->LeftDroppedCount(), 4u);
    EXPECT_EQ(splitter_->RightDroppedCount(), 4u);
}

TEST(StereoFrameSplitterConfigTest, RejectsInvalidConfig) {
    StereoFrameSplitterConfig config;
    config.width = 63;  // 奇数宽无法均分左右
    EXPECT_THROW(StereoFrameSplitter splitter(config), std::invalid_argument);
    config.width = 0;
    EXPECT_THROW(StereoFrameSplitter splitter(config), std::invalid_argument);
    config.width = 64;
    config.height = 0;
    EXPECT_THROW(StereoFrameSplitter splitter(config), std::invalid_argument);
    config.height = 64;
    config.pool_capacity = 0;
    EXPECT_THROW(StereoFrameSplitter splitter(config), std::invalid_argument);
    config.pool_capacity = 4;
    config.input_queue_capacity = 0;
    EXPECT_THROW(StereoFrameSplitter splitter(config), std::invalid_argument);
}

}  // namespace
}  // namespace drone::video
