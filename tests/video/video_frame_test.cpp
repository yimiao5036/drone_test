#include "video/video_frame.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

namespace drone::video {
namespace {

using drone::video::BytesPerPixel;
using drone::video::FrameBuffer;
using drone::video::FrameHandle;
using drone::video::FrameRecycler;
using drone::video::PixelFormat;
using drone::video::VideoFrameInfo;

/// 测试替身：记录归还的槽位编号。
class FakeRecycler : public FrameRecycler {
public:
    void Recycle(std::uint32_t slot_index) noexcept override {
        recycled_slots.push_back(slot_index);
    }

    std::vector<std::uint32_t> recycled_slots;
};

/// 构造一份可用的帧元数据。
VideoFrameInfo MakeInfo(std::uint32_t width = 1920,
                        std::uint32_t height = 1080) {
    VideoFrameInfo info;
    info.width = width;
    info.height = height;
    info.hor_stride = width;   // 假设无对齐
    info.ver_stride = height;
    info.buf_size = width * height * 3 / 2;  // NV12
    info.format = PixelFormat::kYuv420SpNv12;
    info.sequence = 42;
    info.timestamp_ms = 12345;
    return info;
}

TEST(VideoFrameTest, BytesPerPixelForKnownFormats) {
    EXPECT_EQ(BytesPerPixel(PixelFormat::kYuv420SpNv12), 1U);
    EXPECT_EQ(BytesPerPixel(PixelFormat::kRgb888), 3U);
    EXPECT_EQ(BytesPerPixel(PixelFormat::kUnknown), 0U);
}

TEST(VideoFrameTest, LineSizeBytesComputesWithStride) {
    VideoFrameInfo info = MakeInfo();
    info.hor_stride = 1920 + 64;  // 行对齐填充
    EXPECT_EQ(info.LineSizeBytes(), 1920U + 64U);

    info.format = PixelFormat::kRgb888;
    EXPECT_EQ(info.LineSizeBytes(), (1920U + 64U) * 3U);
}

TEST(VideoFrameTest, DefaultHandleIsInvalidAndSafeToRead) {
    FrameHandle handle;
    EXPECT_FALSE(handle.Valid());
    EXPECT_FALSE(handle);
    EXPECT_EQ(handle.Data(), nullptr);
    EXPECT_EQ(handle.Capacity(), 0U);
    // 空句柄读取元数据不崩溃，且元数据无效
    EXPECT_FALSE(handle.Info().Valid());
    EXPECT_EQ(handle.Info().width, 0U);
}

TEST(VideoFrameTest, HandleExposesInfoAndData) {
    std::byte buffer[1024];
    auto recycler = std::make_shared<FakeRecycler>();
    VideoFrameInfo info = MakeInfo();

    auto frame_buffer = std::make_shared<FrameBuffer>(
        info, buffer, sizeof(buffer), recycler, 7);
    FrameHandle handle(frame_buffer);

    ASSERT_TRUE(handle.Valid());
    EXPECT_EQ(handle.Info().width, 1920U);
    EXPECT_EQ(handle.Info().height, 1080U);
    EXPECT_EQ(handle.Info().sequence, 42U);
    EXPECT_EQ(handle.Info().timestamp_ms, 12345);
    EXPECT_EQ(handle.Data(), static_cast<std::byte*>(buffer));
    EXPECT_EQ(handle.Capacity(), sizeof(buffer));
    EXPECT_EQ(handle.Info().format, PixelFormat::kYuv420SpNv12);
}

TEST(VideoFrameTest, HandleIsMovableButNotCopyable) {
    static_assert(!std::is_copy_constructible_v<FrameHandle>,
                  "FrameHandle must not be copyable");
    static_assert(!std::is_copy_assignable_v<FrameHandle>,
                  "FrameHandle must not be copy assignable");
    static_assert(std::is_move_constructible_v<FrameHandle>,
                  "FrameHandle must be movable");
    static_assert(std::is_move_assignable_v<FrameHandle>,
                  "FrameHandle must be move assignable");

    std::byte buffer[1024];
    auto recycler = std::make_shared<FakeRecycler>();
    auto frame_buffer = std::make_shared<FrameBuffer>(
        MakeInfo(), buffer, sizeof(buffer), recycler, 0);

    FrameHandle source(frame_buffer);
    FrameHandle target(std::move(source));

    // 移动后：目标有效，源为空（保证"同一帧只发布一次"）
    ASSERT_TRUE(target.Valid());
    EXPECT_FALSE(source.Valid());
    EXPECT_EQ(target.Data(), static_cast<std::byte*>(buffer));
}

TEST(VideoFrameTest, BufferReturnsToRecyclerWhenLastReferenceDies) {
    std::byte buffer[1024];
    auto recycler = std::make_shared<FakeRecycler>();

    {
        auto frame_buffer = std::make_shared<FrameBuffer>(
            MakeInfo(), buffer, sizeof(buffer), recycler, 5);
        EXPECT_TRUE(recycler->recycled_slots.empty());

        // 模拟 Topic 分发：多个 shared_ptr<const FrameHandle> 共享同一帧
        auto message = std::make_shared<const FrameHandle>(
            FrameHandle(frame_buffer));
        auto second = message;  // 第二个"订阅者"持同一消息
        frame_buffer.reset();   // 池侧不再持有

        EXPECT_TRUE(recycler->recycled_slots.empty());  // 还有引用在途

        second.reset();
        EXPECT_TRUE(recycler->recycled_slots.empty());  // 仍有一个引用

        message.reset();  // 最后一个引用释放
    }

    // 最后一个消费者释放时归还，且只归还一次
    ASSERT_EQ(recycler->recycled_slots.size(), 1U);
    EXPECT_EQ(recycler->recycled_slots.front(), 5U);
}

TEST(VideoFrameTest, ResetRecyclesImmediatelyAndIsIdempotent) {
    std::byte buffer[1024];
    auto recycler = std::make_shared<FakeRecycler>();

    auto frame_buffer = std::make_shared<FrameBuffer>(
        MakeInfo(), buffer, sizeof(buffer), recycler, 9);
    FrameHandle handle(frame_buffer);
    frame_buffer.reset();

    handle.Reset();  // 显式归还
    EXPECT_EQ(recycler->recycled_slots.size(), 1U);
    EXPECT_FALSE(handle.Valid());

    handle.Reset();  // 幂等
    EXPECT_EQ(recycler->recycled_slots.size(), 1U);
}

TEST(VideoFrameTest, BufferWithoutRecyclerSkipsReturn) {
    std::byte buffer[1024];
    // recycler 为空：析构不触发任何归还（外部缓冲 / 测试场景）
    {
        auto frame_buffer = std::make_shared<FrameBuffer>(
            MakeInfo(), buffer, sizeof(buffer), nullptr, 0);
        FrameHandle handle(frame_buffer);
        ASSERT_TRUE(handle.Valid());
    }
    // 无崩溃即通过
}

TEST(VideoFrameTest, InfoValidityReflectsFields) {
    VideoFrameInfo info = MakeInfo();
    EXPECT_TRUE(info.Valid());

    info.format = PixelFormat::kUnknown;
    EXPECT_FALSE(info.Valid());

    info = MakeInfo();
    info.width = 0;
    EXPECT_FALSE(info.Valid());
}

}  // namespace
}  // namespace drone::video
