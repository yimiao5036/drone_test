#include "video/video_frame_pool.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace drone::video {
namespace {

using drone::video::FrameHandle;
using drone::video::PixelFormat;
using drone::video::VideoFrameInfo;
using drone::video::VideoFramePool;

/// 构造 NV12 帧模板（1920×1080，无行对齐）。
VideoFrameInfo MakeNv12Info(std::uint32_t width = 1920,
                            std::uint32_t height = 1080) {
    VideoFrameInfo info;
    info.width = width;
    info.height = height;
    info.hor_stride = width;
    info.ver_stride = height;
    info.format = PixelFormat::kYuv420SpNv12;
    return info;
}

TEST(VideoFramePoolTest, ComputeBufferSizePerFormat) {
    VideoFrameInfo nv12 = MakeNv12Info();
    EXPECT_EQ(VideoFramePool::ComputeBufferSize(nv12),
              std::size_t{1920} * 1080 * 3 / 2);

    VideoFrameInfo rgb;
    rgb.width = 640;
    rgb.height = 640;
    rgb.hor_stride = 640;
    rgb.ver_stride = 640;
    rgb.format = PixelFormat::kRgb888;
    EXPECT_EQ(VideoFramePool::ComputeBufferSize(rgb), std::size_t{640} * 640 * 3);

    VideoFrameInfo unknown;
    unknown.width = 10;
    unknown.height = 10;
    unknown.hor_stride = 10;
    unknown.ver_stride = 10;
    unknown.format = PixelFormat::kUnknown;
    EXPECT_EQ(VideoFramePool::ComputeBufferSize(unknown), 0U);

    // 显式给出 buf_size 时直接返回
    nv12.buf_size = 4096;
    EXPECT_EQ(VideoFramePool::ComputeBufferSize(nv12), 4096U);
}

TEST(VideoFramePoolTest, ConstructorValidatesArguments) {
    EXPECT_THROW(VideoFramePool(0, MakeNv12Info()), std::invalid_argument);
    EXPECT_THROW(VideoFramePool(4, MakeNv12Info(), 0), std::invalid_argument);
    EXPECT_THROW(VideoFramePool(4, MakeNv12Info(), 3), std::invalid_argument);  // 非 2 的幂

    VideoFrameInfo bad_info;
    bad_info.format = PixelFormat::kUnknown;
    EXPECT_THROW(VideoFramePool(4, bad_info), std::invalid_argument);

    VideoFrameInfo empty;
    EXPECT_THROW(VideoFramePool(4, empty), std::invalid_argument);
}

TEST(VideoFramePoolTest, AcquireReturnsValidHandleWithMetadata) {
    auto pool = std::make_shared<VideoFramePool>(2, MakeNv12Info());
    EXPECT_EQ(pool->Capacity(), 2U);
    EXPECT_EQ(pool->IdleCount(), 2U);
    EXPECT_EQ(pool->InFlightCount(), 0U);

    auto first = pool->Acquire();
    auto second = pool->Acquire();

    ASSERT_TRUE(first.Valid());
    ASSERT_TRUE(second.Valid());
    EXPECT_EQ(first.Info().width, 1920U);
    EXPECT_EQ(first.Info().height, 1080U);
    EXPECT_EQ(first.Info().format, PixelFormat::kYuv420SpNv12);
    EXPECT_GT(first.Info().timestamp_ms, 0);  // 单调时钟已打点
    EXPECT_EQ(first.Info().sequence, 0U);     // 序列号递增
    EXPECT_EQ(second.Info().sequence, 1U);
    EXPECT_EQ(first.Capacity(), pool->SlotSize());
    EXPECT_EQ(first.Capacity(), 1920U * 1080 * 3 / 2);  // 64 整除，无填充

    EXPECT_EQ(pool->IdleCount(), 0U);
    EXPECT_EQ(pool->InFlightCount(), 2U);
    EXPECT_EQ(pool->AcquiredCount(), 2U);
    EXPECT_EQ(pool->DroppedCount(), 0U);
}

TEST(VideoFramePoolTest, PoolExhaustionDropsFrameWithoutBlocking) {
    auto pool = std::make_shared<VideoFramePool>(2, MakeNv12Info());

    auto first = pool->Acquire();
    auto second = pool->Acquire();
    auto third = pool->Acquire();  // 池空

    ASSERT_TRUE(first.Valid());
    ASSERT_TRUE(second.Valid());
    EXPECT_FALSE(third.Valid());
    EXPECT_EQ(pool->DroppedCount(), 1U);
}

TEST(VideoFramePoolTest, RecycleReturnsSlotForReuse) {
    auto pool = std::make_shared<VideoFramePool>(2, MakeNv12Info());

    auto first = pool->Acquire();
    ASSERT_TRUE(first.Valid());
    const std::uint32_t slot = first.Info().sequence;  // 与槽位无直接关系，仅占位
    (void)slot;

    first.Reset();  // 显式归还
    EXPECT_EQ(pool->RecycledCount(), 1U);
    EXPECT_EQ(pool->InFlightCount(), 0U);

    auto reused = pool->Acquire();  // 归还后可再次获取
    ASSERT_TRUE(reused.Valid());
    EXPECT_EQ(pool->AcquiredCount(), 2U);
    EXPECT_EQ(pool->DroppedCount(), 0U);
    EXPECT_EQ(pool->IdleCount(), 1U);
}

TEST(VideoFramePoolTest, DuplicateAndOutOfRangeRecycleAreIgnored) {
    auto pool = std::make_shared<VideoFramePool>(2, MakeNv12Info());

    // 从未 Acquire 的槽位归还：状态是空闲，判为重复归还
    pool->Recycle(0);
    EXPECT_EQ(pool->DuplicateRecycleCount(), 1U);

    // 越界归还
    pool->Recycle(100);
    EXPECT_EQ(pool->DuplicateRecycleCount(), 2U);

    // 正常归还后再归还一次：第二次判为重复
    auto frame = pool->Acquire();
    ASSERT_TRUE(frame.Valid());
    frame.Reset();
    EXPECT_EQ(pool->RecycledCount(), 1U);
    pool->Recycle(0);  // 槽位 0 已空闲
    EXPECT_EQ(pool->DuplicateRecycleCount(), 3U);
}

TEST(VideoFramePoolTest, BuffersAreAlignedAndNonOverlapping) {
    auto pool = std::make_shared<VideoFramePool>(3, MakeNv12Info());
    const std::size_t alignment = VideoFramePool::kDefaultAlignment;

    std::vector<FrameHandle> frames;
    for (std::size_t index = 0; index < 3; ++index) {
        auto frame = pool->Acquire();
        ASSERT_TRUE(frame.Valid());
        EXPECT_EQ(reinterpret_cast<std::uintptr_t>(frame.Data()) % alignment, 0U);
        frames.push_back(std::move(frame));
    }

    // 三个槽位地址互不相同；槽位按 LIFO 弹出，地址可能递增也可能递减，
    // 用差值绝对值验证间距恒为槽位大小。
    const auto first = reinterpret_cast<std::uintptr_t>(frames[0].Data());
    const auto second = reinterpret_cast<std::uintptr_t>(frames[1].Data());
    const auto third = reinterpret_cast<std::uintptr_t>(frames[2].Data());
    EXPECT_NE(first, second);
    EXPECT_NE(second, third);
    const auto diff12 = (first > second) ? first - second : second - first;
    const auto diff23 = (second > third) ? second - third : third - second;
    EXPECT_EQ(diff12, pool->SlotSize());
    EXPECT_EQ(diff23, pool->SlotSize());
}

TEST(VideoFramePoolTest, PoolStaysAliveWhileFramesInFlight) {
    std::weak_ptr<VideoFramePool> weak_pool;
    FrameHandle in_flight;
    {
        auto pool = std::make_shared<VideoFramePool>(2, MakeNv12Info());
        weak_pool = pool;
        in_flight = pool->Acquire();
        ASSERT_TRUE(in_flight.Valid());
        // 用户侧引用在此作用域结束，但在途句柄仍持有池引用
    }

    EXPECT_FALSE(weak_pool.expired());
    ASSERT_TRUE(in_flight.Valid());

    in_flight.Reset();  // 最后一个引用释放 → 归还 → 池析构
    EXPECT_TRUE(weak_pool.expired());
}

TEST(VideoFramePoolTest, ConcurrentRecycleIsThreadSafe) {
    constexpr std::size_t kCapacity = 16;
    constexpr std::size_t kThreadCount = 4;
    auto pool = std::make_shared<VideoFramePool>(kCapacity, MakeNv12Info());

    std::vector<FrameHandle> frames;
    frames.reserve(kCapacity);
    for (std::size_t index = 0; index < kCapacity; ++index) {
        auto frame = pool->Acquire();
        ASSERT_TRUE(frame.Valid());
        frames.push_back(std::move(frame));
    }
    EXPECT_EQ(pool->InFlightCount(), kCapacity);

    // 多消费者线程并发归还各自的句柄
    std::vector<std::thread> threads;
    for (std::size_t thread_index = 0; thread_index < kThreadCount; ++thread_index) {
        threads.emplace_back([&frames, thread_index]() {
            const std::size_t begin = thread_index * (kCapacity / kThreadCount);
            const std::size_t end = begin + (kCapacity / kThreadCount);
            for (std::size_t index = begin; index < end; ++index) {
                frames[index].Reset();
            }
        });
    }
    for (auto& thread : threads) {
        thread.join();
    }

    EXPECT_EQ(pool->RecycledCount(), kCapacity);
    EXPECT_EQ(pool->InFlightCount(), 0U);

    // 全部归还后可重新获取全部槽位，且无丢失/重复
    for (std::size_t index = 0; index < kCapacity; ++index) {
        auto frame = pool->Acquire();
        ASSERT_TRUE(frame.Valid());
    }
    EXPECT_EQ(pool->DroppedCount(), 0U);
}

}  // namespace
}  // namespace drone::video
