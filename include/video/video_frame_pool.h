/**
 * @file video_frame_pool.h
 * @brief 固定容量视频帧内存池
 *
 * 属于 drone/video 模块，配合 video_frame.h 的 RAII 句柄实现零拷贝传递。
 *
 * 职责：统一分配并复用视频像素缓冲，避免逐帧热路径反复申请大块内存。
 * 与 FrameHandle / FrameRecycler 配合形成完整闭环：
 *
 *   采集线程 Acquire() ──► 可写 FrameHandle ── Publish ──► 消费者共享
 *        ▲                                                     │
 *        └── 最后引用释放时 FrameBuffer 析构 → Recycle() ───────┘
 *
 * 设计要点：
 * - 构造时一次性分配 capacity × slot_size 连续内存，对齐到 alignment
 *   （默认 64 字节，cache line 对齐，兼容 DMA/RGA 搬运要求）。
 * - 池耗尽时 Acquire() 返回空句柄：不阻塞、不抛异常，递增 DroppedCount()。
 * - 生命周期：池必须用 shared_ptr 创建（内部依赖 enable_shared_from_this）。
 *   在途 FrameHandle 通过 FrameBuffer 持有池的引用，用户侧引用释放后池不会
 *   析构，直到全部缓冲归还——与"逆序停机"流程形成闭环，杜绝悬垂。
 * - 线程安全：Acquire 任意线程可调（实际约定单生产者采集线程）；Recycle
 *   任意线程可调（多消费者归还）。空闲列表由互斥锁保护，临界区仅 O(1)
 *   栈操作，锁粒度最小。
 *
 * 用法：
 * \code
 * auto pool = std::make_shared<VideoFramePool>(8, nv12_frame_template);
 * auto frame = pool->Acquire();          // 池空时返回空句柄
 * if (frame) {
 *     std::memcpy(frame.Data(), src, size);
 *     decoded_frame_topic.Emplace(std::move(frame));  // 发布，源句柄变空
 * }
 * \endcode
 */
#pragma once

#include <cassert>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <vector>

#include "video/video_frame.h"

namespace drone::video {

/// 固定容量视频帧内存池。
class VideoFramePool final : public FrameRecycler,
                             public std::enable_shared_from_this<VideoFramePool> {
public:
    /// 默认缓冲对齐（字节）：64 = cache line，兼容 DMA/RGA 搬运要求。
    static constexpr std::size_t kDefaultAlignment = 64;

    /// 构造固定容量内存池。
    /// @param capacity 池容量（槽位数），必须 > 0
    /// @param frame_template 帧模板：尺寸、stride、像素格式；buf_size 为 0 时
    ///                       按格式自动计算（NV12 = hor_stride×ver_stride×3/2，
    ///                       RGB888 = hor_stride×ver_stride×3）
    /// @param alignment 缓冲对齐字节数，必须是 2 的幂，默认 kDefaultAlignment
    /// @throw std::invalid_argument 容量为 0、对齐非法、帧模板无效或无法计算
    ///        缓冲大小时抛出
    VideoFramePool(std::size_t capacity, const VideoFrameInfo& frame_template,
                   std::size_t alignment = kDefaultAlignment);

    ~VideoFramePool() override;

    VideoFramePool(const VideoFramePool&) = delete;
    VideoFramePool& operator=(const VideoFramePool&) = delete;

    /// 按帧模板计算一帧缓冲的字节数；buf_size 已设置时直接返回。
    /// @return 计算失败（未知格式且未显式给出 buf_size）返回 0
    static std::size_t ComputeBufferSize(const VideoFrameInfo& info) noexcept;

    /// 获取一帧可写句柄（FrameRecycler 接口实现，供 FrameBuffer 归还槽位）。
    /// 越界或重复归还（槽位不在在途状态）被忽略并计入 DuplicateRecycleCount()。
    void Recycle(std::uint32_t slot_index) noexcept override;

    /// 获取一帧可写句柄；池耗尽时返回空句柄（Valid() == false）并计入
    /// DroppedCount()。不阻塞、不抛异常。
    [[nodiscard]] FrameHandle Acquire() noexcept;

    /// 池容量（槽位数）。
    [[nodiscard]] std::size_t Capacity() const noexcept { return capacity_; }

    /// 每个槽位的缓冲字节数（对齐后）。
    [[nodiscard]] std::size_t SlotSize() const noexcept { return slot_size_; }

    /// 当前空闲槽位数。
    [[nodiscard]] std::size_t IdleCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return idle_slots_.size();
    }

    /// 当前在途（已 Acquire 未归还）槽位数。
    [[nodiscard]] std::size_t InFlightCount() const {
        return capacity_ - IdleCount();
    }

    /// 累计成功获取帧数。
    [[nodiscard]] std::uint64_t AcquiredCount() const noexcept {
        return acquired_count_.load();
    }

    /// 累计因池耗尽丢弃的帧数。
    [[nodiscard]] std::uint64_t DroppedCount() const noexcept {
        return dropped_count_.load();
    }

    /// 累计归还帧数。
    [[nodiscard]] std::uint64_t RecycledCount() const noexcept {
        return recycled_count_.load();
    }

    /// 累计被忽略的非法归还次数（越界或重复归还），用于监控逻辑缺陷。
    [[nodiscard]] std::uint64_t DuplicateRecycleCount() const noexcept {
        return duplicate_recycle_count_.load();
    }

private:
    /// 槽位状态：空闲 / 在途，用于防御重复归还。
    enum class SlotState : std::uint8_t { kIdle = 0, kInFlight = 1 };

    /// 计算对齐后的槽位大小；参数非法时抛 std::invalid_argument。
    static std::size_t ComputeSlotSize(const VideoFrameInfo& info,
                                       std::size_t alignment);

    /// 向上对齐到 2 的幂边界。
    static constexpr std::size_t AlignUp(std::size_t value,
                                         std::size_t alignment) noexcept {
        return (value + alignment - 1) & ~(alignment - 1);
    }

    /// 单调时钟当前毫秒，用于帧时间戳（超龄过滤依据）。
    static std::int64_t SteadyNowMs() noexcept {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    /// 槽位缓冲首地址。
    [[nodiscard]] std::byte* StorageAt(std::uint32_t slot_index) const noexcept {
        return storage_ + static_cast<std::size_t>(slot_index) * slot_size_;
    }

    const std::size_t capacity_;
    const std::size_t slot_size_;
    const std::size_t alignment_;
    const VideoFrameInfo frame_template_;

    std::byte* storage_ = nullptr;  ///< 一次性分配的连续缓冲

    mutable std::mutex mutex_;
    std::vector<std::uint32_t> idle_slots_;  ///< 空闲槽位栈（LIFO）
    std::vector<SlotState> slot_states_;     ///< 槽位状态（锁内访问）

    std::atomic<std::uint64_t> next_sequence_{0};
    std::atomic<std::uint64_t> acquired_count_{0};
    std::atomic<std::uint64_t> recycled_count_{0};
    std::atomic<std::uint64_t> dropped_count_{0};
    std::atomic<std::uint64_t> duplicate_recycle_count_{0};
};

}  // namespace drone::video
