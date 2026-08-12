/**
 * @file video_frame_pool.cpp
 * @brief 固定容量视频帧内存池实现
 *
 * 内存一次性分配、槽位按 LIFO 栈复用；Acquire 单生产者、Recycle 多消费者，
 * 空闲列表与槽位状态由互斥锁保护（临界区仅 O(1) 栈操作）。
 */
#include "video/video_frame_pool.h"

#include <utility>

#include <spdlog/spdlog.h>

namespace drone::video {

namespace {

/// 校验对齐值必须是 2 的幂且非零。
void ValidateAlignment(std::size_t alignment) {
    if (alignment == 0 || (alignment & (alignment - 1)) != 0) {
        throw std::invalid_argument(
            "VideoFramePool alignment must be a non-zero power of two");
    }
}

/// 异常日志节流：第 1 次与每满 100 次才打印，避免高频异常刷屏。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

}  // namespace

std::size_t VideoFramePool::ComputeBufferSize(const VideoFrameInfo& info) noexcept {
    if (info.buf_size > 0) {
        return info.buf_size;
    }

    // 帧模板未显式给出 buf_size 时，按格式与 stride 推算。
    // hor_stride 单位像素，单行字节数 = hor_stride × BytesPerPixel(format)。
    const std::size_t y_plane_bytes =
        static_cast<std::size_t>(info.hor_stride) * BytesPerPixel(info.format) *
        info.ver_stride;
    switch (info.format) {
        case PixelFormat::kYuv420SpNv12:
            // Y 平面 + UV 半平面（UV 为 Y 的 1/2）
            return y_plane_bytes + y_plane_bytes / 2;
        case PixelFormat::kRgb888:
            // BytesPerPixel 已含每像素字节数，y_plane_bytes 即总字节数
            return y_plane_bytes;
        case PixelFormat::kUnknown:
        default:
            return 0;  // 无法推算，需调用者显式给出 buf_size
    }
}

std::size_t VideoFramePool::ComputeSlotSize(const VideoFrameInfo& info,
                                            std::size_t alignment) {
    ValidateAlignment(alignment);
    const std::size_t buf_size = ComputeBufferSize(info);
    if (buf_size == 0) {
        SPDLOG_ERROR("视频帧内存池无法推算缓冲大小: format={}",
                     static_cast<int>(info.format));
        throw std::invalid_argument(
            "VideoFramePool cannot determine buffer size: set info.buf_size "
            "explicitly or use a known pixel format");
    }
    return AlignUp(buf_size, alignment);
}

VideoFramePool::VideoFramePool(std::size_t capacity,
                               const VideoFrameInfo& frame_template,
                               std::size_t alignment)
    : capacity_(capacity),
      slot_size_(ComputeSlotSize(frame_template, alignment)),
      alignment_(alignment),
      frame_template_(frame_template),
      storage_(nullptr) {
    if (capacity_ == 0) {
        SPDLOG_ERROR("视频帧内存池容量必须大于 0");
        throw std::invalid_argument("VideoFramePool capacity must be greater than zero");
    }
    if (!frame_template_.Valid()) {
        SPDLOG_ERROR("视频帧内存池帧模板无效: format={} width={} height={}",
                     static_cast<int>(frame_template_.format),
                     frame_template_.width, frame_template_.height);
        throw std::invalid_argument(
            "VideoFramePool frame template must specify format, width and height");
    }

    // 一次性分配全部槽位的内存，后续 Acquire/Recycle 不再申请/释放大块内存。
    const std::size_t total_bytes = capacity_ * slot_size_;
    storage_ = static_cast<std::byte*>(
        ::operator new(total_bytes, std::align_val_t(alignment_)));

    // 初始状态：全部槽位空闲，按编号压栈。
    idle_slots_.reserve(capacity_);
    slot_states_.assign(capacity_, SlotState::kIdle);
    for (std::size_t index = 0; index < capacity_; ++index) {
        idle_slots_.push_back(static_cast<std::uint32_t>(index));
    }

    SPDLOG_INFO("视频帧内存池创建: 容量={} 槽位大小={}B 对齐={}B 总内存={:.1f}MB 格式={}",
                capacity_, slot_size_, alignment_,
                static_cast<double>(total_bytes) / 1024.0 / 1024.0,
                static_cast<int>(frame_template_.format));
}

VideoFramePool::~VideoFramePool() {
    // enable_shared_from_this 保证：只要有在途缓冲，池就不会析构（在途
    // FrameBuffer 持有池引用）。因此析构时全部槽位必然已归还，防御性断言。
    assert(idle_slots_.size() == capacity_);
    ::operator delete(storage_, std::align_val_t(alignment_));
    SPDLOG_INFO("视频帧内存池销毁: 容量={} 累计获取={} 归还={} 丢帧={} 非法归还={}",
                capacity_, acquired_count_.load(), recycled_count_.load(),
                dropped_count_.load(), duplicate_recycle_count_.load());
}

void VideoFramePool::Recycle(std::uint32_t slot_index) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    // 越界或重复归还（槽位不在在途状态）一律忽略，仅计数，便于监控逻辑缺陷。
    if (slot_index >= capacity_ ||
        slot_states_[slot_index] != SlotState::kInFlight) {
        const std::uint64_t duplicate = ++duplicate_recycle_count_;
        // 非法归一是逻辑缺陷，可能高频触发，同样需要节流防刷屏。
        if (ShouldLogThrottled(duplicate)) {
            SPDLOG_ERROR("视频帧内存池收到非法归还: 槽位={} (容量={})，累计 {}",
                         slot_index, capacity_, duplicate);
        }
        return;
    }

    slot_states_[slot_index] = SlotState::kIdle;
    idle_slots_.push_back(slot_index);
    ++recycled_count_;
}

FrameHandle VideoFramePool::Acquire() noexcept {
    std::uint32_t slot_index = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (idle_slots_.empty()) {
            // 池耗尽：直接丢帧，不阻塞调用方（采集线程不得等待）。
            const std::uint64_t dropped = ++dropped_count_;
            if (ShouldLogThrottled(dropped)) {
                SPDLOG_WARN("视频帧内存池已满，丢帧，累计 {}", dropped);
            }
            return FrameHandle{};
        }

        slot_index = idle_slots_.back();
        idle_slots_.pop_back();
        slot_states_[slot_index] = SlotState::kInFlight;
        ++acquired_count_;
    }

    // 锁外拼装元数据与构造缓冲：减少持锁时间；槽位已在锁内标记在途，
    // 并发 Recycle 不会重复分配该槽位。
    VideoFrameInfo info = frame_template_;
    info.sequence = next_sequence_.fetch_add(1);
    info.timestamp_ms = SteadyNowMs();

    // 帧缓冲强持有池引用（shared_from_this），保证在途期间池不被析构。
    auto buffer = std::make_shared<FrameBuffer>(
        info, StorageAt(slot_index), slot_size_, shared_from_this(), slot_index);
    return FrameHandle(std::move(buffer));
}

}  // namespace drone::video
