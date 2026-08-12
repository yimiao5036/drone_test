/**
 * @file video_frame.h
 * @brief 视频帧元数据与 RAII 缓冲句柄
 *
 * 属于 drone/video 模块，是视频链路的公共数据类型。
 *
 * 设计目标：让视频帧以"句柄"形式在 Topic 中零拷贝传递。
 * 链路如下（对应 docs/项目文件结构.md §5）：
 *
 *   内存池分配槽位 ──► FrameHandle（可写）── Publish ──► Topic<FrameHandle>
 *         ▲                                                    │
 *         └──── 最后一个消费者释放句柄时自动归还内存池 ─────────┘
 *
 * 所有权规则：
 * - 底层像素缓冲由固定容量内存池（VideoFramePool，见步骤 2）统一分配，
 *   本文件不涉及缓冲的分配，只定义"如何持有和归还"。
 * - FrameHandle 禁止拷贝、允许移动：同一帧句柄只能发布一次，发布即移动，
 *   源句柄变为空，从机制上杜绝一帧被两个生产者写入。
 * - 归还通过 FrameRecycler 抽象接口解耦：内存池实现该接口并注入 FrameBuffer，
 *   最后一个 shared_ptr<FrameBuffer> 释放时自动调用 Recycle() 归还槽位。
 *
 * 使用约定：
 * - 采集线程：Acquire 得到可写句柄 → 写数据 → Emplace(std::move(handle)) 发布。
 * - 消费线程：收到 const FrameHandle&，只读访问；超龄帧按 Info().timestamp_ms
 *   过滤（Topic 队列容量 1~2 + kDropOldest 已覆盖大部分场景）。
 * - 帧时间戳统一使用单调时钟（毫秒），用于帧率统计与超龄过滤；源端时间戳
 *   （如 RTP 时间戳）单独存放，用于音视频同步，不参与超龄判断。
 */
#ifndef DRONE_VIDEO_VIDEO_FRAME_H_
#define DRONE_VIDEO_VIDEO_FRAME_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

namespace drone::video {

/// 像素格式枚举。
/// 当前覆盖原型验证链路用到的格式，后续按需扩展。
enum class PixelFormat {
    kUnknown = 0,
    kYuv420SpNv12,  ///< NV12（YUV420 半平面），MPP 硬解码输出格式
    kRgb888,        ///< RGB888 连续内存，模型输入 / 叠加输出格式
};

/// 每像素字节数（按格式）。
/// NV12 以 Y 平面计每像素 1 字节；返回 0 表示未知格式。
constexpr std::size_t BytesPerPixel(PixelFormat format) noexcept {
    switch (format) {
        case PixelFormat::kYuv420SpNv12:
            return 1;
        case PixelFormat::kRgb888:
            return 3;
        case PixelFormat::kUnknown:
        default:
            return 0;
    }
}

/// 视频帧元数据（不可变，句柄发布后不再修改）。
/// stride 单位对齐 MPP/RGA 约定：hor_stride 为像素、ver_stride 为行。
struct VideoFrameInfo {
    std::uint32_t width = 0;        ///< 有效像素宽
    std::uint32_t height = 0;       ///< 有效像素高
    std::uint32_t hor_stride = 0;   ///< 水平 stride（像素），行对齐后 >= width
    std::uint32_t ver_stride = 0;   ///< 垂直 stride（行），>= height
    std::size_t buf_size = 0;       ///< 缓冲实际占用字节数（含对齐填充）
    PixelFormat format = PixelFormat::kUnknown;  ///< 像素格式
    std::uint64_t sequence = 0;     ///< 递增帧序号，用于调试与丢帧检测
    std::int64_t timestamp_ms = 0;  ///< 单调时钟时间戳（毫秒），超龄过滤依据
    std::int64_t source_timestamp_ms = 0;  ///< 源端时间戳（可选，如 RTP），同步用

    /// 单行字节数 = 水平 stride × 每像素字节数。
    [[nodiscard]] std::size_t LineSizeBytes() const noexcept {
        return static_cast<std::size_t>(hor_stride) * BytesPerPixel(format);
    }

    /// 元数据是否有效（宽高与格式齐全）。
    [[nodiscard]] bool Valid() const noexcept {
        return format != PixelFormat::kUnknown && width > 0 && height > 0;
    }
};

/// 缓冲归还接口（由 VideoFramePool 实现）。
///
/// 设计为抽象接口的目的：FrameBuffer 只依赖"归还槽位"这一行为，
/// 不依赖具体内存池类型，内存池可替换（如换成 DRM 显存池）而无需改帧句柄。
/// 线程安全：Recycle 可能在任意消费者线程被调用，实现必须可并发调用且幂等。
class FrameRecycler {
public:
    virtual ~FrameRecycler() = default;

    /// 将槽位归还给内存池。
    /// @param slot_index 内存池槽位编号，由 Acquire 时写入 FrameBuffer。
    virtual void Recycle(std::uint32_t slot_index) noexcept = 0;
};

/// 帧缓冲本体：持有像素数据指针、元数据与归还回调。
///
/// 由内存池 Acquire 时构造，通过 std::shared_ptr 共享给 FrameHandle 及
/// Topic 中的消息句柄。最后一个引用释放（析构）时自动归还内存池槽位。
class FrameBuffer {
public:
    /// 构造一个帧缓冲。
    /// @param info 帧元数据
    /// @param data 池内像素缓冲首地址（非空）
    /// @param capacity 池内缓冲容量（字节），应 >= info.buf_size
    /// @param recycler 归还回调，内存池实现；为空则析构时不归还（测试/外部缓冲用）
    /// @param slot_index 内存池槽位编号
    FrameBuffer(VideoFrameInfo info, std::byte* data, std::size_t capacity,
                std::shared_ptr<FrameRecycler> recycler, std::uint32_t slot_index)
        : info_(info),
          data_(data),
          capacity_(capacity),
          recycler_(std::move(recycler)),
          slot_index_(slot_index) {}

    ~FrameBuffer() {
        // 最后一个引用释放时归还内存池槽位。
        // 归还只发生一次：shared_ptr 计数归零仅触发一次析构。
        if (recycler_ && data_ != nullptr) {
            recycler_->Recycle(slot_index_);
        }
    }

    FrameBuffer(const FrameBuffer&) = delete;
    FrameBuffer& operator=(const FrameBuffer&) = delete;

    /// 帧元数据（只读）。
    [[nodiscard]] const VideoFrameInfo& Info() const noexcept { return info_; }

    /// 像素缓冲首地址（可写）。仅供发布前的采集线程使用。
    [[nodiscard]] std::byte* Data() noexcept { return data_; }

    /// 像素缓冲首地址（只读）。供发布后的消费线程使用。
    [[nodiscard]] const std::byte* Data() const noexcept { return data_; }

    /// 池内缓冲容量（字节）。
    [[nodiscard]] std::size_t Capacity() const noexcept { return capacity_; }

    /// 内存池槽位编号。
    [[nodiscard]] std::uint32_t SlotIndex() const noexcept { return slot_index_; }

private:
    VideoFrameInfo info_;
    std::byte* data_;
    std::size_t capacity_;
    std::shared_ptr<FrameRecycler> recycler_;
    std::uint32_t slot_index_;
};

/// 视频帧 RAII 句柄：Topic 消息类型。
///
/// 语义：
/// - 默认构造为空句柄；移动后源句柄变空，保证"同一帧只发布一次"。
/// - 禁止拷贝：多个生产者不能共享同一帧缓冲的写权限。
/// - Valid() 为 true 时持有 FrameBuffer（shared_ptr），最后一个句柄释放时
///   底层缓冲自动归还内存池。
///
/// 用法：
/// \code
/// auto handle = pool.Acquire();              // 可写句柄
/// std::memcpy(handle.Data(), src, size);     // 写入像素数据
/// decoded_frame_topic.Emplace(std::move(handle));  // 发布，源句柄变空
/// \endcode
class FrameHandle {
public:
    /// 空句柄。
    FrameHandle() = default;

    /// 从帧缓冲构造句柄（内存池内部使用）。
    explicit FrameHandle(std::shared_ptr<FrameBuffer> buffer)
        : buffer_(std::move(buffer)) {}

    // 禁拷贝：同一帧缓冲的写权限只属于一个句柄。
    FrameHandle(const FrameHandle&) = delete;
    FrameHandle& operator=(const FrameHandle&) = delete;

    // 可移动：发布即移动，源句柄变空。
    FrameHandle(FrameHandle&&) noexcept = default;
    FrameHandle& operator=(FrameHandle&&) noexcept = default;

    /// 是否持有有效帧缓冲。
    [[nodiscard]] bool Valid() const noexcept { return buffer_ != nullptr; }

    /// Valid() 的便捷写法，支持 if (handle) 判空。
    [[nodiscard]] explicit operator bool() const noexcept { return Valid(); }

    /// 帧元数据；空句柄返回空元数据（Valid() 为 false）。
    [[nodiscard]] const VideoFrameInfo& Info() const noexcept {
        static const VideoFrameInfo kEmptyInfo;
        return buffer_ ? buffer_->Info() : kEmptyInfo;
    }

    /// 像素缓冲首地址（可写）；空句柄返回 nullptr。
    [[nodiscard]] std::byte* Data() noexcept {
        return buffer_ ? buffer_->Data() : nullptr;
    }

    /// 像素缓冲首地址（只读）；空句柄返回 nullptr。
    [[nodiscard]] const std::byte* Data() const noexcept {
        return buffer_ ? buffer_->Data() : nullptr;
    }

    /// 池内缓冲容量（字节）；空句柄返回 0。
    [[nodiscard]] std::size_t Capacity() const noexcept {
        return buffer_ ? buffer_->Capacity() : 0;
    }

    /// 显式释放：立即归还内存池槽位（不等析构）。幂等。
    void Reset() noexcept { buffer_.reset(); }

private:
    std::shared_ptr<FrameBuffer> buffer_;
};

}  // namespace drone::video

#endif  // DRONE_VIDEO_VIDEO_FRAME_H_
