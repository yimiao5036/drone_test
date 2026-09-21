/**
 * @file stereo_frame_splitter.cpp
 * @brief 双目 SBS 全幅帧左右拆分器实现（StereoFrameSplitter）
 *
 * 消费线程订阅全幅 NV12 帧，按配置校验后分别裁剪左右半幅到左右各自
 * 内存池并发布。裁剪优先 RGA（DRONE_HAVE_RGA_DMA 条件编译，imcrop 单目
 * 一次调用），RGA 不可用/失败回退逐行 memcpy（Y 面 + UV 半面，按源/目的
 * stride），回退计入 RgaFallbackCount。
 */
#include "video/stereo_frame_splitter.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#ifdef DRONE_HAVE_RGA_DMA
#include <rga/im2d.hpp>
#include <rga/rga.h>
#endif

#include <spdlog/spdlog.h>

#include "video/video_frame_pool.h"

namespace drone::video {

namespace {

/// 单调时钟微秒（延迟统计与完成时刻）。
std::int64_t MonotonicUs() {
    return static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

/// 异常日志节流：第 1 次与每满 100 次才打印，避免高频异常刷屏。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

/// 像素对齐（向上取整）。
std::uint32_t AlignUp(std::uint32_t value, std::uint32_t alignment) {
    if (alignment == 0) {
        return value;
    }
    const std::uint32_t remainder = value % alignment;
    return remainder == 0 ? value : value + alignment - remainder;
}

}  // namespace

/// StereoFrameSplitter 实现细节（PIMPL）：输出内存池、RGA 路径与消费线程。
struct StereoFrameSplitter::Impl {
    explicit Impl(StereoFrameSplitterConfig config) : config(std::move(config)) {
        if (this->config.width == 0 || (this->config.width % 2) != 0) {
            throw std::invalid_argument("拆分全幅宽必须为正的偶数");
        }
        if (this->config.height == 0) {
            throw std::invalid_argument("拆分全幅高必须大于 0");
        }
        if (this->config.pool_capacity == 0) {
            throw std::invalid_argument("拆分输出内存池容量必须大于 0");
        }
        if (this->config.input_queue_capacity == 0) {
            throw std::invalid_argument("拆分输入订阅队列容量必须大于 0");
        }
        if (this->config.stride_alignment == 0) {
            throw std::invalid_argument("水平 stride 对齐必须大于 0");
        }
        CreatePools();
    }

    ~Impl() { Stop(); }

    StereoFrameSplitterConfig config;
    std::atomic<bool> stop_requested{false};
    std::thread thread;

    common::Topic<FrameHandle>* input_topic = nullptr;
    common::Topic<FrameHandle>::Subscription input_sub;
    common::Topic<FrameHandle> left_output;
    common::Topic<FrameHandle> right_output;

    std::shared_ptr<VideoFramePool> left_pool;
    std::shared_ptr<VideoFramePool> right_pool;

    // 状态计数（原子，跨线程可读）
    std::atomic<uint64_t> left_frame_count{0};
    std::atomic<uint64_t> right_frame_count{0};
    std::atomic<uint64_t> left_dropped_count{0};
    std::atomic<uint64_t> right_dropped_count{0};
    std::atomic<uint64_t> error_count{0};
    std::atomic<uint64_t> rga_fallback_count{0};

    /// 按单目尺寸创建左右输出池（构造期创建，失败即构造失败）。
    void CreatePools() {
        VideoFrameInfo frame_template;
        frame_template.width = config.width / 2;
        frame_template.height = config.height;
        frame_template.hor_stride =
            AlignUp(frame_template.width, config.stride_alignment);
        frame_template.ver_stride = frame_template.height;
        frame_template.format = PixelFormat::kYuv420SpNv12;
        left_pool =
            std::make_shared<VideoFramePool>(config.pool_capacity, frame_template);
        right_pool =
            std::make_shared<VideoFramePool>(config.pool_capacity, frame_template);
        SPDLOG_INFO("双目拆分输出池创建: 容量={}x2 单目={}x{} 水平stride={} 槽位={}B",
                    config.pool_capacity, frame_template.width,
                    frame_template.height, frame_template.hor_stride,
                    left_pool->SlotSize());
    }

    /// memcpy 回退裁剪：NV12 Y 面 h 行 + UV 半面 h/2 行（行宽均为单目宽字节）。
    // 源/目的 stride 可能不同（源全幅 stride vs 目的单目 stride），必须逐行。
    static void CropNv12Memcpy(const FrameHandle& source, FrameHandle& target,
                               std::uint32_t x_offset) {
        const VideoFrameInfo& src = source.Info();
        const VideoFrameInfo& dst = target.Info();
        const std::uint8_t* src_base =
            reinterpret_cast<const std::uint8_t*>(source.Data());
        std::uint8_t* dst_base = reinterpret_cast<std::uint8_t*>(target.Data());

        // Y 平面
        for (std::uint32_t row = 0; row < dst.height; ++row) {
            std::memcpy(dst_base + static_cast<std::size_t>(row) * dst.hor_stride,
                        src_base +
                            static_cast<std::size_t>(row) * src.hor_stride +
                            x_offset,
                        dst.width);
        }
        // UV 半平面：起始偏移 = stride × ver_stride，高度 h/2 行，行宽 = 单目宽字节
        const std::uint8_t* src_uv =
            src_base + static_cast<std::size_t>(src.hor_stride) * src.ver_stride;
        std::uint8_t* dst_uv =
            dst_base + static_cast<std::size_t>(dst.hor_stride) * dst.ver_stride;
        for (std::uint32_t row = 0; row < dst.height / 2; ++row) {
            std::memcpy(dst_uv + static_cast<std::size_t>(row) * dst.hor_stride,
                        src_uv +
                            static_cast<std::size_t>(row) * src.hor_stride +
                            x_offset,
                        dst.width);
        }
    }

    /// 裁剪一目：优先 RGA imcrop，不可用/失败回退 memcpy（计回退次数）。
    void CropEye(const FrameHandle& source, FrameHandle& target,
                 std::uint32_t x_offset) {
#ifdef DRONE_HAVE_RGA_DMA
        if (config.prefer_rga) {
            const VideoFrameInfo& src = source.Info();
            const VideoFrameInfo& dst = target.Info();
            // im2d 六参数重载顺序是 width/height/format/wstride/hstride。
            rga_buffer_t src_img = wrapbuffer_virtualaddr(
                const_cast<std::byte*>(source.Data()), static_cast<int>(src.width),
                static_cast<int>(src.height), RK_FORMAT_YCbCr_420_SP,
                static_cast<int>(src.hor_stride), static_cast<int>(src.ver_stride));
            rga_buffer_t dst_img = wrapbuffer_virtualaddr(
                target.Data(), static_cast<int>(dst.width),
                static_cast<int>(dst.height), RK_FORMAT_YCbCr_420_SP,
                static_cast<int>(dst.hor_stride), static_cast<int>(dst.ver_stride));
            im_rect crop_rect{};
            crop_rect.x = static_cast<int>(x_offset);
            crop_rect.y = 0;
            crop_rect.width = static_cast<int>(dst.width);
            crop_rect.height = static_cast<int>(dst.height);
            const IM_STATUS status = imcrop(src_img, dst_img, crop_rect);
            if (status == IM_STATUS_SUCCESS) {
                return;
            }
            const uint64_t count = rga_fallback_count.fetch_add(1) + 1;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("双目拆分 RGA 裁剪失败回退 memcpy: {}，累计 {}",
                            imStrError(status), count);
            }
        }
#endif
        // x86 无 RGA：memcpy 为正常路径
        CropNv12Memcpy(source, target, x_offset);
    }

    /// 发布一目到对应输出池与主题；池满只影响本目（另一目照常）。
    void PublishEye(const FrameHandle& source, std::uint32_t x_offset,
                    const std::shared_ptr<VideoFramePool>& pool,
                    common::Topic<FrameHandle>& output,
                    std::atomic<uint64_t>& frame_count,
                    std::atomic<uint64_t>& dropped_count,
                    const char* eye_name) {
        auto handle = pool->Acquire(source.Info().pipeline_ingress_time_ms);
        if (!handle.Valid()) {
            const uint64_t count = dropped_count.fetch_add(1) + 1;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("双目拆分{}目输出池满丢帧，累计 {}", eye_name, count);
            }
            return;
        }
        CropEye(source, handle, x_offset);
        handle.SetTiming(MonotonicUs() / 1000,
                         source.Info().pipeline_ingress_time_ms);
        (void)output.Emplace(std::move(handle));
        frame_count.fetch_add(1);
    }

    /// 校验并拆分一帧：输入非法（句柄/格式/尺寸）丢弃计错。
    void ProcessOne(const FrameHandle& frame) {
        const VideoFrameInfo& info = frame.Info();
        if (!frame.Valid() || info.format != PixelFormat::kYuv420SpNv12 ||
            info.width != config.width || info.height != config.height) {
            const uint64_t count = error_count.fetch_add(1) + 1;
            if (ShouldLogThrottled(count)) {
                SPDLOG_ERROR(
                    "双目拆分输入帧非法(句柄={} 格式={} 尺寸={}x{}，要求 NV12 {}x{})，"
                    "累计 {}",
                    frame.Valid(), static_cast<int>(info.format), info.width,
                    info.height, config.width, config.height, count);
            }
            return;
        }
        // 左半幅为主用目（本轮未实证物理左右，若相反仅交换两个 x_offset）。
        PublishEye(frame, 0, left_pool, left_output, left_frame_count,
                   left_dropped_count, "左");
        PublishEye(frame, config.width / 2, right_pool, right_output,
                   right_frame_count, right_dropped_count, "右");
    }

    /// 消费线程主循环。
    void Run() {
        while (!stop_requested.load()) {
            auto message = input_sub.WaitTakeFor(std::chrono::milliseconds(100));
            if (!message) {
                continue;  // 超时或主题关闭；循环顶检查停止标志
            }
            ProcessOne(**message);
        }
    }

    /// 启动消费线程（幂等；Stop 后重启按已存主题重新订阅）。
    bool Start() {
        if (thread.joinable()) {
            return true;
        }
        stop_requested = false;
        if (!input_sub.IsOpen() && input_topic != nullptr) {
            input_sub = input_topic->Subscribe(config.input_queue_capacity);
        }
        thread = std::thread(&Impl::Run, this);
        return true;
    }

    /// 停止：Reset 订阅唤醒 WaitTakeFor，再 join 唯一消费线程。
    void Stop() {
        if (!thread.joinable()) {
            return;
        }
        stop_requested = true;
        input_sub.Reset();
        thread.join();
    }
};

StereoFrameSplitter::StereoFrameSplitter(StereoFrameSplitterConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
#ifdef DRONE_HAVE_RGA_DMA
    const char* rga_desc = impl_->config.prefer_rga ? "优先" : "关闭";
#else
    const char* rga_desc = "无（memcpy）";
#endif
    SPDLOG_INFO("双目拆分器创建: 全幅={}x{} 池容量={}x2 RGA={}",
                impl_->config.width, impl_->config.height,
                impl_->config.pool_capacity, rga_desc);
}

StereoFrameSplitter::~StereoFrameSplitter() {
    if (impl_ != nullptr) {
        impl_->Stop();
    }
    SPDLOG_INFO("双目拆分器销毁");
}

bool StereoFrameSplitter::Start() {
    if (!impl_->Start()) {
        return false;
    }
    SPDLOG_INFO("双目拆分器启动");
    return true;
}

void StereoFrameSplitter::Stop() {
    impl_->Stop();
    SPDLOG_INFO("双目拆分器停止");
}

bool StereoFrameSplitter::IsRunning() const {
    return impl_->thread.joinable();
}

void StereoFrameSplitter::SetInput(common::Topic<FrameHandle>& input) {
    impl_->input_topic = &input;
    impl_->input_sub = input.Subscribe(impl_->config.input_queue_capacity);
}

common::Topic<FrameHandle>& StereoFrameSplitter::LeftOutput() {
    return impl_->left_output;
}

common::Topic<FrameHandle>& StereoFrameSplitter::RightOutput() {
    return impl_->right_output;
}

std::uint64_t StereoFrameSplitter::LeftFrameCount() const {
    return impl_->left_frame_count.load();
}

std::uint64_t StereoFrameSplitter::RightFrameCount() const {
    return impl_->right_frame_count.load();
}

std::uint64_t StereoFrameSplitter::LeftDroppedCount() const {
    return impl_->left_dropped_count.load();
}

std::uint64_t StereoFrameSplitter::RightDroppedCount() const {
    return impl_->right_dropped_count.load();
}

std::uint64_t StereoFrameSplitter::ErrorCount() const {
    return impl_->error_count.load();
}

std::uint64_t StereoFrameSplitter::RgaFallbackCount() const {
    return impl_->rga_fallback_count.load();
}

}  // namespace drone::video
