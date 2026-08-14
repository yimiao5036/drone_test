/**
 * @file video_decoder.cpp
 * @brief 视频解码器实现（VideoDecoder）
 *
 * 订阅 H.264/H.265 码流块（common::EncodedFrame），解码为 NV12 帧，
 * 从内存池分配 FrameHandle 发布（零拷贝共享，最后引用释放自动归还）。
 *
 * 设计要点：
 * - PIMPL 隔离 FFmpeg 头文件，接口头文件不依赖 FFmpeg。
 * - 解码器优先 rkmpp 硬解（香橙派，输出 NV12）；不可用或打开失败时
 *   回退 FFmpeg 软解（开发机验证，YUV420P 经 swscale 转 NV12）。
 * - 帧内存池懒创建：配置未给分辨率时首帧确定尺寸后创建。
 * - 输出统一 NV12（与 video_frame.h PixelFormat::kYuv420SpNv12 对应），
 *   后续 YOLO 经 RGA 转 RGB 输入。
 */
#include "video/video_decoder.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <thread>

#include "video/video_frame_pool.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <spdlog/spdlog.h>

namespace drone::video {

namespace {

/// 异常日志节流：第 1 次与每满 100 次才打印，避免高频异常刷屏。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

/// 将 FFmpeg 错误码转换为可读字符串。
std::string AvErrorToString(int errnum) {
    char buf[AV_ERROR_MAX_STRING_SIZE] = {0};
    av_make_error_string(buf, sizeof(buf), errnum);
    return std::string(buf);
}

/// 像素对齐（向上取整）。
std::uint32_t AlignUp(std::uint32_t value, std::uint32_t alignment) {
    if (alignment == 0) {
        return value;
    }
    return (value + alignment - 1) / alignment * alignment;
}

}  // namespace

/// VideoDecoder 实现细节（PIMPL）：FFmpeg 解码上下文、内存池与解码线程。
struct VideoDecoder::Impl {
    explicit Impl(VideoDecoderConfig config) : config(std::move(config)) {
        if (this->config.pool_capacity == 0) {
            throw std::invalid_argument("解码帧内存池容量必须大于 0");
        }
        if (this->config.stride_alignment == 0) {
            throw std::invalid_argument("水平 stride 对齐必须大于 0");
        }
    }

    ~Impl() {
        Stop();
        CleanupDecoder();
    }

    VideoDecoderConfig config;
    std::atomic<bool> stop_requested{false};
    std::thread thread;

    common::Topic<common::EncodedFrame>::Subscription input_sub;
    common::Topic<FrameHandle> frame_output;

    // FFmpeg 解码上下文
    const AVCodec* decoder = nullptr;
    AVCodecContext* codec_ctx = nullptr;
    AVBufferRef* hw_device = nullptr;  // DRM 硬件设备（硬解路径）
    bool is_hardware = false;
    bool decoder_creation_failed = false;  // 创建失败后不再重试（避免刷屏）
    SwsContext* sws_ctx = nullptr;
    int sws_src_format = -1;  // 当前 sws 上下文对应的源格式（缓存）
    AVFrame* decoded_frame = nullptr;
    AVFrame* sw_frame = nullptr;  // 硬解转存帧（NV12）

    // 帧内存池（懒创建）
    std::shared_ptr<VideoFramePool> pool;

    // 状态计数
    std::atomic<uint64_t> decoded_count{0};
    std::atomic<uint64_t> dropped_count{0};
    std::atomic<uint64_t> error_count{0};

    /// 创建帧内存池（按解码分辨率）。
    /// @throw std::invalid_argument / std::bad_alloc 池参数非法或内存不足
    void CreatePool(int width, int height) {
        VideoFrameInfo frame_template;
        frame_template.width = static_cast<std::uint32_t>(width);
        frame_template.height = static_cast<std::uint32_t>(height);
        frame_template.hor_stride =
            AlignUp(frame_template.width, config.stride_alignment);
        frame_template.ver_stride = frame_template.height;
        frame_template.format = PixelFormat::kYuv420SpNv12;
        pool = std::make_shared<VideoFramePool>(config.pool_capacity, frame_template);
        SPDLOG_INFO("解码帧内存池创建: 容量={} 分辨率={}x{} 水平stride={} 槽位={}B",
                    config.pool_capacity, frame_template.width,
                    frame_template.height, frame_template.hor_stride,
                    pool->SlotSize());
    }

    /// 创建解码器；优先 rkmpp 硬解，不可用或失败时回退软解。
    /// @param parameter_sets 流级参数集（SPS/PPS 等，可空），在打开解码器前设置。
    bool CreateDecoder(common::VideoCodec codec,
                       const std::vector<uint8_t>& parameter_sets) {
        AVCodecID codec_id = AV_CODEC_ID_NONE;
        if (codec == common::VideoCodec::kH265) {
            codec_id = AV_CODEC_ID_HEVC;
        } else if (codec == common::VideoCodec::kH264) {
            codec_id = AV_CODEC_ID_H264;
        }
        if (codec_id == AV_CODEC_ID_NONE) {
            ++error_count;
            SPDLOG_ERROR("视频解码器不支持的编码类型: codec={}", static_cast<int>(codec));
            return false;
        }

        // 硬解优先：RK3588 的 rkmpp 解码器
        bool hardware_attempted = false;
        if (config.prefer_hardware) {
            const AVCodec* hw_decoder = avcodec_find_decoder_by_name(
                codec_id == AV_CODEC_ID_HEVC ? "hevc_rkmpp" : "h264_rkmpp");
            if (hw_decoder != nullptr) {
                hardware_attempted = true;
                if (TryCreateCodec(hw_decoder, true, parameter_sets)) {
                    return true;
                }
            }
        }

        // 回退软解（开发机无 rkmpp 为正常路径，打 INFO；硬件存在但打开失败打 WARN）
        const AVCodec* sw_decoder = avcodec_find_decoder(codec_id);
        if (sw_decoder == nullptr) {
            ++error_count;
            SPDLOG_ERROR("视频解码器未找到对应软解解码器: codec_id={}", static_cast<int>(codec_id));
            return false;
        }
        if (!TryCreateCodec(sw_decoder, false, parameter_sets)) {
            return false;
        }
        if (hardware_attempted) {
            SPDLOG_WARN("视频解码器回退软解（rkmpp 不可用）: {}", sw_decoder->name);
        } else {
            SPDLOG_INFO("视频解码器使用软解: {}", sw_decoder->name);
        }
        return true;
    }

    /// 用指定解码器创建上下文并打开；失败时释放资源返回 false。
    /// @param parameter_sets 流级参数集；在 avcodec_open2 前写入 extradata。
    bool TryCreateCodec(const AVCodec* codec, bool hardware,
                        const std::vector<uint8_t>& parameter_sets) {
        CleanupCodec();

        decoder = codec;
        is_hardware = hardware;
        codec_ctx = avcodec_alloc_context3(decoder);
        if (codec_ctx == nullptr) {
            ++error_count;
            SPDLOG_ERROR("视频解码器分配上下文失败: {}", decoder->name);
            return false;
        }
        codec_ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;

        // 流级参数集（SPS/PPS 等）：RTSP/容器流在流头而非逐帧码流中。
        // 重要（对齐已验证成功的原型 videoPart/rtsp_yolo_stream）：
        // - HEVC（本相机）的容器 extradata 是 HEVCDecoderConfigurationRecord
        //   （MP4 格式），而非 Annex-B NALU 序列。整块塞给 rkmpp 硬解当作参数
        //   集是错误的，会导致 rkmpp 一打开就对码流报 `invalid pps id 0` 并段错误。
        // - 正确做法：硬解交给 rkmpp 从码流内嵌的 SPS/PPS 自行解析（LIVE555 的
        //   RTP 流内嵌参数，原型实测可行），不塞容器 extradata。
        // - 软解（libx264/hevc 等）接受 mp4 config 格式 extradata，保留塞入以
        //   支持“容器头带参数”的流（H264 容器 extradata 是 Annex-B，也可用）。
        if (!is_hardware && !parameter_sets.empty()) {
            codec_ctx->extradata = static_cast<uint8_t*>(
                av_mallocz(parameter_sets.size() + AV_INPUT_BUFFER_PADDING_SIZE));
            if (codec_ctx->extradata == nullptr) {
                ++error_count;
                SPDLOG_ERROR("视频解码器分配参数集内存失败: {}", decoder->name);
                return false;
            }
            std::memcpy(codec_ctx->extradata, parameter_sets.data(),
                        parameter_sets.size());
            codec_ctx->extradata_size = static_cast<int>(parameter_sets.size());
        }

        if (is_hardware) {
            hw_device = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_DRM);
            if (hw_device != nullptr && av_hwdevice_ctx_init(hw_device) == 0) {
                codec_ctx->hw_device_ctx = av_buffer_ref(hw_device);
            } else {
                if (hw_device != nullptr) {
                    av_buffer_unref(&hw_device);
                }
                hw_device = nullptr;
                ++error_count;
                if (ShouldLogThrottled(error_count)) {
                    SPDLOG_WARN("视频解码器初始化 DRM 硬件设备失败，回退软解，累计 {}",
                                error_count.load());
                }
                return false;
            }
        }

        const int open_ret = avcodec_open2(codec_ctx, decoder, nullptr);
        if (open_ret < 0) {
            ++error_count;
            if (ShouldLogThrottled(error_count)) {
                SPDLOG_WARN("视频解码器打开失败: {} ({})，累计 {}",
                            decoder->name, AvErrorToString(open_ret), error_count.load());
            }
            return false;
        }

        SPDLOG_INFO("视频解码器创建: {} 模式={}", decoder->name,
                    is_hardware ? "rkmpp 硬解" : "软解");
        return true;
    }

    /// 释放解码器相关资源（幂等）。
    void CleanupCodec() {
        if (sws_ctx != nullptr) {
            sws_freeContext(sws_ctx);
            sws_ctx = nullptr;
            sws_src_format = -1;
        }
        if (decoded_frame != nullptr) {
            av_frame_free(&decoded_frame);
            decoded_frame = nullptr;
        }
        if (sw_frame != nullptr) {
            av_frame_free(&sw_frame);
            sw_frame = nullptr;
        }
        if (codec_ctx != nullptr) {
            avcodec_free_context(&codec_ctx);
            codec_ctx = nullptr;
        }
        if (hw_device != nullptr) {
            av_buffer_unref(&hw_device);
            hw_device = nullptr;
        }
        decoder = nullptr;
        is_hardware = false;
    }

    void CleanupDecoder() {
        CleanupCodec();
        pool.reset();
    }

    /// 解码线程主循环。
    void DecodeLoop() {
        AVPacket* packet = av_packet_alloc();
        if (packet == nullptr) {
            ++error_count;
            SPDLOG_ERROR("视频解码器分配 AVPacket 失败");
            return;
        }

        while (!stop_requested.load()) {
            auto message = input_sub.WaitTakeFor(std::chrono::milliseconds(100));
            if (!message) {
                continue;  // 超时或主题关闭；循环顶检查停止标志
            }
            const auto& frame = **message;

            // 参数集消息（data 空但带参数集）：仅用于初始化解码器
            if (frame.data.empty()) {
                if (!frame.parameter_sets.empty() && codec_ctx == nullptr &&
                    !decoder_creation_failed) {
                    if (!CreateDecoder(frame.codec, frame.parameter_sets)) {
                        decoder_creation_failed = true;
                    }
                }
                continue;
            }
            // 不再按“未初始化跳过非关键帧”：本款摄像头 RTSP 的 SPS/PPS 是 IDR
            // 之前独立到达的非关键帧小包，旧逻辑会丢弃它们，使 rkmpp 永远收不到
            // 参数集，随后解析 IDR 报 invalid pps 并段错误。
            // 修正（对齐原型 videoPart/rtsp_yolo_stream）：解码器未创建时，由
            // below 的 DecodeOne 用首个数据帧（任意类型）创建并与该帧一并送包，
            // 之后全量送包，由 rkmpp 自行完成 SPS/PPS 参数同步与关键帧等待。
            if (decoder_creation_failed) {
                continue;  // 创建已失败，跳过，避免无效空转
            }
            DecodeOne(frame, packet);
        }
        av_packet_free(&packet);
    }

    /// 解码单个码流块并发布所有输出帧。
    void DecodeOne(const common::EncodedFrame& encoded, AVPacket* packet) {
        // 首个数据帧确定编码类型并创建解码器（参数集随帧或已由参数集消息提供）
        if (codec_ctx == nullptr && !decoder_creation_failed) {
            if (!CreateDecoder(encoded.codec, encoded.parameter_sets)) {
                decoder_creation_failed = true;
                return;
            }
        }
        if (codec_ctx == nullptr) {
            return;  // 解码器创建失败，无法继续
        }
        // 分配解码/转存帧（解码器确定后尺寸已知，创建一次）
        if (decoded_frame == nullptr) {
            decoded_frame = av_frame_alloc();
            sw_frame = av_frame_alloc();
            if (decoded_frame == nullptr || sw_frame == nullptr) {
                ++error_count;
                SPDLOG_ERROR("视频解码器分配 AVFrame 失败");
                return;
            }
        }

        // 拷贝码流数据到 packet（packet 拥有缓冲，unref 语义安全；
        // 骨架期接受拷贝开销，实测不足时再优化为零拷贝引用）
        av_packet_unref(packet);
        const int alloc_ret =
            av_new_packet(packet, static_cast<int>(encoded.data.size()));
        if (alloc_ret < 0) {
            ++error_count;
            if (ShouldLogThrottled(error_count)) {
                SPDLOG_ERROR("视频解码器分配 packet 失败: {}，累计 {}",
                             AvErrorToString(alloc_ret), error_count.load());
            }
            return;
        }
        std::memcpy(packet->data, encoded.data.data(), encoded.data.size());

        const int send_ret = avcodec_send_packet(codec_ctx, packet);
        if (send_ret < 0 && send_ret != AVERROR(EAGAIN)) {
            ++error_count;
            if (ShouldLogThrottled(error_count)) {
                SPDLOG_ERROR("视频解码器送包失败: {}，累计 {}",
                             AvErrorToString(send_ret), error_count.load());
            }
            return;
        }

        // 取出全部可用输出帧
        for (;;) {
            av_frame_unref(decoded_frame);
            const int recv_ret = avcodec_receive_frame(codec_ctx, decoded_frame);
            if (recv_ret == AVERROR(EAGAIN) || recv_ret == AVERROR_EOF) {
                break;
            }
            if (recv_ret < 0) {
                ++error_count;
                if (ShouldLogThrottled(error_count)) {
                    SPDLOG_ERROR("视频解码器取帧失败: {}，累计 {}",
                                 AvErrorToString(recv_ret), error_count.load());
                }
                break;
            }
            PublishFrame(decoded_frame);
        }
    }

    /// 将解码帧转为 NV12 写入内存池并发布。
    void PublishFrame(AVFrame* source) {
        // 硬解帧：从 DRM 显存转存到系统内存（输出 NV12）
        AVFrame* frame = source;
        if (source->format == AV_PIX_FMT_DRM_PRIME) {
            av_frame_unref(sw_frame);
            if (av_hwframe_transfer_data(sw_frame, source, 0) < 0) {
                ++error_count;
                if (ShouldLogThrottled(error_count)) {
                    SPDLOG_ERROR("视频解码器硬件帧转存失败，累计 {}", error_count.load());
                }
                return;
            }
            frame = sw_frame;
        }

        const int width = frame->width;
        const int height = frame->height;
        if (width <= 0 || height <= 0) {
            return;
        }

        // 懒建池：首帧确定分辨率
        if (pool == nullptr) {
            try {
                CreatePool(width, height);
            } catch (const std::exception& e) {
                ++error_count;
                if (ShouldLogThrottled(error_count)) {
                    SPDLOG_ERROR("视频解码器创建帧内存池失败: {}，累计 {}",
                                 e.what(), error_count.load());
                }
                return;
            }
        }

        auto handle = pool->Acquire();
        if (!handle.Valid()) {
            ++dropped_count;
            if (ShouldLogThrottled(dropped_count)) {
                SPDLOG_WARN("解码帧内存池已满丢帧，累计 {}", dropped_count.load());
            }
            return;
        }

        const VideoFrameInfo& info = handle.Info();
        std::byte* dst = handle.Data();

        // 目标 NV12（Y 平面 + UV 半平面），swscale 按行写入
        const int y_bytes =
            static_cast<int>(info.hor_stride) * static_cast<int>(info.height);
        uint8_t* dst_planes[2] = {reinterpret_cast<uint8_t*>(dst),
                                  reinterpret_cast<uint8_t*>(dst) + y_bytes};
        int dst_linesize[2] = {static_cast<int>(info.hor_stride),
                               static_cast<int>(info.hor_stride)};

        // 源格式变化时重建转换上下文（软解 YUV420P→NV12；硬解 NV12→NV12）
        if (sws_ctx == nullptr || sws_src_format != frame->format) {
            if (sws_ctx != nullptr) {
                sws_freeContext(sws_ctx);
            }
            sws_ctx = sws_getContext(width, height,
                                     static_cast<AVPixelFormat>(frame->format),
                                     width, height, AV_PIX_FMT_NV12,
                                     SWS_BILINEAR, nullptr, nullptr, nullptr);
            sws_src_format = frame->format;
            if (sws_ctx == nullptr) {
                ++error_count;
                if (ShouldLogThrottled(error_count)) {
                    SPDLOG_ERROR("视频解码器创建 sws 转换上下文失败(源格式={})，累计 {}",
                                 frame->format, error_count.load());
                }
                return;
            }
        }

        sws_scale(sws_ctx, frame->data, frame->linesize, 0, height,
                  dst_planes, dst_linesize);

        (void)frame_output.Emplace(std::move(handle));
        ++decoded_count;
    }

    void Stop() {
        if (!thread.joinable()) {
            return;
        }
        stop_requested = true;
        input_sub.Reset();  // 唤醒 WaitTakeFor 中的等待
        thread.join();
    }
};

VideoDecoder::VideoDecoder(VideoDecoderConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
    SPDLOG_INFO("视频解码器创建: 池容量={} 预置分辨率={}x{} 硬解优先={}",
                impl_->config.pool_capacity, impl_->config.width,
                impl_->config.height, impl_->config.prefer_hardware);
}

VideoDecoder::~VideoDecoder() {
    if (impl_ != nullptr) {
        impl_->Stop();
    }
    SPDLOG_INFO("视频解码器销毁");
}

bool VideoDecoder::Start() {
    if (impl_->thread.joinable()) {
        return true;  // 已启动，幂等
    }
    impl_->stop_requested = false;
    // 预知分辨率时预建池（避免首帧瞬时分配大块内存）
    if (impl_->config.width > 0 && impl_->config.height > 0) {
        try {
            impl_->CreatePool(impl_->config.width, impl_->config.height);
        } catch (const std::exception& e) {
            impl_->error_count.fetch_add(1);
            SPDLOG_ERROR("视频解码器启动失败（预建内存池）: {}", e.what());
            return false;
        }
    }
    impl_->thread = std::thread(&Impl::DecodeLoop, impl_.get());
    SPDLOG_INFO("视频解码器启动");
    return true;
}

void VideoDecoder::Stop() {
    impl_->Stop();
    SPDLOG_INFO("视频解码器停止");
}

bool VideoDecoder::IsRunning() const {
    return impl_->thread.joinable();
}

void VideoDecoder::SetInput(common::Topic<common::EncodedFrame>& input) {
    // 队列容量 8：码流块小、频率 25fps，适度缓冲抗抖动；解码落后由
    // kDropOldest 策略保证只处理最新
    impl_->input_sub = input.Subscribe(8);
}

common::Topic<FrameHandle>& VideoDecoder::FrameOutput() {
    return impl_->frame_output;
}

uint64_t VideoDecoder::DecodedFrameCount() const {
    return impl_->decoded_count.load();
}

uint64_t VideoDecoder::DroppedFrameCount() const {
    return impl_->dropped_count.load();
}

uint64_t VideoDecoder::ErrorCount() const {
    return impl_->error_count.load();
}

}  // namespace drone::video
