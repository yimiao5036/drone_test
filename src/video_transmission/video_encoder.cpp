/**
 * @file video_encoder.cpp
 * @brief 视频编码 + 图传推送后端实现（FFmpeg 封装）
 *
 * 将 NV12 帧编码（优先 rkmpp 硬编码，回退软编码）并通过 RTSP 推流给图传。
 * 可直接接收 NV12 软件帧：rkmpp 编码器（rkmppenc）内部自动导入 MPP 缓冲，
 * 无需手工分配 DRM/MPP 内存；软编码 libx264 需 YUV420P，经 sws 转换。
 *
 * 与原型 videoPart/rtsp_yolo_stream/src/rtsp_encoder.cpp 的关系：
 * - 去掉 OpenCV BGR 输入（本工程解码即输出 NV12，直接送编码器省一次转换）；
 * - PIMPL 隔离 FFmpeg 头文件（接口头文件不依赖 FFmpeg）；
 * - 抽取为 IVideoEncoderBackend 抽象实现，兼容硬编/软编切换；
 * - 关键路径按项目日志纪律接入 spdlog（错误节流）。
 *
 * 依赖：FFmpeg dev 包（avcodec/avformat/avutil/swscale）。香橙派需
 * ffmpeg-rockchip（含 rkmpp 编码器）；开发机回退 libx264 软编码。
 */
#include "video_transmission/video_encoder.h"

#include "video_transmission/video_encoder.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <spdlog/spdlog.h>

namespace drone::video_transmission {

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

}  // namespace

/// 编码后端实现细节（PIMPL）：FFmpeg 编码上下文与 RTSP 输出会话。
struct VideoEncoderImpl {
    EncoderBackendConfig config;
    std::atomic<uint64_t> sent_count{0};
    std::atomic<uint64_t> error_count{0};
    std::atomic<bool> running{false};

    // FFmpeg 状态
    AVFormatContext* format_ctx_ = nullptr;   // RTSP 输出封装上下文
    AVCodecContext* codec_ctx_ = nullptr;     // 编码器上下文
    AVStream* stream_ = nullptr;
    AVPacket* packet_ = nullptr;
    AVFrame* nv12_in_ = nullptr;              // NV12 输入帧（复用）
    SwsContext* sws_ctx_ = nullptr;           // NV12 -> YUV420P（软编路径）
    AVFrame* pic_out_ = nullptr;              // 软编路径转换后的 YUV420P 帧（复用）
    const AVCodec* encoder_ = nullptr;
    bool flushed_ = false;
    bool trailer_written_ = false;
    bool is_file_ = false;   // mpegts 文件输出：由封装器自理 SPS/PPS，不手动拼 extradata
    std::int64_t frame_counter_ = 0;  // 单调递增帧序号（编码输入 PTS）

    explicit VideoEncoderImpl(EncoderBackendConfig cfg) : config(std::move(cfg)) {
        if (config.url.empty()) {
            throw std::invalid_argument("图传推送地址 url 不能为空");
        }
        // 实际输出地址：优先 output_url（本地文件调试），否则 url（RTSP 推流）
        if (config.output_url.empty()) {
            config.output_url = config.url;
        }
        if (config.width <= 0 || config.height <= 0) {
            throw std::invalid_argument("图传编码尺寸必须大于 0");
        }
        if (config.fps <= 0) {
            throw std::invalid_argument("图传编码帧率必须大于 0");
        }
    }

    ~VideoEncoderImpl() { Stop(); }

    /// 依据配置与可用性选择编码器名字："h264"/"h265" → rkmpp 或软编码。
    std::string PickEncoderName() const {
        const bool h264 = config.codec == "h264";
        const bool h265 = config.codec == "h265";
        if (!h264 && !h265) {
            throw std::invalid_argument("不支持的编码格式: " + config.codec);
        }
        if (config.prefer_hardware) {
            const char* hw_name = h264 ? "h264_rkmpp" : "hevc_rkmpp";
            if (avcodec_find_encoder_by_name(hw_name) != nullptr) {
                return hw_name;
            }
        }
        // 软编码回退（开发机无 rkmpp 为正常路径）
        return h264 ? "libx264" : "libx265";
    }

    bool Start() {
        if (running.load()) {
            return true;  // 幂等
        }
        try {
            const std::string encoder_name = PickEncoderName();
            encoder_ = avcodec_find_encoder_by_name(encoder_name.c_str());
            if (encoder_ == nullptr) {
                ++error_count;
                SPDLOG_ERROR("图传编码器未找到: {}（需 FFmpeg 5.0+ 或 ffmpeg-rockchip）",
                             encoder_name);
                return false;
            }

            codec_ctx_ = avcodec_alloc_context3(encoder_);
            if (codec_ctx_ == nullptr) {
                ++error_count;
                SPDLOG_ERROR("图传编码器分配上下文失败: {}", encoder_name);
                return false;
            }
            codec_ctx_->codec_id = encoder_->id;
            codec_ctx_->codec_type = AVMEDIA_TYPE_VIDEO;
            codec_ctx_->bit_rate = config.bitrate;
            codec_ctx_->width = static_cast<int>(config.width);
            codec_ctx_->height = static_cast<int>(config.height);
            codec_ctx_->time_base = AVRational{1, config.fps};
            codec_ctx_->framerate = AVRational{config.fps, 1};
            codec_ctx_->gop_size = config.gop;
            codec_ctx_->max_b_frames = 0;
            codec_ctx_->pix_fmt = (encoder_name == "libx264" || encoder_name == "libx265")
                                      ? AV_PIX_FMT_YUV420P
                                      : AV_PIX_FMT_NV12;  // rkmpp 直接收 NV12
            // SPS/PPS 放入 extradata（RTSP SDP 需要）
            codec_ctx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

            const int open_ret = avcodec_open2(codec_ctx_, encoder_, nullptr);
            if (open_ret < 0) {
                ++error_count;
                SPDLOG_ERROR("图传编码器打开失败: {} ({})", encoder_name,
                             AvErrorToString(open_ret));
                return false;
            }

            // 输出上下文（rtsp 推流 / mpegts 本地文件调试）
            is_file_ = (config.output_format == "mpegts");
            const char* fmt_name = is_file_ ? "mpegts" : "rtsp";
            const char* out_url = config.output_url.c_str();
            const int alloc_ret = avformat_alloc_output_context2(
                &format_ctx_, nullptr, fmt_name, out_url);
            if (alloc_ret < 0 || format_ctx_ == nullptr) {
                ++error_count;
                SPDLOG_ERROR("图传创建输出上下文失败: {}", AvErrorToString(alloc_ret));
                return false;
            }
            // 文件输出需显式打开 AVIO；RTSP 由 write_header 内部建立会话
            if (is_file_) {
                if (avio_open(&format_ctx_->pb, out_url, AVIO_FLAG_WRITE) < 0) {
                    ++error_count;
                    SPDLOG_ERROR("图传打开输出文件失败: {}", out_url);
                    return false;
                }
            }
            stream_ = avformat_new_stream(format_ctx_, nullptr);
            if (stream_ == nullptr) {
                ++error_count;
                SPDLOG_ERROR("图传创建输出流失败");
                return false;
            }
            stream_->id = static_cast<int>(format_ctx_->nb_streams) - 1;
            stream_->time_base = codec_ctx_->time_base;
            stream_->codecpar->codec_tag = 0;
            if (avcodec_parameters_from_context(stream_->codecpar, codec_ctx_) < 0) {
                ++error_count;
                SPDLOG_ERROR("图传拷贝编码参数失败");
                return false;
            }

            AVDictionary* opt = nullptr;
            if (!is_file_) {
                av_dict_set(&opt, "rtsp_transport", config.transport.c_str(), 0);
            }
            const int write_ret = avformat_write_header(format_ctx_, &opt);
            av_dict_free(&opt);
            if (write_ret < 0) {
                ++error_count;
                SPDLOG_ERROR("图传 RTSP 写头失败(推流地址不可达?): {}",
                             AvErrorToString(write_ret));
                return false;
            }

            // 输入/中间帧缓冲
            nv12_in_ = av_frame_alloc();
            if (nv12_in_ == nullptr) {
                ++error_count;
                SPDLOG_ERROR("图传分配输入帧失败");
                return false;
            }
            nv12_in_->format = AV_PIX_FMT_NV12;
            nv12_in_->width = static_cast<int>(config.width);
            nv12_in_->height = static_cast<int>(config.height);
            if (av_frame_get_buffer(nv12_in_, 16) < 0) {
                ++error_count;
                SPDLOG_ERROR("图传分配 NV12 输入帧缓冲失败");
                return false;
            }
            // 软编路径：NV12 -> YUV420P（libx264 输入需 YUV420P，需转换 + 独立输出帧）
            if (codec_ctx_->pix_fmt == AV_PIX_FMT_YUV420P) {
                sws_ctx_ = sws_getContext(
                    static_cast<int>(config.width), static_cast<int>(config.height),
                    AV_PIX_FMT_NV12, static_cast<int>(config.width),
                    static_cast<int>(config.height), AV_PIX_FMT_YUV420P,
                    SWS_BILINEAR, nullptr, nullptr, nullptr);
                if (sws_ctx_ == nullptr) {
                    ++error_count;
                    SPDLOG_ERROR("图传创建 NV12→YUV420P 转换上下文失败");
                    return false;
                }
                pic_out_ = av_frame_alloc();
                if (pic_out_ == nullptr) {
                    ++error_count;
                    SPDLOG_ERROR("图传分配 YUV420P 输出帧失败");
                    return false;
                }
                pic_out_->format = AV_PIX_FMT_YUV420P;
                pic_out_->width = static_cast<int>(config.width);
                pic_out_->height = static_cast<int>(config.height);
                if (av_frame_get_buffer(pic_out_, 16) < 0) {
                    ++error_count;
                    SPDLOG_ERROR("图传分配 YUV420P 输出帧缓冲失败");
                    return false;
                }
            }

            packet_ = av_packet_alloc();
            if (packet_ == nullptr) {
                ++error_count;
                SPDLOG_ERROR("图传分配 AVPacket 失败");
                return false;
            }

            running = true;
            SPDLOG_INFO("图传编码器就绪: {} → {}, {}x{}@{}fps, {}bps",
                        encoder_name, config.url, config.width, config.height,
                        config.fps, config.bitrate);
            return true;
        } catch (const std::exception& e) {
            ++error_count;
            SPDLOG_ERROR("图传编码器启动异常: {}", e.what());
            Stop();
            return false;
        }
    }

    /// 取出编码器中所有已就绪的包并推流。
    bool DrainPackets() {
        for (;;) {
            const int ret = avcodec_receive_packet(codec_ctx_, packet_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                return true;
            }
            if (ret < 0) {
                ++error_count;
                if (ShouldLogThrottled(error_count)) {
                    SPDLOG_ERROR("图传接收编码包失败: {}，累计 {}", AvErrorToString(ret),
                                 error_count.load());
                }
                return false;
            }
            if (!WritePacket()) {
                return false;
            }
        }
    }

    /// 编码输出的包做时间戳换算、关键帧补 SPS/PPS 后写出。
    bool WritePacket() {
        av_packet_rescale_ts(packet_, codec_ctx_->time_base, stream_->time_base);
        packet_->stream_index = stream_->index;

        // 关键帧前拼接 SPS/PPS（extradata），提高 RTSP 播放器中途接入兼容性；
        // 文件/mpegts 输出由封装器自理 SPS/PPS，不做手动拼接（避免破坏码流）。
        static thread_local std::vector<uint8_t> keyframe_buffer_;
        const AVCodecParameters* par = stream_->codecpar;
        const uint8_t* extra = par->extradata;
        const int extra_size = par->extradata_size;
        if (!is_file_ && (packet_->flags & AV_PKT_FLAG_KEY) && extra != nullptr &&
            extra_size > 0) {
            const size_t total = static_cast<size_t>(extra_size) + packet_->size;
            if (keyframe_buffer_.size() < total) {
                keyframe_buffer_.resize(total);
            }
            std::memcpy(keyframe_buffer_.data(), extra, extra_size);
            std::memcpy(keyframe_buffer_.data() + extra_size, packet_->data,
                        packet_->size);
            packet_->data = keyframe_buffer_.data();
            packet_->size = static_cast<int>(total);
        }

        const int ret = av_interleaved_write_frame(format_ctx_, packet_);
        av_packet_unref(packet_);
        if (ret < 0) {
            ++error_count;
            if (ShouldLogThrottled(error_count)) {
                SPDLOG_ERROR("图传推流写包失败: {}，累计 {}", AvErrorToString(ret),
                             error_count.load());
            }
            return false;
        }
        ++sent_count;
        return true;
    }

    bool EncodeFrame(const video::FrameHandle& frame) {
        if (!running.load()) {
            ++error_count;
            return false;
        }
        const video::VideoFrameInfo& info = frame.Info();
        if (!frame.Valid() || info.format != video::PixelFormat::kYuv420SpNv12) {
            ++error_count;
            if (ShouldLogThrottled(error_count)) {
                SPDLOG_ERROR("图传输入帧非法(格式/句柄无效)，累计 {}",
                             error_count.load());
            }
            return false;
        }

        // 将 NV12 FrameHandle 像素拷入编码输入帧
        const std::uint32_t w = static_cast<std::uint32_t>(codec_ctx_->width);
        const std::uint32_t h = static_cast<std::uint32_t>(codec_ctx_->height);
        if (info.width != w || info.height != h) {
            ++error_count;
            if (ShouldLogThrottled(error_count)) {
                SPDLOG_ERROR("图传输入尺寸不匹配: 编码={}x{} 输入={}x{}，累计 {}",
                             w, h, info.width, info.height, error_count.load());
            }
            return false;
        }
        const std::byte* src = frame.Data();
        if (src == nullptr) {
            ++error_count;
            return false;
        }
        // Y 平面：按行（源 stride 可能大于编码 width）
        const int src_stride = static_cast<int>(info.hor_stride);
        const int out_stride_y = nv12_in_->linesize[0];
        {
            uint8_t* dst_y = nv12_in_->data[0];
            const uint8_t* src_y = reinterpret_cast<const uint8_t*>(src);
            for (std::uint32_t row = 0; row < h; ++row) {
                std::memcpy(dst_y + static_cast<size_t>(row) * out_stride_y,
                            src_y + static_cast<size_t>(row) * src_stride, w);
            }
        }
        // UV 平面：每行 2 字节/像素，行数 h/2，源 UV 偏移 = 源 stride*h
        const std::uint32_t uv_rows = h / 2;
        {
            const uint8_t* src_uv = reinterpret_cast<const uint8_t*>(
                src + static_cast<size_t>(src_stride) * h);
            uint8_t* dst_uv = nv12_in_->data[1];
            const int out_stride_uv = nv12_in_->linesize[1];
            for (std::uint32_t row = 0; row < uv_rows; ++row) {
                std::memcpy(dst_uv + static_cast<size_t>(row) * out_stride_uv,
                            src_uv + static_cast<size_t>(row) * src_stride, w);
            }
        }

        nv12_in_->pts = frame_counter_;

        // 软编路径：NV12 → YUV420P 再送编码器（libx264 不接受 NV12）
        AVFrame* encode_input = nv12_in_;
        if (pic_out_ != nullptr) {
            pic_out_->pts = nv12_in_->pts;
            const uint8_t* src_planes[2] = {nv12_in_->data[0], nv12_in_->data[1]};
            const int src_linesize[2] = {nv12_in_->linesize[0], nv12_in_->linesize[1]};
            sws_scale(sws_ctx_, src_planes, src_linesize, 0, static_cast<int>(h),
                      pic_out_->data, pic_out_->linesize);
            encode_input = pic_out_;
        }

        const int send_ret = avcodec_send_frame(codec_ctx_, encode_input);
        if (send_ret < 0) {
            ++error_count;
            if (ShouldLogThrottled(error_count)) {
                SPDLOG_ERROR("图传送帧编码失败: {}，累计 {}", AvErrorToString(send_ret),
                             error_count.load());
            }
            return false;
        }
        ++frame_counter_;
        return DrainPackets();
    }

    void Stop() {
        if (!running.load()) {
            return;
        }
        running = false;
        // 冲刷编码器尾帧
        if (codec_ctx_ != nullptr) {
            avcodec_send_frame(codec_ctx_, nullptr);
            DrainPackets();
        }
        if (format_ctx_ != nullptr && !trailer_written_) {
            if (format_ctx_->pb != nullptr) {
                av_write_trailer(format_ctx_);
            }
            trailer_written_ = true;
        }
        if (packet_ != nullptr) {
            av_packet_free(&packet_);
            packet_ = nullptr;
        }
        if (sws_ctx_ != nullptr) {
            sws_freeContext(sws_ctx_);
            sws_ctx_ = nullptr;
        }
        if (nv12_in_ != nullptr) {
            av_frame_free(&nv12_in_);
            nv12_in_ = nullptr;
        }
        if (pic_out_ != nullptr) {
            av_frame_free(&pic_out_);
            pic_out_ = nullptr;
        }
        if (format_ctx_ != nullptr) {
            if (format_ctx_->pb != nullptr) {
                avio_close(format_ctx_->pb);
                format_ctx_->pb = nullptr;
            }
            avformat_free_context(format_ctx_);
            format_ctx_ = nullptr;
            stream_ = nullptr;
        }
        if (codec_ctx_ != nullptr) {
            avcodec_free_context(&codec_ctx_);
            codec_ctx_ = nullptr;
        }
        encoder_ = nullptr;
        flushed_ = false;
        trailer_written_ = false;
        SPDLOG_INFO("图传编码器关闭");
    }
};

/// 具体编码后端实现（PIMPL 持有者）。
class FfmpegEncoderBackend final : public IVideoEncoderBackend {
public:
    explicit FfmpegEncoderBackend(EncoderBackendConfig config)
        : impl_(std::make_unique<VideoEncoderImpl>(std::move(config))) {}
    ~FfmpegEncoderBackend() override = default;

    bool Start() override { return impl_->Start(); }
    void Stop() override { impl_->Stop(); }
    bool IsRunning() const override { return impl_->running.load(); }

    bool EncodeFrame(const video::FrameHandle& frame) override {
        return impl_->EncodeFrame(frame);
    }

    std::uint64_t SentFrameCount() const override {
        return impl_->sent_count.load();
    }
    std::uint64_t ErrorCount() const override {
        return impl_->error_count.load();
    }

private:
    std::unique_ptr<VideoEncoderImpl> impl_;
};

/// 工厂：创建默认 FFmpeg 编码后端。
std::unique_ptr<IVideoEncoderBackend> CreateVideoEncoderBackend(
    EncoderBackendConfig config) {
    return std::make_unique<FfmpegEncoderBackend>(std::move(config));
}

}  // namespace drone::video_transmission
