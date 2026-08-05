#include "rtsp_encoder.h"
#include "interrupt_flag.h"

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace {

// 将 FFmpeg 错误码转换为可读字符串
std::string AvErrorToString(int errnum) {
    char buf[AV_ERROR_MAX_STRING_SIZE] = { 0 };
    av_make_error_string(buf, sizeof(buf), errnum);
    return std::string(buf);
}

} // namespace

// ------------------------------------------------------------
// 实现细节（PIMPL）
//
// 依赖官方 FFmpeg 5.0+（或社区维护的 nyanmisaka/ffmpeg-rockchip）中的
// rkmpp 编码器（rkmppenc.c）：该编码器可直接接收普通 NV12 软件帧，
// 内部自动导入 MPP 缓冲，因此本实现无需手工分配 DRM/MPP 内存。
// 注意：香橙派镜像自带的 FFmpeg 4.4 只有 rkmpp 解码器、没有编码器，
// 不满足本工程要求。
// ------------------------------------------------------------
struct RtspEncoder::Impl {
    std::string url_;
    int width_ = 0;
    int height_ = 0;
    int fps_ = 25;

    AVFormatContext* format_ctx_ = nullptr;   // RTSP 输出封装上下文
    AVCodecContext* codec_ctx_ = nullptr;     // rkmpp 硬编码器上下文
    AVStream* stream_ = nullptr;
    AVPacket* packet_ = nullptr;
    AVFrame* nv12_frame_ = nullptr;           // BGR 转换后的 NV12 软件帧（复用）
    SwsContext* sws_ctx_ = nullptr;           // BGR -> NV12 转换器

    std::vector<uint8_t> keyframe_buffer_;    // 关键帧拼接 SPS/PPS 的复用缓冲
    int64_t frame_count_ = 0;
    bool flushed_ = false;
    bool trailer_written_ = false;

    Impl(const std::string& url, int width, int height, int fps, const std::string& codec_name)
        : url_(url), width_(width), height_(height), fps_(fps) {
        avformat_network_init();

        // 1. 查找 rkmpp 硬编码器
        const AVCodec* encoder = avcodec_find_encoder_by_name(codec_name.c_str());
        if (encoder == nullptr) {
            throw std::runtime_error("未找到编码器 " + codec_name +
                                     "（需要 FFmpeg 5.0+ 或 ffmpeg-rockchip，"
                                     "香橙派镜像自带的 FFmpeg 4.4 不含 rkmpp 编码器）");
        }

        codec_ctx_ = avcodec_alloc_context3(encoder);
        if (codec_ctx_ == nullptr) {
            throw std::runtime_error("创建编码器上下文失败");
        }
        codec_ctx_->codec_id = encoder->id;
        codec_ctx_->codec_type = AVMEDIA_TYPE_VIDEO;
        codec_ctx_->bit_rate = 8 * 1024 * 1024;         // 8 Mbps
        codec_ctx_->width = width_;
        codec_ctx_->height = height_;
        codec_ctx_->time_base = AVRational{ 1, fps_ };
        codec_ctx_->framerate = AVRational{ fps_, 1 };
        codec_ctx_->gop_size = fps_ * 2;                // 每 2 秒一个关键帧
        codec_ctx_->max_b_frames = 0;
        codec_ctx_->pix_fmt = AV_PIX_FMT_NV12;          // 官方 rkmppenc 直接接收 NV12 软件帧
        codec_ctx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER; // SPS/PPS 放入 extradata（RTSP SDP 需要）

        // 2. 打开编码器（必须在 write_header 之前，保证 extradata 已生成）
        int ret = avcodec_open2(codec_ctx_, encoder, nullptr);
        if (ret < 0) {
            throw std::runtime_error("打开编码器失败: " + AvErrorToString(ret));
        }

        // 3. 创建 RTSP 输出上下文与视频流
        ret = avformat_alloc_output_context2(&format_ctx_, nullptr, "rtsp", url_.c_str());
        if (ret < 0 || format_ctx_ == nullptr) {
            throw std::runtime_error("创建 RTSP 输出上下文失败: " + AvErrorToString(ret));
        }
        // 挂接中断回调：看门狗或 Ctrl+C 置位 g_net_interrupt 后，
        // 阻塞的 av_interleaved_write_frame / avformat_write_header 会尽快返回，避免僵死
        format_ctx_->interrupt_callback.callback = rtsp_stream::NetInterruptCallback;
        format_ctx_->interrupt_callback.opaque = nullptr;
        stream_ = avformat_new_stream(format_ctx_, nullptr);
        if (stream_ == nullptr) {
            throw std::runtime_error("创建输出流失败");
        }
        stream_->id = static_cast<int>(format_ctx_->nb_streams) - 1;
        stream_->time_base = codec_ctx_->time_base;
        stream_->codecpar->codec_tag = 0;
        ret = avcodec_parameters_from_context(stream_->codecpar, codec_ctx_);
        if (ret < 0) {
            throw std::runtime_error("拷贝编码参数失败: " + AvErrorToString(ret));
        }

        // 4. 写入 RTSP 头（建立 ANNOUNCE/SETUP 会话）
        AVDictionary* opt = nullptr;
        av_dict_set(&opt, "rtsp_transport", "tcp", 0);
        ret = avformat_write_header(format_ctx_, &opt);
        av_dict_free(&opt);
        if (ret < 0) {
            throw std::runtime_error("RTSP 写头失败(推流地址不可达?): " + AvErrorToString(ret));
        }

        // 5. BGR -> NV12 转换器与复用中间帧
        sws_ctx_ = sws_getContext(width_, height_, AV_PIX_FMT_BGR24,
                                  width_, height_, AV_PIX_FMT_NV12,
                                  SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (sws_ctx_ == nullptr) {
            throw std::runtime_error("创建 sws 转换上下文失败");
        }
        nv12_frame_ = av_frame_alloc();
        if (nv12_frame_ == nullptr) {
            throw std::runtime_error("分配 AVFrame 失败");
        }
        nv12_frame_->format = AV_PIX_FMT_NV12;
        nv12_frame_->width = width_;
        nv12_frame_->height = height_;
        if (av_frame_get_buffer(nv12_frame_, 16) < 0) {
            throw std::runtime_error("分配 NV12 缓冲失败");
        }

        packet_ = av_packet_alloc();
        if (packet_ == nullptr) {
            throw std::runtime_error("分配 AVPacket 失败");
        }

        printf("[RtspEncoder] %s -> %s, %dx%d@%dfps\n",
               codec_name.c_str(), url_.c_str(), width_, height_, fps_);
    }

    ~Impl() {
        Flush();
        if (packet_ != nullptr) {
            av_packet_free(&packet_);
        }
        if (nv12_frame_ != nullptr) {
            av_frame_free(&nv12_frame_);
        }
        if (sws_ctx_ != nullptr) {
            sws_freeContext(sws_ctx_);
        }
        if (format_ctx_ != nullptr) {
            if (format_ctx_->pb != nullptr) {
                avio_close(format_ctx_->pb);
            }
            avformat_free_context(format_ctx_);
        }
        if (codec_ctx_ != nullptr) {
            avcodec_free_context(&codec_ctx_);
        }
    }

    // 编码输出的包做时间戳换算、关键帧补 SPS/PPS 后写出
    bool WritePacket() {
        av_packet_rescale_ts(packet_, codec_ctx_->time_base, stream_->time_base);
        packet_->stream_index = stream_->index;

        // 关键帧前拼接 SPS/PPS（extradata），提高播放器中途接入的兼容性
        if ((packet_->flags & AV_PKT_FLAG_KEY) &&
            codec_ctx_->extradata != nullptr && codec_ctx_->extradata_size > 0) {
            size_t total = static_cast<size_t>(codec_ctx_->extradata_size) + packet_->size;
            if (keyframe_buffer_.size() < total) {
                keyframe_buffer_.resize(total);
            }
            memcpy(keyframe_buffer_.data(), codec_ctx_->extradata, codec_ctx_->extradata_size);
            memcpy(keyframe_buffer_.data() + codec_ctx_->extradata_size, packet_->data, packet_->size);
            packet_->data = keyframe_buffer_.data();
            packet_->size = static_cast<int>(total);
        }

        int ret = av_interleaved_write_frame(format_ctx_, packet_);
        av_packet_unref(packet_);
        if (ret < 0) {
            fprintf(stderr, "[RtspEncoder] 推流写包失败: %s\n", AvErrorToString(ret).c_str());
            return false;
        }
        return true;
    }

    // 取出编码器中所有已就绪的包并推流
    bool DrainPackets() {
        while (true) {
            int ret = avcodec_receive_packet(codec_ctx_, packet_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                return true;
            }
            if (ret < 0) {
                fprintf(stderr, "[RtspEncoder] 接收编码包失败: %s\n", AvErrorToString(ret).c_str());
                return false;
            }
            if (!WritePacket()) {
                return false;
            }
        }
    }

    void Flush() {
        if (flushed_ || codec_ctx_ == nullptr) {
            return;
        }
        flushed_ = true;
        // 送入空帧让编码器输出剩余缓存帧
        avcodec_send_frame(codec_ctx_, nullptr);
        DrainPackets();
        if (!trailer_written_ && format_ctx_ != nullptr) {
            av_write_trailer(format_ctx_);
            trailer_written_ = true;
        }
    }
};

RtspEncoder::RtspEncoder(const std::string& rtsp_url, int width, int height, int fps,
                         const std::string& codec_name)
    : impl_(std::make_unique<Impl>(rtsp_url, width, height, fps, codec_name)) {}

RtspEncoder::~RtspEncoder() = default;

bool RtspEncoder::WriteFrame(const cv::Mat& bgr_frame) {
    if (bgr_frame.empty() || bgr_frame.type() != CV_8UC3 ||
        bgr_frame.cols != impl_->width_ || bgr_frame.rows != impl_->height_) {
        fprintf(stderr, "[RtspEncoder] 输入帧尺寸/格式不匹配\n");
        return false;
    }

    // 1. BGR -> NV12（复用同一帧缓冲，rkmppenc 内部会自行导入 MPP 缓冲）
    const uint8_t* src_data[1] = { bgr_frame.data };
    int src_linesize[1] = { static_cast<int>(bgr_frame.step[0]) };
    sws_scale(impl_->sws_ctx_, src_data, src_linesize, 0, impl_->height_,
              impl_->nv12_frame_->data, impl_->nv12_frame_->linesize);
    impl_->nv12_frame_->pts = impl_->frame_count_;

    // 2. 送帧编码
    int ret = avcodec_send_frame(impl_->codec_ctx_, impl_->nv12_frame_);
    if (ret < 0) {
        fprintf(stderr, "[RtspEncoder] 送帧编码失败: %s\n", AvErrorToString(ret).c_str());
        return false;
    }
    impl_->frame_count_++;

    // 3. 取出编码结果并推流
    return impl_->DrainPackets();
}

void RtspEncoder::Flush() { impl_->Flush(); }
