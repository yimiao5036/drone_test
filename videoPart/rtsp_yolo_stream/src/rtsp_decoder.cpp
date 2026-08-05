#include "rtsp_decoder.h"
#include "interrupt_flag.h"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/hwcontext.h>
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
// 实现细节（PIMPL）：FFmpeg 拉流 + rkmpp 硬解码全部封装在这里
// ------------------------------------------------------------
struct RtspDecoder::Impl {
    AVFormatContext* format_ctx_ = nullptr;   // 封装格式上下文（RTSP 输入）
    AVCodecContext* codec_ctx_ = nullptr;     // 解码器上下文
    AVBufferRef* hw_device_ = nullptr;        // DRM 硬件设备
    SwsContext* sws_ctx_ = nullptr;           // NV12 -> BGR 转换器
    AVFrame* frame_ = nullptr;                // 解码输出帧（通常为 DRM_PRIME）
    AVFrame* sw_frame_ = nullptr;             // 从硬件内存转出的软件帧（NV12）
    AVPacket* packet_ = nullptr;

    int video_index_ = -1;
    int width_ = 0;
    int height_ = 0;
    int fps_ = 25;
    bool flushed_ = false;        // 流读取结束后是否已发送 flush 包
    bool have_pending_ = false;   // 是否有尚未成功送入解码器的包
    int sws_src_format_ = -1;     // 当前 sws 上下文对应的源格式（用于缓存）

    explicit Impl(const std::string& rtsp_url) {
        avformat_network_init();

        // RTSP 选项
        AVDictionary* options = nullptr;
        av_dict_set(&options, "rtsp_transport", "tcp", 0);
        av_dict_set(&options, "fflags", "nobuffer+discardcorrupt", 0);
        av_dict_set(&options, "stimeout", "5000000", 0);
        av_dict_set(&options, "reorder_queue_size", "0", 0);     // 禁用 RTP 重排缓冲
        av_dict_set(&options, "max_delay", "0", 0);               // 无额外延迟
        av_dict_set(&options, "probesize", "50000", 0);           // 限制探测数据量
        av_dict_set(&options, "analyzeduration", "100000", 0);    // 限制探测时间 100ms
        av_dict_set(&options, "buffer_size", "102400", 0);        // 较小的 TCP 缓冲

        // 手动分配上下文并挂接中断回调：
        // 看门狗或 Ctrl+C 置位 g_net_interrupt 后，阻塞的 av_read_frame 会尽快返回，避免僵死
        format_ctx_ = avformat_alloc_context();
        if (format_ctx_ == nullptr) {
            throw std::runtime_error("分配输入上下文失败");
        }
        format_ctx_->interrupt_callback.callback = rtsp_stream::NetInterruptCallback;
        format_ctx_->interrupt_callback.opaque = nullptr;

        int ret = avformat_open_input(&format_ctx_, rtsp_url.c_str(), nullptr, &options);
        av_dict_free(&options);
        if (ret < 0) {
            throw std::runtime_error("打开 RTSP 流失败: " + rtsp_url + " (" + AvErrorToString(ret) + ")");
        }

        ret = avformat_find_stream_info(format_ctx_, nullptr);
        if (ret < 0) {
            throw std::runtime_error("获取流信息失败: " + AvErrorToString(ret));
        }

        // 查找第一个视频流
        for (unsigned int i = 0; i < format_ctx_->nb_streams; ++i) {
            if (format_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                video_index_ = static_cast<int>(i);
                break;
            }
        }
        if (video_index_ < 0) {
            throw std::runtime_error("RTSP 流中未找到视频流");
        }

        AVStream* stream = format_ctx_->streams[video_index_];
        AVCodecParameters* codecpar = stream->codecpar;

        // 优先使用 Rockchip 硬解码器，找不到时回退到 FFmpeg 通用解码器
        const AVCodec* decoder = nullptr;
        switch (codecpar->codec_id) {
        case AV_CODEC_ID_HEVC:
            decoder = avcodec_find_decoder_by_name("hevc_rkmpp");
            break;
        case AV_CODEC_ID_H264:
            decoder = avcodec_find_decoder_by_name("h264_rkmpp");
            break;
        default:
            break;
        }
        if (decoder == nullptr) {
            decoder = avcodec_find_decoder(codecpar->codec_id);
        }
        if (decoder == nullptr) {
            throw std::runtime_error("未找到对应的解码器（请确认 FFmpeg 已启用 --enable-rkmpp）");
        }
        printf("[RtspDecoder] 使用解码器: %s\n", decoder->name);

        codec_ctx_ = avcodec_alloc_context3(decoder);
        if (codec_ctx_ == nullptr) {
            throw std::runtime_error("创建解码器上下文失败");
        }
        ret = avcodec_parameters_to_context(codec_ctx_, codecpar);
        if (ret < 0) {
            throw std::runtime_error("拷贝解码参数失败: " + AvErrorToString(ret));
        }
        codec_ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY; // 解码器低延迟模式

        // 创建 DRM 硬件设备并绑定到解码器（RK3588 硬解码路径）
        hw_device_ = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_DRM);
        if (hw_device_ == nullptr) {
            throw std::runtime_error("分配 DRM 硬件设备失败");
        }
        ret = av_hwdevice_ctx_init(hw_device_);
        if (ret < 0) {
            throw std::runtime_error("初始化 DRM 硬件设备失败: " + AvErrorToString(ret));
        }
        codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_);

        ret = avcodec_open2(codec_ctx_, decoder, nullptr);
        if (ret < 0) {
            throw std::runtime_error("打开解码器失败: " + AvErrorToString(ret));
        }

        width_ = codec_ctx_->width;
        height_ = codec_ctx_->height;

        // 估算帧率：优先 avg_frame_rate，其次 r_frame_rate
        fps_ = EstimateFps(stream);

        frame_ = av_frame_alloc();
        sw_frame_ = av_frame_alloc();
        packet_ = av_packet_alloc();
        if (frame_ == nullptr || sw_frame_ == nullptr || packet_ == nullptr) {
            throw std::runtime_error("分配 AVFrame/AVPacket 失败");
        }

        printf("[RtspDecoder] 分辨率 %dx%d, 帧率约 %d fps\n", width_, height_, fps_);
    }

    ~Impl() {
        if (sws_ctx_ != nullptr) {
            sws_freeContext(sws_ctx_);
        }
        if (frame_ != nullptr) {
            av_frame_free(&frame_);
        }
        if (sw_frame_ != nullptr) {
            av_frame_free(&sw_frame_);
        }
        if (packet_ != nullptr) {
            av_packet_free(&packet_);
        }
        if (codec_ctx_ != nullptr) {
            avcodec_free_context(&codec_ctx_);
        }
        if (hw_device_ != nullptr) {
            av_buffer_unref(&hw_device_);
        }
        if (format_ctx_ != nullptr) {
            avformat_close_input(&format_ctx_);
        }
    }

    // 从流信息估算帧率
    static int EstimateFps(const AVStream* stream) {
        AVRational rate = stream->avg_frame_rate;
        if (rate.num <= 0 || rate.den <= 0) {
            rate = stream->r_frame_rate;
        }
        if (rate.num <= 0 || rate.den <= 0) {
            return 25;
        }
        int fps = static_cast<int>(std::lround(av_q2d(rate)));
        return fps > 0 ? fps : 25;
    }

    // 向解码器送入一个视频包，返回 false 表示流已彻底结束且无缓存帧
    bool FeedOnePacket() {
        // 上次送包因解码器缓冲区满而失败，先重试
        if (have_pending_) {
            int ret = avcodec_send_packet(codec_ctx_, packet_);
            if (ret == AVERROR(EAGAIN)) {
                return true; // 依然送不进去，先去取解码帧
            }
            av_packet_unref(packet_);
            have_pending_ = false;
            return true;
        }

        // 流已读完，处于 flush 收尾阶段
        if (flushed_) {
            return true;
        }

        while (av_read_frame(format_ctx_, packet_) >= 0) {
            if (packet_->stream_index != video_index_) {
                av_packet_unref(packet_);
                continue;
            }
            int ret = avcodec_send_packet(codec_ctx_, packet_);
            if (ret == AVERROR(EAGAIN)) {
                have_pending_ = true; // 保留该包，下一轮重试
                return true;
            }
            av_packet_unref(packet_);
            if (ret < 0) {
                fprintf(stderr, "[RtspDecoder] 送包解码失败: %s\n", AvErrorToString(ret).c_str());
                continue;
            }
            return true;
        }

        // 读流结束：发送空包让解码器输出剩余缓存帧
        avcodec_send_packet(codec_ctx_, nullptr);
        flushed_ = true;
        return true;
    }

    // 将解码帧（可能是 DRM_PRIME 硬件帧）转换为 BGR cv::Mat
    bool ConvertToBgr(AVFrame* frame, cv::Mat& bgr) {
        AVFrame* src = frame;

        // 硬件帧：先从 DRM 内存转存到系统内存（输出为 NV12）
        if (frame->format == AV_PIX_FMT_DRM_PRIME) {
            int ret = av_hwframe_transfer_data(sw_frame_, frame, 0);
            if (ret < 0) {
                fprintf(stderr, "[RtspDecoder] 硬件帧转存失败: %s\n", AvErrorToString(ret).c_str());
                return false;
            }
            src = sw_frame_;
        }

        // 源格式变化时重建转换上下文
        if (sws_ctx_ == nullptr || sws_src_format_ != src->format) {
            if (sws_ctx_ != nullptr) {
                sws_freeContext(sws_ctx_);
            }
            sws_ctx_ = sws_getContext(width_, height_, static_cast<AVPixelFormat>(src->format),
                                      width_, height_, AV_PIX_FMT_BGR24,
                                      SWS_BILINEAR, nullptr, nullptr, nullptr);
            if (sws_ctx_ == nullptr) {
                fprintf(stderr, "[RtspDecoder] 创建 sws 转换上下文失败(源格式=%d)\n", src->format);
                return false;
            }
            sws_src_format_ = src->format;
        }

        bgr.create(height_, width_, CV_8UC3);
        uint8_t* dst_data[1] = { bgr.data };
        int dst_linesize[1] = { static_cast<int>(bgr.step[0]) };
        sws_scale(sws_ctx_, src->data, src->linesize, 0, height_, dst_data, dst_linesize);
        return true;
    }
};

RtspDecoder::RtspDecoder(const std::string& rtsp_url)
    : impl_(std::make_unique<Impl>(rtsp_url)) {}

RtspDecoder::~RtspDecoder() = default;

bool RtspDecoder::ReadFrame(cv::Mat& bgr_frame) {
    while (true) {
        // 先尝试从解码器取帧
        int ret = avcodec_receive_frame(impl_->codec_ctx_, impl_->frame_);
        if (ret == 0) {
            return impl_->ConvertToBgr(impl_->frame_, bgr_frame);
        }
        if (ret == AVERROR_EOF) {
            return false; // 解码器已输出完所有缓存帧
        }
        if (ret != AVERROR(EAGAIN)) {
            fprintf(stderr, "[RtspDecoder] 接收解码帧失败: %s\n", AvErrorToString(ret).c_str());
            return false;
        }
        // EAGAIN：需要再送入数据
        if (!impl_->FeedOnePacket()) {
            return false;
        }
    }
}

int RtspDecoder::width() const { return impl_->width_; }
int RtspDecoder::height() const { return impl_->height_; }
int RtspDecoder::fps() const { return impl_->fps_; }
