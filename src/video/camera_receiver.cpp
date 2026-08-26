/**
 * @file camera_receiver.cpp
 * @brief RTSP 接收器实现（CameraReceiver）
 *
 * 使用 FFmpeg avformat 拉取 RTSP 流（TCP/UDP），逐访问单元读取
 * H.264/H.265 码流并发布到输出主题；断流自动重连。
 *
 * 设计要点：
 * - PIMPL 隔离 FFmpeg 头文件，接口头文件不依赖 FFmpeg。
 * - 中断回调（opaque=this）检查停止标志，保证 Stop() 能尽快打断
 *   阻塞中的 av_read_frame，确定性停机。
 * - 重连间隔可配置；建连失败/断流按 WARN 节流记录，逐帧热路径不打日志。
 */
#include "video/camera_receiver.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

extern "C" {
#include <libavformat/avformat.h>
}

#include <spdlog/spdlog.h>

namespace drone::video {

namespace {

/// 单调时钟毫秒（与消息头时间戳约定一致）。
std::uint64_t SteadyNowMs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

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

/// CameraReceiver 实现细节（PIMPL）：FFmpeg 上下文与接收线程。
struct CameraReceiver::Impl {
    explicit Impl(CameraReceiverConfig config) : config(std::move(config)) {
        if (this->config.rtsp_url.empty()) {
            throw std::invalid_argument("RTSP 地址不能为空");
        }
        if (this->config.reconnect_delay.count() < 0) {
            throw std::invalid_argument("重连间隔不能为负");
        }
    }

    ~Impl() { Stop(); }

    CameraReceiverConfig config;
    std::atomic<bool> stop_requested{false};
    std::thread thread;

    AVFormatContext* format_ctx = nullptr;
    int video_index = -1;
    common::VideoCodec codec = common::VideoCodec::kUnknown;

    // 状态计数（原子，跨线程可读）
    std::atomic<bool> connected{false};
    std::atomic<uint64_t> connect_count{0};
    std::atomic<uint64_t> received_bytes{0};
    std::atomic<uint64_t> error_count{0};
    std::atomic<uint64_t> sequence{0};

    common::Topic<common::EncodedFrame> stream_output;

    /// avformat 中断回调：停止请求时返回 1，打断阻塞中的网络读写。
    static int InterruptCallback(void* opaque) {
        const auto* self = static_cast<Impl*>(opaque);
        return self->stop_requested.load() ? 1 : 0;
    }

    /// 打开 RTSP 流并完成探测；失败返回 false（已记录错误日志）。
    bool OpenStream() {
        if (stop_requested.load()) {
            return false;
        }

        AVDictionary* options = nullptr;
        av_dict_set(&options, "rtsp_transport", config.rtsp_transport.c_str(), 0);
        // 以下选项与已验证成功拉流的原型 videoPart/rtsp_yolo_stream/rtsp_decoder.cpp
        // 保持一致：nobuffer+discardcorrupt（低延迟+丢坏包）、禁用 RTP 重排（防 SPS/PPS
        // 与 IDR 顺序打乱导致 rkmpp 收不到参数集）、无额外延迟、小 TCP 缓冲、临时
        // stimeout。这些是实测可稳定解析本摄像头 H265 流的组合。
        av_dict_set(&options, "fflags", "nobuffer+discardcorrupt", 0);
        av_dict_set(&options, "reorder_queue_size", "0", 0);
        av_dict_set(&options, "max_delay", "0", 0);
        av_dict_set(&options, "probesize", "50000", 0);
        av_dict_set(&options, "analyzeduration", "100000", 0);
        av_dict_set(&options, "buffer_size", "102400", 0);
        const std::string timeout_us =
            std::to_string(config.open_timeout.count() * 1000);
        av_dict_set(&options, "stimeout", timeout_us.c_str(), 0);

        format_ctx = avformat_alloc_context();
        if (format_ctx == nullptr) {
            ++error_count;
            SPDLOG_ERROR("摄像头接收器分配输入上下文失败: url={}", config.rtsp_url);
            return false;
        }
        format_ctx->interrupt_callback.callback = InterruptCallback;
        format_ctx->interrupt_callback.opaque = this;

        const int open_ret =
            avformat_open_input(&format_ctx, config.rtsp_url.c_str(), nullptr, &options);
        av_dict_free(&options);
        if (open_ret < 0) {
            ++error_count;
            if (ShouldLogThrottled(error_count)) {
                SPDLOG_WARN("摄像头接收器打开 RTSP 失败: url={} ({})，累计 {}",
                            config.rtsp_url, AvErrorToString(open_ret), error_count.load());
            }
            avformat_close_input(&format_ctx);
            format_ctx = nullptr;
            return false;
        }

        const int info_ret = avformat_find_stream_info(format_ctx, nullptr);
        if (info_ret < 0) {
            ++error_count;
            SPDLOG_ERROR("摄像头接收器获取流信息失败: url={} ({})", config.rtsp_url,
                         AvErrorToString(info_ret));
            avformat_close_input(&format_ctx);
            format_ctx = nullptr;
            return false;
        }

        video_index = -1;
        codec = common::VideoCodec::kUnknown;
        for (unsigned int index = 0; index < format_ctx->nb_streams; ++index) {
            const auto* par = format_ctx->streams[index]->codecpar;
            if (par->codec_type == AVMEDIA_TYPE_VIDEO) {
                video_index = static_cast<int>(index);
                if (par->codec_id == AV_CODEC_ID_HEVC) {
                    codec = common::VideoCodec::kH265;
                } else if (par->codec_id == AV_CODEC_ID_H264) {
                    codec = common::VideoCodec::kH264;
                }
                break;
            }
        }
        if (video_index < 0) {
            ++error_count;
            SPDLOG_ERROR("摄像头接收器流中未找到视频流: url={}", config.rtsp_url);
            avformat_close_input(&format_ctx);
            format_ctx = nullptr;
            return false;
        }

        ++connect_count;
        connected = true;
        SPDLOG_INFO("摄像头接收器已连接: url={} 编码={} 分辨率={}x{} 连接次数={}",
                    config.rtsp_url, codec == common::VideoCodec::kH265 ? "H.265" : "H.264",
                    format_ctx->streams[video_index]->codecpar->width,
                    format_ctx->streams[video_index]->codecpar->height,
                    connect_count.load());

        // 流级参数集（RTSP 从 SDP/容器头提取，如 SPS/PPS）：单独发布一条
        // 仅含参数集的消息（data 为空），供解码器在首帧前完成初始化。
        const auto* par = format_ctx->streams[video_index]->codecpar;
        if (par->extradata != nullptr && par->extradata_size > 0) {
            common::EncodedFrame parameter_frame;
            parameter_frame.header.sequence = sequence.fetch_add(1) + 1;
            parameter_frame.header.receive_time_ms = SteadyNowMs();
            parameter_frame.codec = codec;
            parameter_frame.stream_sequence = parameter_frame.header.sequence;
            parameter_frame.parameter_sets.assign(
                par->extradata, par->extradata + par->extradata_size);
            (void)stream_output.Emplace(std::move(parameter_frame));
        }
        return true;
    }

    /// 关闭当前连接（幂等）。
    void CloseStream() {
        if (format_ctx != nullptr) {
            avformat_close_input(&format_ctx);
            format_ctx = nullptr;
        }
        connected = false;
    }

    /// 接收线程主循环：建连 → 读包发布 → 断流重连。
    void ReceiveLoop() {
        while (!stop_requested.load()) {
            if (!OpenStream()) {
                if (!stop_requested.load()) {
                    std::this_thread::sleep_for(config.reconnect_delay);
                }
                continue;
            }

            AVPacket* packet = av_packet_alloc();
            bool stream_ended = false;
            while (!stop_requested.load() && !stream_ended) {
                av_packet_unref(packet);
                const int read_ret = av_read_frame(format_ctx, packet);
                if (read_ret < 0) {
                    // 断流或停止中断；非正常结束记一次错误并节流
                    if (!stop_requested.load()) {
                        ++error_count;
                        if (ShouldLogThrottled(error_count)) {
                            SPDLOG_WARN("摄像头接收器断流: url={} ({})，累计 {}，准备重连",
                                        config.rtsp_url, AvErrorToString(read_ret),
                                        error_count.load());
                        }
                    }
                    stream_ended = true;
                    break;
                }
                if (packet->stream_index != video_index) {
                    continue;
                }

                common::EncodedFrame frame;
                frame.header.sequence = sequence.fetch_add(1) + 1;
                frame.header.receive_time_ms = SteadyNowMs();
                frame.codec = codec;
                frame.stream_sequence = frame.header.sequence;
                frame.is_key_frame = (packet->flags & AV_PKT_FLAG_KEY) != 0;
                frame.data.assign(packet->data, packet->data + packet->size);
                received_bytes.fetch_add(static_cast<uint64_t>(packet->size));
                (void)stream_output.Emplace(std::move(frame));
            }
            av_packet_free(&packet);

            CloseStream();
            if (!stop_requested.load()) {
                std::this_thread::sleep_for(config.reconnect_delay);
            }
        }
    }

    void Stop() {
        if (!thread.joinable()) {
            return;
        }
        stop_requested = true;  // 中断回调使 av_read_frame 尽快返回
        thread.join();
        CloseStream();
    }
};

CameraReceiver::CameraReceiver(CameraReceiverConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
    SPDLOG_INFO("摄像头接收器创建: url={}", impl_->config.rtsp_url);
}

CameraReceiver::~CameraReceiver() {
    if (impl_ != nullptr) {
        impl_->Stop();
    }
    SPDLOG_INFO("摄像头接收器销毁: url={}", impl_->config.rtsp_url);
}

bool CameraReceiver::Start() {
    if (impl_->thread.joinable()) {
        return true;  // 已启动，幂等
    }
    impl_->stop_requested = false;
    impl_->thread = std::thread(&Impl::ReceiveLoop, impl_.get());
    SPDLOG_INFO("摄像头接收器启动: url={}", impl_->config.rtsp_url);
    return true;
}

void CameraReceiver::Stop() {
    impl_->Stop();
    SPDLOG_INFO("摄像头接收器停止: url={}", impl_->config.rtsp_url);
}

bool CameraReceiver::IsRunning() const {
    return impl_->thread.joinable();
}

bool CameraReceiver::IsConnected() const {
    return impl_->connected.load();
}

uint64_t CameraReceiver::ConnectCount() const {
    return impl_->connect_count.load();
}

uint64_t CameraReceiver::ReceivedBytes() const {
    return impl_->received_bytes.load();
}

uint64_t CameraReceiver::ErrorCount() const {
    return impl_->error_count.load();
}

common::Topic<common::EncodedFrame>& CameraReceiver::StreamOutput() {
    return impl_->stream_output;
}

}  // namespace drone::video
