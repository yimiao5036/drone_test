#ifndef RTSP_YOLO_STREAM_RTSP_ENCODER_H
#define RTSP_YOLO_STREAM_RTSP_ENCODER_H

#include <memory>
#include <string>

#include <opencv2/core.hpp>

// RTSP 推流 + RK3588 硬件编码封装。
// 依赖官方 FFmpeg 5.0+（或 nyanmisaka/ffmpeg-rockchip）的 rkmpp 编码器，
// 输入 BGR 格式的 cv::Mat，内部完成：
//   BGR -> NV12 -> h264_rkmpp/hevc_rkmpp 硬编码 -> RTSP 发送
//
// 使用示例：
//   RtspEncoder encoder("rtsp://127.0.0.1:8554/out", 1920, 1080, 25);
//   encoder.WriteFrame(bgr_mat);
//   encoder.Flush();
class RtspEncoder {
public:
    // codec_name 可选 "h264_rkmpp"（默认）或 "hevc_rkmpp"
    // 失败时抛出 std::runtime_error
    RtspEncoder(const std::string& rtsp_url, int width, int height, int fps,
                const std::string& codec_name = "h264_rkmpp");
    ~RtspEncoder();

    RtspEncoder(const RtspEncoder&) = delete;
    RtspEncoder& operator=(const RtspEncoder&) = delete;

    // 送入一帧 BGR 图像进行硬编码并推流，失败返回 false
    bool WriteFrame(const cv::Mat& bgr_frame);

    // 冲刷编码器缓存并写入流结束信息（析构时也会自动调用）
    void Flush();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#endif // RTSP_YOLO_STREAM_RTSP_ENCODER_H
