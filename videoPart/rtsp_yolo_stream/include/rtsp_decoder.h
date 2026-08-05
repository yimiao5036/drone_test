#ifndef RTSP_YOLO_STREAM_RTSP_DECODER_H
#define RTSP_YOLO_STREAM_RTSP_DECODER_H

#include <memory>
#include <string>

#include <opencv2/core.hpp>

// RTSP 拉流 + RK3588 硬件解码封装。
// 内部自动选择 rkmpp 硬解码器（H.265 -> hevc_rkmpp，H.264 -> h264_rkmpp），
// 解码结果统一转换为 BGR 格式的 cv::Mat 输出。
//
// 使用示例：
//   RtspDecoder decoder("rtsp://192.168.1.100:8554/live");
//   cv::Mat frame;
//   while (decoder.ReadFrame(frame)) { ... }
class RtspDecoder {
public:
    // 打开 RTSP 流并完成解码器初始化，失败时抛出 std::runtime_error
    explicit RtspDecoder(const std::string& rtsp_url);
    ~RtspDecoder();

    // FFmpeg 上下文不可复制
    RtspDecoder(const RtspDecoder&) = delete;
    RtspDecoder& operator=(const RtspDecoder&) = delete;

    // 读取并硬解码一帧，转换为 BGR 写入 bgr_frame。
    // 返回 false 表示流结束或发生不可恢复错误。
    bool ReadFrame(cv::Mat& bgr_frame);

    int width() const;
    int height() const;
    // 根据流信息估算的帧率（无法获取时返回 25）
    int fps() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#endif // RTSP_YOLO_STREAM_RTSP_DECODER_H
