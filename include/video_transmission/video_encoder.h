/**
 * @file video_encoder.h
 * @brief 视频编码 + 图传推送后端抽象（IVideoEncoderBackend）
 *
 * 属于 drone/video_transmission 模块，是图传发送部件的**可替换边界**。
 *
 * 设计目标：
 * - 将"编码 + 图传推流"这一装置相关的环节抽象为接口，方便以后切换图传
 *   设备或编码后端而不改动 VideoSender 主流程。
 * - 默认实现走 FFmpeg 封装：香橙派优先 rkmpp 硬编码（h264_rkmpp），
 *   开发机回退 libx264 软编码，均支持 RTSP 推流。
 *
 * 可替换边界说明（对应 docs/数据接口文档.md §4.3）：
 * - 编码后端：rkmpp 硬编 / 软件编码 / 其他硬件；
 * - 图传协议输出：当前为 RTSP 推流，后续可替换为其他图传协议适配器。
 *
 * 职责约定：
 * - 只接收 NV12 帧（video::FrameHandle）编码并推流，不做叠加/裁剪等图像
 *   处理（那是 FrameCompositor 的职责，见 include/video/frame_compositor.h）。
 * - 拥塞/失败只影响图传链，不得反压上游感知链路（由 VideoSender 的
 *   订阅队列丢弃策略保证）。
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "video/video_frame.h"

namespace drone::video_transmission {

/// 编码与图传推送后端配置。
struct EncoderBackendConfig {
    std::string url;            ///< RTSP 推流地址（如 rtmp/rtsp://...）
    std::string codec = "h264"; ///< 编码格式："h264" / "h265"（rkmpp 或软编码）
    std::uint32_t width = 0;    ///< 编码尺寸宽（与输入帧一致，仅日志展示）
    std::uint32_t height = 0;   ///< 编码尺寸高
    int fps = 25;               ///< 目标帧率（时间基与 I 帧间隔依据）
    std::int64_t bitrate = 8 * 1024 * 1024;  ///< 码率（bps），图传带宽权衡
    int gop = 50;               ///< I 帧间隔（帧数），可换取关键帧接入延迟
    std::string transport = "tcp";  ///< RTSP 传输协议：tcp / udp
    std::string output_format = "rtsp";  ///< 输出封装：rtsp(默认) / mpegts(本地文件调试)
    std::string output_url = "";         ///< 实际输出地址（非空时优先于 url，本地文件调试用）
    bool prefer_hardware = true;    ///< 优先 rkmpp 硬编码（香橙派）；否则软编码
};

/// 视频编码 + 图传推送后端抽象接口。
///
/// 生命周期：Start（建立编码器与 RTSP 会话）→ 多帧 EncodeFrame → Stop。
/// 线程安全：实现应只在 VideoSender 的单线程内被调用。
class IVideoEncoderBackend {
public:
    virtual ~IVideoEncoderBackend() = default;

    // ---- 生命周期 ----
    /// 建立编码器与 RTSP 输出会话。失败返回 false（地址不可达、无可用编码器）。
    virtual bool Start() = 0;
    /// 冲刷尾帧、写入 trailer 并关闭会话。幂等。
    virtual void Stop() = 0;
    /// 是否已建立会话。
    virtual bool IsRunning() const = 0;

    // ---- 帧输入 ----
    /// 编码一帧 NV12 并推流。
    /// @param frame 标注后的 NV12 帧（FrameHandle）。
    /// @return 是否成功送入编码器。推流失败（网络/会话断开）返回 false。
    virtual bool EncodeFrame(const video::FrameHandle& frame) = 0;

    // ---- 状态查询 ----
    /// 累计成功推流帧数。
    virtual std::uint64_t SentFrameCount() const = 0;
    /// 累计错误次数。
    virtual std::uint64_t ErrorCount() const = 0;
};

/// 按配置创建默认编码后端（FFmpeg 封装：rkmpp 硬编优先 / 软编回退）。
/// 返回非空指针；失败时 Start() 返回 false，不直接抛异常。
std::unique_ptr<IVideoEncoderBackend> CreateVideoEncoderBackend(
    EncoderBackendConfig config);

}  // namespace drone::video_transmission
