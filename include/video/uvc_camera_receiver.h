/**
 * @file uvc_camera_receiver.h
 * @brief USB UVC 双目相机接收部件（UvcCameraReceiver）
 *
 * 属于 drone/video 模块。职责：V4L2 mmap 直采 USB UVC 双目相机
 * （SBS 同帧单流，如 2560×720@30 MJPG），逐帧产出 EncodedFrame
 * （codec=kMjpeg）发布到输出主题；断流/拔线自动关流重连。
 *
 * 与 CameraReceiver（RTSP 版）同实现 ICameraReceiver 接口，二者由
 * DroneApplication 按 video.camera_source 配置二选一装配，下游
 * （VideoDecoder/健康监控）无感。
 *
 * 数据流：UVC /dev/videoN ──► ICameraReceiver ──► common::Topic<EncodedFrame> ──► IVideoDecoder
 * 可替换边界：视频协议适配器。
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "common/topic.h"
#include "common/types.h"
#include "video/camera_receiver.h"

namespace drone::video {

/// UVC 双目相机接收配置（V4L2 mmap 采集）。
struct UvcCameraReceiverConfig {
    std::string device = "/dev/video0";                ///< V4L2 采集节点（SBS 同帧单流仅 1 个）
    std::uint32_t width = 2560;                        ///< 全幅 SBS 宽（偶数，左右各半）
    std::uint32_t height = 720;                        ///< 全幅高
    std::uint32_t fps = 30;                            ///< 采集帧率（S_PARM 尽力而为设置）
    std::uint32_t buffer_count = 4;                    ///< V4L2 mmap 缓冲数（>=2）
    std::chrono::milliseconds reconnect_delay{3000};   ///< 断流/拔线后重连间隔
};

/// UVC 双目相机接收器（实现 ICameraReceiver）。
///
/// 职责：V4L2 mmap 直采，S_FMT(MJPG) 后 G_FMT 回读校验（必须拿到配置的
/// 分辨率与 MJPG 格式，否则 Start 失败，防固件错档）；采集线程 DQBUF 逐帧
/// 构成 EncodedFrame（codec=kMjpeg、is_key_frame=true、parameter_sets 空）
/// 发布；无帧累计 1s 或 EIO 视为断流，关流后按 reconnect_delay 重连。
///
/// 时间戳语义（与 docs/双目深度估计思路.md §3.3 一致，不得伪造）：
/// - capture_time_ms / header.source_time_ms：v4l2 buffer 的
///   V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC 时间戳（monotonic/soe，与主机单调
///   时钟同域，探测报告实证可用）；无 MONOTONIC 标志时留 0。
/// - header.receive_time_ms：DQBUF 后主机单调时钟。
///
/// 日志约定：建连成功/断流重连为关键路径日志（INFO/WARN），错误节流
/// （第 1 次 + 每满 100 次）；逐帧热路径不打日志。
class UvcCameraReceiver final : public ICameraReceiver {
public:
    explicit UvcCameraReceiver(UvcCameraReceiverConfig config);
    ~UvcCameraReceiver() override;

    UvcCameraReceiver(const UvcCameraReceiver&) = delete;
    UvcCameraReceiver& operator=(const UvcCameraReceiver&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    bool IsConnected() const override;
    uint64_t ConnectCount() const override;
    uint64_t ReceivedBytes() const override;
    uint64_t ErrorCount() const override;

    common::Topic<common::EncodedFrame>& StreamOutput() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace drone::video
