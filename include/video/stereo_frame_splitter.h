/**
 * @file stereo_frame_splitter.h
 * @brief 双目 SBS 全幅帧左右拆分部件（StereoFrameSplitter）
 *
 * 属于 drone/video 模块。职责：订阅解码器全幅 SBS 帧（如 2560×720 NV12，
 * 左半幅一目、右半幅另一目），用 RGA 裁剪（不可用时逐行 memcpy 回退）拆成
 * 左右两个 1280×720 NV12 帧，分别发布到左目/右目输出主题。
 *
 * 本轮（USB 双目接入 B 层）语义：
 * - 左目输出接现有 YOLO/叠加链路出图；右目输出仅统计，无订阅者。
 * - 左右目对应关系：SBS 左半幅是否为物理左目未经实证，本轮任取左半幅为
 *   主用目；测距阶段开始前必须实证，若相反仅交换两个裁剪 rect 即可。
 *
 * 数据流：Topic<FrameHandle>(全幅) ──► StereoFrameSplitter ──► Topic<FrameHandle>(左目)
 *                                                          └──► Topic<FrameHandle>(右目)
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "common/topic.h"
#include "video/video_frame.h"

namespace drone::video {

/// 双目拆分配置。
struct StereoFrameSplitterConfig {
    std::uint32_t width = 2560;          ///< 全幅 SBS 宽（必须为正的偶数）
    std::uint32_t height = 720;          ///< 全幅高
    std::size_t pool_capacity = 8;       ///< 左右各自输出内存池容量
    std::size_t input_queue_capacity = 1;  ///< 输入订阅队列容量（1=只处理最新帧）
    std::uint32_t stride_alignment = 64;   ///< 输出帧水平 stride 像素对齐（RGA 友好）
    bool prefer_rga = true;              ///< 优先 RGA 裁剪；失败/无 RGA 回退 memcpy
};

/// 双目 SBS 帧拆分器。
///
/// 独立消费线程订阅全幅 NV12 帧（输入队列容量 1、丢最旧），左右各一个
/// 自有 VideoFramePool（width/2 × height NV12，stride 按 stride_alignment
/// 对齐），左右独立容错：一目池满只计该目丢帧，另一目照常发布。
///
/// 帧元数据语义（与解码器一致）：
/// - pipeline_ingress_time_ms 透传源帧（端内总延迟基准）；
/// - timestamp_ms 为拆分完成时刻（SetTiming 写入）；
/// - 序号由左右各自内存池分配（FrameHandle 不支持序号透传），左右帧率
///   一致性以 LeftFrameCount/RightFrameCount 与 ingress 时间戳核对。
///
/// 日志约定：输出池创建/销毁为生命周期 INFO；输入非法（ERROR）、池满丢帧
/// 与 RGA 回退（WARN）节流（第 1 次 + 每满 100 次）；逐帧热路径不打日志。
class StereoFrameSplitter final {
public:
    explicit StereoFrameSplitter(StereoFrameSplitterConfig config);
    ~StereoFrameSplitter();

    StereoFrameSplitter(const StereoFrameSplitter&) = delete;
    StereoFrameSplitter& operator=(const StereoFrameSplitter&) = delete;

    // ---- 生命周期 ----
    bool Start();
    void Stop();
    bool IsRunning() const;

    // ---- 输入 ----
    /// 绑定全幅解码帧输入主题（VideoDecoder::FrameOutput()）。
    void SetInput(common::Topic<FrameHandle>& input);

    // ---- 输出 ----
    /// 左目输出主题（width/2 × height NV12）。
    common::Topic<FrameHandle>& LeftOutput();
    /// 右目输出主题（本轮仅统计，无订阅者）。
    common::Topic<FrameHandle>& RightOutput();

    // ---- 状态查询 ----
    std::uint64_t LeftFrameCount() const;
    std::uint64_t RightFrameCount() const;
    std::uint64_t LeftDroppedCount() const;
    std::uint64_t RightDroppedCount() const;
    std::uint64_t ErrorCount() const;
    std::uint64_t RgaFallbackCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace drone::video
