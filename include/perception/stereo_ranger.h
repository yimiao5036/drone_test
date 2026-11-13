#pragma once

// =============================================================================
// stereo_ranger.h —— 双目测距组件（v2：稀疏 Census 路线）
//
// 对应设计：docs/superpowers/specs/2026-10-10-双目测距v2-稀疏Census-design.md §3
//
// 职责：订阅左目检测框与左右目原始帧（基准图=SBS右半=左目Topic，辅助图=
//   SBS左半），按 frame_sequence 把检测成批、按时间窗把左右帧配对，交
//   StereoRangerCore 测距，每个左目检测框发布一条 StereoTargetDistance。
//
// 数据流：
//   Topic<DetectionResult>(左,kDetection)        ─┐
//   Topic<FrameHandle>(基准,kDecodedFrameLeft)   ─┼─► StereoRanger
//   Topic<FrameHandle>(辅助,kDecodedFrameRight)  ─┘        │
//                                            Topic<StereoTargetDistance>
//                                                        ▼
//                              VisualTrackingShadow::SetDistanceInput（可选，语义不变）
//
// 帧/批次配对语义：
//   - 帧环形缓冲（深度 frame_buffer_depth，受拆分器内存池容量约束）：左右帧
//     按 |Δt|≤pair_window 时间邻接配对（拆分器左右帧时间差实测 0~1ms）；
//   - 检测按 frame_sequence 成批：新序号的首条检测到达、或 settle 超时后批次
//     闭合；闭合批次按序处理；
//   - 帧缺失/被挤出/预热期：该批次每个框仍发布一条消息（valid=false），保证
//     下游每个目标都有节拍。
//
// 边界（影子链路）：只发布 kStereoTargetDistance；唯一消费者是
//   VisualTrackingShadow 的距离通道；不产生控制输出；不注册健康源。
//
// 线程模型：独立消费线程以 10ms 节拍阻塞等待左目检测驱动 Pump，同时排空帧
//   订阅、闭合超时批次、节流摘要；Stop() 置停止标志 → Reset 三路订阅唤醒 →
//   join → core.Reset()。Start/Stop 幂等。热路径零日志。
// =============================================================================

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>

#include "common/topic.h"
#include "common/types.h"
#include "perception/stereo_ranger_core.h"
#include "video/video_frame.h"

namespace drone::perception {

/// 双目测距组件壳（算法在 StereoRangerCore）。
/// 校正矩阵由 config 模块从预置 JSON 加载后传入；构造期构建映射表。
class StereoRanger final {
public:
    StereoRanger(StereoRangerConfig config, StereoRectifyMatrices matrices);
    ~StereoRanger();

    StereoRanger(const StereoRanger&) = delete;
    StereoRanger& operator=(const StereoRanger&) = delete;

    // ---- 生命周期 ----
    bool Start();   ///< 三路输入未接线或已运行返回 false；幂等
    void Stop();    ///< 幂等
    bool IsRunning() const;

    // ---- 输入（Start 前接线）----
    /// 左目检测框（原始左图像素坐标），接 YoloDetector::DetectionOutput()。
    void SetDetectionInput(common::Topic<common::DetectionResult>& input);
    /// 基准帧（SBS 右半=物理左目），接 StereoFrameSplitter::LeftOutput()。
    void SetLeftFrameInput(common::Topic<video::FrameHandle>& input);
    /// 辅助帧（SBS 左半=物理右目），接 StereoFrameSplitter::RightOutput()。
    void SetRightFrameInput(common::Topic<video::FrameHandle>& input);

    // ---- 输出 ----
    common::Topic<common::StereoTargetDistance>& DistanceOutput();

    // ---- 状态查询（摘要日志/测试）----
    std::uint64_t ProcessedFrameCount() const;  ///< core：已处理帧（含跳过）
    std::uint64_t MatchedFrameCount() const;    ///< core：实际匹配帧数
    std::uint64_t AvailableCount() const;       ///< core：累计可用测距框数
    std::uint64_t ClippedCount() const;         ///< core：累计截断告警框数
    std::uint64_t SkippedFrameCount() const;    ///< core：黑帧/预热跳过
    std::uint64_t BatchCount() const;           ///< 已发布结果的检测批次数
    std::uint64_t FrameMissCount() const;       ///< 批次未能配对到帧的次数
    std::uint64_t ErrorCount() const;           ///< 未接线启停等错误
    double LastMatchMs() const;                 ///< core：最近匹配耗时（ms）

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace drone::perception
