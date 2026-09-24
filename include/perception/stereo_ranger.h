#pragma once

// =============================================================================
// stereo_ranger.h —— 双目测距组件（StereoRanger）
//
// 对应设计：docs/superpowers/specs/2026-09-23-双目测距v1.0-design.md §4
//
// 职责：双订阅左右目 DetectionResult，按 header.source_time_ms 以
//   ±pair_window 时间邻接配对为左右帧，交 StereoRangerCore 测距，
//   每个左目目标发布一条 StereoTargetDistance（未匹配 valid=false 照常
//   发布方位）。节流摘要日志：配对率/有效域内占比/|Δy| P95/测距耗时。
//
// 边界：影子链路组件，不接健康源（本轮设计决定）；不做 remap、不做多目标
//   跟踪（v1.1）；右目无检测帧 YOLO 不发布消息，走左帧超龄路径。
//
// 数据流：
//   Topic<DetectionResult>(左,kDetection)      ─┐
//                                                ├─► StereoRanger ──► Topic<StereoTargetDistance>
//   Topic<DetectionResult>(右,kDetectionRight) ─┘
//
// 线程模型：独立消费线程；Stop() 置停止标志 → Reset 订阅唤醒 → join；
//   Start/Stop 幂等。热路径零日志，摘要按 summary_log_interval 节流。
// =============================================================================

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>

#include "common/topic.h"
#include "common/types.h"
#include "perception/stereo_ranger_core.h"

namespace drone::perception {

/// 双目测距组件壳（算法在 StereoRangerCore）。
class StereoRanger final {
public:
    explicit StereoRanger(StereoRangerConfig config);
    ~StereoRanger();

    StereoRanger(const StereoRanger&) = delete;
    StereoRanger& operator=(const StereoRanger&) = delete;

    // ---- 生命周期 ----
    bool Start();   ///< 左右输入未接线或已运行返回 false；幂等
    void Stop();    ///< 幂等
    bool IsRunning() const;

    // ---- 输入（Start 前接线） ----
    void SetLeftInput(common::Topic<common::DetectionResult>& input);   ///< kDetection
    void SetRightInput(common::Topic<common::DetectionResult>& input);  ///< kDetectionRight

    // ---- 输出 ----
    common::Topic<common::StereoTargetDistance>& DistanceOutput();

    // ---- 状态查询 ----
    std::uint64_t ProcessedPairCount() const;   ///< 已处理左目帧数（含超龄未配对）
    std::uint64_t MatchedPairCount() const;     ///< core 透传
    std::uint64_t ValidDistanceCount() const;   ///< core 透传
    std::uint64_t ErrorCount() const;           ///< 输入非法/启动失败等
    double DeltaYP95Px() const;                 ///< core 透传（无样本 NaN）

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace drone::perception
