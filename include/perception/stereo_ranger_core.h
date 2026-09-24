#pragma once

// =============================================================================
// stereo_ranger_core.h —— 双目测距纯算法核心（StereoRangerCore）
//
// 对应设计：docs/superpowers/specs/2026-09-23-双目测距v1.0-design.md
//
// 职责：输入一对已时间配准的左右目 DetectionResult 数组，完成
//   匹配（硬约束+代价贪心，§3.2）→ 去主点视差（§3.1，两目 cx 实测差 54px，
//   不去主点近距视差符号会反）→ Z 三角测量 → 有效域判定（§3.3，≤11m）
//   → Z 一阶低通滤波与跳变重置（§3.4）→ |Δy| 在线校验统计（§3.5）。
//
// 边界：纯算法类，不碰 Topic/线程/日志；不 remap 校正（v1.1）；
//   多目标只做类别+最近邻滤波关联，全量跟踪留 v1.1。
// =============================================================================

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "common/types.h"

namespace drone::perception {

/// 双目测距标定与算法参数。默认值即 2026-09-23 标定实测
/// （config/camera_calibration_uvc_1280x720.json），由 config.cpp 从
/// stereo_ranger 段解析覆盖。输入队列/配对窗/摘要间隔仅组件壳使用。
struct StereoRangerConfig {
    double baseline_m = 0.060085;   ///< 基线（米，立体 T 的 x 分量）
    double fx_left_px = 1007.82;    ///< 左目水平像素焦距
    double fy_left_px = 1008.02;
    double cx_left_px = 620.73;     ///< 左目主点 x（非画面中心）
    double cy_left_px = 532.07;     ///< 左目主点 y（偏中心，实测）
    double fx_right_px = 1006.59;   ///< 右目水平像素焦距
    double fy_right_px = 1006.07;
    double cx_right_px = 674.76;    ///< 右目主点 x（与左目差约 54px）
    double cy_right_px = 526.70;
    double disparity_min_px = 3.0;  ///< 有效判定最小像素视差（δZ 发散门限）
    double distance_min_m = 1.0;    ///< 匹配/有效距离下限（米）
    double distance_max_m = 11.0;   ///< 有效域上限（δZ≤2m 边界，可整定）
    double tau_y_px = 2.0;          ///< 匹配纵向残差门限（去 cy，⚠待板上回填）
    double tau_h_ratio = 0.25;      ///< 匹配框高比门限 |hL−hR|/max
    double smooth_alpha = 0.3;      ///< Z 一阶低通系数 (0,1]
    double jump_reset_m = 5.0;      ///< |ΔZ| 跳变重置门限（米）
    std::size_t input_queue_capacity = 2;                 ///< 组件双订阅队列容量
    std::chrono::milliseconds pair_window{20};            ///< 左右配对时间窗
    std::chrono::milliseconds summary_log_interval{10000};///< 组件摘要日志间隔

    /// 取值校验；非法抛 std::invalid_argument 并携带字段名。
    void Validate() const;
};

/// 一对匹配结果（左/右数组下标 + 匹配代价）。
struct StereoMatch {
    std::size_t left_index = 0;
    std::size_t right_index = 0;
    double cost = 0.0;
};

/// 匹配器接口（依赖注入便于单测；默认实现 GreedyStereoMatcher）。
class IStereoMatcher {
public:
    virtual ~IStereoMatcher() = default;
    /// 硬约束过滤 + 代价升序贪心一一配对（设计 §3.2）。
    virtual std::vector<StereoMatch> Match(
        const std::vector<common::DetectionResult>& left,
        const std::vector<common::DetectionResult>& right) const = 0;
};

/// 贪心匹配器（设计 §3.2 原样）。
/// 硬约束：类别一致；|Δy|（去 cy）≤ tau_y_px；去主点视差对应距离在
/// [distance_min_m, distance_max_m]；|hL−hR|/max(hL,hR) ≤ tau_h_ratio。
/// 代价 cost = |Δy|/tau_y + |Δh|/(tau_h·max)，cost<1 才接受。
class GreedyStereoMatcher final : public IStereoMatcher {
public:
    explicit GreedyStereoMatcher(StereoRangerConfig config);
    std::vector<StereoMatch> Match(
        const std::vector<common::DetectionResult>& left,
        const std::vector<common::DetectionResult>& right) const override;

private:
    StereoRangerConfig config_;
};

/// 双目测距纯算法核心。线程不安全，由组件壳独占调用。
class StereoRangerCore {
public:
    /// @param matcher 为空时使用 GreedyStereoMatcher 默认实现
    explicit StereoRangerCore(StereoRangerConfig config,
                              std::shared_ptr<const IStereoMatcher> matcher = {});
    ~StereoRangerCore();  // 源文件默认实现（PIMPL 完整类型要求）

    /// 处理一对已时间配准的左右帧检测结果；每个左目目标恰好输出一条
    /// （未匹配目标 valid=false、距离字段 NaN、方位字段取左目）。
    /// core 只填 frame_sequence 与 header.source_time_ms；
    /// header.sequence / receive_time_ms 由组件发布时填写。
    std::vector<common::StereoTargetDistance> Process(
        const std::vector<common::DetectionResult>& left,
        const std::vector<common::DetectionResult>& right,
        std::uint64_t frame_sequence,
        std::uint64_t source_time_ms);

    std::uint64_t MatchedPairCount() const;    ///< 累计匹配成功对数
    std::uint64_t ValidDistanceCount() const;  ///< 累计有效域内测距数
    /// |Δy|（去 cy）滑动窗口 P95；无样本返回 NaN。
    double DeltaYP95Px() const;
    /// 清空滤波与统计状态（组件 Stop 时调用）。
    void ResetFilter();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace drone::perception
