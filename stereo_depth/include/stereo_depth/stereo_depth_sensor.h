/**
 * @file stereo_depth_sensor.h
 * @brief 传统双目测距传感器对外总入口（组件壳）
 *
 * 职责：编排「校正 → ROI 提取 → Census → SGM → 亚像素 → 后处理 → 区域聚合 →
 * 深度 → 输出级滤波」，对外只暴露 Process（喂一对左右目灰度帧）与 Query
 * （给一个原始左目像素系矩形，返回该区域距离）。
 *
 * 实时性说明（对设计文档 §8 的实现取舍）：ROI 策略下单目标每帧成本
 * （全分辨率 remap + 小 ROI 匹配）远小于 33ms 帧预算，故 v1 采用**同步**
 * Process/Query，无后台流水线线程；多目标并发或单帧内并行加速留待后续按需
 * 引入（见 stereo_depth.md）。
 *
 * 线程模型：非线程安全，由调用方（主项目适配器的单一消费线程）独占串行调用
 * Process→Query。内部输出级滤波按 track_id 维护状态。
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "stereo_depth/sgm_matcher.h"
#include "stereo_depth/stereo_depth_config.h"
#include "stereo_depth/stereo_depth_types.h"
#include "stereo_depth/stereo_rectifier.h"

namespace stereo_depth {

/// 双目测距传感器。
class StereoDepthSensor {
public:
    explicit StereoDepthSensor(StereoDepthConfig config);
    ~StereoDepthSensor();

    StereoDepthSensor(const StereoDepthSensor&) = delete;
    StereoDepthSensor& operator=(const StereoDepthSensor&) = delete;

    /// 用 config.calibration_path 加载标定并构建校正/匹配器。
    /// 成功返回 true；失败记 ERROR 并返回 false（不抛，便于上层降级）。
    bool Init();
    /// 用显式标定路径初始化（覆盖 config.calibration_path）。
    bool Init(const std::string& calibration_path);

    /// 处理一对左右目灰度帧：全分辨率 remap 校正到内部缓冲。
    /// 尺寸须等于 source_width×source_height（标定档位）。成功返回 true。
    bool Process(const StereoFramePair& pair);

    /// 查询最近一帧中某区域（原始左目像素系）的距离，含输出级滤波。
    DistanceResult Query(const QueryRect& rect);
    /// 查询单点（以 (x,y) 为中心的小窗）。
    DistanceResult QueryPoint(std::int32_t x, std::int32_t y, std::int32_t track_id = 0);

    /// 是否已成功处理过至少一帧。
    [[nodiscard]] bool HasFrame() const noexcept { return has_frame_; }
    /// 清空所有 track 的滤波器状态。
    void ResetFilters();

    // ---- 排查计数 ----
    [[nodiscard]] std::uint64_t ProcessedFrameCount() const noexcept { return processed_frames_; }
    [[nodiscard]] std::uint64_t QueryCount() const noexcept { return query_count_; }
    [[nodiscard]] std::uint64_t ValidCount() const noexcept { return valid_count_; }
    [[nodiscard]] std::uint64_t ErrorCount() const noexcept { return error_count_; }

private:
    /// 每 track 的一阶低通 + 跳变重置滤波状态。
    struct FilterState {
        double z = 0.0;
        bool initialized = false;
    };

    /// 对一次有效原始距离做滤波，返回滤波后距离。
    double ApplyFilter(std::int32_t track_id, double z_raw);

    StereoDepthConfig cfg_;
    StereoCalibration cal_;
    StereoRectifier rectifier_;
    std::unique_ptr<SgmMatcher> matcher_;

    std::vector<std::uint8_t> rect_left_;   ///< 全分辨率 rectified 左目
    std::vector<std::uint8_t> rect_right_;  ///< 全分辨率 rectified 右目
    std::vector<std::uint8_t> roi_left_;    ///< ROI 灰度裁剪缓冲（复用）
    std::vector<std::uint8_t> roi_right_;
    std::vector<std::uint64_t> census_left_;   ///< ROI Census（复用）
    std::vector<std::uint64_t> census_right_;
    std::vector<float> disp_;       ///< ROI 视差（复用）
    std::vector<float> disp_filt_;  ///< 后处理后视差（复用）

    bool ready_ = false;
    bool has_frame_ = false;
    std::unordered_map<std::int32_t, FilterState> filters_;

    std::uint64_t processed_frames_ = 0;
    std::uint64_t query_count_ = 0;
    std::uint64_t valid_count_ = 0;
    std::uint64_t error_count_ = 0;
    std::uint64_t warn_counter_ = 0;  ///< 节流用
};

}  // namespace stereo_depth
