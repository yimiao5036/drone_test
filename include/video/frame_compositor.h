/**
 * @file frame_compositor.h
 * @brief 视频帧叠加器（FrameCompositor）
 *
 * 属于 drone/video 模块。职责：订阅解码帧（NV12）与 YOLO 检测结果，
 * 在帧上绘制目标检测框 + 类型/置信度标注，产出自带叠加的标注帧
 * （kAnnotatedFrame）供图传（VideoSender）编码推流。
 *
 * 对应 docs/数据接口文档.md §5 中 kAnnotatedFrame 的生产者"感知线程（叠加后）"：
 *   kDecodedFrame(FrameHandle) + kDetection(DetectionResult)
 *        ─► FrameCompositor ─► kAnnotatedFrame(FrameHandle)
 *
 * 关键实现点：
 * - 输出用**独立的帧内存池**：把解码帧 NV12 像素拷入新句柄再叠加，避免就地
 *   修改 kDecodedFrame 共享缓冲而被 YOLO 等并发消费线程读到脏数据（数据竞态）。
 * - 不引入 OpenCV/FreeType：旁路由式叠加 + 内置 5×7 ASCII 点阵字模渲染
 *   类型/置信度文字，依赖为零，与 RKNN 后端"不引 OpenCV"的一致做法。
 * - 单消费线程串行：取一帧 → 合并最近检测 → 画框文字 → 发布；拥塞只丢图传帧
 *   不反压检测/解码。
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "common/topic.h"
#include "common/types.h"
#include "video/video_frame.h"

namespace drone::video {

/// 叠加器配置。
struct CompositorConfig {
    std::size_t pool_capacity = 8;     ///< 输出标注帧内存池容量（槽位数）
    std::uint32_t stride_alignment = 64;  ///< 输出帧水平 stride 像素对齐
    int box_line_thickness = 2;        ///< 检测框线宽（像素）
    bool draw_text = true;             ///< 是否绘制类型 + 置信度文字
    int text_scale = 1;                ///< 文字放大倍数（每字符像素 = 5*scale x 7*scale）
};

/// 视频帧叠加器。
///
/// 非"13 个部件"之一，无需 I 接口/Stub（它是 kAnnotatedFrame 的产出单元，
/// 依附于感知链）。提供与部件一致的 Start/Stop/SetInputs/输出主题/状态计数，
/// 便于装配与测试。
///
/// 日志约定：创建/销毁、池满丢帧（WARN 节流）、非法输入/错误（ERROR 节流）
/// 为关键路径日志，逐帧热路径不打日志。
class FrameCompositor final {
public:
    explicit FrameCompositor(CompositorConfig config);
    ~FrameCompositor();

    FrameCompositor(const FrameCompositor&) = delete;
    FrameCompositor& operator=(const FrameCompositor&) = delete;

    // ---- 生命周期 ----
    bool Start();
    void Stop();
    bool IsRunning() const;

    // ---- 输入 ----
    /// 绑定解码帧输入主题（IVideoDecoder::FrameOutput()）。
    void SetDecodedInput(common::Topic<FrameHandle>& decoded);
    /// 绑定检测结果输入主题（IYoloDetector::DetectionOutput()）。
    void SetDetectionInput(common::Topic<common::DetectionResult>& detection);

    // ---- 输出 ----
    /// 标注帧输出主题：video::FrameHandle（NV12，叠加检测框/文字后）。
    common::Topic<FrameHandle>& AnnotatedOutput();

    // ---- 状态查询 ----
    /// 累计成功叠加并发布帧数。
    std::uint64_t AnnotatedCount() const;
    /// 累计丢弃帧数（内存池满、输入非法等）。
    std::uint64_t DroppedFrameCount() const;
    /// 累计错误次数。
    std::uint64_t ErrorCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace drone::video
