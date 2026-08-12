/**
 * @file yolo_detector.h
 * @brief YOLO 目标识别部件接口（IYoloDetector）
 *
 * 属于 drone/perception 模块。职责：订阅解码图像帧（video::FrameHandle），
 * 运行 YOLO 推理，输出结构化检测结果（类别、置信度、像素位置），
 * 发布到检测结果主题。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，RKNN 推理后端（RGA 预处理 + NPU）在实现期接入。
 * - YoloDetectorStub 为骨架占位实现：生命周期可运行，业务方法
 *   记录"未实现"节流日志并返回默认值。
 *
 * 数据流：common::Topic<FrameHandle> ──► IYoloDetector ──► common::Topic<DetectionResult>
 * 可替换边界：模型与推理后端。
 */
#pragma once

#include <cstdint>

#include "common/topic.h"
#include "common/types.h"
#include "video/video_frame.h"

namespace drone::perception {

/// YOLO 目标识别部件抽象接口。
class IYoloDetector {
public:
    virtual ~IYoloDetector() = default;

    // ---- 生命周期 ----
    /// 启动推理（加载模型、启动消费线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止推理并释放模型资源；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;

    // ---- 输入 ----
    /// 绑定解码帧输入主题（IVideoDecoder::FrameOutput()）。
    virtual void SetInput(common::Topic<video::FrameHandle>& input) = 0;

    // ---- 输出 ----
    /// 检测结果输出主题：common::DetectionResult。
    virtual common::Topic<common::DetectionResult>& DetectionOutput() = 0;

    // ---- 状态查询 ----
    /// 累计处理帧数。
    virtual uint64_t ProcessedFrameCount() const = 0;
    /// 最近推理平均耗时（毫秒）；未启动或未推理时为 0。
    virtual float InferenceTimeMsAvg() const = 0;
    /// 累计错误次数（模型推理失败等）。
    virtual uint64_t ErrorCount() const = 0;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class YoloDetectorStub final : public IYoloDetector {
public:
    YoloDetectorStub();
    ~YoloDetectorStub() override;

    YoloDetectorStub(const YoloDetectorStub&) = delete;
    YoloDetectorStub& operator=(const YoloDetectorStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInput(common::Topic<video::FrameHandle>& input) override;

    common::Topic<common::DetectionResult>& DetectionOutput() override;

    uint64_t ProcessedFrameCount() const override;
    float InferenceTimeMsAvg() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t processed_count_ = 0;
    float inference_time_ms_avg_ = 0.f;
    uint64_t error_count_ = 0;
    common::Topic<common::DetectionResult> detection_output_;
};

}  // namespace drone::perception
