/**
 * @file yolo_detector.h
 * @brief YOLO 目标识别部件接口与实现（IYoloDetector / YoloDetector）
 *
 * 属于 drone/perception 模块。职责：订阅解码图像帧（video::FrameHandle），
 * 运行 YOLO 推理，输出结构化检测结果（类别、置信度、像素位置），
 * 发布到检测结果主题。
 *
 * 数据流：common::Topic<FrameHandle> ──► IYoloDetector ──► common::Topic<DetectionResult>
 * 可替换边界：模型与推理后端（IDetectionBackend）。
 *
 * 实现说明（YoloDetector）：
 * - 独立消费线程：订阅解码帧主题，逐帧调用推理后端，发布检测结果。
 * - 推理后端通过 IDetectionBackend 依赖注入：香橙派部署时默认使用
 *   RKNN 后端（RGA 预处理 + NPU 推理），开发机测试注入 Mock 后端。
 * - 后端不可用时 Start() 返回 false 并记录 ERROR 日志，不静默降级。
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "common/topic.h"
#include "common/types.h"
#include "video/video_frame.h"

namespace drone::perception {

class IDetectionBackend;

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

/// YOLO 检测器配置。
struct YoloDetectorConfig {
    std::string model_path;               ///< RKNN 模型文件路径；空 = 必须注入后端
    float conf_threshold = 0.25f;         ///< 检测置信度阈值 [0,1]
    float nms_threshold = 0.45f;          ///< NMS IoU 阈值 [0,1]
    std::size_t input_queue_capacity = 2; ///< 解码帧订阅队列容量（丢最旧）
};

/// YOLO 检测器（实现 IYoloDetector）。
///
/// 职责：订阅解码帧（NV12），经推理后端逐帧检测，将结果转换为
/// common::DetectionResult 发布。线程模型与 VideoDecoder 一致：
/// 独立消费线程 + Topic 订阅，确定性停机（Stop 唤醒并 join）。
///
/// 检测结果发布约定：
/// - 每个检测目标发布一条 DetectionResult，同一帧的多目标共享
///   frame_sequence（= 帧 Info().sequence）与 inference_time_ms。
/// - 一帧无检测时不发布（融合侧按帧超时判断跟踪丢失）。
///
/// 日志约定：创建/销毁（INFO）、启动失败/后端加载失败（ERROR）、
/// 推理失败（ERROR 节流：第 1 次 + 每满 100 次）、无效帧跳过（WARN 节流）。
/// 逐帧热路径不打日志。
class YoloDetector final : public IYoloDetector {
public:
    /// @param config 检测器配置
    /// @param backend 推理后端；为空时按配置创建默认后端
    ///        （编译启用 RKNN 且 model_path 非空 → RKNN 后端；否则无后端，
    ///        Start 返回 false）。测试注入 Mock 后端验证线程与发布逻辑。
    explicit YoloDetector(YoloDetectorConfig config,
                          std::unique_ptr<IDetectionBackend> backend = {});
    ~YoloDetector() override;

    YoloDetector(const YoloDetector&) = delete;
    YoloDetector& operator=(const YoloDetector&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInput(common::Topic<video::FrameHandle>& input) override;

    common::Topic<common::DetectionResult>& DetectionOutput() override;

    uint64_t ProcessedFrameCount() const override;
    float InferenceTimeMsAvg() const override;
    uint64_t ErrorCount() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
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
