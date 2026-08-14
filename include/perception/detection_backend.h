/**
 * @file detection_backend.h
 * @brief 检测后端抽象接口（IDetectionBackend）
 *
 * 属于 drone/perception 模块。YoloDetector 通过该接口调用具体推理后端，
 * 实现"推理后端可替换"的可替换边界：
 *
 *   YoloDetector（线程/订阅/发布/统计）
 *        │  Detect(FrameHandle)
 *        ▼
 *   IDetectionBackend（抽象）
 *        ├── RknnDetectionBackend（香橙派：RGA 预处理 + NPU 推理，条件编译）
 *        └── 测试 Mock（开发机验证 YoloDetector 线程与发布逻辑）
 *
 * 约定：
 * - Detect 输入为解码帧句柄（NV12），输出为原图坐标系像素检测框。
 * - 后端负责模型加载、图像预处理、推理、后处理与坐标还原的全部细节，
 *   对 YoloDetector 只暴露"给一帧，给结果"。
 * - Detect 由 YoloDetector 的检测线程单线程调用，后端无需自加锁。
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "video/video_frame.h"

namespace drone::perception {

/// 后端检测结果（原图坐标系，像素）。
struct BackendDetection {
    int class_id = 0;    ///< 模型类别 ID
    float confidence = 0.f;
    float x1 = 0.f;      ///< 左上角 x
    float y1 = 0.f;      ///< 左上角 y
    float x2 = 0.f;      ///< 右下角 x
    float y2 = 0.f;      ///< 右下角 y
};

/// 检测后端抽象接口。
class IDetectionBackend {
public:
    virtual ~IDetectionBackend() = default;

    /// 加载模型并初始化推理资源；可重复调用（失败后重试）。
    /// @return 是否成功；失败原因记录在日志中。
    virtual bool Load() = 0;

    /// 释放推理资源；幂等。
    virtual void Unload() noexcept = 0;

    /// 模型资源是否已就绪。
    virtual bool IsLoaded() const = 0;

    /// 对一帧执行推理。
    /// @param frame 解码帧（NV12）；后端自行处理分辨率变化与格式校验。
    /// @return 检测结果列表（原图坐标系）；推理失败返回空列表并计数错误。
    virtual std::vector<BackendDetection> Detect(const video::FrameHandle& frame) = 0;
};

/// 创建默认后端。
/// - 编译时启用 RKNN（DRONE_HAVE_RKNN）且模型路径非空：返回 RknnDetectionBackend。
/// - 否则返回 nullptr（调用方需注入自定义后端，或 YoloDetector::Start 失败）。
std::unique_ptr<IDetectionBackend> CreateDefaultDetectionBackend(
    const std::string& model_path, float conf_threshold, float nms_threshold);

}  // namespace drone::perception
