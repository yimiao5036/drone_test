/**
 * @file rknn_detection_backend.h
 * @brief RKNN 推理后端（香橙派 RK3588）
 *
 * 属于 drone/perception 模块，实现 IDetectionBackend。
 * 仅当 CMake 开启 DRONE_HAVE_RKNN（香橙派部署）时编译本文件；
 * 开发机（无 RKNN runtime/RGA）不参与编译，YoloDetector 通过
 * CreateDefaultDetectionBackend 工厂按编译配置创建。
 *
 * 推理链路（对应原型 videoPart/yolo26-rknn）：
 *
 *   NV12 解码帧 ──► RGA 预处理（完整画面等比缩放 + RGB letterbox）
 *              ──► RKNN NPU 推理（3 核上下文，单帧由运行时调度）
 *              ──► 输出张量 NC1HWC2→NCHW 转换（预分配缓冲）
 *              ──► `[1,5,N]` 归一化 xywh 解码 + NMS + letterbox 逆变换
 *              ──► 原图坐标 BackendDetection
 *
 * 依赖（香橙派系统）：
 * - librknnrt.so（RKNN runtime），头文件见 third_party/rknn/include
 * - librga.so 与 /usr/include/rga（RGA 2D 加速）
 */
#pragma once

#include <memory>
#include <string>

#include "perception/detection_backend.h"

namespace drone::perception {

/// RKNN 推理后端：RGA 预处理 + NPU 推理 + 后处理（见文件头注释）。
class RknnDetectionBackend final : public IDetectionBackend {
public:
    /// @param model_path RKNN 模型文件路径（yolo26n_int8.rknn 等）
    /// @param conf_threshold 检测置信度阈值
    /// @param nms_threshold NMS IoU 阈值
    RknnDetectionBackend(std::string model_path, float conf_threshold,
                         float nms_threshold);
    ~RknnDetectionBackend() override;

    RknnDetectionBackend(const RknnDetectionBackend&) = delete;
    RknnDetectionBackend& operator=(const RknnDetectionBackend&) = delete;

    bool Load() override;
    void Unload() noexcept override;
    bool IsLoaded() const override;

    std::vector<BackendDetection> Detect(const video::FrameHandle& frame) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace drone::perception
