/**
 * @file yolo_detector_stub.cpp
 * @brief IYoloDetector 骨架占位实现（YoloDetectorStub）
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（RKNN 推理、RGA 预处理、
 * 检测结果后处理）由 YoloDetector（yolo_detector.cpp）承担。
 * Stub 保留用于骨架冒烟测试与无硬件场景的装配验证。
 */
#include "perception/yolo_detector.h"

#include <spdlog/spdlog.h>

namespace drone::perception {

YoloDetectorStub::YoloDetectorStub() {
    SPDLOG_INFO("YOLO 识别部件骨架创建");
}

YoloDetectorStub::~YoloDetectorStub() {
    SPDLOG_INFO("YOLO 识别部件骨架销毁");
}

bool YoloDetectorStub::Start() {
    running_ = true;
    SPDLOG_INFO("YOLO 识别部件骨架启动");
    return true;
}

void YoloDetectorStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("YOLO 识别部件骨架停止");
}

bool YoloDetectorStub::IsRunning() const {
    return running_;
}

void YoloDetectorStub::SetInput(common::Topic<video::FrameHandle>& /*input*/) {
    // 骨架期忽略输入绑定；实现期保存订阅并启动消费
}

common::Topic<common::DetectionResult>& YoloDetectorStub::DetectionOutput() {
    return detection_output_;
}

uint64_t YoloDetectorStub::ProcessedFrameCount() const {
    return processed_count_;
}

float YoloDetectorStub::InferenceTimeMsAvg() const {
    return inference_time_ms_avg_;
}

uint64_t YoloDetectorStub::ErrorCount() const {
    return error_count_;
}

}  // namespace drone::perception
