/**
 * @file optical_flow_estimator.cpp
 * @brief IOpticalFlowEstimator 骨架占位实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（光流算法、质量评估）
 * 在实现期接入。
 */
#include "perception/optical_flow_estimator.h"

#include <spdlog/spdlog.h>

namespace drone::perception {

OpticalFlowEstimatorStub::OpticalFlowEstimatorStub() {
    SPDLOG_INFO("光流估计部件骨架创建");
}

OpticalFlowEstimatorStub::~OpticalFlowEstimatorStub() {
    SPDLOG_INFO("光流估计部件骨架销毁");
}

bool OpticalFlowEstimatorStub::Start() {
    running_ = true;
    SPDLOG_INFO("光流估计部件骨架启动");
    return true;
}

void OpticalFlowEstimatorStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("光流估计部件骨架停止");
}

bool OpticalFlowEstimatorStub::IsRunning() const {
    return running_;
}

void OpticalFlowEstimatorStub::SetInput(common::Topic<video::FrameHandle>& /*input*/) {
    // 骨架期忽略输入绑定；实现期保存订阅并启动消费
}

common::Topic<common::OpticalFlowResult>& OpticalFlowEstimatorStub::FlowOutput() {
    return flow_output_;
}

uint64_t OpticalFlowEstimatorStub::ProcessedFrameCount() const {
    return processed_count_;
}

uint64_t OpticalFlowEstimatorStub::ErrorCount() const {
    return error_count_;
}

}  // namespace drone::perception
