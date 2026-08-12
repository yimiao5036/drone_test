/**
 * @file target_estimator.cpp
 * @brief ITargetEstimator 骨架占位实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（平滑、预测、目标状态
 * 维护）在实现期接入。
 */
#include "perception/target_estimator.h"

#include <spdlog/spdlog.h>

namespace drone::perception {

TargetEstimatorStub::TargetEstimatorStub() {
    SPDLOG_INFO("目标估计部件骨架创建");
}

TargetEstimatorStub::~TargetEstimatorStub() {
    SPDLOG_INFO("目标估计部件骨架销毁");
}

bool TargetEstimatorStub::Start() {
    running_ = true;
    SPDLOG_INFO("目标估计部件骨架启动");
    return true;
}

void TargetEstimatorStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("目标估计部件骨架停止");
}

bool TargetEstimatorStub::IsRunning() const {
    return running_;
}

void TargetEstimatorStub::SetInputs(common::Topic<common::TargetState>& /*fused*/,
                                    common::Topic<common::GroundStationTarget>& /*ground*/) {
    // 骨架期忽略输入绑定；实现期保存订阅并启动消费
}

common::Topic<common::TargetState>& TargetEstimatorStub::EstimatedOutput() {
    return estimated_output_;
}

uint64_t TargetEstimatorStub::UpdateCount() const {
    return update_count_;
}

uint64_t TargetEstimatorStub::ErrorCount() const {
    return error_count_;
}

}  // namespace drone::perception
