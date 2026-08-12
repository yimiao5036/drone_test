/**
 * @file perception_fusion.cpp
 * @brief IPerceptionFusion 骨架占位实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（多源时间对齐、融合算法）
 * 在实现期接入。
 */
#include "perception/perception_fusion.h"

#include <spdlog/spdlog.h>

namespace drone::perception {

PerceptionFusionStub::PerceptionFusionStub() {
    SPDLOG_INFO("感知融合部件骨架创建");
}

PerceptionFusionStub::~PerceptionFusionStub() {
    SPDLOG_INFO("感知融合部件骨架销毁");
}

bool PerceptionFusionStub::Start() {
    running_ = true;
    SPDLOG_INFO("感知融合部件骨架启动");
    return true;
}

void PerceptionFusionStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("感知融合部件骨架停止");
}

bool PerceptionFusionStub::IsRunning() const {
    return running_;
}

void PerceptionFusionStub::SetInputs(common::Topic<common::DetectionResult>& /*detection*/,
                                     common::Topic<common::OpticalFlowResult>& /*flow*/,
                                     common::Topic<common::LaserRangeSample>& /*range*/,
                                     common::Topic<common::FlightStateSnapshot>& /*flight*/) {
    // 骨架期忽略输入绑定；实现期保存订阅并启动消费
}

common::Topic<common::TargetState>& PerceptionFusionStub::TargetOutput() {
    return target_output_;
}

uint64_t PerceptionFusionStub::FusionCount() const {
    return fusion_count_;
}

uint64_t PerceptionFusionStub::ErrorCount() const {
    return error_count_;
}

}  // namespace drone::perception
