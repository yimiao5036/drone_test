/**
 * @file laser_range_finder.cpp
 * @brief ILaserRangeFinder 骨架占位实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（串口读取、厂商协议
 * 解析、校验与时间标记）在实现期接入。
 */
#include "perception/laser_range_finder.h"

#include <spdlog/spdlog.h>

namespace drone::perception {

LaserRangeFinderStub::LaserRangeFinderStub() {
    SPDLOG_INFO("激光测距部件骨架创建");
}

LaserRangeFinderStub::~LaserRangeFinderStub() {
    SPDLOG_INFO("激光测距部件骨架销毁");
}

bool LaserRangeFinderStub::Start() {
    running_ = true;
    SPDLOG_INFO("激光测距部件骨架启动");
    return true;
}

void LaserRangeFinderStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("激光测距部件骨架停止");
}

bool LaserRangeFinderStub::IsRunning() const {
    return running_;
}

common::Topic<common::LaserRangeSample>& LaserRangeFinderStub::RangeOutput() {
    return range_output_;
}

uint64_t LaserRangeFinderStub::SampleCount() const {
    return sample_count_;
}

uint64_t LaserRangeFinderStub::ErrorCount() const {
    return error_count_;
}

float LaserRangeFinderStub::LastDistanceM() const {
    return last_distance_m_;
}

}  // namespace drone::perception
