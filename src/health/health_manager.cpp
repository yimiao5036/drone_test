/**
 * @file health_manager.cpp
 * @brief IHealthManager 骨架占位实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（数据源注册表、新鲜度
 * 检查、健康汇总与告警位定义）在实现期接入。
 */
#include "health/health_manager.h"

#include <cstdint>
#include <string>

#include <spdlog/spdlog.h>

namespace drone::health {

namespace {

// 数据流：各数据源 RegisterSource/ReportData → 新鲜度检查 → HealthStatus Topic。
// Stub 未实现注册与上报，只占位生命周期与健康输出主题，供装配验证。

/// 异常日志节流：第 1 次与每满 100 次才打印，避免高频异常刷屏。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

}  // namespace

HealthManagerStub::HealthManagerStub() {
    SPDLOG_INFO("健康管理部件骨架创建");
}

HealthManagerStub::~HealthManagerStub() {
    SPDLOG_INFO("健康管理部件骨架销毁");
}

// 启动：骨架无数据源，仅置位运行标志。
bool HealthManagerStub::Start() {
    running_ = true;
    SPDLOG_INFO("健康管理部件骨架启动");
    return true;
}

// 停止：幂等，仅在已启动时置位。
void HealthManagerStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("健康管理部件骨架停止");
}

bool HealthManagerStub::IsRunning() const {
    return running_;
}

// 骨架期未实现数据源注册，返回失败并节流记录。
bool HealthManagerStub::RegisterSource(const std::string& /*name*/,
                                       uint64_t /*max_age_ms*/) {
    ++error_count_;
    if (ShouldLogThrottled(error_count_)) {
        SPDLOG_WARN("健康管理部件 RegisterSource 未实现（骨架占位），累计调用 {}", error_count_);
    }
    return false;
}

// 骨架期未实现数据上报，节流记录。
void HealthManagerStub::ReportData(const std::string& /*name*/,
                                   uint64_t /*receive_time_ms*/) {
    ++error_count_;
    if (ShouldLogThrottled(error_count_)) {
        SPDLOG_WARN("健康管理部件 ReportData 未实现（骨架占位），累计调用 {}", error_count_);
    }
}

common::Topic<common::HealthStatus>& HealthManagerStub::Output() {
    return output_;
}

uint64_t HealthManagerStub::TimeoutEventCount() const {
    return timeout_event_count_;
}

uint64_t HealthManagerStub::ErrorCount() const {
    return error_count_;
}

}  // namespace drone::health
