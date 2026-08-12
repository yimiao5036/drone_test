/**
 * @file health_manager.h
 * @brief 健康管理部件接口（IHealthManager）
 *
 * 属于 drone/health 模块。职责：监控各链路与设备的数据新鲜度与健康状态，
 * 汇总发布 HealthStatus，供状态机决策与地面站回传使用。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，健康规则（新鲜度阈值、告警位定义）在实现期接入。
 * - HealthManagerStub 为骨架占位实现：生命周期可运行，业务方法
 *   记录"未实现"节流日志并返回默认值。
 *
 * 数据流：各数据源上报（RegisterSource/ReportData）──► IHealthManager ──► common::Topic<HealthStatus>
 * 可替换边界：健康规则。
 */
#pragma once

#include <cstdint>
#include <string>

#include "common/topic.h"
#include "common/types.h"

namespace drone::health {

/// 健康管理部件抽象接口。
class IHealthManager {
public:
    virtual ~IHealthManager() = default;

    // ---- 生命周期 ----
    /// 启动健康监控（启动监控周期线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止健康监控；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;

    // ---- 监控 ----
    /// 注册监控数据源。
    /// @param name 数据源名称（与 HealthStatus 位定义对应）。
    /// @param max_age_ms 数据超时阈值；超过即视为数据超时。
    /// @return 是否注册成功（重名或非法参数返回 false）。
    virtual bool RegisterSource(const std::string& name, uint64_t max_age_ms) = 0;

    /// 上报数据源最新数据时间（单调时钟毫秒）。
    /// @param name 已注册的数据源名称。
    /// @param receive_time_ms 最近一次数据到达时间。
    virtual void ReportData(const std::string& name, uint64_t receive_time_ms) = 0;

    // ---- 输出 ----
    /// 健康状态输出主题：common::HealthStatus。
    virtual common::Topic<common::HealthStatus>& Output() = 0;

    // ---- 状态查询 ----
    /// 累计数据超时事件次数。
    virtual uint64_t TimeoutEventCount() const = 0;
    /// 累计错误次数。
    virtual uint64_t ErrorCount() const = 0;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class HealthManagerStub final : public IHealthManager {
public:
    HealthManagerStub();
    ~HealthManagerStub() override;

    HealthManagerStub(const HealthManagerStub&) = delete;
    HealthManagerStub& operator=(const HealthManagerStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    bool RegisterSource(const std::string& name, uint64_t max_age_ms) override;
    void ReportData(const std::string& name, uint64_t receive_time_ms) override;

    common::Topic<common::HealthStatus>& Output() override;

    uint64_t TimeoutEventCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t timeout_event_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::HealthStatus> output_;
};

}  // namespace drone::health
