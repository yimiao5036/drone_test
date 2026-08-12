/**
 * @file laser_range_finder.h
 * @brief 激光测距雷达部件接口（ILaserRangeFinder）
 *
 * 属于 drone/perception 模块。职责：串口读取、校验和时间标记单点距离，
 * 发布原始滤波前数据（LaserRangeSample）。雷达表达机头前向单点距离，
 * 不得自动解释为任意视觉目标的距离。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，雷达型号协议适配（串口参数、厂商协议）在实现期接入。
 * - LaserRangeFinderStub 为骨架占位实现：生命周期可运行，业务方法
 *   记录"未实现"节流日志并返回默认值。
 *
 * 数据流：雷达串口 ──► ILaserRangeFinder ──► common::Topic<LaserRangeSample> ──► 感知融合
 * 可替换边界：雷达型号适配器。
 */
#pragma once

#include <cstdint>

#include "common/topic.h"
#include "common/types.h"

namespace drone::perception {

/// 激光测距雷达部件抽象接口。
class ILaserRangeFinder {
public:
    virtual ~ILaserRangeFinder() = default;

    // ---- 生命周期 ----
    /// 启动测距（打开串口、启动读取线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止测距并关闭串口；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;

    // ---- 输出 ----
    /// 距离采样输出主题：common::LaserRangeSample。
    virtual common::Topic<common::LaserRangeSample>& RangeOutput() = 0;

    // ---- 状态查询 ----
    /// 累计有效采样数。
    virtual uint64_t SampleCount() const = 0;
    /// 累计错误次数（校验失败、串口错误等）。
    virtual uint64_t ErrorCount() const = 0;
    /// 最近一次有效距离（米）；从未收到有效数据时为 0。
    virtual float LastDistanceM() const = 0;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class LaserRangeFinderStub final : public ILaserRangeFinder {
public:
    LaserRangeFinderStub();
    ~LaserRangeFinderStub() override;

    LaserRangeFinderStub(const LaserRangeFinderStub&) = delete;
    LaserRangeFinderStub& operator=(const LaserRangeFinderStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    common::Topic<common::LaserRangeSample>& RangeOutput() override;

    uint64_t SampleCount() const override;
    uint64_t ErrorCount() const override;
    float LastDistanceM() const override;

private:
    bool running_ = false;
    uint64_t sample_count_ = 0;
    uint64_t error_count_ = 0;
    float last_distance_m_ = 0.f;
    common::Topic<common::LaserRangeSample> range_output_;
};

}  // namespace drone::perception
