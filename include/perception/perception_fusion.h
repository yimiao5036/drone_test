/**
 * @file perception_fusion.h
 * @brief 感知融合部件接口（IPerceptionFusion）
 *
 * 属于 drone/perception 模块。职责：对齐视觉检测、光流、雷达距离和
 * 机体姿态（来自飞行状态快照），输出融合后的目标状态（TargetState）。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，融合算法在实现期接入。
 * - PerceptionFusionStub 为骨架占位实现：生命周期可运行，业务方法
 *   记录"未实现"节流日志并返回默认值。
 *
 * 数据流：
 *   common::Topic<DetectionResult> ─┐
 *   common::Topic<OpticalFlowResult>─┤
 *   common::Topic<LaserRangeSample> ─┼──► IPerceptionFusion ──► common::Topic<TargetState>
 *   common::Topic<FlightStateSnapshot>┘
 * 可替换边界：融合算法。
 */
#pragma once

#include <cstdint>

#include "common/topic.h"
#include "common/types.h"

namespace drone::perception {

/// 感知融合部件抽象接口。
class IPerceptionFusion {
public:
    virtual ~IPerceptionFusion() = default;

    // ---- 生命周期 ----
    /// 启动融合（启动消费线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止融合；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;

    // ---- 输入 ----
    /// 绑定各感知输入主题。
    virtual void SetInputs(common::Topic<common::DetectionResult>& detection,
                           common::Topic<common::OpticalFlowResult>& flow,
                           common::Topic<common::LaserRangeSample>& range,
                           common::Topic<common::FlightStateSnapshot>& flight) = 0;

    // ---- 输出 ----
    /// 融合目标状态输出主题：common::TargetState。
    virtual common::Topic<common::TargetState>& TargetOutput() = 0;

    // ---- 状态查询 ----
    /// 累计融合输出次数。
    virtual uint64_t FusionCount() const = 0;
    /// 累计错误次数。
    virtual uint64_t ErrorCount() const = 0;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class PerceptionFusionStub final : public IPerceptionFusion {
public:
    PerceptionFusionStub();
    ~PerceptionFusionStub() override;

    PerceptionFusionStub(const PerceptionFusionStub&) = delete;
    PerceptionFusionStub& operator=(const PerceptionFusionStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInputs(common::Topic<common::DetectionResult>& detection,
                   common::Topic<common::OpticalFlowResult>& flow,
                   common::Topic<common::LaserRangeSample>& range,
                   common::Topic<common::FlightStateSnapshot>& flight) override;

    common::Topic<common::TargetState>& TargetOutput() override;

    uint64_t FusionCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t fusion_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::TargetState> target_output_;
};

}  // namespace drone::perception
