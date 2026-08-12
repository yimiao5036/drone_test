/**
 * @file target_estimator.h
 * @brief 目标估计部件接口（ITargetEstimator）
 *
 * 属于 drone/perception 模块。职责：融合感知目标与地面站目标，
 * 平滑、预测并维护目标状态（卡尔曼或其他估计器），输出可供
 * 状态机/控制直接使用的 TargetState。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，估计器算法在实现期接入。
 * - TargetEstimatorStub 为骨架占位实现：生命周期可运行，业务方法
 *   记录"未实现"节流日志并返回默认值。
 *
 * 数据流：
 *   common::Topic<TargetState>（感知融合输出）──┐
 *   common::Topic<GroundStationTarget>（地面站）─┼──► ITargetEstimator ──► common::Topic<TargetState>
 * 可替换边界：卡尔曼或其他估计器。
 */
#pragma once

#include <cstdint>

#include "common/topic.h"
#include "common/types.h"

namespace drone::perception {

/// 目标估计部件抽象接口。
class ITargetEstimator {
public:
    virtual ~ITargetEstimator() = default;

    // ---- 生命周期 ----
    /// 启动估计（启动消费线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止估计；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;

    // ---- 输入 ----
    /// 绑定融合目标与地面站目标输入主题。
    virtual void SetInputs(common::Topic<common::TargetState>& fused,
                           common::Topic<common::GroundStationTarget>& ground) = 0;

    // ---- 输出 ----
    /// 估计目标状态输出主题：common::TargetState。
    virtual common::Topic<common::TargetState>& EstimatedOutput() = 0;

    // ---- 状态查询 ----
    /// 累计更新次数。
    virtual uint64_t UpdateCount() const = 0;
    /// 累计错误次数。
    virtual uint64_t ErrorCount() const = 0;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class TargetEstimatorStub final : public ITargetEstimator {
public:
    TargetEstimatorStub();
    ~TargetEstimatorStub() override;

    TargetEstimatorStub(const TargetEstimatorStub&) = delete;
    TargetEstimatorStub& operator=(const TargetEstimatorStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInputs(common::Topic<common::TargetState>& fused,
                   common::Topic<common::GroundStationTarget>& ground) override;

    common::Topic<common::TargetState>& EstimatedOutput() override;

    uint64_t UpdateCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t update_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::TargetState> estimated_output_;
};

}  // namespace drone::perception
