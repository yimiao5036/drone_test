#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "common/topic.h"
#include "common/types.h"
#include "perception/linear_target_kalman_filter.h"
#include "perception/visual_ned_converter.h"

namespace drone::perception {

/// 双目视觉观测融合配置（第二阶段，影子）。仅 enabled=true 时生效；
/// 有效期语义：视觉观测只在地面站目标有效期内细化估计，不延展有效期
/// （过期政策归状态机，见 docs/状态机设计.md 与 docs/物理追踪思路.md §6.4）。
struct VisualFusionConfig {
    bool enabled = false;  ///< 由 runtime.enable_visual_position_fusion 装配期写入
    std::size_t queue_capacity = 8;  ///< 视觉订阅队列容量（丢最旧）
    std::chrono::milliseconds max_observation_age_ms{200};   ///< source_time 超龄拒绝
    std::chrono::milliseconds flight_state_max_age_ms{150};  ///< PX4 快照超龄门禁
    double disparity_std_px = 0.5;  ///< σ 模型：视差误差 std（σ_z=z²·σ_d/fxb）
    double min_confidence = 0.0;    ///< 观测置信度下限（0=不过滤）
    double fxb_m_px = 0.0;  ///< 深度系数，装配期从双目校正矩阵拷贝（单一真源）；
                            ///< enabled=true 时必须为正，否则 Validate 拒绝
    VisualMountExtrinsics mount;  ///< 相机安装外参；默认全 0=准直安装假设（未标定）

    void Validate() const;
};

/// 单目标估计器配置。融合地面站目标并以PX4 Home为局部NED原点；
/// 可选接入双目视觉观测（visual.enabled），仍只发布影子 TargetState。
struct TargetEstimatorConfig {
    std::size_t ground_target_queue_capacity = 4;
    std::size_t flight_state_queue_capacity = 2;
    std::chrono::milliseconds publish_interval{100};
    double process_acceleration_std_mps2 = 8.0;
    double initial_velocity_std_mps = 20.0;
    double velocity_measurement_std_mps = 5.0;
    double default_horizontal_accuracy_m = 10.0;
    double default_vertical_accuracy_m = 15.0;
    double minimum_measurement_std_m = 0.5;
    VisualFusionConfig visual;

    void Validate() const;
};

/// 目标估计部件接口。输入地面站单目标和PX4状态（可选：双目视觉距离），
/// 输出局部NED TargetState。
class ITargetEstimator {
public:
    virtual ~ITargetEstimator() = default;

    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual bool IsRunning() const = 0;

    virtual void SetGroundTargetInput(
        common::Topic<common::GroundStationTarget>& ground) = 0;
    virtual void SetFlightStateInput(
        common::Topic<common::FlightStateSnapshot>& flight) = 0;
    /// 双目视觉距离输入（可选；visual.enabled=true 时 Start 前必须接线，
    /// 否则启动失败）。
    virtual void SetVisualDistanceInput(
        common::Topic<common::StereoTargetDistance>& visual) = 0;

    virtual common::Topic<common::TargetState>& EstimatedOutput() = 0;
    virtual common::TargetState LastState() const = 0;

    /// 累计接受并更新滤波器的地面站目标数。
    virtual uint64_t UpdateCount() const = 0;
    /// 累计接受的视觉观测更新数。
    virtual uint64_t VisualUpdateCount() const = 0;
    /// 累计拒绝的非法、过期或乱序观测数。
    virtual uint64_t ErrorCount() const = 0;

    /// 最近一次接受的视觉观测与卡尔曼预测的水平创新距离（米，诊断接口）。
    /// 语义：观测历元处的预测位置到观测位置的 N/E 水平距离（更新前差异），
    /// 供交接判据的"创新一致性"门限消费（阶段四 Task 8，防错锁伪目标）。
    /// 无有效视觉更新（从未接受、滤波器重置/重新初始化后）返回 NaN——
    /// 调用方必须把 NaN 当作"无值"而不是"一致"。
    virtual double LastVisualInnovationM() const = 0;
};

/// 正式单目标线性卡尔曼估计器。仅发布影子TargetState，不产生ControlIntent/Px4Setpoint。
class TargetEstimator final : public ITargetEstimator {
public:
    explicit TargetEstimator(TargetEstimatorConfig config = TargetEstimatorConfig{});
    ~TargetEstimator() override;

    TargetEstimator(const TargetEstimator&) = delete;
    TargetEstimator& operator=(const TargetEstimator&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetGroundTargetInput(
        common::Topic<common::GroundStationTarget>& ground) override;
    void SetFlightStateInput(
        common::Topic<common::FlightStateSnapshot>& flight) override;
    void SetVisualDistanceInput(
        common::Topic<common::StereoTargetDistance>& visual) override;

    common::Topic<common::TargetState>& EstimatedOutput() override;
    common::TargetState LastState() const override;

    uint64_t UpdateCount() const override;
    uint64_t VisualUpdateCount() const override;
    uint64_t ErrorCount() const override;
    double LastVisualInnovationM() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// 保留骨架实现供接口冒烟测试；正式DroneApplication不再使用该类。
class TargetEstimatorStub final : public ITargetEstimator {
public:
    TargetEstimatorStub();
    ~TargetEstimatorStub() override;

    TargetEstimatorStub(const TargetEstimatorStub&) = delete;
    TargetEstimatorStub& operator=(const TargetEstimatorStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetGroundTargetInput(
        common::Topic<common::GroundStationTarget>& ground) override;
    void SetFlightStateInput(
        common::Topic<common::FlightStateSnapshot>& flight) override;
    void SetVisualDistanceInput(
        common::Topic<common::StereoTargetDistance>& visual) override;

    common::Topic<common::TargetState>& EstimatedOutput() override;
    common::TargetState LastState() const override;

    uint64_t UpdateCount() const override;
    uint64_t VisualUpdateCount() const override;
    uint64_t ErrorCount() const override;
    double LastVisualInnovationM() const override;

private:
    bool running_ = false;
    common::Topic<common::TargetState> estimated_output_;
};

}  // namespace drone::perception
