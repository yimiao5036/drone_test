#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "common/topic.h"
#include "common/types.h"
#include "perception/linear_target_kalman_filter.h"

namespace drone::perception {

/// 单目标估计器配置。第一阶段仅融合地面站目标，并以PX4 Home为局部NED原点。
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

    void Validate() const;
};

/// 目标估计部件接口。第一阶段输入地面站单目标和PX4状态，输出局部NED TargetState。
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

    virtual common::Topic<common::TargetState>& EstimatedOutput() = 0;
    virtual common::TargetState LastState() const = 0;

    /// 累计接受并更新滤波器的地面站目标数。
    virtual uint64_t UpdateCount() const = 0;
    /// 累计拒绝的非法、过期或乱序观测数。
    virtual uint64_t ErrorCount() const = 0;
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

    common::Topic<common::TargetState>& EstimatedOutput() override;
    common::TargetState LastState() const override;

    uint64_t UpdateCount() const override;
    uint64_t ErrorCount() const override;

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

    common::Topic<common::TargetState>& EstimatedOutput() override;
    common::TargetState LastState() const override;

    uint64_t UpdateCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    common::Topic<common::TargetState> estimated_output_;
};

}  // namespace drone::perception
