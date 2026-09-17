#pragma once

#include <cstdint>

namespace drone::perception {

/// 单目标恒速度线性卡尔曼参数。第一阶段仅影子运行，参数必须通过录制数据再整定。
struct LinearTargetKalmanConfig {
    double process_acceleration_std_mps2 = 8.0;
    double initial_velocity_std_mps = 20.0;
    double velocity_measurement_std_mps = 5.0;
    double minimum_measurement_std_m = 0.5;
};

/// 一次已转换到局部NED的目标观测。
struct LinearTargetMeasurement {
    double north_m = 0.0;
    double east_m = 0.0;
    double down_m = 0.0;
    bool down_valid = false;

    double velocity_north_mps = 0.0;
    double velocity_east_mps = 0.0;
    double velocity_down_mps = 0.0;
    bool velocity_north_valid = false;
    bool velocity_east_valid = false;
    bool velocity_down_valid = false;

    double horizontal_position_std_m = 1.0;
    double vertical_position_std_m = 1.0;
};

/// 线性卡尔曼状态快照，坐标系为局部NED。
struct LinearTargetKalmanState {
    double north_m = 0.0;
    double east_m = 0.0;
    double down_m = 0.0;
    double velocity_north_mps = 0.0;
    double velocity_east_mps = 0.0;
    double velocity_down_mps = 0.0;
    bool horizontal_valid = false;
    bool vertical_valid = false;
    double horizontal_position_std_m = 0.0;
    double vertical_position_std_m = 0.0;
};

/// 六维[N,E,D,VN,VE,VD]恒速度线性卡尔曼。
/// 实现按N/E/D三个相互独立的[position, velocity]二状态滤波器计算，
/// 等价于忽略轴间相关项的块对角六维模型。
class LinearTargetKalmanFilter final {
public:
    explicit LinearTargetKalmanFilter(
        LinearTargetKalmanConfig config = LinearTargetKalmanConfig{});

    void Reset();
    bool IsInitialized() const;

    /// 使用首个位置观测初始化；水平位置必须有效，垂直轴可选。
    bool Initialize(const LinearTargetMeasurement& measurement);

    /// 从当前状态向前预测dt秒；dt必须有限且位于[0, 10]。
    bool Predict(double dt_seconds);

    /// 在当前状态时刻执行位置及可选速度更新。
    bool Update(const LinearTargetMeasurement& measurement);

    LinearTargetKalmanState State() const;

    /// 不修改当前滤波器，返回向前预测dt秒后的快照。
    LinearTargetKalmanState PredictedState(double dt_seconds) const;

private:
    struct AxisFilter {
        bool initialized = false;
        double position = 0.0;
        double velocity = 0.0;
        double p00 = 0.0;
        double p01 = 0.0;
        double p10 = 0.0;
        double p11 = 0.0;
    };

    bool ValidateMeasurement(const LinearTargetMeasurement& measurement) const;
    void InitializeAxis(AxisFilter& axis, double position, double position_variance,
                        bool velocity_valid, double velocity);
    void PredictAxis(AxisFilter& axis, double dt_seconds);
    void UpdatePosition(AxisFilter& axis, double position, double variance);
    void UpdateVelocity(AxisFilter& axis, double velocity, double variance);
    LinearTargetKalmanState BuildState() const;

    LinearTargetKalmanConfig config_;
    AxisFilter north_;
    AxisFilter east_;
    AxisFilter down_;
};

}  // namespace drone::perception
