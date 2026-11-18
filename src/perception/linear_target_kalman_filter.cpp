#include "perception/linear_target_kalman_filter.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone::perception {
namespace {

constexpr double kMaximumPredictionStepSeconds = 10.0;

bool IsPositiveFinite(double value) {
    return std::isfinite(value) && value > 0.0;
}

}  // namespace

LinearTargetKalmanFilter::LinearTargetKalmanFilter(
    LinearTargetKalmanConfig config)
    : config_(config) {
    if (!IsPositiveFinite(config_.process_acceleration_std_mps2) ||
        !IsPositiveFinite(config_.initial_velocity_std_mps) ||
        !IsPositiveFinite(config_.velocity_measurement_std_mps) ||
        !IsPositiveFinite(config_.minimum_measurement_std_m)) {
        throw std::invalid_argument("线性目标卡尔曼参数必须为有限正数");
    }
}

void LinearTargetKalmanFilter::Reset() {
    north_ = AxisFilter{};
    east_ = AxisFilter{};
    down_ = AxisFilter{};
}

bool LinearTargetKalmanFilter::IsInitialized() const {
    return north_.initialized && east_.initialized;
}

bool LinearTargetKalmanFilter::ValidateMeasurement(
    const LinearTargetMeasurement& measurement) const {
    if (!std::isfinite(measurement.north_m) ||
        !std::isfinite(measurement.east_m) ||
        !IsPositiveFinite(measurement.horizontal_position_std_m)) {
        return false;
    }
    if (measurement.down_valid &&
        (!std::isfinite(measurement.down_m) ||
         !IsPositiveFinite(measurement.vertical_position_std_m))) {
        return false;
    }
    if ((measurement.velocity_north_valid &&
         !std::isfinite(measurement.velocity_north_mps)) ||
        (measurement.velocity_east_valid &&
         !std::isfinite(measurement.velocity_east_mps)) ||
        (measurement.velocity_down_valid &&
         !std::isfinite(measurement.velocity_down_mps))) {
        return false;
    }
    return true;
}

void LinearTargetKalmanFilter::InitializeAxis(
    AxisFilter& axis, double position, double position_variance,
    bool velocity_valid, double velocity) {
    axis.initialized = true;
    axis.position = position;
    axis.velocity = velocity_valid ? velocity : 0.0;
    axis.p00 = position_variance;
    axis.p01 = 0.0;
    axis.p10 = 0.0;
    const double velocity_std = velocity_valid
                                    ? config_.velocity_measurement_std_mps
                                    : config_.initial_velocity_std_mps;
    axis.p11 = velocity_std * velocity_std;
}

bool LinearTargetKalmanFilter::Initialize(
    const LinearTargetMeasurement& measurement) {
    if (!ValidateMeasurement(measurement)) {
        return false;
    }

    Reset();
    const double horizontal_std = std::max(
        measurement.horizontal_position_std_m,
        config_.minimum_measurement_std_m);
    const double horizontal_variance = horizontal_std * horizontal_std;
    InitializeAxis(north_, measurement.north_m, horizontal_variance,
                   measurement.velocity_north_valid,
                   measurement.velocity_north_mps);
    InitializeAxis(east_, measurement.east_m, horizontal_variance,
                   measurement.velocity_east_valid,
                   measurement.velocity_east_mps);

    if (measurement.down_valid) {
        const double vertical_std = std::max(
            measurement.vertical_position_std_m,
            config_.minimum_measurement_std_m);
        InitializeAxis(down_, measurement.down_m, vertical_std * vertical_std,
                       measurement.velocity_down_valid,
                       measurement.velocity_down_mps);
    }
    return true;
}

void LinearTargetKalmanFilter::PredictAxis(AxisFilter& axis, double dt_seconds) {
    if (!axis.initialized || dt_seconds == 0.0) {
        return;
    }

    axis.position += axis.velocity * dt_seconds;

    // 白噪声加速度恒速度模型：Q = sigma_a^2 * [[dt^4/4, dt^3/2],
    //                                                   [dt^3/2, dt^2]]。
    const double dt2 = dt_seconds * dt_seconds;
    const double dt3 = dt2 * dt_seconds;
    const double dt4 = dt2 * dt2;
    const double acceleration_variance =
        config_.process_acceleration_std_mps2 *
        config_.process_acceleration_std_mps2;
    const double q00 = acceleration_variance * dt4 * 0.25;
    const double q01 = acceleration_variance * dt3 * 0.5;
    const double q11 = acceleration_variance * dt2;

    const double old_p00 = axis.p00;
    const double old_p01 = axis.p01;
    const double old_p10 = axis.p10;
    const double old_p11 = axis.p11;
    axis.p00 = old_p00 + dt_seconds * (old_p10 + old_p01) +
               dt2 * old_p11 + q00;
    axis.p01 = old_p01 + dt_seconds * old_p11 + q01;
    axis.p10 = old_p10 + dt_seconds * old_p11 + q01;
    axis.p11 = old_p11 + q11;
}

bool LinearTargetKalmanFilter::Predict(double dt_seconds) {
    if (!IsInitialized() || !std::isfinite(dt_seconds) || dt_seconds < 0.0 ||
        dt_seconds > kMaximumPredictionStepSeconds) {
        return false;
    }
    PredictAxis(north_, dt_seconds);
    PredictAxis(east_, dt_seconds);
    PredictAxis(down_, dt_seconds);
    return true;
}

void LinearTargetKalmanFilter::UpdatePosition(AxisFilter& axis, double position,
                                              double variance) {
    if (!axis.initialized) {
        return;
    }
    const double innovation_variance = axis.p00 + variance;
    if (!IsPositiveFinite(innovation_variance)) {
        return;
    }
    const double gain_position = axis.p00 / innovation_variance;
    const double gain_velocity = axis.p10 / innovation_variance;
    const double innovation = position - axis.position;
    axis.position += gain_position * innovation;
    axis.velocity += gain_velocity * innovation;

    const double old_p00 = axis.p00;
    const double old_p01 = axis.p01;
    const double old_p10 = axis.p10;
    const double old_p11 = axis.p11;
    axis.p00 = (1.0 - gain_position) * old_p00;
    axis.p01 = (1.0 - gain_position) * old_p01;
    axis.p10 = old_p10 - gain_velocity * old_p00;
    axis.p11 = old_p11 - gain_velocity * old_p01;
    // 抑制浮点误差造成的非对称和微小负方差。
    const double cross = 0.5 * (axis.p01 + axis.p10);
    axis.p01 = cross;
    axis.p10 = cross;
    axis.p00 = std::max(axis.p00, 0.0);
    axis.p11 = std::max(axis.p11, 0.0);
}

void LinearTargetKalmanFilter::UpdateVelocity(AxisFilter& axis, double velocity,
                                              double variance) {
    if (!axis.initialized) {
        return;
    }
    const double innovation_variance = axis.p11 + variance;
    if (!IsPositiveFinite(innovation_variance)) {
        return;
    }
    const double gain_position = axis.p01 / innovation_variance;
    const double gain_velocity = axis.p11 / innovation_variance;
    const double innovation = velocity - axis.velocity;
    axis.position += gain_position * innovation;
    axis.velocity += gain_velocity * innovation;

    const double old_p00 = axis.p00;
    const double old_p01 = axis.p01;
    const double old_p10 = axis.p10;
    const double old_p11 = axis.p11;
    axis.p00 = old_p00 - gain_position * old_p10;
    axis.p01 = old_p01 - gain_position * old_p11;
    axis.p10 = (1.0 - gain_velocity) * old_p10;
    axis.p11 = (1.0 - gain_velocity) * old_p11;
    const double cross = 0.5 * (axis.p01 + axis.p10);
    axis.p01 = cross;
    axis.p10 = cross;
    axis.p00 = std::max(axis.p00, 0.0);
    axis.p11 = std::max(axis.p11, 0.0);
}

bool LinearTargetKalmanFilter::Update(
    const LinearTargetMeasurement& measurement) {
    if (!IsInitialized() || !ValidateMeasurement(measurement)) {
        return false;
    }

    const double horizontal_std = std::max(
        measurement.horizontal_position_std_m,
        config_.minimum_measurement_std_m);
    const double horizontal_variance = horizontal_std * horizontal_std;
    UpdatePosition(north_, measurement.north_m, horizontal_variance);
    UpdatePosition(east_, measurement.east_m, horizontal_variance);

    if (measurement.down_valid) {
        const double vertical_std = std::max(
            measurement.vertical_position_std_m,
            config_.minimum_measurement_std_m);
        const double vertical_variance = vertical_std * vertical_std;
        if (!down_.initialized) {
            InitializeAxis(down_, measurement.down_m, vertical_variance,
                           measurement.velocity_down_valid,
                           measurement.velocity_down_mps);
        } else {
            UpdatePosition(down_, measurement.down_m, vertical_variance);
        }
    }

    const double velocity_variance =
        config_.velocity_measurement_std_mps *
        config_.velocity_measurement_std_mps;
    if (measurement.velocity_north_valid) {
        UpdateVelocity(north_, measurement.velocity_north_mps,
                       velocity_variance);
    }
    if (measurement.velocity_east_valid) {
        UpdateVelocity(east_, measurement.velocity_east_mps,
                       velocity_variance);
    }
    if (measurement.velocity_down_valid && down_.initialized) {
        UpdateVelocity(down_, measurement.velocity_down_mps,
                       velocity_variance);
    }
    return true;
}

LinearTargetKalmanState LinearTargetKalmanFilter::BuildState() const {
    LinearTargetKalmanState state;
    state.horizontal_valid = IsInitialized();
    state.vertical_valid = down_.initialized;
    if (north_.initialized) {
        state.north_m = north_.position;
        state.velocity_north_mps = north_.velocity;
    }
    if (east_.initialized) {
        state.east_m = east_.position;
        state.velocity_east_mps = east_.velocity;
    }
    if (down_.initialized) {
        state.down_m = down_.position;
        state.velocity_down_mps = down_.velocity;
    }
    if (state.horizontal_valid) {
        state.horizontal_position_std_m =
            std::sqrt(std::max(north_.p00 + east_.p00, 0.0));
    }
    if (state.vertical_valid) {
        state.vertical_position_std_m =
            std::sqrt(std::max(down_.p00, 0.0));
    }
    return state;
}

LinearTargetKalmanState LinearTargetKalmanFilter::State() const {
    return BuildState();
}

LinearTargetKalmanState LinearTargetKalmanFilter::PredictedState(
    double dt_seconds) const {
    LinearTargetKalmanFilter copy = *this;
    if (!copy.Predict(dt_seconds)) {
        return LinearTargetKalmanState{};
    }
    return copy.BuildState();
}

}  // namespace drone::perception
