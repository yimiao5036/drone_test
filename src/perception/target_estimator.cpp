#include "perception/target_estimator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

#include "perception/geodetic_converter.h"

namespace drone::perception {
namespace {

constexpr uint32_t kAltitudeValid = 1U << 0U;
constexpr uint32_t kVelocityNorthValid = 1U << 1U;
constexpr uint32_t kVelocityEastValid = 1U << 2U;
constexpr uint32_t kVelocityDownValid = 1U << 3U;
constexpr uint32_t kHorizontalAccuracyValid = 1U << 5U;
constexpr uint32_t kVerticalAccuracyValid = 1U << 6U;
constexpr uint8_t kFrameLocalNed = 3;
constexpr double kMillisecondsToSeconds = 0.001;

uint64_t SteadyNowMs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool IsPositiveFinite(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool IsValidLatitudeLongitude(int32_t latitude_1e7, int32_t longitude_1e7) {
    return latitude_1e7 >= -900000000 && latitude_1e7 <= 900000000 &&
           longitude_1e7 >= -1800000000 && longitude_1e7 <= 1800000000;
}

bool ShouldLogThrottled(uint64_t count) {
    return count == 1 || count % 100 == 0;
}

uint64_t SaturatingAdd(uint64_t left, uint64_t right) {
    if (right > std::numeric_limits<uint64_t>::max() - left) {
        return std::numeric_limits<uint64_t>::max();
    }
    return left + right;
}

}  // namespace

void TargetEstimatorConfig::Validate() const {
    if (ground_target_queue_capacity == 0 || flight_state_queue_capacity == 0) {
        throw std::invalid_argument("目标估计器订阅队列容量必须为正数");
    }
    if (publish_interval.count() <= 0) {
        throw std::invalid_argument("目标估计器发布周期必须为正数");
    }
    if (!IsPositiveFinite(process_acceleration_std_mps2) ||
        !IsPositiveFinite(initial_velocity_std_mps) ||
        !IsPositiveFinite(velocity_measurement_std_mps) ||
        !IsPositiveFinite(default_horizontal_accuracy_m) ||
        !IsPositiveFinite(default_vertical_accuracy_m) ||
        !IsPositiveFinite(minimum_measurement_std_m)) {
        throw std::invalid_argument("目标估计器噪声与精度参数必须为有限正数");
    }
}

struct TargetEstimator::Impl {
    explicit Impl(TargetEstimatorConfig value)
        : config(std::move(value)),
          filter(LinearTargetKalmanConfig{
              config.process_acceleration_std_mps2,
              config.initial_velocity_std_mps,
              config.velocity_measurement_std_mps,
              config.minimum_measurement_std_m}) {}

    TargetEstimatorConfig config;
    LinearTargetKalmanFilter filter;
    common::Topic<common::GroundStationTarget>* ground_topic = nullptr;
    common::Topic<common::FlightStateSnapshot>* flight_topic = nullptr;
    common::Topic<common::GroundStationTarget>::Subscription ground_sub;
    common::Topic<common::FlightStateSnapshot>::Subscription flight_sub;
    common::Topic<common::TargetState> output;
    std::atomic<bool> running{false};
    std::thread thread;
    std::atomic<uint64_t> update_count{0};
    std::atomic<uint64_t> error_count{0};
    uint64_t waiting_home_count = 0;
    uint64_t output_sequence = 0;

    std::optional<GeodeticCoordinate> home;
    uint32_t target_id = 0;
    uint32_t last_ground_boot_id = 0;
    uint32_t last_update_seq = 0;
    uint64_t state_time_ms = 0;
    uint64_t last_observed_ms = 0;
    uint64_t valid_until_ms = 0;
    uint64_t original_valid_for_ms = 0;
    bool expired_state_published = false;
    mutable std::mutex last_state_mutex;
    common::TargetState last_state;

    void ResetFilterState() {
        filter.Reset();
        target_id = 0;
        last_ground_boot_id = 0;
        last_update_seq = 0;
        state_time_ms = 0;
        last_observed_ms = 0;
        valid_until_ms = 0;
        original_valid_for_ms = 0;
        expired_state_published = false;
    }

    void HandleFlightState(const common::FlightStateSnapshot& state) {
        if (!state.home_valid ||
            !IsValidLatitudeLongitude(state.home_lat_1e7, state.home_lon_1e7)) {
            return;
        }

        GeodeticCoordinate candidate;
        candidate.latitude_deg = static_cast<double>(state.home_lat_1e7) * 1e-7;
        candidate.longitude_deg = static_cast<double>(state.home_lon_1e7) * 1e-7;
        candidate.altitude_m = static_cast<double>(state.home_altitude_mm) * 1e-3;
        if (!std::isfinite(candidate.altitude_m)) {
            return;
        }

        const bool changed = home.has_value() &&
            (home->latitude_deg != candidate.latitude_deg ||
             home->longitude_deg != candidate.longitude_deg ||
             home->altitude_m != candidate.altitude_m);
        home = candidate;
        if (changed) {
            ResetFilterState();
            SPDLOG_WARN("PX4 Home变化，目标估计器已重置局部NED滤波状态");
        }
    }

    void DrainFlightStates() {
        while (auto message = flight_sub.TryTake()) {
            HandleFlightState(**message);
        }
    }

    void Reject(const char* reason) {
        const uint64_t count = ++error_count;
        if (ShouldLogThrottled(count)) {
            SPDLOG_WARN("目标估计观测已拒绝: reason={} count={}", reason, count);
        }
    }

    bool BuildMeasurement(const common::GroundStationTarget& target,
                          LinearTargetMeasurement& measurement) {
        if (!home.has_value()) {
            return false;
        }
        if (!IsValidLatitudeLongitude(target.latitude_1e7,
                                      target.longitude_1e7)) {
            Reject("经纬度非法");
            return false;
        }

        const bool altitude_valid =
            (target.validity_flags & kAltitudeValid) != 0U &&
            target.alt_reference == 1;
        const double target_altitude_m = altitude_valid
            ? static_cast<double>(target.altitude_mm) * 1e-3
            : home->altitude_m;
        const auto ned = GeodeticToLocalNed(
            *home,
            GeodeticCoordinate{
                static_cast<double>(target.latitude_1e7) * 1e-7,
                static_cast<double>(target.longitude_1e7) * 1e-7,
                target_altitude_m});
        if (!ned.has_value()) {
            Reject("WGS84到NED转换失败");
            return false;
        }

        measurement.north_m = ned->north_m;
        measurement.east_m = ned->east_m;
        measurement.down_m = ned->down_m;
        measurement.down_valid = altitude_valid;
        measurement.velocity_north_mps = target.velocity_north_mps;
        measurement.velocity_east_mps = target.velocity_east_mps;
        measurement.velocity_down_mps = target.velocity_down_mps;
        measurement.velocity_north_valid =
            (target.validity_flags & kVelocityNorthValid) != 0U;
        measurement.velocity_east_valid =
            (target.validity_flags & kVelocityEastValid) != 0U;
        measurement.velocity_down_valid = altitude_valid &&
            (target.validity_flags & kVelocityDownValid) != 0U;
        measurement.horizontal_position_std_m =
            (target.validity_flags & kHorizontalAccuracyValid) != 0U &&
                    IsPositiveFinite(target.horizontal_accuracy_m)
                ? target.horizontal_accuracy_m
                : config.default_horizontal_accuracy_m;
        measurement.vertical_position_std_m =
            (target.validity_flags & kVerticalAccuracyValid) != 0U &&
                    IsPositiveFinite(target.vertical_accuracy_m)
                ? target.vertical_accuracy_m
                : config.default_vertical_accuracy_m;
        return true;
    }

    void HandleGroundTarget(const common::GroundStationTarget& target) {
        const uint64_t now_ms = SteadyNowMs();
        if (target.target_id == 0 || target.header.receive_time_ms == 0 ||
            target.header.valid_for_ms == 0) {
            Reject("目标标识或时效字段非法");
            return;
        }
        const uint64_t target_valid_until = SaturatingAdd(
            target.header.receive_time_ms, target.header.valid_for_ms);
        if (now_ms >= target_valid_until) {
            Reject("目标已经过期");
            return;
        }
        if (!home.has_value()) {
            const uint64_t count = ++waiting_home_count;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("目标估计等待PX4 Home，暂不处理地面站目标: count={}",
                            count);
            }
            return;
        }
        if (target.ground_station_boot_id == last_ground_boot_id &&
            target.update_seq <= last_update_seq) {
            Reject("目标更新重复或倒序");
            return;
        }

        if (target.transport_age_ms > target.header.receive_time_ms) {
            Reject("传输年龄大于本机接收时刻");
            return;
        }
        const uint64_t observation_time_ms =
            target.header.receive_time_ms - target.transport_age_ms;
        if (filter.IsInitialized() && observation_time_ms < state_time_ms) {
            Reject("观测时间早于滤波状态");
            return;
        }

        LinearTargetMeasurement measurement;
        if (!BuildMeasurement(target, measurement)) {
            return;
        }

        if (filter.IsInitialized() &&
            (target.target_id != target_id || now_ms >= valid_until_ms)) {
            const bool id_changed = target.target_id != target_id;
            ResetFilterState();
            SPDLOG_INFO("目标估计器重新初始化: reason={} target_id={}",
                        id_changed ? "target_id_changed" : "previous_state_expired",
                        target.target_id);
        }

        bool accepted = false;
        if (!filter.IsInitialized()) {
            accepted = filter.Initialize(measurement);
        } else {
            const double dt_seconds =
                static_cast<double>(observation_time_ms - state_time_ms) *
                kMillisecondsToSeconds;
            accepted = filter.Predict(dt_seconds) && filter.Update(measurement);
        }
        if (!accepted) {
            Reject("线性卡尔曼更新失败");
            return;
        }

        target_id = target.target_id;
        last_ground_boot_id = target.ground_station_boot_id;
        last_update_seq = target.update_seq;
        state_time_ms = observation_time_ms;
        last_observed_ms = observation_time_ms;
        valid_until_ms = target_valid_until;
        original_valid_for_ms = target.header.valid_for_ms;
        expired_state_published = false;
        const uint64_t count = ++update_count;
        if (ShouldLogThrottled(count)) {
            const auto state = filter.State();
            const double down_m = state.vertical_valid
                ? state.down_m
                : std::numeric_limits<double>::quiet_NaN();
            SPDLOG_INFO("目标估计已接受地面站观测: target_id={} seq={} N={:.2f} E={:.2f} D={:.2f} VN={:.2f} VE={:.2f} altitude_valid={} count={}",
                        target_id, target.update_seq, state.north_m, state.east_m,
                        down_m, state.velocity_north_mps,
                        state.velocity_east_mps, measurement.down_valid, count);
        }
    }

    common::TargetState BuildOutputState(uint64_t now_ms) {
        common::TargetState state;
        if (!filter.IsInitialized()) {
            return state;
        }

        const double dt_seconds = now_ms >= state_time_ms
            ? static_cast<double>(now_ms - state_time_ms) * kMillisecondsToSeconds
            : 0.0;
        const auto estimate = filter.PredictedState(
            std::min(dt_seconds, 10.0));
        state.header.sequence = ++output_sequence;
        state.header.source_time_ms = now_ms;
        state.header.receive_time_ms = now_ms;
        state.header.valid_for_ms = now_ms < valid_until_ms
            ? valid_until_ms - now_ms
            : 0;
        state.header.source_id = 0;
        state.header.health = now_ms < valid_until_ms ? 1 : 2;
        state.header.frame_id = kFrameLocalNed;
        state.target_id = target_id;
        state.pos_x_m = static_cast<float>(estimate.north_m);
        state.pos_y_m = static_cast<float>(estimate.east_m);
        state.pos_z_m = estimate.vertical_valid
            ? static_cast<float>(estimate.down_m)
            : std::numeric_limits<float>::quiet_NaN();
        state.vel_x_mps = static_cast<float>(estimate.velocity_north_mps);
        state.vel_y_mps = static_cast<float>(estimate.velocity_east_mps);
        state.vel_z_mps = estimate.vertical_valid
            ? static_cast<float>(estimate.velocity_down_mps)
            : std::numeric_limits<float>::quiet_NaN();
        const double remaining_ratio = original_valid_for_ms > 0 &&
                                               now_ms < valid_until_ms
            ? static_cast<double>(valid_until_ms - now_ms) /
                  static_cast<double>(original_valid_for_ms)
            : 0.0;
        state.confidence = static_cast<float>(
            std::clamp(remaining_ratio, 0.0, 1.0));
        state.last_observed_ms = last_observed_ms;
        // 第一阶段只要求水平NED可用；垂直分量无效时用NaN显式表示。
        state.valid = estimate.horizontal_valid && now_ms < valid_until_ms;
        return state;
    }

    void PublishState(uint64_t now_ms) {
        if (!filter.IsInitialized()) {
            return;
        }
        common::TargetState state = BuildOutputState(now_ms);
        if (!state.valid && expired_state_published) {
            return;
        }
        if (!state.valid) {
            SPDLOG_WARN("目标估计状态已过期: target_id={} last_observed_ms={}",
                        state.target_id, state.last_observed_ms);
        }
        expired_state_published = !state.valid;
        {
            std::lock_guard<std::mutex> lock(last_state_mutex);
            last_state = state;
        }
        (void)output.Publish(
            std::make_shared<const common::TargetState>(std::move(state)));
    }

    void Run() {
        uint64_t next_publish_ms = SteadyNowMs();
        while (running.load()) {
            DrainFlightStates();
            const auto target = ground_sub.WaitTakeFor(
                std::min(config.publish_interval, std::chrono::milliseconds(20)));
            DrainFlightStates();
            if (target.has_value()) {
                HandleGroundTarget(**target);
            }

            const uint64_t now_ms = SteadyNowMs();
            if (now_ms >= next_publish_ms) {
                PublishState(now_ms);
                next_publish_ms = SaturatingAdd(
                    now_ms, static_cast<uint64_t>(config.publish_interval.count()));
            }
        }
    }

    void Stop() {
        if (!running.exchange(false)) {
            return;
        }
        ground_sub.Reset();
        flight_sub.Reset();
        if (thread.joinable()) {
            thread.join();
        }
    }
};

TargetEstimator::TargetEstimator(TargetEstimatorConfig config) {
    config.Validate();
    impl_ = std::make_unique<Impl>(std::move(config));
    SPDLOG_INFO("目标估计器创建: model=linear_cv shadow=true control_output=false");
}

TargetEstimator::~TargetEstimator() {
    impl_->Stop();
    SPDLOG_INFO("目标估计器销毁");
}

bool TargetEstimator::Start() {
    if (impl_->running.load()) {
        return true;
    }
    if (impl_->ground_topic == nullptr || impl_->flight_topic == nullptr) {
        ++impl_->error_count;
        SPDLOG_ERROR("目标估计器启动失败: 地面站目标或PX4状态输入未绑定");
        return false;
    }
    if (!impl_->ground_sub.IsOpen()) {
        impl_->ground_sub = impl_->ground_topic->Subscribe(
            impl_->config.ground_target_queue_capacity);
    }
    if (!impl_->flight_sub.IsOpen()) {
        impl_->flight_sub = impl_->flight_topic->Subscribe(
            impl_->config.flight_state_queue_capacity);
    }
    impl_->ResetFilterState();
    impl_->home.reset();
    {
        std::lock_guard<std::mutex> lock(impl_->last_state_mutex);
        impl_->last_state = common::TargetState{};
    }
    impl_->running = true;
    impl_->thread = std::thread(&Impl::Run, impl_.get());
    SPDLOG_INFO("目标估计器启动: shadow=true");
    return true;
}

void TargetEstimator::Stop() {
    const bool was_running = impl_->running.load();
    impl_->Stop();
    if (was_running) {
        SPDLOG_INFO("目标估计器停止");
    }
}

bool TargetEstimator::IsRunning() const {
    return impl_->running.load();
}

void TargetEstimator::SetGroundTargetInput(
    common::Topic<common::GroundStationTarget>& ground) {
    impl_->ground_topic = &ground;
    impl_->ground_sub = ground.Subscribe(
        impl_->config.ground_target_queue_capacity);
}

void TargetEstimator::SetFlightStateInput(
    common::Topic<common::FlightStateSnapshot>& flight) {
    impl_->flight_topic = &flight;
    impl_->flight_sub = flight.Subscribe(
        impl_->config.flight_state_queue_capacity);
}

common::Topic<common::TargetState>& TargetEstimator::EstimatedOutput() {
    return impl_->output;
}

common::TargetState TargetEstimator::LastState() const {
    std::lock_guard<std::mutex> lock(impl_->last_state_mutex);
    return impl_->last_state;
}

uint64_t TargetEstimator::UpdateCount() const {
    return impl_->update_count.load();
}

uint64_t TargetEstimator::ErrorCount() const {
    return impl_->error_count.load();
}

TargetEstimatorStub::TargetEstimatorStub() {
    SPDLOG_INFO("目标估计部件骨架创建");
}

TargetEstimatorStub::~TargetEstimatorStub() {
    SPDLOG_INFO("目标估计部件骨架销毁");
}

bool TargetEstimatorStub::Start() {
    running_ = true;
    SPDLOG_INFO("目标估计部件骨架启动");
    return true;
}

void TargetEstimatorStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("目标估计部件骨架停止");
}

bool TargetEstimatorStub::IsRunning() const {
    return running_;
}

void TargetEstimatorStub::SetGroundTargetInput(
    common::Topic<common::GroundStationTarget>&) {}

void TargetEstimatorStub::SetFlightStateInput(
    common::Topic<common::FlightStateSnapshot>&) {}

common::Topic<common::TargetState>& TargetEstimatorStub::EstimatedOutput() {
    return estimated_output_;
}

common::TargetState TargetEstimatorStub::LastState() const {
    return common::TargetState{};
}

uint64_t TargetEstimatorStub::UpdateCount() const {
    return 0;
}

uint64_t TargetEstimatorStub::ErrorCount() const {
    return 0;
}

}  // namespace drone::perception
