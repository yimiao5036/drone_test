#include "perception/target_estimator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <spdlog/fmt/fmt.h>
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
// Home 漂移观测：首次变化立即记录，其后按 30s 窗口汇总（PX4 地面阶段 0.5Hz
// 刷新 Home，逐条打印会以 ≈2s 周期刷屏并淹没真实告警）。
constexpr uint64_t kHomeLogSummaryIntervalMs = 30000;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
// 仅用于日志范围读数（不参与定位）：1° 纬度 ≈ 111.32km。
constexpr double kMetersPerDegreeLatitude = 111320.0;

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

void VisualFusionConfig::Validate() const {
    if (queue_capacity == 0) {
        throw std::invalid_argument("视觉融合订阅队列容量必须为正数");
    }
    if (max_observation_age_ms.count() <= 0 ||
        flight_state_max_age_ms.count() <= 0) {
        throw std::invalid_argument("视觉融合超龄门限必须为正数");
    }
    if (!IsPositiveFinite(disparity_std_px)) {
        throw std::invalid_argument("视觉融合视差误差std必须为有限正数");
    }
    if (!std::isfinite(min_confidence) || min_confidence < 0.0 ||
        min_confidence > 1.0) {
        throw std::invalid_argument("视觉融合置信度下限必须在[0,1]");
    }
    if (!std::isfinite(mount.roll_deg) || !std::isfinite(mount.pitch_deg) ||
        !std::isfinite(mount.yaw_deg) || !std::isfinite(mount.x_m) ||
        !std::isfinite(mount.y_m) || !std::isfinite(mount.z_m)) {
        throw std::invalid_argument("视觉融合安装外参必须为有限数");
    }
    if (enabled && !IsPositiveFinite(fxb_m_px)) {
        throw std::invalid_argument(
            "视觉融合启用时fxb_m_px必须为正（由双目校正矩阵装配填充）");
    }
}

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
    if (min_velocity_position_updates == 0) {
        throw std::invalid_argument(
            "目标估计器 min_velocity_position_updates 必须为正数");
    }
    visual.Validate();
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
    common::Topic<common::StereoTargetDistance>* visual_topic = nullptr;
    common::Topic<common::GroundStationTarget>::Subscription ground_sub;
    common::Topic<common::FlightStateSnapshot>::Subscription flight_sub;
    common::Topic<common::StereoTargetDistance>::Subscription visual_sub;
    common::Topic<common::TargetState> output;
    std::atomic<bool> running{false};
    std::thread thread;
    std::atomic<uint64_t> update_count{0};
    std::atomic<uint64_t> visual_update_count{0};
    std::atomic<uint64_t> error_count{0};
    /// 最近一次接受的视觉观测与预测的水平创新距离（诊断；NaN=无有效值）。
    /// 消费线程写、任意线程读（交接判据），原子量避免额外锁：
    /// 这是一个"最新值"诊断，不要求与滤波状态严格同步。
    std::atomic<double> visual_innovation_m{
        std::numeric_limits<double>::quiet_NaN()};
    uint64_t waiting_anchor_count = 0;
    uint64_t output_sequence = 0;

    /// 转换基准：EKF 投影参考锚点（`GPS_GLOBAL_ORIGIN` #49 的 `ref_*`，即
    /// PX4 LOCAL_NED 原点、设定值执行帧的原点）。方案 C（2026-10-16）：目标
    /// 经纬度一律以锚点为原点换算；仅锚点变化（EKF 原点重设）才重置滤波器。
    std::optional<GeodeticCoordinate> anchor;
    /// Home：仅作漂移观测（PX4 地面跟随/解锁冻结），不再参与换算与重置。
    std::optional<GeodeticCoordinate> home;
    /// Home 漂移观测窗口：首次变化立即记录，其后每 30s 汇总一次窗口统计，
    /// 保留观测能力的同时避免 0.5Hz Home 刷新逐条刷屏。
    struct HomeChangeWindow {
        bool active = false;
        uint64_t count = 0;
        uint64_t start_ms = 0;
        uint64_t last_ms = 0;
        GeodeticCoordinate first{};
        GeodeticCoordinate last{};
        double min_latitude_deg = 0.0;
        double max_latitude_deg = 0.0;
        double min_longitude_deg = 0.0;
        double max_longitude_deg = 0.0;
        double min_altitude_m = 0.0;
        double max_altitude_m = 0.0;
        double max_step_horizontal_m = 0.0;
    };
    HomeChangeWindow home_window;
    bool home_change_logged = false;  ///< 本次启动是否已记录首条 Home 变化明细
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

    // ---- 视觉融合（第二阶段，仅 config.visual.enabled 时生效；消费线程独占）----
    // 最新 PX4 快照（视觉转换的姿态/自身位置来源；Home 逻辑仍用 HandleFlightState）
    common::FlightStateSnapshot latest_flight;
    bool has_flight_snapshot = false;
    uint64_t latest_flight_receive_ms = 0;
    // 多框歧义防线（同一左目帧只接受第一条有效观测）
    uint64_t visual_last_frame_sequence = 0; ///< 最近接受的左目帧序号
    bool visual_frame_seen = false;
    // 细分计数（拒绝计入 error_count 并节流 WARN；跳过类不计错误）
    uint64_t visual_reject_nan = 0;
    uint64_t visual_reject_time_zero = 0;
    uint64_t visual_reject_future = 0;
    uint64_t visual_reject_stale = 0;
    uint64_t visual_reject_out_of_order = 0;
    uint64_t visual_reject_ambiguous_frame = 0;
    uint64_t visual_reject_low_confidence = 0;
    uint64_t visual_reject_attitude = 0;
    uint64_t visual_reject_local_position = 0;
    uint64_t visual_reject_flight_stale = 0;
    uint64_t visual_skipped_uninitialized = 0;
    uint64_t visual_skipped_ground_expired = 0;
    uint64_t ground_degraded_out_of_order = 0;  ///< 视觉推前状态后地面站原地降级更新
    // ---- 速度前馈可用性（P3，2026-10-19）----
    // 地面站显式速度位优先；否则 KF 自算速度需累计足够位置更新才可信。
    // 计数自滤波器重置起算（初始化计为第 1 次）；水平/垂直分别计数。
    uint64_t horizontal_position_update_count = 0;
    uint64_t vertical_position_update_count = 0;
    uint8_t ground_velocity_valid_mask = 0;  ///< kVelocityValid*（地面站最新接受观测）
    double ground_velocity_north_mps = 0.0;
    double ground_velocity_east_mps = 0.0;
    double ground_velocity_down_mps = 0.0;

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
        // 速度前馈可用性随滤波器重置清空：新旧目标/原点的速度不可延续
        horizontal_position_update_count = 0;
        vertical_position_update_count = 0;
        ground_velocity_valid_mask = 0;
        ground_velocity_north_mps = 0.0;
        ground_velocity_east_mps = 0.0;
        ground_velocity_down_mps = 0.0;
        // 视觉记账一并清空（锚点变化/target_id 切换走同一入口）
        visual_last_frame_sequence = 0;
        visual_frame_seen = false;
        // 创新诊断一并清空：跨原点/跨目标的创新值没有意义（防陈旧值通过门限）
        visual_innovation_m.store(std::numeric_limits<double>::quiet_NaN(),
                                  std::memory_order_relaxed);
    }

    void HandleFlightState(const common::FlightStateSnapshot& state) {
        // 留存最新快照供视觉观测转换（姿态 + 自身局部位置）
        latest_flight = state;
        has_flight_snapshot = true;
        latest_flight_receive_ms = state.header.receive_time_ms;

        // ---- 转换基准：EKF 投影参考锚点（方案 C，2026-10-16）----
        // 锚点 = PX4 LOCAL_NED 原点（#49 ref_*），与设定值执行帧同源：目标经纬度
        // 一律以锚点为原点换算，消除与 Home 的基准差 H（详见
        // docs/方案C锚点基准实施清单.md）；仅锚点变化（EKF 原点重设）才重置滤波。
        if (state.ned_ref_valid &&
            IsValidLatitudeLongitude(state.ned_ref_lat_1e7,
                                     state.ned_ref_lon_1e7)) {
            GeodeticCoordinate anchor_candidate;
            anchor_candidate.latitude_deg =
                static_cast<double>(state.ned_ref_lat_1e7) * 1e-7;
            anchor_candidate.longitude_deg =
                static_cast<double>(state.ned_ref_lon_1e7) * 1e-7;
            anchor_candidate.altitude_m =
                static_cast<double>(state.ned_ref_alt_mm) * 1e-3;
            if (std::isfinite(anchor_candidate.altitude_m)) {
                const bool anchor_changed = anchor.has_value() &&
                    (anchor->latitude_deg != anchor_candidate.latitude_deg ||
                     anchor->longitude_deg != anchor_candidate.longitude_deg ||
                     anchor->altitude_m != anchor_candidate.altitude_m);
                const GeodeticCoordinate anchor_previous =
                    anchor.value_or(anchor_candidate);
                anchor = anchor_candidate;
                if (anchor_changed) {
                    ResetFilterState();
                    SPDLOG_WARN(
                        "EKF 锚点变化，目标估计器已重置局部NED滤波状态: "
                        "旧=({:.7f},{:.7f},{:.3f}m) 新=({:.7f},{:.7f},{:.3f}m) {}",
                        anchor_previous.latitude_deg,
                        anchor_previous.longitude_deg,
                        anchor_previous.altitude_m,
                        anchor_candidate.latitude_deg,
                        anchor_candidate.longitude_deg,
                        anchor_candidate.altitude_m,
                        FormatFlightContext(state));
                }
            }
        }

        // ---- Home：仅漂移观测，不参与换算与重置（方案 C）----
        // PX4 地面阶段持续把 Home 贴到当前位置估计（阈值 1~2m 级）、解锁/起飞后
        // 冻结；变化记录保留 GPS 上下文与锚点/局部位置，用于 Home 漂移裁决观测。
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
        const GeodeticCoordinate previous = home.value_or(candidate);
        home = candidate;
        if (changed) {
            const uint64_t now_ms = SteadyNowMs();
            if (!home_change_logged) {
                // 本次启动的第一条变化保留完整明细（旧/新 + 上下文），
                // 后续变化进入窗口汇总，不再逐条打印。
                home_change_logged = true;
                SPDLOG_WARN("PX4 Home变化（诊断记录，估计器不重置）: "
                            "旧=({:.7f},{:.7f},{:.3f}m) 新=({:.7f},{:.7f},{:.3f}m) {}",
                            previous.latitude_deg, previous.longitude_deg,
                            previous.altitude_m, candidate.latitude_deg,
                            candidate.longitude_deg, candidate.altitude_m,
                            FormatFlightContext(state));
            } else {
                AccumulateHomeChange(previous, candidate, now_ms);
                MaybeFlushHomeWindow(now_ms, "周期");
            }
        }
    }

    /// 两点水平距离（等距圆柱近似，仅用于日志范围读数）。
    static double HorizontalDistanceMeters(const GeodeticCoordinate& from,
                                           const GeodeticCoordinate& to) {
        const double mean_latitude_rad =
            (from.latitude_deg + to.latitude_deg) * 0.5 * kDegreesToRadians;
        const double north_m =
            (to.latitude_deg - from.latitude_deg) * kMetersPerDegreeLatitude;
        const double east_m =
            (to.longitude_deg - from.longitude_deg) * kMetersPerDegreeLatitude *
            std::cos(mean_latitude_rad);
        return std::hypot(north_m, east_m);
    }

    /// 把一次 Home 变化并入当前窗口；窗口首条记录时初始化边界。
    void AccumulateHomeChange(const GeodeticCoordinate& previous,
                              const GeodeticCoordinate& current,
                              uint64_t now_ms) {
        if (!home_window.active) {
            home_window = HomeChangeWindow{};
            home_window.active = true;
            home_window.start_ms = now_ms;
            home_window.first = current;
            home_window.min_latitude_deg = home_window.max_latitude_deg =
                current.latitude_deg;
            home_window.min_longitude_deg = home_window.max_longitude_deg =
                current.longitude_deg;
            home_window.min_altitude_m = home_window.max_altitude_m =
                current.altitude_m;
        }
        home_window.count++;
        home_window.last_ms = now_ms;
        home_window.last = current;
        home_window.min_latitude_deg =
            std::min(home_window.min_latitude_deg, current.latitude_deg);
        home_window.max_latitude_deg =
            std::max(home_window.max_latitude_deg, current.latitude_deg);
        home_window.min_longitude_deg =
            std::min(home_window.min_longitude_deg, current.longitude_deg);
        home_window.max_longitude_deg =
            std::max(home_window.max_longitude_deg, current.longitude_deg);
        home_window.min_altitude_m =
            std::min(home_window.min_altitude_m, current.altitude_m);
        home_window.max_altitude_m =
            std::max(home_window.max_altitude_m, current.altitude_m);
        home_window.max_step_horizontal_m = std::max(
            home_window.max_step_horizontal_m,
            HorizontalDistanceMeters(previous, current));
    }

    /// 窗口到期（或停止时强制）输出一条汇总并清空窗口。
    void EmitHomeWindowSummary(const char* reason) {
        if (!home_window.active) {
            return;
        }
        const double duration_s = static_cast<double>(
            home_window.last_ms - home_window.start_ms) * kMillisecondsToSeconds;
        const double mean_latitude_rad =
            (home_window.min_latitude_deg + home_window.max_latitude_deg) *
            0.5 * kDegreesToRadians;
        const double latitude_span_m =
            (home_window.max_latitude_deg - home_window.min_latitude_deg) *
            kMetersPerDegreeLatitude;
        const double longitude_span_m =
            (home_window.max_longitude_deg - home_window.min_longitude_deg) *
            kMetersPerDegreeLatitude * std::cos(mean_latitude_rad);
        const double net_horizontal_m =
            HorizontalDistanceMeters(home_window.first, home_window.last);
        const double net_altitude_m =
            home_window.last.altitude_m - home_window.first.altitude_m;
        const double altitude_span_m =
            home_window.max_altitude_m - home_window.min_altitude_m;
        SPDLOG_WARN(
            "PX4 Home变化窗口汇总（{}）: 窗口={:.1f}s 变化={}次 "
            "首=({:.7f},{:.7f},{:.3f}m) 末=({:.7f},{:.7f},{:.3f}m) "
            "水平范围={:.1f}x{:.1f}m 净漂移={:.1f}m "
            "高度净漂移={:+.2f}m 高度范围={:.2f}m 最大单步={:.2f}m {}",
            reason, duration_s, home_window.count,
            home_window.first.latitude_deg, home_window.first.longitude_deg,
            home_window.first.altitude_m, home_window.last.latitude_deg,
            home_window.last.longitude_deg, home_window.last.altitude_m,
            latitude_span_m, longitude_span_m, net_horizontal_m,
            net_altitude_m, altitude_span_m,
            home_window.max_step_horizontal_m,
            FormatFlightContext(latest_flight));
        home_window = HomeChangeWindow{};
    }

    /// 周期检查：窗口自首条变化起满 30s 即汇总（不依赖后续变化触发）。
    void MaybeFlushHomeWindow(uint64_t now_ms, const char* reason) {
        if (!home_window.active ||
            now_ms < home_window.start_ms + kHomeLogSummaryIntervalMs) {
            return;
        }
        EmitHomeWindowSummary(reason);
    }

    /// 飞行上下文诊断串（GPS 质量 + 锚点 + 局部位置），锚点/Home 变化日志共用。
    static std::string FormatFlightContext(
        const common::FlightStateSnapshot& state) {
        return fmt::format(
            "gps_fix={} 卫星={} eph={} epv={} 锚点ref={} 局部位置={}",
            state.gps_fix_type,
            state.gps_satellites_valid
                ? std::to_string(state.gps_satellites_visible)
                : std::string("--"),
            state.gps_accuracy_valid
                ? fmt::format("{:.2f}m", state.gps_eph_m)
                : std::string("--"),
            state.gps_accuracy_valid
                ? fmt::format("{:.2f}m", state.gps_epv_m)
                : std::string("--"),
            state.ned_ref_valid
                ? fmt::format("({:.7f},{:.7f},{:.2f}m,收{})",
                              static_cast<double>(state.ned_ref_lat_1e7) * 1e-7,
                              static_cast<double>(state.ned_ref_lon_1e7) * 1e-7,
                              static_cast<double>(state.ned_ref_alt_mm) * 1e-3,
                              state.ned_ref_received_count)
                : std::string("--"),
            state.local_position_valid
                ? fmt::format("({:.2f},{:.2f},{:.2f})", state.local_x_m,
                              state.local_y_m, state.local_z_m)
                : std::string("--"));
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

    /// 视觉观测拒绝：细分计数 + 并入 error_count，节流 WARN（带细分次数）。
    void VisualReject(uint64_t& counter, const char* reason) {
        ++error_count;
        const uint64_t count = ++counter;
        if (ShouldLogThrottled(count)) {
            SPDLOG_WARN("目标估计视觉观测已拒绝: reason={} count={}", reason,
                        count);
        }
    }

    // ---- 视觉观测处理（第二阶段）----
    //
    // 语义（docs/物理追踪思路.md §6.3/§6.4）：
    //   - 视觉只细化、不初始化、不延展：滤波器须已被地面站目标初始化且
    //     地面站有效期未过，否则跳过（不计错误，过期政策归状态机）；
    //   - 同一左目帧只接受第一条有效观测（v2 单目标约定，多框歧义防线）；
    //   - 视觉高频，早于滤波状态的观测直接丢弃（不走地面站的降级路径）。
    void HandleVisualDistance(const common::StereoTargetDistance& msg,
                              uint64_t now_ms) {
        if (!msg.valid || !std::isfinite(msg.forward_distance_m) ||
            !std::isfinite(msg.camera_x_m) || !std::isfinite(msg.camera_y_m)) {
            VisualReject(visual_reject_nan, "视觉距离不可用或字段非有限");
            return;
        }
        if (msg.header.source_time_ms == 0) {
            VisualReject(visual_reject_time_zero, "视觉观测时刻为0");
            return;
        }
        const uint64_t obs_time_ms = msg.header.source_time_ms;
        if (obs_time_ms > now_ms) {
            VisualReject(visual_reject_future, "视觉观测时刻在未来（钟域错乱）");
            return;
        }
        if (now_ms - obs_time_ms >
            static_cast<uint64_t>(config.visual.max_observation_age_ms.count())) {
            VisualReject(visual_reject_stale, "视觉观测超龄");
            return;
        }
        if (static_cast<double>(msg.confidence) < config.visual.min_confidence) {
            VisualReject(visual_reject_low_confidence, "视觉观测置信度低于下限");
            return;
        }
        if (visual_frame_seen && msg.frame_sequence == visual_last_frame_sequence) {
            VisualReject(visual_reject_ambiguous_frame,
                         "同一帧多框歧义，仅接受首条");
            return;
        }
        if (!filter.IsInitialized()) {
            ++visual_skipped_uninitialized;  // 地面站未给目标前视觉不造目标
            return;
        }
        if (now_ms >= valid_until_ms) {
            ++visual_skipped_ground_expired;  // 不延展有效期（状态机政策）
            return;
        }
        if (obs_time_ms < state_time_ms) {
            VisualReject(visual_reject_out_of_order, "视觉观测早于滤波状态");
            return;
        }
        if (!has_flight_snapshot ||
            now_ms - latest_flight_receive_ms >
                static_cast<uint64_t>(
                    config.visual.flight_state_max_age_ms.count())) {
            VisualReject(visual_reject_flight_stale, "PX4快照缺失或超龄");
            return;
        }
        if (!latest_flight.attitude_valid) {
            VisualReject(visual_reject_attitude, "PX4姿态无效");
            return;
        }
        if (!latest_flight.local_position_valid) {
            VisualReject(visual_reject_local_position, "PX4局部位置无效");
            return;
        }

        const LocalNedCoordinate self_position{
            static_cast<double>(latest_flight.local_x_m),
            static_cast<double>(latest_flight.local_y_m),
            static_cast<double>(latest_flight.local_z_m)};
        const auto ned = VisualCameraToLocalNed(
            msg.camera_x_m, msg.camera_y_m,
            static_cast<double>(msg.forward_distance_m), config.visual.mount,
            static_cast<double>(latest_flight.roll_rad),
            static_cast<double>(latest_flight.pitch_rad),
            static_cast<double>(latest_flight.yaw_rad), self_position);
        if (!ned.has_value()) {
            VisualReject(visual_reject_nan, "视觉观测NED转换结果非有限");
            return;
        }

        // σ 模型：z = fxb/d ⇒ σ_z = z²·σ_d/fxb；v1 水平/垂直共用 σ_z
        //（大离轴角下不严格保守，见实现文档）。KF 内部另有最小 std 下限。
        const double z_m = static_cast<double>(msg.forward_distance_m);
        const double sigma_z =
            z_m * z_m * config.visual.disparity_std_px / config.visual.fxb_m_px;
        LinearTargetMeasurement measurement;
        measurement.north_m = ned->north_m;
        measurement.east_m = ned->east_m;
        measurement.down_m = ned->down_m;
        measurement.down_valid = true;
        measurement.horizontal_position_std_m = sigma_z;
        measurement.vertical_position_std_m = sigma_z;

        const double dt_seconds =
            static_cast<double>(obs_time_ms - state_time_ms) *
            kMillisecondsToSeconds;
        if (!filter.Predict(dt_seconds)) {
            Reject("线性卡尔曼预测失败");
            return;
        }
        // 创新（更新前）：观测历元处预测位置到观测位置的水平距离。
        // 交接判据的创新门限依赖该值判断"视觉锁定与卡尔曼航迹是否同一目标"，
        // 必须在 Update 之前取预测态（更新后差异会被滤波修正吃掉）。
        const auto predicted = filter.State();
        const double innovation_m =
            std::hypot(ned->north_m - predicted.north_m,
                       ned->east_m - predicted.east_m);
        if (!filter.Update(measurement)) {
            Reject("线性卡尔曼更新失败");
            return;
        }
        visual_innovation_m.store(innovation_m, std::memory_order_relaxed);
        // 位置观测计数：视觉观测带有效垂直分量（down_valid=true），
        // 水平/垂直计数同时推进，供 KF 速度前馈门限使用。
        ++horizontal_position_update_count;
        ++vertical_position_update_count;

        visual_last_frame_sequence = msg.frame_sequence;
        visual_frame_seen = true;
        state_time_ms = obs_time_ms;
        last_observed_ms = obs_time_ms;
        // 注意：不触碰 valid_until_ms / original_valid_for_ms（不延展有效期）
        const uint64_t count = ++visual_update_count;
        if (ShouldLogThrottled(count)) {
            SPDLOG_INFO(
                "目标估计已接受视觉观测: frame={} N={:.2f} E={:.2f} D={:.2f} "
                "sigma={:.3f}m z={:.2f}m 创新={:.2f}m count={}",
                msg.frame_sequence, ned->north_m, ned->east_m, ned->down_m,
                sigma_z, z_m, innovation_m, count);
        }
    }

    void DrainVisualDistances() {
        if (!visual_sub.IsOpen()) {
            return;
        }
        const uint64_t now_ms = SteadyNowMs();
        while (auto message = visual_sub.TryTake()) {
            HandleVisualDistance(**message, now_ms);
        }
    }

    bool BuildMeasurement(const common::GroundStationTarget& target,
                          LinearTargetMeasurement& measurement) {
        if (!anchor.has_value()) {
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
            : anchor->altitude_m;
        const auto ned = GeodeticToLocalNed(
            *anchor,
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
        if (!anchor.has_value()) {
            const uint64_t count = ++waiting_anchor_count;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("目标估计等待EKF锚点，暂不处理地面站目标: count={}",
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
        // 视觉融合开启后，视觉更新会把滤波状态时刻推前；此后到达的地面站观测
        //（obs=receive−transport_age）必然“早于状态”，直接拒绝会让地面站链路
        // 在有效期内饿死。降级：不回溯 Predict，在状态时刻原地 Update
        //（误差上界≈目标速度×时间差）。开关关闭时保持原有拒绝行为。
        bool ground_out_of_order_degraded = false;
        if (filter.IsInitialized() && observation_time_ms < state_time_ms) {
            if (!config.visual.enabled) {
                Reject("观测时间早于滤波状态");
                return;
            }
            ground_out_of_order_degraded = true;
            const uint64_t count = ++ground_degraded_out_of_order;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN(
                    "地面站观测早于滤波状态（视觉已推前），原地降级更新: count={}",
                    count);
            }
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
                ground_out_of_order_degraded
                    ? 0.0  // 降级：原地更新，不回溯 Predict（也避免无符号回绕）
                    : static_cast<double>(observation_time_ms - state_time_ms) *
                          kMillisecondsToSeconds;
            accepted = filter.Predict(dt_seconds) && filter.Update(measurement);
        }
        if (!accepted) {
            Reject("线性卡尔曼更新失败");
            return;
        }

        // 速度前馈可用性：地面站显式速度位优先并立即生效；
        // 未带速度位的轴回退 KF（需累计更新次数达门限）。
        ++horizontal_position_update_count;
        if (measurement.down_valid) {
            ++vertical_position_update_count;
        }
        ground_velocity_valid_mask = 0;
        ground_velocity_north_mps = 0.0;
        ground_velocity_east_mps = 0.0;
        ground_velocity_down_mps = 0.0;
        if (measurement.velocity_north_valid) {
            ground_velocity_valid_mask |= common::kVelocityValidNorth;
            ground_velocity_north_mps = measurement.velocity_north_mps;
        }
        if (measurement.velocity_east_valid) {
            ground_velocity_valid_mask |= common::kVelocityValidEast;
            ground_velocity_east_mps = measurement.velocity_east_mps;
        }
        if (measurement.velocity_down_valid) {
            ground_velocity_valid_mask |= common::kVelocityValidDown;
            ground_velocity_down_mps = measurement.velocity_down_mps;
        }

        target_id = target.target_id;
        last_ground_boot_id = target.ground_station_boot_id;
        last_update_seq = target.update_seq;
        if (!ground_out_of_order_degraded) {
            state_time_ms = observation_time_ms;  // 降级时不回退状态时刻
        }
        last_observed_ms = std::max(last_observed_ms, observation_time_ms);
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
        // ---- 速度前馈可用性（P3，2026-10-19）----
        // 地面站显式速度位优先（不受更新次数门限约束）；缺失轴回退 KF，
        // 需自滤波器重置后累计足够位置更新。水平/垂直逐轴独立；过期清零。
        const bool fresh = now_ms < valid_until_ms;
        const bool kf_horizontal_ready =
            horizontal_position_update_count >= config.min_velocity_position_updates;
        const bool kf_vertical_ready =
            vertical_position_update_count >= config.min_velocity_position_updates;
        uint8_t velocity_valid_mask = 0;
        if (fresh && estimate.horizontal_valid) {
            if ((ground_velocity_valid_mask & common::kVelocityValidNorth) != 0 &&
                std::isfinite(ground_velocity_north_mps)) {
                state.vel_x_mps = static_cast<float>(ground_velocity_north_mps);
                velocity_valid_mask |= common::kVelocityValidNorth;
            } else if (kf_horizontal_ready &&
                       std::isfinite(estimate.velocity_north_mps)) {
                velocity_valid_mask |= common::kVelocityValidNorth;
            }
            if ((ground_velocity_valid_mask & common::kVelocityValidEast) != 0 &&
                std::isfinite(ground_velocity_east_mps)) {
                state.vel_y_mps = static_cast<float>(ground_velocity_east_mps);
                velocity_valid_mask |= common::kVelocityValidEast;
            } else if (kf_horizontal_ready &&
                       std::isfinite(estimate.velocity_east_mps)) {
                velocity_valid_mask |= common::kVelocityValidEast;
            }
        }
        if (fresh && estimate.vertical_valid) {
            if ((ground_velocity_valid_mask & common::kVelocityValidDown) != 0 &&
                std::isfinite(ground_velocity_down_mps)) {
                state.vel_z_mps = static_cast<float>(ground_velocity_down_mps);
                velocity_valid_mask |= common::kVelocityValidDown;
            } else if (kf_vertical_ready &&
                       std::isfinite(estimate.velocity_down_mps)) {
                velocity_valid_mask |= common::kVelocityValidDown;
            }
        }
        state.velocity_valid_mask = velocity_valid_mask;
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
            DrainVisualDistances();
            const auto target = ground_sub.WaitTakeFor(
                std::min(config.publish_interval, std::chrono::milliseconds(20)));
            DrainFlightStates();
            DrainVisualDistances();
            if (target.has_value()) {
                HandleGroundTarget(**target);
            }

            const uint64_t now_ms = SteadyNowMs();
            MaybeFlushHomeWindow(now_ms, "周期");
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
        visual_sub.Reset();
        if (thread.joinable()) {
            thread.join();
        }
        // 停止时强制输出最后一个未满 30s 的窗口，避免尾部观测丢失。
        EmitHomeWindowSummary("停止");
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
    if (impl_->config.visual.enabled && impl_->visual_topic == nullptr) {
        ++impl_->error_count;
        SPDLOG_ERROR("目标估计器启动失败: 视觉融合已启用但视觉距离输入未绑定");
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
    if (impl_->config.visual.enabled && !impl_->visual_sub.IsOpen()) {
        impl_->visual_sub = impl_->visual_topic->Subscribe(
            impl_->config.visual.queue_capacity,
            common::Topic<common::StereoTargetDistance>::OverflowPolicy::
                kDropOldest);
    }
    impl_->ResetFilterState();
    impl_->anchor.reset();
    impl_->home.reset();
    impl_->home_window = decltype(impl_->home_window){};
    impl_->home_change_logged = false;
    impl_->has_flight_snapshot = false;
    impl_->latest_flight_receive_ms = 0;
    {
        std::lock_guard<std::mutex> lock(impl_->last_state_mutex);
        impl_->last_state = common::TargetState{};
    }
    impl_->running = true;
    impl_->thread = std::thread(&Impl::Run, impl_.get());
    SPDLOG_INFO("目标估计器启动: shadow=true visual_fusion={}",
                impl_->config.visual.enabled);
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

void TargetEstimator::SetVisualDistanceInput(
    common::Topic<common::StereoTargetDistance>& visual) {
    impl_->visual_topic = &visual;
    impl_->visual_sub = visual.Subscribe(
        impl_->config.visual.queue_capacity,
        common::Topic<common::StereoTargetDistance>::OverflowPolicy::
            kDropOldest);
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

uint64_t TargetEstimator::VisualUpdateCount() const {
    return impl_->visual_update_count.load();
}

uint64_t TargetEstimator::ErrorCount() const {
    return impl_->error_count.load();
}

double TargetEstimator::LastVisualInnovationM() const {
    return impl_->visual_innovation_m.load(std::memory_order_relaxed);
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

void TargetEstimatorStub::SetVisualDistanceInput(
    common::Topic<common::StereoTargetDistance>&) {}

common::Topic<common::TargetState>& TargetEstimatorStub::EstimatedOutput() {
    return estimated_output_;
}

common::TargetState TargetEstimatorStub::LastState() const {
    return common::TargetState{};
}

uint64_t TargetEstimatorStub::UpdateCount() const {
    return 0;
}

uint64_t TargetEstimatorStub::VisualUpdateCount() const {
    return 0;
}

uint64_t TargetEstimatorStub::ErrorCount() const {
    return 0;
}

double TargetEstimatorStub::LastVisualInnovationM() const {
    return std::numeric_limits<double>::quiet_NaN();  // 骨架实现不产生视觉创新
}

}  // namespace drone::perception
