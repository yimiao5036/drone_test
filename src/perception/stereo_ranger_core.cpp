#include "perception/stereo_ranger_core.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace drone::perception {
namespace {

// 帧间同类别滤波关联门限（像素，按左目中心距离）。v1.0 单/少目标约定，
// 多目标全量跟踪留 v1.1，不进配置（设计 §5 配置集已冻结）。
constexpr double kAssociationGatePx = 150.0;
// |Δy| 在线校验滑动窗口容量（约 10s @30fps 全匹配）。
constexpr std::size_t kDeltaYWindowCapacity = 300;

}  // namespace

void StereoRangerConfig::Validate() const {
    if (baseline_m <= 0.0) {
        throw std::invalid_argument("stereo_ranger.baseline_m必须为正数");
    }
    if (fx_left_px <= 0.0 || fy_left_px <= 0.0 ||
        fx_right_px <= 0.0 || fy_right_px <= 0.0) {
        throw std::invalid_argument("stereo_ranger 焦距必须为正数");
    }
    if (cx_left_px < 0.0 || cy_left_px < 0.0 ||
        cx_right_px < 0.0 || cy_right_px < 0.0) {
        throw std::invalid_argument("stereo_ranger 主点不得为负");
    }
    if (disparity_min_px <= 0.0) {
        throw std::invalid_argument("stereo_ranger.disparity_min_px必须为正数");
    }
    if (distance_min_m <= 0.0 || distance_min_m >= distance_max_m) {
        throw std::invalid_argument(
            "stereo_ranger距离域必须满足 0<distance_min_m<distance_max_m");
    }
    if (tau_y_px <= 0.0) {
        throw std::invalid_argument("stereo_ranger.tau_y_px必须为正数");
    }
    if (tau_h_ratio <= 0.0 || tau_h_ratio > 1.0) {
        throw std::invalid_argument("stereo_ranger.tau_h_ratio必须在(0,1]");
    }
    if (smooth_alpha <= 0.0 || smooth_alpha > 1.0) {
        throw std::invalid_argument("stereo_ranger.smooth_alpha必须在(0,1]");
    }
    if (jump_reset_m <= 0.0) {
        throw std::invalid_argument("stereo_ranger.jump_reset_m必须为正数");
    }
    if (input_queue_capacity == 0) {
        throw std::invalid_argument("stereo_ranger.input_queue_capacity必须为正数");
    }
    if (pair_window.count() <= 0 || summary_log_interval.count() <= 0) {
        throw std::invalid_argument("stereo_ranger时间参数必须为正数");
    }
}

GreedyStereoMatcher::GreedyStereoMatcher(StereoRangerConfig config)
    : config_(config) {}

std::vector<StereoMatch> GreedyStereoMatcher::Match(
    const std::vector<common::DetectionResult>& left,
    const std::vector<common::DetectionResult>& right) const {
    struct Candidate {
        StereoMatch match;
    };
    std::vector<Candidate> candidates;
    for (std::size_t il = 0; il < left.size(); ++il) {
        const auto& dl = left[il];
        const double u_left = (dl.bbox_x - config_.cx_left_px) / config_.fx_left_px;
        const double dy_left = dl.center_pixel_y - config_.cy_left_px;
        for (std::size_t ir = 0; ir < right.size(); ++ir) {
            const auto& dr = right[ir];
            if (dl.class_id != dr.class_id) {
                continue;  // 硬约束：类别一致
            }
            // 纵向残差（去各自主点 cy，设计 §3.1）
            const double delta_y =
                std::abs(dy_left - (dr.center_pixel_y - config_.cy_right_px));
            if (delta_y > config_.tau_y_px) {
                continue;
            }
            // 去主点视差（同侧左边缘）→ 距离域硬约束
            const double u_right =
                (dr.bbox_x - config_.cx_right_px) / config_.fx_right_px;
            const double d_norm = u_left - u_right;
            if (d_norm <= 0.0) {
                continue;
            }
            const double z = config_.baseline_m / d_norm;
            if (z < config_.distance_min_m || z > config_.distance_max_m) {
                continue;
            }
            // 框高比
            const double max_h =
                std::max(static_cast<double>(dl.bbox_h),
                         static_cast<double>(dr.bbox_h));
            const double delta_h =
                std::abs(static_cast<double>(dl.bbox_h) - dr.bbox_h);
            if (max_h <= 0.0 || delta_h / max_h > config_.tau_h_ratio) {
                continue;
            }
            const double cost = delta_y / config_.tau_y_px +
                                delta_h / (config_.tau_h_ratio * max_h);
            if (cost >= 1.0) {
                continue;  // 代价门限
            }
            candidates.push_back({StereoMatch{il, ir, cost}});
        }
    }
    // 代价升序贪心一一配对
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) {
                  return a.match.cost < b.match.cost;
              });
    std::vector<bool> left_used(left.size(), false);
    std::vector<bool> right_used(right.size(), false);
    std::vector<StereoMatch> result;
    for (const auto& c : candidates) {
        if (left_used[c.match.left_index] || right_used[c.match.right_index]) {
            continue;
        }
        left_used[c.match.left_index] = true;
        right_used[c.match.right_index] = true;
        result.push_back(c.match);
    }
    return result;
}

// ---- StereoRangerCore：本任务只建骨架（Process 在 Task 2 实现） ----

struct StereoRangerCore::Impl {
    Impl(StereoRangerConfig cfg, std::shared_ptr<const IStereoMatcher> m)
        : config(std::move(cfg)), matcher(std::move(m)) {}
    StereoRangerConfig config;
    std::shared_ptr<const IStereoMatcher> matcher;
    std::uint64_t matched_pair_count = 0;
    std::uint64_t valid_distance_count = 0;
    std::vector<double> delta_y_window;  // 环形追加，超容量删最旧
};

StereoRangerCore::StereoRangerCore(StereoRangerConfig config,
                                   std::shared_ptr<const IStereoMatcher> matcher)
    : impl_(std::make_unique<Impl>(
          config, matcher ? std::move(matcher)
                          : std::shared_ptr<const IStereoMatcher>(
                                std::make_shared<GreedyStereoMatcher>(config)))) {
    impl_->config.Validate();  // 构造期校验（初始化抛异常规范）
}

std::vector<common::StereoTargetDistance> StereoRangerCore::Process(
    const std::vector<common::DetectionResult>& /*left*/,
    const std::vector<common::DetectionResult>& /*right*/,
    std::uint64_t /*frame_sequence*/, std::uint64_t /*source_time_ms*/) {
    return {};  // Task 2 实现
}

std::uint64_t StereoRangerCore::MatchedPairCount() const {
    return impl_->matched_pair_count;
}
std::uint64_t StereoRangerCore::ValidDistanceCount() const {
    return impl_->valid_distance_count;
}
double StereoRangerCore::DeltaYP95Px() const {
    return std::numeric_limits<double>::quiet_NaN();  // Task 2 实现
}
void StereoRangerCore::ResetFilter() {}  // Task 2 实现

}  // namespace drone::perception
