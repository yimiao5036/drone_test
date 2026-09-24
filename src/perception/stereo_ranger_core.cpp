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

    // 滤波状态：按类别+最近邻（左目中心，门限 kAssociationGatePx）关联
    struct FilterState {
        bool active = false;
        std::uint32_t class_id = 0;
        double center_x = 0.0;
        double center_y = 0.0;
        double z = 0.0;
    };

    /// 关联并滤波 Z；返回滤波后 z。
    double FilterZ(std::uint32_t class_id, double center_x, double center_y,
                   double z_raw) {
        FilterState* best = nullptr;
        double best_dist = kAssociationGatePx;
        for (auto& s : filters) {
            if (!s.active || s.class_id != class_id) {
                continue;
            }
            const double dist =
                std::hypot(s.center_x - center_x, s.center_y - center_y);
            if (dist < best_dist) {
                best_dist = dist;
                best = &s;
            }
        }
        double z;
        if (best == nullptr) {
            if (filters.size() < 16) {  // v1.0 少目标上限，防内存漂移
                filters.push_back(FilterState{});
                best = &filters.back();
            } else {
                best = &filters.front();  // 兜底复用最旧
            }
            z = z_raw;  // 初始化直通
        } else if (std::abs(z_raw - best->z) > config.jump_reset_m) {
            z = z_raw;  // 跳变重置（§3.4）
        } else {
            z = config.smooth_alpha * z_raw +
                (1.0 - config.smooth_alpha) * best->z;
        }
        best->active = true;
        best->class_id = class_id;
        best->center_x = center_x;
        best->center_y = center_y;
        best->z = z;
        return z;
    }

    void PushDeltaY(double delta_y) {
        if (delta_y_window.size() >= kDeltaYWindowCapacity) {
            delta_y_window.erase(delta_y_window.begin());
        }
        delta_y_window.push_back(delta_y);
    }

    StereoRangerConfig config;
    std::shared_ptr<const IStereoMatcher> matcher;
    std::uint64_t matched_pair_count = 0;
    std::uint64_t valid_distance_count = 0;
    std::vector<FilterState> filters;
    std::vector<double> delta_y_window;
};

StereoRangerCore::StereoRangerCore(StereoRangerConfig config,
                                   std::shared_ptr<const IStereoMatcher> matcher)
    : impl_(std::make_unique<Impl>(
          config, matcher ? std::move(matcher)
                          : std::shared_ptr<const IStereoMatcher>(
                                std::make_shared<GreedyStereoMatcher>(config)))) {
    impl_->config.Validate();  // 构造期校验（初始化抛异常规范）
}

StereoRangerCore::~StereoRangerCore() = default;

std::vector<common::StereoTargetDistance> StereoRangerCore::Process(
    const std::vector<common::DetectionResult>& left,
    const std::vector<common::DetectionResult>& right,
    std::uint64_t frame_sequence, std::uint64_t source_time_ms) {
    const auto matches = impl_->matcher->Match(left, right);
    std::vector<const StereoMatch*> match_of(left.size(), nullptr);
    for (const auto& m : matches) {
        match_of[m.left_index] = &m;
    }

    std::vector<common::StereoTargetDistance> output;
    output.reserve(left.size());
    for (std::size_t i = 0; i < left.size(); ++i) {
        const auto& dl = left[i];
        common::StereoTargetDistance msg;
        msg.frame_sequence = frame_sequence;
        msg.header.source_time_ms = source_time_ms;
        msg.class_id = dl.class_id;
        msg.track_id = 0;  // v1.0 恒 0
        msg.confidence = dl.confidence;
        msg.center_pixel_x_mid = dl.center_pixel_x;
        msg.bbox_x = dl.bbox_x;
        msg.bbox_y = dl.bbox_y;
        msg.bbox_w = dl.bbox_w;
        msg.bbox_h = dl.bbox_h;

        const StereoMatch* match = match_of[i];
        if (match == nullptr) {
            output.push_back(msg);  // 未匹配：valid=false，方位照常（§3.2）
            continue;
        }
        const auto& dr = right[match->right_index];

        // 去主点视差（§3.1）：两目 cx 差 54px，直接像素差近距会反号
        const auto& c = impl_->config;
        const double u_left = (dl.bbox_x - c.cx_left_px) / c.fx_left_px;
        const double u_right = (dr.bbox_x - c.cx_right_px) / c.fx_right_px;
        const double d_norm = u_left - u_right;
        if (d_norm <= 0.0) {
            output.push_back(msg);  // 防御：匹配器已排除，理论不可达
            continue;
        }
        const double v_left = (dl.center_pixel_y - c.cy_left_px) / c.fy_left_px;
        const double v_right = (dr.center_pixel_y - c.cy_right_px) / c.fy_right_px;
        const double z_raw = c.baseline_m / d_norm;
        const double x_c = 0.5 * c.baseline_m * (u_left + u_right) / d_norm;
        const double y_c = 0.5 * c.baseline_m * (v_left + v_right) / d_norm;

        const double z = impl_->FilterZ(dl.class_id, dl.center_pixel_x,
                                        dl.center_pixel_y, z_raw);
        const double r = std::sqrt(x_c * x_c + y_c * y_c + z * z);

        msg.forward_distance_m = static_cast<float>(z);
        msg.slant_range_m = static_cast<float>(r);
        msg.disparity_px = static_cast<float>(d_norm * c.fx_left_px);
        msg.confidence = std::min(dl.confidence, dr.confidence);
        msg.center_pixel_x_mid =
            static_cast<float>((dl.center_pixel_x + dr.center_pixel_x) * 0.5);
        msg.valid = msg.disparity_px >= c.disparity_min_px &&
                    z >= c.distance_min_m && z <= c.distance_max_m;

        ++impl_->matched_pair_count;
        if (msg.valid) {
            ++impl_->valid_distance_count;
        }
        // 在线校正质量校验统计（§3.5，去 cy 纵向残差）
        impl_->PushDeltaY(std::abs((dl.center_pixel_y - c.cy_left_px) -
                                   (dr.center_pixel_y - c.cy_right_px)));
        output.push_back(msg);
    }
    return output;
}

std::uint64_t StereoRangerCore::MatchedPairCount() const {
    return impl_->matched_pair_count;
}
std::uint64_t StereoRangerCore::ValidDistanceCount() const {
    return impl_->valid_distance_count;
}
double StereoRangerCore::DeltaYP95Px() const {
    if (impl_->delta_y_window.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    auto sorted = impl_->delta_y_window;  // 统计路径才复制排序
    std::sort(sorted.begin(), sorted.end());
    const std::size_t index =
        static_cast<std::size_t>(std::ceil(0.95 * sorted.size())) - 1;
    return sorted[std::min(index, sorted.size() - 1)];
}

void StereoRangerCore::ResetFilter() {
    impl_->filters.clear();
    impl_->delta_y_window.clear();
}

}  // namespace drone::perception
