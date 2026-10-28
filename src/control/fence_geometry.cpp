/**
 * @file fence_geometry.cpp
 * @brief 多边形围栏几何实现（局部 NED 平面）
 */
#include "control/fence_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone::control::fence {
namespace {

/// 线段最近点参数 t∈[0,1] 与最近点坐标。
struct ClosestOnSegment {
    common::Vec2Ned point;
    double distance_m = 0.0;
    /// 线段单位方向向量（用于构造法线）；退化线段为零向量。
    common::Vec2Ned direction;
};

ClosestOnSegment ClosestPointOnSegment(const common::Vec2Ned& a,
                                       const common::Vec2Ned& b,
                                       const common::Vec2Ned& p) {
    const double dx = b.north_m - a.north_m;
    const double dy = b.east_m - a.east_m;
    const double length_sq = dx * dx + dy * dy;
    ClosestOnSegment result;
    double t = 0.0;
    if (length_sq > 0.0) {
        t = ((p.north_m - a.north_m) * dx + (p.east_m - a.east_m) * dy) / length_sq;
        t = std::clamp(t, 0.0, 1.0);
    }
    result.point.north_m = a.north_m + dx * t;
    result.point.east_m = a.east_m + dy * t;
    result.distance_m = std::hypot(p.north_m - result.point.north_m,
                                   p.east_m - result.point.east_m);
    if (length_sq > 0.0) {
        const double length = std::sqrt(length_sq);
        result.direction.north_m = dx / length;
        result.direction.east_m = dy / length;
    }
    return result;
}

/// 内法线：两条候选法线里让测试点落在多边形内部的那条；都判不进去时
/// 退化到"指向质心"的方向（凹多边形极端情形兜底）。
common::Vec2Ned InwardNormal(const common::Vec2Ned& boundary_point,
                             const common::Vec2Ned& direction,
                             const std::vector<common::Vec2Ned>& polygon) {
    common::Vec2Ned normal{-direction.east_m, direction.north_m};
    const double probe_m = 0.05;
    const auto test = [&](const common::Vec2Ned& n) {
        common::Vec2Ned probe{boundary_point.north_m + n.north_m * probe_m,
                              boundary_point.east_m + n.east_m * probe_m};
        return PointInPolygon(polygon, probe);
    };
    if (test(normal)) {
        return normal;
    }
    common::Vec2Ned opposite{-normal.north_m, -normal.east_m};
    if (test(opposite)) {
        return opposite;
    }
    const common::Vec2Ned center = Centroid(polygon);
    const double dx = center.north_m - boundary_point.north_m;
    const double dy = center.east_m - boundary_point.east_m;
    const double length = std::hypot(dx, dy);
    if (length > 1e-9) {
        return common::Vec2Ned{dx / length, dy / length};
    }
    return normal;
}

}  // namespace

bool PointInPolygon(const std::vector<common::Vec2Ned>& polygon,
                    const common::Vec2Ned& point) {
    if (polygon.size() < 3) {
        return false;
    }
    bool inside = false;
    const std::size_t count = polygon.size();
    for (std::size_t i = 0, j = count - 1; i < count; j = i++) {
        const auto& pi = polygon[i];
        const auto& pj = polygon[j];
        // 落在边上按内部处理（避免"恰在边界"被判越界而误中止）。
        if (ClosestPointOnSegment(pj, pi, point).distance_m <= 1e-9) {
            return true;
        }
        const bool straddles = ((pi.east_m > point.east_m) != (pj.east_m > point.east_m));
        if (!straddles) {
            continue;
        }
        const double t = (point.east_m - pi.east_m) / (pj.east_m - pi.east_m);
        const double cross_north = pi.north_m + t * (pj.north_m - pi.north_m);
        if (point.north_m < cross_north) {
            inside = !inside;
        }
    }
    return inside;
}

static double DistanceToBoundaryImpl(const std::vector<common::Vec2Ned>& polygon,
                                     const common::Vec2Ned& point) {
    if (polygon.size() < 3) {
        return 0.0;
    }
    double best = std::numeric_limits<double>::infinity();
    const std::size_t count = polygon.size();
    for (std::size_t i = 0, j = count - 1; i < count; j = i++) {
        best = std::min(best, ClosestPointOnSegment(polygon[j], polygon[i], point).distance_m);
    }
    return std::isfinite(best) ? best : 0.0;
}

double DistanceToBoundary(const std::vector<common::Vec2Ned>& polygon,
                          const common::Vec2Ned& point) {
    return DistanceToBoundaryImpl(polygon, point);
}

double SignedDistanceToBoundary(const std::vector<common::Vec2Ned>& polygon,
                                const common::Vec2Ned& point) {
    const double distance = DistanceToBoundary(polygon, point);
    return PointInPolygon(polygon, point) ? distance : -distance;
}

common::Vec2Ned Centroid(const std::vector<common::Vec2Ned>& polygon) {
    common::Vec2Ned center;
    if (polygon.empty()) {
        return center;
    }
    for (const auto& vertex : polygon) {
        center.north_m += vertex.north_m;
        center.east_m += vertex.east_m;
    }
    const double count = static_cast<double>(polygon.size());
    center.north_m /= count;
    center.east_m /= count;
    return center;
}

common::Vec2Ned ClampIntoPolygon(const std::vector<common::Vec2Ned>& polygon,
                                 const common::Vec2Ned& point,
                                 double margin_m) {
    if (polygon.size() < 3) {
        return point;
    }
    const double margin = margin_m > 0.0 ? margin_m : 0.0;
    const double signed_distance = SignedDistanceToBoundary(polygon, point);
    if (signed_distance >= margin) {
        return point;  // 已在内侧且余量足够，不干预
    }
    // 找最近的边界点（用于构造内缩基准）。
    ClosestOnSegment nearest;
    nearest.distance_m = std::numeric_limits<double>::infinity();
    const std::size_t count = polygon.size();
    for (std::size_t i = 0, j = count - 1; i < count; j = i++) {
        const ClosestOnSegment candidate = ClosestPointOnSegment(polygon[j], polygon[i], point);
        if (candidate.distance_m < nearest.distance_m) {
            nearest = candidate;
        }
    }
    if (!std::isfinite(nearest.distance_m)) {
        return point;
    }
    const common::Vec2Ned inward =
        InwardNormal(nearest.point, nearest.direction, polygon);
    common::Vec2Ned clamped{nearest.point.north_m + inward.north_m * margin,
                            nearest.point.east_m + inward.east_m * margin};
    // 凸角情形：最近边界点常是顶点，沿单条边法线内缩会贴着另一条边（余量仍为
    // 0）。这里以质心为目标逐步退让，直到既在体内又满足 margin；多边形本身
    // 小于 margin 时退到质心为止（系统下限，调用方以 valid() 保证 ≥3 顶点）。
    const common::Vec2Ned center = Centroid(polygon);
    for (int step = 0; step < 8; ++step) {
        const bool inside = PointInPolygon(polygon, clamped);
        if (inside && DistanceToBoundaryImpl(polygon, clamped) >= margin) {
            break;
        }
        clamped.north_m += (center.north_m - clamped.north_m) * 0.5;
        clamped.east_m += (center.east_m - clamped.east_m) * 0.5;
    }
    return clamped;
}

}  // namespace drone::control::fence
