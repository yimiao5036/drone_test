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

namespace {

// 区域校验参数：与越界判定的空间敏感度匹配，小于该尺度视为退化。
constexpr double kMetersPerDegreeLat = 111320.0;
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kMinEdgeLengthM = 0.5;
constexpr double kMinZoneAreaM2 = 1.0;

struct LocalVertex {
    double north_m = 0.0;
    double east_m = 0.0;
};

bool Fail(std::string* reason, const char* text) {
    if (reason != nullptr) {
        *reason = text;
    }
    return false;
}

double Cross(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

/// 点 p 是否落在（含端点的）线段 ab 上（用于共线/端点接触的自交判定）。
bool OnSegment(const LocalVertex& p, const LocalVertex& a,
               const LocalVertex& b) {
    constexpr double kEps = 1e-6;
    if (std::abs(Cross(a.north_m, a.east_m, b.north_m, b.east_m, p.north_m,
                       p.east_m)) > kEps) {
        return false;
    }
    return p.north_m >= std::min(a.north_m, b.north_m) - kEps &&
           p.north_m <= std::max(a.north_m, b.north_m) + kEps &&
           p.east_m >= std::min(a.east_m, b.east_m) - kEps &&
           p.east_m <= std::max(a.east_m, b.east_m) + kEps;
}

/// 线段相交（含共线重叠与端点接触）——用于拒绝自交多边形。
bool SegmentsIntersect(const LocalVertex& p1, const LocalVertex& p2,
                       const LocalVertex& p3, const LocalVertex& p4) {
    const double d1 = Cross(p3.north_m, p3.east_m, p4.north_m, p4.east_m,
                            p1.north_m, p1.east_m);
    const double d2 = Cross(p3.north_m, p3.east_m, p4.north_m, p4.east_m,
                            p2.north_m, p2.east_m);
    const double d3 = Cross(p1.north_m, p1.east_m, p2.north_m, p2.east_m,
                            p3.north_m, p3.east_m);
    const double d4 = Cross(p1.north_m, p1.east_m, p2.north_m, p2.east_m,
                            p4.north_m, p4.east_m);
    constexpr double kEps = 1e-9;
    const bool opposite_12 =
        (d1 > kEps && d2 < -kEps) || (d1 < -kEps && d2 > kEps);
    const bool opposite_34 =
        (d3 > kEps && d4 < -kEps) || (d3 < -kEps && d4 > kEps);
    if (opposite_12 && opposite_34) {
        return true;
    }
    return OnSegment(p1, p3, p4) || OnSegment(p2, p3, p4) ||
           OnSegment(p3, p1, p2) || OnSegment(p4, p1, p2);
}

}  // namespace

bool ValidateZone(const common::FenceZone& zone, std::string* reason) {
    if (zone.zone_id == 0U || zone.version == 0U) {
        return Fail(reason, "zone_id/version 为 0");
    }
    if (zone.altitude_reference > 1U) {
        return Fail(reason, "altitude_reference 只能为 0(AMSL) 或 1(Home)");
    }
    if (!std::isfinite(zone.alt_min_m) || !std::isfinite(zone.alt_max_m)) {
        return Fail(reason, "高度带不是有限数");
    }
    // 高度带下限允许为负：相对 Home 时表示低于起飞点（地面站 UI/校验允许
    // -1000m，协议字段为 i16 0.1m，最小 -3276.8m）。只拒绝超出协议可编码范围。
    if (zone.alt_min_m < -3276.8) {
        return Fail(reason, "alt_min_m 超出协议 i16 0.1m 范围（<-3276.8m）");
    }
    if (zone.alt_max_m > 0.0 && zone.alt_max_m < zone.alt_min_m) {
        return Fail(reason, "alt_max_m 小于 alt_min_m");
    }
    if (zone.vertices.size() < 3U) {
        return Fail(reason, "顶点不足 3 个");
    }
    double lat_sum = 0.0;
    double lon_sum = 0.0;
    for (const auto& vertex : zone.vertices) {
        if (!std::isfinite(vertex.latitude_deg) ||
            !std::isfinite(vertex.longitude_deg)) {
            return Fail(reason, "顶点坐标不是有限数");
        }
        if (std::abs(vertex.latitude_deg) > 90.0 ||
            std::abs(vertex.longitude_deg) > 180.0) {
            return Fail(reason, "顶点经纬度越界");
        }
        lat_sum += vertex.latitude_deg;
        lon_sum += vertex.longitude_deg;
    }
    // 以顶点均值为参考投影到局部等距平面（只用于边/面积/自交判定）。
    const double count = static_cast<double>(zone.vertices.size());
    const double lat_ref = lat_sum / count;
    const double lon_ref = lon_sum / count;
    const double meters_per_deg_lon =
        kMetersPerDegreeLat * std::cos(lat_ref * kDegToRad);
    std::vector<LocalVertex> local;
    local.reserve(zone.vertices.size());
    for (const auto& vertex : zone.vertices) {
        local.push_back(LocalVertex{
            (vertex.latitude_deg - lat_ref) * kMetersPerDegreeLat,
            (vertex.longitude_deg - lon_ref) * meters_per_deg_lon});
    }
    const std::size_t size = local.size();
    double twice_area = 0.0;
    for (std::size_t i = 0; i < size; ++i) {
        const LocalVertex& a = local[i];
        const LocalVertex& b = local[(i + 1) % size];
        if (std::hypot(b.north_m - a.north_m, b.east_m - a.east_m) <
            kMinEdgeLengthM) {
            return Fail(reason, "存在重复顶点或过短边（<0.5m）");
        }
        twice_area += a.north_m * b.east_m - b.north_m * a.east_m;
    }
    if (std::abs(twice_area) * 0.5 < kMinZoneAreaM2) {
        return Fail(reason, "多边形面积过小（退化，<1m²）");
    }
    for (std::size_t i = 0; i < size; ++i) {
        for (std::size_t j = i + 1; j < size; ++j) {
            // 相邻边共享端点，跳过；首尾边亦相邻。
            if (j == i + 1 || (i == 0 && j + 1 == size)) {
                continue;
            }
            if (SegmentsIntersect(local[i], local[(i + 1) % size], local[j],
                                  local[(j + 1) % size])) {
                return Fail(reason, "多边形自交");
            }
        }
    }
    return true;
}

}  // namespace drone::control::fence
