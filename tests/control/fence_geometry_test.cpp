// 围栏几何纯函数测试（2026-10-29，多边形围栏 P1）。
//
// 覆盖点：射线法内外判定（含凹多边形）、点到边界距离、带符号距离、越界点
// 按内法线回拉、边角区域回拉（避免从顶点"抄近路"）、非法输入的退化行为。
// 所有用例只依赖纯函数，无 Topic/线程/日志。
#include "control/fence_geometry.h"

#include <gtest/gtest.h>

#include <cmath>

namespace drone::control::fence {
namespace {

// 边长 100m 的正方形（NED，北/东均为中心在原点）。
std::vector<common::Vec2Ned> Square() {
    return {{-50.0, -50.0}, {-50.0, 50.0}, {50.0, 50.0}, {50.0, -50.0}};
}

// 凹多边形（L 形）：缺右下角。用于验证射线法在凹顶点处不误判。
std::vector<common::Vec2Ned> ConcaveL() {
    return {{0.0, 0.0}, {0.0, 60.0}, {30.0, 60.0}, {30.0, 30.0}, {60.0, 30.0},
            {60.0, 0.0}};
}

}  // namespace

TEST(FenceGeometryTest, PointInPolygonReturnsTrueInsideAndFalseOutside) {
    const auto square = Square();
    EXPECT_TRUE(PointInPolygon(square, {0.0, 0.0}));
    EXPECT_TRUE(PointInPolygon(square, {49.0, -49.0}));
    EXPECT_FALSE(PointInPolygon(square, {51.0, 0.0}));
    EXPECT_FALSE(PointInPolygon(square, {0.0, -60.0}));
}

TEST(FenceGeometryTest, PointOnEdgeCountsAsInside) {
    const auto square = Square();
    // 恰好在北边界（north=-50）上：越界判定必须从严不触发，避免边界抖动。
    EXPECT_TRUE(PointInPolygon(square, {-50.0, 10.0}));
    EXPECT_TRUE(PointInPolygon(square, {50.0, 0.0}));
}

TEST(FenceGeometryTest, ConcavePolygonNotchIsOutside) {
    const auto polygon = ConcaveL();
    EXPECT_TRUE(PointInPolygon(polygon, {15.0, 15.0}));
    EXPECT_TRUE(PointInPolygon(polygon, {45.0, 15.0}));
    // L 形缺口（north > 30 且 east > 30）在区域外。
    EXPECT_TRUE(PointInPolygon(polygon, {15.0, 45.0}));
    EXPECT_FALSE(PointInPolygon(polygon, {45.0, 45.0}));
}

TEST(FenceGeometryTest, DegeneratePolygonIsNeverInside) {
    const std::vector<common::Vec2Ned> too_few{{0.0, 0.0}, {10.0, 0.0}};
    EXPECT_FALSE(PointInPolygon(too_few, {1.0, 1.0}));
    EXPECT_FALSE(PointInPolygon({}, {0.0, 0.0}));
}

TEST(FenceGeometryTest, DistanceToBoundaryIsShortestEdgeDistance) {
    const auto square = Square();
    // 中心到四条边均为 50m。
    EXPECT_NEAR(DistanceToBoundary(square, {0.0, 0.0}), 50.0, 1e-9);
    // 靠近北边界 5m（north=-45 → 距北边界 5m）。
    EXPECT_NEAR(DistanceToBoundary(square, {-45.0, 0.0}), 5.0, 1e-9);
    // 在外的点同样返回绝对距离。
    EXPECT_NEAR(DistanceToBoundary(square, {-60.0, 0.0}), 10.0, 1e-9);
}

TEST(FenceGeometryTest, SignedDistanceIsPositiveInsideNegativeOutside) {
    const auto square = Square();
    EXPECT_GT(SignedDistanceToBoundary(square, {0.0, 0.0}), 0.0);
    EXPECT_NEAR(SignedDistanceToBoundary(square, {-40.0, 0.0}), 10.0, 1e-9);
    EXPECT_LT(SignedDistanceToBoundary(square, {0.0, 70.0}), 0.0);
    EXPECT_NEAR(SignedDistanceToBoundary(square, {0.0, 70.0}), -20.0, 1e-9);
}

TEST(FenceGeometryTest, ClampKeepsInsidePointUnchanged) {
    const auto square = Square();
    const auto point = common::Vec2Ned{0.0, 0.0};
    const auto clamped = ClampIntoPolygon(square, point, 1.0);
    EXPECT_NEAR(clamped.north_m, point.north_m, 1e-9);
    EXPECT_NEAR(clamped.east_m, point.east_m, 1e-9);
}

TEST(FenceGeometryTest, ClampPullsOutsidePointInsideByMargin) {
    const auto square = Square();
    // 北侧越界 10m：应回拉到距边界 margin=1m 处（north=-49）。
    const auto clamped = ClampIntoPolygon(square, {-60.0, 0.0}, 1.0);
    EXPECT_TRUE(PointInPolygon(square, clamped));
    EXPECT_NEAR(clamped.north_m, -49.0, 0.5);
    EXPECT_NEAR(clamped.east_m, 0.0, 0.5);
    EXPECT_NEAR(DistanceToBoundary(square, clamped), 1.0, 0.5);
}

TEST(FenceGeometryTest, ClampHandlesCornerExitWithoutCuttingThrough) {
    const auto square = Square();
    // 西北角外 20m：回拉结果必须落在区域内且距边界 >= margin。
    const auto clamped = ClampIntoPolygon(square, {-70.0, -70.0}, 2.0);
    EXPECT_TRUE(PointInPolygon(square, clamped));
    EXPECT_GE(DistanceToBoundary(square, clamped), 1.5);
}

TEST(FenceGeometryTest, ClampToleratesConcavePolygon) {
    const auto polygon = ConcaveL();
    // 缺口内的点（区域外）应被拉回 L 形内部。
    const auto clamped = ClampIntoPolygon(polygon, {50.0, 20.0}, 1.0);
    EXPECT_TRUE(PointInPolygon(polygon, clamped));
    EXPECT_GE(DistanceToBoundary(polygon, clamped), 0.9);
}

TEST(FenceGeometryTest, ClampDegeneratePolygonReturnsInput) {
    const std::vector<common::Vec2Ned> too_few{{0.0, 0.0}, {10.0, 0.0}};
    const auto clamped = ClampIntoPolygon(too_few, {42.0, 42.0}, 1.0);
    EXPECT_NEAR(clamped.north_m, 42.0, 1e-9);
    EXPECT_NEAR(clamped.east_m, 42.0, 1e-9);
}

TEST(FenceGeometryTest, CentroidOfSquareIsOrigin) {
    const auto centroid = Centroid(Square());
    EXPECT_NEAR(centroid.north_m, 0.0, 1e-9);
    EXPECT_NEAR(centroid.east_m, 0.0, 1e-9);
    // 空多边形返回原点（调用方以 valid() 判定，不应依赖该返回值）。
    const auto empty = Centroid({});
    EXPECT_NEAR(empty.north_m, 0.0, 1e-9);
    EXPECT_NEAR(empty.east_m, 0.0, 1e-9);
}

// ---- 区域定义合法性（2026-10-30 审查修复：65018/zone.json 退化几何拒收）----

namespace {

// 以 23.12N/113.12E 为中心、半边长约 100m 的方形区域。
common::FenceZone MakeSquareZone() {
    common::FenceZone zone;
    zone.protocol_version = 1;
    zone.zone_id = 5;
    zone.version = 1;
    zone.type = common::FenceZoneType::kPolygon;
    zone.action = common::FenceAction::kReturn;
    zone.altitude_reference = 1;  // Home 相对
    zone.alt_min_m = 0.0;
    zone.alt_max_m = 30.0;
    zone.vertices = {{23.1191, 113.1190},
                     {23.1209, 113.1190},
                     {23.1209, 113.1210},
                     {23.1191, 113.1210}};
    return zone;
}

}  // namespace

TEST(FenceGeometryTest, ValidateZoneAcceptsWellFormedZone) {
    std::string reason;
    EXPECT_TRUE(ValidateZone(MakeSquareZone(), &reason)) << reason;
}

TEST(FenceGeometryTest, ValidateZoneRejectsMissingIds) {
    auto zone = MakeSquareZone();
    zone.zone_id = 0;
    EXPECT_FALSE(ValidateZone(zone, nullptr));
    zone = MakeSquareZone();
    zone.version = 0;
    EXPECT_FALSE(ValidateZone(zone, nullptr));
}

TEST(FenceGeometryTest, ValidateZoneRejectsTooFewOrTooManyVertices) {
    auto zone = MakeSquareZone();
    zone.vertices = {{23.1191, 113.1190}, {23.1209, 113.1209}};
    EXPECT_FALSE(ValidateZone(zone, nullptr));
    zone = MakeSquareZone();
    zone.vertices.assign(257, common::FenceVertex{23.1200, 113.1200});
    EXPECT_FALSE(ValidateZone(zone, nullptr));
}

TEST(FenceGeometryTest, ValidateZoneRejectsDuplicateVertex) {
    auto zone = MakeSquareZone();
    zone.vertices = {{23.1191, 113.1190},
                     {23.1191, 113.1190},
                     {23.1209, 113.1190},
                     {23.1191, 113.1210}};
    EXPECT_FALSE(ValidateZone(zone, nullptr));
}

TEST(FenceGeometryTest, ValidateZoneRejectsCollinearZeroArea) {
    auto zone = MakeSquareZone();
    zone.vertices = {{23.1200, 113.1190},
                     {23.1200, 113.1195},
                     {23.1200, 113.1200},
                     {23.1200, 113.1205}};
    EXPECT_FALSE(ValidateZone(zone, nullptr));
}

TEST(FenceGeometryTest, ValidateZoneRejectsSelfIntersection) {
    auto zone = MakeSquareZone();
    const common::FenceVertex keep = zone.vertices[1];
    zone.vertices[1] = zone.vertices[2];
    zone.vertices[2] = keep;
    std::string reason;
    EXPECT_FALSE(ValidateZone(zone, &reason));
    EXPECT_FALSE(reason.empty());
}

TEST(FenceGeometryTest, ValidateZoneRejectsInvertedAltitudeBand) {
    auto zone = MakeSquareZone();
    zone.alt_min_m = 20.0;
    zone.alt_max_m = 10.0;
    EXPECT_FALSE(ValidateZone(zone, nullptr));
    // alt_max=0 表示不限高度，min 再大也合法。
    zone.alt_max_m = 0.0;
    EXPECT_TRUE(ValidateZone(zone, nullptr));
}

TEST(FenceGeometryTest, ValidateZoneRejectsUnknownAltitudeReference) {
    auto zone = MakeSquareZone();
    zone.altitude_reference = 2;
    EXPECT_FALSE(ValidateZone(zone, nullptr));
}

// 高度带下限允许为负（相对 Home 低于起飞点）：地面站 UI/校验允许 -1000m，
// 机载不能把这种合法区域判为非法，否则地面站定义后永远无法生效。
TEST(FenceGeometryTest, ValidateZoneAcceptsNegativeAltMin) {
    auto zone = MakeSquareZone();
    zone.alt_min_m = -50.0;
    zone.alt_max_m = 30.0;
    std::string reason;
    EXPECT_TRUE(ValidateZone(zone, &reason)) << reason;
    // 协议为 i16 0.1m，最低可编码 -3276.8m；再低就超出可下发范围。
    zone.alt_min_m = -4000.0;
    EXPECT_FALSE(ValidateZone(zone, nullptr));
}

}  // namespace drone::control::fence
