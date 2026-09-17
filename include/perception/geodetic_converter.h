#pragma once

#include <optional>

namespace drone::perception {

/// WGS-84大地坐标。高度基准必须由调用方保证一致。
struct GeodeticCoordinate {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
};

/// 以参考点为原点的局部NED坐标，单位米。
struct LocalNedCoordinate {
    double north_m = 0.0;
    double east_m = 0.0;
    double down_m = 0.0;
};

/// 将目标WGS-84坐标转换为以reference为原点的局部NED。
/// 第一阶段采用WGS-84参考椭球曲率半径的局部切平面近似，适用于任务局部区域。
/// reference与target高度基准不一致时不得调用；非法/非有限输入返回nullopt。
std::optional<LocalNedCoordinate> GeodeticToLocalNed(
    const GeodeticCoordinate& reference,
    const GeodeticCoordinate& target);

}  // namespace drone::perception
