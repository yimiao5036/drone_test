/**
 * @file fence_geometry.h
 * @brief 多边形围栏几何（纯函数，无状态、无日志、无线程）
 *
 * 属于 drone/control 模块。职责：局部 NED 平面上的点在多边形内判定、
 * 有符号边界距离、越界点钳制。被 `MissionStateMachine`（越界判定）与
 * `FlightController`（位置意图钳制）共用，故独立成零依赖的头文件 + 实现。
 *
 * 坐标系：局部 NED，单位为米；多边形顶点按任意环绕方向给出（不要求凸、
 * 不要求顺时针，允许凹多边形；不支持自交与多环）。
 *
 * 实现文档见同目录 fence_geometry.md。
 */
#pragma once

#include <vector>

#include "common/types.h"

namespace drone::control::fence {

/// 点是否位于多边形内部（射线法；恰落在边上按"内部"处理）。
/// 顶点少于 3 个时返回 false。
bool PointInPolygon(const std::vector<common::Vec2Ned>& polygon,
                    const common::Vec2Ned& point);

/// 点到多边形边界的最短距离（米，恒非负）；顶点不足 3 个返回 0。
double DistanceToBoundary(const std::vector<common::Vec2Ned>& polygon,
                          const common::Vec2Ned& point);

/// 有符号边界距离（米）：区域内为正、区域外为负、恰在边界为 0。
/// 越界判定的主判据更偏好"约定边界守距"（如越界 5m 才处置），
/// 因此提供符号而不只是距离。
double SignedDistanceToBoundary(const std::vector<common::Vec2Ned>& polygon,
                                const common::Vec2Ned& point);

/// 把点钳制为"至少距边界 margin_m 且位于多边形内"的点：
/// - 已满足（在内且距边界 >= margin）时原样返回；
/// - 否则取最近的边界点，再沿边界指向区域内部的法线内缩 margin。
/// margin_m <= 0 按 0 处理；多边形退化（顶点 < 3）时原样返回 point。
common::Vec2Ned ClampIntoPolygon(const std::vector<common::Vec2Ned>& polygon,
                                 const common::Vec2Ned& point,
                                 double margin_m);

/// 多边形质心（顶点算术平均；仅用于内法线方向的兜底判定）。
common::Vec2Ned Centroid(const std::vector<common::Vec2Ned>& polygon);

}  // namespace drone::control::fence
