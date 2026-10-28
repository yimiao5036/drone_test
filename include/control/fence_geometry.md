# 多边形围栏几何（fence_geometry）

## 功能职责

提供机载多边形围栏所需的**纯几何函数**：点是否在多边形内、点到边界的距离、
带符号距离、把设定值回拉进多边形（含安全边距）、多边形质心。

- 不做：不读配置、不打日志、不起线程、不做 WGS-84 换算、不做越界判定与状态迁移。
  越界判定与动作在 `MissionStateMachine`（见 `include/state_machine/mission_state_machine.md`），
  经纬度→局部 NED 在 `MissionStateMachine::RebuildFencePolygon()` 里用
  `perception::GeodeticToLocalNed()` 完成。
- 坐标系：全部输入输出都是**局部 NED 平面米**（`common::Vec2Ned{north_m, east_m}`），
  不处理高度（高度带由状态机判定）。

## 接口与数据流

```cpp
namespace drone::control::fence {

/// 射线法；顶点数 < 3 恒 false；恰好落在边上算"内"。
bool PointInPolygon(const std::vector<common::Vec2Ned>& polygon,
                    const common::Vec2Ned& point);

/// 到最近边的绝对距离（米）；顶点数 < 3 返回 0。
double DistanceToBoundary(const std::vector<common::Vec2Ned>& polygon,
                          const common::Vec2Ned& point);

/// 内部为正当、外部为负的距离（米）。
double SignedDistanceToBoundary(const std::vector<common::Vec2Ned>& polygon,
                                const common::Vec2Ned& point);

/// 把点回拉进多边形并保留 margin_m 的安全边距；已在内部且余量足够时原样返回。
common::Vec2Ned ClampIntoPolygon(const std::vector<common::Vec2Ned>& polygon,
                                 const common::Vec2Ned& point,
                                 double margin_m);

/// 顶点质心（退化输入返回 {0,0}）。
common::Vec2Ned Centroid(const std::vector<common::Vec2Ned>& polygon);

}
```

数据流：`MissionStateMachine::RebuildFencePolygon()` 得到顶点 NED →
`FlightController::PublishPosition()` 每拍用 `ClampIntoPolygon()` 钳制位置设定值；
状态机用 `PointInPolygon()` + `SignedDistanceToBoundary()` 判正常/接近/越界。

## 关键实现点

- **射线法**：以被测点向东发射水平射线，统计与各边的交点数；`(y_i > p.y) != (y_j > p.y)`
  才可能相交，交点比较用浮点叉积方向判定，避免除零。点恰在边上（距离 < 1e-9）显式判内。
- **最近点投影**：`ClosestOnSegment()` 用 `Clamp(dot / len2, 0, 1)` 求线段参数，
  并对零长边做保护。`DistanceToBoundary` 取所有边最小距离。
- **内法线**：`InwardNormal()` 用 0.05m 探针分别试探一条边的两条法线方向，
  取投影点落在多边形内部的那条；两条都判外（退化/极窄凹角）时退化为指向质心的方向。
- **回拉策略**：点在外部（或距边界不足 `margin_m`）→ 取最近边界点沿内法线推进
  `margin_m`；若推进后仍在体外（凹角/尖锐角），再向质心方向二分退让，
  **最多 8 次、每次减半**，直到同时满足"在内且距边界 ≥ margin"或退让次数用尽。
  这一步是实机最容易出问题的地方：只沿单边法线内缩，凸角处会贴住另一条边（距离 0）。
- **无全局状态**：全部函数 const 且无分配（除返回 vector 外不建临时容器），
  可在 50ms 控制周期内每拍调用。

## 日志行为

本模块**不打任何日志**（纯函数，便于单测与复用）。钳制是否发生、位移多少，由
`FlightController` 在发生钳制时打 WARN 并在节流消息里带累计次数。

## 测试方式

```bash
cmake --build build -j$(nproc) --target fence_geometry_test
./build/fence_geometry_test            # 12 例
```

覆盖：正方形内外、边上算内、凹 L 形缺口内外、顶点不足退化、距离与带符号距离、
内部点不干预、外侧点按 margin 回拉、**角点越界回拉后 `DistanceToBoundary >= margin`**、
凹多边形回拉、质心。几何部分不依赖硬件，全部在 WSL2 Ubuntu 24.04 用单测验证。

## 排查/修改要点

- 设点"贴边不进去"或越界后抖动：先看 `ClampIntoPolygon` 的退让循环是否被触发
  （可临时在测试里打印回拉前后的 `SignedDistanceToBoundary`）。
- 改边距语义时同步 `config::FenceConfig::boundary_margin_m` 与
  `FlightControllerConfig::fence_margin_m`（后者由配置解析时拷贝，控制器不自读 fence 节）。
- 顶点顺序无关（射线法对顺时针/逆时针都成立），但**不要传重复点或自交多边形**；
  自交时"内/外"在数学上无定义，状态机会把它当成 `kZoneInvalid` 的边界情况。
