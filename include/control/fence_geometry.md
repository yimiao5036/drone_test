# 多边形围栏几何（fence_geometry）

## 功能职责

提供机载多边形围栏所需的**纯几何函数**：点是否在多边形内、点到边界的距离、
带符号距离、把设定值回拉进多边形（含安全边距）、多边形质心，以及区域定义
（65018/zone.json）的**几何/拓扑合法性校验** `ValidateZone()`。

- 不做：不读配置、不打日志、不起线程、不做越界判定与状态迁移。
  `ValidateZone()` 内部把顶点按局部平面米近似（纬度每度 111320m），只用于
  尺寸/面积/自交判定，**不产出** NED 顶点；真正的 WGS-84→局部 NED 换算仍在
  `MissionStateMachine::RebuildFencePolygon()` 里用
  `perception::GeodeticToLocalNed()` 完成。越界判定与动作在 `MissionStateMachine`
  （见 `include/state_machine/mission_state_machine.md`）。
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

/// 区域定义合法性校验（65018 / zone.json 共用）：成功返回 true；
/// 失败返回 false 并写入中文原因（可为 nullptr）。检查项：
/// zone_id/version 非 0、顶点 3~256、经纬度数值合法、边长 >= 0.5m、
/// |面积| >= 1m²、多边形不自交、altitude_reference ∈ {0(AMSL),1(Home)}、
/// 高度带数值有限；下限允许为负（相对 Home 低于起飞点，地面站允许 -1000m，
/// 只受协议 i16 0.1m 约束，最低 -3276.8m）；alt_max>0 时 alt_min<=alt_max。
bool ValidateZone(const common::FenceZone& zone, std::string* reason);

}
```

数据流：`MissionStateMachine::ApplyFenceZone()` 在收 65018/装 `zone.json` 前调用
`ValidateZone()` 拒收退化区域；`RebuildFencePolygon()` 得到顶点 NED →
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
- **区域合法性（2026-10-30 审查修复）**：`ValidateZone()` 把顶点按局部平面米近似
  （lat*111320、lon*111320*cos(lat0)），然后：零长边 → 拒收；鞋带公式面积 <1m²
  → 拒收（共线/重复点/极小区域）；逐对非相邻边做**线段相交**判定 → 自交拒收
  （旧实现只查顶点数 ≥3，自交/共线区域能通过，`PointInPolygon` 对自交多边形
  结果无意义，钳制也会把设定值推到区域外）；高度带倒挂与非法
  `altitude_reference` 同层拒收。只做 O(n²) 边相交（n≤256 → 至多 3.2 万次比较，
  仅在收区域/装载时执行一次，不进控制热路径）。

## 日志行为

本模块**不打任何日志**（纯函数，便于单测与复用）。钳制是否发生、位移多少，由
`FlightController` 在发生钳制时打 WARN 并在节流消息里带累计次数。

## 测试方式

```bash
cmake --build build -j$(nproc) --target fence_geometry_test
./build/fence_geometry_test            # 20 例
```

覆盖：正方形内外、边上算内、凹 L 形缺口内外、顶点不足退化、距离与带符号距离、
内部点不干预、外侧点按 margin 回拉、**角点越界回拉后 `DistanceToBoundary >= margin`**、
凹多边形回拉、质心；以及 `ValidateZone()` 的合法区域、id 缺失、顶点过少/过多、
重复顶点、共线零面积、自交（蝴蝶结）、高度带倒挂（含 `alt_max=0` 不限高）、
非法高度基准。`MissionStateMachine` 侧另有 `FenceRejectsDegenerateZoneAndKeepsPrevious`
验证"拒收后保留上一份可用区域"。几何部分不依赖硬件，全部在 WSL2 Ubuntu 24.04 用单测验证。

## 排查/修改要点

- 设点"贴边不进去"或越界后抖动：先看 `ClampIntoPolygon` 的退让循环是否被触发
  （可临时在测试里打印回拉前后的 `SignedDistanceToBoundary`）。
- 改边距语义时同步 `config::FenceConfig::boundary_margin_m` 与
  `FlightControllerConfig::fence_margin_m`（后者由配置解析时拷贝，控制器不自读 fence 节）。
- 顶点顺序无关（射线法对顺时针/逆时针都成立），但**不要传重复点或自交多边形**；
  自交时"内/外"在数学上无定义，状态机会把它当成 `kZoneInvalid` 的边界情况。
