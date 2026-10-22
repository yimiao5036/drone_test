# 任务分配助手（mission_assignment）

> 对应实现：`include/state_machine/mission_assignment.h`、`src/state_machine/mission_assignment.cpp`
> 使用方：`MissionStateMachine`（持有实例；接受 65014 时 `Accept`，S3/S4 推进，
> 终态 `Terminal`；重复分配用 `WasHandled` 重发 65015）。
> **接线状态（2026-10-22）**：`AllowsTarget` 已实现并有单测，但状态机 S5 的
> 65010 目标过滤尚未接线，属已知缺口。

## 1. 功能边界

`MissionAssignmentTracker` 只回答“这一次任务分配走到哪一步”，不做任何
硬件、协议或飞行判断：

| 做 | 不做 |
|---|---|
| 记录 assignment_id/target_id/拦截类型/起飞原点 | 校验版本、flags、valid_for_ms（状态机预检） |
| 跟踪分配阶段与一次上电一次空中任务 | 判断 armed/landed/PX4 连接（调用方传入事实） |
| `AllowsTarget` 按分配目标过滤 65010 | 解析 65010/65014、访问 Topic |
| `Availability` 映射 0~5 | 判断电池、健康、控制路径是否可调度 |
| 保存最近 8 条回执供重复消息重发 | 发送 65015、重试、读系统时钟、写日志 |

线程安全：无内部同步，只能在状态机决策线程内调用。所有接口不抛异常；
非法调用返回 `false` 或保持当前状态，不崩溃。

## 2. 阶段状态图

```text
kIdle --Accept(target!=0)--> kAssigned --BeginTakeoff(x,y,z)--> kTakingOff
kTakingOff --TakeoffComplete--> kAirborne --BeginEngage--> kEngaged
kAirborne/kEngaged --BeginReturn--> kReturning --Terminal--> kTerminal

地面中止：kAssigned/kTakingOff --AbortOnGround（未 consumed）--> kIdle
空中中止：consumed 或 kAirborne/kEngaged/kReturning --AbortAirborne--> kReturning
终态：kTerminal --Accept--> 拒绝（进程重启前不再分配）
```

- `MarkAirborne(now_ms)`：调用方在观测到 PX4 `armed && !landed` 时调用，
  只置 `consumed_=true`，**不推进阶段**（时刻参数不影响判定，超时计时由
  状态机自己维护）。是否到达 S4 任务高度由 `TakeoffComplete()` 单独推进，
  二者解耦。
- `TakeoffComplete()`：仅 `kTakingOff → kAirborne`；其他阶段为空操作。
- `BeginEngage()`：`kAirborne`，或已 `consumed` 且仍在 `kTakingOff`
  （调用方漏调可选的 `TakeoffComplete`）时进入 `kEngaged`。
- `BeginReturn()`：`kAirborne/kEngaged → kReturning`。低电、目标丢失等
  空中异常中止走 `AbortAirborne()`，同样进入 `kReturning`。
- `Terminal()`：无条件进入 `kTerminal`，重复调用幂等。保留最后一次
  assignment_id/target_id 供 65013 展示，但 `HasActiveAssignment()` 为 false。
- `AbortOnGround()`：仅未 `consumed` 且处于 `kAssigned/kTakingOff` 时
  清空本次分配并回 `kIdle`；已离地调用为空操作。
- `AbortAirborne()`：`kIdle/kTerminal` 之外，只要 `consumed` 或处于
  `kAirborne/kEngaged/kReturning` 就进入 `kReturning`；空中中止不得回
  `kIdle`。落地后由调用方 `Terminal()`。

## 3. 一次上电语义

- **预检拒绝不消耗任务机会**：`Accept` 失败（非 idle、target=0、已 consumed）
  不改变任何字段。
- **地面失败可重新分配**：已接受但实际未离地时 `AbortOnGround()` 回
  `kIdle`，地面站必须用新的 `assignment_id` 重新下发；助手与状态机都不
  自动重试。
- **实际离地即消耗**：一旦 `MarkAirborne` 被调用（PX4 `armed && !landed`），
  本次上电唯一的空中任务机会用尽；此后即使地面中止逻辑被误调也不会回
  `kIdle`，只能沿返航 → `Terminal` 收口。
- **只进 S4 未离地不算执行空中任务**：`BeginTakeoff/TakeoffComplete` 本身
  不置 `consumed`。
- **终态直到重启**：`kTerminal` 后 `Accept` 永久返回 false。

## 4. 可用状态映射（65013/65015）

| 阶段 | `MissionAvailability` | 可被调度 |
|---|---|---|
| `kIdle` | `kReady` (1) | 是（其余可调度性由状态机预检） |
| `kAssigned` / `kTakingOff` | `kAssigned` (2) | 否 |
| `kAirborne` / `kEngaged` | `kEngaged` (3) | 否 |
| `kReturning` | `kReturning` (4) | 否 |
| `kTerminal` | `kTerminal` (5) | 否 |

`kIdle` 固定返回 `kReady`：控制路径是否注入、PX4 是否连接、电池/健康是否
满足等由状态机 S2 预检决定是否回 `unavailable`，助手不判断，也不返回
`kUnavailable`。

## 5. 重复分配与结果重发

- `RememberResult(assignment_id, result, reason)` 保存回执；同一 id 覆盖为
  最新结果（例如 accepted 后又变成 aborted `battery_low`）。
- 容量固定 8 条，写满后环形覆盖最旧记录，内存有界。
- 状态机收到 65014 时先 `WasHandled(id, &result, &reason)`：
  - 命中：只按保存的 result/reason 重发一次 65015，**不得再次 `Accept`、
    不得重新进入 S3、不得重复起飞**；
  - 未命中：走正常预检路径。
- `WasHandled` 的输出指针可传 `nullptr`；未命中时不修改输出参数。

## 6. 目标过滤

`AllowsTarget(target_id)` 仅在活动分配阶段（`kAssigned`~`kReturning`）且
`target_id != 0` 且等于本次分配目标时返回 true。其他 target_id 的 65010
目标由状态机直接忽略，不影响本机；`kIdle/kTerminal` 一律不匹配。

## 7. 日志与排查要点

助手本身不打日志（纯逻辑、无时钟、无 Topic），排查依赖状态机在阶段变化时
输出的生命周期日志：

- 收到分配后一直 `kIdle`：看 `Accept` 是否失败——target_id 是否为 0、
  是否已 `consumed`、上一次分配是否未收口到 `kTerminal`。
- 地面中止后无法重新分配：确认中止前是否已 `MarkAirborne`；一旦实际离地，
  按一次上电语义不得再接受新分配。
- 起飞后卡在 `kTakingOff`：调用方可能未调用 `TakeoffComplete()`；也可直接
  `BeginEngage()`（已 `consumed` 时允许）。
- 同一 65014 重复下发导致重复动作：检查状态机是否先查 `WasHandled`；重复
  id 必须只重发 65015。
- 65013 显示 `ready` 但不能调度：这是预期分工——助手只给阶段映射，真实
  可调度性看状态机预检（控制路径/电池/PX4/健康）。

## 8. 测试

`MissionAssignmentTest.*`（`tests/state_machine/mission_assignment_test.cpp`）：

- 初始 `kIdle`/`kReady`、字段与原点清零；
- `Accept` 记录 id/target/type；非 idle 拒绝；target=0 拒绝；
- `BeginTakeoff` 原点记录、非 `kAssigned` 空操作；
- `MarkAirborne` 消耗机会但不推进阶段、幂等、无分配时忽略；
- `TakeoffComplete` → `kAirborne`；`BeginEngage` → `kEngaged`；
- `BeginReturn` → `kReturning`；`Terminal` → `kTerminal`；
- `AbortOnGround` 回 idle 且可用新 id 重新接受；已离地时忽略；
- `AbortAirborne` 仅在实际离地后生效，进入 `kReturning`，终态后拒绝分配；
- `AllowsTarget` 活动期目标过滤、idle/terminal 不匹配；
- availability 全阶段映射 0~5；
- 重复 id 重发（最新结果覆盖、未命中不改输出、空指针查询）；
- 回执容量 8 条、环形覆盖不无界增长；
- 非法顺序调用不改变状态、不崩溃。

运行方式：

```bash
cmake -S . -B build
cmake --build build -j$(nproc) --target mission_assignment_test
./build/mission_assignment_test
```
