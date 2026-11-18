# 到点判定与「到点后视觉未识别」等待计时（handover_wait）

## 功能职责

纯逻辑模块：判定飞机是否**已经到达分配目标点**，并在「已到点但视觉迟迟
没有锁定目标」时累计等待时长，产出告警/中止信号。

- 做：给定单拍到点样本（水平距离、垂直距离、地速、视觉是否锁定）与阈值，
  推进一个状态机（idle → enroute → settling → arrived / no_visual），输出
  `waited_ms`、`warn`、`abort`。
- 不做：不读 Topic、不持有配置对象、不打日志、不生成控制意图、不决定返航。
  复位时机、告警位组装、中止动作全部由 `MissionStateMachine` 负责。

### 为什么需要它

`EvaluateTargetWatchdog()` 只在目标**失效**时计时（`target_wait_timeout_ms` /
`target_lost_timeout_ms` / `target_hold_max_ms`）。现场 0100 记录里 65010 以
5 Hz 持续送达、`valid_for_ms=999`，`HasFreshEstimate()` 每拍为真，三条计时器
都不会启动：飞机悬停在目标点上方、视觉整段 `SEARCHING`，地面站却收不到任何
提示，3.5 分钟没有自动返航。本模块用**独立判据**补上这一段，与看门狗互不影响。

## 接口

```cpp
namespace drone::state_machine {

struct HandoverWaitParams {
    double arrival_radius_m = 3.0;      // 到点水平半径
    double arrival_vertical_m = 2.0;    // 到点垂直半径
    double arrival_speed_mps = 0.5;     // "停稳"地速门限
    std::chrono::milliseconds arrival_hold{2000};
    std::chrono::milliseconds visual_warn{20000};
    std::chrono::milliseconds visual_abort{0};  // 0 = 不自动处置
};

struct HandoverWaitSample {
    double horizontal_m;
    double vertical_m;
    double ground_speed_mps;
    bool visual_locked;
};

enum class HandoverWaitPhase { kIdle, kEnroute, kSettling, kArrived, kNoVisual };

struct HandoverWaitStatus {
    HandoverWaitPhase phase;
    uint64_t waited_ms;
    bool warn;
    bool abort;
};

class HandoverWaitTracker {
    HandoverWaitStatus Update(const HandoverWaitSample&, uint64_t now_ms,
                              const HandoverWaitParams&);
    void Reset();
    bool arrived() const;
    uint64_t arrived_at_ms() const;
};
}
```

## 关键实现点

- **到点 = 三条件同时满足并保持**：水平 ≤ `arrival_radius_m`、垂直 ≤
  `arrival_vertical_m`、地速 ≤ `arrival_speed_mps`，且连续保持
  `arrival_hold_ms` 才进入 `kArrived`/`kNoVisual`；保持期内任一条不满足即退回
  `kEnroute` 并清计时（避免掠过目标点被当成到点）。
- **迟滞**：进入用 `arrival_radius_m` / `arrival_vertical_m`，离开用其 **1.5 倍**。
  半径边缘的抖动不会把"已到点"反复清零，越出迟滞带才真正放弃。
  口径与 `handover.exit_lost_frames` 的"进入/退出门限不同"一致。
- **等待计时**：`waiting_since_ms_` 在判到点的那一拍置位；视觉锁定（`kArrived`）
  时清零，再次丢失从**当前拍**重新起算，不沿用旧起点。
- **告警/中止阈值独立**：`warn = waited_ms >= visual_warn`；
  `abort = visual_abort > 0 && waited_ms >= visual_abort`——现场默认
  `visual_abort=0`，即"只提示、人工决定"，避免地面站误判时代替飞手返航。
- **状态全在成员里**，`Reset()` 与新建等价；无静态变量、无跨实例状态。

## 状态机接线（`src/state_machine/mission_state_machine.cpp`）

- 常量：`kWarningHandoverNoVisual = 1U << 6U`（65013 `active_warning_bits` bit6）、
  `kHandoverArrivalSpeedMps = 0.5`（地速门限固定值，不做配置键）。
- `EvaluateHandoverWait(now_ms)`：仅在「控制任务注入 + 当前 S5 + 分配目标估计
  新鲜 + 估计坐标系为局部 NED + 本机局部位置有效」时推进，否则 `Reset()` 后
  返回空状态。目标点取 `latest_estimated_`（与位置意图同帧），自身位置取
  `latest_flight_->local_*`，地速取 `vx/vy` 合成，视觉锁定取
  `VisualStatusStale(now) == false && state == kLocked`（与 S6 进入判据同口径）。
- 主循环在 `EvaluateTargetWatchdog(now_ms)` 之后调用，`warn` 时置 bit6；
  `abort` 时走 `AbortForTargetWatchdog(kTargetWaitTimeout, ...)` 收口到 S12。
- 状态 `TransitionTo` 进入 S5 时与 `BeginTargetWatchdog()` 同拍 `Reset()`。

## 日志行为

- 只在**首次**越过 `visual_wait_warn_ms` 时打一条 WARN（`到点后视觉未识别:
  waited=…ms 水平=…m 垂直=…m 地速=…m/s`），条件解除后允许再次打印；单位是
  每段到点最多一条，不随决策拍刷屏。
- `abort` 触发时打一条 WARN（含 `visual_wait_abort_ms` 时长），随后由
  `AbortForTargetWatchdog` 打既有的中止日志。

## 消费方（地面站）

机载只把结论摊成 `65013.active_warning_bits` 的 **bit6**，不带毫秒字段，因此地面站
（`tools/web平台/`）的分工是：

- **是否未识别**：以 bit6 为准——置位即判未识别，角标直接红、卡片行标「机载确认」，
  不再用"`state≠5` 且 bit5 清零"的近似推翻它；
- **等了多久**：由地面端自己的到点计时给出（几何阈值与本模块同源），因为 bit6 不含
  时长，按置位时刻起算只能得到"≥ `visual_wait_warn_ms`"的粗粒度；
- **20 s 之前**：bit6 还没亮，地面端用近似判据做早期提示，两边不一致最多差几秒提醒，
  不影响任何动作（自动处置只由 `visual_wait_abort_ms` 决定）。

## 测试方式

- 单元测试：`tests/state_machine/handover_wait_test.cpp`（9 例，无硬件、无时钟），
  覆盖迟滞带、保持门槛、地速门槛、warn/abort 独立、视觉锁定清零、Reset。
  ```bash
  cmake --build build --target handover_wait_test
  ./build/handover_wait_test
  ```
- 集成测试：`tests/state_machine/mission_state_machine_test.cpp` 的
  `S5ArrivedWithoutVisualRaisesHandoverWarning`（bit6 置位、不中止）与
  `S5ArrivedWithoutVisualAbortsWhenConfigured`（abort 非 0 → S12）。

## 排查/修改要点

- **地面站看到 bit6 但飞机没动**：这是设计意图（`visual_wait_abort_ms=0`），
  处置权在地面站；要自动返航必须显式配置该键。
- **bit6 一直不出现**：先确认是否真的"到点"——目标估计与本机位置的水平/垂直
  距离是否都进了半径、地速是否 ≤0.5 m/s、是否已保持 `arrival_hold_ms`；
  再看视觉是否其实已经 LOCKED（`VisualTargetStatus.state=2` 且未超龄）。
- **bit6 误报**：检查 `latest_estimated_` 是否被地面站目标拖着走（目标点在动），
  以及 `arrival_radius_m` 是否配得过大——半径是"到点"的唯一几何口径。
- 改动 `HandoverWaitParams` 语义时必须同步：`include/config/config.h` 的
  `HandoverConfig`、`src/config/config.cpp` 的解析与 `Validate()`、
  `include/config/配置管理.md` 配置表、`docs/数据接口文档.md` 的 bit6 说明。
