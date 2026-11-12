# return_speed_law（S12 返航水平速度剖面）

> 代码：`include/control/return_speed_law.h`、`src/control/return_speed_law.cpp`
> 引入：2026-11-12（维护者现场裁决：返航巡航顶速 4 m/s，独立于追踪速度；
> 返航必须自己算加减速；不要变加速度；不要"追不上就一直加速"）

## 1. 功能职责

按**分段恒定加速度**生成 S12 返航的水平速度指令，覆盖三种必须区分的情形：

| 情形 | 现场触发条件 | 剖面行为 |
|---|---|---|
| ① 起步加速 | 目标未运动 / 目标已掉落，飞机被追停到 0 m/s 后才中转返航 | 从实测 0 m/s 按 `accel_plan_mps2` 斜坡升到巡航顶速 |
| ② 超速减速 | 始终追不上目标而中转返航，飞机当前速度可能高于巡航顶速 | 按 `decel_plan_mps2` 斜坡先降回巡航顶速 |
| ③ 末端制动 | 接近返航点 | 制动包络 `v ≤ √(2·a·(d−R))` 收敛；进入 `final_zone_m` 后顶速降为 `final_speed_mps` |

**不做的事（明确边界）**：

- **不做 jerk（二阶）平滑**——只做分段恒定加速度。维护者 2026-11-12：
  "不需要变加速度"。
- **不做"追不上就持续加速"的能力提升**。`ApproachSpeedLaw` 的 `auto_boost`
  （4 s 无进展就按 0.5 m/s³ 抬到物理上限 9.8）是给追**运动目标**设计的；
  返航目标是**固定点**，永久可达，任何加速补偿都是逻辑错误。返航侧的停滞
  只由状态机打 WARN（`返航停滞: ..s 未靠近返航目标`），不改速度剖面。
  单测 `NoBoostWhenStalled` 用"剩余距离恒为 8m 却飞不动"的退化场景锁死
  这条约束：速度只能被压低、绝不上升。
- 不读配置、不读时间、不打日志：纯函数，参数由调用方给出，便于单测。

## 2. 接口与数据流

```text
return_home.{cruise_speed_mps, accel_plan_mps2, decel_plan_mps2, brake_safety,
             final_speed_mps, final_zone_m, arrival_radius_m}
        ↓ ReturnSpeedLawConfig
MissionStateMachine::Impl::UpdateReturnSpeed(remaining_m, ground_speed_mps, now_ms)
        ↓ ReturnSpeedInput + ReturnSpeedState（状态机独占）
control::ComputeReturnSpeed() → ReturnSpeedOutput.speed_mps
        ↓ PositionIntentTarget::max_speed_mps
FlightController::PublishPosition() 位置引导限速
        ↓ Px4Setpoint.x/.y
PX4 位置环
```

### 输入

| 字段 | 来源 | 为什么用它 |
|---|---|---|
| `remaining_m` | 到返航水平目标（起飞原点优先）的水平距离 | 制动包络与末端低速档的唯一依据 |
| `ground_speed_mps` | PX4 快照 `hypot(vx, vy)` **实测地速** | 起步/超速判定的起点。**不能用上一拍指令**：起步段机体会被风阻/前馈丢失吃掉速度，按指令积分会让剖面"以为自己已在巡航"、永远给不出加速段（正是"目标静止时飞机会停住"的成因）；减速段同理必须从实测出发，否则超速进入后指令一直高于实况 |
| `dt_s` | 决策拍实测间隔，钳在 `[0.02, 0.5]` s | 斜坡步长 = a × dt。异常节拍（调度抖动/系统卡顿）下避免一步跨到顶速或斜坡停滞 |

### 输出

`ReturnSpeedOutput.speed_mps` 直接作为位置意图的 `max_speed_mps`，
由控制器位置引导限速执行（见 `include/control/flight_controller.md`
「位置引导限速与相位速度上限」）。

`ReturnSpeedPhase` 只用于日志/诊断，不参与控制分支：
`accel` / `cruise` / `decel` / `brake` / `final` / `idle`。

## 3. 关键实现点

### 3.1 计算顺序

```text
① 顶速 ceiling = in_final_zone ? final_speed_mps : cruise_mps
② 制动包络 envelope = √(2·(decel/brake_safety)·max(0, d − arrival_radius))
   ceiling = min(ceiling, envelope)
③ 斜坡（起点 = 实测地速）
   ceiling > 实测 + 死区 → speed = min(ceiling, 实测 + accel·dt)   [accel]
   ceiling < 实测 − 死区 → speed = max(ceiling, 实测 − decel·dt)   [decel/brake]
   否则                  → speed = ceiling                       [cruise/final]
④ 钳制 speed ∈ [min(0.1, cruise), max(cruise, 实测)]
```

### 3.2 为什么输出上界是 `max(cruise, 实测)` 而不是 `cruise`

超速回落段的指令**必须允许暂时高于 cruise**——那是正在衰减的现有速度。
若把指令一律钳到 `cruise`，8 m/s 进返航会在一拍内跳成 4 m/s，斜坡失效、
飞机被 PX4 限幅硬拽（正是维护者要避免的"突然减速"）。
单测 `OverspeedCommandMayExceedCruiseWhileDecaying` 锁死这一点。

同时 `加速/巡航` 段恒 ≤ `cruise`：返航在原理上**绝不会**跑得比巡航顶速快。

### 3.3 死区与零值语义

- 顶速比较死区 `kSpeedDeadbandMps = 1e-2`：避免在 cruise 附近每拍
  accel/decel 反复切换分级（诊断日志抖动）。
- `accel_mps2 = 0` 或 `decel_mps2 = 0`：该方向不设斜坡（直通）。
  `decel_mps2 = 0` 时制动包络也**不启用**——包络本身就是减速能力，
  减速为 0 时谈包络没有物理意义。
- `dt_s ≤ 0`（时钟回退/首拍）：不推进斜坡，不跳变（保持实测速度；仅
  `accel/decel=0` 这种显式关斜坡的配置才直通 ceiling）。
- **输出恒 ≥ `kReturnSpeedMinCapMps = 0.1` m/s**（2026-11-12 复审修正）：
  位置引导把 `max_speed_mps == 0` 解释成"未填相位上限"并回退控制器硬上限
  （10 m/s）。末端包络在 `d ≤ arrival_radius` 时收敛到 0，若原样下发会把最后
  几米的限速整个解除、被 PX4 位置环重新加速。剖面因此把 0 抬到下限，真正的
  停车交给 S12 到达判据（速度 ≤ `arrival_speed_mps` 且稳定）→ S13。

### 3.4 与"变加速度"的区别（口径澄清）

本模块的"恒定加速度"指**每个分段内加速度恒定**：
起步段恒 `accel_plan_mps2`、超速回落段恒 `decel_plan_mps2`、末端包络段
恒 `decel_plan_mps2 / brake_safety`。分段之间加速度会阶跃（无 jerk 平滑），
这正是维护者要的行为：结构简单、现场可预期，避免二阶平滑带来的超调。

## 4. 参数与默认值

`config/config.json` 的 `return_home` 节：

| 键 | 默认 | 含义 |
|---|---|---|
| `cruise_speed_mps` | **4.0** | 巡航顶速（独立于 `controller.max_speed_mps=10`） |
| `accel_plan_mps2` | 1.5 | 起步加速斜率 |
| `decel_plan_mps2` | 2.0 | 超速回落与末端制动斜率 |
| `brake_safety` | 1.3 | 末端包络安全系数（>1） |
| `final_speed_mps` | 1.0 | 末端低速档顶速 |
| `final_zone_m` | 10.0 | 末端低速档作用距离 |
| `arrival_radius_m` | 2.0 | 到达半径，包络以 `d − R` 为可用距离 |

加载期校验（`config::ReturnHomeConfig::Validate()`）：
`accel/decel ≥ 0`、`brake_safety ≥ 1`、`final_speed_mps > 0 且 ≤ cruise_speed_mps`、
`final_zone_m ≥ 0`。

## 5. 日志行为

本模块**不打日志**（纯函数）。状态机侧 1Hz `S12 返航摘要` 打印剖面分级：

```text
S12 返航摘要: 距目标=493.8m 垂直差=0.00m 地速=5.05m/s 指令上限=4.85m/s 剖面=decel 爬升段=否 相对高度=39.9m
```

现场判读：

| 剖面值 | 含义 |
|---|---|
| `accel` | 正在加速（起步或从低速恢复）。若长时间停在 accel 且地速不涨 → 查逆风/机体限幅 |
| `cruise` | 已达巡航顶速，正常返航 |
| `decel` | 正在从超速回落（追不上转返航的典型状态） |
| `brake` | 末端制动包络在压速度，距目标进入 `final_zone_m` 且剩余距离不足 |
| `final` | 末端低速档（≤ `final_speed_mps`） |

## 6. 测试方式

```bash
cmake -S . -B build && cmake --build build -j$(nproc)
ctest --test-dir build -R return_speed_law --output-on-failure
# 或单跑
./build/return_speed_law_test
```

13 条用例（`tests/control/return_speed_law_test.cpp`）：

| 用例 | 锁定行为 |
|---|---|
| `AcceleratesFromStandstillWhenTargetFarAway` | 静止起步每拍 +accel·dt，约 2.7 s 收敛到 4.0 |
| `NeverJumpsToCruiseInOneStep` | 任何剩余距离下静止起步首拍都只有 0.15（无瞬跳） |
| `DeceleratesFromOverspeedDownToCruise` | 8 m/s 进返航每拍 −decel·dt，20 拍回到 4.0 |
| `OverspeedCommandMayExceedCruiseWhileDecaying` | 减速段指令允许高于 cruise（斜坡不被钳死） |
| `NeverExceedsCruiseWhenAccelerating` | 加速/巡航段恒 ≤ cruise |
| `BrakingEnvelopeLimitsApproachSpeed` | 末端 6 m 顶速降到 `final_speed_mps` 并逐拍收敛 |
| `FinalSpeedNotHigherThanBrakingEnvelopeNearTarget` | 极近目标时速度收敛到包络允许值内 |
| `ArrivalRadiusGivesZeroEnvelope` | 到达半径内包络为 0，输出收敛到下限 0.1（不得输出 0：控制器 0=不限速） |
| `InvalidInputKeepsStateUntouched` | 无有效输入不改变剖面状态 |
| `ZeroDtDoesNotJumpToCeiling` | `dt<=0`（首拍/时钟回退）保持实测速度，不直通 ceiling、不瞬跳 |
| `ResetClearsProfileState` | 进入/重启 S12 时状态清零 |
| `ZeroAccelPassesThroughToCeiling` | `accel=0` 直通（排障配置） |
| `NoBoostWhenStalled` | 飞不动时速度只降不升（无 auto_boost） |

集成层另有 S12 用例覆盖：`InterceptionFiresThenReturnsHomeAndLands`
（静止起步首拍走斜坡、速率不超过 cruise）。

## 7. 排查 / 修改要点

| 现象 | 定位 | 处置 |
|---|---|---|
| 返航一直 4 m/s 不变，感觉慢 | `S12 返航摘要` 剖面恒为 `cruise` | 预期行为。想更快改 `cruise_speed_mps`（无需重编译）；注意它独立于追踪的 `max_speed_mps` |
| 返航起步"顿一下"再动 | 剖面 `accel` 但地速不涨 | 起步斜坡是设计行为（约 2.7 s 到 4 m/s）。若现场嫌慢，抬 `accel_plan_mps2` |
| 进返航瞬间急刹 | 剖面 `decel` 且首拍指令远低于地速 | 检查是否把 `decel_plan_mps2` 配得过大（>3 会让 10 m/s 在 2 s 内刹停）；更可能是 `brake_safety` 配得小 |
| 末端撞进到达半径才停 | 剖面 `brake`/`final` 收敛慢 | 抬 `decel_plan_mps2` 或提前 `final_zone_m`；注意 `arrival_speed_mps=0.5` 判据需要速度真的降下来 |
| 返航飞不动也不加速 | 剖面一直 `cruise` 但地速远低于它 | 这是**设计**（无 auto_boost）。查逆风/机体限幅/围栏钳制；返航停滞 WARN 会提示 |
| 改了 `cruise_speed_mps` 但实际更快 | 控制器 `position_guide_speed_mps` 是硬上限（10） | 本剖面的 `speed_mps` 会作为 `max_speed_mps` 下发，控制器取 min；若实际超了，查是否有其它路径填了更大的值 |
| 末端最后几米突然加速/冲进到达半径 | 旧版本剖面在包络为 0 时输出 0，控制器按"未填上限"回退 10 m/s | 新版输出下限 0.1；若仍出现，查是否还有其它路径给返航意图填 0（`max_speed_mps=0` 一律等于不限速） |

**修改时必须同步的关联点**：

1. `include/config/config.h` 的 `ReturnHomeConfig`（新增字段 + 注释）与
   `src/config/config.cpp` 的两处（`Validate()` + JSON 解析）；
2. `config/config.json` 的 `return_home` 节（含 `_comment` 说明口径）；
3. `src/state_machine/mission_state_machine.cpp` 的 `ReturnSpeedConfig()` 映射
   —— 漏映射会让新参数静默失效；
4. `include/control/flight_controller.md`「位置引导限速」节（执行侧语义）；
5. `docs/故障排查手册.md` §17.10.x 的返航速度条目；
6. `include/config/配置管理.md` 的 `return_home` 键表。

**不要做的事**：不要把 `cruise_speed_mps` 接回 `controller.max_speed_mps`
（那正是 2026-11-12 要解耦的东西）；不要给本模块加 boost 钩子。
