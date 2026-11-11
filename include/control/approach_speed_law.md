# 接近段速度律（approach_speed_law）

> 对应实现：`include/control/approach_speed_law.h`、`src/control/approach_speed_law.cpp`
> 规格来源：[`../../docs/接近段速度律-数学分析-2026-11-09.md`](../../docs/接近段速度律-数学分析-2026-11-09.md) §6
> 演示与推导：[`../../docs/示意图/追踪梯形曲线-交互演示-2026-11-05.html`](../../docs/示意图/追踪梯形曲线-交互演示-2026-11-05.html)
> 使用方：`MissionStateMachine` 的 S5 位置意图发布（`PublishEstimateGuidedIntent`）

## 1. 功能职责

把 S5 从“目标点 + 固定提前 + 速率上限”升级为**相对系制动包络**：每拍按
目标估计、自机位置/速度与配置，输出 `(p_ref, v_ref)`，由状态机映射为
`kPositionGuide` 位置意图 + 速度前馈。

做：

- 时延补偿 + 距离自适应提前量（`lead = min(k·(d−R), max) + extra`）；
- 制动包络（含一拍反应距离、`brake_safety` 余量）与速度预算（二维
  `√(V²−v_t⊥²)−v_t∥`）；
- 加速度圆侧向预算、转弯约束、分相位加速度调度（8~12 m 过渡）；
- 目标机动需求峰值保持、停滞自动提能力、末端有符号比例收敛；
- 垂直通道（速度匹配 + 制动余量），输出 NED 垂直速度。

不做（边界）：

- 不读目标真值；目标加速度由相邻速度样本差分 + 一阶低通；
- 不含演示页的 UI/评价量（穿透计数、加速度图、`stop` 模式的碰撞三角形）；
- 未移植 jerk 二阶平滑（演示页默认 `J=0`），指令限幅为一阶 slew；
- 不替代 S6/S7 交接判据，也不改变 `FlightController` 的硬限幅与围栏。

## 2. 接口与数据流

```text
TargetState（估计目标：pos/vel/时间戳） ─┐
FlightStateSnapshot（自机 local NED 位置/速度） ─┤
config.approach.speed_law（approach 节） ───────┴─► ApproachSpeedLaw::Update
                                                        │
                                        ApproachOutput { v_n/v_e/v_d, ref_n/e/d, d, closing, phase }
                                                        │
                     MissionStateMachine::PublishEstimateGuidedIntent
                                                        │
                     PositionIntentTarget { x/y/z = ref, feedforward = v, max_speed = max(|v|,0.5) }
                                                        │
                                          ControlIntent(kPositionGuide) → FlightController → PX4
```

- 输入 `ApproachInput` 全部为局部 NED（z 向下为正）；`target_sample_ms` 取
  `TargetState.last_observed_ms`（最近有效观测；为 0 时回退
  `header.receive_time_ms`），用于时延补偿与速度差分。**不能用发布时刻**：
  估计器每拍发布、发布时刻几乎等于 now，会把时延补偿吃掉。
- 输出 `ref_*` 是**自机 + v_ref × ref_lead_s** 的速度锚点（上限 `ref_lead_max_m`）：
  位置环只追这个点，不再把目标点/目标前 standoff 点当成位置目标——否则位置误差
  会在制动段额外加速，2026-11-11 SITL 实测冲进控制间距（0.65m）。
- `max_speed_mps = max(|v_ref|, 0.5)`，只用于控制器“位置引导限速”；控制器
  仍有 `position_guide_speed_mps` 与前馈硬上限兜底。
- S5 只有本律一条路径（2026-11-11 已删除旧“目标点 + 固定提前”回退）；输入无效
  （无局部位置/目标无效）时本拍不发意图并打节流 WARN。

## 3. 关键实现点

| 环节 | 实现 |
|---|---|
| 目标加速度 | 新样本到达才差分，`h>1ms`，幅值限 20 m/s²，一阶低通 `τ=0.12s` |
| 提前量 | 距离自适应 `min(0.06·(d−R), 2m)` + `lead_extra_m`，再除以 `max(0.5,|v_t|)` |
| 制动包络 | `u ≤ √((a·dt)²+2a(d−R)) − a·dt`，`a = min(制动力预算)/brake_safety` |
| 速度预算 | `speed_match=true` 时 `√(V²−v_t⊥²)−v_t∥`；追不上则预算为 0 |
| 侧向预算 | 加速度圆先扣 `target_side + closing·λ̇`，剩余给径向 |
| 分相位 | 12 m 外 `a_plan`、8 m 内 `a_term_eff`，线性过渡，`w` 变化率 ≤0.8/s |
| 近段下限 | `a_term_eff = max(a_term+boost, min(|â_t|峰值+0.5, a_phys))` |
| 提能力 | 停滞/能力受限/穿透累计 4 s 后，`a_plan/a_term` 以 0.5 m/s²/s 抬向 `a_phys` |
| 末端 | `d<R` 时有符号比例回退 `-min(1.5(R−d), 制动曲线)` |
| 指令限幅 | 二维速度变化率 ≤ `slew_eff·dt`；硬速度上限优先 |
| 垂直 | `vz_up = v_tz↑ + sign(dz)·min(VZ, √(2·az/1.3·max(0,|dz|−0.2)))`，再转 NED |

**线程模型**：单线程调用（状态机决策线程），成员无锁、无堆分配、热路径不打日志。

## 4. 配置

`approach.speed_law`（全部可选，缺席用默认值；2026-11-11 起 S5 唯一引导方式）。

| 键 | 默认 | 说明 |
|---|---|---|
| `standoff_m` | 2.9 | 控制间距 R（3.0 到点半径 − 0.1） |
| `accel_plan_mps2` / `decel_plan_mps2` | 3.0 / 3.0 | 远段规划加/减速度 |
| `accel_phys_mps2` | 9.8 | 机体物理加速度上限（只封顶） |
| `accel_term_mps2` | 1.5 | 近段加速度下限 |
| `slew_mps2` | 3.0 | 指令加速度上限；0=关闭一阶限幅 |
| `speed_cap_mps` | 6.0（生产）/10.0（代码默认） | 机体总速度上限；与 `controller.max_feedforward_mps` 对齐 |
| `brake_safety` | 1.3 | 制动力先除该系数 |
| `terminal_gain` | 1.5 | 末端比例收敛增益 |
| `lead_scaled_k` / `lead_scaled_max_m` / `lead_extra_m` | 0.06 / 2.0 / 0.0 | 距离自适应提前量 |
| `t_lead_max_s` | 0.5 | `stop` 模式单次提前量上限（默认 `speed_match=true` 不启用） |
| `target_brake_guard_mps2` | 0.0 | 目标急停预留；>0 时收紧制动预算 |
| `speed_match` | true | 末端相对速度收敛到 0（跟住目标）；false=末端停车 |
| `auto_boost` / `boost_delay_s` / `boost_rate_mps2_per_s` | true / 4.0 / 0.5 | 停滞自动提能力 |
| `target_need_hold` | true | 目标机动需求峰值保持 |
| `ref_lead_s` / `ref_lead_max_m` | 0.3 / 6.0 | 位置锚点前瞻：`p_ref = 自机 + v_ref × 0.3s`（上限 6m） |
| `vertical_speed_cap_mps` / `vertical_accel_cap_mps2` / `vertical_margin_m` | 2.0 / 2.0 / 0.2 | 垂直通道 |

启动期 `ApproachSpeedLawConfig::Validate()` 非法值直接抛异常（配置加载与
状态机构造各兜底一次）。

## 5. 日志行为

- 首次生效打一条 INFO：
  `S5 接近速度律启动: target_id=... d=...m closing=...m/s phase=... ref=(...) v=(...)`
  （每进程一次，避免 10 Hz 刷屏）；
- 关闭开关或输入不足时无额外日志，回退旧逻辑并沿用原 `GPS_APPROACH 位置引导意图启动`；
- 控制器侧若前馈/位置引导被硬上限钳制，沿用既有限幅 WARN。

## 6. 测试方式

```bash
cmake -S . -B build && cmake --build build -j$(nproc)
ctest --test-dir build -R 'ApproachSpeedLaw|SpeedLaw' --output-on-failure
```

- `ApproachSpeedLawTest.*`（`tests/control/approach_speed_law_test.cpp`）：
  静态目标收敛不穿透、运动目标速度匹配、目标快于上限不虚假接近、垂直通道
  方向/限幅、非法配置 fail-fast。
- `MissionStateMachineTest.GpsApproachSpeedLawProducesStandoffReference`：
  S5 位置落点为 `目标 − 视线×2.9 m`、三轴前馈有效、机头指向目标。

## 7. 排查/修改要点

- **S5 没走速度律**：先查 `latest_flight_->local_position_valid`、目标
  `frame_id == kFrameLocalNed` 与 `target.valid`；任一不满足本拍不发意图（节流 WARN）。
- **首拍速度很小**：正常——指令从自机速度按 `slew` 爬坡（10 Hz 下 0.3 m/s/拍）；
  不是“没生效”。
- **到点前冲/回退**：先看 `brake_safety`、`decel_plan_mps2`、`accel_term_mps2`；
  再确认 `speed_cap_mps` 与 `controller.max_feedforward_mps` 是否一致（不一致时
  实际能力比规划低，包络会显得保守/激进）。
- **追运动目标越拉越远**：查 `speed_cap_mps` 是否大于 `|v_t|`，以及
  `v_catch` 是否被侧向预算削到 0（目标横向速度过大）。
- **坡/高度超调**：垂直通道与水平解耦，`vertical_margin_m` 与
  `vertical_accel_cap_mps2` 是主要旋钮；水平律修改不应影响垂直。
- 修改本模块时同步：`docs/接近段速度律-数学分析-2026-11-09.md` §6、
  `docs/追踪算法-接近段速度律实现说明-2026-11-11.md`、`include/config/配置管理.md`。
