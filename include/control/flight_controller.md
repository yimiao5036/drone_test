# flight_controller —— 飞行控制器（IFlightController / FlightController / FlightControllerStub）

## 功能边界

消费状态机发布的 `ControlIntent`，经门禁链（有限性/超龄/控制权）后生成
`Px4Setpoint` 发布，供 PX4 通信线程发送。属于执行层入口，是正式控制输出
的唯一来源。

当前阶段（阶段四控制接入 Task 3~5）边界：

- `FlightController` 正式实现骨架已落地：固定周期线程、意图队列排干
  （只留最新）、入口门禁（有限性/超龄拒绝计数）。
- **限幅流水线与无扰衔接已接入（Task 4）**：速度幅值等比钳制 →
  `SlewRateLimiter` 加速度限幅；来源/类型变化时斜坡起点=上一周期实际
  输出；时钟回退冻结输出；未定义 `control_source`（>4）按安全语义降级
  （仅刹停类意图生效）。
- **控制权三态与 PX4 门禁已接入（Task 5）**：每拍经 `SetPx4LinkSource()`
  注入的 `IPx4Link::LatestState()` 快照核验 `connected + armed +
  Offboard(flight_mode=6/sub=0)`，并要求最新意图携带状态机显式请求位
  （`reason_code` bit31）；**未注入快照源时授权链恒不通过，控制器保持
  IDLE 影子运行（不发布任何设定值）**。ACTIVE/COAST 中门禁失效立即发布
  `valid=false` 停发并转 IDLE；意图超龄转 COAST 零速刹车保持，
  `coast_timeout` 超时转 IDLE 停发。
- **Home 相对围栏已接入（Task 5）**：位置式意图水平模长超 `max_radius_m`
  等比钳制、相对高度超 `max_altitude_m` 钳制；`home_valid=false` 时
  位置式意图拒绝（无基准不可围栏）。
- **自动起飞预授权已接入（Task 6，spec §8）**：控制权增加 `kBootstrap`；
  地面/未解锁且收到未超龄的 `kTakeoff` 意图（bit31）时发布起飞原点
  位置保持流，为 PX4 进入 Offboard 建立合法 setpoint 流；`armed+Offboard`
  后转 ACTIVE；条件失效/意图撤销/超过 `bootstrap_timeout` 只回 IDLE 停流，
  **不进入 COAST**。门禁**不读取任何 RC 字段**。
- **坐标边界（Task 9 明确）**：`kVelocityHeading` 意图的速度三分量约定为
  **局部 NED（北/东/地）**——机体系→NED 旋转由意图生产方在输出端完成
  （S7 视觉伺服在 `TrackingControlLaw` 内部按快照航向角旋转，状态机意图以
  `frame_id=3` 标注），`FlightController` 只做限幅/无扰衔接/门禁，
  **不做任何二次旋转**；位置式意图同理为局部 NED（原点=PX4 Home）。
- `FlightControllerStub` 骨架占位保留不动，供骨架冒烟测试使用。

## 接口与数据流

```text
common::Topic<ControlIntent>（状态机，kControlIntent）
    ──► FlightController（固定周期线程 Tick）
        │ 门禁链：排干队列只留最新 → 有限性 → 超龄（valid_until + intent_timeout）
        │        → bootstrap 分流（kTakeoff+bit31 → kBootstrap 原点保持）
        │        → 控制权门禁（快照 connected+armed+Offboard 6/0 + 请求位，
        │          每拍核验）→ 持有期再超龄 → COAST 刹车
        │ 限幅流水线：未知来源安全降级 → 速度幅值等比钳制
        │        → 无扰衔接（斜坡起点=上一周期实际输出）→ slew 加速度限幅
        │ 围栏（位置式）：Home 相对水平半径/高度钳制（home_valid=false 拒绝）
        ▼
common::Topic<Px4Setpoint> ──► IPx4Link（PX4 通信线程）
```

- `SetInput(Topic<ControlIntent>&)`：绑定意图输入，订阅容量 4、丢旧保新。
  **须在 `Start()` 前调用**（Start 后周期线程即开始消费订阅）。
- `SetpointOutput()`：设定值输出 Topic。
- `Authority()` / `RejectedIntentCount()` / `ClampedCount()` /
  `SetpointCount()` / `ErrorCount()`：诊断计数，原子量，供测试与门禁使用。
  - `RejectedIntentCount()` 语义：入口门禁拒绝（非有限/到达即超龄）按到达
    计——上游每拍重发同一条违例意图时表现为按拍增长；**授权后持有期
    超龄在转入 COAST 那一拍 +1**（Task 5 起超龄即转 COAST，不再逐拍
    增长）；位置式意图无 Home 基准拒绝按拍计，直至意图超龄转 COAST。
  - `ClampedCount()` 为限幅触发累计计数（速度幅值等比钳制 + 位置式围栏
    钳制，每次钳制生效 +1，供 G2 验收核对）。
- `SetPx4LinkSource(const communication::IPx4Link*)`：注入 PX4 只读快照源
  （控制权门禁与 Home 围栏数据源；装配期 Start 前调用；**未注入时授权链
  恒不通过**，控制器保持 IDLE 影子运行）。门禁判据只取快照的 `connected`
  字段——链路断开方向有滞后窗口（先清原子标志后发布快照），不得与
  `IsConnected()` 交叉推理。
- 测试入口（生产装配不得调用）：`TestForceAuthority()` 强制控制权状态
  （授权链每拍重估，门禁不通时下一拍即撤到 IDLE——维持 ACTIVE 须配合
  MockPx4Link 快照与意图请求位）；`TestTick(now_ms)` 以指定时间戳同步执行
  一拍（须在未 Start 状态调用；**运行时守卫**：Start 后调用直接忽略并
  节流 WARN，防止与周期线程并发访问 Tick 内部无锁状态）。

## 控制权状态机（spec §4，Task 5/6 接入）

四态：`IDLE` / `BOOTSTRAP` / `ACTIVE` / `COAST`。`BOOTSTRAP` 的进入/退出
门禁见下节「自动起飞预授权 kBootstrap」。

授权链（本控制器侧部分，部署/健康门禁归装配层与健康管理）：

```text
SetPx4LinkSource 已注入（未注入=影子模式，恒不授权）
  → 快照 connected && armed
  → flight_mode == 6 && flight_sub_mode == 0（PX4 1.17.0 Offboard 编码）
  → 状态机显式请求锁存（最新通过入口门禁的意图 reason_code bit31 置位）
```

**显式请求位约定（本任务冻结）**：`ControlIntent.reason_code` bit31 =
`common::kControlIntentReasonRequestControl`（`include/common/types.h`），
表示状态机显式请求控制权；低位段沿用 `docs/数据接口文档.md` §3.5 枚举
（0~7 规划语义、100+ 视觉跟踪影子段）。锁存规则：跟随**最新通过入口门禁**
的意图刷新（被拒绝的意图不刷新）；进入 IDLE 时清零——回 ACTIVE 必须重新
满足授权链（含显式请求）。锁存本身**不随时间过期**：IDLE→ACTIVE 另要求
持有新鲜（未超龄）请求位意图，陈旧锁存不得事后激活授权（评审 fix round 2）。
状态机侧置位在 Task 7 落地。

三态迁移表（每次迁移打 WARN 级状态变化日志，含原因串）：

| 当前态 | 条件 | 下一态 | 动作 |
|---|---|---|---|
| IDLE | bootstrap 门禁全通（kTakeoff + bit31 + 地面/未解锁等） | BOOTSTRAP | 记录起飞原点，发布原点保持位置流 |
| IDLE | 授权链全通且持有新鲜请求位意图 | ACTIVE | 无（随后走正常发布路径） |
| IDLE | 授权链不通或请求意图已超龄 | IDLE | 停发（不发布任何 setpoint） |
| BOOTSTRAP | `armed && Offboard(6/0)` | ACTIVE | 当拍仍发布原点保持，下一拍起走正常路径 |
| BOOTSTRAP | 条件失效/意图撤销或超龄/超过 `bootstrap_timeout` | IDLE | 发布 `valid=false` 停流（**不进入 COAST**） |
| ACTIVE | 授权链断开（心跳断/非 Offboard/未 armed/请求撤销） | IDLE | 发布 `valid=false` 停发 |
| ACTIVE | 无意图或持有期超龄 | COAST | 记录 `coast_since`，发布零速刹车 |
| COAST | 授权链断开 | IDLE | 发布 `valid=false` 停发 |
| COAST | `now - coast_since > coast_timeout` | IDLE | 发布 `valid=false` 停发 |
| COAST | 意图恢复新鲜 | ACTIVE | 随后走正常发布路径 |
| COAST | 意图仍超龄且未超时 | COAST | 每拍发布零速刹车（`valid=true`） |

设计要点：

- **离开即撤，无迟滞**：ACTIVE/COAST 中门禁失效当拍停发转 IDLE——模式
  抖动不会导致授权抖动（回 ACTIVE 需授权链重新全通，含显式请求位）。
- COAST 计时以**首次进入**为准，重复进入不刷新（防止刹车保持被续期）。
- 时钟回退安全：COAST 超时判定用有符号差值（回退时差值为负，不误判超时）；
  超龄判定沿用 `StalenessChecker`（`now < sample` 视为不可信直接判超龄）。
- 停发/刹车发布时 slew 状态归零、`last_output_` 重置：恢复授权后速度式
  输出从 0 斜坡起步（延续 S6 首条指令突变抑制语义）。

## 自动起飞预授权 kBootstrap（Task 6，spec §8）

PX4 进入 Offboard 前必须已有 setpoint 流。状态机 S3 每决策拍发布
`kTakeoff` 意图（目标 = 起飞原点 `x0/y0/z0`，`reason_code` bit31），控制器
在 `kIdle` 下据此进入 `kBootstrap`，先建立合法的原点保持流，随后状态机再发
`176`（Offboard）/`400`（arm）。

**进入门禁**（同时满足，每拍重估）：

```text
最新意图 type == kTakeoff && reason_code bit31 && 未超龄
  && 快照源已注入 && connected
  && landed_state_valid && landed && !armed
  && local_position_valid && attitude_valid && home_valid
  && 本轮起飞请求未超过 bootstrap_timeout
```

- **不读取任何 RC 字段**（`rc_state_valid` / `rc_connected` /
  `manual_override_active`）：机载不做 RC 在线/接管门禁，RC 由 PX4 自己管理；
  无 RC 也必须能自动起飞。
- `bootstrap_timeout`（`controller.bootstrap_timeout_ms`，默认 6000ms）是
  **防卡死兜底**：状态机 S3 的 `mode_timeout`/`arm_timeout` 才是权威。
  装配时由 `DroneApplication` 从 `takeoff.bootstrap_timeout` 注入。
- 同一轮起飞请求的兜底窗口以**首次进入 kBootstrap 的时刻**为起点，失败重试
  不续期；状态机中止（意图不再是新鲜 `kTakeoff`）后才复位窗口。

**输出**（每拍，进入当拍即开始）：

- `SetpointType::kPosition`，`x/y/z` = 进入时最新 `kTakeoff` 意图的
  `target_x/y/z`（局部 NED 起飞原点），`valid=true`；
- 航向保持进入时意图的 `yaw_deg`（按 `use_yaw = yaw_deg != 0` 约定透传）；
- 不带速度前馈（掩码 0）；`header.frame_id` 由 `Publish()` 统一填 3。

**迁移**：

- `armed && connected && Offboard(6/0)` → `kActive`；迁移当拍仍发布
  `valid=true` 的原点保持，避免空档（下一拍起走 ACTIVE 正常路径）；
- 条件失效 / 意图撤销或超龄 / 超时 → `kIdle` + `PublishInvalidStop`
  （`valid=false`），**绝不进入 `kCoast`**——COAST 是"已获授权后意图超龄
  刹车"的语义，bootstrap 未获授权不能刹车保持；
- 进入 `kIdle` 时沿用既有行为清零状态机显式请求锁存。

## 多边形围栏钳制（2026-10-29）

`SetFencePolygonSource(const common::IFencePolygonSource*)` 由装配层（`DroneApplication::BindTopics()`）
在 Start 前注入，实现者是 `MissionStateMachine`。控制周期内用原子指针读取最新多边形快照，
`PublishPosition()` 处理顺序：

1. 读取 `LatestFencePolygon()`；`vertices.size() >= 3` 才视为生效（否则完全回退旧行为）。
2. 位置设定值经 `fence::ClampIntoPolygon(vertices, {x,y}, controller.fence_margin_m)` 回拉，
   位移 > 1e-6m 即计数 `clamped_count_` 并打节流 WARN（带 zone_id/version、位移、累计次数）。
3. 圆形半径围栏（`controller.max_radius_m`）继续执行，作为**绝对兜底**：
   多边形来源失效或顶点不足时，仍按 Home 半径保护。
4. 高度上限：多边形生效且 `alt_max_m > 0` 时用区域高度带上限，否则用 `controller.max_altitude_m`。
5. 航向：`setpoint.use_yaw = intent.yaw_valid || intent.yaw_deg != 0.f`——新增
   `ControlIntent.yaw_valid` 解决“正北 0° 无法表达”的历史问题；S12 返航用它
   请求机头朝飞行方向，其余位置意图不置位即保持原机头平移。

取舍：钳制放在控制器而不是状态机，是为了让**任何来源**的位置意图（自动驾驶、视觉伺服、
返航、测试钩子）都经过同一道几何闸门，避免某条路径忘记钳制。

## Home 相对围栏（Task 5）

仅作用于位置式意图（`kPositionGuide`）；速度式意图受速度幅值上限约束，
围栏语义由 PX4 侧 geofence 兜底（本控制器不做速度→位置积分预测）。

- **坐标约定**：意图目标为局部 NED，项目约定局部 NED 原点即 PX4 Home
  （`docs/数据接口文档.md`、`include/perception/目标估计器.md`），故目标值
  本身即 Home 相对量；`LatestState()` 快照的 `home_valid` 是该约定的
  可信基准门——`home_valid=false` 时位置式意图拒绝输出并计入
  `RejectedIntentCount`（无基准不可围栏）。
- **水平半径**：`hypot(x, y) > max_radius_m` 时等比钳制到边界（方向不变）。
- **高度**：NED z 向下为正，相对高度 = `-z`；超 `max_altitude_m` 钳制到
  `z = -max_altitude_m`（不设下限，近地/贴地由任务层负责）。
- 每次钳制生效 `ClampedCount()+1` 并节流 WARN（第 1 次与每满 100 次）。

## 位置速度前馈（P3，2026-10-19）

位置式意图可携带逐轴速度前馈，经 `Px4Link` 与位置同时下发（PX4 位置环把它作为
速度前馈叠加到位置 P 输出，消除恒速目标的尾追稳态滞后）。边界：

- **配置**：`controller.enable_position_velocity_feedforward`（默认 `false`）、
  `controller.max_feedforward_mps`（默认 `2.0`，独立于 `max_speed_mps`；
  后者只约束速度式意图）。开关关闭时完全忽略意图前馈，输出与旧版逐位等价。
- **来源**：仅 S5 `kPositionGuide` 使用（状态机从 `TargetState.velocity_valid_mask`
  逐轴填充）；S6 冻结段与 S7 速度意图不带前馈。
- **流水线（`PublishPosition`，围栏钳制之后）**：
  1. 掩码清洗：未知位丢弃并节流 WARN；
  2. 逐轴有限性降级：置位轴非有限 → 只清该轴、值写 0、`FeedforwardDroppedCount+1`，
     **位置照常发布**（前馈是增强项，不因坏值拒绝整条意图）；
  3. 水平有效分量 `hypot` 超 `max_feedforward_mps` → 等比缩放，`ClampedCount+1`；
     垂直 D 分量按绝对值保号钳制，`ClampedCount+1`；
  4. 写入 `Px4Setpoint.feedforward_*`。
- **不做 slew**：与位置通道“透传不 slew”一致；前馈阶跃由 PX4 速度环/倾角限制吸收。
- **不参与位置围栏**：围栏只钳制位置目标；前馈是速度量，边界附近可能产生瞬态
  过冲，由 PX4 geofence 兜底（与速度式意图的既有语义一致）。
- **无扰衔接**：位置→速度切换仍由 `PublishVelocity` 的“上一输出非速度式从 0
  起步”守卫处理，前馈不参与斜坡初值。
- **坐标系**：前馈为局部 NED（与位置目标同帧），控制器不做二次旋转。
- **诊断/日志**：`ClampedCount` 现包含速度幅值 + 位置围栏 + 位置前馈钳制；
  新增 `FeedforwardDroppedCount()`（非有限/未知掩码丢弃轴次）。启动日志带
  `feedforward=on/off cap=...`；钳制/丢弃按第 1 次与每满 100 次节流 WARN。
- **测试**：`FlightControllerPipelineTest.PositionFeedforwardIgnoredWhenSwitchOff`；
  `FlightControllerFeedforwardTest.*`（透传、水平/垂直独立钳制、逐轴非有限降级、
  位置→速度从 0 起步、速度意图忽略前馈字段）；跨模块 PTY 契约
  `FlightControllerLinkContractTest.PositionWithFeedforwardReachesPx4LinkWire`。

## Home 漂移对控制的影响（2026-10-16 结论）

控制链里 Home 只从三处进入。**速度型控制不受影响**（速度/角速度是平移不变量，
与坐标原点无关；S7 锁定后由视觉闭环修正残余），受影响的是位置作用点、围栏与
经估计器的交接准入。

### 两套坐标系的差：H

- **锚点坐标系**：PX4 `LOCAL_NED` 的真实原点 = EKF 投影参考锚点
  （`GPS_GLOBAL_ORIGIN` #49，2026-10-16 实测稳定），飞机的 `局部位置`、setpoint
  执行与 PX4 板载闭环全在这套系里；
- **Home 坐标系**：目标估计器把地面站目标换算成"相对 Home 的 NED"
  （`GeodeticToLocalNed(home, target)`），本控制器按"局部 NED 原点即 Home"的
  约定原样下发（见上节坐标约定）；
- 两套原点的差 **H = Home 点在锚点坐标系里的位置**。

### 地面（未解锁）与空中（解锁/起飞后）

PX4 的 Home 生命周期（`Commander.cpp:1841`、`2145-2150`，详见
`docs/故障排查手册.md` §12.1）：

- **未解锁 + 落地 + 无任务**：Home 持续贴住飞机当前位置（阈值水平
  max(2×eph,1m)、垂直 max(2×epv,1.5m)）→ **H ≈ 飞机自身局部坐标**，误差
  ≤ 阈值（1~2m）。漂移的大幅摆动被"贴住"机制吸收，不会以同等幅度进入控制；
  副作用是估计器随每次 Home 重设而重置（位置意图小台阶）——**该瞬态已随方案 C
  消除**（2026-10-16：估计器改锚点基准，Home 变化不再重置）；
- **解锁即冻结**（起飞瞬间再重设一次）→ **H 固定 = 起飞时刻飞机的局部坐标**。

### 空中固定偏置 H 的三个后果（示意）

```text
        ↑北
   45m  ✖ 真目标
        ┆
        ┆ ↕ 差 H = 5m        ← S5 引导终点偏 H
   40m  ✈ 飞机实际到达（PX4 按锚点系执行 40）
        ┆
    5m  ◆ Home（起飞瞬间冻结）
        ┆ ↕ H = 5m
    0   ● 锚点原点（上电点，局部位置 0）
```

数字例子：飞机上电后原地漂到局部 (5,0,0) → 地面期间 Home 跟到 (5,0,0)（H=5m）→
解锁起飞冻结 H；地面站在局部 (45,0,0) 放真目标 → 估计器输出"相对 Home"=(40,0,0)
→ PX4 按锚点系执行 40 → **飞机停在离真目标 5m（=H）处**。

1. **位置式意图（S5/S6）**：终点偏 H。若上电点与起飞点相距 L（上电后搬机），
   地面期间 Home 一路跟随，H≈L，空中偏置即为 L；
2. **围栏**：圆心为 Home，而执行在锚点系 → 围栏约束的是"指令坐标"，实际安全圈
   相对 Home 偏 H（H=5m/半径 100m 时可忽略；搬机场景需计入）；
3. **经估计器的准入**：视觉观测以**锚点系**表达（自机 `local_*` + 姿态），
   目标估计以 **Home 系**表达，两者创新偏差 = H，而 S7 交接创新门限为 3m ——
   **|H| > 3m 时交接判定会被吃掉**（属控制准入问题，不只是精度问题）。

### 观测与处置

- 观测（方案 C 前）：地面阶段位置意图可随每次 `Home变化` 出现小台阶（估计器重置瞬态）；
  方案 C 后该台阶已消除，Home 变化日志改为「首条明细 + 30s 窗口汇总」；
  空中看 `目标估计已接受视觉观测 … 创新=` 是否稳定在噪声量级；
- **处置结果（2026-10-16 晚，方案 C 已落地）**：目标估计器改以锚点为换算原点、
  重置触发改锚点变化 → **目标 NED 与自身/视觉同帧，H 归零**：S5 终点偏置与
  S7 交接创新偏差同时消除，地面阶段的估计器重置瞬态一并消失。决策：D1 等待锚点、
  D2 围栏语义按锚点原点（上电点即围栏中心）、D3 不留回退开关。清单与实施记录：
  `docs/方案C锚点基准实施清单.md`（C1~C3 完成，ctest 437/437）。

## 关键实现

- **固定周期线程**：`Run()` 以 `config_.period` 为节拍
  （`next += period; sleep_until(next)`），每拍执行一次 `Tick(MonotonicMs())`；
  **Tick 超时停摆后钳制下一拍起点**（`if (next < now) next = now`），
  不连发补偿拍——追赶无意义（上游意图只留最新，限幅器以固定 dt 推进）。
- **意图消费**：`DrainIntents()` 循环 `TryTake()` 排干订阅队列，只保留通过
  入口门禁的最新一条（`latest_intent_`）。控制器是意图的接收边界：上游未打
  `receive_time_ms` 时补当前单调时钟，保证超龄判定有可信取样时间。
- **入口门禁（排干时逐条执行，拒绝即计数）**：
  1. 有限性：`target_x/y/z/yaw_deg/yaw_rate_dps/max_speed_mps` 全部
     `std::isfinite`，NaN/Inf 直接拒绝（`RejectedIntentCount+1`）；
  2. 超龄：`now >= valid_until_ms`（含到达即超龄）或
     `StalenessChecker::IsStale(receive_time_ms, now, intent_timeout)`。
- **控制权门禁**：每拍执行 `AuthorityGateOpen()`（快照核验 + 请求位），
  按四态迁移表驱动状态机；详见上节「控制权状态机」。
- **bootstrap 分流顺序**：`DrainIntents()` 之后、常规授权门禁之前先处理——
  `kBootstrap` 每拍走 `HandleBootstrapTick()`；`kIdle` 先试
  `BootstrapEntryGateOpen()`，进入当拍直接发布原点保持并 `return`
  （不落入 IDLE→ACTIVE 分支）。
- **持有期再超龄**（ACTIVE 路径）：意图在消费间隔内过期 → 拒绝计数 +
  转 COAST 并发布零速刹停设定值（`SetpointType::kVelocity`，三分量 0，
  `valid=true`）；COAST 中超时/门禁失效转 IDLE 并发布 `valid=false` 停发
  （Px4Link 收到 `valid=false` 即停止向 PX4 发送，另有 500ms 硬超时兜底）。
- **未知来源安全降级**：`control_source > 4`（已定义：0 位置/1 视觉/2 安全/
  3 遥控器/4 PX4 保护）→ 节流 WARN + 按 `kSafety`(2) 语义处理——仅
  `kBrakeHover` 刹停类意图生效，其余抑制输出（不崩溃不直通），计入
  `ErrorCount`。
- **限幅流水线**（`PublishVelocity()`，热路径零分配，限幅器为成员跨周期
  保持状态）：
  1. **速度幅值限幅**：合速度 `|v| > max_speed_mps` 时等比钳制到上限
     （方向不变），每次生效 `ClampedCount+1` 并节流 WARN；
  2. **无扰衔接**：`control_source` 或意图类型变化时重建 slew 限幅器，
     斜坡起点=上一周期实际输出（`last_output_`）；**上一输出非速度式
     （位置/首拍）时量纲不同，从 0 起步**——防止位置值被当成速度初值
     引发跳变。刹停发布时 slew 状态归零（`SlewRateLimiter::Reset` 语义），
     保证恢复跟踪后指令从零渐变；
  3. **slew 加速度限幅**：`SlewRateLimiter(max_accel_mps2)` 按固定
     `dt=period` 推进，单步变化 ≤ `max_accel_mps2·period`；`dt<=0`
     （时钟回退/首拍）时限幅器冻结输出保持上一值——既有语义，不改。
     时钟回退检测：`now_ms <= last_output_ms_` 时本拍 `dt=0`。
- **坐标系标注（stage4 门禁实现期代码核对发现并补入）**：所有发布路径
  （速度/位置/刹车/停发）在 `Publish()` 统一写入 `header.frame_id = 3`
  （局部 NED，编码见 `include/common/types.h`）——`Px4Link::IsSetpointStructurallyValid`
  强制核验该值，缺失会使控制器全部设定值被静默拒绝（零发送）。
  跨模块回归网：`tests/control/flight_controller_link_contract_test.cpp`
  （PTY 真实链路契约用例，见下「测试方式」）。
- **类型映射**：`kVelocityHeading`→`SetpointType::kVelocity`（限幅流水线）；
  `kPositionGuide`→`kPosition`（**透传位置目标不做 slew**——位置阶跃由 PX4
  内部位置控制器平滑，本控制器只管速度式连续性；**发布前经 Home 相对围栏
  钳制**，见「Home 相对围栏」节；P3 开关打开时按「位置速度前馈」节处理
  逐轴前馈与独立幅值钳制）；`kBrakeHover`→零速刹停；
  起飞/降落/返航/人工接管等类型暂不支持，计入 `ErrorCount`。
- **航向通道透传**：`use_yaw = (yaw_deg != 0)`、
  `use_yaw_rate = (yaw_rate_dps != 0)`；互斥约束沿用 Px4Link 现有校验
  （`use_yaw` 与 `use_yaw_rate` 不得同时为 true，违例由 Px4Link 拒发）。
- **线程安全**：`latest_intent_`/`setpoint_sequence_`/限幅器与衔接状态/
  `state_machine_requested_`/`coast_since_ms_` 仅周期线程访问；诊断计数与
  控制权状态为原子量，跨线程只读。
- **生命周期**：`Start()`/`Stop()` 幂等（`compare_exchange`/`exchange`）；
  构造不调用 `drone::config::Validate()`（加载期 `LoadAppConfig` 已校验，
  且避免 drone_control_core → drone_config 链接依赖反转），但对
  `period<=0` 做本地防御——构造期抛 `std::invalid_argument`
  （否则周期线程 `sleep_until` 立即返回、退化为忙等空转）；析构 RAII
  自动 Stop 并 join 线程。

## 日志行为

- INFO：创建、启动（带 period/intent_timeout/coast_timeout/
  bootstrap_timeout）、停止（带 setpoint/拒绝计数汇总）。
- WARN：**控制权状态迁移**（IDLE/BOOTSTRAP/ACTIVE/COAST 进出，含原因串，
  每次迁移一条；bootstrap 失败原因如"bootstrap 超时"/"起飞意图超龄"/
  "已 armed 但未进入 Offboard"）；拒绝意图（非有限/超龄/无 Home 基准位置意图）、不支持的意图
  类型、速度幅值限幅触发、围栏钳制触发、未定义控制来源降级、TestTick
  运行态误调用——节流打印（第 1 次与每满 100 次），控制周期热路径禁日志。

## 测试方式

`tests/control/flight_controller_test.cpp`（目标 `flight_controller_test`）：

- `IdlePublishesNothingWithoutAuthority`：未注入快照源（影子）下有意图也
  不发布任何 setpoint（`SetpointCount()==0`、输出订阅为空）。
- `RejectsNanIntent`：NaN 意图字段 → 拒绝计数递增。
- `RejectsAlreadyStaleIntent`：`valid_until_ms` 到达即过期 → 拒绝计数递增。
- 限幅流水线用例（`FlightControllerPipelineTest` 夹具：合成时间轴 +
  MockPx4Link Offboard 快照 + 请求位意图 + `TestForceAuthority(kActive)` +
  `TestTick` 同步驱动，确定性无睡眠）：
  - `SpeedMagnitudeClamped`：合速度 5m/s 超 `max_speed_mps=2` → 等比
    钳制到 2、方向比不变、`ClampedCount>0`；
  - `AccelerationSlewLimitsStepResponse`：阶跃 0→2m/s，相邻周期输出差
    ≤ `max_accel_mps2·period`，首拍从 0 起步只推进一拍；
  - `BumplessTransitionStartsFromLastOutput`：`control_source` 0→1 切换，
    新意图首条输出斜坡起点=上一周期输出（不回零）；
  - `ClockRegressionFreezesOutput`：now 回退（dt≤0）输出保持上一值；
  - `UnknownControlSourceTreatedAsSafe`：`control_source=99` → 速度意图
    抑制 + `ErrorCount` 计数，刹停意图放行；
  - `PositionGuidePassthrough`：位置式透传（首拍即目标值、不 slew、
    类型映射 `kPosition`、航向透传）；
  - `PositionToVelocitySlewStartsFromZero`：位置→速度类型切换，速度
    斜坡从 0 起步（量纲守卫）。
- 位置速度前馈用例（P3）：
  - `PositionFeedforwardIgnoredWhenSwitchOff`：开关默认关闭时忽略意图前馈；
  - `FlightControllerFeedforwardTest.PositionFeedforwardPassthrough`：开关打开时
    位置与逐轴前馈原值透传；
  - `HorizontalAndVerticalClampedIndependently`：水平模长/垂直分量分别钳制，
    `ClampedCount==2`；
  - `PerAxisFiniteDegradationKeepsPosition`：坏前馈轴丢弃（计数）但位置照发；
  - `PositionToVelocitySlewStartsFromZero`：前馈不参与位置→速度斜坡初值；
  - `VelocityIntentIgnoresFeedforwardFields`：速度意图不读前馈字段。
- 控制权三态用例（`FlightControllerAuthorityTest` 夹具：MockPx4Link 注入
  快照 + 合成时间轴，配置 period=20ms/intent_timeout=100ms/coast_timeout=
  400ms）：
  - `ActiveOnlyInOffboardAndArmed`：未 armed 保持 IDLE；Offboard(6/0)+armed
    +请求位 → ACTIVE 出输出；模式变 4/3(AUTO/LOITER) → 下一拍
    `valid=false` 停发并转 IDLE；
  - `ModeFlapDoesNotChatterAuthority`：Offboard↔Position 抖动——每次离开
    即 IDLE 停发；抖回 Offboard 无新意图时保持 IDLE（请求锁存已清零）；
    状态机重新显式请求才回 ACTIVE；
  - `CoastBrakesThenIdles`：ACTIVE 中意图断粮 → 超龄转 COAST 零速刹车；
    COAST 中意图恢复回 ACTIVE；再次断粮转 COAST 后 coast_timeout 耗尽
    → IDLE + `valid=false` 停发；
  - `StaleRequestDoesNotActivateAuthority`：请求位意图到达后长期 IDLE
    （无新意图），授权链事后就绪时陈旧请求不得激活 ACTIVE（评审
    fix round 2 回归）；补新鲜请求位意图后才转 ACTIVE；
  - `FenceClampsPositionIntent`：位置意图超 `max_radius_m`/`max_altitude_m`
    → 钳制到边界 + `ClampedCount` 计数；`home_valid=false` → 位置式意图
    拒绝输出并计 `RejectedIntentCount`。
- 自动起飞预授权用例（`FlightControllerBootstrapTest` 夹具：地面/未解锁
  MockPx4Link 快照 + 合成时间轴，`bootstrap_timeout=200ms`）：
  - `EntersBootstrapAndHoldsOriginWithoutRc`：kTakeoff(bit31) + 地面条件
    → `kBootstrap`，输出 `kPosition` 原点、`valid=true`、frame=3、无前馈；
    快照 RC 字段全为 false/invalid 不影响进入；
  - `EntryRequiresGroundConditions`：`armed`/落地状态无效/未落地/本地位置/
    姿态/Home 无效/未连接任一失效 → 保持 IDLE 且无 setpoint；
  - `ArmedOffboardTransitionsToActive`：`armed+Offboard` 当拍转 ACTIVE，
    该拍仍发布 `valid=true` 原点保持；
  - `TimeoutReturnsIdleWithoutCoast`：超过 `bootstrap_timeout` → IDLE +
    `valid=false`，不是 COAST；状态机重发也不立刻重新进入；
  - `StopsBootstrapWhenIntentRevoked` / `StopsBootstrapWhenIntentTypeChanges`
    / `StopsBootstrapWhenGroundConditionsLost`：请求位撤销、类型变化、
    外部打断（armed 但非 Offboard）→ IDLE 停流；
  - `DoesNotEnterWithoutValidTakeoffRequest`：无意图/非 kTakeoff/无 bit31
    均不进入；
  - `DoesNotEnterWithStaleTakeoffIntent`：到达即超龄 → 拒绝计数、不进入。

```bash
ctest --test-dir build -R FlightController --output-on-failure
```

## 排查要点

- 无设定值输出：先查 `Authority()`——未注入快照源 / 快照非
  Offboard(6/0)+armed / 意图未带请求位（reason_code bit31）都会停在
  `kIdle`；再查 `RejectedIntentCount()` 是否递增（意图被门禁拒绝）；
  ACTIVE 下仍无输出时查 `ErrorCount()`——未定义 `control_source`（>4）
  会抑制非刹停意图。
- 控制权反复 IDLE：看 WARN 迁移日志原因串（"授权链断开"/"COAST 超时"）；
  模式抖动场景每拍核验，离开 Offboard 当拍即撤属预期（人工接管语义）。
- 自动起飞停在 IDLE、没有 bootstrap 流：查迁移日志——`IDLE -> BOOTSTRAP`
  从未出现时核对 kTakeoff 意图（类型/bit31/未超龄）与快照
  `connected/landed_state_valid&&landed/!armed/local/attitude/home`；
  **不要查 RC**（门禁不读 RC）。出现 `BOOTSTRAP -> IDLE` 时看原因串
  （"bootstrap 超时"/"起飞意图超龄"/"已 armed 但未进入 Offboard"）；
  bootstrap 失败后 `SetpointOutput()` 会收到一条 `valid=false` 停流。
- 位置意图被钳小/被拒：查 `ClampedCount()` 是否递增（围栏生效）与
  `max_radius_m`/`max_altitude_m` 配置；`home_valid=false` 时位置式意图
  一律拒绝（无基准不可围栏）——先查 PX4 Home 上报链路。
- 输出被钳小/爬坡慢：查 `ClampedCount()` 是否递增（幅值限幅生效）与
  `max_speed_mps`/`max_accel_mps2` 配置；slew 单步上限 =
  `max_accel_mps2 × period`。
- 意图全部被判超龄：确认上游打戳 `receive_time_ms`/`valid_until_ms`
  使用同一单调时钟源（`steady_clock` 毫秒）；未打 `receive_time_ms` 时
  控制器补接收时刻，但 `valid_until_ms=0` 会被判立即过期。
- 拒绝/限幅日志节流：第 1 次与每满 100 次才打印，计数接口为准。
