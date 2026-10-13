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

## 控制权状态机（spec §4，Task 5 接入）

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
| IDLE | 授权链全通且持有新鲜请求位意图 | ACTIVE | 无（随后走正常发布路径） |
| IDLE | 授权链不通或请求意图已超龄 | IDLE | 停发（不发布任何 setpoint） |
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
  按三态迁移表驱动状态机；详见上节「控制权状态机」。
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
  内部轨迹规划平滑，本控制器只管速度式连续性；速度上限写入由 Px4Link 侧
  type mask 语义保证；完整联调测试归 Task 7；**发布前经 Home 相对围栏
  钳制**，见「Home 相对围栏」节）；`kBrakeHover`→零速刹停；
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

- INFO：创建、启动（带 period/intent_timeout/coast_timeout）、停止
  （带 setpoint/拒绝计数汇总）。
- WARN：**控制权状态迁移**（IDLE/ACTIVE/COAST 进出，含原因串，每次迁移
  一条）；拒绝意图（非有限/超龄/无 Home 基准位置意图）、不支持的意图
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
