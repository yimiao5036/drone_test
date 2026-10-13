# flight_controller —— 飞行控制器（IFlightController / FlightController / FlightControllerStub）

## 功能边界

消费状态机发布的 `ControlIntent`，经门禁链（有限性/超龄/控制权）后生成
`Px4Setpoint` 发布，供 PX4 通信线程发送。属于执行层入口，是正式控制输出
的唯一来源。

当前阶段（阶段四控制接入 Task 3/4）边界：

- `FlightController` 正式实现骨架已落地：固定周期线程、意图队列排干
  （只留最新）、入口门禁（有限性/超龄拒绝计数）、控制权 IDLE 门禁。
- **限幅流水线与无扰衔接已接入（Task 4）**：速度幅值等比钳制 →
  `SlewRateLimiter` 加速度限幅；来源/类型变化时斜坡起点=上一周期实际
  输出；时钟回退冻结输出；未定义 `control_source`（>4）按安全语义降级
  （仅刹停类意图生效）。
- **控制权默认 `kIdle`，不发布任何设定值**——Task 5 才接入授权
  （`kActive`/`kCoast`）与 PX4 快照门禁；单测可经 `TestForceAuthority()`
  强制放行发布路径。
- `FlightControllerStub` 骨架占位保留不动，供骨架冒烟测试使用。
- `SetPx4LinkSource()` 接口已预留（存指针，不做判定），Task 5 注入
  `communication::IPx4Link::LatestState()` 只读快照后启用。

## 接口与数据流

```text
common::Topic<ControlIntent>（状态机，kControlIntent）
    ──► FlightController（固定周期线程 Tick）
        │ 门禁链：排干队列只留最新 → 有限性 → 超龄（valid_until + intent_timeout）
        │        → 控制权（IDLE 直接返回）→ 持有期再超龄 → 刹停降级
        │ 限幅流水线：未知来源安全降级 → 速度幅值等比钳制
        │        → 无扰衔接（斜坡起点=上一周期实际输出）→ slew 加速度限幅
        ▼
common::Topic<Px4Setpoint> ──► IPx4Link（PX4 通信线程）
```

- `SetInput(Topic<ControlIntent>&)`：绑定意图输入，订阅容量 4、丢旧保新。
  **须在 `Start()` 前调用**（Start 后周期线程即开始消费订阅）。
- `SetpointOutput()`：设定值输出 Topic。
- `Authority()` / `RejectedIntentCount()` / `ClampedCount()` /
  `SetpointCount()` / `ErrorCount()`：诊断计数，原子量，供测试与门禁使用。
  `ClampedCount()` 为速度幅值限幅触发累计计数（每次钳制生效 +1，
  供 G2 验收核对）。
- `SetPx4LinkSource(const communication::IPx4Link*)`：注入 PX4 只读快照源
  （Task 5 门禁用，本任务仅保存）。
- 测试入口（生产装配不得调用）：`TestForceAuthority()` 强制控制权状态
  （Task 5 授权链接入前的单测放行手段，公开诊断方法而非 `#ifdef` 测试钩）；
  `TestTick(now_ms)` 以指定时间戳同步执行一拍（须在未 Start 状态调用，
  用于时钟回退等时序用例的确定性驱动）。

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
- **控制权门禁**：`authority_ == kIdle` 时 Tick 直接返回，不发布任何
  设定值（默认 IDLE，Task 5 接入授权链）。
- **持有期再超龄**（授权后路径，Task 5 启用）：意图在消费间隔内过期 →
  拒绝计数 + 发布零速刹停设定值（`SetpointType::kVelocity`，三分量 0，
  `valid=true`）；完整 COAST 状态转移由 Task 5 实现。
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
- **类型映射**：`kVelocityHeading`→`SetpointType::kVelocity`（限幅流水线）；
  `kPositionGuide`→`kPosition`（**透传位置目标不做 slew**——位置阶跃由 PX4
  内部轨迹规划平滑，本控制器只管速度式连续性；速度上限写入由 Px4Link 侧
  type mask 语义保证；完整联调测试归 Task 7）；`kBrakeHover`→零速刹停；
  起飞/降落/返航/人工接管等类型暂不支持，计入 `ErrorCount`。
- **航向通道透传**：`use_yaw = (yaw_deg != 0)`、
  `use_yaw_rate = (yaw_rate_dps != 0)`；互斥约束沿用 Px4Link 现有校验
  （`use_yaw` 与 `use_yaw_rate` 不得同时为 true，违例由 Px4Link 拒发）。
- **线程安全**：`latest_intent_`/`setpoint_sequence_`/限幅器与衔接状态
  仅周期线程访问；诊断计数与控制权状态为原子量，跨线程只读。
- **生命周期**：`Start()`/`Stop()` 幂等（`compare_exchange`/`exchange`）；
  构造不调用 `drone::config::Validate()`（加载期 `LoadAppConfig` 已校验，
  且避免 drone_control_core → drone_config 链接依赖反转），但对
  `period<=0` 做本地防御——构造期抛 `std::invalid_argument`
  （否则周期线程 `sleep_until` 立即返回、退化为忙等空转）；析构 RAII
  自动 Stop 并 join 线程。

## 日志行为

- INFO：创建、启动（带 period/intent_timeout/coast_timeout）、停止
  （带 setpoint/拒绝计数汇总）。
- WARN：拒绝意图（非有限/超龄）、不支持的意图类型、速度幅值限幅触发、
  未定义控制来源降级——节流打印（第 1 次与每满 100 次），控制周期
  热路径禁日志。

## 测试方式

`tests/control/flight_controller_test.cpp`（目标 `flight_controller_test`）：

- `IdlePublishesNothingWithoutAuthority`：默认 IDLE 下有意图也不发布
  任何 setpoint（`SetpointCount()==0`、输出订阅为空）。
- `RejectsNanIntent`：NaN 意图字段 → 拒绝计数递增。
- `RejectsAlreadyStaleIntent`：`valid_until_ms` 到达即过期 → 拒绝计数递增。
- 限幅流水线用例（`FlightControllerPipelineTest` 夹具：合成时间轴 +
  `TestForceAuthority(kActive)` + `TestTick` 同步驱动，确定性无睡眠）：
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

```bash
ctest --test-dir build -R FlightController --output-on-failure
```

## 排查要点

- 无设定值输出：先查 `Authority()`（默认 `kIdle` 属预期，Task 5 接入授权
  链），再查 `RejectedIntentCount()` 是否递增（意图被门禁拒绝）；授权后
  仍无输出时查 `ErrorCount()`——未定义 `control_source`（>4）会抑制
  非刹停意图。
- 输出被钳小/爬坡慢：查 `ClampedCount()` 是否递增（幅值限幅生效）与
  `max_speed_mps`/`max_accel_mps2` 配置；slew 单步上限 =
  `max_accel_mps2 × period`。
- 意图全部被判超龄：确认上游打戳 `receive_time_ms`/`valid_until_ms`
  使用同一单调时钟源（`steady_clock` 毫秒）；未打 `receive_time_ms` 时
  控制器补接收时刻，但 `valid_until_ms=0` 会被判立即过期。
- 拒绝/限幅日志节流：第 1 次与每满 100 次才打印，计数接口为准。
