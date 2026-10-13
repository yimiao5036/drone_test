# flight_controller —— 飞行控制器（IFlightController / FlightController / FlightControllerStub）

## 功能边界

消费状态机发布的 `ControlIntent`，经门禁链（有限性/超龄/控制权）后生成
`Px4Setpoint` 发布，供 PX4 通信线程发送。属于执行层入口，是正式控制输出
的唯一来源。

当前阶段（阶段四控制接入 Task 3）边界：

- `FlightController` 正式实现骨架已落地：固定周期线程、意图队列排干
  （只留最新）、入口门禁（有限性/超龄拒绝计数）、控制权 IDLE 门禁。
- **控制权恒为 `kIdle`，不发布任何设定值**——Task 5 才接入授权
  （`kActive`/`kCoast`）与 PX4 快照门禁；限幅与无扰衔接由 Task 4 接入。
- `FlightControllerStub` 骨架占位保留不动，供骨架冒烟测试使用。
- `SetPx4LinkSource()` 接口已预留（存指针，不做判定），Task 5 注入
  `communication::IPx4Link::LatestState()` 只读快照后启用。

## 接口与数据流

```text
common::Topic<ControlIntent>（状态机，kControlIntent）
    ──► FlightController（固定周期线程 Tick）
        │ 门禁链：排干队列只留最新 → 有限性 → 超龄（valid_until + intent_timeout）
        │        → 控制权（IDLE 直接返回）
        ▼
common::Topic<Px4Setpoint> ──► IPx4Link（PX4 通信线程）
```

- `SetInput(Topic<ControlIntent>&)`：绑定意图输入，订阅容量 4、丢旧保新。
- `SetpointOutput()`：设定值输出 Topic。
- `Authority()` / `RejectedIntentCount()` / `SetpointCount()` /
  `ErrorCount()`：诊断计数，原子量，供测试与门禁使用。
- `SetPx4LinkSource(const communication::IPx4Link*)`：注入 PX4 只读快照源
  （Task 5 门禁用，本任务仅保存）。

## 关键实现

- **固定周期线程**：`Run()` 以 `config_.period` 为节拍
  （`next += period; sleep_until(next)`），每拍执行一次 `Tick(MonotonicMs())`。
- **意图消费**：`DrainIntents()` 循环 `TryTake()` 排干订阅队列，只保留通过
  入口门禁的最新一条（`latest_intent_`）。控制器是意图的接收边界：上游未打
  `receive_time_ms` 时补当前单调时钟，保证超龄判定有可信取样时间。
- **入口门禁（排干时逐条执行，拒绝即计数）**：
  1. 有限性：`target_x/y/z/yaw_deg/yaw_rate_dps/max_speed_mps` 全部
     `std::isfinite`，NaN/Inf 直接拒绝（`RejectedIntentCount+1`）；
  2. 超龄：`now >= valid_until_ms`（含到达即超龄）或
     `StalenessChecker::IsStale(receive_time_ms, now, intent_timeout)`。
- **控制权门禁**：`authority_ == kIdle` 时 Tick 直接返回，不发布任何
  设定值（本任务恒 IDLE）。
- **持有期再超龄**（授权后路径，Task 5 启用）：意图在消费间隔内过期 →
  拒绝计数 + 发布零速刹停设定值（`SetpointType::kVelocity`，三分量 0，
  `valid=true`）；完整 COAST 状态转移由 Task 5 实现。
- **意图直通**：`PublishFromIntent()` 当前不做限幅（Task 4 接入）——
  `kVelocityHeading`→`kVelocity`、`kPositionGuide`→`kPosition`、
  `kBrakeHover`→零速刹停；起飞/降落/返航/人工接管等类型骨架期不支持，
  计入 `ErrorCount`。
- **线程安全**：`latest_intent_`/`setpoint_sequence_` 仅周期线程访问；
  诊断计数与控制权状态为原子量，跨线程只读。
- **生命周期**：`Start()`/`Stop()` 幂等（`compare_exchange`/`exchange`）；
  构造不重复校验配置（加载期 `LoadAppConfig` 已 `Validate()` fail-fast，
  避免 drone_control_core → drone_config 链接依赖反转）；析构 RAII 自动
  Stop 并 join 线程。

## 日志行为

- INFO：创建、启动（带 period/intent_timeout/coast_timeout）、停止
  （带 setpoint/拒绝计数汇总）。
- WARN：拒绝意图（非有限/超龄）、不支持的意图类型——节流打印
  （第 1 次与每满 100 次），控制周期热路径禁日志。

## 测试方式

`tests/control/flight_controller_test.cpp`（目标 `flight_controller_test`）：

- `IdlePublishesNothingWithoutAuthority`：默认 IDLE 下有意图也不发布
  任何 setpoint（`SetpointCount()==0`、输出订阅为空）。
- `RejectsNanIntent`：NaN 意图字段 → 拒绝计数递增。
- `RejectsAlreadyStaleIntent`：`valid_until_ms` 到达即过期 → 拒绝计数递增。

```bash
ctest --test-dir build -R FlightController --output-on-failure
```

## 排查要点

- 无设定值输出：先查 `Authority()`（本任务恒 `kIdle` 属预期），再查
  `RejectedIntentCount()` 是否递增（意图被门禁拒绝）。
- 意图全部被判超龄：确认上游打戳 `receive_time_ms`/`valid_until_ms`
  使用同一单调时钟源（`steady_clock` 毫秒）；未打 `receive_time_ms` 时
  控制器补接收时刻，但 `valid_until_ms=0` 会被判立即过期。
- 拒绝日志节流：第 1 次与每满 100 次才打印，计数接口为准。
