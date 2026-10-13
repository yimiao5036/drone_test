# 任务状态机实现文档

## 功能职责

`MissionStateMachine`负责消费地面站目标、PX4飞行状态和健康状态快照，按任务阶段维护`MissionStatus`。默认仍是安全影子语义：验证`GroundStationTarget`已经能进入任务链路，并在满足导航快照基本条件时进入`GPS_APPROACH`影子状态。

阶段四新增（Task 7）：经`SetEstimationInputs`绑定目标估计/视觉稳定性/双目测距三个输入后，`GPS_APPROACH`（S5）开始发布`kPositionGuide`位置引导意图（指向估计目标 NED，`control_source=0`，`reason_code`带 bit31 控制权请求位`common::kControlIntentReasonRequestControl`）。**未绑定时不出意图、保持原影子行为**。意图是否真正转化为 PX4 输出由`FlightController`授权链与`enable_control`装配门禁决定，状态机侧不做最终裁决。

阶段四新增（Task 8）：经`SetEstimationDiagnostics`注入估计器诊断接口后，**交接判据**（`handover` 配置节）生效，状态在 S5→S6→S7 之间按下列判据推进/回退（`handover` 阈值全部来自配置，无代码字面量）：

| 迁移 | 判据 |
|---|---|
| S5→S6 | 视觉 `LOCKED` 连续帧 ≥ `lock_frames` + 双目距离有效且未超龄 + 视线角距视场边缘 ≥ `fow_margin_deg` + 视觉观测与卡尔曼预测创新 ≤ `innovation_gate_m`（四项同时满足） |
| S6→S7 | 进入判据持续满足 **且** 视觉链再推进一条新的 `LOCKED` 帧（交接期不断流） |
| S6→S5 | 进入判据任一失效（撤销交接）；或驻留超 `timeout_ms`（首条视觉指令未生效，防卡死） |
| S7→S5 | 三条独立路径任一成立：① 视觉状态消息断流超视觉新鲜度阈值（`visual_stale`）；② 测距失效（invalid 或超龄）连续达 `distance_lost_ms`；③ 视觉 `LOST` 连续帧 ≥ `exit_lost_frames`（进入/退出门限不同构成迟滞，防状态抖动） |

**未注入估计器诊断时交接判据不评估**（保持 S5，启动记 ERROR）：无法核验"视觉锁定与卡尔曼航迹是否同一目标"就不允许切进视觉末段。

本阶段仍不做以下事情：

- 不发送PX4命令、模式切换、解锁、起飞、降落或setpoint（`SetCommandSender`注入的命令接口当前仅持有，供后续 ARM/执行机构任务使用）。
- 不直接调用拦截设备。
- 不把地面站目标透传为PX4航点。
- S6/S7 期间不发布控制意图（视觉伺服意图由 Task 9 落地；当前位置意图只在 S5 发布）；
  设计 §2.2 的"位置误差/地速/首条视觉指令经限幅生效"验证量依赖控制侧反馈与
  尚未冻结的阈值，本任务以"条件持续 + 视觉链推进 + 驻留超时"替代最小验证，
  留待 Task 9/T12 补齐。
- 设计 §2.1「进入 S6 后冻结最后位置目标」的**发布策略**（S6/S7 期间是否继续发布
  冻结的位置意图、直至首条视觉指令被接收）由 **Task 9 显式决定**：本任务既不冻结
  也不发布（位置引导意图随离开 S5 即停发，不引入未消费状态，见 .cpp 注释）；
  Task 9 若决定发布，需同时定义冻结意图与首条视觉意图的切换时序。

完整自动任务链仍以`docs/状态机设计.md`为准；当前跳过`ARMING/TAKEOFF`只用于无控制输出的地面站目标接入验证，后续接入控制门禁后需要恢复完整状态推进。

## 接口与数据流

```cpp
struct MissionStateMachineConfig {
    std::chrono::milliseconds intent_valid_for{300};  // 意图有效期，与 controller.intent_timeout 同源
    config::HandoverConfig handover;                  // 交接迟滞参数（handover 配置节同源）
    control::TrackingCameraGroupConfig camera;        // 标定内参（visual_tracking.camera 同源）
    std::chrono::milliseconds visual_stale{200};      // 视觉状态超龄（visual_tracking.control.visual_stale_ms 同源）
    std::chrono::milliseconds distance_stale{500};    // 测距超龄（distance_stale_ms 同源）
};

class MissionStateMachine final : public IMissionStateMachine {
public:
    explicit MissionStateMachine(MissionStateMachineConfig config = {});
    void SetInputs(Topic<GroundStationTarget>& ground_target,
                   Topic<FlightStateSnapshot>& flight,
                   Topic<HealthStatus>& health) override;
    void SetEstimationInputs(Topic<TargetState>& estimated,
                             Topic<VisualTargetStatus>& visual_status,
                             Topic<StereoTargetDistance>& distance) override;
    void SetCommandSender(communication::IPx4CommandSender* sender) override;
    void SetEstimationDiagnostics(const perception::ITargetEstimator* estimator) override;
    Topic<ControlIntent>& IntentOutput() override;
    Topic<MissionStatus>& StatusOutput() override;
};
```

当前主程序接线：

```text
GroundStationLink::TargetOutput() ─┐
Px4Link::StateOutput() ────────────┼─► SetInputs(...)
HealthManager::Output() ───────────┘

TargetEstimator::EstimatedOutput() ──┐
VisualTargetMonitor::StatusOutput() ─┼─► SetEstimationInputs(...)（阶段四正式化，三者一组）
StereoRanger::DistanceOutput() ──────┘

TargetEstimator（ITargetEstimator）─► SetEstimationDiagnostics(...)（装配估计器即注入，只读诊断）
MissionStateMachine::IntentOutput() ─► FlightController::SetInput(...)
MissionStateMachine::StatusOutput() ─► GroundStationLink::SetMissionStatusInput(...)
Px4Link（IPx4CommandSender）─► SetCommandSender(...)（仅 enable_control=true 装配分支）
```

配置注入：`intent_valid_for`由`DroneApplication`从`config_.controller.intent_timeout`
同源传入，保证意图`valid_until_ms`与控制器意图超龄门禁一致；`handover`、`camera`
与两个超龄阈值分别从`config_.handover`、`config_.visual_tracking.camera`、
`config_.visual_tracking.control.{visual_stale_ms, distance_stale_ms}`同源传入。
`handover` 的关系校验（`exit_lost_frames > lock_frames` 等）由配置加载期
`HandoverConfig::Validate`强制，状态机构造不重复校验（与 FlightController 同约定）。

## 关键实现点

- 状态机拥有独立工作线程，周期为100ms。
- 输入通过`Topic`订阅队列获取：目标队列容量4，飞行/健康/估计/视觉/测距快照队列容量2。
- 每个周期先抽干订阅队列，只保留最新不可变快照，再做状态判断，避免一次决策周期内读到不一致数据。
- 启动后先发布`BOOT`，随后进入`SELF_CHECK`。
- `SELF_CHECK`阶段等待PX4快照满足：连接在线、GPS fix有效、全局位置有效、Home有效。
  PX4 已到位（2026-10-10：装机体+GPS）：正常应能越过自检；仍停留则按
  心跳 → GPS fix → Home 顺序排查（见`docs/故障排查手册.md`），不要改条件绕过。
  **室内提示**：室内可能无 GPS fix/Home，停留自检属预期、不是通信问题
  （判据是「PX4 心跳已连接」是否出现）；分阶段验证见`docs/PX4实链验证清单.md`。
- `READY`阶段等待有效地面站目标；目标有效且导航仍有效时，进入`GPS_APPROACH`影子状态。
- `GPS_APPROACH`阶段维护目标新鲜度和状态回传；估计输入已绑定且估计新鲜有效时
  （`TargetState.valid=true` 且 `receive_time_ms + valid_for_ms` 未过期），
  发布`kPositionGuide`意图：`target_x/y/z = pos_x/y/z_m`（`pos_z_m`为 NaN 时原样
  透传——高度无效的合法语义，由 FlightController 有限性门禁拒绝，状态机侧不得改 0），
  `reason_code = kControlIntentReasonRequestControl | 7（地面站任务下发）`，
  `valid_for_ms/valid_until_ms`来自注入的意图有效期配置。
- 意图节流纪律：内容（目标位置/有效性）无变化按 10Hz 上限重发（意图是周期语义，
  FlightController 50ms 周期消费）；内容变化立即发。NaN 与 NaN 视为内容相等。
- 估计失效或超龄时停止发布意图（不补发安全类意图，刹停语义归后续状态迁移任务）；
  超龄收口由 FlightController 的`intent_timeout`门禁接管。
- 估计/视觉/测距三输入必须成组绑定；只绑部分时记 ERROR 并按未绑定处理（保持影子）。
- **交接判据（Task 8）**：
  - 判定逻辑抽为内部纯函数`HandoverEntryConditionsMet(HandoverInputs, HandoverConfig)`
    （匿名命名空间，供 Task 9 复用）：视觉 `LOCKED` → 距离有效且未超龄 →
    视线角余量（`|视线角| + fow_margin_deg ≤ 视场半角`，方位/俯仰各判一次）→
    创新门限（NaN 视为无值，不放行）。
  - 视线角由视觉平滑中心经`PixelToAngle`（`control/line_of_sight.h`，标定内参
    `fx/fy/cx/cy`）换算，视场半角在构造期由内参推：
    `atan((W−cx)/fx)`、`atan((H−cy)/fy)`。注意主点偏离画面中心（cy≈532@720），
    画面中心不是视线零点；垂直半角取 (H−cy) 侧，对上侧偏保守。
  - **迟滞计数是交接层独立口径**（`consecutive_lock_frames_`/`consecutive_lost_frames_`），
    与`VisualTargetMonitor`内部的`lock_frames`判据是两套东西，不得混用：
    LOCKED 仅在新`frame_sequence`到达时计数（监视器会周期重发同一帧，重发不重复计数）；
    LOST 按每条状态消息计数（观测断流时帧序冻结，按帧序去重会让退出永不触发）；
    ACQUIRING/SEARCHING 两个计数都清零。状态迁移（回退/超时）时复位计数，
    重新进入 S6 必须依赖新的锁定帧（防超时后立即回跳）。
  - 视觉/测距状态消息不携带`valid_for_ms`（生产者未设置），新鲜度按
    `receive_time_ms + 注入超龄阈值`判定（`StalenessChecker`）；估计新鲜
    （`HasFreshEstimate`）是创新门限的前提（卡尔曼必须存活）。
  - 创新值取自`ITargetEstimator::LastVisualInnovationM()`（最近一次接受的视觉观测
    与卡尔曼预测的水平距离）；该接口未注入或返回 NaN 时不放行交接。
  - **S7 退出收口三条路径**（缺一都会让 S7 在异常下永不退出）：
    ① 视觉状态断流——监视器线程死亡/链路中断时无 `LOST` 消息可计数、帧序冻结，
    按「最近一条 `VisualTargetStatus` 的接收年龄 > `visual_stale`」直接退出
    （与 S6 进入判据同一新鲜度阈值、同一`StalenessChecker`判定，见`VisualStatusStale`）；
    ② 测距失效计时——`valid=false` 或超龄即累计，连续达 `distance_lost_ms` 退出，
    中途恢复有效立刻清零（`distance_lost_since_ms_`），覆盖"测距丢失但视觉仍
    锁定、控制律 `no_distance_action` 不触发退出"的场景；
    ③ 视觉 `LOST` 连续帧达 `exit_lost_frames`。
    S7 期间不评估进入条件；健康监视器按 `publish_interval`（100ms）发布，
    200ms 超龄阈值留 2 倍余量，短时丢帧不触发 ①，图像空间 coast 仍归 Task 9
    的视觉伺服（覆盖阈值以内的短时缺口）。
- `SetCommandSender`/`SetEstimationDiagnostics`须在 Start 前调用；运行中注入/重绑输入被拒绝并记 ERROR。
- 控制输出是否接 PX4 仍由`DroneApplication`的`enable_control`门禁决定（绑定
  `Px4Setpoint`+注入快照源+注入命令接口只在 true 分支）；诊断接口注入与
  `enable_control` 无关（只读诊断，影子期即可用）。

## 日志行为

- INFO：状态机创建、启动、停止、销毁；状态转移（含交接进入/验证通过/撤销/超时/
  视觉断流退出/测距失效退出/退出迟滞回退，一一对应中文 reason，便于事后区分
  是哪条退出路径生效）；位置引导意图启动/停止；
  PX4 命令接口与估计器诊断接口注入；首次/节流接收地面站目标。
- WARN：启动前未绑定输入；目标过期；估计失效导致意图停止（节流）；导航状态丢失。
- ERROR：订阅或线程启动异常；运行中重绑输入/注入接口；估计输入部分绑定；
  估计输入已绑定但未注入估计器诊断（交接不评估）。
- 高频周期（100ms 决策、10Hz 意图重发）不打印日志；交接判据每周期静默评估，
  只在状态迁移时打日志；重复异常使用第1次和每100次节流。

## 测试方式

开发机Ubuntu 24.04：

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
./build/mission_state_machine_test
ctest --test-dir build --output-on-failure
```

测试覆盖：

- 未绑定输入时拒绝启动。
- 启动后发布`BOOT -> SELF_CHECK`。
- PX4导航快照有效后进入`READY`。
- 合法`GroundStationTarget`进入`GPS_APPROACH`影子状态并发布`MissionStatus`。
- 未绑定估计输入时`IntentOutput()`不发布自动控制意图（影子行为兜底）。
- 绑定估计输入后 S5 发布`kPositionGuide`意图：指向估计目标、`control_source=0`、
  reason_code 带 bit31 控制权请求位、`valid_until_ms = receive_time_ms + valid_for_ms`。
- 意图有效期来自注入配置（不写死字面量）。
- 估计失效（`valid=false`）后停止发布意图。
- 交接判据（`HandoverFixture` + `MakeHandoverTestConfig`，阈值取小值压缩用例时长；
  估计器诊断用假件注入固定创新值）：
  - `HandoverRequiresConsecutiveLockFrames`：LOCKED 断续不累计（中断清零），
    连续 N 条才进 S6；
  - `HandoverRejectsInnovationBeyondGate`：创新 NaN（无值）与超门限均不放行，
    回到门限内即放行；
  - `HandoverRejectsTargetNearFovEdge`：视线角距视场边缘不足 `fow_margin_deg`
    不放行（像素 x=1240 时方位角约 31.6°，标定内参下水平半角约 33.2°）；
  - `HandoverTimeoutReturnsToGpsApproach`：S6 驻留超 `timeout_ms` 回 S5，
    且不因旧锁定帧立即回跳；
  - `ExitRequiresMoreLostFramesThanEntry`：S7 中 LOST 连续帧 > `lock_frames`
    但 < `exit_lost_frames` 保持 S7，达到门限回退 S5；
  - `ExitWhenDistanceLostBeyondThreshold`：S7 中测距 `valid=false` 持续约 250ms
    （未达 300ms 阈值）后恢复有效 → 计时清零不退出；持续失效达阈值 → 回退 S5
    并停在 S5（锁定计数已复位）；
  - `ExitWhenVisualStreamStalls`：S7 中不再发布视觉状态（等价监视器死亡）→
    未达 `visual_stale` 阈值不退出，超龄后按断流回退 S5；
  - `HandoverAbortsWhenLockLostInHandover`：S6 中视觉丢失 → 立即撤销回 S5；
  - `HandoverRequiresEstimatorDiagnostics`：未注入诊断接口时不放行交接并记 ERROR。

## 排查/修改要点

- 如果地面站目标ACK已成功但状态机不进入`GPS_APPROACH`，先检查PX4快照是否满足`connected/gps_fix/global_position_valid/home_valid`（PX4 已到位后该组条件可实链成立，不再是"必然不满足"）。
- 如果状态机没有状态回传，检查主程序是否启用了PX4和地面站链路，以及`GroundStationLink::TargetOutput()`是否已绑定到状态机。
- S5 不出意图：确认`SetEstimationInputs`三个输入都已绑定（部分绑定按未绑定处理并记 ERROR），且`TargetState.valid=true`且剩余有效期未过期。
- 意图被 FlightController 拒绝但状态机侧正常发布：查 reason_code bit31 是否置位、`valid_until_ms`是否超龄、`target_z`是否为 NaN（高度无效时有限性门禁拒绝属预期）。
- 停在 S5 不进 S6：按四项判据逐项查——① 视觉状态是否 `LOCKED` 且状态消息新鲜
  （`visual_stale`）；② 距离 `valid=true` 且未超龄（`distance_stale`）；
  ③ 视线中心是否贴近画面边缘（垂直方向尤其注意：标定主点 cy≈532，
  目标靠近画面中部上方即可能触发余量门限）；④ 估计器是否已注入诊断接口、
  创新值是否为 NaN 或超 `innovation_gate_m`（视觉首次观测前为 NaN，属预期）。
- S6 很快回 S5：确认视觉链在交接期是否继续产生新的 `LOCKED` 帧（无新帧会走
  `timeout_ms` 超时）；若日志是"进入判据失效"则按上一行四项排查。
- 停在 S7 不退出：按三条退出路径逐项查——① 视觉状态消息是否仍在到达
  （断流超 `visual_stale` 会立即退出，日志 reason="视觉状态断流超龄"）；
  ② 测距是否 `valid=true` 且未超龄（失效连续达 `distance_lost_ms` 会退出，
  reason="双目测距失效达距离丢失门限"）；③ `LOST` 连续帧数是否未达
  `exit_lost_frames`（进入门限约 1s、退出门限约 3s 属设计迟滞——这是唯一
  "应当停在 S7"的情形）。
- S7 提前退出（非预期）：先看状态转移日志的 reason——"断流超龄"说明监视器/
  链路停止发布状态（查监视器线程与上游 YOLO 观测）；"测距失效"说明双目匹配
  连续失效或测距线程停发（查测距组件与视差有效域）；二者都可能在视觉仍
  LOCKED 时发生，属安全收口而非缺陷。
- 后续恢复完整自动链时，不能直接从`READY`跳到`GPS_APPROACH`输出控制；必须补齐`ARMING/TAKEOFF`门禁、SITL、拆桨台架验证和`enable_control`安全配置。
