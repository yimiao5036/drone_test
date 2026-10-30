# 任务状态机实现文档

## 功能职责

`MissionStateMachine`负责消费地面站目标、PX4飞行状态和健康状态快照，按任务阶段维护`MissionStatus`。默认仍是安全影子语义：验证`GroundStationTarget`已经能进入任务链路，并在满足导航快照基本条件时进入`GPS_APPROACH`影子状态。

阶段四新增（Task 7）：经`SetEstimationInputs`绑定目标估计/视觉稳定性/双目测距三个输入后，`GPS_APPROACH`（S5）开始发布`kPositionGuide`位置引导意图（指向估计目标 NED，`control_source=0`，`reason_code`带 bit31 控制权请求位`common::kControlIntentReasonRequestControl`）。**未绑定时不出意图、保持原影子行为**。意图是否真正转化为 PX4 输出由`FlightController`授权链与`enable_control`装配门禁决定，状态机侧不做最终裁决。

> **注（2026-10-16）**：S5 引导为"目标当前估计位置"的直接位置设定值，对持续运动目标**本质是尾追**（能否跟住取决于速度优势与提前量，不由刷新率决定）；时间前视、位置+速度前馈、S7 比例导引等改进路径见 `docs/S5运动目标追踪改进方案.md`（待排期）。

阶段四新增（Task 8）：经`SetEstimationDiagnostics`注入估计器诊断接口后，**交接判据**（`handover` 配置节）生效，状态在 S5→S6→S7 之间按下列判据推进/回退（`handover` 阈值全部来自配置，无代码字面量）：

| 迁移 | 判据 |
|---|---|
| S5→S6 | 视觉 `LOCKED` 连续帧 ≥ `lock_frames` + 双目距离有效且未超龄 + 视线角距视场边缘 ≥ `fow_margin_deg` + 视觉观测与卡尔曼预测创新 ≤ `innovation_gate_m`（四项同时满足） |
| S6→S7 | 进入判据持续满足 **且** 视觉链再推进一条新的 `LOCKED` 帧（交接期不断流） |
| S6→S5 | 进入判据任一失效（撤销交接）；或驻留超 `timeout_ms`（首条视觉指令未生效，防卡死） |
| S7→S5 | 四条独立路径任一成立：① 视觉状态消息断流超视觉新鲜度阈值（`visual_stale`）；② 测距失效（invalid 或超龄）连续达 `distance_lost_ms`；③ 视觉 `LOST` 连续帧 ≥ `exit_lost_frames`（进入/退出门限不同构成迟滞，防状态抖动）；④ 伺服未启动超时——S7 驻留超 `timeout_ms` 仍未见首条有效视觉意图（控制律持续降级，如姿态超龄），防冻结位置续发无限续期 |

**未注入估计器诊断时交接判据不评估**（保持 S5，启动记 ERROR）：无法核验"视觉锁定与卡尔曼航迹是否同一目标"就不允许切进视觉末段。

阶段四新增（Task 9）：**S7（VISUAL_TRACKING）视觉伺服意图迁入状态机**——内部伺服协作者复用`TrackingControlLaw`（配置为`MissionStateMachineConfig.tracking`，即`visual_tracking`节整节注入），逐拍组装控制快照（视觉稳定性状态 + 双目测距 + `FlightStateSnapshot`姿态）计算并发布`control_source=1`的速度航向意图（NED 速度，`reason_code`带 bit31 控制权请求位）。输出→意图映射约定：

| 控制律输出 | 映射意图 |
|---|---|
| `kVelocityHeading`（valid=true） | `kVelocityHeading`：NED 速度分量 + 航向通道（速率式填`yaw_rate_dps`/位置式填`yaw_deg`），reason 低位 100 正常跟踪 |
| `kBrakeHover`（视觉短期 LOST 判丢刹车，valid=true） | `kBrakeHover`零速安全意图（速度分量全零），reason 低位 102；**主链路无预测坐标**（`is_predicted`恒 false，101 Coast 低码不可达）——图像空间 coast 在本链路以判丢刹车形态落地 |
| `kDegradedHold`（valid=false：姿态/视觉超龄） | **不发布意图**（安全语义：控制律已明确"禁止正常控制"，补发零速与其矛盾）；超龄收口交 FlightController 意图超时门禁（`intent_timeout`）刹停 |

**S6 冻结最后位置目标（评审裁决 (a)，Task 9 落地）**：S5→S6 当拍冻结进入时刻的`TargetState`坐标；S6 驻留期逐拍续发该位置意图（`valid_until`由状态机每拍滚动续期），直到首条视觉速度意图被发出再停（S7 首拍若伺服降级停发，冻结续发继续顶住，不留意图缺口）。此策略对应用户设计 §2.1「进入 S6 后冻结最后位置目标」与 §2.5「位置式→速度式无扰斜坡过渡」——若交接期停发位置意图，控制器在`intent_timeout`(300ms)后 COAST 刹车，真实系统会在交接点先刹停再起步。**P3 决策（2026-10-19）：S6 冻结续发保持纯位置、显式不带速度前馈**（即使冻结快照带速度位），避免固定位置目标 + 持续前馈使飞机越过冻结点；S7 速度伺服接管后自然连续。

**热备份核对（设计 §2.4）**：S7 期间每秒核对卡尔曼状态年龄——`TargetState.header.receive_time_ms`年龄是否超出其自报有效期窗（`valid_for_ms`，与 S5 意图门禁`HasFreshEstimate`同一口径；无快照/`valid=false`亦视为断粮）。断粮 → 节流 WARN「S7 热备份断粮」+ `MissionStatus.active_warning_bits` bit2 置位，**不退出 S7**（S7 去留由四条收口路径决定，热备份告警只提示"假备份"风险）。

**与 VisualTrackingShadow 并存（Topic 隔离）**：控制律调用迁入状态机后，影子部件保留（现场观察用），**影子意图主题与状态机`IntentOutput()`是两个独立 Topic**；装配层只把状态机意图接入`FlightController`（见`DroneApplication::BindTopics`注释），不存在双意图生产者竞争。

阶段四新增（Task 10 / Task 5，2026-10-21）：**捕网执行器 `INetActuator`**——注入命令窄接口
`IPx4CommandSender`后，工厂按 `actuator.type` 创建实现（当前 `single_pwm`）。状态机**每拍**
调用 `Update(ActuatorFeedback)` 推进执行器内部时序（ACK 确认 → 触发保持 → 复位 → 复位稳定
→ Ready），并在 **S10（INTERCEPT_EXECUTING）** 调一次 `Trigger()`（执行器 `Fired()` 防重复；
入队失败保持 Ready、下拍重试，超过 `return_home.fire_timeout` 判故障）。**S10→S11 以
`Fired()`（触发命令已入队）为依据，不等发射 ACK**；复位不再由“离开拦截序列”触发——执行器
在触发 ACK ACCEPTED 后按 `actuator.reset_delay` 保持，再发复位行程，复位 ACK 后按
`actuator.reset_settle` 稳定回 Ready；离开拦截序列（S9 未触发）不发任何命令。状态机不感知
单发/多发、通道数或弹仓结构；后续多发只需实现同一接口并在工厂按 `type` 接入。

**故障 S16（Task 5）**：执行器在等待 ACK 期间遇到 187 超时 / 非 ACCEPTED（IN_PROGRESS 除外）
/ PX4 断连，或状态机在 S10 发现 `fire_timeout` 内触发命令未入队、飞行快照存在但
`connected=false`，即 `HandleMissionFault()`：置 `kWarningActuatorFault`（bit4）、调用
`IPx4CommandSender::CancelPendingCommands()` **恰好一次**、打一条 ERROR、迁移 S16 FAILSAFE。
此后 `Evaluate` 只维护告警位与状态发布，**停发位置/伺服/返航/降落一切控制意图**；不重试、
不补发复位（机构停在原位交人工处置）。故障为终态，`Start()` 才复位 `mission_faulted_`。
无飞行快照（启动早期/测试未注入）不算连接证据，**不触发**；但 S10 内持续无快照
超过 `return_home.fire_timeout` 仍按“触发命令未入队（发射超时）”判故障并进入 S16
（生产 S10 入口要求快照/连接证据，测试路径除外）。

**MAVLink/PX4 语义**：`MAV_CMD_DO_SET_ACTUATOR=187` 的 p1..p6 = Actuator Set 1..6 的
归一化值（-1..1，NaN=忽略），p7=执行器组序号（PX4 仅组 0）。**PWM 输出功能、映射区间
对应输出通道的 PWM 映射区间（当前 MAIN5 为 `PWM_MAIN_MIN5/MAX5`）、舵机行程与方向
全部在 PX4/QGC 侧配置**；机载只配置 `actuator.net_channel` 和归一化量：

```json
"actuator": {
  "type": "single_pwm",
  "net_channel": 1,
  "net_fire_norm": -0.7,
  "net_idle_norm": 0.0
}
```

**当前装机（2026-10-21 确认）**：捕网接 **MAIN5**，Output Function =
`Peripheral via Actuator Set 1`，PWM 已配置；`net_channel` 是 Actuator Set 编号，
**不是物理通道号 5**。

单发捕网在 PX4 标准 `1000/2000µs` 映射下对应 `1150µs 发射 / 1500µs 待机`。机载不再保存
PWM 微秒或输出区间，避免与 PX4 参数重复维护。接口与工厂见
`include/state_machine/net_actuator.h` / `.md`。

**拦截序列入口（2026-10-19 起配置化）**：S9/S10 入口判据已实现——`interception` 配置节（默认 `enabled=false`）提供连续锁定帧 `lock_frames`、测距窗口 `[min_range_m,max_range_m]`（上限 2026-10-23 冻结为双目测距极限 8m，下限仍占位）与 `auto_authorize`/`authorization_timeout_ms`；运行时还要求视觉 `LOCKED`、PX4 `connected+armed+Offboard(6/0)`、姿态/局部位置有效、命令窄接口已注入、执行器未触发且 `Ready()`（触发保持/复位/稳定途中不重入）、未进入故障。S7→S9→S10（自动授权）；S9 条件失效回 S7；授权超时/发射结束按 S12 返航；S9/S10 继续输出视觉伺服意图保持瞄准。默认关闭时整机仍不自动发射；测试钩子/独立工具可显式开启。

阶段四新增（Task 12，2026-10-13）：**S7 报告状态与实际控制输出背离的可观测位**——`MissionStatus.active_warning_bits` 新增 bit3「S7 伺服停发中」（`kWarningServoSuspended`）：处于 `VISUAL_TRACKING` 且**本拍未发出视觉伺服意图**时置位，覆盖两种背离形态：① 伺服尚未启动（S6/S7 过渡期仍在续发冻结位置意图，报告已是 S7 而输出仍是位置式）；② 控制律降级停发（姿态/视觉超龄 → 控制器意图超时门禁接手刹停，报告仍是 S7 而无输出）。伺服意图正常发出即清除；离开 S7 亦不置位。位语义与 bit1（目标过期）/bit2（热备份断粮）并列，映射表见 `docs/数据接口文档.md` §3.4 与 `docs/地面站目标位置与时间同步协议.md` §14.3（65013 `active_warning_bits`）。

**捕网执行器故障告警位 bit4（Task 5，2026-10-21）**：`MissionStatus.active_warning_bits`
bit4 =「捕网执行器 ACK 故障」（`kWarningActuatorFault`）。触发条件为执行器 187 ACK 超时/
非 ACCEPTED/PX4 断连，或 S10 触发命令入队超时；置位同时状态机进入 S16 FAILSAFE（见上段）。
故障为终态，bit4 持续置位（不再叠加其余告警位）直至重启，供地面站识别“任务已中止、交
PX4/遥控”。

**目标保持告警位 bit5（2026-10-22）**：`MissionStatus.active_warning_bits`
bit5 =「S5 目标失效保持中」（`kWarningTargetHold`）。目标不可用且正在发原地保持
位置意图（原因码 9）时置位；目标恢复（连续 3 拍）或离开 S5 后清除。与 bit1
（目标过期）可同时出现——bit1 表示“目标已过期”，bit5 表示“控制流已切换为原地
保持”。映射表见 `docs/数据接口文档.md` §3.4/§3.5。

**G3 全链验收（Task 12，2026-10-13，`tools/px4_link_smoke --sitl-stage4-mission`）**：状态机转正验收走**独立装配工具**（工具装配 `MissionStateMachine → FlightController → Px4Link`，扮演地面站/视觉监视器/双目测距三路生产者注入模拟数据），覆盖 S5→S6→S7→回退→停发全序列、交接缝意图无超龄（末条 S5 位置意图→首条 S7 速度意图间隔 < `intent_timeout` 且窗口内未跌入 COAST——`lock_frames` 门限 + 20Hz 注入使 S6 只驻留 1 决策拍，S6 冻结续发的逐拍滚动语义由本模块单测覆盖，工具侧只作诊断记录）、S7 伺服启动、短期/持续 LOST 语义、目标过期停发链与卡尔曼备份接管。**`DroneApplication` 生产装配仍未调用 `SetEstimationInputs`**（G3 裁决：生产接线与 `enable_control` 打开一并做，避免在未启控制时改变整机行为）——整机停在 S5 属预期，工具装配与生产装配共享同一状态机实现与配置注入路径。

本阶段仍不做以下事情：

- 不发送外部 setpoint（控制输出仍由 FlightController 产生）。`SetCommandSender` 当前消费点：S3 的 176/400（Task 5）、S10 捕网触发/复位、S13 AUTO.LAND、S14 前 DISARM；`FlightController::kBootstrap`（原点保持流）由 Task 6 接入，本模块只保证 kTakeoff 意图持续发布。
- S9/S10 入口判据与自动授权已落地（上段），但**默认 `interception.enabled=false`**；地面站确认授权协议尚未实现（`auto_authorize=false` 时只等超时返航），执行机构就绪仍以命令接口注入 + 未触发且 `Ready()`（无在途触发/复位时序）为代理，未接硬件状态反馈。
- 不把地面站目标透传为PX4航点。
- 不实现设计 §2.2 的"位置误差/地速/首条视觉指令经限幅生效"三项验证量（依赖
  控制侧反馈与尚未冻结的阈值配置），S6→S7 仍用"条件持续 + 视觉链推进 +
  驻留超时"作最小验证。**T12 处置（2026-10-13）**：G3 全链验收以"迁移序列与
  设计 §2 判据一致 + 交接处输出连续（速度设定值无越限跳变）"作为等价验收面，
  三项量化条件待控制律整定（设计 §7）后在 G3 实跑/回放中冻结，阈值不落代码。
- S7 无姿态超龄退出路径（边界声明）：姿态超龄由控制律降级停发 → FlightController
  授权链收口（姿态异常同时意味着 PX4 数据链异常，由控制器门禁裁决）。

**多机任务分配入口（Task 4，2026-10-22）**：新增`SetAssignmentInput()`（65014
`MissionAssignment`）与`AssignmentAckOutput()`（65015 `MissionAssignmentAck`）。
状态机只在 **S2 READY** 处理分配：字段与整机预检全部通过才记录
`MissionAssignmentTracker`、回 `accepted` 并迁移 **S3 ARMING**；任一失败回
`rejected` + 具体 reason 并保持 S2（预检拒绝不消耗任务机会）。
`MissionAssignmentTracker` 负责一次上电一次空中任务、重复
`assignment_id` 只重发原 65015、起飞原点记录。`PublishStatus` 的 65013 v2
字段（`assignment_id/target_id/availability/interceptor_type`）同步填写。

**65014 取消、S5 target_id 过滤与目标等待/丢失（Task 10，2026-10-22，设计
§5.1/§7.5）**：

- **取消（`flags.bit1`）**：在 `EvaluateAssignmentEntry` 的重复/普通预检
  **之前**处理；取消消息只校验协议版本与未知 flags 位，**不要求 bit0**。
  `assignment_id` 必须匹配当前活动分配：
  - 未离地（tracker 未消耗空中任务机会且飞行快照非 armed 空中）：S3/S4 复用
    `AbortTakeoff(kCancelledByGcs, "地面站取消")`（停流、必要时一次 DISARM、
    回 READY、65015 cancelled/aborted）；S2 已接受未进 S3 的防御路径
    `AbortOnGround()` 后保持 READY；
  - 已离地（tracker 已消耗任务机会或 S5~S13）：`AbortAirborne()` → **S12**，
    65015 cancelled/aborted（`cancelled_by_gcs`），不空中 disarm、不触发 187。
    已处于 S12/S13/S14 时只记录并回执，不把降落链重置回 S12；
  - 幂等：重复同一 `assignment_id` 的取消重发最近 cancelled 结果；500ms 内
    不重复执行取消动作（含 DISARM）；无活动分配且该 id 从未处理 →
    rejected(`not_ready`)；活动分配是其他 id → rejected(`already_assigned`)；
  - 被取消的分配不可恢复（重复 65014 重发 cancelled）；地面取消后回到 READY，
    可用**新的** `assignment_id` 重新分配（不自动重试）。
- **S5 target_id 过滤**：仅控制任务（`command_sender_ != nullptr` 且 tracker
  持有活动分配）下，`HasFreshGroundTarget`/`HasFreshEstimate` 对
  `target_id != assignment.target_id` 返回 false——非匹配 65010/估计不进入 S5、
  不更新位置引导、不参与交接；`GroundStationLink` 仍可照常做链路 ACK。
  影子模式（未注入命令接口）保持既有 S5 行为，不过滤。
- **S5 目标看门狗**：S5 入口（`TransitionTo(kGpsApproach)`）在控制任务下记录
  `target_wait_started_ms_`；每拍 `EvaluateTargetWatchdog()`：
  - 未获得匹配目标（估计或 65010 任一新鲜即视为获得）超过
    `takeoff.target_wait_timeout` → 65015 aborted(`target_wait_timeout`) → S12；
  - 已获得后匹配目标持续失效超过 `takeoff.target_lost_timeout` →
    65015 aborted(`target_lost`) → S12；恢复即清零丢失计时；
  - 中止当拍停发位置引导（S12 接管返航意图），不触发拦截/187；
    `mission_abort_latched_` 保证一次任务只中止一次（接受新分配/`Start()` 复位）；
  - 仅在 S5 生效，不改变 S6/S7 既有交接/视觉丢失逻辑；影子模式不启用。

**S5 目标失效原地保持（2026-10-22，替代“直接停发”）**：目标不可用（过期 /
`target_id` 不匹配 / 估计器无输出）时，控制任务在 S5 不再立即停发位置意图，
改为每决策拍发布 **`kPositionGuide` 原地保持意图**：

- 目标 = PX4 **当前局部位置** `local_x/y/z_m`（`frame_id=3` 局部 NED），
  **绝不使用过期目标坐标**；`reason_code = bit31 | 9`（新增低位原因码 9），
  不带速度前馈；
- 目的：保持 setpoint 流不断 → FlightController 维持 ACTIVE、PX4 维持
  Offboard，避免 `COM_OF_LOSS_T`（1s）触发 PX4 失效兜底（Position 回落；
  SITL 无 RC 时 `NAV_RCL_ACT=2` 会升级为 RTL），让状态机保留控制权直到
  看门狗作出中止决定；
- 恢复迟滞：连续 `kHoldReleaseFreshFrames=3` 个决策拍（100ms/拍）看到新鲜
  估计才退出保持、回到估计引导（原因码 7），防抖动来回切换；
- 能力门禁：仅控制任务（`command_sender_ != nullptr`）、仅 S5、且 PX4
  `connected && local_position_valid`；局部位置不可用时回落为停发（刹停
  语义仍归 FlightController 意图超时门禁），不猜坐标；
- 累计上限 `takeoff.target_hold_max_ms`（默认 60000）：目标反复闪烁会不断
  清零 `target_lost_timeout` 计时，本上限按本次 S5 **累计保持时长**收口，
  超限 → 65015 aborted（已获得目标按 `target_lost`，从未获得按
  `target_wait_timeout`）→ S12；`ResetTargetHold()` 在 S5 入口/中止复位；
- 可观测：保持期间 65013 `active_warning_bits` 置 **bit5**（目标保持中），
  进入/退出各打一条 INFO（`S5 目标失效，进入原地保持` /
  `S5 原地保持结束`），保持期内不刷屏；
- 单测覆盖：保持目标/无前馈/10Hz 续流、3 拍恢复迟滞、影子不保持、
  局部位置无效回落、累计上限两条回执路径、引导期间不因历史累计中止
  （`S5*Hold*` 用例）。

**S3 ARMING / S4 TAKEOFF 自动起飞（Task 5，2026-10-22）**：由
`EvaluateTakeoffSequence()` 每决策拍推进（放在分配入口之后、交接/目标/拦截
逻辑之前；S3/S4 期间后者按状态判定均不参与）。

- **S3 进入**：接受分配时 `MissionAssignmentTracker::BeginTakeoff()` 已记录
  起飞原点 `x0/y0/z0`（局部 NED，D 向上为负）。S3 每拍发布 **`type=kTakeoff`**
  控制意图（**不用 `kPositionGuide` 代替**）：目标 `x0/y0/z0`，`reason_code =
  bit31 | 7`（控制权请求 + 地面站任务），`valid_until_ms = now +
  intent_valid_for`；坐标系局部 NED（frame_id=3）。
- **176**：S3 驻留满 `takeoff.bootstrap_lead` 后发送**一次**
  `MAV_CMD_DO_SET_MODE(176)`：`param1=1`（CUSTOM_MODE_ENABLED）、`param2=6`
  （PX4 main mode Offboard）、`param3=0`；之后任何情况下都不重发。
- **实际模式**：等待 `flight_mode==6 && flight_sub_mode==0`（ACK 只作过程
  反馈）；超时 `takeoff.mode_timeout` → 中止。
- **400**：实际 6/0 后发送**一次** `MAV_CMD_COMPONENT_ARM_DISARM(400)`
  `param1=1.0`；等待实际 `armed==true`；超时 `takeoff.arm_timeout` → 中止。
- **ACK**：176/400 的最近 ACK 若 `last_command` 匹配且 `last_result` 非 0
  （ACCEPTED）且非 5（IN_PROGRESS）→ 立即中止（176 记 mode_timeout，400 记
  arm_timeout）。ACK 匹配计数基线在 S3 入口记录，上一次任务/命令的陈旧 ACK
  不误伤本次起飞；实际模式/armed 仍是最终判据。
- **入队失败**：`SendCommand()` 返回 false 同样中止、不重试（176 失败记
  bootstrap_timeout，400 失败记 arm_timeout）。
- **成功**：实际 `armed && Offboard(6/0)` → S4。实际 `armed && !landed` 时
  每拍调用 `MissionAssignmentTracker::MarkAirborne()`（幂等；消耗一次上电
  一次空中任务机会，但不推进 tracker 阶段）。
- **S4**：水平保持 `x0/y0`；z 目标 `z_target = z0 - takeoff.height_m`，
  每拍按 `climb_rate_mps * dt` 向目标推进（**不阶跃**）；意图继续带 bit31，
  `yaw_deg` 尽量取当前姿态 yaw。到达判据同时满足并保持 `stable_time`：
  `|z-z_target| ≤ z_tolerance_m`、水平误差 `≤ xy_tolerance_m`、
  `|vz| ≤ max_vz_mps`；满足后 `TakeoffComplete()` + `BeginEngage()` → **S5**
  （availability=engaged）。
- **S4 空中超时**（`height_timeout`）：PX4 仍在 Offboard → 转 **S13**，由现有
  `EvaluateLandingAndDisarm` 发 AUTO.LAND(4/6)；PX4 已离开 Offboard → 设置
  让权标志，转 **S13 仅监控**，不再发 AUTO.LAND、不再争控。
- **地面超时/条件失败**：停止发布 kTakeoff/位置意图；若 `landed` 且
  （`armed` **或本轮 400 ACK 已 ACCEPTED**）只发一次 DISARM（400 param1=0）——
  后者覆盖 armed 位尚未随心跳/遥测刷新的竞态（SITL 2026-10-22 实测地面取消
  会短暂留下 armed）；`AbortOnGround()`；回 S2 READY（状态不明时
  availability=unavailable）。
- **已离地后任何中止**（armed && !landed）：`AbortAirborne()` → **S12**；
  **不空中 disarm**（返航/降落链接管，最终由 S13/S14 解锁）。
- **65015 aborted**：`result=kCancelledOrAborted`，reason 为
  mode_timeout/arm_timeout/bootstrap_timeout/takeoff_timeout/
  px4_disconnected/mission_fault。回执使用接受分配时保存的
  `accepted_assignment_`（此时 `latest_assignment_` 早已清空）；同时
  `RememberResult` 更新为 aborted，重复 `assignment_id` 重发最新结果。
- **状态不确定**（PX4 断连/离地状态不明/无快照）：置 per-boot 不可调度标志
  `takeoff_unavailable_latched_`，`availability=unavailable`，新分配回
  `not_ready`；直到进程重启（`Start()` 复位），**不错误回到 ready**。
- **不重试**：176/400 各只发一次；中止后不原地循环重发，中止当拍起停止
  发布起飞意图（空中中止由 S12 返航链按设计接管）。

**影子兼容（关键）**：未注入命令接口（`enable_control=false` 装配）时，
65014 一律回 `control_disabled`，`availability=unavailable`（影子模式不可被调度）；
既有“新鲜 65010 目标 + 导航就绪 → S2→S5 影子验证”路径**原样保留**。注入命令
接口后，65010 目标不得单独把 S2 推到 S5——必须由 65014 分配经预检进入 S3。
此分叉只在 Evaluate 的 S2→S5 分支增加 `command_sender_ == nullptr` 条件，
不删除任何影子测试路径。

完整自动任务链仍以`docs/状态机设计.md`为准。S3/S4 的算力板侧时序已由本模块
落地（Task 5）；`FlightController::ControlAuthority::kBootstrap` 的原点保持
流已在控制器侧实现（Task 6），PX4 参数读取（`BAT_LOW_THR`/
`COM_LOW_BAT_ACT`）由 Px4Link 提供（Task 7），状态机侧电池门禁（S2 预检、
availability、飞行中 S12）由 Task 8 接入；65014 `flags.bit1` 取消、S5 的
`target_id` 过滤与 `target_wait_timeout`/`target_lost_timeout` 收口由 Task 10
接入（2026-10-22），原「已知缺口」已关闭。

## 接口与数据流

```cpp
struct MissionStateMachineConfig {
    std::chrono::milliseconds intent_valid_for{300};  // 意图有效期，与 controller.intent_timeout 同源
    config::HandoverConfig handover;                  // 交接迟滞参数（handover 配置节同源）
    control::TrackingConfig tracking;                 // 视觉跟踪控制律整节（visual_tracking 同源：
                                                      // camera 供视线角/视场换算，control/heading/
                                                      // vertical/distance 供 S7 伺服控制律）
    std::chrono::milliseconds visual_stale{200};      // 视觉状态超龄（visual_tracking.control.visual_stale_ms 同源）
    std::chrono::milliseconds distance_stale{500};    // 测距超龄（distance_stale_ms 同源）
    config::ActuatorConfig actuator;                  // 捕网执行器（actuator 配置节同源，Task 10）
    config::TakeoffConfig takeoff;                    // 自动起飞时序（takeoff 配置节同源，2026-10-22）
    config::FenceConfig fence;                        // 多边形围栏（fence 配置节同源，2026-10-29）
    uint8_t expected_interceptor_type = 25;           // 本机功能类别（ground_station.aircraft_component_id 同源）
};

class MissionStateMachine final : public IMissionStateMachine,
                                  public common::IFencePolygonSource {
public:
    explicit MissionStateMachine(MissionStateMachineConfig config = {});
    void SetInputs(Topic<GroundStationTarget>& ground_target,
                   Topic<FlightStateSnapshot>& flight,
                   Topic<HealthStatus>& health) override;
    void SetEstimationInputs(Topic<TargetState>& estimated,
                             Topic<VisualTargetStatus>& visual_status,
                             Topic<StereoTargetDistance>& distance) override;
    void SetCommandSender(communication::IPx4CommandSender* sender) override;
    void SetAssignmentInput(Topic<MissionAssignment>& assignment) override;   // 65014（可选绑定）
    void SetFenceInput(Topic<common::FenceZone>& zone) override;              // 65018（可选绑定）
    Topic<common::FenceStatus>& FenceStatusOutput() override;                 // 65019
    common::FencePolygonNed LatestFencePolygon() const override;              // 供飞行控制器钳制
    void SetEstimationDiagnostics(const perception::ITargetEstimator* estimator) override;
    Topic<ControlIntent>& IntentOutput() override;
    Topic<MissionStatus>& StatusOutput() override;                            // 65013 v2
    Topic<MissionAssignmentAck>& AssignmentAckOutput() override;              // 65015

    // ---- 测试入口（生产装配不得调用） ----
    void ForceStateForTest(common::MissionState state);  // 强制迁移（需 test_hooks_enabled=true）

    /// 预置一份「已接受的任务分配」（需 test_hooks_enabled=true）。
    /// 用途：SITL 里起飞由工具直接驱动 PX4、不走 S2 的 65014 入口，而中止回执
    /// 65015 的前置是 accepted_assignment_ 有值；没有它就无法覆盖「已分配 + 65020
    /// → 65015 cancelled_by_gcs」组合。
    void SeedAcceptedAssignmentForTest(const common::MissionAssignment& assignment);
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
GroundStationLink::MissionAssignmentOutput() ─► SetAssignmentInput(...)（65014，可选绑定）
MissionStateMachine::IntentOutput() ─► FlightController::SetInput(...)
MissionStateMachine::StatusOutput() ─► GroundStationLink::SetMissionStatusInput(...)（65013 v2）
MissionStateMachine::AssignmentAckOutput() ─► GroundStationLink::SetMissionAssignmentAckInput(...)（65015）
Px4Link（IPx4CommandSender）─► SetCommandSender(...)（仅 enable_control=true 装配分支；
                             当前消费点=捕网执行器触发/复位，Task 10）
```

配置注入：`intent_valid_for`由`DroneApplication`从`config_.controller.intent_timeout`
同源传入，保证意图`valid_until_ms`与控制器意图超龄门禁一致；`handover`、
`tracking`（`visual_tracking`整节）、`actuator`（`actuator`整节，Task 10）
与两个超龄阈值分别从`config_.handover`、
`config_.visual_tracking`、`config_.visual_tracking.control.{visual_stale_ms,
distance_stale_ms}`同源传入。控制律内部超龄阈值取`tracking.control.*`
（生产与状态机侧两阈值同源注入，测试可分别收紧）。
`handover` 的关系校验（`exit_lost_frames > lock_frames` 等）由配置加载期
`HandoverConfig::Validate`强制，状态机构造不重复校验（与 FlightController 同约定）。

**接线现状（T12 裁决后）**：`DroneApplication`**仍未**调用`SetEstimationInputs`
（整机因此停在 S5 影子行为，不影响既有链路）——T12 G3 全链验收改走独立装配工具
（`tools/px4_link_smoke --sitl-stage4-mission`，三输入一组绑定 + 真实估计器诊断），
生产接线与 `enable_control` 打开一并做（避免控制未启时改变整机行为）；
`SetEstimationDiagnostics`已在装配估计器时注入；
`SetCommandSender`已在`enable_control=true`装配门禁分支注入（T6），
消费点=捕网执行器触发/复位（T10，enable_control=false 时零调用）。

## 关键实现点

- **多边形围栏（2026-10-29，P1/P2）**：地面站用 65018 下发多边形区域（zone_id/version
  唯一标识，顶点为 WGS-84），机载自治执行。
  - 输入：`SetFenceInput()` 绑定 `kFenceZone`（Start 前）；`Start()` 时若
    `fence.enabled` 且 `zone_cache_path` 存在则先 `ZoneStore::Load()` 装载缓存，
    作为首拍区域。在线帧与缓存都经 `ApplyFenceZone()` 做同一套合法性检查
    （zone_id/version 非 0、顶点 3~256、经纬度数值合法、边长 ≥0.5m、面积 ≥1m²、
    多边形不自交、`altitude_reference∈{0,1}`、高度带不倒挂
    `alt_max>0 && alt_min>alt_max`），非法帧节流 WARN 后丢弃并保留上一份可用区域；
    几何/拓扑校验统一由 `control::fence::ValidateZone()` 实现（2026-10-30 审查修复：
    旧实现只查 zone_id/version/顶点数，共线、自交、重复点、面积 0 的区域会被接受并
    参与判内/钳制，地面站一次误配置就让围栏永久失效）。区域是否"变化"按**内容**
    比较（id/version + 类型/动作/高度基准与上下限 + 顶点），同 id/version 改几何也会
    落盘 `zone.json` 并回读（旧实现只比 id/version：地面站忘递增版本号时只在内存生效，
    飞机重启后回滚到缓存里的旧区域）。
  - 坐标换算：`RebuildFencePolygon()` 用当前 `FlightStateSnapshot` 的 EKF 锚点
    （`ned_ref_lat_1e7/ned_ref_lon_1e7`）把每个顶点经 `perception::GeodeticToLocalNed()`
    换成局部 NED；锚点变化后下一拍自动重建（区域只在 `fence_dirty_` 时重建）。
    **返回值语义（2026-10-29 修）**：函数返回 `bool`，只有重建成功才清 `fence_dirty_`
    （`fence_dirty_ = !RebuildFencePolygon()`）。起飞早期 EKF 锚点可能尚未就绪，
    若此时无条件清标记，多边形会一直为空、控制器永不钳制（SITL 实测症状：
    `ClampedCount=0`、位置设定值跑到 149.99m）。
  - 默认区域与回读（2026-10-29 增）：`EnsureDefaultFenceZone()` 在 `enabled` 为真、
    无区域、`home_valid` 且 `default_half_side_m>0` 时以 Home 为中心生成方形默认区域
    （`default_zone_id/version/action/alt_*`），落盘 `zone.json` 并回读；
    `PublishFenceZoneReport()` 在区域变化或每 `report_period_ms`（默认 5s）发 65022
    给地面站对拍。`fence.enabled` 默认已改为 `true`。
  - 判定：`EvaluateFenceGuard()` 排在电池守卫之后、目标看门狗之前（几何安全事件：
    优先级低于电池、高于任务）。用 `PointInPolygon()` 与 `SignedDistanceToBoundary()`
    得出 `kNormal / kNearBoundary（距离 ≤ boundary_warn_m）/ kViolated（出区或超高度带）`，
    越界持续到 `violation_debounce_ms` 才动作，避免单帧抖动触发中止链。
  - 高度带语义（2026-10-30 现场日志 0078 修正）：`altitude_reference=1`（相对 Home）
    一律**不判下限**（有效下限恒 0）——高度从地面起算，起飞必然穿过 0→设定值，而
    S3/S4 在围栏动作生效态、防抖只有 `violation_debounce_ms`（1.5s），旧实现
    （`alt_min=5m`）会让地面静止就被判 `kViolated`、解锁后立刻中止返航；地面站表单
    对该基准把最低高度固定 0 并置灰。`altitude_reference=0`（AMSL）保留绝对海拔下限
    （跨海拔场地仍有意义），但 S0~S4（未起飞/爬升）同样不判下限，S5 起才按绝对海拔
    生效。实现为 `FenceEffectiveAltMin(zone, CurrentState())`，`alt_max≤0` 仍表示不限。
  - 动作：`follow_zone_action=true` 时按区域声明的 action，否则按 `violation_action`。
    `kHold` → 每拍把位置意图钉在当前局部位置（reason=10 `kReasonFenceViolation`），
    任务状态不迁移；`kReturn` → `AbortForFence()`（停发位置意图、`assignment_tracker_.AbortAirborne()`、
    65015 `aborted reason=kFenceViolation(22)`、转 S12 返航；
    2026-10-30 起不再误用 `kTargetLost(14)`（地面站可区分几何事件与目标丢失）；`kLand` → 同一 `AbortForFence()` 入口、转 S13 降落
    （2026-10-30 审查修复：旧实现只 `TransitionTo(kLanding)`，不发 65015、不
    `AbortAirborne`，S12/S13 入口语义与返航不一致）。中止锁 `fence_abort_latched_`
    与 `mission_abort_latched_` 同语义：per-boot 复位、新分配被接受时复位，
    同一次越界不重复触发中止链/重复回执。
  - 只在**飞行相关状态**动作（`kArming`~`kPostIntercept`）：地面 S0~S2、S12/S13/S14
    与故障态只上报 65019，不触发中止链，避免地面就锁死任务。
- **人工任务指令（2026-10-29，65020/65021）**：地面站可下发"主动返航"打断当前任务。
  - 输入：`SetMissionCommandInput()` 绑定 `kMissionCommand`（Start 前）；`DrainInputs()`
    每拍只保留最新一条（`latest_mission_command_`），`valid_for_ms=1000` 由链路层写入，
    过期指令由状态机侧按时钟判定丢弃。
  - 判定顺序（`EvaluateMissionCommand()`，排在电池守卫之后、围栏守卫之前——人工指令
    优先于几何事件）：重复 `command_id` → `kDuplicateCommand`（不动作、不回退状态）；
    幂等槽位 `last_mission_command_id_` 只在**真正执行动作**的分支登记
    （`RecordExecutedMissionCommandId()`：`kArming/kTakeoff` 中止、飞行态中止返航），
    被版本/类型/状态守卫拒绝或「已在返航」早退的指令不占槽位——地面站修正后可用同一
    `command_id` 重发（2026-10-30 修复：旧实现在校验前登记，被拒 id 再发被判重复）；
    版本不符 → `kUnsupportedVersion`；非 `kReturnHome` → `kUnsupportedCommand`；
    `mission_faulted_` → `kMissionFault`；已在 `kReturnHome/kLanding` → `kCompleted`
    + `kAlreadyReturning`（幂等，不重复发 65015）；`kArming/kTakeoff` →
    `AbortTakeoff(kCancelledByGcs)`；其余飞行态 → 复用内部中止入口。
  - 接受路径：`mission_abort_latched_=true`、`StopPositionIntent()`、`ResetTargetHold()`、
    `assignment_tracker_.AbortAirborne()`、`RememberTakeoffAbort(kCancelledByGcs)`、
    `TransitionTo(kReturnHome,"地面站主动返航指令，中止任务并返航")`，并发 65015
    `aborted reason=kCancelledByGcs(12)`。**不是 PX4 RTL**：返航轨迹仍由 S12 位置意图控制。
  - 回执：`PublishMissionCommandAck()` 走 `kMissionCommandAck`，字段含 result/reason/
    `mission_state`（回执时刻状态），地面站无需猜测机载处于哪一态；`header.sequence`
    每条回执只自增一次（2026-10-30 审查修复：旧实现同一条回执自增两次、序号跳 2，
    地面站按序号做去重/丢包统计会误判）。
  - **65015 的既有门槛（2026-10-28 复核）**：`PublishTakeoffAbortAck()` 首行是
    `if (!accepted_assignment_.has_value()) return;`，因此只有在先接受过 65014 分配的
    任务里，中止才会同步 65015；影子/未分配场景（含 SITL `--sitl-stage4-command`）只看
    65021 与状态迁移。判读现场日志时不要把它当回归。
- **S12 返航机头朝向**：`return_home.heading_mode=1`（默认）时，S12 非爬升段按
  期望航向（指向 Home，`atan2(east, north)`）下发 `yaw_valid=true` 的位置意图，
  经角速度/角加速度二阶限幅（`heading_rate_limit_dps=30`、`heading_accel_limit_dps2=60`）；
  首拍直接对齐当前方位角，水平距离 <1m 时保持上一拍航向，避免在 Home 上方原地打转。
  `heading_mode=0` 恢复旧行为（不指定航向，PX4 保持原机头平移）。
- 状态机拥有独立工作线程，周期为100ms。
- 输入通过`Topic`订阅队列获取：目标队列容量4，飞行/健康/估计/视觉/测距快照队列容量2。
- 每个周期先抽干订阅队列，只保留最新不可变快照，再做状态判断，避免一次决策周期内读到不一致数据。
- 启动后先发布`BOOT`，随后进入`SELF_CHECK`。
- `SELF_CHECK`阶段等待PX4快照满足：连接在线、GPS fix有效、全局位置有效、Home有效。
  PX4 已到位（2026-10-10：装机体+GPS）：正常应能越过自检；仍停留则按
  心跳 → GPS fix → Home 顺序排查（见`docs/故障排查手册.md`），不要改条件绕过。
  **室内提示**：室内可能无 GPS fix/Home，停留自检属预期、不是通信问题
  （判据是「PX4 心跳已连接」是否出现）；分阶段验证见`docs/PX4实链验证清单.md`。
- `READY`阶段等待有效地面站目标；**未注入命令接口时**，目标有效且导航仍有效
  即进入`GPS_APPROACH`影子状态（既有语义，保留）。
- **S2 分配预检（Task 4，2026-10-22）**：只在`CurrentState()==kReady`处理
  最新一条 65014，且每条消息只处理一次（处理即清空`latest_assignment_`，
  不按 100ms 决策周期重复回 65015）。检查顺序：
  0. `flags.bit1` 取消：**先于**重复/普通预检处理（取消不要求 bit0；活动分配
     在 tracker 中已记 accepted，若先查重复会把取消误当重复分配）；
  1. 重复`assignment_id`：`MissionAssignmentTracker::WasHandled` 命中 → 只重发
     原 65015（result/reason 不变），不再预检、不迁移；
  2. `protocol_version == 1`，否则 `unsupported_version`；
  3. `flags` 只允许已知位（bit0 授权、bit1 取消），且必须带 bit0，否则
     `invalid_message`；
  4. `target_id != 0`，否则 `invalid_message`；
  5. `interceptor_type == expected_interceptor_type`（默认 25 捕网），否则
     `unsupported_interceptor_type`；
  6. `valid_for_ms > 0` 且 `now <= received_ms + valid_for_ms`，否则
     `invalid_message`（按本机单调时钟；0 视为非法）；
  7. `command_sender_ != nullptr`，否则 `control_disabled`（影子装配）；
  8. `estimation_inputs_active_`（估计/视觉/测距三输入已绑定），否则
     `preflight_failed`；
  9. `latest_flight_`：连接在线、`landed_state_valid && landed`、`!armed`、
     局部位置/姿态/Home 有效；断连为 `px4_disconnected`，其余为
     `preflight_failed`；
  10. 电池参数（Task 8，PX4 `BAT_LOW_THR`/`COM_LOW_BAT_ACT`）：
     `battery_low_threshold_valid && low_battery_action_valid`、
     `0 < battery_low_threshold_pct < 100`、`low_battery_action == 0`
     （Warning Only），否则 `battery_config_unknown`；
  11. 电池遥测：`battery_valid && battery_remaining_valid` 且百分比有限，
     否则 `battery_telemetry_invalid`；
  12. `battery_remaining_pct > BAT_LOW_THR`，否则 `battery_low`（机载不设
     本地百分比阈值，不估算剩余飞行时间）；
  13. 无任务故障/执行器故障，否则 `mission_fault`；
  14. tracker 可接受（`kIdle` 且未消耗空中任务机会），否则 `already_assigned`。
  失败：回 `rejected` + reason、保持 S2、`RememberResult` 记录回执（重复 id
  可重发），不自动重试、不消耗任务机会；状态不确定（未连接/故障）时
  `availability=unavailable`。
  成功：tracker `Accept` → 记 accepted 回执 → `BeginTakeoff(local_x,local_y,
  local_z)` 记录原点 → 回 accepted（`availability=assigned`）→ 迁移 S3
  （reason「接受任务分配」）。S3 起每拍发布 `kTakeoff` 原点意图，176/400
  按 S3 时序各发一次（见上「S3 ARMING / S4 TAKEOFF」）。
- **availability 映射**：未注入命令接口、任务故障/执行器故障、未连接、已消耗
  空中任务机会、起飞中止后状态不确定（`takeoff_unavailable_latched_`）或
  **电池门禁不满足**（参数缺失/遥测失效/不高于 `BAT_LOW_THR`）→
  `unavailable`；有控制路径时用 tracker 阶段：`kIdle→ready`、
  `kAssigned/kTakingOff→assigned`、`kAirborne/kEngaged→engaged`、
  `kReturning→returning`、`kTerminal→terminal`。65015 与 65013 共用该映射。
  地面低电/遥测失效**不锁存**：电量恢复且参数有效、未执行空中任务时自然回
  `ready`；只有空中中止才按“一次上电一次任务”进入 terminal。
- **飞行中电池守卫（Task 8，2026-10-22）**：`EvaluateBatteryGuard()` 在
  `Evaluate` 中位于 `EvaluateTakeoffSequence` 之后、交接/拦截逻辑之前，
  因此低电中止当拍即进入 S12，后续 S9/S10 入口与捕网触发没有机会执行。
  - 生效阶段：S3/S4 与 S5~S10；S12/S13/S14/S16 不再触发；影子装配
    （`command_sender_ == nullptr`）不生效（S5 影子验证语义保留）。
  - **遥测失效**（`battery_valid=false`、`battery_remaining_valid=false`、
    百分比非有限或无飞行快照）→ 立即中止，reason
    `battery_telemetry_invalid`，不等防抖。
  - **低电**：`pct <= BAT_LOW_THR` 连续 `takeoff.battery_low_debounce` 后
    中止，reason `battery_low`；恢复高于阈值即清零计时。
  - S3/S4 复用 `AbortTakeoff()`：地面 → 停流/必要时一次 DISARM → READY；
    已离地 → `AbortAirborne()` → S12。
  - S5~S10：`AbortAirborne()` + 65015 aborted → **S12 机载位置返航**；
    不使用 PX4 自动 RTL、不空中 disarm、不触发新的拦截/发射。
  - `battery_abort_latched_` 保证一次任务只中止/回 65015 一次；接受新分配
    或 `Start()` 时复位。
  - 空中中止后落地沿用现有 S14 路径 → tracker terminal，本次上电不再分配；
    不新增自动恢复。
  - PX4 自身低电动作导致离开 Offboard 时，现有 S12/S13 逻辑停发让权、
    只监控，不争控。
- **S3/S4 自动起飞（Task 5）**（`EvaluateTakeoffSequence` /
  `EvaluateArmingSequence` / `EvaluateClimbSequence` / `AbortTakeoff`）：
  - 调用位置：`Evaluate` 中在 `EvaluateAssignmentEntry` 之后、交接/目标/
    拦截逻辑之前；S3/S4 状态不满足 S5/S6/S7/S9/S10 的分支条件，因此不会
    进入交接/拦截。
  - S3 阶段机：`kOriginHold`（原点保持意图，等 bootstrap_lead）→
    `kWaitOffboard`（176 已发，等实际 6/0）→ `kWaitArmed`（400 已发，等实际
    armed）。176/400 由阶段迁移保证各一次；`SendCommand` 入队失败立即中止。
  - 起飞意图每拍直接发布（10Hz，不走内容节流/重发上限）：控制器需要持续的
    原点保持/爬升流；`type=kTakeoff`、`frame_id=3`、`reason=bit31|7`。
  - S4 爬升：`takeoff_z_cmd_` 从原点 z 逐拍按 `climb_rate*dt` 推进到
    `z0-height_m`，到达判据看**实际** z/xy/vz（不是指令值），保持
    `stable_time` 后 `TakeoffComplete()+BeginEngage()` → S5。
  - 中止统一入口 `AbortTakeoff()`：已离地 → S12 不 disarm；地面且
    `armed+landed` → 一次 DISARM(400 param1=0) → S2；不确定 → 置
    `takeoff_unavailable_latched_` → S2 且 unavailable。S4 空中超时单独走
    S13（仍 Offboard 发 AUTO.LAND；已让权则仅监控）。
  - 任务故障（S16 路径）若发生在 S3/S4：`AbortTakeoffForMissionFault()` 先
    按实际离地状态收口 tracker 并回 65015 aborted(mission_fault)，再进入
    S16。
- **控制路径下 65010 不再单独进入 S5**：S2→S5 分支增加
  `command_sender_ == nullptr` 条件；注入命令接口后必须由 65014 分配进入 S3
  （Task 5 再推进 S4/S5）。
- **65014 取消（Task 10）**：`HandleAssignmentCancel()` 位于
  `EvaluateAssignmentEntry()` 最前。取消匹配活动分配后按 `AssignmentAirborne()`
  （tracker `ConsumedAirborneMission()` 或飞行快照 `armed && !landed`）分叉：
  地面 S3/S4 → `AbortTakeoff(cancelled_by_gcs)`；空中 → `AbortAirborne()` +
  `TransitionTo(kReturnHome)` + 65015 aborted。S12/S13/S14 收到取消只
  `RememberResult`/回执，不重置降落链。500ms 内不重复执行取消动作；
  重复 id 重发最近结果。取消不触发拦截，`AbortAirborne()` 后消费机会已存在，
  后续 `Terminal()` 由 S14 完成。
- **S5 target_id 过滤（Task 10）**：控制任务下 `TargetMatchesAssignment()`
  使用 `MissionAssignmentTracker::AllowsTarget()`（`target_id != 0` 且等于
  `assignment.target_id`），在 `HasFreshGroundTarget`/`HasFreshEstimate` 内
  统一生效，因此 S5 引导、交接判据、热备份与看门狗看到的是同一口径；
  非匹配 65010 不影响状态机（GroundStationLink 的链路 ACK 不受影响）。
- **S5 目标等待/丢失看门狗（Task 10）**：`Evaluate()` 中位于
  `EvaluateBatteryGuard()` 之后、`EvaluateHandover()` 之前；只在
  `command_sender_ != nullptr` 且状态为 S5 时评估。`HasFreshAssignedTarget()`
  将“匹配分配的估计或 65010 任一新鲜”视为已获得目标；等待/丢失超时统一走
  `AbortForTargetWatchdog()`（`StopPositionIntent` + `AbortAirborne` +
  65015 aborted + S12），`mission_abort_latched_` 防止重复中止。S6/S7 的
  交接与视觉丢失逻辑不变。
- `GPS_APPROACH`阶段维护目标新鲜度和状态回传；估计输入已绑定且估计新鲜有效时
  （`TargetState.valid=true` 且 `receive_time_ms + valid_for_ms` 未过期），
  发布`kPositionGuide`意图：`target_x/y/z = pos_x/y/z_m`（`pos_z_m`为 NaN 时原样
  透传——高度无效的合法语义，由 FlightController 有限性门禁拒绝，状态机侧不得改 0），
  `reason_code = kControlIntentReasonRequestControl | 7（地面站任务下发）`，
  `valid_for_ms/valid_until_ms`来自注入的意图有效期配置。
- **S5 速度前馈（P3，2026-10-19）**：位置意图同时携带 `feedforward_vx/vy/vz_mps`
  与 `feedforward_valid_mask`。来源为 `TargetState.velocity_valid_mask`（地面站
  速度位优先、KF 自算速度 ≥5 次位置更新兜底）；状态机逐轴归一化——只有位为 1
  且值有限的轴保留，其余轴值清零/位清零（无效 KF 值不污染意图）。状态机不做
  幅值钳制，全部交 FlightController（开关/上限/降级）。S5 启动日志追加
  `ff_mask/ff=(...)` 摘要，仅来源切换时打印一次。
- 意图节流纪律：内容（目标位置/有效性/**前馈速度与逐轴有效位**）无变化按 10Hz
  上限重发（意图是周期语义，FlightController 50ms 周期消费）；内容变化立即发。
  NaN 与 NaN 视为内容相等；未置位轴不参与比较。
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
  - **S7 退出收口四条路径**（缺一都会让 S7 在异常下永不退出或意图挂死）：
    ① 视觉状态断流——监视器线程死亡/链路中断时无 `LOST` 消息可计数、帧序冻结，
    按「最近一条 `VisualTargetStatus` 的接收年龄 > `visual_stale`」直接退出
    （与 S6 进入判据同一新鲜度阈值、同一`StalenessChecker`判定，见`VisualStatusStale`）；
    ② 测距失效计时——`valid=false` 或超龄即累计，连续达 `distance_lost_ms` 退出，
    中途恢复有效立刻清零（`distance_lost_since_ms_`），覆盖"测距丢失但视觉仍
    锁定、控制律 `no_distance_action` 不触发退出"的场景；
    ③ 视觉 `LOST` 连续帧达 `exit_lost_frames`；
    ④ **伺服未启动超时**——S7 驻留超 `timeout_ms`（自进入 S7 起计）仍未见首条
    有效视觉意图：控制律持续降级（可持续情形为姿态超龄）时伺服永不启动，
    冻结位置续发不得无限续期（S6 有 `timeout_ms` 封顶，S7 需等价封顶），
    收口回 S5 重新走交接判据（reason 中文区分"伺服未启动超时"）。
    一旦首条视觉意图（速度或刹车）已发出，本条不再评估（伺服启动后的降级
    停发由其余三条路径与控制器门禁收口）。
    S7 期间不评估进入条件；健康监视器按 `publish_interval`（100ms）发布，
    200ms 超龄阈值留 2 倍余量，短时丢帧不触发 ①，图像空间 coast 由 Task 9
    的视觉伺服承担（阈值以内的短时缺口按判丢刹车形态落地，见下）。
- **视觉伺服（Task 9）**（`EvaluatePositionIntent` / `EvaluateVisualServoIntent` /
  `BuildVisualIntent` / `EvaluateHotBackup`）：
  - 调用节拍：跟随状态机决策周期（100ms/10Hz）；控制律按实际时钟差推算`dt`
    （`tracking.control.frequency_hz`仅作首拍兜底），限幅在真实时间上推进；
    热路径零日志（意图切换/降级才打）。控制律单线程约定由决策线程独占满足。
  - 快照组装：视觉`LOCKED`→`tracked=true`（其余状态 false）；`is_predicted`恒
    false（主链路无预测坐标）；距离仅在`valid=true`且接收时间有效时透传
    （超龄判定在控制律内，阈值取`tracking.control.distance_stale_ms`）；
    姿态取`FlightStateSnapshot`真实值（**正式路径不做虚拟姿态兜底**——影子
    部件的`virtual_attitude_when_absent`属观察开关，不进入正式装配）。
  - 意图坐标系：速度分量已是**局部 NED**（机体系→NED 旋转在`TrackingControlLaw`
    内完成），意图`frame_id=3`标注；FlightController 不做二次旋转
    （边界见`include/control/flight_controller.md`）。
  - 降级停发语义：`output.valid=false`（姿态/视觉超龄）→ 不发布意图 + 节流
    WARN（reason 区分姿态超龄/视觉超龄/持续无距离退出）。**不补发零速**：
    控制律已明确"禁止正常控制"，补发会与其矛盾；刹停收口交 FlightController
    意图超时门禁（`intent_timeout` 后 COAST 刹车）。
  - **S7 伺服停发告警位（Task 12）**：`servo_intent_active_` 由
    `EvaluateVisualServoIntent` 每拍置位（仅实际发布视觉意图时为 true；离开
    S7 不更新，交接段收口清零）；`Evaluate` 末段按"当拍状态==S7 且未发出意图"
    置 `kWarningServoSuspended`（bit3）。该位只做**观测标注**，不改变任何迁移
    或输出判定（S7 去留仍由四条收口路径决定）。
  - **热备份核对**：S7 内每秒（进入后立即核对一次）检查`HasFreshEstimate`
    ——即`TargetState`年龄是否超出其自报有效期窗`valid_for_ms`（生产者声明的
    "数据允许使用的最长时间"）；断粮 WARN 节流（第 1 次与每满 100 次）+
    `MissionStatus`告警位 bit2，**不退出 S7**。告警位随离开 S7 清除。
  - 冻结位置续发：S5→S6 当拍`FreezeHandoverTarget()`（取进入时刻`TargetState`），
    S6 逐拍`PublishPositionIntent`滚动续期；S7 首拍伺服未发出意图前继续顶住
    （**以 `timeout_ms` 封顶**：自进入 S7 起计，超时未见首条有效视觉意图 →
    退出路径 ④ 收口回 S5，不得无限续期）；发出后停发（`StopPositionIntent`）。
    交接段收口（含冻结清空、控制律`Reset`、热备份状态清零）统一在
    `ResetHandoverHysteresis`内完成。
- **捕网执行器（Task 10 / Task 5，2026-10-21）**（`UpdateNetActuator` /
  `MaybeTriggerNetActuator` / `HandleMissionFault` / `INetActuator`）：
  - 每拍顺序：`Evaluate` 开头先 `UpdateNetActuator()`（组装 `ActuatorFeedback`：
    PX4 连接证据取最新飞行快照、ACK 快照取 `IPx4CommandSender::AckStatus()`），
    故障则整拍只维护告警/发布状态并返回；随后 `EvaluateInterceptSequence` →
    `MaybeTriggerNetActuator` → 其余控制意图评估。
  - 触发语义：S10 内且执行器未 `Fired()` 时，至少满足 `Ready()` 与
    `RemainingShots()>0` 才调 `Trigger()`；**无飞行快照不触发**（没有连接证据），
    快照存在但 `connected=false` → 故障；入队失败保持 Ready、下拍重试，
    超过 `return_home.fire_timeout` → 故障。S9 只标记序列开始，不触发。
  - 完成语义：`EvaluateInterceptionCompletion` 以 `Fired()`（触发命令已入队）
    判 S10→S11，**不等发射 ACK**；V1 不判命中。
  - 复位语义：**由执行器自身计时**——触发 ACK ACCEPTED 后保持
    `reset_delay`，发复位行程；复位 ACK 后稳定 `reset_settle` 回到 Ready。
    状态机不复制计时逻辑。未触发即离开 S9/S10 不发任何命令。
  - 故障（S16）：187 ACK 超时/非 ACCEPTED/PX4 断连/发射入队超时 →
    `HandleMissionFault()` 置 bit4、`CancelPendingCommands()` 恰好一次、ERROR、
    迁移 S16；不重试、不补发复位。之后 `Evaluate` 早退，停发一切控制意图。
  - 命令参数（single_pwm）：只有本 Actuator Set 参数位填归一化值，**其余位
    NaN**（规范"忽略"；0=中位行程会误动其他 Set），p7=0。机载只发 -0.7/0.0，
    PWM 映射在 PX4 侧。
  - 门禁：`net_actuator_==nullptr`（enable_control=false 装配语义）→ 零调用，
    本功能物理缺席；触发/复位同为控制输出，不另开旁路。
  - 进程退出不补发复位行程（`Stop()` 不触发机构动作）：机构实际位置由
    G4 台架/人工确认；拦截执行段内停机属异常场景，PX4 侧输出保持最后值。
- **S9/S10 入口判据（2026-10-19）**：`EvaluateInterceptSequence` 在 S7 每拍评估
  `InterceptEntrySatisfied`——`interception.enabled`、视觉新鲜且 `LOCKED`、
  `consecutive_lock_frames_ >= lock_frames`、测距 `valid`+新鲜且
  `[min_range_m,max_range_m]`（**上限 2026-10-23 冻结 = 双目测距极限 8m**，
  下限 2.0 仍占位）、`connected+armed+Offboard(6/0)`、姿态/局部位置有效、
  `net_actuator_` 非空且未触发、`Ready()`、未故障、任务未进入 S16；可选
  `min_battery_percent>0` 时还要求电池遥测有效且剩余电量达标（**当前生产保持 0，
  实机后冻结**）。满足→S9；
  S9 条件失效→S7；`auto_authorize=true`→S10；false 时等
  `authorization_timeout_ms` 超时→按任务中止收口转 S12（2026-10-30 审查修复：
  停发视觉伺服意图、`ResetTargetHold()`、`AbortAirborne()`、`RememberTakeoffAbort()`、
  65015 `aborted reason=target_wait_timeout`——旧实现只 `TransitionTo(kReturnHome)`，
  S12 起没有任何意图发布，控制器在 `intent_timeout` 后刹车保持、到点不返航）。
  S9/S10 仍走 `EvaluateVisualServoIntent`
  保持伺服/瞄准，不因状态切换产生意图空窗。默认关闭（生产安全）。
- **拦截后返航（S11~S14，2026-10-19 V1 单发口径）**：
  - **完成判据**：S10 内执行器 `Fired()`（触发命令成功入队，不等 ACK）→ S11；
    **V1 不判断命中**，随后 S11 当拍/下一拍进入 S12。发射入队超时/断连不再走
    “失败返航”，而是 `HandleMissionFault()` 进入 S16。摄像头状态不参与返航判断。
  - **Home 局部 NED**：`HomeLocalNed()` 用 Home 经纬度（`home_*`）+ EKF 锚点
    （`ned_ref_*`）经 `perception::GeodeticToLocalNed` 换算；不依赖
    `HOME_POSITION.x/y/z`（MAVLink 无 `valid_lpos` 标志）。`home_valid`/
    `ned_ref_valid`/`local_position_valid` 任一无效 → 原地悬停并告警（不盲目返航）。
  - **S12 返航**：相对 Home 高度 <`altitude_m`（5m）先原地爬升，达标后再水平
    飞 Home；≥5m 保持当前高度。位置意图 10Hz 重发，`reason_code` 低位=8
    （`kReasonReturnHome`），不带速度前馈。PX4 离开 Offboard（failsafe/人工）→
    立即停发、转 S13 仅监控，不与其争控。
  - **到达判据**：水平距 Home ≤`arrival_radius_m`（2m）、垂直误差
    ≤`arrival_vertical_m`（0.5m）、水平速度 ≤`arrival_speed_mps`（0.5m/s）
    持续 `arrival_hold`（1s）→ S13。返航超 `timeout`（60s）→ 原地悬停告警。
  - **进入 S12 时已落地/上锁**（`landed && !armed`，例如 S5 无目标停发后
    PX4 自行 RTL 完成、模式又恢复 Offboard）：不再发返航位置意图，直接转
    S13 → S14 收口（SITL 2026-10-22 实测：否则会停在 S12 直到返航超时）。
  - **S13/S14**：S13 发 `MAV_CMD_DO_SET_MODE`（AUTO.LAND，4/6）；停止位置意图；
    落地后若仍 armed 发 `MAV_CMD_COMPONENT_ARM_DISARM`（0）；确认 disarmed →
    S14。正常返航不使用 PX4 自动 RTL；PX4 已接管则只监控 landed/disarmed。
  - **配置**：`MissionStateMachineConfig::return_home`（`config::ReturnHomeConfig`，
    默认 `enabled=true`；`interception`/`return_home` 两节已接入 JSON 加载）。
    捕网执行器隔离用例显式 `enabled=false`。
  - **依赖**：状态机为 Home 换算链接 `drone_perception`（纯函数 `GeodeticToLocalNed`）。
  - **边界**：S9/S10 入口已配置化（`interception` 节，默认关闭；射程上限 2026-10-23
    冻结为双目测距极限 8m，下限仍占位）；
    V1 返航段可通过测试钩子/测试用例或显式开启配置驱动。多发命中判据/再交战策略
    V2 立项；`MissionStatus` 结果字段暂不新增（用状态+日志）。
- `SetCommandSender`/`SetEstimationDiagnostics`须在 Start 前调用；运行中注入/重绑输入被拒绝并记 ERROR。
- `SeedAcceptedAssignmentForTest`（2026-10-28 增）为测试入口：把给定分配写进
  `assignment_tracker_`/`RememberResult(kAccepted)`/`accepted_assignment_`，并在
  飞行快照有效时 `BeginTakeoff(...)`；同样受 `test_hooks_enabled` 门控，未启用时
  `RecordError` 拒绝。**它不是生产入口**：生产装配不置位，唯一的调用方是
  `px4_link_smoke --sitl-stage4-command-assigned`。
- `ForceStateForTest`为测试入口（**运行期门控**）：与生产路径共用`TransitionTo`
  统一迁移挂点，用于驱动拦截序列（S9/S10/S11）的进入/离开动作。**门控**：
  `MissionStateMachineConfig::test_hooks_enabled` 默认 false（生产装配语义，
  `DroneApplication` 不置位），此时调用**直接拒绝并记 ERROR、不产生任何迁移**——
  强推到 S9/S10 会在 `enable_control=true` 装配下经命令窄接口发出**不可逆的捕网
  触发命令**；单测与 SITL 验收工具显式置 true。生产侧另有 `interception.enabled`
  显式开关（默认 false）。
- 构造期对 `actuator` 配置执行 `Validate()` 兜底（single_pwm 实现以
  `params[net_channel-1]` 索引固定 7 元素参数数组：程序化构造的 Set 越界即 UB；
  归一化值超出 [-1,1] 也会被拒）；配置加载期已校验，此处对程序化构造 fail-fast，
  先例见 FlightController 构造期 period 防御。
- 控制输出是否接 PX4 仍由`DroneApplication`的`enable_control`门禁决定（绑定
  `Px4Setpoint`+注入快照源+注入命令接口只在 true 分支）；诊断接口注入与
  `enable_control` 无关（只读诊断，影子期即可用）。

## 日志行为

围栏相关（2026-10-29）：

- INFO：`围栏区域已更新: zone=../.. 顶点=.. 动作=.. 高度带=[..,..]m 基准=..`（仅内容变化时）、
  `围栏区域缓存已装载/已保存`、`任务状态机围栏区域输入已绑定`。
- WARN：`围栏非法区域已忽略`（节流，带累计计数）、`围栏区域缓存不可用`、
  `围栏越界保持: zone=../.. 距边界=..m`（节流）、`围栏越界中止: zone=../.. 距边界=..m 持续=..ms → S12 返航`。
- 65019 由 `fence.status_period_ms` 控制上报节拍，本身不打日志（链路层有节流发送日志），
  否则 1Hz 状态会淹没热路径。

人工指令相关（2026-10-29）：

- INFO：`任务状态机任务指令输入已绑定（65020 主动返航）`（启动期一次）；
  `任务指令回执已发布: command_id=.. type=.. result=.. reason=.. state=.. count=..`（节流，首条与每 100 条）；
  `任务指令重复已忽略`、`任务指令（主动返航）: 已在返航/降落流程，回执完成`、
  `任务指令（主动返航）: 当前状态不可执行`。
- WARN：`任务指令协议版本不支持`、`任务指令类型不支持`、
  `任务指令（主动返航）: state=.. → S12 返航 command_id=..`（接受路径，关键动作不节流）、
  `任务指令（主动返航）: 起飞阶段中止 returning=.. command_id=..`。


- INFO：状态机创建、启动、停止、销毁；状态转移（含交接进入/验证通过/撤销/超时/
  视觉断流退出/测距失效退出/退出迟滞回退/伺服未启动超时，一一对应中文 reason，
  便于事后区分是哪条退出路径生效）；位置引导意图启动/停止（含"交接冻结最后位置目标"、
  "交接冻结位置意图续发"、"位置引导意图停止: 视觉伺服接续"、交接段收口）；
  S7 视觉伺服意图启动（带首拍 vx/vy/vz/yaw_rate 摘要）；PX4 命令接口与估计器
  诊断接口注入；首次/节流接收地面站目标；**返航开始/爬升完成/降落阶段/
  AUTO.LAND 入队/DISARM 入队/任务结束（S14）**（S11~S14，2026-10-19）；
  **接收任务分配（节流）/分配已接受（含 assignment_id/target_id/起飞原点）/
  重复分配只重发 65015**（2026-10-22）；**S3 自动起飞开始（原点/lead/超时）/
  176 DO_SET_MODE 已入队 / 400 ARM_DISARM 已入队 / S4 自动爬升开始（高度/
  爬升率/到达判据）/ 到达任务高度 / 地面中止 DISARM 已入队**（Task 5），
  均为状态变化级、每次起飞最多一条。
- WARN：启动前未绑定输入；目标过期；估计失效导致位置意图停止（节流）；导航状态丢失；
  S7 视觉伺服停发意图（控制律降级，节流，带降级原因）；S7 热备份断粮（节流，不退出 S7）；
  **捕网执行器触发/复位命令已入队**（状态变化级事件，带通道/行程/归一化值摘要）；
  捕网命令入队失败（节流，下拍重试）；**单发发射超时/返航超时/Home 或锚点无效/
  PX4 接管返航/AUTO.LAND 或 DISARM 入队失败**（S11~S14，节流）；
  **任务分配被拒绝（节流，带 assignment_id/target_id/reason/当前状态）**；
  **自动起飞中止（带 reason/detail/airborne/uncertain，每次中止一条）/
  地面中止 DISARM 入队失败（不重试）**（Task 5）；
  **任务取消被拒绝（节流）/地面站取消任务（地面/已离地，状态变化级每条）/
  S5 目标看门狗中止（带 reason/detail）**（Task 10）。
- INFO（Task 10）：**S5 目标看门狗启动**（target_id/wait_timeout/lost_timeout）。
- ERROR：订阅或线程启动异常；运行中重绑输入/注入接口；估计输入部分绑定；
  估计输入已绑定但未注入估计器诊断（交接不评估）。
- 高频周期（100ms 决策、10Hz 意图重发、S7 逐拍伺服计算）不打印日志；
  交接判据与伺服计算每周期静默评估，只在状态迁移、意图来源切换与降级时打日志；
  重复异常使用第1次和每100次节流。

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
- P3 前馈（`GpsApproachPublishesPositionIntent` 扩展 +
  `GpsApproachNormalizesUnsetFeedforwardAxes`/`GpsApproachRepublishesOnFeedforwardChange`）：
  S5 逐轴复制有效且有限的 `TargetState.vel_*` 到 `feedforward_*`；未置位轴清零；
  只改前馈值/掩码也会立即重发；S6 冻结意图 `feedforward_valid_mask==0`。
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
- 视觉伺服（Task 9，`VisualServoFixture` + `MakeServoTestConfig`：在交接测试配置上
  注入非零距离环比例增益 kp=0.1——占位默认 kp=0 会让前向速度恒为零；用例按期
  刷新视觉/测距/姿态三路输入满足控制律内部超龄阈值）：
  - `VisualTrackingPublishesVelocityIntent`：S7 发布`control_source=1`速度航向
    意图（bit31、frame_id=3、有效期来自配置），速度分量对拍控制律期望值
    （vx=kp·Δd=0.5m/s、vz=vz_gain·α≈−0.0004m/s、yaw_rate=gain·β≈1.095°/s）；
  - `VisualTrackingIntentIsLocalNed`：姿态 yaw=90°时机体前向 0.5m/s 映射为
    NED 东向 0.5/北向 0（机体系→NED 旋转已在意图内完成）；
  - `VisualLostBrakesThenExitsToGpsApproach`：S7 中 LOST 4 条（< exit_lost_frames=6）
    保持 S7且意图转零速`kBrakeHover`（reason=102）；补足 6 条回退 S5 后位置引导
    链接管（重新出现`kPositionGuide`）；
  - `HotBackupKalmanStarvedWarnsInS7`：S7 中估计断粮 → 告警位 bit2 置位 +
    节流 WARN「热备份断粮」，状态保持 S7；
  - `HandoverPublishesFrozenPositionUntilServoStarts`：S6 驻留期估计更新后冻结
    续发仍指向进入时刻值（12.5,−3,−8），`valid_until`逐拍滚动；S7 首条视觉速度
    意图发出后不再出现位置意图；
  - `S7ServoNotStartedFallsBackAfterTimeout`：S7 中控制律持续降级（不提供姿态 →
    姿态超龄）时伺服永不启动——冻结位置续发在 `timeout_ms` 内持续滚动（无速度/
    刹车意图），超时收口回退 S5，日志 reason="S7伺服未启动超时"；
  - `ServoSuspendedBitTracksVisualServoOutput`：S7 中伺服未启动（控制律降级）→
    `MissionStatus` 告警位 bit3 置位且状态保持 S7；恢复新鲜姿态、首条视觉意图
    发出后 bit3 在同一 S7 驻留内清除（标注"报告状态与实际输出背离"的两态）。
- 捕网执行器（Task 10 / Task 5，`FakeCommandSender` 假件按调用顺序记录
  SendCommand（命令/参数/入队结果）并可脚本化入队失败/ACK/超时/取消；用例经
  `ForceStateForTest`（配置置 `test_hooks_enabled=true`）驱动拦截序列——与生产
  路径共用 TransitionTo/决策线程挂点。**触发类用例必须先发布 connected 飞行
  快照**（Task 5 起无连接证据不触发）：
  - `InterceptExecutingFiresOnce`：进入 S10 → 恰好一条 187 命令
    （p3=归一化触发行程 -0.7），停留多拍不重复；
  - `LeavingInterceptWithoutFireSendsNoCommand`：S9 未触发即离开 → 零命令
    （复位不再由离开触发）；
  - `ResetRunsAfterFireAckWithoutLeavingS10`：触发 ACK 到达后 `reset_delay=0`
    → 执行器发复位行程（p3=0.0），状态保持 S10；
  - `NetActuatorFiringRetriesWhenEnqueueFails`：前两次入队失败 → 下拍重试直至
    成功（成功入队后不再补发，重试内容一致）；
  - `FireEnqueueTimeoutFaults`：入队持续失败超过 `fire_timeout` → S16、
    `CancelPendingCommands` 恰好一次；
  - `FireAckTimeoutEntersFailsafeAndCancelsCommands`：187 ACK 超时 → S16、
    bit4 置位、取消一次、不再补发复位；
  - `DisconnectDuringFireWaitFaults`：等待触发 ACK 期间 PX4 断连 → S16、取消一次、
    零补发；
  - `ResetRunsDuringReturnAfterFireAck`：已进 S12 后 ACK 到达，执行器仍按计时
    发复位（p1=0.0），证明复位与状态迁移解耦；
  - `NoCommandSenderNeverFires`：未注入命令接口 → 即使进入拦截态也零调用；
  - `ForceStateForTestRejectedWithoutTestHooks`：`test_hooks_enabled=false`（生产
    装配默认）时强制迁移被拒绝并记 ERROR、状态不变、已注入的命令接口零调用
    （防"生产路径误触发射网"）；
  - `SeedAcceptedAssignmentRejectedWithoutTestHooks` / `SeededAssignmentThenCommandEmitsCancelledAck`
    （2026-10-28）：测试钩子门控 + 「已预置分配 + 65020」必须补发 65015
    `cancelled_by_gcs`（SITL `--sitl-stage4-command-assigned` 共用同一入口）。
- 拦截后返航（S11~S14，2026-10-19；`MakeReturnHomeTestConfig` 默认
  `return_home.enabled=true`，缩短到达保持/命令重试/发射超时便于测试）：
  - `InterceptionFiresThenReturnsHomeAndLands`：强制 S10 → 发射命令入队 →
    S11/S12 → 位置意图 reason=8、目标 Home 局部 NED（≥5m 保持当前高度）→
    到达后 S13 发 176（AUTO.LAND，param2=4/param3=6）→ landed+armed 发 400
    （DISARM）→ disarmed 进 S14；
  - `ReturnHomeClimbsToFiveMetersBeforeCruise`：相对 Home 2m → 先原地爬升到
    Home 上方 5m（intent x/y=当前），到高后改为水平飞 Home；
  - `ReturnHomeHoldsWhenHomeInvalid`：`home_valid=false` → 原地悬停意图
    （当前 x/y/z），状态保持 S12，不盲目返航；
  - `ReturnHomeYieldsWhenPx4LeavesOffboard`：S12 中 PX4 切 4/5 → 停发返航
    设定值、转 S13 仅监控，不再发 AUTO.LAND（176）。
  - `MakeNetActuatorTestConfig` 显式 `return_home.enabled=false`，保持捕网
    执行器用例的 S10 停留/显式离开语义不变。
- S9/S10 入口（2026-10-19；`MakeInterceptEntryTestConfig` 开启
  `interception.enabled=true`、`lock_frames=3`、窗口 2~12m、自动授权）：
  - `InterceptEntryLockAndRangeThenFiresAndReturns`：强制 S7 + 连续 LOCKED
    帧 + 8m 测距 + Offboard/armed/姿态/位置有效 → S9 → 自动授权 S10 →
    捕网 187 入队 → S11/S12；
  - `InterceptEntryRejectsDistanceOutsideWindow`：测距 20m（>12m）→ 停在 S7、
    不发射（0 次命令）。
  - `InterceptAuthorizationTimeoutAbortsAndReturns`（2026-10-30 审查修复回归）：
    `auto_authorize=false` + `authorization_timeout=150ms`，S9 条件持续满足到超时 →
    转 S12、65015 `aborted reason=target_wait_timeout`、停发视觉伺服、零发射命令。
- 围栏回归（2026-10-30 审查修复）：`FenceLandActionAbortsToLanding`（land 动作
  走 `AbortForFence()`：转 S13 + 65015 aborted + 零 187）、
  `FenceRejectsDegenerateZoneAndKeepsPrevious`（自交/共线/重复点/高度带倒挂/
  非法高度基准均拒收并保留上一份可用区域）、
  `FenceSameIdVersionGeometryChangeRewritesCache`（同 id/version 改几何也落盘）；
  `MissionCommandAckSequenceIncrementsOnce`（65021 序号逐条 +1）。
- 配置（`interception`/`return_home` 节）：默认值、显式解析、非法窗口/
  返航高度拒绝、`interception.enabled=true` 且 `return_home.enabled=false`
  拒绝（发射后必须有返航收口）。
- 任务分配入口（Task 4，2026-10-22；`AssignmentHarness` 装配 65014/65015
  Topic、命令接口与估计输入，`MakeGroundedFlight` 提供地面/未 armed/导航有效
  快照）：
  - `AssignmentRejectedWithoutCommandSender`：影子装配 → `control_disabled`、
    S2、availability=unavailable；
  - `AssignmentRejectedWithoutEstimationInputs`：有命令接口但未绑估计 →
    `preflight_failed`、S2；
  - `AssignmentInvalidFieldsRejected`：版本 2、未知 flags、未授权、target_id=0、
    类型 26、valid_for=0、已过期 → 分别 `unsupported_version`/
    `invalid_message`/`unsupported_interceptor_type`，全部保持 S2；
  - `AssignmentAcceptedEntersArming`：合法分配 → accepted、
    availability=assigned、S2→S3、65013 写入 assignment/target/availability/
    interceptor_type，且命令接口零调用（Task 4 不发 176/400）；
  - `DuplicateAssignmentResendsAckWithoutReentry`：先以 armed 快照拒绝；
    重复 id（含恢复地面状态后）只重发原拒绝回执、不进入 S3；换新 id 才接受；
  - `GroundTargetDoesNotEnterGpsApproachWithControl`：注入命令接口后 65010
    目标不单独进入 S5；
  - `ShadowGroundTargetStillEntersGpsApproach`：绑定分配输入但未注入命令
    接口时，既有 S2→S5 影子路径保持。
- S3/S4 自动起飞（Task 5，`AssignmentHarness` 可注入缩短的
  `MissionStateMachineConfig`；分配类用例默认放宽 bootstrap/mode/arm 超时，
  避免不驱动 PX4 的 Task 4 用例被 S3 超时打断）：
  - `TakeoffArmingPublishesIntentAndSendsModeOnce`：S3 发布 kTakeoff 意图
    （原点/bit31/frame=3/有效期）；bootstrap_lead 内零命令，之后恰好一次
    176（p1=1/p2=6/p3=0），停留不重复；
  - `TakeoffOffboardThenArmEntersS4AndReachesS5`：6/0 前不发 400；实际 6/0 后
    恰好一次 400（p1=1）；armed 后进 S4；到达判据满足后进 S5 且
    availability=engaged；
  - `TakeoffModeTimeoutAbortsWithoutRetry`：176 超时 → 65015
    aborted/mode_timeout、回 S2 READY、不重发、无残余意图；重复 assignment_id
    重发最新 aborted 结果；
  - `TakeoffArmTimeoutAbortsWithoutRetry`：400 超时 → aborted/arm_timeout、
    未 armed 不 DISARM、不重发；
  - `TakeoffGroundArmedAbortSendsDisarmOnce`：地面 armed 时 176 ACK 失败 →
    aborted/mode_timeout，只发一次 DISARM(param1=0)；
  - `TakeoffAirborneAbortReturnsHomeWithoutDisarm`：已离地 PX4 断连 →
    aborted/px4_disconnected、availability=unavailable、转 S12 且不 DISARM；
  - `TakeoffClimbRampsZTargetAndReachesS5`：S4 z 目标逐拍推进（不阶跃），
    到达后 S5；
  - `TakeoffClimbGroundTimeoutDisarmsAndReturnsReady`：S4 地面超时 →
    aborted/takeoff_timeout、一次 DISARM、回 S2 READY；
  - `TakeoffClimbAirTimeoutLandsInPlace`：S4 空中超时且仍 Offboard → S13
    发 AUTO.LAND(4/6)；
  - `TakeoffClimbAirTimeoutYieldsWhenOffboardLost`：S4 空中超时且 PX4 已离开
    Offboard → S13 仅监控，不发 AUTO.LAND；
  - `TakeoffAckFailureAbortsImmediately`：176 ACK IN_PROGRESS(5) 不中止、
    明确失败(4) 立即中止，不重发；
  - `TakeoffUncertainStateLatchesUnavailable`：中止时离地状态不明 →
    65015 aborted availability=unavailable，回 S2 后新分配被拒 not_ready、
    仍 unavailable，直到重启。
- 电池守卫（Task 8，2026-10-22；夹具默认带 `BAT_LOW_THR=15%`、
  `COM_LOW_BAT_ACT=0`、遥测有效且剩余 80%，用例只覆写单项）：
  - `BatteryPreflightRejectsInvalidParameters`：阈值/动作未读到、动作非 0、
    阈值 0/100 → `battery_config_unknown`、availability=unavailable、S2；
  - `BatteryPreflightRejectsInvalidTelemetry`：`battery_valid=false`/
    `remaining_valid=false`/百分比 NaN → `battery_telemetry_invalid`；
  - `BatteryLowOnGroundUnavailableThenRecovers`：`pct == BAT_LOW_THR` →
    `battery_low`、unavailable；恢复后新分配可接受（不锁存 terminal）；
  - `BatteryLowInArmingAbortsOnGround`：S3 地面低电 → aborted/battery_low、
    回 READY、无 176/400/187；
  - `BatteryLowAirborneInTakeoffReturnsHome`：S4 空中低电 → aborted/
    battery_low → S12，不 disarm、不触发 187；
  - `BatteryLowInGpsApproachDebouncesThenReturnsHome`：S5 低电不足防抖不
    中止；持续后 → S12，且 S12 不重复 aborted；
  - `BatteryTelemetryLossInGpsApproachReturnsImmediately`：S5 遥测失效
    → 立即 aborted/battery_telemetry_invalid → S12，不等防抖、无 187。
- 取消/目标过滤/看门狗（Task 10，2026-10-22）：
  - `CancelOnGroundInArmingReturnsReady`：S3 地面取消（无 bit0）→ 65015
    cancelled/cancelled_by_gcs、回 READY、零 176/400/187；重复取消重发结果；
  - `CancelGroundArmedSendsDisarmOnce`：地面 armed 取消 → 一次 DISARM
    (param1=0)，重复取消不重发命令；
  - `CancelAirborneInTakeoffReturnsHomeWithoutDisarm`：S4 空中取消 → S12、
    availability=returning、不 disarm、无 187；
  - `CancelAirborneInGpsApproachReturnsHome`：S5 取消 → S12、不 disarm、无 187；
  - `CancelWithoutActiveAssignmentRejected`：无活动分配取消 →
    rejected/not_ready、保持 S2；
  - `S5TargetIdFilterIgnoresNonMatchingTargets`：控制任务 S5 下 target_id=99 的
    65010/估计不产生位置意图；匹配 target_id=7 后恢复引导；
  - `S5TargetWaitTimeoutReturnsHome`：S5 无匹配目标超过
    `target_wait_timeout=300ms` → aborted/target_wait_timeout → S12，不重复
    aborted、无 187；
  - `S5TargetLostTimeoutWithRecovery`：S5 短暂丢失（200ms < 400ms）并恢复不
    中止；再次持续失效超过 `target_lost_timeout=400ms` → aborted/target_lost
    → S12，无 187；
  - `ShadowS5IgnoresFilterAndWaitTimeout`：影子 S5 不按 target_id 过滤，
    `target_wait_timeout` 不生效（既有影子行为保持）。
- SITL 返航专项（2026-10-19，`tools/px4_link_smoke --sitl-stage4-return` +
  `config/px4_sitl_return.json`）：S7→S9→S10 发射→S12 算力板返航 Home→
  S13 AUTO.LAND→S14 DISARMED 全 PASS（状态机迁移 11 次、错误 0；最大高度
  4.99m；S12 起始射程约 11.6m）；证据 `tools/return_evidence/`。该场景为
  非门禁，`interception.enabled` 生产默认关闭。

## 排查/修改要点

- 65019 一直 `no_zone(3)`：先确认 `fence.enabled=true`、65018 在发、`config/zone.json`
  是否存在；再确认 `SetFenceInput()` 已在 Start 前绑定（否则只吃缓存）。
- 65019 是 `zone_invalid(4)`：顶点不足 3、zone_id/version 为 0，或 EKF 锚点无效
  （`ned_ref_valid=false`，此时无法把经纬度换成 NED，多边形保持无效而不误判越界）。
- 越界不动作：检查当前状态是否在 `kArming`~`kPostIntercept` 之间（地面态按设计不动作）、
  `violation_debounce_ms` 是否过长、以及 65016 之外的 `action` 是否被 `follow_zone_action` 覆盖。
- 起飞被 65015 `reason=21 zone_mismatch` 拒绝：地面站 65014 的 zone_id/zone_version 与
  机载缓存不一致——先发 65018 对齐区域，或让地面站发 zone_id=0（未声明，靠机载缓存）。
- 修改顶点/动作字段必须同步 `GroundStationLink::HandleFenceZone()` 与
  `docs/数据接口文档.md` §3.2.3，并保持 65014 保持 v1、65013 保持 v2（新字段只占保留区）。

- **S10 发射后不返航**：查 `MissionStateMachineConfig::return_home.enabled`
  （默认 true；捕网执行器隔离测试显式 false）、执行器 `Fired()`/发射超时日志；
  若已进入 S16，说明发射入队超时或执行器故障，属故障处置而非返航路径。
- **S12 原地不动/不飞 Home**：查 `home_valid`/`ned_ref_valid`/
  `local_position_valid`（任一无效按设计原地悬停并 WARN）；查返航超时日志
  （60s 后原地悬停）；确认 PX4 仍在 Offboard(6/0)，否则状态机按设计转 S13 让权。
- **S12 不进入 S13**：到达判据为水平 2m、垂直 0.5m、水平速度 0.5m/s
  持续 1s；查位置/速度遥测是否持续满足；位置意图 reason 低位应为 8。
- **S13 不发 AUTO.LAND**：确认 `SetCommandSender` 已注入（`enable_control=true`
  装配分支），查 `AUTO.LAND 命令已入队`/入队失败 WARN；PX4 已被 failsafe
  接管时状态机按设计不再发 176，只等 landed/disarmed。
- **S13 已落地但不到 S14**：通常为仍 armed 且 DISARM 未 accepted；查 400 命令
  入队/ACK 与 PX4 disarmed 状态。
- **65014 发送后没有 65015**：确认 `SetAssignmentInput` 已绑定且
  `GroundStationLink::MissionAssignmentOutput()` 已接到该 Topic；地址/来源
  不匹配的 65014 由 GroundStationLink 丢弃（设计上不回 65015），不是状态机问题。
- **65014 回 `control_disabled`**：影子装配（未注入命令接口）属预期；正式
  自动起飞必须在 `enable_control=true` 装配分支下注入 `SetCommandSender`。
- **65014 回 `preflight_failed`**：按顺序查：估计/视觉/测距三输入是否成组
  绑定；飞行快照是否 `landed_state_valid && landed && !armed`；局部位置/
  姿态/Home 是否有效。回 `px4_disconnected` 时查 PX4 心跳。
- **65014 回 `unsupported_interceptor_type`**：`MissionStateMachineConfig::
  expected_interceptor_type`（生产应取 `ground_station.aircraft_component_id`）
  与地面站填写的 `interceptor_type` 不一致（25 捕网/26 火箭）。
- **65014 回 `invalid_message`**：检查版本=1、flags 只置已知位且 bit0=1、
  target_id=0 非法、`valid_for_ms>0` 且未过期（按本机收到时刻起算）。
- **重复发送同一 assignment_id 一直回原拒绝/中止**：这是幂等语义（只重发
  最新 65015，不再预检）；地面站需换新的 assignment_id 才能重新分配。
- **65014 取消回 rejected(not_ready)**：没有活动分配（S2 未分配，或已
  cancel/terminal）；取消必须携带当前活动分配的 assignment_id。地面取消后
  机载回 READY，可用**新** id 重新分配；空中取消后 tracker 保持
  returning/terminal，本次上电不再分配。
- **S5 无目标自动返航 / 目标丢失自动返航**：查 65015 reason
  `target_wait_timeout`（从未获得匹配目标）或 `target_lost`（获得后持续失效）；
  阈值取 `takeoff.target_wait_timeout`/`target_lost_timeout`。常见原因是
  65010 与估计器的 `target_id` 与 `assignment.target_id` 不一致（控制任务下
  非匹配目标被过滤，不会更新 S5 引导）、估计器断流（`valid=false`/超龄）或
  65010 过期。
- **S3 不发 176**：确认已注入命令接口（`enable_control=true` 装配分支）、
  已进入 S3；176 在 `takeoff.bootstrap_lead` 之后才发，期间每拍只发
  kTakeoff 原点意图。若日志有「176 入队失败」→ 命令链路/队列异常，按设计
  中止（bootstrap_timeout），不重试。
- **S3 超时回 READY**：看 65015 reason 与「自动起飞中止」WARN：mode_timeout
  =未等到实际 6/0（查 PX4 Offboard 是否接受、遥控/模式开关、176 ACK），
  arm_timeout=未等到实际 armed（查 PX4 解锁检查/ACK，PX4 拒绝 arm 表现为
  超时），bootstrap_timeout=176 入队失败，px4_disconnected=心跳断。
  **不重试 176/400**，需地面站用新 assignment_id 重新分配。
- **S4 不爬升/不前进**：确认仍在 Offboard(6/0) 且 kTakeoff 意图的 target_z
  在逐拍下降（10Hz 每次 `climb_rate*dt`，不是一步到目标）；高度/水平/速度
  到达判据看实际快照，持续 `stable_time` 才进 S5。S4 空中超时仍 Offboard →
  S13 发 AUTO.LAND；已离开 Offboard → S13 仅监控（不发 176）。
- **起飞中止后 availability 一直 unavailable、新分配被拒 not_ready**：中止
  时 PX4 状态不确定（断连/离地状态不明）置了 per-boot
  `takeoff_unavailable_latched_`；确认飞机已安全落地/上锁后重启进程（`Start`
  复位）再调度，不要绕过该标志。
- **起飞中止后仍看到 S12 位置意图**：已离地中止按设计转 S12 返航（不是
  残余起飞意图）；S3/S4 自身的 kTakeoff 意图在中止当拍即停发。
- **65014 回 `battery_config_unknown`**：PX4 `BAT_LOW_THR`/`COM_LOW_BAT_ACT`
  尚未读到或非法（阈值不在 (0,100)、动作不是 0）。查 Px4Link 参数回包日志与
  `battery_low_threshold_valid`/`low_battery_action_valid`；可先用 QGC 确认
  PX4 侧参数，不要改机载阈值绕过。
- **65014 回 `battery_telemetry_invalid`**：PX4 电池遥测无效（电压/剩余量
  字段或新鲜度）。查电源模块接线、`SYS_STATUS`/`BATTERY_STATUS` 是否到达；
  不得把无效值伪造为 0 继续调度。
- **65014 回 `battery_low` / 地面 availability 一直 unavailable**：剩余电量
  不高于 `BAT_LOW_THR`（含相等）。充电/换电后遥测恢复且未执行空中任务，
  availability 自然回 ready，可重新分配（需新的 assignment_id）。
- **飞行中突然转 S12 且 65015 aborted**：先看 reason：`battery_low`（持续
  低电超过 `takeoff.battery_low_debounce`）或 `battery_telemetry_invalid`
  （遥测立即失效）。这是设计内中止：S12 用机载位置设定值返航，不走 PX4
  RTL、不空中 disarm；`battery_abort_latched_` 保证一次任务只回一次。
  S12/S13 里再收到低电不会重复中止，属预期。

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
- 停在 S7 不退出：按四条退出路径逐项查——① 视觉状态消息是否仍在到达
  （断流超 `visual_stale` 会立即退出，日志 reason="视觉状态断流超龄"）；
  ② 测距是否 `valid=true` 且未超龄（失效连续达 `distance_lost_ms` 会退出，
  reason="双目测距失效达距离丢失门限"）；③ `LOST` 连续帧数是否未达
  `exit_lost_frames`（进入门限约 1s、退出门限约 3s 属设计迟滞）；
  ④ 伺服是否已启动（首条视觉意图发出前驻留超 `timeout_ms` 会按
  reason="S7伺服未启动超时"退出——若停在 S7 且日志无该 reason，
  说明伺服已启动过，属③的"应当停在 S7"情形）。
- S7 提前退出（非预期）：先看状态转移日志的 reason——"断流超龄"说明监视器/
  链路停止发布状态（查监视器线程与上游 YOLO 观测）；"测距失效"说明双目匹配
  连续失效或测距线程停发（查测距组件与视差有效域）；"伺服未启动超时"说明
  首条视觉意图在 `timeout_ms` 内始终发不出（先按下一行"没有速度意图"查降级
  原因，多为姿态超龄）；前两条可能在视觉仍 LOCKED 时发生，属安全收口而非缺陷。
- S7 没有速度意图（WARN"视觉伺服停发意图: 控制律降级"）：按 reason 查——
  ①"姿态超龄"：`FlightStateSnapshot.attitude_valid`为 false 或接收时间超
  `attitude_stale_ms`（查 PX4 姿态流是否在发布、时间戳是否新鲜）；
  ②"视觉超龄"：`VisualTargetStatus`接收时间超`visual_stale_ms`；
  ③"持续无距离退出"：测距失效超`no_distance_exit_grace_ms`（该档位下可见）。
  三者均属控制律安全降级（不发布意图 → 控制器意图超时刹车），不是状态机缺陷；
  若持续超 `timeout_ms` 未恢复，S7 会按退出路径 ④ 收口回 S5。
- S7 收到`kBrakeHover`零速意图而非速度意图：短期视觉 LOST 的判丢刹车
  （reason=102）——主链路无预测坐标，coast 以刹车形态落地；LOST 连续帧未达
  `exit_lost_frames`前属预期，达门限即退出 S7。
- WARN"热备份断粮"：S7 期间卡尔曼状态年龄超出其自报有效期窗（估计断粮/
  `valid=false`/从未收到）。**不退出 S7**——查地面站目标是否过期、估计器线程
  与视觉融合是否仍在更新；该告警提示"假备份"风险（回退后 S5 位置链将无
  新鲜估计可用）。
- 交接点出现"先刹停再起步"（速度意图前有零速段）：确认 S6 驻留期是否有
  "交接冻结位置意图续发"日志与滚动续期的位置意图（2000ms 超时档位下
  `timeout_ms` 需大于冻结续发失效时间）；若位置意图在 S6 中途停止，查
  进入判据是否失效（撤销回 S5 的分支，属预期收口而非冻结失效）。
- 整机实测停在 S5、从不进 S6：**属预期（T12 裁决后仍成立）**——`DroneApplication`
  尚未调用 `SetEstimationInputs`（三输入未绑定，状态机保持影子）。G3 全链验收走
  独立装配工具 `tools/px4_link_smoke --sitl-stage4-mission`（见 `tools/px4_link_smoke.md`）；
  生产接线与 `enable_control` 打开一并做。勿据此判缺陷。
- 地面站看到"状态=S7 但飞机不动/无控制输出"：查 `MissionStatus.active_warning_bits`
  bit3（S7 伺服停发中）——置位说明伺服尚未启动或控制律降级停发，实际输出要么是
  冻结位置意图、要么无意图（控制器意图超时刹停）；服务器侧诊断先用该位区分
  "状态机在跟踪"与"实际控制输出在跟"，再查 WARN"视觉伺服停发意图"的降级原因。
- 捕网不触发（日志无「捕网执行器触发命令已入队」）：先确认是否**根本不满足触发条件**——
  ① 整机当前**无拦截序列入口**（S9/S10 进入判据属后续任务，`DroneApplication`
  不驱动），状态达不到 S10，属本任务交付边界而非缺陷；② `SetCommandSender`
  未注入（只有 `enable_control=true` 装配分支注入）→ 本功能物理缺席（零调用）；
  ③ 已注入但仍无日志 → 查「入队失败（下拍重试）」节流 WARN（PX4 未连接/命令
  队列满）；三者都在调试时用状态转移日志确认当前是否真的到过 S10。
- 捕网触发了但机构不动/行程反向：① 核对 PX4 侧该输出通道是否已分配「Peripheral
  via Actuator Set N」（当前装机：MAIN5 = Actuator Set 1，与 `actuator.net_channel=1`
  对应）且已解锁/预解锁（PX4 默认在 disarmed 下不驱动执行器，台架用 pre-arm
  状态测）；② 核对 `PWM_MAIN_MIN5/MAX5` 与舵机实际行程（机载只发 -0.7/0.0，
  映射在 PX4 侧）；
  ③ 行程方向以 **G4 台架**为准（本层只保证归一化命令语义正确，方向属 PX4/
  接线配置）；日志中的 `set/norm` 摘要可核对。
- 触发后机构停在触发位、无复位：复位由执行器在**触发 ACK** 后按 `reset_delay`
  发起，不再由离开 S9/S10 触发。先查 PX4 是否回 187 ACK（无 ACK 会先按超时
  → S16）；查「复位行程命令入队失败（下拍重试）」节流 WARN（连接正常但队列满
  时下拍重试）；若已进 S16，按故障处置（不补发复位）。
- **进 S16 / 地面站 bit4 置位**：查 ERROR「捕网执行器故障，停止任务控制并交
  PX4/遥控」的原因串（187 ACK 超时 / 非 ACCEPTED / PX4 断连 / 触发命令未入队
  （发射超时））。S16 为终态，状态机不再发任何控制意图，需人工/维护处置后
  重启程序；`CancelPendingCommands()` 只取消待发命令，不会撤回机构已执行动作。
- 后续恢复完整自动链时，不能直接从`READY`跳到`GPS_APPROACH`输出控制；必须补齐`ARMING/TAKEOFF`门禁、SITL、拆桨台架验证和`enable_control`安全配置。
