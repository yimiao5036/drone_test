# 任务状态机实现文档

## 功能职责

`MissionStateMachine`负责消费地面站目标、PX4飞行状态和健康状态快照，按任务阶段维护`MissionStatus`。任务链路按阶段推进：满足导航快照基本条件时进入`GPS_APPROACH`。

阶段四新增（Task 7）：经`SetEstimationInputs`绑定目标估计/视觉稳定性/双目测距三个输入后，`GPS_APPROACH`（S5）开始发布`kPositionGuide`位置引导意图（2026-11-11 起落点为速度律 `p_ref` 锚点，三轴速度前馈同源；`control_source=0`，`reason_code`带 bit31 控制权请求位`common::kControlIntentReasonRequestControl`）。**未绑定时不出意图**。意图是否真正转化为 PX4 输出由`FlightController`授权链与**运行模式**装配决定，状态机侧不做最终裁决；
**本模块不读 `enable_control`**——需要判断“能不能发指令”时只看 `command_sender_ != nullptr`
（定义见 [`docs/运行模式与模块状态定义.md`](../../docs/运行模式与模块状态定义.md)）。

**S5 接近段速度律（2026-11-11 落机载，唯一 S5 引导方式）**：
`PublishEstimateGuidedIntent()` 由 `ApproachSpeedLaw` 每拍产出 `(p_ref, v_ref)`：

- 位置锚点 `p_ref` = 自机 + `v_ref × ref_lead_s`（默认 0.3s，上限 6m）；
- 三轴速度前馈 = 律输出（相对系制动包络 + 速度预算 + 侧向/转弯约束 + 分相位）；
- `max_speed_mps = max(|v_ref|, 0.5)`，仅用于控制器位置引导限速；
- 机头仍由 `ApplyHeadingToward` 指向目标；输入无效（无局部位置/目标无效）时
  本拍不发意图并打节流 WARN，**不再回退旧位置引导**。
- 实现与参数：`include/control/approach_speed_law.md`；测试：
  `MissionStateMachineTest.GpsApproachPublishesSpeedLawIntent`、
  `GpsApproachFacesTargetWithSpeedLaw` 与 `ApproachSpeedLawTest.*`。


阶段四新增（Task 8）：经`SetEstimationDiagnostics`注入估计器诊断接口后，**交接判据**（`handover` 配置节）生效，状态在 S5→S6→S7 之间按下列判据推进/回退（`handover` 阈值全部来自配置，无代码字面量）：

| 迁移 | 判据 |
|---|---|
| S5→S6 | 视觉 `LOCKED` 连续帧 ≥ `enter_s6_lock_frames` + 双目距离有效且未超龄 + 视线角距视场边缘 ≥ `fow_margin_deg` + 视觉观测与卡尔曼预测创新 ≤ `innovation_gate_m`（四项同时满足） |
| S6→S7 | 进入判据持续满足 **且** 视觉链再推进一条新的 `LOCKED` 帧（交接期不断流） |
| S6→S5 | 进入判据任一失效（撤销交接）；或驻留超 `timeout_ms`（首条视觉指令未生效，防卡死） |
| S7→S5 | 四条独立路径任一成立：① 视觉状态消息断流超视觉新鲜度阈值（`visual_stale`）；② 测距失效（invalid 或超龄）连续达 `distance_lost_ms`；③ 视觉 `LOST` 连续帧 ≥ `exit_lost_frames`（进入/退出门限不同构成迟滞，防状态抖动）；④ 伺服未启动超时——S7 驻留超 `timeout_ms` 仍未见首条有效视觉意图（控制律持续降级，如姿态超龄），防冻结位置续发无限续期 |

**未注入估计器诊断时交接判据不评估**（保持 S5，启动记 ERROR）：无法核验"视觉锁定与卡尔曼航迹是否同一目标"就不允许切进视觉末段。

阶段四新增（Task 9）：**S7（VISUAL_TRACKING）视觉伺服意图迁入状态机**——内部伺服协作者经
`IVisualControlLaw` 接口持有控制律（2026-11-11 起；当前实现为`TrackingControlLaw`，按
`MissionStateMachineConfig.visual_profile` 装配，配置为`MissionStateMachineConfig.tracking`，
即`visual_tracking`节整节注入），逐拍组装控制快照（视觉稳定性状态 + 双目测距 +
`FlightStateSnapshot`姿态）计算并发布`control_source=1`的速度航向意图（NED 速度，
`reason_code`带 bit31 控制权请求位）。输出→意图映射约定：

| 控制律输出 | 映射意图 |
|---|---|
| `kVelocityHeading`（valid=true） | `kVelocityHeading`：NED 速度分量 + 航向通道（速率式填`yaw_rate_dps`/位置式填`yaw_deg`），reason 低位 100 正常跟踪 |
| `kBrakeHover`（视觉短期 LOST 判丢刹车，valid=true） | `kBrakeHover`零速安全意图（速度分量全零），reason 低位 102；**主链路无预测坐标**（`is_predicted`恒 false，101 Coast 低码不可达）——图像空间 coast 在本链路以判丢刹车形态落地 |
| `kDegradedHold`（valid=false：姿态/视觉超龄） | **不发布意图**（安全语义：控制律已明确"禁止正常控制"，补发零速与其矛盾）；超龄收口交 FlightController 意图超时门禁（`intent_timeout`）刹停 |

**S6 冻结最后位置目标（评审裁决 (a)，Task 9 落地）**：S5→S6 当拍冻结**进入时刻最后一条 S5 位置意图**（速度律的 p_ref 锚点 + 机头，显式去掉速度前馈），S6 驻留期逐拍续发该位置意图（`valid_until`由状态机每拍滚动续期），直到首条视觉速度意图被发出再停（S7 首拍若伺服降级停发，冻结续发继续顶住，不留意图缺口）。此策略对应用户设计 §2.1「进入 S6 后冻结最后位置目标」与 §2.5「位置式→速度式无扰斜坡过渡」——若交接期停发位置意图，控制器在`intent_timeout`(300ms)后 COAST 刹车，真实系统会在交接点先刹停再起步。**P3 决策（2026-10-19）：S6 冻结续发保持纯位置、显式不带速度前馈**；S7 速度伺服接管后自然连续。2026-11-11 起冻结值不再是目标估计点（旧实现会把位置环拉向目标本身、交接瞬间产生数米跳变）。
2026-11-12 复审三轮：S6 续发时**每拍按当前速度重算**停止点预测 cap（`PublishFrozenHandoverIntent()`），
不沿用冻结时的旧值——冻结的是目标点，飞机仍在动。

**热备份核对（设计 §2.4）**：S7 期间每秒核对卡尔曼状态年龄——`TargetState.header.receive_time_ms`年龄是否超出其自报有效期窗（`valid_for_ms`，与 S5 意图门禁`HasFreshEstimate`同一口径；无快照/`valid=false`亦视为断粮）。断粮 → 节流 WARN「S7 热备份断粮」+ `MissionStatus.active_warning_bits` bit2 置位，**不退出 S7**（S7 去留由四条收口路径决定，热备份告警只提示"假备份"风险）。

**与 VisualTrackingShadow 并存（Topic 隔离）**：控制律调用迁入状态机后，观测组件保留（现场观察用），**观测意图主题与状态机`IntentOutput()`是两个独立 Topic**；装配层只把状态机意图接入`FlightController`（见`DroneApplication::BindTopics`注释），不存在双意图生产者竞争。

阶段四新增（Task 10 / Task 5，2026-10-21；2026-11-11 泛化）：**通用拦截设备
`IInterceptionDevice`（当前为 `single_pwm` 捕网执行器）**——注入命令窄接口
`IPx4CommandSender`后，工厂按 `actuator.type` 创建实现（当前 `single_pwm`）。状态机**每拍**
调用 `Update(InterceptFeedback)` 推进执行器内部时序（ACK 确认 → 触发保持 → 复位 → 复位稳定
→ Ready），并在 **S10（INTERCEPT_EXECUTING）** 调一次 `Execute()`（执行器 `Triggered()` 防重复；
入队失败保持 Ready、下拍重试，超过 `return_home.fire_timeout` 判故障）。**S10→S11 以
`Triggered()`（触发命令已入队）为依据，不等发射 ACK**；复位不再由“离开拦截序列”触发——执行器
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
`include/state_machine/interception_device.h` / `.md`。

**拦截序列入口（2026-10-19 起配置化）**：S9/S10 入口判据已实现——`interception` 配置节（默认 `enabled=false`）提供连续锁定帧 `enter_s9_lock_frames`、测距窗口 `[min_range_m,max_range_m]`（上限 2026-10-23 冻结为双目测距极限 8m，下限仍占位）与 `auto_authorize`/`authorization_timeout_ms`；运行时还要求视觉 `LOCKED`、PX4 `connected+armed+Offboard(6/0)`、姿态/局部位置有效、命令窄接口已注入、执行器未触发且 `Ready()`（触发保持/复位/稳定途中不重入）、未进入故障。S7→S9→S10（自动授权）；S9 条件失效回 S7；授权超时/发射结束按 S12 返航；S9/S10 继续输出视觉伺服意图保持瞄准。默认关闭时整机仍不自动发射；测试钩子/独立工具可显式开启。

阶段四新增（Task 12，2026-10-13）：**S7 报告状态与实际控制输出背离的可观测位**——`MissionStatus.active_warning_bits` 新增 bit3「S7 伺服停发中」（`kWarningServoSuspended`）：处于 `VISUAL_TRACKING` 且**本拍未发出视觉伺服意图**时置位，覆盖两种背离形态：① 伺服尚未启动（S6/S7 过渡期仍在续发冻结位置意图，报告已是 S7 而输出仍是位置式）；② 控制律降级停发（姿态/视觉超龄 → 控制器意图超时门禁接手刹停，报告仍是 S7 而无输出）。伺服意图正常发出即清除；离开 S7 亦不置位。位语义与 bit1（目标过期）/bit2（热备份断粮）并列，映射表见 `docs/数据接口文档.md` §3.4 与 `docs/地面站目标位置与时间同步协议.md` §14.3（65013 `active_warning_bits`）。

**拦截设备故障告警位 bit4（Task 5，2026-10-21）**：`MissionStatus.active_warning_bits`
bit4 =「拦截设备 ACK 故障」（`kWarningActuatorFault`）。触发条件为执行器 187 ACK 超时/
非 ACCEPTED/PX4 断连，或 S10 触发命令入队超时；置位同时状态机进入 S16 FAILSAFE（见上段）。
故障为终态，bit4 持续置位（不再叠加其余告警位）直至重启，供地面站识别“任务已中止、交
PX4/遥控”。

**目标保持告警位 bit5（2026-10-22）**：`MissionStatus.active_warning_bits`
bit5 =「S5 目标失效保持中」（`kWarningTargetHold`）。目标不可用且正在发原地保持
位置意图（原因码 9）时置位；目标恢复（连续 3 拍）或离开 S5 后清除。与 bit1
（目标过期）可同时出现——bit1 表示“目标已过期”，bit5 表示“控制流已切换为原地
保持”。映射表见 `docs/数据接口文档.md` §3.4/§3.5。

**引导点越出围栏告警位 bit7（2026-11-05）**：`MissionStatus.active_warning_bits`
bit7 =「引导点越出围栏」（`kWarningGuideOutsideFence`）。判据：本拍 `PublishPositionIntent`
下发的局部 NED 引导点（成员 `guide_point_`，每拍刷新）落在围栏多边形外，且当前处于
S5/S6/S7。背景是现场"目标跑出区域后飞机贴边悬停、页面没有任何信息"：控制器在发
setpoint 前用 `fence::ClampIntoPolygon()` 把越界设定值按 `controller.fence_margin_m`
拉回边界（`src/control/flight_controller.cpp`，只打一条节流 WARN），而
`EvaluateFenceGuard` 只按**飞机自身**位置判定，于是这件事对 65013/65019 都不可见。
bit7 只提示、不改变动作——越界处置仍由 `fence.violation_action` 与自身越界判定决定；
每拍重算，引导点回到区域内即清除（跃变时打一条 WARN「引导点越出围栏」）。
地面站消费口径见 `docs/数据接口文档.md` §3.4。

**2026-11-12 起 bit7 覆盖贴边保持**：制导层钳制（见下「S5 贴边保持」）会把引导点
主动拉回界内，若只按"引导点在多边形外"判定，告警会永久消失——而操作员恰恰需要
知道"目标在区域外、飞机被围栏挡住"。因此新增判据：处于 S5/S6/S7 且
`guide_edge_hold_=true` 时同样置 bit7。中止/离开跟踪链（S12、S3/S4 等）时清
`guide_edge_hold_`，避免跨任务沿用旧滞回阈值（2026-11-12 复审）。

**预测越界告警位 bit8（2026-11-12 复审三轮）**：`MissionStatus.active_warning_bits`
bit8 =「围栏停止点预测已介入」（`kWarningPredictedFenceBreach`）。S5/S6/S7 任一
路径的预测限速生效即置位；与 bit7（引导点已经在界外）区分，条件解除即清除。
消费口径见 `docs/数据接口文档.md` §3.1。

**目标出区保持告警位 bit9（2026-11-13）**：bit9 =「目标出区保持中」
（`kWarningTargetOutsideFence`）。目标 NED 位置在围栏外、状态机已停止追击并
发位置保持时置位；条件解除即清除。

**围栏停止点预测（2026-11-12 复审三轮，维护者裁决"两级越界判断"）**：
第一重仍是 `EvaluateFenceGuard()` 的**当前自身位置**判定（深度容差 + 回界滞回 +
防抖 + 区域动作）。第二重是**沿实测速度方向的停止点预测**
（`PredictiveFenceSpeedCap()`）：
`p_stop = own + v̂·(v·guard_reaction_s + v²/(2·guard_decel_mps2))`；
用 `fence::RayDistanceToBoundary()` 取到边界的**首个**交点，若 `p_stop` 越过边界
（含 `boundary_margin_m`）则反解允许速度
`v = −aτ + √((aτ)² + 2a·(d−margin))` 作为本拍速度上限（下限 0.1 m/s：
`max_speed_mps=0` 在控制器侧是"未填上限→回退硬上限"）。
- **只提前限速/保持，不中止**；实际越界仍由第一重处置。
- 适用 S5/S6（位置引导：`max_speed_mps=min(当前,允许)`）与 S7/S9/S10
  （速度意图：速度通道没有控制器的位置钳制；允许速度 ≤0.15 m/s 时清零平移
  速度做**贴边悬停**，偏航跟踪保留并置 bit8）。
- 方向取**实测速度**（<0.5 m/s 不启用，交几何保持）；减速度取可达值，
  **不要用接近律当前相位的计划加速度**；`guard_decel_mps2=0` 关闭预测。

**S5 贴边保持（2026-11-12，维护者裁决"贴边保持 + 告警、不中止"）**：
`ApplyGuideEdgeHold()` 在 `PublishEstimateGuidedIntent()` 组装完速度律输出后执行。
**先叠加上述停止点预测限速**（与参考点是否被挡住解耦——10 m/s 的物理刹停距离
约 17 m，等参考点出界才把限速砸到 0.5 已经太晚），再判几何保持：
参考点距边界不足 `boundary_margin_m`（含界内侧预警带，界外当然也算）→ 进入保持；
参考点回到界内且距边界超过 `fence.guide_hold_release_m`（默认 2 m）→ 解除（滞回）。
进入后做三件事，**全部在制导层**：
1. 位置目标钳到多边形内（`fence::ClampIntoPolygon()` + `boundary_margin_m`）；
2. 三轴速度前馈与前馈掩码全部清零（不再朝外顶）；
3. `max_speed_mps` 取 `min(当前, 0.5)`（贴边微调；预测限速可能已压得更低）。

**不改变任务状态、不发 65015**：任务继续在 S5 等待目标回界内。
`fence.guide_hold_enabled=false` 可回退到 2026-11-11 行为（只靠控制器逐拍钳制）。
进入/解除各打一条日志（`S5 贴边保持: 目标越出区域…` / `S5 贴边保持解除: 目标回界内…`）。

**目标出区保持（2026-11-13，维护者裁决）**：`UpdateTargetOutsideHold()` 在
`EvaluateFenceGuard()` 内每拍更新，`Evaluate` 末尾由
`PublishTargetOutsideHoldIntent()` 覆盖当拍位置/速度意图（原因码 11
`kReasonTargetOutsideFence`）。判据：目标 NED 位置（估计器，局部 NED）落在
多边形外（`target_distance < 0`）；**S5/S6/S7 生效**（S9/S10 拦截段不打断）。
- 飞机在区内：保持点 = 进入保持瞬间的自机位置（"减速到哪就悬停在哪"）；
- 飞机在区外（只可能是减速没减完）：保持点 = `fence::ClampIntoPolygon(自机,
  boundary_margin_m)`，钳回区域内、低速飞回后悬停；不要求停在边界线上；
- 保持期间**不执行区域动作 return/land**（只上报 65019/65013），无超时兜底；
- 解除：目标回到区内、距边界 ≥ `fence.target_outside_release_m`（默认 50 m）
  持续 `fence.target_outside_release_hold_ms`（默认 10000 ms），**且自机已回到区内**；
- 视觉段：S7（以及若后续纳入的 S9/S10）速度意图被位置保持覆盖，即"切换到
  位置引导"；本版保持只在 S5/S6/S7，S9/S10 拦截流程不被打断。
- 日志：`目标出区保持: 目标=(..) 距边界=..m 自机距边界=..m 保持点=(..)`（WARN）、
  `目标出区保持解除: …`（INFO）。

**G3 全链验收（Task 12，2026-10-13，`tools/px4_link_smoke --sitl-stage4-mission`）**：状态机转正验收走**独立装配工具**（工具装配 `MissionStateMachine → FlightController → Px4Link`，扮演地面站/视觉监视器/双目测距三路生产者注入模拟数据），覆盖 S5→S6→S7→回退→停发全序列、交接缝意图无超龄（末条 S5 位置意图→首条 S7 速度意图间隔 < `intent_timeout` 且窗口内未跌入 COAST——`enter_s6_lock_frames` 门限 + 20Hz 注入使 S6 只驻留 1 决策拍，S6 冻结续发的逐拍滚动语义由本模块单测覆盖，工具侧只作诊断记录）、S7 伺服启动、短期/持续 LOST 语义、目标过期停发链与卡尔曼备份接管。**生产装配的 `SetEstimationInputs` 已于 2026-10-22 随多机任务分配接线落地**（估计器/视觉稳定性/双目测距三者齐备时成组绑定）；G3 工具仍保留，作为不依赖真实观测链的独立验收路径，与生产装配共享同一状态机实现与配置注入路径。

本阶段仍不做以下事情：

- 不发送外部 setpoint（控制输出仍由 FlightController 产生）。`SetCommandSender` 当前消费点：S3 的 176/400（Task 5）、S10 捕网触发/复位、S13 交段 AUTO.LAND（2026-11-17 起由 S13a 交段判据/保护触发，见下）、S14 前 DISARM；`FlightController::kBootstrap`（原点保持流）由 Task 6 接入，本模块只保证 kTakeoff 意图持续发布。
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
  只读运行（未注入命令接口）时保持既有 S5 行为，不过滤。
- **S5 目标看门狗**：S5 入口（`TransitionTo(kGpsApproach)`）在控制任务下记录
  `target_wait_started_ms_`；每拍 `EvaluateTargetWatchdog()`：
  - 未获得匹配目标（估计或 65010 任一新鲜即视为获得）超过
    `takeoff.target_wait_timeout` → 65015 aborted(`target_wait_timeout`) → S12；
  - 已获得后匹配目标持续失效超过 `takeoff.target_lost_timeout` →
    65015 aborted(`target_lost`) → S12；恢复即清零丢失计时；
  - 中止当拍停发位置引导（S12 接管返航意图），不触发拦截/187；
    `mission_abort_latched_` 保证一次任务只中止一次（接受新分配/`Start()` 复位）；
  - 仅在 S5 生效，不改变 S6/S7 既有交接/视觉丢失逻辑；只读运行时不启用。

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
- 单测覆盖：保持目标/无前馈/10Hz 续流、3 拍恢复迟滞、无命令接口时不保持、
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
- **S4**：水平保持 `x0/y0`；z 目标 `z_target = z0 - takeoff.height_m`。
  **2026-11-02 起改用“位置台阶 + 垂直速度前馈”**：仍每拍发 `kTakeoff` 位置
  意图（维持控制器起飞预授权链连续），z 指令按 `climb_rate_mps` 逐拍抬升；
  距目标高度 > `max(0.20, z_tolerance_m)` 时额外带**垂直速度前馈**
  （`feedforward_vz_mps = -climb_rate_mps`，mask 含 `kVelocityValidDown`），
  进入末段后直接发目标高度并撤掉前馈。理由：位置台阶会被 PX4 位置环按拍
  执行成“加速→到位→减速”，是 0082 实飞高度抖动的主要来源；补垂直前馈后
  PX4 速度环按恒定速率推进，且不改意图类型（曾试过纯速度型
  `kVelocityHeading`，因意图类型变化触发授权链 `BOOTSTRAP→IDLE→ACTIVE`
  抖动、起飞延迟，已废弃）。
  意图继续带 bit31；`yaw_deg` 尽量取当前姿态 yaw。到达判据同时满足并保持
  `stable_time`：`|z-z_target| ≤ z_tolerance_m`、水平误差 `≤ xy_tolerance_m`、
  `|vz| ≤ max_vz_mps`；满足后 `TakeoffComplete()` + `BeginEngage()` → **S5**
  （availability=engaged）。
- **S4 空中超时**（`height_timeout`）：PX4 仍在 Offboard → 转 **S13**，由现有
  S13 两段落链发 AUTO.LAND(4/6)（2026-11-17 起先进 S13a 机载下降，到交段判据/
  保护/超时才发 176；只读运行/回滚路径仍是直接发）；PX4 已离开 Offboard → 设置
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

**只读运行兼容（关键）**：未注入命令接口（迁移条件 `command_sender_ == nullptr`）时，
65014 一律回 `control_disabled`，`availability=unavailable`（不可被调度）；
既有“新鲜 65010 目标 + 导航就绪 → S2→S5”路径**原样保留**。注入命令
接口后，65010 目标不得单独把 S2 推到 S5——必须由 65014 分配经预检进入 S3。
此分叉只在 Evaluate 的 S2→S5 分支增加 `command_sender_ == nullptr` 条件，
**不读 `enable_control`**，也不删除任何既有测试路径。

完整自动任务链仍以`docs/状态机设计.md`为准。S3/S4 的算力板侧时序已由本模块
落地（Task 5）；`FlightController::ControlAuthority::kBootstrap` 的原点保持
流已在控制器侧实现（Task 6），PX4 参数读取（`BAT_LOW_THR`/
`COM_LOW_BAT_ACT`）由 Px4Link 提供（Task 7），状态机侧电池门禁（S2 预检、
availability、飞行中 S12）由 Task 8 接入；65014 `flags.bit1` 取消、S5 的
`target_id` 过滤与 `target_wait_timeout`/`target_lost_timeout` 收口由 Task 10
接入（2026-10-22），原「已知缺口」已关闭。

**估计器降级处置（2026-11-17，降落与估计器保护 P3）**：每决策拍（100 ms）把最新
`FlightStateSnapshot` 交给 `EstimatorGuard` 判"位置/高度估计还能不能信"（判据与极性见
[`estimator_guard.md`](estimator_guard.md)，设计与实施口径见
[`降落与估计器保护设计-2026-11-17.md`](../../docs/降落与估计器保护设计-2026-11-17.md)
§3.3/§5）。8 类判据（z/xy 单拍跳变、`ESTIMATOR_STATUS.flags` 位、新息检验比、
关键传感器健康位、气压/EKF 高度交叉校验、`STATUSTEXT` 关键字、`LOCAL_POSITION_NED`
超时）任一命中即**锁存降级**，并**一次性**完成：发 `MAV_CMD_DO_SET_MODE(176)`
切 **POSCTL(3/0)** 悬停、停止发布一切控制意图、冻结任务推进。**明确不做**：不自动降落、
不自动返航、不空中上锁、不自动重试切模式、不纠偏估计。解除只允许「重置任务状态」
（65020 `type=2`）或进程重启。**只读运行**（未注入命令接口）时只打一条 WARN、
**不锁存、不冻结**——功能内部按能力判断，不读 `enable_control`。
**冻结期间 65020 仍可达（2026-11-17 P4b 修复）**：冻结分支里照样调用
`EvaluateMissionCommand()`，所以锁存期间 65020 **逐条回 65021**——空中（`armed=true`）
被 `kRejected/kNotOnGround` 拒（悬停中不许重启飞控），**落地 + 已上锁后的重置是解除
降级锁存的唯一入口**（`ResetSortieState()` 清 `estimator_degraded_`，再按
`mission.reset_reboot_px4` 走 S1 重启或 S2 回滚），不必重启机载程序。

**重置任务状态 = 清状态 + 重启 PX4（2026-11-17 降落与估计器保护 P4b）**：65020
`type=2` 的职责从"把 S14 拉回 S2"扩为"清空本次任务状态 + 重启 PX4"。准入**只看飞控
事实**（遥测在线、未上锁、已落地证据、非任务故障），**不再要求 `state == S14`**——
旧门槛在状态机与 PX4 不同步时无路可走，只能重启机载程序。受理后发
`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(246) param1=1`，迁移 **S1 SELF_CHECK** 等飞控
重新上线；等待期间本模块**不发任何控制意图与命令**（246 重发除外），仍逐条回 65021、
周期发布 65013。`mission.reset_reboot_px4=false`（或命令接口未注入）回滚为"只清状态、
直接回 S2 READY"。**明确不做**：不修复硬件（罗盘失效/Z 轴振动重启后会复发）、不自动
重启机载程序、不在被拒/无 ACK 后无限重发、不自动解除估计器降级锁存（只清状态）；
重启不是"修好飞机"，只是把飞控与状态机拉回同一起点（设计与实施口径见
[`降落与估计器保护设计-2026-11-17.md`](../../docs/降落与估计器保护设计-2026-11-17.md) §6/§6.7）。

**人工返航指令 = 受理 + 收口两条 65021（2026-11-17 §11.6 E 项）**：地面站 65020 `type=1`
"主动返航"受理（空中任务态，或已离地的起飞阶段/已在返航）时只回 `accepted`（"已开始执行"），
真正的终点在任务收口时才报——
S14 落地上锁补 `completed`、S16 任务故障补 `rejected/mission_fault`，两条回执**同
`command_id`、同 `type`**，地面站按 `command_id` 更新同一条记录；`type=2`（重置）受理当拍
即离开进行中，没有第二条。机制与判读见「关键实现点」的「延迟终态回执」段。

**起飞前门禁（2026-11-18 H 项）**：本模块在**受理 65014 之前**校验"这架飞机现在能不能起飞"——
关键传感器（陀螺/加速度计/磁罗盘/AHRS）的 `SYS_STATUS` 健康位持续掉线 → 拒绝分配
（65015 `reason=23 kSensorUnhealthy` + 65013 **bit11**）；振动 `VIBRATION.vibration_z`
超告警阈值 → 只 WARN + 65013 **bit12**，**默认不阻断**（维护者 2026-11-18 裁决；
`preflight_gate.vibration_block_mps2>0` 才拒绝 `reason=24 kVibrationHigh`）。门禁**只在
地面待命态评估、只作用于受理 65014**，不阻断 S1→S2，也不接管飞行中的传感器退化（那是 P3
`EstimatorGuard` 的职责）。另：`400 ARM_DISARM` 被 PX4 拒绝时回 `kPreflightFailed(6)`
（不再错报 `kArmTimeout(9)`），并把 PX4 的 `STATUSTEXT` 打进日志。**明确不做**：抓不到
"未校准"（不是健康位）、没有 `SYS_STATUS` 时不做断言（跳过并 WARN 一次）、不做振动阻断的
默认开启。配置与回滚见
[`../config/配置管理.md`](../config/配置管理.md)「preflight_gate」，判据与 PX4 预检的分工见
[`../../docs/状态机设计.md`](../../docs/状态机设计.md) §6.6。

**测距雷达与单一高度出口（2026-11-18 P5 软件侧；硬件后到）**：装下视雷达后，低空多了一个
**完全不经 EKF** 的物理量（离地距离）。本模块所有"相对高度"读数统一走
`common::AboveTakeoffM()`（`include/common/types.h`）：`radar.enabled=false` 时**只用 EKF
差分**（与 P5 之前逐位一致）；`true` 时按 **雷达 AGL → 气压 AGL → EKF 差分**逐级退化，
三路都不可用则返回 `nullopt`、**绝不填 0 或 `max_distance`**。使用点是 S13a 交段判据与
保护 1/2、S13b 落地确认。另有两处门禁：`radar.preflight_required=true` 时受理 65014 前
要求雷达**在报数**（收到过 `DISTANCE_SENSOR` 且时效 ≤ `radar.stale_ms`，**不判量程**——
地面上本来就在盲区），否则回 `reason=25 kRadarUnavailable` + 65013 **bit13**；
`landing.landed_max_agl_m>0` 且 `radar.enabled=true` 时，PX4 报 `landed` 仍要求独立高度见证
≤ 该值才发 DISARM。配置与回滚见 [`../config/配置管理.md`](../config/配置管理.md)「radar」，
设计口径见 [`../../docs/状态机设计.md`](../../docs/状态机设计.md) §6.7。

## 接口与数据流

```cpp
struct MissionStateMachineConfig {
    std::chrono::milliseconds intent_valid_for{300};  // 意图有效期，与 controller.intent_timeout 同源
    config::HandoverConfig handover;                  // 交接迟滞参数（handover 配置节同源）
    control::TrackingConfig tracking;                 // 视觉跟踪控制律整节（visual_tracking 同源：
                                                      // camera 供视线角/视场换算，control/heading/
                                                      // vertical/distance 供 S7 伺服控制律）
    control::VisualProfile visual_profile = control::VisualProfile::kNetCapture;
                                                      // 拦截剖面（装配层按 component 映射：
                                                      // 25=net_capture 27=jammer 26=impact，
                                                      // impact 预留 fail-fast）
    std::chrono::milliseconds visual_stale{200};      // 视觉状态超龄（visual_tracking.control.visual_stale_ms 同源）
    std::chrono::milliseconds distance_stale{500};    // 测距超龄（distance_stale_ms 同源）
    config::ActuatorConfig actuator;                  // 捕网执行器（actuator 配置节同源，Task 10）
    config::TakeoffConfig takeoff;                    // 自动起飞时序（takeoff 配置节同源，2026-10-22）
    config::FenceConfig fence;                        // 多边形围栏（fence 配置节同源，2026-10-29）
    config::ReturnHomeConfig return_home;             // S11~S14 返航（return_home 配置节同源）
    config::LandingConfig landing;                    // S13 拆两段（landing 配置节同源，2026-11-17 P2）
    config::EstimatorGuardConfig estimator_guard;     // 估计器降级处置（estimator_guard 节同源，2026-11-17 P3）
    config::MissionConfig mission;                    // 重置动作（mission 节同源，2026-11-17 P4b）：
                                                      // reset_reboot_px4/reboot_timeout_ms/
                                                      // reboot_retry/reboot_min_interval_ms
    config::PreflightGateConfig preflight_gate;       // 起飞前门禁（preflight_gate 节同源，2026-11-18 H 项）：
                                                      // require_sensor_health/sensor_health_hold_ms/
                                                      // vibration_warn_mps2/vibration_block_mps2/
                                                      // vibration_hold_ms；构造期 Validate() fail-fast
    config::RadarConfig radar;                        // 测距雷达（radar 节同源，2026-11-18 P5）：
                                                      // enabled/preflight_required/min_signal_quality/
                                                      // stale_ms；高度出口判据由 ToGateParams() 统一生成，
                                                      // 构造期 Validate() fail-fast（含 preflight_required
                                                      // 必须搭配 enabled 的组合校验）
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

    // ---- 降级/PX4 重启可观测出口（2026-11-17 P3/P4b）----
    /// 降级是否已锁存（测试/SITL 断言用；非虚，不扩张 IMissionStateMachine）。
    bool EstimatorDegraded() const;
    /// PX4 重启等待是否进行中（测试/SITL 断言用；非虚，不扩张 IMissionStateMachine）。
    /// 65020 `type=2` 受理后在 S1 SELF_CHECK 等飞控回来期间为 true。
    bool Px4RebootPending() const;

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
`handover` 的关系校验（`exit_lost_frames > enter_s6_lock_frames` 等）由配置加载期
`HandoverConfig::Validate`强制，状态机构造不重复校验（与 FlightController 同约定）。
起飞前门禁（2026-11-18 H 项）走**整节注入**：`config_.preflight_gate` →
`MissionStateMachineConfig::preflight_gate`（`config::PreflightGateConfig`），构造期调
`PreflightGateConfig::Validate()` 兜底（与 landing/estimator_guard/mission 同一先例：
程序化构造的非法值 fail-fast）。
测距雷达（2026-11-18 P5）同样**整节注入**：`config_.radar` →
`MissionStateMachineConfig::radar`（`config::RadarConfig`），构造期调
`RadarConfig::Validate()` 兜底。高度出口的判据参数一律由 `radar_.ToGateParams()` 生成，
**不在 S13a/S13b/预检门禁各写一套阈值**；`radar_.enabled=false`（硬件未到位时的默认）
让高度出口行为与 P5 之前逐位一致。

**接线现状**：`DroneApplication` **已接线**——估计器/视觉稳定性/双目测距三者同时装配时
调用`SetEstimationInputs`（缺一不绑定，状态机不发布 S5 意图）；估计器存在时注入
`SetEstimationDiagnostics`。T12 当时"生产接线与 `enable_control` 打开一并做"的约定
已随 2026-10-30 控制运行落地。裁剪装配（如视频探针）关掉上述组件时仍会出现
"不出 S5 意图"，属预期；G3 全链验收另走独立装配工具
（`tools/px4_link_smoke --sitl-stage4-mission`）。
`SetEstimationDiagnostics`已在装配估计器时注入；
`SetCommandSender`已在控制运行（`enable_control=true`）的装配门禁分支注入（T6），
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
    得出 `kNormal / kNearBoundary（距离 ≤ boundary_warn_m）/ kViolated`，
    越界持续到 `violation_debounce_ms` 才动作，避免单帧抖动触发中止链。
  - **越界深度容差 + 回界滞回（2026-11-12）**：水平越界不再用零容差。判据改为
    「飞机自身位置越出边界超过 `violation_depth_m`（默认 1.5 m）才算越界；
    已判越界后要退到 `−violation_depth_m + violation_hysteresis_m`（默认 −1.0 m）
    才清除」（实现为 `fence_depth_violated_` 迟滞状态）。
    **滞回落在容差带内、不在边界内侧**：若把清除阈值放到边界内侧（如 +0.5 m），
    飞机停在边界外但仍在容差内的位置（如 −0.9 m）会一直判越界并持续累计时长，
    防抖凑满后照样中止——那等于容差没生效。
    根因：0120 现场日志显示目标贴边时飞机被位置环顶出边界 1~3 m 又被拉回，
    零容差把一次 1.4 m 的瞬时穿透判成越界，防抖凑满 1.5 s 后直接中止返航
    （操作员描述"波动两下就返航"）。两个参数都为 0 时退化为历史零容差语义。
    65019 的 `distance_to_boundary_m` 仍如实上报带符号距离（内正外负），
    只是不再据此动作。**65018 区域内容变化（含同 id 新 version）时必须复位
    `fence_depth_violated_`**：否则新区域会沿用旧回界阈值、把本在容差内的点
    持续判越界（2026-11-12 复审；用例 `FenceViolationClearNeedsHysteresis` 覆盖）。
  - 高度带语义（2026-10-30 现场日志 0078 修正，**2026-11-04 换基准**）：`altitude_reference=1`
    （相对起飞点，原"相对 Home"）的高度由 **EKF 差分**给出——
    `相对高度基准 − local_z_m`，基准统一取 `common::RelativeAltitudeBaseZ()`：
    **优先 `HOME_POSITION.z`**（PX4 在 EKF 漂移超 `2×EPH/2×EPV` 时重设 Home，基准随
    局部坐标系一起平移，对 EKF 高度重置免疫、与 PX4 任务相对高度同源），
    `HOME_POSITION` 未到达时退化到解锁上升沿抓取的 `takeoff_ref_local_z_m`
    （见 `include/communication/px4_link.md`）；地面恒 0，不吃 GNSS 海拔漂移；
    该基准**一律不判下限**（有效下限恒 0）——高度从地面起算，起飞必然穿过 0→设定值，而
    S3/S4 在围栏动作生效态、防抖只有 `violation_debounce_ms`（1.5s），旧实现
    （`alt_min=5m`）会让地面静止就被判 `kViolated`、解锁后立刻中止返航；地面站表单
    对该基准把最低高度固定 0 并置灰。`altitude_reference=0`（AMSL）保留绝对海拔下限
    （跨海拔场地仍有意义），但 S0~S4（未起飞/爬升）同样不判下限，S5 起才按绝对海拔
    生效。实现为 `FenceEffectiveAltMin(zone, CurrentState())`，`alt_max≤0` 仍表示不限。
  - 高度基准不可用（2026-11-04）：`AltitudeForFence()` 返回 `std::optional<double>`——
    无 `local_position_valid`、相对参考无起飞基准、AMSL 无 `ned_ref_valid` 时为空，
    该拍**只判水平**、不判高度（旧实现退回 `−local_z`，地面约 +39 m，会把地面判成越界）。
  - 高度迟滞（2026-11-04）：`kFenceAltitudeHysteresisM = 1.5`——进入越界用原始阈值，
    从越界恢复要多退 1.5 m（`fence_altitude_ok_` 记忆上一拍结论），避免在阈值附近
    抖动刷出一串 65019 `state=2`。
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
- **人工任务指令（2026-10-29，65020/65021；2026-11-04 增 type=2 重置状态；2026-11-17 P4b
  改为"清状态 + 重启 PX4"）**：地面站可下发"主动返航"打断当前任务，或"重置任务状态"
  清空本次任务并重启 PX4（落地且未上锁即可，不再要求 S14）。
  - 输入：`SetMissionCommandInput()` 绑定 `kMissionCommand`（Start 前）；`DrainInputs()`
    每拍只保留最新一条（`latest_mission_command_`），`valid_for_ms=1000` 由链路层写入，
    过期指令由状态机侧按时钟判定丢弃。
  - 判定顺序（`EvaluateMissionCommand()`，排在电池守卫之后、围栏守卫之前——人工指令
    优先于几何事件）：重复 `command_id` → `kDuplicateCommand`（不动作、不回退状态）；
    幂等槽位 `last_mission_command_id_` 只在**真正执行动作**的分支登记
    （`RecordExecutedMissionCommandId()`：`kArming/kTakeoff` 中止、飞行态中止返航），
    被版本/类型/状态守卫拒绝或「已在返航」早退的指令不占槽位——地面站修正后可用同一
    `command_id` 重发（2026-10-30 修复：旧实现在校验前登记，被拒 id 再发被判重复）；
    版本不符 → `kUnsupportedVersion`；`type=2 kResetState` 在版本校验后立即分派给
    `ExecuteResetStateCommand()`（不走返航判定）；其余非 `kReturnHome` → `kUnsupportedCommand`；
    `mission_faulted_` → `kMissionFault`；已在 `kReturnHome/kLanding` → `kCompleted`
    + `kAlreadyReturning`（幂等，不重复发 65015）；`kArming/kTakeoff` →
    `AbortTakeoff(kCancelledByGcs)`；其余飞行态 → 复用内部中止入口。
  - 接受路径：`mission_abort_latched_=true`、`StopPositionIntent()`、`ResetTargetHold()`、
    `assignment_tracker_.AbortAirborne()`、`RememberTakeoffAbort(kCancelledByGcs)`、
    `TransitionTo(kReturnHome,"地面站主动返航指令，中止任务并返航")`，并发 65015
    `aborted reason=kCancelledByGcs(12)`。**不是 PX4 RTL**：返航轨迹仍由 S12 位置意图控制。
  - **延迟终态回执（2026-11-17 §11.6 E 项）**：`type=1` 受理当拍只能回"已开始执行"，
    终态必须等任务真正收口，因此把这条 65020 原样登记在 `Impl::deferred_return_ack_`，
    收口时补发**第二条 65021**（同 `command_id`、同 `type`）：
    - **登记点三处**（都先 `RecordExecutedMissionCommandId()`）：`kReturnHome` 已在返航
      （"继续返航"）、`kArming/kTakeoff` 起飞中止且已离地（`AbortTakeoff` 后状态确为
      S12）、`kGpsApproach`~`kPostIntercept` 空中态中止进 S12。`kLanding` 分支当拍回
      `kCompleted/kAlreadyReturning` 直接终态，不登记。
    - **收口点两处**（都在 `TransitionTo()` 内、`state_` 已改成 `next_state` 之后）：
      落到 **S14 `kDisarmed`** → `kCompleted/kNone`（`mission_state=14`）；落到
      **S16 `kFailsafe`** → `kRejected/kMissionFault(7)`（`mission_state=16`）。字段由
      `PublishMissionCommandAck()` 组装，`type`/`command_id` 逐字复制受理那条，只有
      `result/reason/mission_state` 变化——地面站按 `command_id` 更新既有记录，不是新增
      一条指令。
    - **清空**：`ResetSortieState()`（进程启动与 65020 `type=2` 重置共用）清空登记，避免
      上一轮的 `command_id` 在下一轮 S14 被"补"出来；补发后立即清空，同一轮只发一次。
    - **为什么不算重复指令**：去重只针对**新收到的 65020**（`last_mission_command_id_`），
      补发走 `PublishMissionCommandAck()` 直接出版，不重入 `EvaluateMissionCommand()`；
      受理那条已占幂等槽位，同一 `command_id` 再下发仍回 `kDuplicateCommand`。
      `type=2` 重置没有第二条（受理当拍已离开进行中，`ResetSortieState()` 也把登记清空）。
  - **`type=2` 重置任务状态 = 清状态 + 重启 PX4（2026-11-04 引入；2026-11-17 P4b 改语义）**：
    `ExecuteResetStateCommand()` 的准入**只看飞控事实**（`state` 编号只用于幂等与日志，
    不再当门槛）。判据按下列顺序逐条短路：

    | 顺序 | 判据 | 不过时的回执 | 说明 |
    |---|---|---|---|
    | ① | `mission_faulted_` | `kRejected/kMissionFault` | 故障态先人工处置 |
    | ② | 当前状态 ∈ {S1 SELF_CHECK, S2 READY, S0 BOOT} | `kCompleted/kNone` | 幂等：不重复清状态，**也不重启飞控** |
    | ③ | 无飞行快照或 `connected == false` | `kRejected/kTelemetryStale(9)` | 没有遥测就判不了"能不能安全重启飞控" |
    | ④ | `armed == true` | `kRejected/kNotOnGround(8)` | 解锁态重启飞控 = 空中断控制 |
    | ⑤ | 未确认落地：`landed_state_valid` 时看 `landed`；否则退化为"相对高度 ≤ 0.5 m"（`RelativeAltitudeAgl()`）；两者都不可用同样拒绝 | `kRejected/kNotOnGround(8)` | "不知道高度" ≠ "在地面"，不得用 0 代高度 |
    | ⑥ | tracker 仍认为在空中（`ConsumedAirborneMission() && Phase() != kTerminal`） | —— | **只 WARN 不拒绝**：`ResetSortieState()` 会一并清空 tracker，拒绝反而把人卡死 |
    | ⑦ | 距上次重启 < `mission.reboot_min_interval`（默认 30 s） | `kRejected/kDuplicateCommand` | 刚重启的飞控不许被再打一遍 |

    准入通过 → `RecordExecutedMissionCommandId()` → `ResetSortieState()`（与进程启动共用
    同一份复位清单：任务分配记忆、起飞/返航/降落/围栏/目标保持、估计器降级锁存、
    各类锁存与计次）→ 默认置 `px4_reboot_pending_=true` +
    `TransitionTo(kSelfCheck,"地面站重置任务状态，重启 PX4")` + 受理当拍发一次 246 →
    **最后**才回 `kAccepted/kNone`（此时 `ack.mission_state == S1 SELF_CHECK`，地面站据此
    显示"自检/重启中"；仍在 Web 的 3 s 超时内。回执放在迁移之后是 P4b 的落地口径，
    设计稿 §6.2 原写"先回执再动作"）。
    246 参数：`param1=1`（重启飞控）、`param2~7=0`。编号与语义已核对：
    `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246`（`third_party/c_library_v2-master/common/common.xml`）；
    PX4 `Commander.cpp` 的该分支只读 `param1`（`param1==1 && !isArmed()` 才 ACCEPTED，
    否则 DENIED），**`param6` 的"安全互锁/强制"在当前固件没有生效**——落地互锁完全由
    本模块的准入（④⑤）负责。
    回滚路径：`mission.reset_reboot_px4=false` **或命令接口未注入**（`command_sender_ == nullptr`）
    → 只清状态、`TransitionTo(kReady,...)`、不发 246、回 `kAccepted/kNone`
    （`ack.mission_state == S2 READY`）；此时仍记 `last_px4_reboot_ms_` 占住最小间隔窗口。
  - **等待 PX4 回来（`Px4RebootPending() == true`）**：`Evaluate()` 最前面**早返回**——
    不发任何控制意图/命令（246 重发除外）、不推进任何任务逻辑，只做三件事：
    ① 246 重发：首次已由受理当拍发出，之后每 1 s 一次，**总尝试次数 ≤
    `mission.reboot_retry`（默认 3，含首次；入队失败同样计数）**；收到 ACK `result=0`
    立即停止重发；ACK `result ∈ {2,3,4}`（DENIED/UNSUPPORTED/FAILED）→ 停止重发、
    下一决策拍**立刻回退 S2 READY** 并 WARN（原因多为固件未编板级 reset 能力，
    **SITL 恒为 DENIED**），同时清掉最小间隔计时（飞控没真重启，不该占窗口）；
    `result=1`（TEMPORARILY_REJECTED）按"未确认"继续重试；
    ② 恢复判据：**必须先观察到一次断连**（`connected == false`）之后**再**出现
    `connected && HasNavigationForGps()`（心跳 + 3D fix + 全局位置 + Home）→
    `px4_reboot_pending_=false`、INFO「PX4 重启完成…」，下一拍走常规 S1→S2 判据。
    先要求断连是为了挡住"PX4 只 ACK 但实际没重启"时把旧心跳当成已恢复；
    ③ 超时 `mission.reboot_timeout`（默认 90 s，`Validate` 允许 100 ms~300 s）到点 →
    ERROR **一次** + 65013 **bit10**（`kWarningPx4RebootTimeout = 1U << 10U = 1024`，
    `PX4 重启超时`），**保持 S1**，不自动重启机载程序。
    等待期间**仍逐条回 65021**（`EvaluateMissionCommand()` 在早退分支里照样调用：
    重置类走 ② 的 `kCompleted`，返航类走"未起飞"`kNotAirborne`，都不发命令），
    并周期发布 65013（bit10 置位后一直带到回到 S2 为止）。
  - **S16 故障态也处理 65020（2026-11-17 P4b 行为修复）**：故障态的两个早退分支里
    补上了 `EvaluateMissionCommand()`。此前故障态早退导致 65021 根本不发，
    `kMissionFault` 这条 reason 不可达；而 P4b 之后重置是唯一的恢复入口，不能失联。
    现在故障态收到重置/返航都如实回 `kRejected/kMissionFault`。
  - 动机与边界：旧实现要求 `state == S14`，落地后状态机/PX4 不同步（现场常卡在
    S13/S12）时无路可走、只能重启机载程序；P4b 改看飞控事实后，"落地 + 未上锁 +
    遥测在线 + 非故障"即可重置。**重启治标不治本**——罗盘失效、Z 轴振动等硬件问题
    重启后会复发，这是能力边界，文档与界面文案都写明。
  - 回执：`PublishMissionCommandAck()` 走 `kMissionCommandAck`，字段含 result/reason/
    `mission_state`（回执时刻状态），地面站无需猜测机载处于哪一态；`header.sequence`
    每条回执只自增一次（2026-10-30 审查修复：旧实现同一条回执自增两次、序号跳 2，
    地面站按序号做去重/丢包统计会误判）。
  - **65015 的既有门槛（2026-10-28 复核）**：`PublishTakeoffAbortAck()` 首行是
    `if (!accepted_assignment_.has_value()) return;`，因此只有在先接受过 65014 分配的
    任务里，中止才会同步 65015；未分配/只读运行场景（含 SITL `--sitl-stage4-command`）只看
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
  即进入`GPS_APPROACH`（既有语义，保留）。
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
  7. `command_sender_ != nullptr`，否则 `control_disabled`（只读运行）；
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
  - 生效阶段：S3/S4 与 S5~S10；S12/S13/S14/S16 不再触发；只读运行
    （`command_sender_ == nullptr`）时不生效（S5 既有验证语义保留）。
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
  - S4 爬升：仍发 `kTakeoff` 位置意图（z 指令按 `climb_rate` 逐拍抬升）；
    远段（`|实际 z - 目标| > max(0.20, z_tolerance_m)`）附带垂直速度前馈
    `feedforward_vz_mps = -climb_rate`、mask 含 `kVelocityValidDown`；末段
    改盯目标高度并撤前馈。到达判据看**实际** z/xy/vz（不是指令值），
    保持 `stable_time` 后 `TakeoffComplete()+BeginEngage()` → S5。
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
- 估计/视觉/测距三输入必须成组绑定；只绑部分时记 ERROR 并按未绑定处理（不发布 S5 意图）。
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
    与`VisualTargetMonitor`内部的`acquire_lock_frames`判据是两套东西，不得混用：
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
    姿态取`FlightStateSnapshot`真实值（**正式路径不做虚拟姿态兜底**——观测
    组件的`virtual_attitude_when_absent`属观察开关，不进入正式装配）。
  - 意图坐标系：速度分量已是**局部 NED**（机体系→NED 旋转在视觉控制律
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
- **通用拦截设备（Task 10 / Task 5，2026-10-21；2026-11-11 泛化）**（`UpdateInterceptionDevice` /
  `MaybeExecuteInterception` / `HandleMissionFault` / `IInterceptionDevice`）：
  - 每拍顺序：`Evaluate` 开头先 `UpdateInterceptionDevice()`（组装 `InterceptFeedback`：
    PX4 连接证据取最新飞行快照、ACK 快照取 `IPx4CommandSender::AckStatus()`），
    故障则整拍只维护告警/发布状态并返回；随后 `EvaluateInterceptSequence` →
    `MaybeExecuteInterception` → 其余控制意图评估。
  - 触发语义：S10 内且执行器未 `Triggered()` 时，至少满足 `Ready()` 与
    `RemainingUses()>0` 才调 `Execute()`；**无飞行快照不触发**（没有连接证据），
    快照存在但 `connected=false` → 故障；入队失败保持 Ready、下拍重试，
    超过 `return_home.fire_timeout` → 故障。S9 只标记序列开始，不触发。
  - 完成语义：`EvaluateInterceptionCompletion` 以 `Triggered()`（触发命令已入队）
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
  - 门禁：`intercept_device_==nullptr`（只读运行或未装配执行器）→ 零调用，
    本功能物理缺席；触发/复位同为控制输出，不另开旁路。
  - 进程退出不补发复位行程（`Stop()` 不触发机构动作）：机构实际位置由
    G4 台架/人工确认；拦截执行段内停机属异常场景，PX4 侧输出保持最后值。
- **S9/S10 入口判据（2026-10-19）**：`EvaluateInterceptSequence` 在 S7 每拍评估
  `InterceptEntrySatisfied`——`interception.enabled`、视觉新鲜且 `LOCKED`、
  `consecutive_lock_frames_ >= enter_s6_lock_frames`、测距 `valid`+新鲜且
  `[min_range_m,max_range_m]`（**上限 2026-10-23 冻结 = 双目测距极限 8m**，
  下限 2.0 仍占位）、`connected+armed+Offboard(6/0)`、姿态/局部位置有效、
  `intercept_device_` 非空且未触发、`Ready()`、未故障、任务未进入 S16；可选
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
  - **完成判据**：S10 内执行器 `Triggered()`（触发命令成功入队，不等 ACK）→ S11；
    **V1 不判断命中**，随后 S11 当拍/下一拍进入 S12。发射入队超时/断连不再走
    “失败返航”，而是 `HandleMissionFault()` 进入 S16。摄像头状态不参与返航判断。
  - **返航水平目标（2026-11-04 现场回归）**：优先用 S3 接受分配时记录的**起飞原点**
    （`RememberTakeoffOrigin()` 存 `local_x/y/z`，与 `local_position` 同一坐标系），
    不再默认用 Home 经纬度换算——PX4 Home 经纬度随 GPS 漂（现场 25 分钟水平漂
    5~16 m），用它换算会让返航点偏离真实起飞点，表现为“飞到起飞点附近又拐个弯、
    最后落到坡上”。只有无起飞原点（如 65020 人工返航且未走任务分配）时才退回
    `HomeLocalNed()`。`HomeLocalNed()` 用 Home 经纬度（`home_*`）+ EKF 锚点
    （`ned_ref_*`）经 `perception::GeodeticToLocalNed` 换算；不依赖
    `HOME_POSITION.x/y/z`（MAVLink 无 `valid_lpos` 标志）。两者都不可用 →
    原地悬停并告警（不盲目返航）。
  - **S12 返航**：相对高度基准（`common::RelativeAltitudeBaseZ()`，与围栏/下传同口径）
    下低于**返航高度**先原地爬升，达标后再水平飞返航目标；高于则保持当前高度。
    返航高度由 `ReturnAltitudeM()` 给出：`return_home.altitude_m`（5m）起步，若区域
    `altitude_reference != 0`（相对起飞点）且 `alt_max_m > 0`，收敛为
    `min(5.0, alt_max_m − 0.5)`（2026-11-04 现场回归：区域上限 20m 时无影响，但区域上限
    压到 5m 以下时，飞控会把设定值钳在区域内 → 垂直误差永远超 `arrival_vertical_m`、
    S12 永远等不到"到达"），首次被压低时打 WARN
    `返航高度受区域高度带上限约束: 5.0m -> ..m`。基准不可用时退回
    `HomeLocalNed().down_m`，仍无则悬停告警。位置意图 10Hz 重发，
    `reason_code` 低位=8
    （`kReasonReturnHome`），不带速度前馈。**爬升与水平段都带相位速度上限**
    （2026-11-03）：爬升段 `max_speed_mps=climb_rate_mps`（默认 0.5）；水平段
    **由返航速度剖面逐拍给出**（2026-11-12，见下条），由控制器“位置引导限速”
    逐拍推进——此前不带上限，位置目标从当前高度阶跃到 5m，PX4 按自身
    `MPC_Z_VEL_MAX_UP`（3m/s）冲上去，实机反馈“低于 5m 时上升很快”。PX4 离开
    Offboard（failsafe/人工）→ 立即停发、转 S13 仅监控，不与其争控。
  - **返航速度剖面（2026-11-12，`control::return_speed_law`）**：水平段的
    `max_speed_mps` 不再是固定 `cruise_speed_mps`，而是纯函数
    `ComputeReturnSpeed()` 按**分段恒定加速度**给出的本拍速率上限。
    巡航顶速 `cruise_speed_mps` 定为 **4.0 m/s**，**独立于追踪速度**
    （`controller.max_speed_mps=10`）——返航是安全收口，顶速不需要跟追踪同量级。
    剖面覆盖三种必须区分的现场情形：
    ① **起步加速**：目标未运动/已掉落，飞机被追停到 0 才进返航 → 从实测 0 按
    `accel_plan_mps2`(1.5) 斜坡升到 4；
    ② **超速减速**：始终追不上目标而中转返航，飞机速度可能高于 4 → 按
    `decel_plan_mps2`(2.0) 斜坡先降回 4（指令允许暂时高于 cruise，否则斜坡失效）；
    ③ **末端制动**：`v ≤ √(2·(decel/brake_safety)·(d−arrival_radius))`，
    进入 `final_zone_m`(10m) 后顶速降为 `final_speed_mps`(1.0)。
    **不做 jerk**（不变加速度）、**不做任何"追不上就加速"补偿**——返航目标是固定点，
    永久可达；停滞只按上一条的 WARN 处理。起点用**实测地速**而非上一拍指令：
    否则起步段剖面会"以为自己已在巡航"、永远给不出加速段。
    进入 S12 时 `ResetReturnSpeedState()` 复位（本轮从当前实测速度重新起步）。
    **输出下限 0.1 m/s**（2026-11-12 复审）：`max_speed_mps=0` 在控制器语义里
    是"未填相位上限 → 回退硬上限 10 m/s"，末端包络收敛到 0 时若原样下发会把
    最后几米变成不限速，必须抬到小正数；停车由到达判据 → S13 负责。
    地速输入逐轴做有限性降级（坏轴按 0），避免 NaN 被当成"实测 0"重新起步。
    完整口径见 `include/control/return_speed_law.md`。
  - **无返航基准/无高度基准的悬停意图也限速**（2026-11-12）：`ReturnHorizontalTarget()`
    或相对高度基准不可用时下发的是"当前位置"悬停意图，以前 `max_speed_mps`
    传 0（控制器语义 = 回退硬上限 10 m/s）。虽然目标=当前位置本身不推进，但
    一旦分支行为变动就会按 10 m/s 跑；现统一改传 `cruise_speed_mps`。
  - **到达判据**：水平距返航目标（起飞原点或 Home）≤`arrival_radius_m`（2m）、
    垂直误差 ≤`arrival_vertical_m`（0.5m）、水平速度 ≤`arrival_speed_mps`（0.5m/s）
    持续 `arrival_hold`（1s）→ S13。
  - **不设总时长上限（2026-11-04 改）**：原 `return_home.timeout`（60s）是"从进入 S12
    起的绝对时长"，与距离和限速无关——现场 128m 返航在 1.5m/s 限速下需 ≈85s，60s 必超，
    超时后只把"当前位置"当目标原地悬停，等于任务卡死。现在 `timeout` 只作**停滞告警间隔**：
    返航距离在 `kReturnStallProgressM`（0.3m）内无改善持续该时长才打 WARN
    `返航停滞: ..s 未靠近返航目标（距目标 ..m，检查逆风/区域钳制/遥控接管），继续返航`，
    **只告警、不中止**。真正的兜底交给 PX4（失联 `COM_OF_LOSS_T`、低电动作）。
  - **进入 S12 时已落地/上锁**（`landed && !armed`，例如 S5 无目标停发后
    PX4 自行 RTL 完成、模式又恢复 Offboard）：不再发返航位置意图，直接转
    S13 → S14 收口（SITL 2026-10-22 实测：否则会停在 S12 空转）。
  - **S12 内再收 65020 `type=1`**：视为"继续返航"，重置停滞计时与最佳距离，
    回 65021 `kAccepted/kNone`（日志 `任务指令（主动返航）: 已在返航，按继续返航处理`）；
    S13 仍回 `kCompleted/kAlreadyReturning`。地面站「主动返航」按钮本身只在
    `mission.state ∈ [3,11]` 可点，这条路径主要给脚本/命令行重发用。
  - **S13 拆两段（2026-11-17，降落与估计器保护 P2；设计
    [`降落与估计器保护设计-2026-11-17.md`](../../docs/降落与估计器保护设计-2026-11-17.md) §4）**：
    旧流程 S13 发一次 `AUTO.LAND` 后即放手，27m 全程无人监督；2026-11-17 10:24 现场
    EKF 全状态重置把 `local z` 平移 9m，PX4 以为已触地而砍推力 → 翻滚。现在把最后一段
    高度留在我们手里，**不新增状态编号**（仍是 `kLanding`），内部用两个标志分段：

    | 段 | 条件 | 动作 |
    |---|---|---|
    | **S13a** 机载下降 | `landing_descent_phase_ == true` | 每拍发位置引导意图（低位原因码 **12**）：目标 = Home 水平 + 相对高度 `landing.descend_target_ag_m`(1.2m)，推进速度 `descend_speed_mps`(0.8)/低空档 `descend_slow_speed_mps`(0.3)；跑三条保护 + 超时 + 交段判据 |
    | **S13b** 交 PX4 | `landing.enabled && !descent_phase_` | 发 `DO_SET_MODE(AUTO.LAND 4/6)`（按 `return_home.command_retry` 重试）；**确认前继续保持控制权**（按当前高度发布保持意图，不推进下降目标）；确认（ACK `result=0` 或观察到 PX4 已进 4/6）后停止发布 intent，只监视 `landed/armed` |
    | 回滚路径 | `landing.enabled == false` | 完全保持 2026-11 之前的行为：直接发 `AUTO.LAND`、不发布任何降落位置意图；**只读运行**（未注入命令接口）在建 S13b 前就返回，同样什么都不发 |

    - **启用判据**（进入 `kLanding` 时在 `TransitionTo` 里算一次）：
      `landing_descent_phase_ = landing.enabled && command_sender_ != nullptr &&
      !landing_takeover_`。命令接口是否注入就是"能不能发控制"的唯一事实依据
      （生产只在 `enable_control=true` 时注入），**不另设装配门禁字段**，也不在
      状态机内重复判断 `enable_control`。
    - **几何口径**（2026-11-18 P5 起改为**单一高度出口**）：决策用高度 =
      `common::AboveTakeoffM()`（`include/common/types.h`）——`radar.enabled=false` 时等价于
      旧的 EKF 差分（`RelativeAltitudeBaseZ() − local_z`，基准 HOME_POSITION.z 优先、退化到
      起飞快照、再退化到 `HomeLocalNed().down_m`，**与 P5 之前逐位一致**），
      `=true` 时为 **雷达 AGL → 气压 AGL → EKF 差分**逐级退化；三条都不可用则本函数返回
      `nullopt`、按保护 3 计数（**绝不用 0 代替**）。`ComputeLandingGeometry()` 额外把
      EKF 差分留一份（`ekf_relative_alt_m`）供交叉校验，并记下本次用的 `HeightSource` 与
      雷达未被采用的原因（`RadarRejectReason`）。**口径差异**：雷达是"离地"，气压/EKF 是
      "离起飞点"，地形起伏处本来就有差（交叉校验门限要给地形差留余量）。
    - **交段判据**：相对高度 ≤ `descend_target_ag_m + handover_alt_tolerance_m`(0.15)
      且 `|vz| ≤ handover_vz_mps`(0.2) 且 `水平误差 ≤ handover_horizontal_m`(0.5)，
      连续保持 `handover_hold`(500ms)。判据里的"相对高度"就是上面那个**决策高度**。
    - **三条保护（缺一不可，设计 §4.5）**：
      ① **高度源交叉校验**：**对照源随"正在用的高度源"变**（2026-11-18 P5）——用雷达时
      对照气压 AGL（同为"离地"口径，最贴近同一物理量；气压不可用则退 EKF 差分），
      用气压时对照 EKF 差分（专抓 EKF 高度重置），用 EKF（含 `radar.enabled=false` 的旧
      路径）时对照气压 AGL（**P2 原逻辑，必须逐位保留**）。两者之差 > `baro_mismatch_m`(1.5m)
      持续 `baro_mismatch_hold`(2s) → 锁存 + WARN + 立即交 S13b（**不硬降到 1.2m**）；
      没有可用对照源时跳过（不冒充证据）；
      ② **低空水平漂移**：相对高度 ≤ `descend_slow_zone_m`(3m) 且（水平误差 > 1.0m 或地速
      > 0.5m/s）→ 立即交 S13b（宁可让 PX4 落，也不在 1m 高度画圈）；这两个门限是冻结值，
      不进配置（`kLandingLowAltitudeDriftM` / `kLandingLowAltitudeDriftSpeedMps`）；
      ③ **控制权保持**：每拍必须能发布可用意图。`FlightController` 没有回执通道，
      因此把"被拒"落到本模块**能看见**的不可用条件上：无局部位置 / 无高度基准 /
      **单一高度出口三条都不可用** / 无 Home 水平目标 / Home 落在围栏外（控制器会
      `ClampIntoPolygon` 把落点带偏）。连续 `kLandingIntentUnusableFrames`(10 拍 = 1s)
      不可用 → 交 S13b，途中按 `ShouldLogThrottled` 记累计计数。
    - **落地确认门禁（2026-11-18 P5，仅 `radar.enabled=true` 生效）**：PX4 报 `landed` 且
      **仍上锁**时，要求单一高度出口的离地高度 ≤ `landing.landed_max_agl_m`（默认 1.0m，
      0=关闭）才认账并发 DISARM；矛盾持续超过 `landing.landed_agl_timeout_ms`（默认 3000ms）
      则**按 PX4 处置**继续 DISARM 并打一条 ERROR（避免雷达卡死时任务永远收不了尾）。
      **能力边界**：这条只约束"我们是否认账并上锁"，**拦不住** PX4 在 `AUTO.LAND` 里自己的
      land detector 决策（它判已落地就砍推力）。**为什么只在 `radar.enabled=true` 时生效**：
      P5 之前没有独立高度见证，拿 EKF 自己当见证等于自证。
    - **兜底**：`landing.descent_timeout`(120s) 到点即交 S13b（27m 按 0.8m/s 约 33s）；
      控制权被抢（PX4 离开 Offboard 且非 AUTO.LAND）沿用 `landing_takeover_ = true`，
      只等落地、不抢回控制权。
    - **围栏前置判据（设计 §4.6）**：S13a **首拍**（决策线程，不是在状态迁移里——迁移可能
      由测试钩子从其它线程触发，读飞行快照不安全）判 Home 水平目标是否在多边形内；
      不在界内（或水平目标不可用）→ WARN + 跳过 S13a 直接 S13b。
    - **`SetCommandSender` 消费点新增**：S13a 不新增命令，只是把"发 AUTO.LAND 的时机"
      从"进入 S13 立刻"改为"交段判据满足/保护命中/超时"。
  - **S13/S14 收口**：落地后若仍 armed 发 `MAV_CMD_COMPONENT_ARM_DISARM`（0）；确认
    disarmed → S14。正常返航不使用 PX4 自动 RTL；PX4 已接管则只监控 landed/disarmed。
  - **配置**：`MissionStateMachineConfig::landing`（`config::LandingConfig`，`landing` 节，
    14 个 key，其中 `landed_max_agl_m`/`landed_agl_timeout_ms` 为 P5 落地确认门禁用；
    `enabled=false` 是回滚开关），构造期 `LandingConfig::Validate()` 兜底。
  - **配置**：`MissionStateMachineConfig::return_home`（`config::ReturnHomeConfig`，
    默认 `enabled=true`；`interception`/`return_home` 两节已接入 JSON 加载）。
    捕网执行器隔离用例显式 `enabled=false`。
  - **依赖**：状态机为 Home 换算链接 `drone_perception`（纯函数 `GeodeticToLocalNed`）。
  - **边界**：S9/S10 入口已配置化（`interception` 节，默认关闭；射程上限 2026-10-23
    冻结为双目测距极限 8m，下限仍占位）；
    V1 返航段可通过测试钩子/测试用例或显式开启配置驱动。多发命中判据/再交战策略
    V2 立项；`MissionStatus` 结果字段暂不新增（用状态+日志）。
- **估计器降级处置（2026-11-17，降落与估计器保护 P3）**：`EstimatorGuard` 从"看得见
  但不动作"的观测层接进决策链。命中即**切 POSCTL 悬停 + 停发一切控制意图 + 冻结任务
  推进**，把飞机交给飞手（设计 §5，实施口径以 §5.6 为准）：

  | 项 | 实现事实 |
  |---|---|
  | 接线点 | `Evaluate()` **首行**调 `EvaluateEstimatorGuard(now_ms)`（在拦截设备推进 `UpdateInterceptionDevice()` 之前），每决策拍（100 ms）取最新 `FlightStateSnapshot` 调一次 `EstimatorGuard::Update(state, now_ms, state.armed)`；无快照时直接跳过。`EstimatorGuard` 无内部同步，**只在决策线程调用** |
  | 判据与锁存 | 8 类判据任一命中 → `estimator_degraded_`（`std::atomic<bool>`）**锁存**；判据模块内部同样锁存，不因后续帧好转自动恢复。清除只有两条：65020 `type=2`「重置任务状态」经 `ResetSortieState()`，或进程重启（`Start()` 走同一份清单） |
  | 切 POSCTL（一次性） | 已注入命令接口时发 `MAV_CMD_DO_SET_MODE(176)`：`param1=1`（custom mode enabled）/`param2=3`（PX4 主模式 POSCTL）/`param3=0`，其余 0；**入队成功后永不再发**，全程最多一帧。仅当 `SendCommand` 返回 false（命令没离开本机：链路未就绪/队列满）时按 `return_home.command_retry`（默认 1000 ms）重试，**最多 3 次**；3 次都失败打一条 ERROR 交飞手 |
  | 早返回冻结分支 | 锁存后 `Evaluate()` 在 `UpdateInterceptionDevice()` 之后**早返回**：跳过分配入口/自动起飞时序/电池守卫/任务指令/围栏守卫/目标看门狗/交接等待/引导越界告警/交接/S9-S10 入口与拦截执行/位置与视觉伺服意图/围栏与出区保持意图/热备份/S11~S14 返航降落整链——即**不发任何控制意图、不推进任何状态**；只保留告警位维护（仅 `kWarningNavigationUnavailable`）与 `MissionStatus` 周期发布。拦截设备**自身**时序（ACK/保持/复位）仍按拍推进，被跳过的是拦截**决策** |
  | 唯一允许的迁移 | `kLanding` 且 `landed_state_valid && landed && !armed` → S14，日志 `任务结束: S14 DISARMED（估计器降级期间收口）`。**不发 DISARM**：判据前提就是位置/高度不可信，`landed` 也是用坏估计判出来的，空中上锁等于摔机；落地后由 PX4 `COM_DISARM_LAND` 或操作员手动上锁，机载只认"已上锁"这个事实 |
  | 与 P2 保护 1 的优先级 | S13a 期间 baro 与 EKF 高度差超限会同时命中 P2 保护 1（立即交 AUTO.LAND）与 P3 判据；P3 在 `Evaluate()` 更早位置生效，**实际行为是冻结**（不交 AUTO.LAND）。要恢复 P2 处置只能关 `estimator_guard.enabled` |
  | 能力判断而非装配门禁 | 未注入命令接口（只读运行）：只打一条 WARN，**不锁存、不冻结**；功能内部不读 `enable_control`，只看 `command_sender_ == nullptr` |
  | 对外可观测 | 新增**非虚** `EstimatorDegraded()`（测试/SITL 断言用，不扩张 `IMissionStateMachine`）；降级状态本期**不进 65013**（设计 Q1），机载 WARN + 10 s 存活摘要是全部出口 |
  | 配置 | `estimator_guard` 节 13 键（`enabled` + 12 个阈值/掩码），`DroneApplication::BindTopics` 整节注入 `MissionStateMachineConfig::estimator_guard`；`config::EstimatorGuardConfig` → `EstimatorGuardThresholds` 的换算唯一发生在 `mission_state_machine.cpp` 的 `ToEstimatorThresholds()`；`enabled=false` 是唯一回滚开关 |
- **起飞前门禁（2026-11-18，H 项）**：`EvaluatePreflightGate(now_ms)` 每决策拍评估地面这一层的
  判据并**锁存**，结论供受理 65014 时读取。设计口径见
  [`降落与估计器保护设计-2026-11-17.md`](../../docs/降落与估计器保护设计-2026-11-17.md) §11.6，
  判据表与 PX4 预检的分工见 [`状态机设计.md`](../../docs/状态机设计.md) §6.6：

  | 项 | 实现事实 |
  |---|---|
  | 评估时机 | `Evaluate()` 里在「S1→S2」与「分配入口」**之前**调 `EvaluatePreflightGate(now_ms)`，返回的告警位并进本拍 `warnings`；只在 `BOOT/SELF_CHECK/READY/DISARMED` 且 `latest_flight_` 有效时评估，其余状态直接返回 0（飞行中的传感器退化归 P3 `EstimatorGuard`）。估计器降级的冻结早退分支**不评估**（那一层本来就不推进任务） |
  | 锁存 | `preflight_sensor_blocked_` / `preflight_vibration_blocked_` 两个 bool，`ValidateAssignment()` **只读锁存**；hold 计时用 `preflight_sensor_unhealthy_since_ms_` / `preflight_vibration_since_ms_`，条件消失即清零并解除锁存（各打一条 INFO）。单帧抖动不会拒绝一次合法分配 |
  | 判据 ① 传感器健康 | `SYS_STATUS` 的 `present & ~health` 与 `kPreflightCriticalSensorMask`（bit0 陀螺 \| bit1 加速度计 \| bit2 磁罗盘 \| bit21 AHRS，**不含 GPS**）取交，非空且持续 `sensor_health_hold` → 锁存 → 65015 `reason=23 kSensorUnhealthy` + 65013 bit11（2048）；恢复打 INFO「关键传感器健康位恢复，允许接受任务分配」 |
  | 判据 ② 振动（默认只告警） | `VIBRATION.vibration_z` ≥ `vibration_warn_mps2` 持续 `vibration_hold` → WARN + 65013 bit12（4096），**不影响受理**；`vibration_block_mps2>0` 且 ≥ 该值持续 hold → 锁存 → `reason=24 kVibrationHigh`。指标口径：`vibration_z` 是**加速度计高频指标**（m/s/s），`x`=陀螺锥动、`y`=陀螺高频，**没有一个来自磁力计** |
  | 判据 ③ 没有 `SYS_STATUS` | `sensors_status_valid=false` → 清锁存、跳过判据、**WARN 一次**（`preflight_sensor_status_missing_warned_` 节流），**不拒绝**。理由：拿不到证据 ≠ 传感器坏了；真故障 PX4 会在 ARM 时拒绝，而把"收不到一条流"当拒绝理由会让全队禁飞 |
  | 判据 ④ 没有磁罗盘 | `present` 里没有 bit2（`SYS_HAS_MAG=0` 的装配）→ **WARN 一次**（`preflight_missing_mag_warned_`），不拒绝：PX4 不做磁力计预检，但估计器没有航向观测，航向只能靠 GPS 航迹 |
  | ARM 被拒改报预检失败 | `TakeoffAckRejected(400)` 为真时 `AbortTakeoff(kPreflightFailed(6), "400 ARM ACK 非 ACCEPTED（PX4 预检/健康检查未通过）")`（旧版回 `kArmTimeout(9)`，把预检失败报成"解锁超时"）。`AbortTakeoff()` 里额外打印 PX4 最近一条 `STATUSTEXT`（`statustext_valid` 时）：`自动起飞中止: PX4 最近告警文本（severity=…）: …`——这是现场唯一能看到的"为什么拒绝解锁"。`reason=9` 从此只表示"等实际 armed 超时" |
  | 构造期 | INFO 一行打印生效阈值：`起飞前门禁: 传感器健康=启用（hold=..ms，关键位=陀螺\|加速度计\|磁罗盘\|AHRS） 振动告警=..m/s² 阻断=..m/s²（0=只告警不阻断） hold=..ms` |
  | 边界（抓不到什么） | **抓不到"未校准"**（校准状态不是 `SYS_STATUS` 健康位），也抓不到磁强/磁倾角异常（PX4 的 `COM_ARM_MAG_STR` / `EKF2_MAG_CHK_*` 校核没有复刻）。这两类仍由 PX4 在 ARM 时拒绝，本模块负责把 `STATUSTEXT` 打出来 |
  | 回滚 | `preflight_gate.require_sensor_health=false`（传感器判据完全不跑）/ `vibration_block_mps2=0`（振动回到只告警）；改配置重启生效 |
- **雷达预检门禁与 65013 bit13（2026-11-18 P5）**：与 H 项同构——`EvaluatePreflightGate()`
  里多评估一条，`ValidateAssignment()` 只读锁存：

  | 项 | 实现事实 |
  |---|---|
  | 判据 | `radar.preflight_required=true` 时判 `common::RadarAlive(flight, now_ms, radar.stale_ms)`：**收到过** `DISTANCE_SENSOR(132)` 且距上次到达 ≤ `radar.stale_ms`（默认 200ms）。**不判量程、不判质量**——落在地上的下视测距必然在盲区（TFmini-S 最小 0.4m，装机后离地 0.1~0.2m），拿"读数量程有效"当起飞门禁等于任何时刻都拒绝起飞；与 PX4 `distanceSensorChecks`（topic 存在 + 时间戳 1s 内）口径一致 |
  | 锁存 | 不报数持续 `preflight_gate.sensor_health_hold_ms`（默认 1s，复用同一 hold）→ `preflight_radar_blocked_`；恢复报数即清零并打 INFO「雷达恢复报数，允许接受任务分配」。`ResetSortieState()` 一并清 `preflight_radar_*`（锁存不跨任务轮次） |
  | 出手 | 受理 65014 时若已锁存 → 65015 `reason=25 kRadarUnavailable`（WARN `任务指令（分配预检）: 雷达持续不报数（已≥…ms），拒绝分配（kRadarUnavailable）`）；锁存期间 65013 置 **bit13**（8192） |
  | 不出手的场景 | **飞行中雷达掉数据不置 bit13、不冻结任务**：那时高度出口已按设计退化到气压/EKF，属安全方向而不是故障（这也正是 `EstimatorGuard` 的传感器掩码里**没有** `LASER_POSITION` 的原因，见 [`estimator_guard.md`](estimator_guard.md) §3） |
  | 与 `RadarAglM` 的分工 | 量程/方向/质量/过期是**高度出口**层的拒绝原因（`RadarRejectReason`，日志 `S13a 高度源: … 原因=…`），只影响"这次下降用哪路高度"，**不会拒绝任务分配** |
  | 不做 | 不用 `SYS_STATUS.LASER_POSITION`(bit8=0x100) 做门禁——本版 PX4 的 `mavlink/streams/SYS_STATUS.hpp` 的 `fillOutComponent()` 组件列表里没有 `distance_sensor`，该位永远不置位。PX4 侧"是否要求装测距"由参数 `SYS_HAS_NUM_DIST` 管（>0 时缺测距 PX4 自己在 ARM 时拒绝） |
  | 构造期 | INFO 一行：`测距雷达: 启用/关闭（…）（高度源=… signal_quality门槛=…（0=不判） 时效>…ms 预检门禁=…）` |
  | 回滚 | `radar.preflight_required=false`（不再拿雷达卡起飞）或 `radar.enabled=false`（连高度出口一起回到 P5 之前）；改配置重启生效 |
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
- 控制输出是否接 PX4 由`DroneApplication`按**运行模式**决定（绑定
  `Px4Setpoint`+注入快照源+注入命令接口只在控制运行分支）；定义见
  [`docs/运行模式与模块状态定义.md`](../../docs/运行模式与模块状态定义.md)。
  诊断接口注入与运行模式无关（只读诊断，任何模式下都可用）。
  **状态机自身不读 `enable_control`**：需要判断"能不能发指令"时只看
  `command_sender_ != nullptr`。

## 日志行为

围栏相关（2026-10-29）：

- INFO：`围栏区域已更新: zone=../.. 顶点=.. 动作=.. 高度带=[..,..]m 基准=..`（仅内容变化时）、
  `围栏区域缓存已装载/已保存`、`任务状态机围栏区域输入已绑定`。
- WARN：`围栏非法区域已忽略`（节流，带累计计数）、`围栏区域缓存不可用`、
  `围栏越界保持: zone=../.. 距边界=..m`（节流）、`围栏越界中止: zone=../.. 距边界=..m 持续=..ms → S12 返航`、
  `S5 贴边保持: 目标越出区域 …（保持贴边不中止任务，目标回界内后自动恢复）`（每次进入一条，
  另有节流的累计次数）、`S5 贴边保持解除: 目标回界内 …`（INFO）。
- 65019 由 `fence.status_period_ms` 控制上报节拍，本身不打日志（链路层有节流发送日志），
  否则 1Hz 状态会淹没热路径。

返航速度诊断（2026-11-11；**2026-11-12 增剖面分级**）：

- INFO：S12 期间每 1s 一行
  `S12 返航摘要: 距目标=..m 垂直差=..m 地速=..m/s 指令上限=..m/s 剖面=.. 爬升段=是/否 相对高度=..m`。
  `指令上限` 爬升段取 `return_home.climb_rate_mps`、水平段取**速度剖面本拍输出**；
  `剖面` 为 `accel/cruise/decel/brake/final`（见 `include/control/return_speed_law.md`）；
  `地速` 是 EKF 水平速度模长。此前 S12 没有任何周期日志，只能靠"进入 S12 / 到达"两个
  时刻反推平均速度（165m/64s≈2.6m/s），无法判断是配置上限、执行滞后还是卡住。
  判读：`剖面=cruise` 且地速≈指令上限 → 正常；`剖面=decel` 持续出现 → 超速回落
  （追不上转返航进入）；`剖面=accel` 但地速不涨 → 查逆风/机体限幅。

降落两段（2026-11-17 P2；2026-11-18 P5 起高度源改为单一高度出口、S13b 增加落地确认门禁）：

- INFO：构造期 `降落两段配置: 启用（S13a 机载下降 → S13b 交 PX4 AUTO.LAND）/关闭（…）`
  + 全部阈值一行（现场核配置不必再翻 JSON）；
- INFO：进入 S13 时 `S13a 机载受控下降开始: 交段相对高度=..m 主降速=../s 低速档=..(≤..m) 超时=..ms`，
  随后 `降落阶段: S13a … / 发 AUTO.LAND（4/6）/ PX4 已接管…`；
- INFO：S13a 每 1s 一行
  `S13a 下降摘要: 高度=..m@<高度源> 目标=..m 垂速=..m/s 水平误差=..m 地速=..m/s 限速=..m/s 低空档=是/否 交叉校验=正常/已锁存不一致`；
  `<高度源>` 取 `雷达AGL`/`气压AGL`/`EKF差分`（`common::HeightSourceText()`）；
- INFO（**每段降落一条**，`landing_height_source_logged_`；仅 `radar.enabled=true` 时打）：
  `S13a 高度源: 雷达AGL（1.84m）`；没用上雷达时
  `S13a 高度源: 气压AGL（2.10m）——雷达未用上，原因=测距数据过期；这是设计内的逐级退化，不是故障`
  ——**这是"为什么这次没用雷达"的唯一出口**（原因取自 `RadarRejectReason`：雷达未启用 /
  未收到测距数据 / 测距数据过期 / 本帧无有效测量 / 超出量程 / 信号无效 /
  信号质量低于门槛 / 不是向下安装）；
- INFO：`S13a 交段判据满足（高度=..m@<高度源> 垂速=..m/s 水平误差=..m 保持=..ms），发 AUTO.LAND（4/6）交 PX4`、
  `S13b AUTO.LAND 命令已入队（下降已交 PX4）`、`S13 交段确认: PX4 已接管降落（AUTO.LAND），停止发布 intent…`、
  `S13a→S13b: PX4 已在 AUTO.LAND（4/6），停止发布下降意图`；
- WARN：三条保护与兜底各一条（都是安全事件，逐条不节流）：
  `S13a 跳过: Home 水平目标不在围栏区域内…`、
  `S13a 高度源交叉校验失败: <在用源>=..m <对照源>=..m 偏差=..m 持续..ms（门限../..），立即交 PX4`
  （对照源随在用高度源切换：雷达↔气压、气压↔EKF、EKF↔气压）、
  `S13a 低空水平漂移: 相对高度=..m 水平误差=..m 地速=../s（门限 1.0m / 0.5m/s）`、
  `S13a 下降超时（..ms，相对高度仍有 ..m）`、
  `S13a 连续 .. 拍无法发布下降意图`、
  `S13a PX4 已离开 Offboard 且非 AUTO.LAND（mode=../..），停止下降与命令重试`、
  `PX4 拒绝 AUTO.LAND（DO_SET_MODE result=..）`（首次一条）；
- WARN（节流）：`S13a 下降意图停发: 局部位置/高度基准/Home 水平目标不可用（连续 .. 拍）累计=..`。
- WARN（落地确认门禁，节流，**仅 `radar.enabled=true` 且 `landed_max_agl_m>0`**）：
  `S13b 落地确认: PX4 报 landed，但高度见证（雷达AGL）为 1.83m > 1.00m，暂不发 DISARM（累计 N 拍）`
  ——PX4 说已落地、独立高度见证说还悬着，先别上锁（上锁即砍推力）；
- ERROR（同一条门禁的兜底，**一条**）：矛盾持续超过 `landing.landed_agl_timeout_ms`（默认 3000ms）
  后按 PX4 处置继续 DISARM：
  `S13b 落地确认: 矛盾已持续 3010ms（PX4 landed vs 雷达AGL=1.83m > 1.00m），按 PX4 处置继续 DISARM；landing.landed_max_agl_m=0 可关闭该门禁`。

估计器降级处置（2026-11-17 P3；判据模块 `EstimatorGuard` 本身**不打日志**，日志全部在状态机侧）：

- INFO（构造期，一行含全部生效阈值）：
  `估计器降级处置: 启用（命中即切 POSCTL + 停发意图 + 冻结任务）（z跳变>x.xxm xy跳变>x.xxm
  新息比>x.xx@xxxxms baro偏差>x.xxm@xxxxms 传感器不健康@xxxms 局部位置超时>xxxms
  掩码 传感器=0x… 估计器位=0x…）`；关闭时前缀为
  `估计器降级处置: 关闭（回滚开关 estimator_guard.enabled=false：判据不跑）`。
- WARN（**每次降级一条**）：`估计器降级: 切 POSCTL 悬停，请飞手切 STABILIZED 接管:
  判据=<判据中文名> 数值=<数值> 判定耗时=<ms>ms 模式命令=已入队（POSCTL 3/0）/入队失败（将重试）
  state=<状态名>`。
- WARN（只读运行，**一条**，`estimator_mode_unavailable_logged_` 节流）：
  `估计器降级判据命中（命令接口未注入：只读运行，不处置）: 判据=… 数值=… armed=0/1`。
- INFO（每 10 s 一条**冻结存活摘要**）：`估计器降级冻结中: state=… 判据=…
  已持续=<s>s 模式命令=已入队/未入队（不发意图/不返航/不降落/不上锁）`。
- ERROR（3 次入队都失败）：`估计器降级: 切 POSCTL 连续 3 次入队失败，模式仍在原状态，
  请飞手立即接管（STABILIZED）`。
- INFO（S14 收口）：`任务结束: S14 DISARMED（估计器降级期间收口），拦截结果=已发射/未发射`。
- 另：`任务状态机创建: …` 与 `任务状态机启动` 两条日志不再带历史遗留的 `shadow=true`
  前缀（2026-11-17 清理；运行模式语义只由装配层与运行模式文档定义）。

起飞前门禁（2026-11-18 H 项；都是"能不能起飞"级别的安全事件，**不节流**）：

- INFO（构造期，一行含生效值）：
  `起飞前门禁: 传感器健康=启用（hold=1000ms，关键位=陀螺|加速度计|磁罗盘|AHRS） 振动告警=20.0m/s² 阻断=0.0m/s²（0=只告警不阻断） hold=1000ms`；
  关掉传感器判据时该字段为 `传感器健康=关闭（…）`。
- ERROR（**锁存时一条**）：
  `起飞前门禁: 关键传感器健康位持续掉线（掩码=0x…，position=<中文位名>）→ 拒绝新的任务分配（65015 reason=23 kSensorUnhealthy）。常见原因：罗盘未接线/驱动不报数/磁故障；未校准不在本判据内`。
- INFO（恢复时一条）：`起飞前门禁: 关键传感器健康位恢复，允许接受任务分配`。
- WARN（振动超告警，**锁存时一条**）：
  `起飞前门禁: 振动 X.XX m/s² 超过告警阈值 20.00（加速度计高频指标，非磁力计）——建议检查桨平衡/电机轴承/减振，本项默认只告警不阻断`；
  回落后 INFO `起飞前门禁: 振动回落到告警阈值以下`。
- ERROR（振动阻断，**只有 `vibration_block_mps2>0` 才可能出现**）：
  `起飞前门禁: 振动 X.XX m/s² 超过阻断阈值 Y.YY→ 拒绝新的任务分配（65015 reason=24 kVibrationHigh）。把 preflight_gate.vibration_block_mps2 设为 0 可回到只告警`。
- WARN（各一条，靠 `_warned_` 标志位节流）：
  `起飞前门禁: 未收到 SYS_STATUS，传感器健康门禁暂不生效（ARM 时仍由 PX4 自己的预检兜底）`、
  `起飞前门禁: SYS_STATUS 未声明磁罗盘（SYS_HAS_MAG=0？）——PX4 不会做磁力计预检，估计器无航向观测；如非有意配置请检查罗盘`。
- WARN（受理 65014 时的拒绝原因，**不节流**）：
  `任务指令（分配预检）: 关键传感器健康位持续掉线（掩码=0x… 已持续≥…ms），拒绝分配（kSensorUnhealthy）` /
  `任务指令（分配预检）: 振动 X.XX m/s² 超过阻断阈值 Y.YY（已持续≥…ms），拒绝分配（kVibrationHigh）` /
  `任务指令（分配预检）: 未收到 SYS_STATUS，本次跳过传感器健康门禁（PX4 仍会在 ARM 时做自己的预检）`。
- WARN（ARM 被拒，每次中止一条；有 `STATUSTEXT` 时再补一条）：
  `自动起飞中止: reason=6 detail=400 ARM ACK 非 ACCEPTED（PX4 预检/健康检查未通过） airborne=0 uncertain=0`
  与 `自动起飞中止: PX4 最近告警文本（severity=…）: Preflight Fail: Compass 1 uncalibrated`。

测距雷达（2026-11-18 P5；相同的"能不能起飞/降错高度"级别，**不节流**）：

- INFO（构造期一行，现场确认 `radar` 节生效以它为准）：
  `测距雷达: 关闭（回滚开关 radar.enabled=false：只剩气压/EKF）（高度源=只用 EKF差分（气压只做交叉校验） signal_quality门槛=30（0=不判） 时效>200ms 预检门禁=关（不拿雷达卡起飞））`；
  `enabled=true` 时首字段为 `启用（高度出口优先用雷达 AGL）`、高度源为
  `雷达AGL → 气压AGL → EKF差分`。
- WARN（**一条**，`preflight_radar_warned_` 节流）：
  `起飞前门禁: 雷达未报数（观察 1000ms；判据=收到过 DISTANCE_SENSOR 且时效≤200ms，不判量程——地面上本来就在盲区）`。
- ERROR（**锁存时一条**）：
  `起飞前门禁: 雷达持续不报数（≥1000ms）→ 拒绝新的任务分配（65015 reason=25 kRadarUnavailable）。检查：接线/供电、PX4 驱动是否起来（dmesg/distance_sensor 实例）、SYS_HAS_NUM_DIST；radar.preflight_required=false 可关掉该门禁`。
- INFO（恢复时一条）：`起飞前门禁: 雷达恢复报数，允许接受任务分配`。
- WARN（受理 65014 时的拒绝原因）：`任务指令（分配预检）: 雷达持续不报数（已≥1000ms），拒绝分配（kRadarUnavailable）`。
- 注意：**量程/质量不产生任何门禁日志**——它们只出现在 `S13a 高度源: … 原因=…` 里，
  说明"这次下降为什么没用雷达"，不会拒绝任务分配。

人工指令相关（2026-10-29）：
- INFO：`任务状态机任务指令输入已绑定（65020 主动返航/重置状态）`（启动期一次）；
  `任务指令回执已发布: command_id=.. type=.. result=.. reason=.. state=.. count=..`（节流，首条与每 100 条）；
  `任务指令收口: 补发 65021 type=.. command_id=.. result=.. reason=..`（2026-11-17 起，
  `type=1` 收口时一条：S14 → `result=1`/`reason=0`，S16 → `result=2`/`reason=7`；
  与受理那条同 `command_id`，不节流）；
  `任务指令重复已忽略`、`任务指令（主动返航）: 已在返航，按继续返航处理 command_id=..`
  （S12）、`任务指令（主动返航）: 已在返航/降落流程，回执完成`（S13）、
  `任务指令（主动返航）: 当前状态不可执行`。
- WARN：`任务指令（重置状态）: …`（受理、各类拒绝、回滚各一条，逐条见下节
  「重置任务状态 = 清状态 + 重启 PX4」）。
- WARN：`任务指令协议版本不支持`、`任务指令类型不支持`、
  `任务指令（主动返航）: state=.. → S12 返航 command_id=..`（接受路径，关键动作不节流）、
  `任务指令（主动返航）: 起飞阶段中止 returning=.. command_id=..`。

重置任务状态 = 清状态 + 重启 PX4（2026-11-17 P4b；与人工指令同一通道，日志里
`PX4 重启` / `重置动作配置` 前缀可单独筛）：

- INFO（构造期，一行含生效值）：`重置动作配置: 清状态 + 重启 PX4（发 246 param1=1，S1 等
  飞控回来）（超时=90000ms 246重发=3次 最小间隔=30000ms）`；回滚时该行前缀为
  `重置动作配置: 回滚：只清状态不重启（reset_reboot_px4=false，直接回 S2）（…）`。
- INFO：`PX4 重启命令已入队（246 param1=1，第 N 次）`（每次受理/重发一条；入队失败另打
  节流 WARN `PX4 重启命令入队失败（第 N 次，按 1s 重试）累计=..`）、
  `PX4 重启命令已确认（246 ACK accepted），等待飞控重新上线`（仅一次）、
  `PX4 重启完成: 断连已确认、心跳/3D fix/全局位置/Home 全部恢复，耗时 X.Xs（246 发送 N 次
  ACK=已确认/未确认），本拍起回到常规 S1→S2 判据`（仅一次）、
  `任务指令（重置状态）: 已在 S1/S2，回执完成 state=..`（幂等）。
- WARN（受理与拒绝，均为关键动作、不节流）：`任务指令（重置状态）: 已清空任务状态并请求
  重启 PX4（S1 等飞控回来，期间不发任何 setpoint/命令）command_id=.. 超时=..ms`、
  回滚受理 `任务指令（重置状态）: 已清空本次任务状态 LANDING → S2 READY command_id=..
  （原因：mission.reset_reboot_px4=false 回滚）`（原因也可能是"命令接口未注入"）、
  `任务指令（重置状态）: 飞控遥测不可用，拒绝 state=..`、
  `任务指令（重置状态）: 飞控仍上锁(armed)，拒绝 state=..`、
  `任务指令（重置状态）: 未确认落地，拒绝 state=.. landed_valid=.. landed=..`、
  `任务指令（重置状态）: 距上次重启仅 ..ms（最小间隔 ..ms），拒绝`、
  `任务指令（重置状态）: 任务故障态拒绝，需人工处置 state=..`、
  `任务指令（重置状态）: tracker 仍在空中任务相（phase=..），照常执行重置
  （ResetSortieState 会一并清空）`（只告警不拒绝）、
  `PX4 拒绝重启（246 ACK result=X，2=DENIED 3=UNSUPPORTED 4=FAILED）：已清空任务状态，
  但飞控未重启，回 S2 READY。原因多为固件未编入板级 reset 能力（SITL 即如此）或解锁
  竞态；需要真重启请人工断电/重启飞控`（仅一次）、
  `PX4 重启命令未收到 ACK（已发 N 次，reboot_retry=M）：不再重发，继续等心跳`（仅一次）。
- ERROR（仅一次）：`PX4 重启超时: ..ms 内未恢复遥测（connected=0/1 断连已见=0/1 246 发送
  N 次 ACK=已确认/未确认），保持 S1 并置 65013 bit10。重启不修复硬件（罗盘/振动会复发），
  请人工检查飞控与链路`。


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
  **拦截设备触发/复位命令已入队**（状态变化级事件，带通道/行程/归一化值摘要）；
  捕网命令入队失败（节流，下拍重试）；**单发发射超时/返航停滞（只告警，检测到
  距返航目标 0.3m 内无进展且持续 `return_home.timeout`）/返航高度被区域上限压低/
  Home 或锚点无效/PX4 接管返航/AUTO.LAND 或 DISARM 入队失败**（S11~S14，节流）；
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
- 合法`GroundStationTarget`进入`GPS_APPROACH`并发布`MissionStatus`。
- 未绑定估计输入时`IntentOutput()`不发布自动控制意图（兜底）。
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
  - `ExitRequiresMoreLostFramesThanEntry`：S7 中 LOST 连续帧 > `enter_s6_lock_frames`
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
    到达后进 S13a（2026-11-17 P2 起两段：喂 1.2m 快照满交段保持后）发 176
    （AUTO.LAND，param2=4/param3=6）→ landed+armed 发 400（DISARM）→ disarmed 进 S14；
    两段细节本身由 `Landing_*` 用例覆盖；
  - `ReturnHomeClimbsToFiveMetersBeforeCruise`：相对 Home 2m → 先原地爬升到
    Home 上方 5m（intent x/y=当前），到高后改为水平飞 Home；
  - `ReturnHomeTargetsTakeoffOriginNotDriftedHome`（2026-11-04 现场回归）：S3 起飞点
    (10,5,−10) → Home 经纬度北漂 ~10m → 返航意图仍为 (10,5,−10)，不走 Home 换算；
    注意测试要等 ≥2 个决策周期（`kDecisionPeriod=100ms`）再 `SeedAcceptedAssignmentForTest`，
    否则 `latest_flight_` 未就绪、起飞原点记不下来；
  - `ReturnHomeHoldsWhenHomeInvalid`：`home_valid=false` → 原地悬停意图
    （当前 x/y/z），状态保持 S12，不盲目返航；
  - `ReturnHomeYieldsWhenPx4LeavesOffboard`：S12 中 PX4 切 4/5 → 停发返航
    设定值、转 S13 仅监控，不再发 AUTO.LAND（176）。
  - `MakeNetActuatorTestConfig` 显式 `return_home.enabled=false`，保持捕网
    执行器用例的 S10 停留/显式离开语义不变。
- 降落两段（2026-11-17 P2；`MakeLandingDescentTestConfig` = `MakeReturnHomeTestConfig`
  + 默认 `landing` 节，用例都注入 `SetCommandSender`，因此自动走 S13a→S13b）：
  - `Landing_DescentReachesHandover`：相对高度 10m → 主降速档意图
    （reason 低位 12、目标 (Home, −1.2)、`max_speed_mps=0.8`、无前馈、不指定航向、
    带 bit31）→ 2.5m 切低速档 0.3 → 1.2m 稳定 500ms 后发 176（param2=4/param3=6）
    → landed+disarmed 收口 S14；
  - `Landing_BaroMismatchSkipsToPx4`：气压 AGL 5m vs EKF 10m（偏差 5m）持续
    `baro_mismatch_hold=500ms` → 立即交 PX4（全程只喂 10m 高度，因此 176 只能来自保护）；
  - `Landing_LowAltitudeDriftSkipsToPx4`：相对高度 2.5m、水平误差 1.2m → 立即交 PX4；
  - `Landing_DescentTimeout`：相对高度恒 10m（永不满足交段）→ `descent_timeout=10s`
    前不得发 176，到点后必须交段（配置范围下限 10s，本题按真实时钟等待）；
  - `Landing_RollbackSwitchKeepsOldPath`：`landing.enabled=false` → 直接发 AUTO.LAND，
    且**一条** reason=12 的降落意图都不许出现（回滚语义＝2026-11 之前的旧行为）。
- 估计器降级处置（2026-11-17 P3；`MissionStateMachineTest.EstimatorGuard_*`，快照 helper
  `MakeEstimatorGuardFlight()` / `PublishEstimatorGuardFlight()` 在既有返航/降落快照上
  补 `local_position_time_valid/time_ms`，**时间戳必须单调递增**，否则判据首个 armed 拍
  就会判"局部位置超时"，用例验证的就不再是注入的跳变）：
  - `EstimatorGuard_DegradedSwitchesToPosctlOnceAndLatches`：局部位置新鲜、上锁、z 单拍
    0→9 m 跳变 → 锁存 `EstimatorDegraded()`、恰好一条 176（`param1=1`/`param2=3`/
    `param3=0`，`param4~7=0`）；再连续喂同等跳变仍只有这一条（锁存后不重复切模式）；
  - `EstimatorGuard_DisarmedDoesNotReact`：未上锁时 0↔9 m 往复 + 末帧缺局部位置时间戳
    → 不锁存、0 条 176（命令接口已注入，证明"没切模式"不是没地方发，而是判据没动作）；
  - `EstimatorGuard_FreezesMissionAndStopsIntents`：S13a 中注入 z −10→−19 m 跳变 →
    状态留在 `kLanding`、1.5 s 内 0 条控制意图、无 `AUTO.LAND(4/6)`（176 只有切 POSCTL
    的那一条）。
- **既有用例的工厂统一关闭判据**：`mission_state_machine_test.cpp` 里所有既有
  `MissionStateMachineConfig` 工厂（含派生工厂）都关闭判据——底座工厂里显式写了
  `config.estimator_guard.enabled = false`：它们的快照不模拟 `LOCAL_POSITION_NED`
  新鲜度（P1 才新增的字段），判据会在第一拍误判"局部位置超时"并冻结，与这些用例要
  验证的东西无关。**新写用例要测降级必须显式打开判据**，并按
  `MakeEstimatorGuardFlight()` 那组 helper 给快照填时间戳；判据逻辑本身仍由
  `estimator_guard_test` 13 条覆盖，接线由上面 3 条覆盖。
- 只跑估计器降级这组用例（两条等价）：

  ```bash
  ctest --test-dir build -R EstimatorGuard_ --output-on-failure
  ./build/mission_state_machine_test --gtest_filter='*EstimatorGuard*'
  ```

- 起飞前门禁（2026-11-18 H 项；`MissionStateMachineTest.PreflightGate_*`，快照 helper
  `MakePreflightGateFlight()`，用例把两个 hold 收到配置下限 100ms 以压缩时长；判据从
  65015 `reason` 与 65013 告警位两头断言）：
  - `PreflightGate_SensorUnhealthyRejectsAssignment`：`present` 含磁罗盘但 `health` 清其位，
    持续一个 hold → 65013 bit11（2048）亮、状态**仍停在 S2 READY**（评估本身不迁移）、
    65014 回 `rejected/kSensorUnhealthy`、零落飞控命令；
  - `PreflightGate_SensorHealthRecoveryAllowsAssignment`：先以同样方式建立拒绝锁存，再把健康位
    全部恢复持续一个 hold → 同一夹具发 65014 得到 `accepted` 并进入 S3（锁存解除、不再按
    `kSensorUnhealthy` 拒绝）；
  - `PreflightGate_VibrationWarnOnlyKeepsAssignmentAccepted`：`vibration_z=25`（> 默认 warn 20）
    持续 hold → bit12（4096）亮，但默认 `vibration_block_mps2=0`，65014 照常受理进 S3、
    reason 不得是 `kVibrationHigh`；
  - `PreflightGate_VibrationBlockRejectsWhenConfigured`：配置 `vibration_block_mps2=30`
    （> warn 20）后 `vibration_z=35` 持续 hold → 65014 回 `rejected/kVibrationHigh`、
    停在 S2 READY、零落飞控命令（证明振动阻断是"配了才生效"）；
  - `PreflightGate_ArmRejectionReportsPreflightFailed`：S3 中 176/400 都已入队后模拟
    `400` ACK `result=1`（TEMPORARILY_REJECTED，等价 PX4 预检拒绝解锁）→ 65015 aborted 的
    reason 必须是 `kPreflightFailed(6)`（旧实现错报 `kArmTimeout(9)`）。
  只跑这一组：

  ```bash
  ctest --test-dir build -R PreflightGate_ --output-on-failure
  ./build/mission_state_machine_test --gtest_filter='*PreflightGate_*'
  ```

- 测距雷达（2026-11-18 P5；`MissionStateMachineTest.Radar_*` / `Landing_*Evidence*` /
  `PreflightGate_Radar*`，快照 helper `MakeRadarLandingFlight()` 在
  `MakeReturnHomeFlight` 基线之上补齐单一高度出口所需字段——**雷达帧时间戳取当前单调时钟**，
  否则会立即被判"测距数据过期"而测不出想测的分支）：
  - `Radar_DescentHandoverUsesRadarAglNotEkf`：构造"EKF 说 5.0m、雷达与气压都说 1.2m"的分歧
    快照 → **必须走交段**（若还在吃 EKF 则永远不交段）；
  - `Radar_OutOfRangeFallsBackToBaroAndCrossChecks`：雷达帧停在 5 s 前（`distance_time_valid`
    仍为真、age ≫ `stale_ms`）→ 退到气压 AGL 并**继续发布下降意图**（不停发、不误交段）；
    过期/超量程在机载侧的唯一表现就是"没有新帧"，这条用例把它钉死；
  - `Landing_HigherThanEvidenceDoesNotDisarmImmediately`：S13b 中 PX4 报 `landed` 且仍
    armed，雷达高度见证 2.0m > `landed_max_agl_m`(1.0m) → **不发 DISARM**（先不上锁）；
  - `Landing_DisarmsWhenEvidenceAgrees`：同样 `landed`+仍上锁，但见证 0.6m ≤ 1.0m →
    认账并发 DISARM；
  - `PreflightGate_RadarRequiredRejectsWhenNoReading`：`radar.preflight_required=true` 且
    持续没有 `DISTANCE_SENSOR` → hold 后 65013 bit13（8192）亮、65014 回
    `rejected/kRadarUnavailable(25)`、**保持 S2 READY、零落飞控命令**；
  - `PreflightGate_RadarReadingRecoversAccepts`：先建立拒绝锁存，再持续发布新鲜雷达帧
    （**刻意用盲区内的越界读数** 0.05m < min 0.4m）→ 锁存解除、65014 正常受理——这条正是
    "只判在报数、不判量程"的回归证据。
  只跑这一组：

  ```bash
  ctest --test-dir build -R 'Radar_|PreflightGate_Radar|Landing_.*Evidence' --output-on-failure
  ./build/mission_state_machine_test --gtest_filter='*Radar_*:*PreflightGate_Radar*:*Evidence*'
  ```

  纯类型层的退化链/拒绝原因由独立目标 `height_source_test` 覆盖（不启线程、不依赖状态机）：

  ```bash
  ctest --test-dir build -R height_source --output-on-failure
  ./build/height_source_test
  ```

- S9/S10 入口（2026-10-19；`MakeInterceptEntryTestConfig` 开启
  `interception.enabled=true`、`enter_s9_lock_frames=3`、窗口 2~12m、自动授权）：
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
- 围栏高度基准（2026-11-04）：`FenceAltitudeUsesTakeoffReference`（`altitude_reference=1` 用
  EKF 差分——Home 海拔跳 20 m 不改变相对高度，爬升越过 `alt_max` 才报 `kViolated`）、
  `FenceAltitudeFollowsHomeReanchor`（Home 与 `local_z` 同时平移 15 m，相对高度不变；
  旧快照仍被使用的话会算成 13 m 误判）、
  `FenceAltitudeUnavailableJudgesHorizontalOnly`（无起飞基准时只判水平、不判高度）、
  `FenceAltitudeHysteresisAwaitsClearance`（−10.5 m 越界 → −9.5 m 仍越界 → −8.0 m 才恢复）。
- 65020 重置任务状态（2026-11-04 引入；2026-11-17 P4b 改为"清状态 + 重启 PX4"）：
  `MissionCommandResetStateReturnsDisarmedToReady`（改写：S14 + 未 arm → `kAccepted`、
  切 S1、246 入队、`Px4RebootPending()` 为真）、
  `MissionCommandResetStateRejectedWhenArmed`（`armed=true` → `kRejected/kNotOnGround`、
  状态不变）、`MissionCommandResetStateIdempotentInReady`（S2 再重置 → `kCompleted`）、
  `ResetGuard_LandedFromS13Accepted`（原 `...RejectedOutsideDisarmed` 改写：S13 已落地 +
  未上锁也受理——旧代码会拒）、`ResetGuard_NoTelemetryRejected`（断连 →
  `kRejected/kTelemetryStale(9)`）、`ResetGuard_FaultRejected`（S16 故障态 →
  `kRejected/kMissionFault`，且 65021 必须真的发出来）、`ResetGuard_NotLandedRejected`
  （未落地 → `kRejected/kNotOnGround`）、`ResetGuard_RollbackSwitchKeepsOldPath`
  （`reset_reboot_px4=false` → 回 S2、零 246）、`Reboot_WaitsForHeartbeatThenReady`
  （等断连→恢复 → `Px4RebootPending()` 归零、S1→S2）、`Reboot_TimeoutStaysInSelfCheck`
  （超时 → 保持 S1 + 65013 bit10）、`Reboot_RetryStopsAfterLimit`（246 发满
  `reboot_retry` 次即停）、`Reboot_DeniedFallsBackToReady`（ACK DENIED → 停重发、
  回 S2 READY）、`ResetGuard_ClearsEstimatorDegradationAfterLanding`（估计器降级冻结中
  空中重置被 `kNotOnGround` 拒、落地 disarm 后重置受理并清掉降级锁存——冻结分支里
  仍处理 65020，否则"重置解除降级"不可达）。`mission` 配置 3 条在 `config_test`：`ConfigTest.
  MissionDefaultsWhenSectionMissing` / `MissionPartialOverrideKeepsDefaults` /
  `MissionOutOfRangeRejected`。只跑这一组（两条等价）：

  ```bash
  ctest --test-dir build -R 'ResetGuard_|Reboot_' --output-on-failure
  ./build/mission_state_machine_test --gtest_filter='*ResetGuard_*:*Reboot_*'
  ```
- 65020 主动返航两条回执（2026-11-17 §11.6 E 项）：
  `MissionCommandReturnHomeCompletedAckAtLanding`（受理回 `kAccepted/kNone`、`mission_state`
  已是 S12(12)；落地 disarm 进 S14 后补第二条 65021 `kCompleted/kNone`、`mission_state=14`，
  `command_id`/`type` 与受理那条一致）、`MissionCommandReturnHomeFailsafeAckOnFault`
  （受理后任务故障进 S16 → 补 `kRejected/kMissionFault(7)`、`mission_state=16`）。
  只跑这一组：

  ```bash
  ctest --test-dir build -R 'MissionCommandReturnHome' --output-on-failure
  ./build/mission_state_machine_test --gtest_filter='*MissionCommandReturnHome*'
  ```

- 配置（`interception`/`return_home` 节）：默认值、显式解析、非法窗口/
  返航高度拒绝、`interception.enabled=true` 且 `return_home.enabled=false`
  拒绝（发射后必须有返航收口）。
- 任务分配入口（Task 4，2026-10-22；`AssignmentHarness` 装配 65014/65015
  Topic、命令接口与估计输入，`MakeGroundedFlight` 提供地面/未 armed/导航有效
  快照）：
  - `AssignmentRejectedWithoutCommandSender`：未注入命令接口（只读运行）→ `control_disabled`、
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
    接口（只读运行）时，既有 S2→S5 路径保持（函数名沿用历史称呼，语义即"只读运行"）。
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
  - `TakeoffClimbRampsPositionWithVerticalFeedforward`：S4 远段持续发
    `kTakeoff` 位置意图（z 在 `[目标, 起飞点]` 之间逐拍抬升）并带
    `feedforward_vz_mps=-climb_rate`、mask 含 `kVelocityValidDown`；发布
    到达目标高度的飞行快照后，末段撤前馈并直接盯目标高度，随后进入 S5；
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
  - `ShadowS5IgnoresFilterAndWaitTimeout`：未注入命令接口时的 S5 不按 target_id 过滤，
    `target_wait_timeout` 不生效（既有行为保持；函数名沿用历史称呼，语义即"只读运行"）。
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
  （默认 true；捕网执行器隔离测试显式 false）、执行器 `Triggered()`/发射超时日志；
  若已进入 S16，说明发射入队超时或执行器故障，属故障处置而非返航路径。
- **S12 原地不动/不飞 Home**：查 `home_valid`/`ned_ref_valid`/
  `local_position_valid`（任一无效按设计原地悬停并 WARN）；查
  `返航停滞: ..s 未靠近返航目标` WARN（2026-11-04 起只告警不中止，飞机仍应继续飞）；
  日志若出现旧版 `返航超时，原地悬停` 说明跑的是改造前的固件。确认 PX4 仍在
  Offboard(6/0)，否则状态机按设计转 S13 让权。
- **S12 不远却很久不判"到达"**：先看有没有
  `返航高度受区域高度带上限约束: 5.0m -> ..m`——区域高度带上限低于返航高度时，
  飞控会把设定值钳在区域内，垂直误差永远超 0.5m；此时要么抬高区域上限，
  要么按改造后的语义让返航高度自动收敛（2026-11-04 起）。
- **S12 不进入 S13**：到达判据为水平 2m、垂直 0.5m、水平速度 0.5m/s
  持续 1s；查位置/速度遥测是否持续满足；位置意图 reason 低位应为 8。
- **S13 不发 AUTO.LAND**：确认 `SetCommandSender` 已注入（`enable_control=true`
  装配分支），查 `AUTO.LAND 命令已入队`/`S13b AUTO.LAND 命令已入队`/入队失败 WARN；
  PX4 已被 failsafe 接管时状态机按设计不再发 176，只等 landed/disarmed。
- **S13 站着不降、也不发 AUTO.LAND（2026-11-17 起的新现象）**：说明处于 **S13a**。
  看 `S13a 下降摘要`（1Hz）里的 `相对高度/目标/垂速/水平误差`：
  - 相对高度不降 → 查位置意图 reason 低位是否为 12、控制器是否在发 setpoint
    （`PX4 设定值下发统计` 应保持 ~24Hz）、PX4 是否仍在 Offboard；
  - `相对高度` 卡在 `目标` 之上且 `限速=0.30` → 低速档在收敛，正常；
  - 若紧接着出现 `S13a 交段判据满足` 才是正常的交段；否则看有没有 WARN：
    `高度源交叉校验失败`（气压与 EKF 不一致 → 已交 PX4）、`低空水平漂移`、
    `下降超时`、`连续 .. 拍无法发布下降意图`（缺局部位置/高度基准/Home 水平目标）。
- **S13a 刚开始就 WARN `跳过: Home 水平目标不在围栏区域内`**：区域多边形不包含
  Home（或 Home 水平目标算不出来）→ 直接交 PX4，这是设计行为（避免控制器钳制把
  落点带偏）。若属误报，核对 `fence` 区域与 Home/起飞原点。
- **S13b 长时间没有 `交段确认`**：AUTO.LAND 可能被 PX4 拒绝（看
  `PX4 拒绝 AUTO.LAND（DO_SET_MODE result=..）`），或 PX4 还没切到 4/6。
  此期间状态机**仍在发保持意图**（不推进下降），不会掉进 Offboard 丢失兜底；
  先查 PX4 侧模式切换是否被拒（如 `COM_DISARM_LAND`/land detector 状态）。
- **65015 `reason=25 kRadarUnavailable`（雷达不报数）**：先看构造期 `测距雷达:` 一行确认
  `预检门禁=硬判`（即 `radar.preflight_required=true`），再按顺序查：①`DISTANCE_SENSOR`
  到底有没有到（`px4_link_smoke` 的 `DISTANCE_SENSOR(132 测距) 收到 N 条`、或机载日志里
  `S13a 高度源` 是否出现过 `雷达AGL`）；②PX4 侧驱动是否起来（`dmesg`、`distance_sensor`
  实例、`SYS_HAS_NUM_DIST`）；③接线/供电/串口。**判据不含量程**——落在地下视测距本来就在
  盲区（TFmini-S 最小 0.4m，装机后离地 0.1~0.2m），所以"地面上读数很小/无读数"是正常的，
  只要 `DISTANCE_SENSOR` 在按 10Hz 到达就算"在报数"。复现：`SITL_LIDAR=1 bash
  tools/sitl_harness.sh stack`（有雷达）对 `bash tools/sitl_harness.sh stack`（默认机体，
  无雷达 → 应拒绝分配）。回滚：`radar.preflight_required=false`。
- **S13a 用了气压/EKF 而没用雷达**：看那条 `S13a 高度源: …——雷达未用上，原因=…`。常见原因
  与对应查法：`测距数据过期`（超量程/无回波时 PX4 的 TFmini 驱动干脆不发布，查距离是否
  超出 0.4~12m、`radar.stale_ms` 是否被调小）、`不是向下安装`（orientation 必须是 25）、
  `信号质量低于门槛`（`min_signal_quality` 与实测值；不给质量的雷达恒 0=未知，不受约束）、
  `超出量程`、`未收到测距数据`（`radar.enabled` 关了？链路没订上 132？）。
  **这是设计内的逐级退化，不是故障**——雷达没了退气压、再退 EKF，不会中断降落。
- **S13b 迟迟不发 DISARM / 一直停在 `kLanding`**（`radar.enabled=true` 时）：先看
  `S13b 落地确认: PX4 报 landed，但高度见证（…）=…m > …m，暂不发 DISARM`——说明 PX4 的
  land detector 说已落地，而我们的独立高度见证（首选雷达 AGL）说还悬着（典型是雷达卡在某个
  读数、或地形/安装偏差让 AGL 偏大）。矛盾超过 `landing.landed_agl_timeout_ms`（默认 3000ms）
  会按 PX4 处置继续 DISARM 并打 ERROR，不会永久卡住。要临时绕过：`landing.landed_max_agl_m=0`
  （关掉该门禁）。**注意能力边界**：这条拦不住 PX4 自己在 `AUTO.LAND` 里判落地砍推力。
- **飞行中雷达掉数据，65013 bit13 却不亮**：这是设计——bit13 只在**地面待命 + 预检期间**
  置位；飞行中高度出口退化到气压/EKF 是安全方向，不是故障（见
  [`estimator_guard.md`](estimator_guard.md)「为什么不加 LASER_POSITION」）。
- **降落后想回到"直接 AUTO.LAND"**：`landing.enabled=false`（回滚开关，重启生效）。
  只读运行（未注入命令接口）本来就发不出降落意图，自然等于旧路径。
- **日志出现 `估计器降级:` 但飞机没换模式**：先看这条 WARN 的 `模式命令=`——
  `入队失败（将重试）` 表示 `SendCommand` 返回 false（命令没离开本机：链路未就绪/队列满），
  会按 `return_home.command_retry` 重试最多 3 次，3 次都失败会打 ERROR
  `切 POSCTL 连续 3 次入队失败…请飞手立即接管（STABILIZED）`；再查
  `command_sender_ == nullptr`（只读运行，此时只有 `命令接口未注入：只读运行，不处置` 的
  WARN、没有命中 WARN），最后查 PX4 串口/链路（对照 `docs/故障排查手册.md` §17.11）。
- **想临时关掉降级判据**：`config/config.json` 的 `estimator_guard.enabled=false`
  （重启生效，唯一回滚开关）。关掉后判据完全不跑：不告警、不切模式、不冻结，
  调度行为回到 P2（S13a 的三条保护仍在，但**不再兜 EKF 跳变/传感器失效**）。
- **降级后任务卡住不动、不返航也不降落**：这是设计，不是卡死——冻结期间跳过
  电池守卫/围栏/看门狗/返航/降落全部逻辑（低电也不会自动返航），处置完全交给飞手。
  **恢复入口是「重置任务状态」（65020 `type=2`），不是重启机载程序**：冻结分支里仍处理
  65020 并逐条回 65021，空中（`armed=true`）被 `kRejected/kNotOnGround` 拒，落地 disarm
  后按 P4b 准入（遥测在线 + 未上锁 + 已落地 + 非故障）受理——清状态会一并清
  `estimator_degraded_`，随后按 `mission.reset_reboot_px4` 重启 PX4 并停在 S1 等飞控
  回来，或（回滚开关）只清状态直接回 S2。
- **重置后一直停在 S1、日志有 `PX4 重启超时`**：说明 246 之后飞控既没断连、也没恢复
  遥测。先确认飞控是否真的重启（听继电器/看 PX4 自身日志/串口是否重开），再查供电、
  串口 `/dev/ttyS1` 与链路；超时只置 65013 bit10 并保持 S1，**不会**自动重启机载程序。
- **日志 `PX4 拒绝重启（246 ACK result=2/3/4）`**：SITL 恒为 DENIED（未编
  `CONFIG_BOARDCTL_RESET`）；真机多为固件未编板级 reset 能力。此时任务状态已清干净、
  机载回 S2 READY 可继续接任务；需要真重启请人工断电/重启飞控。不要把 result=1
  （TEMPORARILY_REJECTED，会继续重试）误判成拒绝。
- **"重置了但没重启"**：先看构造期 `重置动作配置:` 一行——`回滚：只清状态不重启` 说明
  `mission.reset_reboot_px4=false`；配置为启用却没发 246，则确认命令接口已注入
  （只读运行 `command_sender_ == nullptr` 时按设计只清状态、直接回 S2）。
- **改判据阈值时别只改字典**：`EstimatorGuardThresholds` 的默认值与 `config.h` 的
  `EstimatorGuardConfig` 同源，生产值来自 `estimator_guard` 节 13 键；启动日志
  `估计器降级处置: …` 打印的是**生效值**（含 `critical_sensor_mask=0` 被归一成
  `0x200007`），排查"为什么没告警/为什么误告警"以日志为准，改配置后必须重启再对一遍。
- **S13 已落地但不到 S14**：通常为仍 armed 且 DISARM 未 accepted；查 400 命令
  入队/ACK 与 PX4 disarmed 状态。
- **地面站显示"已受理"后一直不再变化（飞机其实在返航/已落地）**：`type=1` 受理只回
  `accepted`，终态靠 S14/S16 补第二条同 `command_id` 的 65021。先在机载日志里找
  `任务指令收口: 补发 65021 type=1 command_id=.. result=.. reason=..`：
  - **没有这条** → 状态机没走到 S14/S16：按上面「S12 不进入 S13」「S13 已落地但不到
    S14」两条查（没到收口点自然没有终态回执，不是回执丢了）；
  - **有这条但界面仍不动** → 地面站侧没按 `command_id` 更新记录（老 Web 把第二条当成
    "重复指令"丢弃）；Web 与机载需成对升级，机载侧无法替 Web 显示。
  - `type=2` 重置本来就没有第二条 65021，别拿它套这个现象。
- **65014 发送后没有 65015**：确认 `SetAssignmentInput` 已绑定且
  `GroundStationLink::MissionAssignmentOutput()` 已接到该 Topic；地址/来源
  不匹配的 65014 由 GroundStationLink 丢弃（设计上不回 65015），不是状态机问题。
- **65014 回 `control_disabled`**：未注入命令接口（只读运行，或装配异常）属预期；
  自动起飞必须在控制运行的装配分支下注入 `SetCommandSender`。
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
- **S4 不爬升/不前进**：确认仍在 Offboard(6/0) 且意图在流动——全程是
  `kTakeoff` 位置意图（z 指令按 `climb_rate` 逐拍抬升），远段另带垂直前馈
  `feedforward_vz_mps=-climb_rate`（mask 含 `kVelocityValidDown`），进入
  `|z-z_target| ≤ max(0.20, z_tolerance_m)` 后改为直接盯目标高度并撤前馈；
  高度/水平/速度
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
- 整机实测停在 S5、从不进 S6：先看这是不是**裁剪装配**——估计器/视觉稳定性/双目测距
  三者必须同时装配才会调用 `SetEstimationInputs`（缺一不绑定，状态机不发布 S5 意图）。
  生产配置默认三者齐备，因此正式飞行中"停在 S5"要按 S5 判据本身排查（锁定帧数/
  测距/视场余量/创新门限），不要再当作"生产未接线"。G3 全链验收另走独立装配工具
  `tools/px4_link_smoke --sitl-stage4-mission`（见 `tools/px4_link_smoke.md`）。
- 地面站看到"状态=S7 但飞机不动/无控制输出"：查 `MissionStatus.active_warning_bits`
  bit3（S7 伺服停发中）——置位说明伺服尚未启动或控制律降级停发，实际输出要么是
  冻结位置意图、要么无意图（控制器意图超时刹停）；服务器侧诊断先用该位区分
  "状态机在跟踪"与"实际控制输出在跟"，再查 WARN"视觉伺服停发意图"的降级原因。
- 捕网不触发（日志无「拦截设备触发命令已入队」）：先确认是否**根本不满足触发条件**——
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
- **进 S16 / 地面站 bit4 置位**：查 ERROR「拦截设备故障，停止任务控制并交
  PX4/遥控」的原因串（187 ACK 超时 / 非 ACCEPTED / PX4 断连 / 触发命令未入队
  （发射超时））。S16 为终态，状态机不再发任何控制意图，需人工/维护处置后
  重启程序；`CancelPendingCommands()` 只取消待发命令，不会撤回机构已执行动作。
- **65014 回 `sensor_unhealthy(23)` / `vibration_high(24)`**（2026-11-18 H 项）：先把 65013
  告警位与日志对齐——bit11 + ERROR「关键传感器健康位持续掉线（掩码=0x…，position=…）」说明
  是 `SYS_STATUS` 健康位判据（常见：罗盘未接线/驱动不报数/磁故障）；bit12 只表示振动超
  **告警**线（默认**不影响受理**，65015 出现 `vibration_high` 只有在
  `preflight_gate.vibration_block_mps2>0` 时才可能）。健康位恢复后锁存自动解除并打 INFO，
  **不需要重启机载程序**；要临时回到旧行为，改 `require_sensor_health=false` 或
  `vibration_block_mps2=0` 后重启。**别把它当成"能不能起飞"的唯一判据**：没收到 `SYS_STATUS`
  时本门禁主动跳过（只 WARN 一次），而"未校准"永远看不到健康位——这两类仍由 PX4 在 ARM 时拒绝。
- **65015 `reason=6` 与 `reason=9` 别混**：`6 kPreflightFailed` = PX4 **明确拒绝解锁**
  （`400` ACK 非 ACCEPTED；紧跟一条 `自动起飞中止: PX4 最近告警文本（severity=…）: Preflight Fail: …`），
  照着那句英文修预检项（罗盘未校准/强磁干扰/传感器缺数据/加速度计未标定）；`9 kArmTimeout` =
  命令已发出、PX4 既不拒也不上锁，等满 `takeoff.arm_timeout`，属链路/时序问题。
  看到 6 去加大超时或 force arm 是错的方向。
- 后续恢复完整自动链时，不能直接从`READY`跳到`GPS_APPROACH`输出控制；必须补齐`ARMING/TAKEOFF`门禁、SITL、拆桨台架验证和运行模式（`enable_control`）安全确认。
