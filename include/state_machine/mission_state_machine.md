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

阶段四新增（Task 10）：**捕网执行器命令（`MAV_CMD_DO_SET_ACTUATOR`）**——注入命令窄接口`IPx4CommandSender`后，状态机在进入 **S10（INTERCEPT_EXECUTING）** 时发一次捕网触发命令（`net_fired_`防重复；入队失败不置位、下拍重试），离开拦截序列（S9/S10 → 回退/返航/POST_INTERCEPT）时发**复位行程**并清除触发标志（机构归位到待命行程；复位入队失败同样下拍重试，且复位未入队前不发触发）。命令参数按配置换算：`actuator`节的 PWM 微秒行程经 PX4 输出映射区间归一到 **-1..1**（换算式见下），只有本通道对应的参数位填值、**其余参数位 NaN**（MAVLink 规范语义"忽略"；不得填 0——0 为中位行程，会误动其他执行器组），`p7=0`（执行器组序号，PX4 仅处理组 0）。

**MAVLink/PX4 语义核对（2026-10-13，实现前查证）**：`MAV_CMD_DO_SET_ACTUATOR=187` 的官方参数语义是 **p1..p6 = Actuator Set 1..6 的归一化值（-1..1，NaN=忽略）**、**p7 = 执行器组序号**——执行器"通道号"是**参数位**而非参数值。PX4 把 Actuator Set N 映射到配置了输出功能「Peripheral via Actuator Set N」的输出（`FunctionActuatorSet`，仅组 0、最多 6 组），并把 -1..1 线性映射到该输出的 PWM 区间（`PWM_AUX_MINn/MAXn`，默认 1000/2000µs）。因此实现**不把毫秒/微秒直填参数**，而是：

```text
norm = 2·(pwm_us − output_pwm_min_us) / (output_pwm_max_us − output_pwm_min_us) − 1
```

`output_pwm_min_us/max_us`为配置项（必须与所接通道的 `PWM_AUX_MINn/MAXn` 一致，装机后按实配核对）；行程必须落在该区间内（`ActuatorConfig::Validate` fail-fast，否则归一化超出 -1..1 不可表达）。默认行程 1100/1900µs 在默认区间下换算为 **-0.8 / +0.8**。换算公式的单测见`ConfigTest.ActuatorNormalizePwmMapsOutputRangeToMinusOneToOne`；实际行程方向/到位由 **G4 拆桨台架**核对。

**拦截序列入口（2026-10-19 起配置化）**：S9/S10 入口判据已实现——`interception` 配置节（默认 `enabled=false`）提供连续锁定帧 `lock_frames`、测距窗口 `[min_range_m,max_range_m]`（占位值，待实机射程冻结）与 `auto_authorize`/`authorization_timeout_ms`；运行时还要求视觉 `LOCKED`、PX4 `connected+armed+Offboard(6/0)`、姿态/局部位置有效、命令窄接口已注入、捕网未触发/无复位挂起。S7→S9→S10（自动授权）；S9 条件失效回 S7；授权超时/发射结束按 S12 返航；S9/S10 继续输出视觉伺服意图保持瞄准。默认关闭时整机仍不自动发射；测试钩子/独立工具可显式开启。

阶段四新增（Task 12，2026-10-13）：**S7 报告状态与实际控制输出背离的可观测位**——`MissionStatus.active_warning_bits` 新增 bit3「S7 伺服停发中」（`kWarningServoSuspended`）：处于 `VISUAL_TRACKING` 且**本拍未发出视觉伺服意图**时置位，覆盖两种背离形态：① 伺服尚未启动（S6/S7 过渡期仍在续发冻结位置意图，报告已是 S7 而输出仍是位置式）；② 控制律降级停发（姿态/视觉超龄 → 控制器意图超时门禁接手刹停，报告仍是 S7 而无输出）。伺服意图正常发出即清除；离开 S7 亦不置位。位语义与 bit1（目标过期）/bit2（热备份断粮）并列，映射表见 `docs/数据接口文档.md` §3.4 与 `docs/地面站目标位置与时间同步协议.md` §14.3（65013 `active_warning_bits`）。

**G3 全链验收（Task 12，2026-10-13，`tools/px4_link_smoke --sitl-stage4-mission`）**：状态机转正验收走**独立装配工具**（工具装配 `MissionStateMachine → FlightController → Px4Link`，扮演地面站/视觉监视器/双目测距三路生产者注入模拟数据），覆盖 S5→S6→S7→回退→停发全序列、交接缝意图无超龄（末条 S5 位置意图→首条 S7 速度意图间隔 < `intent_timeout` 且窗口内未跌入 COAST——`lock_frames` 门限 + 20Hz 注入使 S6 只驻留 1 决策拍，S6 冻结续发的逐拍滚动语义由本模块单测覆盖，工具侧只作诊断记录）、S7 伺服启动、短期/持续 LOST 语义、目标过期停发链与卡尔曼备份接管。**`DroneApplication` 生产装配仍未调用 `SetEstimationInputs`**（G3 裁决：生产接线与 `enable_control` 打开一并做，避免在未启控制时改变整机行为）——整机停在 S5 属预期，工具装配与生产装配共享同一状态机实现与配置注入路径。

本阶段仍不做以下事情：

- 不发送解锁、起飞或外部 setpoint（`SetCommandSender` 当前消费点：S10 捕网触发/复位、S13 AUTO.LAND、S14 前 DISARM；ARM/起飞仍待后续任务）。
- S9/S10 入口判据与自动授权已落地（上段），但**默认 `interception.enabled=false`**；地面站确认授权协议尚未实现（`auto_authorize=false` 时只等超时返航），执行机构就绪仍以命令接口注入 + 未触发/无复位挂起为代理，未接硬件状态反馈。
- 不把地面站目标透传为PX4航点。
- 不实现设计 §2.2 的"位置误差/地速/首条视觉指令经限幅生效"三项验证量（依赖
  控制侧反馈与尚未冻结的阈值配置），S6→S7 仍用"条件持续 + 视觉链推进 +
  驻留超时"作最小验证。**T12 处置（2026-10-13）**：G3 全链验收以"迁移序列与
  设计 §2 判据一致 + 交接处输出连续（速度设定值无越限跳变）"作为等价验收面，
  三项量化条件待控制律整定（设计 §7）后在 G3 实跑/回放中冻结，阈值不落代码。
- S7 无姿态超龄退出路径（边界声明）：姿态超龄由控制律降级停发 → FlightController
  授权链收口（姿态异常同时意味着 PX4 数据链异常，由控制器门禁裁决）。

完整自动任务链仍以`docs/状态机设计.md`为准；当前跳过`ARMING/TAKEOFF`只用于无控制输出的地面站目标接入验证，后续接入控制门禁后需要恢复完整状态推进。

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

    // ---- 测试入口（生产装配不得调用） ----
    void ForceStateForTest(common::MissionState state);  // 强制迁移（需 test_hooks_enabled=true）
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
Px4Link（IPx4CommandSender）─► SetCommandSender(...)（仅 enable_control=true 装配分支；
                             当前唯一消费点=捕网执行器触发/复位，Task 10）
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
- **捕网执行器（Task 10）**（`EvaluateNetActuator` / `SendNetActuatorStroke`）：
  - 判定按"状态驻留"而非"迁移边沿"：决策线程每拍末段评估（`Evaluate` 内迁移后
    立即调用，进入 S10 当拍触发、离开拦截序列当拍复位，无额外延迟）；
    拦截序列={S9, S10}，离开=任一其他状态（回退/返航/POST_INTERCEPT）。
  - 触发语义：S10 内 `net_fired_` 未置位才发；`SendCommand` 成功（入队）才置位，
    失败（PX4 未连接/命令队列满）节流 WARN 并下拍重试——触发命令不得因一次
    入队失败而丢失。S9 只标记序列开始，不触发（设计：S10 才调用执行机构）。
  - 复位语义：离开拦截序列置 `net_reset_pending_` 并当拍尝试发复位行程；
    入队成功才清除，失败下拍重试；**复位未入队前不发触发**（先复位、后触发
    的次序在异常重试下仍保持）。`net_fired_` 随离开清除——再次进入 S10 是新
    一次拦截（"防重复"仅限同一拦截段；设计 V1「一次上电一次拦截」由后续入口
    判据承担，本层不越权裁决）。
  - 参数组装：只有本通道参数位填归一化值，**其余位 NaN**（规范"忽略"；
    0=中位行程会误动其他执行器组），p7=0。命令号 187 以常量携带
    （`kMavCmdDoSetActuator`，状态机不引 MAVLink 头）。
  - 门禁：`command_sender_==nullptr`（enable_control=false 装配语义）→ 零调用，
    本功能物理缺席；触发/复位同为控制输出，不另开旁路。
  - 进程退出不补发复位行程（`Stop()` 不触发机构动作）：机构实际位置由
    G4 台架/人工确认；拦截执行段内停机属异常场景，PX4 侧输出保持最后值。
  - 边界：状态驻留 <1 决策周期（100ms）的极端迁移可能被跳过——生产语义下
    S10 驻留由拦截结果决定（远长于决策周期），不构成风险。
- **S9/S10 入口判据（2026-10-19）**：`EvaluateInterceptSequence` 在 S7 每拍评估
  `InterceptEntrySatisfied`——`interception.enabled`、视觉新鲜且 `LOCKED`、
  `consecutive_lock_frames_ >= lock_frames`、测距 `valid`+新鲜且
  `[min_range_m,max_range_m]`、`connected+armed+Offboard(6/0)`、姿态/局部位置有效、
  `command_sender_` 非空、`net_fired_/net_reset_pending_` 均为 false；可选
  `min_battery_percent>0` 时还要求电池遥测有效且剩余电量达标（**当前生产保持 0，
  实机后冻结**）。满足→S9；
  S9 条件失效→S7；`auto_authorize=true`→S10；false 时等
  `authorization_timeout_ms` 超时→S12。S9/S10 仍走 `EvaluateVisualServoIntent`
  保持伺服/瞄准，不因状态切换产生意图空窗。默认关闭（生产安全）。
- **拦截后返航（S11~S14，2026-10-19 V1 单发口径）**：
  - **完成判据**：S10 内 `net_fired_`（触发命令成功入队）或发射重试超时
    （`ReturnHomeConfig::fire_timeout`）→ S11；**V1 不判断命中**，只记录
    “已发射/发射失败”，随后 S11 当拍/下一拍进入 S12。摄像头状态不参与返航判断。
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
  - **S13/S14**：S13 发 `MAV_CMD_DO_SET_MODE`（AUTO.LAND，4/6）；停止位置意图；
    落地后若仍 armed 发 `MAV_CMD_COMPONENT_ARM_DISARM`（0）；确认 disarmed →
    S14。正常返航不使用 PX4 自动 RTL；PX4 已接管则只监控 landed/disarmed。
  - **配置**：`MissionStateMachineConfig::return_home`（`config::ReturnHomeConfig`，
    默认 `enabled=true`；`interception`/`return_home` 两节已接入 JSON 加载）。
    捕网执行器隔离用例显式 `enabled=false`。
  - **依赖**：状态机为 Home 换算链接 `drone_perception`（纯函数 `GeodeticToLocalNed`）。
  - **边界**：S9/S10 入口已配置化（`interception` 节，默认关闭；射程占位待实机）；
    V1 返航段可通过测试钩子/测试用例或显式开启配置驱动。多发命中判据/再交战策略
    V2 立项；`MissionStatus` 结果字段暂不新增（用状态+日志）。
- `SetCommandSender`/`SetEstimationDiagnostics`须在 Start 前调用；运行中注入/重绑输入被拒绝并记 ERROR。
- `ForceStateForTest`为测试入口（**运行期门控**）：与生产路径共用`TransitionTo`
  统一迁移挂点，用于驱动拦截序列（S9/S10/S11）的进入/离开动作。**门控**：
  `MissionStateMachineConfig::test_hooks_enabled` 默认 false（生产装配语义，
  `DroneApplication` 不置位），此时调用**直接拒绝并记 ERROR、不产生任何迁移**——
  强推到 S9/S10 会在 `enable_control=true` 装配下经命令窄接口发出**不可逆的捕网
  触发命令**；单测与 SITL 验收工具显式置 true。生产侧另有 `interception.enabled`
  显式开关（默认 false）。
- 构造期对 `actuator` 配置执行 `Validate()` 兜底（`SendNetActuatorStroke` 以
  `params[net_channel-1]` 索引固定 7 元素参数数组：程序化构造的 channel 越界即 UB、
  channel=7 静默无效）；配置加载期已校验，此处对程序化构造 fail-fast，先例见
  FlightController 构造期 period 防御。
- 控制输出是否接 PX4 仍由`DroneApplication`的`enable_control`门禁决定（绑定
  `Px4Setpoint`+注入快照源+注入命令接口只在 true 分支）；诊断接口注入与
  `enable_control` 无关（只读诊断，影子期即可用）。

## 日志行为

- INFO：状态机创建、启动、停止、销毁；状态转移（含交接进入/验证通过/撤销/超时/
  视觉断流退出/测距失效退出/退出迟滞回退/伺服未启动超时，一一对应中文 reason，
  便于事后区分是哪条退出路径生效）；位置引导意图启动/停止（含"交接冻结最后位置目标"、
  "交接冻结位置意图续发"、"位置引导意图停止: 视觉伺服接续"、交接段收口）；
  S7 视觉伺服意图启动（带首拍 vx/vy/vz/yaw_rate 摘要）；PX4 命令接口与估计器
  诊断接口注入；首次/节流接收地面站目标；**返航开始/爬升完成/降落阶段/
  AUTO.LAND 入队/DISARM 入队/任务结束（S14）**（S11~S14，2026-10-19）。
- WARN：启动前未绑定输入；目标过期；估计失效导致位置意图停止（节流）；导航状态丢失；
  S7 视觉伺服停发意图（控制律降级，节流，带降级原因）；S7 热备份断粮（节流，不退出 S7）；
  **捕网执行器触发/复位命令已入队**（状态变化级事件，带通道/行程/归一化值摘要）；
  捕网命令入队失败（节流，下拍重试）；**单发发射超时/返航超时/Home 或锚点无效/
  PX4 接管返航/AUTO.LAND 或 DISARM 入队失败**（S11~S14，节流）。
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
- 捕网执行器（Task 10，`FakeCommandSender` 假件按调用顺序记录 SendCommand
  （命令/参数/入队结果）并可脚本化入队失败；用例经 `ForceStateForTest`（配置
  置 `test_hooks_enabled=true`）驱动拦截序列——与生产路径共用 TransitionTo/
  决策线程挂点）：
  - `InterceptExecutingFiresNetActuator`：进入 S10 → 恰好一条 187 命令
    （p3=归一化触发行程 −0.8、其余参数位 NaN、p7=0），停留多拍不重复；
    离开（S10→S11）→ 复位行程（+0.8）且只发一次；再次进入 → 新一次触发；
  - `NetActuatorFiringRetriesWhenEnqueueFails`：前两次入队失败 → 下拍重试直至
    成功（成功入队后不再补发，重试内容一致）；
  - `NetActuatorResetRetriesWhenEnqueueFails`：复位前两次入队失败 → 下拍重试
    直至成功（按调用顺序断言：重试期间只反复位行程、不混入触发；成功入队后
    不再补发）；
  - `NetActuatorTriggerWaitsForResetEnqueued`：复位连续三次入队失败（挂起重试）
    期间重进 S10 → 复位未成功入队前不发触发，触发排在复位成功入队之后
    （逐条断言命令参数与入队结果，钉住"先复位、后触发"在异常重试下的次序）；
  - `LeavingInterceptSequenceSendsResetStroke`：S9 不触发；S9→返航离开 →
    发复位行程（不依赖是否触发过）；
  - `NoCommandSenderNeverFires`：未注入命令接口 → 即使进入拦截态也零调用；
  - `ForceStateForTestRejectedWithoutTestHooks`：`test_hooks_enabled=false`（生产
    装配默认）时强制迁移被拒绝并记 ERROR、状态不变、已注入的命令接口零调用
    （防"生产路径误触发射网"）。
  - 负对照（一次性注入 3 处破坏：未用参数位填 0、入队失败也置位、离开不发复位）
    → 恰好对应 3 个用例失败，恢复后全绿；评审修复另作 2 处次序负对照——触发
    判据前移到复位挂起检查之前 → `NetActuatorTriggerWaitsForResetEnqueued`
    失败；复位入队失败即清除挂起（不重试）→ 两个复位用例失败。
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
- 配置（`interception`/`return_home` 节）：默认值、显式解析、非法窗口/
  返航高度拒绝、`interception.enabled=true` 且 `return_home.enabled=false`
  拒绝（发射后必须有返航收口）。
- SITL 返航专项（2026-10-19，`tools/px4_link_smoke --sitl-stage4-return` +
  `config/px4_sitl_return.json`）：S7→S9→S10 发射→S12 算力板返航 Home→
  S13 AUTO.LAND→S14 DISARMED 全 PASS（状态机迁移 11 次、错误 0；最大高度
  4.99m；S12 起始射程约 11.6m）；证据 `tools/return_evidence/`。该场景为
  非门禁，`interception.enabled` 生产默认关闭。

## 排查/修改要点

- **S10 发射后不返航**：查 `MissionStateMachineConfig::return_home.enabled`
  （默认 true；捕网执行器隔离测试显式 false）、`net_fired_`/发射超时日志；
  S9/S10 生产入口仍未冻结，整机只能经测试钩子进入。
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
  via Actuator Set N」（N 与 `actuator.net_channel` 对应）且已解锁/预解锁
  （PX4 默认在 disarmed 下不驱动执行器，台架用 pre-arm 状态测）；② 核对
  `actuator.output_pwm_min_us/max_us` 与该通道 `PWM_AUX_MINn/MAXn` 是否一致
  （不一致则归一化值对应的实际 PWM 偏移）；③ 行程方向以 **G4 台架**为准
  （本层只保证换算与命令语义正确，方向属装机接线问题）；日志中的
  `pwm/norm` 摘要即换算式输入输出，可反算核对。
- 离开拦截序列后机构停在触发位：查「复位行程命令入队失败（下拍重试）」节流
  WARN——入队失败会持续重试直至成功；若从未出现复位日志，确认离开时是否
  真的到过 S9/S10（`net_sequence_active_` 只在对序列内状态驻留过之后才置位）。
- 后续恢复完整自动链时，不能直接从`READY`跳到`GPS_APPROACH`输出控制；必须补齐`ARMING/TAKEOFF`门禁、SITL、拆桨台架验证和`enable_control`安全配置。
