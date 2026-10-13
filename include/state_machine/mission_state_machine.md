# 任务状态机实现文档

## 功能职责

`MissionStateMachine`负责消费地面站目标、PX4飞行状态和健康状态快照，按任务阶段维护`MissionStatus`。默认仍是安全影子语义：验证`GroundStationTarget`已经能进入任务链路，并在满足导航快照基本条件时进入`GPS_APPROACH`影子状态。

阶段四新增（Task 7）：经`SetEstimationInputs`绑定目标估计/视觉稳定性/双目测距三个输入后，`GPS_APPROACH`（S5）开始发布`kPositionGuide`位置引导意图（指向估计目标 NED，`control_source=0`，`reason_code`带 bit31 控制权请求位`common::kControlIntentReasonRequestControl`）。**未绑定时不出意图、保持原影子行为**。意图是否真正转化为 PX4 输出由`FlightController`授权链与`enable_control`装配门禁决定，状态机侧不做最终裁决。

本阶段仍不做以下事情：

- 不发送PX4命令、模式切换、解锁、起飞、降落或setpoint（`SetCommandSender`注入的命令接口当前仅持有，供后续 ARM/执行机构任务使用）。
- 不直接调用拦截设备。
- 不把地面站目标透传为PX4航点。

完整自动任务链仍以`docs/状态机设计.md`为准；当前跳过`ARMING/TAKEOFF`只用于无控制输出的地面站目标接入验证，后续接入控制门禁后需要恢复完整状态推进。

## 接口与数据流

```cpp
struct MissionStateMachineConfig {
    std::chrono::milliseconds intent_valid_for{300};  // 意图有效期，与 controller.intent_timeout 同源
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

MissionStateMachine::IntentOutput() ─► FlightController::SetInput(...)
MissionStateMachine::StatusOutput() ─► GroundStationLink::SetMissionStatusInput(...)
Px4Link（IPx4CommandSender）─► SetCommandSender(...)（仅 enable_control=true 装配分支）
```

配置注入：`MissionStateMachineConfig.intent_valid_for`由`DroneApplication`从
`config_.controller.intent_timeout`同源传入，保证意图`valid_until_ms`与控制器
意图超龄门禁一致；默认 300ms。

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
- 视觉稳定性与测距输入当前只抽干保留最新快照，供后续交接（S6/S7）任务消费。
- `SetCommandSender`仅`enable_control=true`装配期调用；运行中注入/重绑输入被拒绝并记 ERROR。
- 控制输出是否接 PX4 仍由`DroneApplication`的`enable_control`门禁决定（绑定
  `Px4Setpoint`+注入快照源+注入命令接口只在 true 分支）。

## 日志行为

- INFO：状态机创建、启动、停止、销毁；状态转移；位置引导意图启动/停止；
  PX4 命令接口注入；首次/节流接收地面站目标。
- WARN：启动前未绑定输入；目标过期；估计失效导致意图停止（节流）；导航状态丢失。
- ERROR：订阅或线程启动异常；运行中重绑输入/注入命令接口；估计输入部分绑定。
- 高频周期（100ms 决策、10Hz 意图重发）不打印日志；重复异常使用第1次和每100次节流。

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

## 排查/修改要点

- 如果地面站目标ACK已成功但状态机不进入`GPS_APPROACH`，先检查PX4快照是否满足`connected/gps_fix/global_position_valid/home_valid`（PX4 已到位后该组条件可实链成立，不再是"必然不满足"）。
- 如果状态机没有状态回传，检查主程序是否启用了PX4和地面站链路，以及`GroundStationLink::TargetOutput()`是否已绑定到状态机。
- S5 不出意图：确认`SetEstimationInputs`三个输入都已绑定（部分绑定按未绑定处理并记 ERROR），且`TargetState.valid=true`且剩余有效期未过期。
- 意图被 FlightController 拒绝但状态机侧正常发布：查 reason_code bit31 是否置位、`valid_until_ms`是否超龄、`target_z`是否为 NaN（高度无效时有限性门禁拒绝属预期）。
- 后续恢复完整自动链时，不能直接从`READY`跳到`GPS_APPROACH`输出控制；必须补齐`ARMING/TAKEOFF`门禁、SITL、拆桨台架验证和`enable_control`安全配置。
