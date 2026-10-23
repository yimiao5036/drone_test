# GcsNetFire 地面站直控发射服务（G4-4）

## 功能职责

`GcsNetFire` 消费地面站 `65016 NET_FIRE` 指令，经落地/未解锁门禁后占用捕网执行器
（`INetActuator`，当前 `single_pwm`），并把 `65017 NET_FIRE_ACK` 回执交回
`GroundStationLink` 下行给地面站。

**做什么**：

- 单线程（20ms 节拍）消费 `kNetFireCommand`，逐条判定五个门禁并发布回执；
- 推进执行器时序（触发 ACK 确认 → 保持 `reset_delay` → 复位 → 稳定）并补发
  `kCompleted` 回执；
- 同一 `command_id` 幂等：重传帧回放上一次结论，绝不重复触发；
- 诊断计数 `AcceptedCount()`/`RejectedCount()`，仅日志使用，不接健康位。

**不做什么**（边界）：

- 不改变任务状态机：不推进 `MissionState`、不发布 `ControlIntent`、不发
  ARM/起飞/模式切换命令；发射与任务分配/自动起飞完全解耦；
- 不替代 `interception`（S9/S10 自动拦截授权链）：二者互斥，配置加载期
  `actuator.gcs_fire_enabled=true` 与 `interception.enabled=true` 同时出现直接抛
  `std::invalid_argument`；
- 不做多发/自动补发：`single_pwm` 触发一次后发数归零，需人工维护；
- 不做串口/MAVLink 编解码（在 `GroundStationLink`）。

## 接口与数据流

```text
地面站 255/190 ──65016──▶ GroundStationLink ──kNetFireCommand──▶ GcsNetFire
                                                                   │ 门禁
                                                    INetActuator::Trigger（DO_SET_ACTUATOR 187）
                                                                   │
GcsNetFire ──kNetFireAck──▶ GroundStationLink ──65017──▶ 地面站 255/190
```

| 接口 | 说明 |
|---|---|
| `GcsNetFire(config::ActuatorConfig, communication::IPx4CommandSender*)` | 构造即按 `actuator.type` 经 `CreateNetActuator` 创建执行器；`sender==nullptr` 时执行器缺席，全部请求按 `kActuatorUnavailable` 拒绝 |
| `SetFlightStateInput(Topic<FlightStateSnapshot>&)` | 绑定 `Px4Link::StateOutput()`，必须 `Start()` 前调用 |
| `SetCommandInput(Topic<NetFireCommand>&)` | 绑定 `GroundStationLink::NetFireCommandOutput()`，必须 `Start()` 前调用 |
| `AckOutput()` | `Topic<NetFireAck>`，接 `GroundStationLink::SetNetFireAckInput()` |
| `Start()/Stop()/IsRunning()` | 启动/停止工作线程，幂等；未绑定主题或执行器缺席时 `Start()` 返回 false |
| `AcceptedCount()/RejectedCount()` | 诊断计数（含重复帧回执） |

消息布局见 `docs/数据接口文档.md`（65016/65017 各 55 字节短帧，地址双字段在
50/51，54 为短帧标记；链路层已做长度/地址校验，服务层只做语义门禁）。

## 门禁与回执语义

按顺序判定，任一失败立即回 `REJECTED` + 原因码：

1. `protocol_version != 1` → `kUnsupportedVersion`；
2. 同一 `command_id` 已处理过 → 回 `DUPLICATE_COMMAND`（`result` 回放上次结论，
   **不触发**）；
3. `flags.bit0` 未置位 → `kConfirmationMissing`（`kNetFireFlagConfirmed`）；
4. 指令滞留超过 `header.valid_for_ms`（链路置 2000ms）→ `kCommandStale`；
5. 飞行状态缺失/超龄（>1s）/`landed_state_valid=false` → `kFlightStateStale`；
6. `armed=true` → `kArmed`；`landed=false` → `kNotLanded`；
7. 执行器缺席 → `kActuatorUnavailable`；终态故障 → `kActuatorFaulted`；
   未 Ready（时序中）→ `kActuatorBusy`；发数耗尽 → `kNoRemainingShots`；
8. 距上次成功触发不足 `gcs_fire_cooldown` → `kCooldown`（单发实现发数先耗尽，
   该门禁实际由多发实现触发）；
9. `Trigger()` 入队失败 → `kActuatorUnavailable`（链路未就绪/队列满）。

受理成功后：立即回 `ACCEPTED`，`last_fire_ms` 与 `pending_completion_` 记账；
执行器走完 `FireQueued → Hold → ResetQueued → Settle → Ready` 后，服务补发
`COMPLETED`（`actuator_state=0`）。执行器在时序中进入终态故障则补发
`REJECTED`+`kActuatorFaulted`。`actuator_state`：0=Ready、1=Busy、2=Faulted。

## 关键实现点

- **线程模型**：单工作线程，`steady_clock` 单调时钟，20ms 节拍；每拍先清空
  飞行状态订阅（容量 2，只留最新），再消费指令（容量 8，丢最旧），最后
  `UpdateActuator()` 推进时序并检查完成/故障。指令是人工低频操作，不设阻塞等待。
- **时钟同源**：判定使用本机单调时钟，与 `ActuatorFeedback.now_ms` 同源；
  65016 不携带地面站时间戳（`source_time_ms=0`），链路层收帧时打
  `received_ms`/`receive_time_ms`。
- **与装配门禁的关系**：`DroneApplication::BuildComponents()` 仅在
  `actuator.gcs_fire_enabled=true` 且 PX4/地面站链路均存在时创建本服务；
  `BindTopics()` 接线 `kNetFireCommand`/`kNetFireAck`。**与 `enable_control`
  无关**：直控发射是拆桨台架/测试通道，不经过飞行控制授权链，但仍受落地未解锁
  门禁约束。
- **反误触发**：65016 帧头与 payload 地址双字段必须同时匹配本机才发布
  （`GroundStationLink`），广播帧不会触发任何一架；`command_id` 幂等防止地面站
  重传导致二次点火；`flags.bit0` 必须显式确认。

## 日志行为

- `INFO`：启动/停止（含 type/channel/cooldown、accepted/rejected 计数）、重复指令
  回放、发射周期完成；
- `WARN`：每次拒绝（带 `command_id` 与原因码）、受理入队、`gcs_fire_enabled=false`
  时执行器不创建；
- `ERROR`：执行器终态故障（只打一次）、绑定缺失导致启动失败；
- 链路层收帧计数走 `ShouldLogThrottled` 节流；服务层指令低频，不额外节流。

## 测试方式

- 单元测试：`tests/state_machine/gcs_net_fire_test.cpp`（8 例：无飞行状态/已解锁/
  未落地/缺确认位/版本不支持拒绝，受理→完成回执，command_id 幂等，单发发数耗尽）。
  ```bash
  cmake --build build -j"$(nproc)" --target gcs_net_fire_test
  ./build/gcs_net_fire_test
  ```
- 配置校验：`tests/config/config_test.cpp` 中 `GcsNetFireDefaultsDisabled`/
  `RejectsGcsFireWithInterceptionBothEnabled`/`RejectsGcsFireCooldownOutOfRange`。
- 台架实跑（拆桨，必须）：本地副本把 `actuator.gcs_fire_enabled` 置 true 后启动
  正式程序，用 `python3 tools/net_fire_tester.py --connection <链路> --aircraft-system
  N --aircraft-component 25` 发送 65016，观察 65017 `ACCEPTED → COMPLETED` 与舵机
  行程；负向测试用 `--no-confirm`（预期 `CONFIRMATION_MISSING`）、`--repeat 3`
  （预期仅首次受理）。正式 `config/config.json` 保持 `gcs_fire_enabled=false`。

## 排查/修改要点

- **地面站收不到 65017**：确认 `actuator.gcs_fire_enabled=true` 且程序启动日志出现
  “地面站直控发射已装配”；未装配时 65016 由链路丢弃，不会回任何 ACK。
- **总是 `FLIGHT_STATE_STALE`**：PX4 未连接或无落地状态（`landed_state_valid`），
  台架需要 PX4 心跳与 LANDED 判定；不安装 PX4 时无法测试直控发射。
- **`ARMED`/`NOT_LANDED`**：先上锁/落地再发；解锁状态下本服务拒绝一切发射。
- **受理后一直无 `COMPLETED`**：PX4 侧 Output Function/PWM 映射或 ACK 链路问题，
  执行器停在 Hold/ResetQueued，超时会进终态故障（ERROR 日志带原因）。
- 修改 65016/65017 布局时必须同步：`include/common/types.h`、`GroundStationLink`
  编解码、`tools/net_fire_tester.py`、`docs/数据接口文档.md`、本文件。
