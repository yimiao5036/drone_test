# 捕网执行器 ACK 复位时序设计

> 日期：2026-10-21
>
> 状态：设计已确认，待实现
>
> 范围：单发捕网执行器的“触发 ACK → 保持 → 复位 → 复位 ACK → 稳定 → Ready”时序、故障处置，以及多发所需接口预留。
>
> 相关实现：`include/state_machine/net_actuator.h`、`src/state_machine/net_actuator.cpp`、`src/state_machine/mission_state_machine.cpp`、`include/communication/px4_link.h`、`src/communication/px4_link.cpp`、`include/config/config.h`、`src/config/config.cpp`。

## 1. 背景

当前 V1 单发流程：

```text
进入 S10 → Trigger() 入队一次
  → 同拍 S10→S11（不判命中）
  → 下一决策拍离开 S9/S10，EvaluateNetActuator 发复位
```

复位与触发之间只隔约一个 100ms 决策周期，没有“保持时间”概念，也不观察 PX4 的 `COMMAND_ACK`。这带来两个问题：

1. 当前捕网枪说明书要求：**遥控器控制时，发射保持 1~2 秒后回到待机**。约 100ms 的触发行程可能不足以完成发射。
2. 触发/复位命令是否被 PX4 接受无法确认；命令排队或链路异常时，可能出现“返航过程中才发出触发行程”的风险。

已确认的装机事实（维护者提供）：

- 捕网枪接 PX4 **MAIN5**；
- MAIN5 Output Function = **`Peripheral via Actuator Set 1`**；
- `PWM_MAIN_MIN5/MAX5` 已在 QGC 配置；
- 机载 `actuator.net_channel=1`（Actuator Set 编号，不是物理通道号 5）；
- 说明书：**发射保持 1~2 秒后回到待机**。

安全基线保持不变：

```json
"runtime.enable_control": false,
"interception.enabled": false
```

## 2. 目标与非目标

### 2.1 目标

- 以 **PX4 触发 ACK ACCEPTED** 为计时起点，保持 `reset_delay_ms`（默认 1500ms）后再发复位；
- 以 **复位 ACK ACCEPTED** 为起点，等待 `reset_settle_ms` 后标记 `Ready()`；
- 单发允许复位发生在 S11/S12 返航期间；单发不要求复位后才能返航；
- 触发或复位 **没有 ACK / ACK 非 ACCEPTED / 等待期间断连**，视为飞控或指令契约异常：不重试、不继续任务，停止控制交 PX4/遥控；
- 为多发预留 `Ready()` / `RemainingShots()` / `Fired()` / `Faulted()` 接口，本次不实现多发策略。

### 2.2 非目标

- 多发通道/弹仓推进、真实弹药计数硬件；
- 命中/脱靶判定、同一目标补射、下一目标再交战；
- S16 的悬停/返航策略（交 PX4 failsafe / 遥控）；
- 亚 100ms 精确计时；
- PX4 侧 PWM 输出功能、`PWM_MAIN_MIN5/MAX5`、舵机行程参数修改。

## 3. 关键决策

| 决策 | 结论 | 理由 |
|---|---|---|
| 复位计时起点 | 触发 ACK ACCEPTED，而不是入队时刻 | 说明书要求 1~2s 保持；ACK 才代表 PX4 接受命令 |
| 默认保持时间 | `reset_delay_ms=1500` | 说明书 1~2s 的区间中值；G4-3 实测后冻结 |
| 复位完成判定 | 复位 ACK ACCEPTED + `reset_settle_ms` | 用户选择；ACK 确认 PX4 接受，稳定时间覆盖机构回程 |
| 无 ACK 处置 | 严重故障，不重试；停止控制交 PX4/遥控 | 正常飞控+正常链路下不应缺失 ACK；缺失只可能是指令错误/断连/PX4 异常 |
| 时序归属 | 执行器内部状态机 | 多发时复用同一契约，状态机不感知机构结构 |
| 本次范围 | 复位时序 + 接口预留 | 多发策略、命中判定、下一目标另立规格 |

## 4. 接口设计

### 4.1 `IPx4CommandSender` 增加 ACK 状态与故障清理

```cpp
struct CommandAckStatus {
    uint64_t matched_count = 0;        ///< 累计匹配 ACK 数（含 IN_PROGRESS）
    uint64_t timeout_count = 0;        ///< 累计 ACK 超时数
    uint16_t last_command = 0;         ///< 最近匹配 ACK 的命令
    uint8_t  last_result = 0;          ///< MAV_RESULT（0=ACCEPTED）
    uint64_t last_time_ms = 0;         ///< 最近 ACK 接收时刻（单调 ms）
    uint16_t last_timeout_command = 0; ///< 最近 ACK 超时的命令
};

class IPx4CommandSender {
public:
    virtual bool SendCommand(uint16_t command, float param1, ..., float param7) = 0;
    virtual uint64_t AckMatchCount() const = 0;      // 保留兼容
    virtual uint64_t AckTimeoutCount() const = 0;    // 保留兼容
    virtual CommandAckStatus AckStatus() const = 0;
    /// 故障处置：清空机载排队与在途命令，防止迟到的触发行程在返航时发出。
    virtual void CancelPendingCommands() = 0;
};
```

`Px4Link` 实现要点：

- `AckStatus()` 返回线程安全快照；`last_command/result/time` 在匹配 ACK 时更新，`last_timeout_command` 在 `CheckCommandTimeout()` 中更新；
- `CancelPendingCommands()` **不在调用线程直接改在途状态**，而是置原子 `cancel_pending_` 标志；PX4 通信线程下一次循环先执行 `ClearCommands()`（清空排队 + 在途），再跳过发送。串口读超时 20ms，取消延迟 ≤ 一次读超时；
- 断连/心跳超时已有的 `ClearCommands()` 行为保持不变；
- `Px4LinkStub` 返回默认 `CommandAckStatus`，`CancelPendingCommands()` 为空实现（骨架占位）。

### 4.2 `INetActuator` 扩展

```cpp
struct ActuatorFeedback {
    bool px4_connected = false;
    CommandAckStatus ack;
    uint64_t now_ms = 0;
};

class INetActuator {
public:
    virtual bool Trigger() = 0;                       ///< 入队触发；false=未就绪/入队失败
    virtual bool Reset() = 0;                         ///< 手动/测试复位；正常周期由实现内部发
    virtual void Update(const ActuatorFeedback&) = 0; ///< 每决策拍调用
    virtual bool Ready() const = 0;                   ///< 可发下一发
    virtual uint32_t RemainingShots() const = 0;      ///< 剩余发数（single_pwm：1→0）
    virtual bool Fired() const = 0;                   ///< 本发触发已入队（不代表已 ACK）
    virtual bool Faulted() const = 0;                 ///< ACK 契约破坏后的终态
    virtual const char* FaultReason() const = 0;
    virtual const char* Type() const = 0;
};
```

`single_pwm` 语义：

- 初始：`Ready()=true`、`RemainingShots()=1`、`Fired()=false`、`Faulted()=false`；
- `Trigger()` 成功入队后：`Fired()=true`、`RemainingShots()=0`、`Ready()=false`；
- 复位 ACK + `reset_settle_ms` 后：`Ready()=true`，但 `RemainingShots()` 仍为 0（复位不等于补弹）；
- `Faulted()=true` 后为终态，忽略后续 `Trigger/Reset/Update`；
- `Reset()` 用于测试/人工强制归位：未故障时立即发待机行程、取消当前保持/稳定计时并回到 `Ready`，**不恢复剩余发数**；故障态调用返回 false。正常周期由内部计时调用，不依赖状态机在离开 S9/S10 时调用。

### 4.3 `ActuatorConfig` 新增字段

```cpp
struct ActuatorConfig {
    std::string type = "single_pwm";
    int32_t net_channel = 1;
    double net_fire_norm = -0.7;
    double net_idle_norm = 0.0;
    std::chrono::milliseconds reset_delay{1500};  ///< 触发 ACK 后保持时间
    std::chrono::milliseconds reset_settle{0};    ///< 复位 ACK 后稳定时间
    void Validate() const;
};
```

JSON：

```json
"actuator": {
  "type": "single_pwm",
  "net_channel": 1,
  "net_fire_norm": -0.7,
  "net_idle_norm": 0.0,
  "reset_delay_ms": 1500,
  "reset_settle_ms": 0
}
```

校验：

- `reset_delay_ms`、`reset_settle_ms` 必须为整数且 ≥0；
- `type=single_pwm` 时，若 `reset_delay_ms` 不在 `[1000, 2000]`，启动时打 **WARN**（不拒绝）：
  「当前捕网枪说明书要求发射保持 1~2s，reset_delay_ms=… 请以 G4-3 实测行程冻结」；
- 在 G4-3 冻结前，`interception.enabled` 必须保持 `false`。

## 5. 执行器状态机

```text
                 Trigger() 入队成功
Idle ─────────────────────────────────► FireQueued
 ▲                                          │ 收到 187 ACCEPTED
 │                                          ▼
 │                                      FireAcked
 │                                          │ 保持 reset_delay_ms
 │                                          ▼
 │                                      ResetQueued ◄── 发送复位
 │                                          │ 收到 187 ACCEPTED
 │                                          ▼
 │                                      ResetAcked
 │                                          │ 等待 reset_settle_ms
 │                                          ▼
 └──────────────────────────────────────  Ready（RemainingShots 仍为 0）
```

### 5.1 ACK 处理规则

| 事件 | 动作 |
|---|---|
| `matched_count` 增加且 `last_command=187`、`last_result=ACCEPTED` | FireQueued→FireAcked / ResetQueued→ResetAcked |
| `last_command=187`、`last_result=IN_PROGRESS` | 保持等待；PX4 会刷新在途命令超时 |
| `last_command=187`、其他结果 | 进入 Faulted（不重试） |
| `timeout_count` 增加且 `last_timeout_command=187` | 进入 Faulted |
| 等待 ACK 或等待复位发送期间 `px4_connected=false` | 进入 Faulted |
| 无关命令的 ACK/超时 | 只更新基线，不推进、不故障 |

### 5.2 计时精度

- `Update()` 由状态机 100ms 决策拍驱动；
- `reset_delay_ms` / `reset_settle_ms` 的实际误差 ≤1 拍（约 100ms）；
- `reset_delay_ms=1500` 时，实际保持约 1.5~1.6s，仍在说明书 1~2s 范围内；
- 本期不引入独立定时线程。

### 5.3 触发入队失败

- `Trigger()` 返回 false：
  - `feedback.px4_connected=false` → 执行器进入 Faulted；
  - 已连接但命令队列满 → 保持 Idle，下拍重试，不置 `Fired`、不扣发数；
- 状态机在 S10 持续未 `Fired()` 达到 `return_home.fire_timeout_ms`（默认 3s）→ 按任务故障处置（S16），不再按旧逻辑正常返航。

## 6. 状态机集成

### 6.1 决策拍顺序调整

把执行器反馈更新/故障检查提前，触发判断放在 S9→S10 迁移之后：

```text
Evaluate(now_ms):
  1. UpdateNetActuator()        // Update(feedback) + Faulted 检查
     └─ 故障 → HandleMissionFault(...) → 跳过后续任务逻辑，仍发布 MissionStatus
  2. 导航/目标新鲜度、交接评估
  3. EvaluateInterceptSequence() // 可能 S7→S9→S10
  4. MaybeTriggerNetActuator()   // S10 && Ready && RemainingShots>0 → Trigger()
  5. 位置/视觉伺服意图
  6. EvaluateInterceptionCompletion() // Fired → S11；未 Fired 超 fire_timeout → 故障
  7. EvaluateReturnHome / EvaluateLandingAndDisarm
  8. SetWarnings / PublishStatus
```

- 故障检查提前，避免故障当拍仍发布控制意图；
- `Update()` 每拍都调用（包括 S11/S12/S13），因此返航期间复位计时继续；
- 删除状态机成员 `net_fired_`、`net_reset_pending_`、`net_sequence_active_`；
- `EvaluateInterceptionCompletion()` 用 `net_actuator_->Fired()` 判断 S10→S11，不再等 ACK；
- 状态机销毁/进程退出时不再补发复位（单发可接受）。

### 6.2 单发时间线

```text
t0         S10 入口，Trigger() 入队；Fired=true，RemainingShots=0
t0 同拍    S10→S11（不判命中）
t1         执行器收到触发 ACK ACCEPTED → FireAcked
t1+1.5s    发复位（默认 reset_delay_ms=1500）
t2         执行器收到复位 ACK ACCEPTED → ResetAcked
t2+0ms     默认 reset_settle_ms=0 → Ready=true（剩余发数仍为 0）
```

复位可以发生在 S11/S12 返航期间；不要求等待复位完成才返航。

### 6.3 多发预留

未来多发状态机只需：

```cpp
if (state == S10 && net_actuator_->Ready() &&
    net_actuator_->RemainingShots() > 0) {
    net_actuator_->Trigger();
}
```

通道/弹仓推进、剩余发数递减由执行器实现内部完成。本次不实现 `S11→再交战`、命中判定或下一目标选择。

## 7. 故障与安全处置

### 7.1 故障判定与动作

```text
执行器 Faulted 或 S10 触发入队超时
  → kWarningActuatorFault = 1U << 4U（active_warning_bits bit4）
  → ERROR 日志（只打一次：原因、type、set、norm、累计计数）
  → command_sender_->CancelPendingCommands()
  → TransitionTo(S16 FAILSAFE, "捕网执行器 ACK 契约破坏/触发失败")
  → 停止发布一切 ControlIntent
  → 不重试、不补发复位、不自动恢复
```

停止意图后的收口沿用现有链路：

```text
最后一条意图超龄（controller.intent_timeout，默认 500ms）
  → COAST → IDLE → 停止 setpoint
  → PX4 Offboard 丢失保护 / 遥控接管
```

### 7.2 迟到触发行程防护

`CancelPendingCommands()` 清空机载**排队 + 在途**命令，重点防止：

- 触发命令排在其它命令后面，ACK 超时后队列继续推进；
- 返航过程中才把排队中的触发行程发出去。

已经写入串口/飞控的命令无法撤回；该边界由 PX4 自身行为与 G4 台架验证覆盖，文档中明确写出。

### 7.3 S16 最小实现

本次只增加：

- `TransitionTo(kFailsafe, ...)`；
- 故障标志阻止后续 Trigger/Reset 与意图发布；
- `MissionStatus.state=kFailsafe`、`active_warning_bits` bit4 可被地面站观测；
- 不实现 S16 悬停/返航策略，交 PX4 failsafe / 遥控 / 人工。

S16 为终态；V1 不自动恢复、不自动接受第二次任务。

## 8. 配置与安全边界

- 生产 `config/config.json`：`reset_delay_ms=1500`、`reset_settle_ms=0`；
- `enable_control=false` 时执行器为 nullptr，零命令、零影响；
- `interception.enabled=false` 时不进入 S9/S10，新增逻辑不触达；
- G4-3 台架确认 1~2s 保持、复位行程/方向后，才允许现场本地副本开启 `interception.enabled`；
- 打开 `enable_control` 仍需 G4/G5 与前置清单全部完成。

## 9. 测试计划

### 9.1 新增 `net_actuator_test`

使用 FakeCommandSender 与显式 `now_ms`，不依赖真实线程/睡眠：

- 初始 `Ready=true`、`RemainingShots=1`、`Fired=false`；
- Trigger 入队成功 → `Fired=true`、剩余 0、重复 Trigger 被忽略；
- 触发 ACK ACCEPTED → 保持 `reset_delay` → 发复位；`reset_delay=0` 时同拍发；
- 复位 ACK ACCEPTED → 等待 `reset_settle` → `Ready=true`；
- `IN_PROGRESS` 保持等待，最终 ACCEPTED 推进；
- 非 ACCEPTED、187 ACK 超时、等待期间断连 → `Faulted`，之后零命令；
- 无关命令 ACK/超时不影响状态机；
- 已连接入队失败 → 下拍重试；断开入队失败 → Faulted；
- 手动 `Reset()` 取消保持/稳定计时、回到 Ready，但不恢复剩余发数；Faulted 后 `Reset()` 返回 false。

### 9.2 状态机/通信/配置

- S10→S11 依据 `Fired()`（入队），不等 ACK；
- 复位在 S12 返航期间由执行器计时完成；
- 故障 → S16、停止意图、bit4、`CancelPendingCommands()` 恰好一次、不再触发/复位；
- 触发 3s 未入队 → S16（不再走正常返航）；
- `Px4Link::AckStatus()` 的 ACK/超时/最近超时命令字段；
- `CancelPendingCommands()` 清空排队+在途（PTY 用例）；
- 配置解析、缺失默认值、负数拒绝、单发范围 WARN。

测试数量增加后，同步仓库中 10 处测试基线数字。

## 10. 文档同步

- `include/state_machine/net_actuator.md`：状态机、时序、ACK/故障规则、配置；
- `include/state_machine/mission_state_machine.md`：S10→S11、S16、bit4、`CancelPendingCommands`；
- `include/communication/px4_link.md`：`AckStatus()` / `CancelPendingCommands()`；
- `include/config/配置管理.md`：`reset_delay_ms` / `reset_settle_ms`；
- `docs/状态机设计.md`：S10/S11/S16 最小实现与故障口径；
- `docs/数据接口文档.md` §3.4、`docs/地面站目标位置与时间同步协议.md` §14.3：bit4；
- `docs/模块索引.md`、`docs/开发进度.md`、`docs/PX4实链验证清单.md`（G4-3 增加 ACK/保持时序项）；
- `docs/项目总览与数据流.md`：捕网执行器时序与 S16 故障口径。

## 11. 风险与后续

1. **机构实际保持/回程时间未知**：默认 1500ms 来自说明书 1~2s 中值，必须由 G4-3 实测冻结；`reset_settle_ms` 目前为 0，多发前需实测回程时间。
2. **ACK 关联假设**：拦截窗口内只有 187 命令，`AckStatus` 的“最近 ACK”足够；未来若在同一窗口引入其它命令，需要升级为按命令/序列的 ACK 关联。
3. **100ms 计时精度**：保持时间误差约 ±1 拍；如需更精确，后续可改为执行器独立定时。
4. **进程提前退出**：若在复位计时完成前退出，不再补发复位；单发可接受，多发需在任务中止流程中统一处理。
5. **多发/命中/下一目标**：本次仅预留接口，不作为本规格实现内容；后续单独立项。
