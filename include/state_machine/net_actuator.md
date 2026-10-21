# 捕网执行器抽象（net_actuator）

> 对应实现：`include/state_machine/net_actuator.h`、`src/state_machine/net_actuator.cpp`
> 使用方：`MissionStateMachine`（S10 触发、离开拦截序列复位；ACK 反馈接线在
> Task 5 接入，当前状态机只调用 `Trigger()/Reset()`）

## 1. 功能边界

状态机只依赖 `INetActuator` 的“触发一次/复位到待机”语义与内部时序推进，
不感知单发/多发、通道数、弹仓或物理接线。当前实现：

| type | 行为 |
|---|---|
| `single_pwm` | 经 `IPx4CommandSender` 发 `MAV_CMD_DO_SET_ACTUATOR`（187）；配置 `net_channel=1`（Actuator Set 1）、归一化 `net_fire_norm=-0.7`、`net_idle_norm=0.0`；单发 `RemainingShots 1→0` |

未来多发实现（多个 Actuator Set、转盘/弹仓、多发时序）实现同一接口，并在
`CreateNetActuator()` 按 `actuator.type` 注册即可；状态机 S9/S10/S11 流程不改。

## 2. 内部时序状态机（single_pwm）

计时逻辑全部在 `src/state_machine/net_actuator.cpp`，状态机只每拍转发
`ActuatorFeedback{px4_connected, ack, now_ms}` 并观察
`Ready()/Fired()/Faulted()/FaultReason()`。

```text
Ready --Trigger() 入队成功--> FireQueued
FireQueued --187 ACK ACCEPTED--> FireAcked --(同拍)--> Holding
Holding --now >= ACK 时刻 + reset_delay--> ResetQueued（发复位行程）
ResetQueued --187 ACK ACCEPTED--> ResetAcked --(同拍)--> Settling
Settling --now >= ACK 时刻 + reset_settle--> Ready

FireQueued/Holding/ResetQueued --PX4 断连--> Faulted
任意等待态 --187 ACK 超时 / 非 ACCEPTED（IN_PROGRESS 除外）--> Faulted
Faulted：终态，Update/Trigger/Reset 均不再产生命令
```

- **计时起点**：对应命令 ACK 的 `last_time_ms`（PX4 接收时刻）；为 0 时回退
  本拍 `now_ms`。保持/稳定截止均以 ACK 时间为基准，不受决策拍抖动影响。
- **IN_PROGRESS（5）**：保持原等待态，继续等待最终 ACK，不判故障。
- **无关 ACK**：`last_command != 187` 的新 ACK 被忽略，不推进也不判故障。
- **ACK 判定**：以 `matched_count/timeout_count` 基线识别“新到达”的 ACK，
  同一 ACK 不会被重复消费；不做 ACK 重试。
- **发数**：`Trigger()` 成功入队才扣发数（1→0）；入队失败保留 pending，
  调用方下拍重试，发数不变。复位不恢复发数，故单发执行器全生命周期只触发一次。
- **手动 `Reset()`**：取消进行中的保持/复位周期，立即发复位行程并回到 Ready，
  但不恢复发数；Faulted 时返回 false（故障不自动清除）。

## 3. 接口

```cpp
struct ActuatorFeedback {
    bool px4_connected = false;
    common::CommandAckStatus ack;
    uint64_t now_ms = 0;
};

class INetActuator {
    virtual bool Trigger() = 0;                       // 仅 Ready 且有发数时成功
    virtual bool Reset() = 0;                         // 手动归位；Faulted 拒绝
    virtual void Update(const ActuatorFeedback&) = 0; // 每拍推进内部时序
    virtual bool Ready() const = 0;
    virtual uint32_t RemainingShots() const = 0;      // 单发 1→0
    virtual bool Fired() const = 0;
    virtual bool Faulted() const = 0;
    virtual const char* FaultReason() const = 0;
    virtual const char* Type() const = 0;
};
```

- `Trigger()/Reset()` 返回 `false` 表示链路未就绪/命令队列满（或状态不允许）：
  调用方保留 pending 标志并在下一拍重试；`true` 只代表命令入队，不代表机构已动作。
- `SetCommandSender(nullptr)` 或 `enable_control=false` 时不创建执行器，功能物理缺席。

## 4. 配置

| 键 | 默认 | 说明 |
|---|---|---|
| `actuator.type` | `single_pwm` | 执行器实现类型 |
| `actuator.net_channel` | `1` | PX4 Actuator Set N（1..6），不是物理通道号 |
| `actuator.net_fire_norm` | `-0.7` | 触发行程归一化量 |
| `actuator.net_idle_norm` | `0.0` | 待机/复位行程归一化量 |
| `actuator.reset_delay` | `1500ms` | 触发 ACK 后的保持时间（`reset_delay_ms`） |
| `actuator.reset_settle` | `0ms` | 复位 ACK 后的稳定时间（`reset_settle_ms`） |

捕网枪说明书要求发射保持 1~2s：`single_pwm` 的 `reset_delay` 超出
[1000, 2000]ms 时 `CreateNetActuator()` 打一条启动 WARN，但不阻止启动；
最终行程以 G4-3 台架实测冻结。

## 5. PX4 / 机载边界

- **PX4/QGC**：把物理输出的 Output Function 设为 `Peripheral via Actuator Set N`，
  设置该输出通道的 PWM 映射区间（MAIN 输出为 `PWM_MAIN_MINn/MAXn`，AUX 输出为
  `PWM_AUX_MINn/MAXn`，标准 1000/2000µs），核对舵机行程/方向；
- **当前装机（2026-10-21 确认）**：捕网接 **MAIN5**，Output Function =
  `Peripheral via Actuator Set 1`，PWM 已配置；因此机载 `net_channel=1`。
  **注意 `net_channel` 是 Actuator Set 编号，不是物理通道号 5**；
- **机载**：只配置 `actuator.net_channel`（Actuator Set N）与归一化量；
  `MAV_CMD_DO_SET_ACTUATOR` 的 p1..p6 只有对应 Set 位填值，其余 NaN，p7=0；
- 单发捕网：`-0.7` → 1150µs（发射），`0.0` → 1500µs（待机，按 1000/2000µs
  标准映射计算；实际行程以 QGC 中 MAIN5 的 `PWM_MAIN_MIN5/MAX5` 为准）；
- 机载不再保存 PWM 微秒/输出区间，避免与 PX4 参数重复维护。

## 6. 日志与排查

- 成功入队（WARN，低频动作日志）：
  `捕网执行器触发/复位行程命令已入队: type=single_pwm set=N norm=...`
- 入队失败：首次与每满 100 次节流
  `WARN ...入队失败（下拍重试）: type=single_pwm set=N count=...`
- 故障（ERROR，终态）：
  `捕网执行器故障: type=single_pwm set=N reason=<原因>`
  原因含：`等待 187 ACK 超时`、`187 ACK 结果非 ACCEPTED`、
  `等待 PX4 ACK 期间连接断开`、`执行器通道越界`。
- 机构不动/反向：先查 MAIN5 的输出功能是否为 `Peripheral via Actuator Set 1`、
  `PWM_MAIN_MIN5/MAX5` 与接线；机载只保证归一化命令语义。
- 触发后一直不进入复位：查 187 ACK 是否到达（`px4_link` ACK 计数）、
  `reset_delay` 是否过大；ACK 超时/拒绝会直接 Faulted 等待人工处置。
- 未注入命令接口（`enable_control=false`）时零日志、零命令，属预期影子行为。

## 7. 测试

- `NetActuatorTest.*`（`tests/state_machine/net_actuator_test.cpp`，使用共享
  `FakeCommandSender`）：初始 Ready/单发；触发参数与扣发数；ACK 后保持
  `reset_delay` 再复位；复位 ACK + `reset_settle` 回 Ready；IN_PROGRESS 保持；
  拒绝 ACK / 187 超时 / 等待期断连 → Faulted 且不补发；无关 ACK 忽略；
  入队失败重试不扣发数、复位重试不混入触发行程；`reset_delay` 越界告警。
- `MissionStateMachineTest.*NetActuator*`：S10 触发一次、停留不重复、离开
  拦截序列复位、入队失败重试、未注入命令接口零调用（ACK 驱动时序集成待 Task 5）。
- 命令参数断言：`net_channel=1` 时 p1=`-0.7`（触发）/`0.0`（复位），其余 NaN，p7=0。
