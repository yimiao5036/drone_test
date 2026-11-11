# 通用拦截设备抽象（interception_device）

> 对应实现：`include/state_machine/interception_device.h`、`src/state_machine/interception_device.cpp`
> 使用方：`MissionStateMachine`（每拍 `Update`；S10 发起、设备自行计时复位；
> 故障由状态机映射为 S16）、`GcsNetFire`（65016 直控发射）
> 前身：`net_actuator.h/.cpp`（2026-11-11 抽象为通用接口，行为不变）

## 1. 功能边界

状态机与地面站直控服务只依赖 `IInterceptionDevice` 的**通用动作语义**，
不感知具体执行机构：

| `InterceptMode` | 含义 | 当前实现 |
|---|---|---|
| `kPulse` | 一次性动作：执行 → 保持 → 复位 | `single_pwm` 捕网枪（已实现） |
| `kHold` | 持续作用：开始 → 保持 → 停止 | 干扰（预留） |
| `kNone` | 无执行器：末端由飞控执行 | 撞击（预留） |

接口方法：

```cpp
class IInterceptionDevice {
    virtual InterceptMode Mode() const = 0;
    virtual bool Ready() const = 0;
    virtual bool Execute() = 0;                       // 发起动作
    virtual bool Stop() = 0;                          // 停止/复位
    virtual void Update(const InterceptFeedback&) = 0; // 每拍推进时序
    virtual bool Triggered() const = 0;               // 动作已发起过
    virtual uint32_t RemainingUses() const = 0;
    virtual bool Faulted() const = 0;
    virtual const char* FaultReason() const = 0;
    virtual const char* Type() const = 0;
};
```

`Execute()/Stop()` 返回 `false` 表示链路未就绪/命令队列满/状态不允许：
调用方保留 pending 并在下一拍重试；`true` 只代表命令入队，不代表机构已动作。

## 2. 内部时序（single_pwm / kPulse）

```text
Ready --Execute() 入队成功--> FireQueued
FireQueued --187 ACK ACCEPTED--> FireAcked --(同拍)--> Holding
Holding --now >= ACK 时刻 + reset_delay--> ResetQueued（发复位行程）
ResetQueued --187 ACK ACCEPTED--> ResetAcked --(同拍)--> Settling
Settling --now >= ACK 时刻 + reset_settle--> Ready

FireQueued/Holding/ResetQueued --PX4 断连--> Faulted
任意等待态 --187 ACK 超时 / 非 ACCEPTED（IN_PROGRESS 除外）--> Faulted
Faulted：终态，Update/Execute/Stop 均不再产生命令
```

- **计时起点**：对应命令 ACK 的 `last_time_ms`；为 0 时回退本拍 `now_ms`。
- **IN_PROGRESS（5）**：保持原等待态，继续等最终 ACK，不判故障。
- **ACK 关联（187 专用）**：只消费 `CommandAckStatus` 的
  `do_set_actuator_ack_count / last_result / last_time_ms / timeout_count`，
  不使用全局字段。复位等待横跨 S12~S14，期间 AUTO.LAND(176)/DISARM(400)
  的 ACK 不得覆盖或推进本时序。
- **新 ACK 识别**：以 187 专用计数基线识别；基线必须在命令入队**之前**抓取
  （PX4 worker 可能在 `SendCommand()` 返回前完成往返）。
- **发数**：`Execute()` 成功入队才扣次数（1→0）；入队失败保留 pending，
  下拍重试，次数不变。复位不恢复次数，故单发设备全生命周期只动作一次。
- **手动 `Stop()`**：取消进行中的保持/复位周期，立即发复位行程并回到 Ready，
  但不恢复次数；Faulted 时返回 false（故障不自动清除）。状态机正常流程
  （含 S16 故障处置）不调用，也不补发复位。

## 3. 配置

| 键 | 默认 | 说明 |
|---|---|---|
| `actuator.type` | `single_pwm` | 设备实现类型（kPulse）；`kHold/kNone` 待接入 |
| `actuator.net_channel` | `1` | PX4 Actuator Set N（1..6），不是物理通道号 |
| `actuator.net_fire_norm` | `-0.7` | 触发行程归一化量 |
| `actuator.net_idle_norm` | `0.0` | 待机/复位行程归一化量 |
| `actuator.reset_delay_ms` | `1500` | 触发 ACK 后的保持时间 |
| `actuator.reset_settle_ms` | `0` | 复位 ACK 后的稳定时间 |

捕网枪说明书要求发射保持 1~2s：`single_pwm` 的 `reset_delay` 超出
[1000, 2000]ms 时 `CreateInterceptionDevice()` 打一条启动 WARN，但不阻止启动。
**2026-10-23 维护者确认**：机构为舵机、纯 PWM 驱动，保持 **1500ms** 即生产冻结值；
有效射程 = 双目测距极限 ≤8m。

## 4. PX4 / 机载边界

- **PX4/QGC**：物理输出 Output Function 设为 `Peripheral via Actuator Set N`，
  并设置该通道 PWM 映射区间（MAIN 为 `PWM_MAIN_MINn/MAXn`，AUX 为
  `PWM_AUX_MINn/MAXn`，标准 1000/2000µs）；舵机行程与方向由映射和安装决定；
- **当前装机（2026-10-21/10-28）**：捕网接 **MAIN5**，Output Function =
  `Peripheral via Actuator Set 1`，`PWM_MAIN_MIN5=1000` / `MAX5=2000`、
  `DIS5=1500`、`Rev Range=off`；机载 `net_channel=1`；
- **机载**：只配置 Actuator Set N 与归一化量；`MAV_CMD_DO_SET_ACTUATOR`
  只有对应 Set 位填值，其余 NaN，p7=0；
- 单发捕网：`-0.7` → 1150µs（发射），`0.0` → 1500µs（待机）。

## 5. 日志与排查

- 入队成功（WARN，低频动作日志）：
  `拦截设备触发/复位行程命令已入队: type=single_pwm mode=pulse set=N norm=...`
- 入队失败：首次与每满 100 次节流
  `WARN 拦截设备...命令入队失败（下拍重试）: type=... count=...`
- 故障（ERROR，终态）：
  `拦截设备故障: type=single_pwm set=N reason=<原因>`
  原因含：`等待 187 ACK 超时`、`187 ACK 结果非 ACCEPTED`、
  `等待 PX4 ACK 期间连接断开`、`执行器通道越界`。
- 机构不动/反向：先查 MAIN5 输出功能与 `PWM_MAIN_MIN5/MAX5` 及接线；
- 触发后不进入复位：查 187 ACK 是否到达、`reset_delay` 是否过大；
- 未注入命令接口（`enable_control=false`）时零日志、零命令，属预期影子行为。

## 6. 测试

- `InterceptionDeviceTest.*`（`tests/state_machine/interception_device_test.cpp`，
  共享 `FakeCommandSender`）：初始 Ready/单发；动作参数与扣次数；ACK 后保持
  `reset_delay` 再复位；复位 ACK + `reset_settle` 回 Ready；IN_PROGRESS 保持；
  拒绝 ACK / 187 超时 / 等待期断连 → Faulted 且不补发；无关 ACK/超时忽略；
  同一拍 176 ACK 不吞 187；手动 `Stop()` 取消周期且不补发；入队失败重试
  不扣次数、复位重试不混入触发行程；`reset_delay` 越界告警。
- 状态机集成（`mission_state_machine_test`）：S10 触发一次、停留不重复、
  触发 ACK 后复位（状态保持 S10）、S12 中 ACK 后仍复位、入队失败重试、未触发
  离开零命令、ACK 超时/断连/入队超时 → S16 + bit4 + 取消一次、未注入命令接口
  零调用。
- 命令参数断言：`net_channel=1` 时 p1=`-0.7`（触发）/`0.0`（复位），其余 NaN，p7=0。
