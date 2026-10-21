# 捕网执行器抽象（net_actuator）

> 对应实现：`include/state_machine/net_actuator.h`、`src/state_machine/net_actuator.cpp`
> 使用方：`MissionStateMachine`（S10 触发、离开拦截序列复位）

## 1. 功能边界

状态机只依赖 `INetActuator` 的“触发一次/复位到待机”语义，不感知单发/多发、
通道数、弹仓或物理接线。当前实现：

| type | 行为 |
|---|---|
| `single_pwm` | 经 `IPx4CommandSender` 发 `MAV_CMD_DO_SET_ACTUATOR`；配置归一化 `net_fire_norm=-0.7`、`net_idle_norm=0.0` |

未来多发实现（多个 Actuator Set、转盘/弹仓、多发时序）实现同一接口，并在
`CreateNetActuator()` 按 `actuator.type` 注册即可；状态机 S9/S10/S11 流程不改。

## 2. 接口

```cpp
class INetActuator {
    virtual bool Trigger() = 0;  // 触发一次/下一发；true=命令已入队
    virtual bool Reset() = 0;    // 复位到待机/待命行程
    virtual const char* Type() const = 0;
};
```

- `false` 表示链路未就绪/命令队列满；状态机保留 `net_fired_`/`net_reset_pending_`
  并在下一拍重试。
- `true` 只代表命令入队，不代表机构已动作；ACK/到位由 PX4 与 G4 台架确认。
- `SetCommandSender(nullptr)` 或 `enable_control=false` 时不创建执行器，功能物理缺席。

## 3. PX4 / 机载边界

- **PX4/QGC**：把 AUX 输出功能设为 `Peripheral via Actuator Set N`，设置
  `PWM_AUX_MINn/MAXn`（标准 1000/2000µs），核对舵机行程/方向；
- **机载**：只配置 `actuator.net_channel`（Actuator Set N）与归一化量；
  `MAV_CMD_DO_SET_ACTUATOR` 的 p1..p6 只有对应 Set 位填值，其余 NaN，p7=0；
- 单发捕网：`-0.7` → 1150µs（发射），`0.0` → 1500µs（待机）；
- 机载不再保存 PWM 微秒/输出区间，避免与 PX4 参数重复维护。

## 4. 日志与排查

- 成功：`WARN 捕网执行器触发/复位行程命令已入队: type=single_pwm set=N norm=...`
- 失败：首次与每 100 次节流 `WARN ...入队失败（下拍重试）`
- 机构不动/反向：先查 PX4 输出功能、Actuator Set N、`PWM_AUX_MINn/MAXn`
  与接线；机载只保证归一化命令语义。
- 未注入命令接口（`enable_control=false`）时零日志、零命令，属预期影子行为。

## 5. 测试

- `ConfigTest.*Actuator*`：类型、Set 范围、归一化值范围/相等拒绝；
- `MissionStateMachineTest.*NetActuator*`：S10 触发一次、离开复位、失败重试、
  复位未入队前不发触发、再次进入重新触发；
- 命令参数断言：Set 3 的 p3=`-0.7`（触发）/`0.0`（复位），其余 NaN，p7=0。
