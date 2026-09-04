# 任务状态机实现文档

## 功能职责

`MissionStateMachine`负责消费地面站目标、PX4飞行状态和健康状态快照，按任务阶段维护`MissionStatus`。当前实现是安全影子阶段：验证`GroundStationTarget`已经能进入任务链路，并在满足导航快照基本条件时进入`GPS_APPROACH`影子状态。

本阶段不做以下事情：

- 不发送PX4命令、模式切换、解锁、起飞、降落或setpoint。
- 不发布非空自动控制意图；`IntentOutput()`仅保留给后续控制器接入。
- 不直接调用拦截设备。
- 不把地面站目标透传为PX4航点。

完整自动任务链仍以`docs/状态机设计.md`为准；当前跳过`ARMING/TAKEOFF`只用于无控制输出的地面站目标接入验证，后续接入控制门禁后需要恢复完整状态推进。

## 接口与数据流

```cpp
class MissionStateMachine final : public IMissionStateMachine {
public:
    void SetInputs(Topic<GroundStationTarget>& ground_target,
                   Topic<FlightStateSnapshot>& flight,
                   Topic<HealthStatus>& health) override;
    Topic<ControlIntent>& IntentOutput() override;
    Topic<MissionStatus>& StatusOutput() override;
};
```

当前主程序接线：

```text
GroundStationLink::TargetOutput()
  -> MissionStateMachine::SetInputs(...)
  -> MissionStateMachine::StatusOutput()
  -> GroundStationLink::SetMissionStatusInput(...)
```

飞行状态来自：

```text
Px4Link::StateOutput() -> MissionStateMachine
```

健康状态Topic当前由主程序预留，后续接入`HealthManager`后替换为真实发布源。

## 关键实现点

- 状态机拥有独立工作线程，周期为100ms。
- 输入通过`Topic`订阅队列获取，目标队列容量4，飞行和健康快照队列容量2。
- 每个周期先抽干订阅队列，只保留最新不可变快照，再做状态判断，避免一次决策周期内读到不一致数据。
- 启动后先发布`BOOT`，随后进入`SELF_CHECK`。
- `SELF_CHECK`阶段等待PX4快照满足：连接在线、GPS fix有效、全局位置有效、Home有效。
- `READY`阶段等待有效地面站目标；目标有效且导航仍有效时，进入`GPS_APPROACH`影子状态。
- `GPS_APPROACH`阶段只维护目标新鲜度和状态回传，不生成位置控制意图。目标短暂过期会置告警，后续策略再决定返航/等待。
- 所有控制输出保持断开，`DroneApplication`仍不会绑定`Px4Setpoint`。

## 日志行为

- INFO：状态机创建、启动、停止、销毁；状态转移；首次/节流接收地面站目标。
- WARN：启动前未绑定输入；目标过期；导航状态丢失。
- ERROR：订阅或线程启动异常。
- 高频周期不打印日志；重复异常使用第1次和每100次节流。

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
- `IntentOutput()`不发布自动控制意图。

## 排查/修改要点

- 如果地面站目标ACK已成功但状态机不进入`GPS_APPROACH`，先检查PX4快照是否满足`connected/gps_fix/global_position_valid/home_valid`。
- 如果状态机没有状态回传，检查主程序是否启用了PX4和地面站链路，以及`GroundStationLink::TargetOutput()`是否已绑定到状态机。
- 后续恢复完整自动链时，不能直接从`READY`跳到`GPS_APPROACH`输出控制；必须补齐`ARMING/TAKEOFF`门禁、SITL、拆桨台架验证和`enable_control`安全配置。
