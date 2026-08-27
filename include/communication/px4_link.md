# PX4 通信链路（px4_link）

> 对应实现：`include/communication/px4_link.h`、`src/communication/px4_link.cpp`
> 协议基础：`include/communication/mavlink_handler.md`
> 串口基础：`include/communication/serial_port.md`

## 1. 功能职责

`Px4Link` 是 Pixhawk/PX4 MAVLink 物理链路的唯一拥有者。当前第一阶段已实现：

- 根据 `Px4LinkConfig` 打开 PX4 串口；
- 启动一条独占通信线程，只有该线程执行串口读写；
- 通过 `MavlinkHandler` 增量解析 MAVLink 1/2；
- 周期发送机载电脑 `HEARTBEAT`；
- 接收并筛选目标 PX4 `HEARTBEAT`；
- 维护 connected、armed、PX4 main/sub mode 和 system status；
- 解析固件版本/能力、落地、全局/局部位置、姿态、GPS、电池、Home 和 RC；
- 心跳超时后将连接状态置为断开，各类遥测按 `telemetry_timeout` 独立失效；
- 周期及状态变化时发布 `Topic<FlightStateSnapshot>`；
- Start/Stop 幂等，停止后可以重新启动。

当前不做：`Px4Setpoint` 编码发送、`COMMAND_LONG/COMMAND_ACK` 队列和关联、串口运行时
自动重连。这些是后续阶段任务，接口和配置已预留。

`Px4LinkStub` 继续保留，仅供骨架冒烟测试，不参与真实硬件通信。

## 2. 接口与数据流

```cpp
struct Px4LinkConfig {
    SerialPortConfig serial;
    std::string firmware_version;  // 当前为 1.17.0
    uint8_t onboard_system_id;
    uint8_t onboard_component_id;
    uint8_t target_system_id;
    uint8_t target_component_id;
    uint8_t mavlink_version;
    std::chrono::milliseconds heartbeat_send_interval;
    std::chrono::milliseconds heartbeat_timeout;
    std::chrono::milliseconds telemetry_timeout;
    std::chrono::milliseconds state_publish_interval;
    std::chrono::milliseconds reconnect_interval;
    std::size_t setpoint_queue_capacity;
};

class Px4Link : public IPx4Link {
public:
    explicit Px4Link(Px4LinkConfig config);
    bool Start();
    void Stop();
    bool IsRunning() const;
    bool IsConnected() const;
    void SetInput(Topic<Px4Setpoint>& setpoint);
    Topic<FlightStateSnapshot>& StateOutput();
};
```

当前接收数据流：

```text
PX4 串口
  → SerialPort::Read
  → MavlinkHandler::Feed
  → 目标 system/component ID 过滤
  → 心跳/版本/飞行遥测解码与新鲜度校验
  → FlightStateSnapshot
  → kFlightState Topic
  → 后续状态机/控制/感知融合
```

当前发送数据流：

```text
Px4Link 周期调度
  → 机载电脑 HEARTBEAT pack_status
  → MavlinkHandler::Encode
  → SerialPort::Write
  → PX4
```

`SetInput()` 必须在 `Start()` 前调用。当前仅建立容量可配置、丢旧留新的设定值订阅；实际
`SET_POSITION_TARGET_LOCAL_NED` 发送将在下一阶段实现。

## 3. 关键实现点

### 3.1 独占线程

通信线程按以下顺序循环：

1. 带超时读取串口；
2. 增量解析收到的 MAVLink 帧；
3. 检查 PX4 心跳超时；
4. 到期发送机载电脑心跳；
5. 到期发布飞行状态快照。

其他线程不得访问 `SerialPort::Write()`。后续设定值和命令也只能进入 Topic/有界队列，由本线程发送。

### 3.2 身份过滤

只有同时匹配以下配置的消息进入 PX4 业务处理：

```text
message.sysid  == target_system_id
message.compid == target_component_id
```

HEARTBEAT 还必须满足：

```text
heartbeat.autopilot == MAV_AUTOPILOT_PX4
```

这样可避免同一 MAVLink 总线上其他组件或地面站心跳被误判为飞控连接。

### 3.3 PX4 1.17.0 模式解析

`HEARTBEAT.custom_mode` 按 PX4 1.17.0 格式保存并拆分：

```text
bits 16..23 → flight_mode（main mode）
bits 24..31 → flight_sub_mode（sub mode）
```

同时保留原始 `base_mode` 和 `custom_mode`，避免上层丢失版本适配信息。armed 来自
`MAV_MODE_FLAG_SAFETY_ARMED`。PX4 1.17.0 在旧模式基础上增加了 `TERMINATION`、
`ALTITUDE_CRUISE`、`AUTO_VTOL_TAKEOFF`、`POSCTL_SLOW` 等值，因此业务层不得只按旧版
枚举范围判断合法性。ACK 不能代替这些真实状态。

### 3.4 快照有效性

已解析字段包括：

- 心跳：`connected/armed/base_mode/custom_mode/flight_mode/flight_sub_mode/system_status`；
- 落地：`landed`，仅 `landed_state_valid=true` 时可用；
- 导航：GPS fix、全局位置、局部 NED 位置和 NED 速度；
- 姿态：roll/pitch/yaw；
- 动力电池：电压、电流、剩余量及各自有效标志；
- Home：经纬度与 MSL 高度；
- RC：链路有效性、在线状态和 RSSI，不自动等同人工接管；
- 固件：`flight_sw_version`、语义版本、capabilities。

各类周期遥测使用 `telemetry_timeout` 判断新鲜度。超时后保留最后数值供排查，但将对应 valid
标志清零，控制模块只能使用 valid=true 的字段。Home 在心跳连接期间保持有效，但连接断开时
立即失效；固件版本属于诊断信息，可保留到链路重新 Start 时统一重置。

快照头：

- `receive_time_ms`：算力板单调时钟发布时间；
- `valid_for_ms`：心跳超时配置；
- `source_id`：PX4 target system ID；
- `health`：连接时 1，断开时 3；
- `source_time_ms=0`：HEARTBEAT 不提供可靠源时间，不伪造。

### 3.5 停机与重启

`Stop()` 先清除运行标志，等待串口 `poll()` 在 `read_timeout` 内返回并退出线程，然后关闭串口。
若停止前已连接，会发布最终 `connected=false` 快照。再次 `Start()` 时重置 MAVLink 半包、发送序号、
状态快照和心跳时间，不恢复旧控制状态。

## 4. 日志行为

| 等级 | 场景 |
|---|---|
| INFO | 创建/销毁、启动/停止、首次 PX4 心跳、固件版本上报、正常停止时连接置断 |
| WARN | PX4 心跳超时、上报固件与配置不一致、当前阶段调用未实现的 `SendCommand`（节流） |
| ERROR | 启动打开串口失败、运行期读写错误统计、Start 后错误绑定 Topic（节流） |

不逐帧打印 HEARTBEAT 或状态快照，避免高频刷屏。底层串口错误由 `SerialPort` 记录一次，
`Px4Link` 只累计链路级错误，不重复打印同一调用链错误。

## 5. 测试方式

测试：`tests/communication/px4_link_test.cpp`，使用 Linux PTY 模拟 Pixhawk 串口，不依赖硬件。

覆盖：

- 配置身份和 MAVLink 版本校验；
- 启动后发送标准机载电脑 HEARTBEAT；
- 接收 PX4 HEARTBEAT 后连接；
- armed、main/sub mode、system status 解码；
- 落地、GPS、全局/局部位置、姿态、电池、Home、RC、固件版本与 capabilities；
- `FlightStateSnapshot` 发布、消息头和 NED frame ID；
- 心跳超时断开并发布快照；
- 遥测超时后有效标志清零但心跳连接保持；
- 忽略非目标 system ID；
- Stop 后重新 Start；
- 命令发送在当前阶段明确返回未实现。

运行：

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
./build/px4_link_test
ctest --test-dir build --output-on-failure
```

真实硬件使用独立 `px4_link_smoke` 验证，详见 `tools/px4_link_smoke.md`。该程序只发送机载电脑
HEARTBEAT，不发送任何控制命令，可在正式 main 全量装配前验证 `/dev/ttyS1`、波特率、目标 ID
和 PX4 实际消息流。

## 6. 排查/修改要点

| 现象 | 排查方向 |
|---|---|
| `Start()` 返回 false | JSON 设备名、权限、波特率、UART 是否被占用 |
| 能收到字节但不连接 | target system/component ID 或 `MAV_AUTOPILOT_PX4` 不匹配 |
| 心跳连接反复变化 | 波特率/接线不稳定，或 `heartbeat_timeout` 过短 |
| 模式数值异常 | 检查 PX4 固件是否为 1.17.0，以及 custom mode 位布局 |
| Stop 延迟 | `serial.read_timeout_ms` 决定最坏退出等待时间 |
| 重启后旧控制恢复 | 属于缺陷；Start 必须清空状态和后续的设定值/命令缓存 |
| 收到其他 MAVLink 设备消息 | 当前按目标 ID 过滤；未来多组件遥测需明确白名单后扩展 |

下一阶段扩展遥测时必须同步更新 `FlightStateSnapshot` 有效性语义、本文档、
`docs/通信与数据定义.md` 和对应录制字节流/PTY 测试。
