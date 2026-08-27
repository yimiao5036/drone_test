# PX4 通信链路（px4_link）

> 对应实现：`include/communication/px4_link.h`、`src/communication/px4_link.cpp`
> 协议基础：`include/communication/mavlink_handler.md`
> 串口基础：`include/communication/serial_port.md`

## 1. 功能职责

`Px4Link` 是 Pixhawk/PX4 MAVLink 链路的唯一拥有者，可通过 `ICommunicationTransport`
选择真实串口或 PX4 SITL UDP。当前已实现：

- 根据 `Px4LinkConfig.transport` 打开 PX4 串口或 UDP；
- 启动一条独占通信线程，只有该线程执行串口读写；
- 通过 `MavlinkHandler` 增量解析 MAVLink 1/2；
- 周期发送机载电脑 `HEARTBEAT`；
- 接收并筛选目标 PX4 `HEARTBEAT`；
- 维护 connected、armed、PX4 main/sub mode 和 system status；
- 解析固件版本/能力、落地、全局/局部位置、姿态、GPS、电池、Home 和 RC；
- 心跳超时后将连接状态置为断开，各类遥测按 `telemetry_timeout` 独立失效；
- 周期及状态变化时发布 `Topic<FlightStateSnapshot>`；
- Start/Stop 幂等，停止后可以重新启动。

当前不做：姿态设定值、全局坐标设定值和串口运行时自动重连。`Px4Setpoint` 的本地 NED
位置/速度发送、可靠 `COMMAND_LONG` 队列、ACK 和安全遥测请求已经实现。

`Px4LinkStub` 继续保留，仅供骨架冒烟测试，不参与真实硬件通信。

## 2. 接口与数据流

```cpp
struct Px4LinkConfig {
    std::string transport;  // serial / udp
    SerialPortConfig serial;
    UdpTransportConfig udp;
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
    std::chrono::milliseconds command_ack_timeout;
    std::chrono::milliseconds setpoint_send_interval;
    std::chrono::milliseconds setpoint_timeout;
    std::size_t setpoint_queue_capacity;
    std::size_t command_queue_capacity;
    std::vector<uint32_t> one_shot_message_requests;
    std::vector<MavlinkMessageIntervalRequest> message_interval_requests;
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
机载电脑 HEARTBEAT / SendCommand / 自动遥测请求
  → 有界命令队列（命令）
  → 单条在途 COMMAND_LONG
  → MavlinkHandler::Encode
  → SerialPort::Write
  → PX4
  → COMMAND_ACK 匹配/超时
```

`SetInput()` 必须在 `Start()` 前调用。订阅采用容量可配置、丢旧留新策略；通信线程缓存最新
有效设定值，并按 `setpoint_send_interval` 重复发送 `SET_POSITION_TARGET_LOCAL_NED`。

## 3. 关键实现点

### 3.1 独占线程

通信线程按以下顺序循环：

1. 带超时读取串口；
2. 增量解析收到的 MAVLink 帧；
3. 检查 PX4 心跳和命令 ACK 超时；
4. 发送下一条排队命令；
5. 到期发送机载电脑心跳；
6. 到期发布飞行状态快照。

其他线程不得访问 `SerialPort::Write()`。设定值和命令只能进入 Topic/有界队列，由本线程发送。

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

### 3.5 本地 NED 设定值

当前支持：

- `SetpointType::kPosition`：x/y/z 为 NED 位置（米）；
- `SetpointType::kVelocity`：x/y/z 为 NED 速度（m/s）；
- 可选 yaw 或 yaw rate，度/度每秒在发送前转换为弧度/弧度每秒；
- 坐标系必须为 `header.frame_id=3`（本地 NED）；
- 根据类型构造严格的 `POSITION_TARGET_TYPEMASK`，忽略未使用的位置/速度/加速度/yaw 字段。

安全规则：

- `valid=false` 清除缓存并停止发送，不会用空 mask 维持 Offboard；
- 刹停/悬停必须由控制器生成有效零速度或位置保持设定值；
- `kNone/kAttitude`、非 NED、NaN/Inf、同时启用 yaw 和 yaw rate 均拒绝；
- 有效期取 `min(header.valid_for_ms, setpoint_timeout)`；头部未给有效期时使用配置上限；
- 设定值收到时已过期、时间戳在未来或缓存过期时停止发送；
- 只在 PX4 心跳连接期间发送，断开/Stop/Start 都清除旧设定值。

### 3.6 可靠命令与安全遥测请求

`SendCommand()` 只负责把命令放入有界队列，返回 true 表示成功入队，不表示 PX4 已接受。
链路线程同一时刻只发送一条 `COMMAND_LONG` 并等待同 command ID 的 `COMMAND_ACK`：

- 最终 ACK 增加 `AckMatchCount()`，更新快照最近 ACK 后发送下一条；
- `MAV_RESULT_IN_PROGRESS` 保持在途并刷新超时时间；
- 超过 `command_ack_timeout` 增加 `AckTimeoutCount()`，丢弃该在途命令并继续队列；
- 队列满、未运行或未连接时拒绝外部命令；
- 心跳断开和 Stop 清空排队/在途命令，禁止重连后恢复旧命令。

首次建立心跳后按 JSON 自动排队：

- `MAV_CMD_REQUEST_MESSAGE`：请求 `AUTOPILOT_VERSION` 和 `HOME_POSITION`；
- `MAV_CMD_SET_MESSAGE_INTERVAL`：请求 SYS_STATUS、GPS、姿态、本地/全局位置、RC、电池和
  `EXTENDED_SYS_STATE` 的配置频率。

这些内部命令只改变本 MAVLink 链路的消息输出，不执行解锁、模式切换或飞行控制。

### 3.7 停机与重启

`Stop()` 先清除运行标志，等待串口 `poll()` 在 `read_timeout` 内返回并退出线程，然后关闭串口。
若停止前已连接，会发布最终 `connected=false` 快照。再次 `Start()` 时重置 MAVLink 半包、发送序号、
状态快照和心跳时间，不恢复旧控制状态。

## 4. 日志行为

| 等级 | 场景 |
|---|---|
| INFO | 创建/销毁、启动/停止、首次心跳、遥测请求入队、命令 ACK、固件版本、正常停止 |
| WARN | 心跳超时、固件不一致、命令队列满、ACK 超时、非法/过期设定值（均节流） |
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
- 未连接拒绝命令；
- `COMMAND_LONG` 参数编码、最终 ACK 匹配和快照更新；
- ACK 超时计数后继续队列；
- 首次心跳后按顺序发送单次消息请求和消息频率请求；
- 本地 NED 位置设定值、type mask 和 yaw 角转换；
- 本地 NED 速度设定值、type mask 和 yaw rate 转换；
- 过期设定值停止重发；
- 非 NED、姿态、NaN/Inf 和 yaw 选择冲突拒绝。

运行：

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
./build/px4_link_test
ctest --test-dir build --output-on-failure
```

真实硬件和 SITL 均使用独立 `px4_link_smoke` 验证，详见 `tools/px4_link_smoke.md` 和
`tools/PX4_SITL分级测试.md`。硬件模式只发送机载电脑
HEARTBEAT、`REQUEST_MESSAGE` 和 `SET_MESSAGE_INTERVAL`，不发送解锁、模式或飞行控制命令，
可在正式 main 全量装配前验证 `/dev/ttyS1`、ACK 和 PX4 实际消息流。

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

下一阶段必须先在 PX4 1.17.0 SITL 验证 Offboard 前置设定值流、位置/速度效果和设定值中断
保护，再允许拆桨台架测试。姿态/机体系/全局设定值需要独立接口评审后才能扩展。
