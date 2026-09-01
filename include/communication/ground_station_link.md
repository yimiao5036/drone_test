# 地面站数传链路实现文档

## 功能职责

`GroundStationLink`通过独立串口与自研地面站进行MAVLink 2通信。它订阅`Px4Link::StateOutput()`发布的`FlightStateSnapshot`，按配置频率重新编码为标准MAVLink遥测消息发送给地面站；同时接收并识别`MAV_TYPE_GCS`心跳，维护地面站在线状态。

本模块不是PX4原始字节透传器，不允许地面站绕过状态机直接控制PX4。目标位置上行协议尚未确定，当前只保留`TargetOutput()`接口，不解析或发布伪造目标。`MissionStatus`和`HealthStatus`输入接口已预留，但在自定义状态协议确定前不编码发送。

## 接口与数据流

```cpp
struct GroundStationLinkConfig;
class GroundStationLink final : public IGroundStationLink;

void SetFlightStateInput(Topic<FlightStateSnapshot>&);
void SetMissionStatusInput(Topic<MissionStatus>&); // 预留
void SetHealthInput(Topic<HealthStatus>&);          // 预留
Topic<GroundStationTarget>& TargetOutput();         // 预留
```

正式接线：

```text
Px4Link::StateOutput()
  → Topic<FlightStateSnapshot>的独立订阅队列
  → GroundStationLink独占串口线程
  → 标准MAVLink 2遥测
  → /dev/ttyS6（可在JSON修改）
  → 自研Vue3地面站
```

当前已运行实现仍复用`mavlink.onboard_system_id/onboard_component_id`，实机验证身份为`1/191`。多机阶段将按`docs/多机身份与功能寻址设计.md`拆分地面站链路身份：system ID表示同类型编号，component ID表示功能类别，捕网类使用25、火箭类使用26；PX4链路继续保持1/191→1/1。该设计尚未进入代码，修改前后必须保证两条链路配置互不覆盖。

当前下行消息：

| 内容 | MAVLink消息 | 默认周期 |
|---|---|---:|
| 在线、armed、PX4模式和系统状态 | `HEARTBEAT` | 1000ms |
| 姿态 | `ATTITUDE` | 100ms |
| 局部NED位置和速度 | `LOCAL_POSITION_NED` | 200ms |
| 经纬度、高度和速度 | `GLOBAL_POSITION_INT` | 200ms |
| GPS fix | `GPS_RAW_INT` | 500ms |
| landed状态 | `EXTENDED_SYS_STATE` | 500ms |
| 总电压、电流和剩余电量 | `SYS_STATUS` | 1000ms |
| 电池电流和剩余电量 | `BATTERY_STATUS` | 1000ms |
| Home点 | `HOME_POSITION` | 5000ms |

所有周期均在`ground_station.send_interval_ms`配置。状态首次到达及连接、armed、模式、landed、GPS、电池或Home有效性变化时，会立即发送关键状态，减少事件显示延迟。

## 关键实现点

- 一条串口只由`GroundStationLink`工作线程独占，统一复用`SerialPort`和独立`MavlinkHandler`，不与PX4链路共享解析/发送序号。
- `FlightStateSnapshot`订阅队列默认容量2，满时丢最旧状态，避免电台拥塞积压旧遥测。
- 只在对应有效标志为真时发送姿态、位置、GPS、电池和Home消息；无效数据不伪造成有效零值。
- `FlightStateSnapshot`只有动力电池总压，没有单体电压，因此总压放在`SYS_STATUS.voltage_battery`；`BATTERY_STATUS.voltages[]`保持未知，禁止把总压误当成首节电芯电压。
- `GLOBAL_POSITION_INT.relative_alt`仅在Home有效时由MSL高度差计算；Home无效时为0，地面站应结合Home有效消息判断。
- 接收方向目前只处理`MAV_TYPE_GCS`心跳。其他合法MAVLink消息计入接收统计但不执行，不存在地面站到PX4的控制转发。
- `Start()`前必须绑定飞行状态Topic；`Start/Stop`幂等，停止后可以重新启动。

## 日志行为

- INFO：部件创建/销毁、串口启动/停止、首次收到地面站心跳。
- WARN：地面站心跳超时、运行期降级。
- ERROR：配置/启动/绑定时序错误和发送失败；错误按第1次及每100次节流。
- 不逐条记录遥测消息，不在高频发送路径打印成功日志。

## 测试方式

Ubuntu 24.04开发机：

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
./build/ground_station_link_test
ctest --test-dir build --output-on-failure
```

`tests/communication/ground_station_link_test.cpp`使用Linux PTY覆盖：配置校验、未绑定拒绝启动、MAVLink 2机载身份心跳、飞行快照字段编码、GCS心跳建立/超时、幂等停止和重启。硬件阶段在香橙派确认实际串口6设备节点后修改JSON并验证Vue3地面站解码。

## 排查与修改要点

- 串口打不开：检查`ground_station.serial.device`是否为实际Linux节点、用户组权限和设备树串口使能；不需要修改代码。
- RK3588同一UART可有多组引脚复用。香橙派实测曾误启用`uart6-m2`而接线位于`uart6-m1`，表现为`/dev/ttyS6`可打开且`tx`计数增加，但排针无正确收发。必须保证设备树Overlay选择与实际TX/RX排针一致；当前实机使用`uart6-m1`。
- 地面站无数据：先抓取`HEARTBEAT`，核对MAVLink 2、115200 8N1和地面站解析器是否接受完整system/component二元身份。HM30 UDP实机参数为地面端`192.168.144.12:19856`；当前单机实现由Windows发送`255/190` GCS心跳、飞机下行`1/191`并已完成Vue3显示验证。多机实现后应看到捕网N号=`N/25`、火箭N号=`N/26`。
- 有心跳但无GPS/电池：检查PX4快照对应`*_valid`标志；本模块不会发送失效字段。
- 模式显示：读取机载心跳中的`base_mode/custom_mode`，其中`custom_mode`保持PX4原始编码。
- 修改发送频率只改JSON；不得在工作循环写死新周期。
- 增加目标协议时，在接收分支校验来源、字段范围、时间戳和有效期后发布`GroundStationTarget`，禁止直接生成PX4命令。
- 增加任务/健康自定义消息时同步更新dialect、Vue3解析器、测试和本文档。
