# PX4 只读串口冒烟测试（px4_link_smoke）

> 对应实现：`tools/px4_link_smoke.cpp`
> 使用模块：`SerialPort`、`MavlinkHandler`、`Px4Link`、`FlightStateSnapshot`

## 1. 功能职责

该程序用于在香橙派上尽早验证真实 Pixhawk/PX4 串口，不依赖视频、YOLO、图传、状态机和控制器。

测试内容：

- 严格加载 JSON 中的机载电脑身份、PX4 目标身份、固件版本和串口参数；
- 打开 `/dev/ttyS1`，启动真实 `Px4Link`；
- 发送标准机载电脑 HEARTBEAT；
- 接收 PX4 HEARTBEAT 和已配置输出的遥测；
- 每秒打印一次状态摘要，连接/armed/模式变化时立即打印；
- 到期或 Ctrl+C 后停止并输出 PASS/WARN/FAIL 验收报告；
- 根据基础链路结果返回进程退出码。

安全边界：程序不绑定 `Px4Setpoint`，不会发送解锁、模式切换、起飞、降落或 Offboard 控制。
主动发送内容仅包括机载电脑 HEARTBEAT、`MAV_CMD_REQUEST_MESSAGE` 和
`MAV_CMD_SET_MESSAGE_INTERVAL`，用于请求遥测及其频率。

## 2. 构建与运行

香橙派仓库目录中执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target px4_link_smoke -j6
./build/px4_link_smoke --config config/config.json --duration 30
echo "退出码=$?"
```

构建目标会把 `config/` 复制到 `build/config/`，因此也可直接执行：

```bash
./build/px4_link_smoke --duration 30
```

参数：

| 参数 | 说明 |
|---|---|
| `--config <路径>` | 显式指定 JSON；缺省按“可执行文件旁 config → 当前目录 config”查找 |
| `--duration <秒>` | 测试时长，范围 1~3600，默认 30 秒 |
| `--help` | 显示帮助 |

## 3. 当前配置

```json
"mavlink": {
    "onboard_system_id": 1,
    "onboard_component_id": 191
},
"px4": {
    "firmware_version": "1.17.0",
    "target_system_id": 1,
    "target_component_id": 1,
    "mavlink_version": 2,
    "command_ack_timeout_ms": 1000,
    "command_queue_capacity": 16,
    "one_shot_message_requests": [148, 242],
    "message_interval_requests": [
        {"message_id": 30, "interval_us": 100000},
        {"message_id": 32, "interval_us": 100000}
    ],
    "serial": {
        "device": "/dev/ttyS1",
        "baud_rate": 115200,
        "data_bits": 8,
        "stop_bits": 1,
        "parity": "N"
    }
}
```

波特率必须与 PX4 对应 MAVLink 实例一致。若实机不是 115200，只修改 JSON 后重新运行，不改代码。

## 4. 上板前检查

### 4.1 设备与权限

```bash
ls -l /dev/ttyS1
groups
```

当前用户应有串口读写权限，通常属于 `dialout` 组。若不属于：

```bash
sudo usermod -aG dialout "$USER"
```

随后注销并重新登录。不要长期使用 sudo 运行测试，否则日志目录可能产生 root 所有文件。

### 4.2 检查串口是否被系统控制台占用

```bash
sudo lsof /dev/ttyS1
systemctl status serial-getty@ttyS1.service
```

只有确认 `serial-getty@ttyS1` 正在占用该 PX4 专用串口时，才执行：

```bash
sudo systemctl disable --now serial-getty@ttyS1.service
```

### 4.3 硬件安全

- 首次联调拆除螺旋桨；
- 确认香橙派 UART 与 Pixhawk TELEM 接口均为兼容 TTL 电平；
- TX/RX 交叉连接并共地；
- 不把 RS-232 电平直接接到 TTL UART；
- 遥控器保持可用，PX4 不允许自动解锁。

## 5. 输出与退出码

运行中摘要示例：

```text
[3s] connected=是 armed=否 mode=3/0 landed=是 gps=3D+
     global=是 local=是 attitude=是 battery=是 home=是 rc=是
```

最终报告：

```text
[PASS] PX4 HEARTBEAT
[PASS] 测试结束时心跳仍连接
[PASS] 遥测请求 COMMAND_ACK
[PASS] AUTOPILOT_VERSION 收到 1 条
[PASS] ATTITUDE
...
结果: 基础 PX4 串口链路通过
```

退出码：

| 退出码 | 含义 |
|---|---|
| 0 | PX4 心跳建立、结束时仍连接且链路错误为 0 |
| 1 | 参数、配置文件或 JSON 解析失败 |
| 2 | 串口打开/线程启动失败 |
| 3 | 未收到 PX4 心跳、结束时断开或链路出现错误 |

缺少某一遥测消息只记 WARN，不直接判定基础串口失败。它通常表示 PX4 对应 MAVLink 实例没有
配置该消息流，后续需要设置流频率或由 `Px4Link` 主动请求。

## 6. 日志行为

- 控制台：每秒状态摘要、状态变化、最终验收报告；
- 文件日志：沿用 `config.log` 配置和工程异步日志；
- 不逐条打印高频姿态/位置消息；
- `Px4Link` 仅记录生命周期、连接变化、版本首次上报和节流错误。

## 7. 排查要点

| 现象 | 排查方向 |
|---|---|
| 无法打开 `/dev/ttyS1` | 设备节点、dialout 权限、serial-getty/lsof 占用 |
| 串口打开但 HEARTBEAT FAIL | TX/RX、共地、波特率、PX4 MAVLink 端口、target ID |
| HEARTBEAT 反复断开 | 接线/电平、波特率、心跳频率、系统负载 |
| 只有 HEARTBEAT，其他全 WARN | PX4 MAVLink stream 配置或尚未主动请求消息 |
| 版本 WARN | PX4 未主动发送 AUTOPILOT_VERSION；后续命令阶段主动请求 |
| 收到消息但数值无效 | GPS/估计器/电池/RC 本身未就绪，检查 valid 标志而非默认零值 |
| 退出码 3 且错误计数增加 | 查看 `logs/` 中串口 poll/read/write 节流错误 |

测试后请保存完整控制台输出和对应日志文件，供后续调整 PX4 1.17.0 消息流与串口参数。

## 8. 首次香橙派实测结果（2026-08-27）

```text
设备：/dev/ttyS1，115200，MAVLink 2
目标：system/component = 1/1
时长：30 秒
接收目标消息：763
ACK：首次版本尚未实现请求
链路错误：0
退出码：0
结果：基础 PX4 串口链路通过
```

- HEARTBEAT 全程稳定，实测模式 `4/3` 对应 PX4 1.17.0 `AUTO/LOITER`，飞控未解锁；
- 已收到 `EXTENDED_SYS_STATE`、`LOCAL_POSITION_NED`、`ATTITUDE`、`GPS_RAW_INT`；
- GPS 消息存在但没有 3D fix，因此 `GLOBAL_POSITION_INT`、Home 不可用符合当前台架条件；
- `AUTOPILOT_VERSION`、电池、Home、RC 未收到或未形成有效状态，需要主动请求或检查硬件；
- landed/local 偶尔显示未知，是消息间隔超过 `telemetry_timeout_ms=2000`，心跳并未断开。

下一轮 smoke 已具备 `MAV_CMD_REQUEST_MESSAGE`、`MAV_CMD_SET_MESSAGE_INTERVAL`、ACK
匹配/超时和每类 message ID 计数，可区分“消息未配置”“消息已收到但字段无效”和“解析缺陷”。

## 9. 第二次香橙派实测结果（2026-08-27）

```text
时长：30 秒
ACK 匹配：10
ACK 超时：0
sequence 1：REQUEST_MESSAGE(AUTOPILOT_VERSION)，result=0 ACCEPTED
sequence 2：REQUEST_MESSAGE(HOME_POSITION)，result=2 DENIED（当前无 GPS/Home）
sequence 3~10：SET_MESSAGE_INTERVAL，全部 result=0 ACCEPTED
AUTOPILOT_VERSION：1 条，版本 1.17.0，capabilities=59647
EXTENDED_SYS_STATE：17 条
LOCAL_POSITION_NED：85 条
ATTITUDE：86 条
GPS_RAW_INT：42 条，无 3D fix
SYS_STATUS/BATTERY_STATUS：18 条，但电池字段无效
GLOBAL_POSITION_INT / HOME_POSITION / RC_CHANNELS：未收到
链路错误：0
退出码：0
```

结论：PX4 1.17.0 接受版本请求和全部 8 条频率请求；Home 请求被明确拒绝，符合当前没有
GPS 3D fix、尚未建立 Home 的状态。可靠命令/ACK 链路实机通过；
落地和局部位置不再因 2 秒超时反复失效。无 GPS 3D fix 时全局位置/Home 缺失符合预期。
实机测试时未插动力电池、未开启遥控器，因此电池消息字段无效和 RC 未收到属于预期状态；
后续接入硬件后再验证。PX4 `mavlink status` 确认对应 instance #0 为 PX4 内部
`/dev/ttyS0 @115200`（对端香橙派 `/dev/ttyS1`），mode=Normal、tx rate max=1200 B/s、
rate multiplier=0.283，解释了实际消息频率约为请求值的三分之一。

## 10. 第三次香橙派实测结果：提高 MAVLink 带宽（2026-08-27）

将 PX4 参数 `MAV_0_RATE` 从 1200 调整为 0，使 PX4 按 115200 波特率自动使用约
`baudrate/20 = 5760 B/s` 上限。30 秒结果：

```text
EXTENDED_SYS_STATE：59 条（约 2 Hz）
LOCAL_POSITION_NED：294 条（约 10 Hz）
ATTITUDE：304 条（约 10 Hz）
GPS_RAW_INT：150 条（5 Hz）
SYS_STATUS/BATTERY_STATUS：60 条（两类各约 1 Hz）
总接收目标消息：1674
ACK 匹配：10
ACK 超时：0
链路错误：0
```

请求频率已全部达到预期，且 30 秒内状态有效性稳定。无全局位置/Home、无有效电池和无 RC
分别与无 GPS 3D fix、未插动力电池、未开启遥控器一致。当前 115200 串口带宽满足基础遥测，
PX4 接收与安全命令阶段验收完成。
