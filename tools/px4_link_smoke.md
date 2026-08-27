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

安全边界：程序不绑定 `Px4Setpoint`，不调用 `SendCommand`，不会发送解锁、模式切换、起飞、
降落或 Offboard 控制指令。当前唯一主动发送消息是机载电脑 HEARTBEAT。

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
[WARN] AUTOPILOT_VERSION
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
