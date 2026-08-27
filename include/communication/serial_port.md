# 串口封装（serial_port）

> 对应实现：`include/communication/serial_port.h`、`src/communication/serial_port.cpp`
> 使用方：PX4 串口、地面站电台、激光雷达（后续实现）

## 1. 功能职责

所有串口链路的统一收发封装（Linux termios）：

- 打开/关闭、参数校验、超时读、全量写、清空缓冲区。
- 错误处理遵循 AGENTS.md：构造/打开失败抛异常；运行时读写错误返回码 + `ErrorCount()` 监控。

不做什么：不解析协议（MAVLink、雷达协议由上层实现）；不负责线程管理（每链路一线程，由调用方）。

## 2. 接口与数据流

```cpp
struct SerialPortConfig {
    std::string device = "/dev/ttyS0";   // 设备路径
    int baud_rate = 115200;              // 波特率（9600~921600）
    uint8_t data_bits = 8;               // 5/6/7/8
    uint8_t stop_bits = 1;               // 1/2
    char parity = 'N';                   // N/E/O
    std::chrono::milliseconds read_timeout{100};   // 单次读超时
    std::chrono::milliseconds write_timeout{100};  // 单帧全量写超时
    void Validate() const;               // 非法参数抛 std::invalid_argument
};

class SerialPort {
    explicit SerialPort(SerialPortConfig config);
    void Open();                         // 失败抛 std::system_error
    void Close();                        // 幂等；析构自动调用
    bool IsOpen() const;
    std::ptrdiff_t Read(uint8_t* buffer, std::size_t size);  // 读取字节数；0=超时；-1=错误
    bool Write(const uint8_t* data, std::size_t size);        // 全量写
    void Flush();                        // 清空收发缓冲（重同步用）
    uint64_t ReadBytes() const; uint64_t WriteBytes() const; uint64_t ErrorCount() const;
    const SerialPortConfig& Config() const;
};
```

线程模型：单线程使用（每链路一线程独占），内部不加锁。PX4、地面站等链路各自创建独立
`SerialPort` 实例和通信线程，不共享 fd。

## 3. 关键实现点

- **termios 配置**：`cfmakeraw` + `CLOCAL | CREAD` + 波特率映射 + 数据位/校验位/停止位，
  显式关闭软硬件流控。
- **非阻塞 fd**：使用 `O_NONBLOCK | O_CLOEXEC` 打开，避免读写永久阻塞或 fd 泄漏到子进程。
- **精确超时**：`VMIN=0`、`VTIME=0`，读写均使用 `poll()` 和单调时钟控制毫秒级超时；
  `EINTR` 在剩余超时时间内继续等待。
- **全量写**：`write` 可能部分写入，循环直到整帧完成；超过 `write_timeout` 返回失败，
  禁止把半条协议帧当作发送成功。
- **链路异常**：`POLLERR/POLLHUP/POLLNVAL` 作为运行时错误返回，由上层链路模块决定重连。
- **移动语义**：可移动不可拷贝（fd 转移）。

## 4. 日志行为

| 等级 | 场景 |
|------|------|
| INFO | 对象创建（设备/波特率/数据位/停止位/校验）、打开成功、关闭、销毁 |
| ERROR（节流） | 读取/写入错误（errno + 描述 + 累计计数）、写入零字节 |

## 5. 测试方式

- **配置校验**：`tests/skeleton/stub_smoke_test.cpp` 的 `SerialPortConfigValidation`
  （非法波特率/校验位/数据位抛异常）。
- **自动化伪终端测试**：`tests/communication/serial_port_test.cpp` 使用 Linux PTY 验证幂等启停、
  双向收发、毫秒级读取超时、关闭后重启及写入超时参数校验，不依赖真实硬件。
- **真实读写**：香橙派上仍需使用 Pixhawk/电台实机验证设备名、波特率、引脚电平和长期稳定性。

## 6. 排查/修改要点

| 现象 | 排查方向 |
|------|----------|
| `Open()` 抛 system_error | 设备不存在/权限不足（`/dev/ttyS0` 属 dialout 组）、被占用 |
| 读一直超时(0) | 对端无数据；检查波特率/数据位/校验位是否匹配 |
| 读到乱码 | 波特率不匹配、`parity`/`stop_bits` 错误 |
| 写失败 | 设备断开；`Flush()` 在协议重同步时调用 |
| 需要更高波特率 | `ToBaudConstant` 映射表扩展 |

## 7. 当前配置占位

`config/config.json` 已预留机载电脑 MAVLink 身份和 PX4 串口参数：

```json
"mavlink": {
    "onboard_system_id": 1,
    "onboard_component_id": 191
},
"px4": {
    "serial": {
        "device": "/dev/ttyS0",
        "baud_rate": 115200,
        "data_bits": 8,
        "stop_bits": 1,
        "parity": "N",
        "read_timeout_ms": 20,
        "write_timeout_ms": 100
    },
    "reconnect_interval_ms": 1000
}
```

`onboard_component_id=191` 对应 MAVLink 标准 `MAV_COMP_ID_ONBOARD_COMPUTER`。当前设备名和
波特率是可修改占位值；待确认香橙派实际 UART 后只改 JSON。配置读取与注入由后续
`Px4Link` 实现接入，`SerialPort` 本身不依赖 JSON。
