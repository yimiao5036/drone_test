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
    std::chrono::milliseconds read_timeout{100};  // 单次读超时
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

线程模型：单线程使用（每链路一线程独占），内部不加锁。

## 3. 关键实现点

- **termios 配置**：`cfmakeraw` + 波特率映射 + 数据位/校验位/停止位。
- **超时读**：`VMIN=0 + VTIME`（单位 0.1s），阻塞读最多等待 `read_timeout`；骨架期简化实现，
  精度不足时后续可替换 poll/epoll。
- **全量写**：`write` 可能部分写入，循环直到写完。
- **移动语义**：可移动不可拷贝（fd 转移）。

## 4. 日志行为

| 等级 | 场景 |
|------|------|
| INFO | 对象创建（设备/波特率/数据位/停止位/校验）、打开成功、关闭、销毁 |
| ERROR（节流） | 读取/写入错误（errno + 描述 + 累计计数）、写入零字节 |

## 5. 测试方式

- **配置校验**：`tests/skeleton/stub_smoke_test.cpp` 的 `SerialPortConfigValidation`
  （非法波特率/校验位/数据位抛异常）。
- **真实读写**：需串口设备，香橙派实机验证；开发机可借 `socat` 建伪终端（`socat -d -d pty,raw,echo=0 pty,raw,echo=0`）做读写回环验证。

## 6. 排查/修改要点

| 现象 | 排查方向 |
|------|----------|
| `Open()` 抛 system_error | 设备不存在/权限不足（`/dev/ttyS0` 属 dialout 组）、被占用 |
| 读一直超时(0) | 对端无数据；检查波特率/数据位/校验位是否匹配 |
| 读到乱码 | 波特率不匹配、`parity`/`stop_bits` 错误 |
| 写失败 | 设备断开；`Flush()` 在协议重同步时调用 |
| 需要更高波特率 | `ToBaudConstant` 映射表扩展 |
