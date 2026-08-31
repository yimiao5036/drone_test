# 通信传输抽象（communication_transport）

> 对应实现：`include/communication/communication_transport.h`、
> `src/communication/communication_transport.cpp`

## 1. 功能职责

为 `Px4Link` 和后续地面站链路提供统一字节传输边界：

- `SerialTransport`：复用 `SerialPort`，用于香橙派 `/dev/ttyS1`；
- `UdpTransport`：用于 PX4 SITL 和未来网络数传；
- Link 只依赖 `ICommunicationTransport`，MAVLink、Topic 和状态逻辑不因物理链路改变。

不负责 MAVLink 解析、线程、自动重连和业务消息转换。

## 2. 接口

```cpp
class ICommunicationTransport {
public:
    virtual void Open() = 0;
    virtual void Close() = 0;
    virtual bool IsOpen() const = 0;
    virtual std::ptrdiff_t Read(uint8_t*, std::size_t) = 0;
    virtual bool Write(const uint8_t*, std::size_t) = 0;
    virtual void Flush() = 0;
    virtual uint64_t ErrorCount() const = 0;
    virtual std::string Description() const = 0;
};
```

## 3. UDP 行为

- 非阻塞 UDP socket + `poll()` 毫秒级读写超时；
- 绑定 `bind_address:bind_port`；
- 可显式配置远端；也可留空，收到首个数据报后学习对端地址并原路回复；
- 一个 `Read()` 对应一个 UDP 数据报；MAVLink 半包/多帧仍由 `MavlinkHandler` 处理；
- `SO_REUSEADDR` 仅便于测试重启，不允许两个活跃进程同时占用同一 SITL 端口。

PX4 SITL 当前配置：本机绑定 `127.0.0.1:14540`，发送到 PX4 SITL
`127.0.0.1:14580`。若本机 SITL 实例端口不同，应修改 `config/px4_sitl.json`。

## 4. 线程与日志

每个 transport 由一条 Link 线程独占，内部不加业务锁。INFO 只记录打开、关闭和首次学习 UDP
对端；错误按第 1 次和每 100 次节流。

## 5. 测试

`tests/communication/communication_transport_test.cpp` 覆盖：

- 两个 UDP transport 双向数据报；
- 无预设远端时从首个数据报学习对端；
- 远端地址/端口配置一致性校验。

```bash
./build/communication_transport_test
ctest --test-dir build --output-on-failure
```

## 6. 排查要点

| 现象 | 排查 |
|---|---|
| UDP bind 失败 | 端口被 QGC/MAVSDK/旧测试进程占用，使用 `ss -lunp` |
| 无心跳 | PX4 SITL 是否向 14540 发包，检查 SITL `mavlink status` |
| 能收不能发 | remote_port 是否为 PX4 的本地监听端口（通常 14580） |
| 真实串口受影响 | 生产 JSON 必须保持 `px4.transport="serial"` |
