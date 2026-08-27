# MAVLink 字节流处理器（mavlink_handler）

> 对应实现：`include/communication/mavlink_handler.h`、`src/communication/mavlink_handler.cpp`
> 使用方：`Px4Link`、后续 `GroundStationLink`

## 1. 功能职责

`MavlinkHandler` 是单条物理链路独占的 MAVLink 协议基础层：

- 增量接收任意长度字节流，处理逐字节、半包、粘包和连续多帧；
- 校验 MAVLink CRC/签名状态，坏帧后继续寻找下一帧；
- 同时接收 MAVLink 1 和 MAVLink 2；
- 使用实例私有发送状态维护 MAVLink 序号，序列化完整帧；
- 统计收发字节、收发消息、解析错误和协议序号丢包。

不做什么：不打开串口/网络、不创建线程、不解释 PX4 或地面站业务字段、不执行 ACK 关联。
业务语义分别由 `Px4Link` 和 `GroundStationLink` 负责。

## 2. 接口与数据流

```cpp
enum class MavlinkVersion : uint8_t { kV1 = 1, kV2 = 2 };

class MavlinkHandler {
public:
    using MessageCallback = std::function<void(const mavlink_message_t&)>;

    explicit MavlinkHandler(MavlinkVersion output_version = MavlinkVersion::kV2);

    std::size_t Feed(const uint8_t* data, std::size_t size,
                     const MessageCallback& callback);

    template <typename Packer>
    std::vector<uint8_t> Encode(Packer&& packer);

    void SetOutputVersion(MavlinkVersion version);
    MavlinkVersion OutputVersion() const;
    void Reset();
};
```

接收：

```text
SerialPort/UDP/TCP 字节
  → MavlinkHandler::Feed
  → mavlink_message_t
  → Px4Link/GroundStationLink 业务解码
  → 项目内部 Topic
```

发送：

```text
项目内部消息/命令
  → 业务 Link 调用 MAVLink *_pack_status
  → MavlinkHandler::Encode
  → 完整 MAVLink 帧字节
  → 物理 Transport::Write
```

## 3. 关键实现点

### 3.1 每条链路独立实例

解析使用 `mavlink_frame_char_buffer()` 和实例私有的 `mavlink_message_t/mavlink_status_t`，不使用
生成库的全局 channel 缓冲。PX4 和地面站链路必须分别创建实例，否则两条链路的半包、序号和
错误状态会互相污染。

### 3.2 私有发送序号

发送端要求调用生成库的 `*_pack_status` 函数，例如：

```cpp
auto frame = handler.Encode(
    [&](mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_heartbeat_pack_status(
            system_id, component_id, status, message,
            type, autopilot, base_mode, custom_mode, system_status);
    });
```

不得在多链路代码中使用依赖全局 channel 状态的普通 `*_pack()` 来维护发送序号。

### 3.3 版本行为

- 接收端无需预先选择版本，可混合解析 MAVLink 1/2；
- 输出版本由 `MavlinkVersion` 控制，默认 MAVLink 2；
- `SetOutputVersion()` 只影响后续发送；
- `Reset()` 在物理链路重连后清空半包、收发序号和统计，同时保留配置的输出版本。

### 3.4 错误与丢包

- `MAVLINK_FRAMING_BAD_CRC` 与 `MAVLINK_FRAMING_BAD_SIGNATURE` 增加解析错误计数；
- 坏帧不向业务回调；
- 后续正确帧仍可恢复解析；
- `DroppedPacketCount()` 是 MAVLink 序号诊断数据，不直接作为飞行控制故障判据；链路健康仍以
  心跳和关键遥测新鲜度为准。

## 4. 线程与日志行为

`MavlinkHandler` 本身不加锁，单个实例只能由对应链路线程独占调用。解析属于高频热路径，模块
不逐帧打日志；业务 Link 根据连接变化和累计错误按项目节流规则记录日志。

## 5. 测试方式

测试文件：`tests/communication/mavlink_handler_test.cpp`。

覆盖：

- MAVLink 2 HEARTBEAT 编码/解码；
- MAVLink 1 输出与输入；
- 一帧拆成多段输入；
- 多帧粘在一次输入中；
- CRC 错误拒绝与下一帧恢复；
- 两个处理器实例的半包状态隔离；
- 重置解析状态和统计；
- 空输入安全忽略。

运行：

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
./build/mavlink_handler_test
ctest --test-dir build --output-on-failure
```

## 6. 排查/修改要点

| 现象 | 排查方向 |
|---|---|
| 一直无法完成解析 | 串口参数错误、未收到完整帧、dialect 不匹配 |
| CRC 错误持续增长 | 波特率/电平/接线错误，或发送端 dialect 与本工程不同 |
| 两条链路偶发互相干扰 | 是否错误共享同一个 `MavlinkHandler` 实例或使用全局 channel pack API |
| 对端拒收输出帧 | 检查输出 MAVLink 版本、system/component ID、消息 dialect |
| 重连后首帧异常 | 物理链路重连时调用 `Reset()` 清空残留半包 |
| 丢包计数增加 | 检查链路带宽与发送频率；不要仅凭序号计数直接触发飞行故障 |

修改生成 MAVLink 库目录或 dialect 后，必须重新运行 v1/v2、CRC 和多实例隔离测试。
