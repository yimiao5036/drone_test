# RTSP 接收（camera_receiver）

> 对应实现：`include/video/camera_receiver.h`、`src/video/camera_receiver.cpp`
> 接口：`ICameraReceiver`（同头文件）；占位实现 `CameraReceiverStub`（`src/video/camera_receiver_stub.cpp`）
> 下游：解码器 `video_decoder.md`

## 1. 功能职责

RTSP 拉流（TCP/UDP），逐访问单元读取 H.264/H.265 码流并发布到输出主题：

- 建连、断流自动重连（间隔可配置）、确定性停机（中断回调）。
- 连接成功后单独发布一条**流级参数集消息**（SPS/PPS，从 SDP/容器头提取），供解码器初始化。

不做什么：不解码（解码是 `VideoDecoder` 的职责）；不负责厂商私有 TCP 协议（如未来摄像头
不用 RTSP，替换实现类即可，接口不变）。

## 2. 接口与数据流

```cpp
struct CameraReceiverConfig {
    std::string rtsp_url;                    // 如 rtsp://192.168.1.100:8554/live
    std::string rtsp_transport = "tcp";      // tcp / udp
    std::chrono::milliseconds open_timeout{5000};    // 建连探测超时（stimeout）
    std::chrono::milliseconds reconnect_delay{3000}; // 断线重连间隔
};

class CameraReceiver final : public ICameraReceiver {
    // Start/Stop/IsRunning；IsConnected/ConnectCount/ReceivedBytes/ErrorCount
    // Topic<common::EncodedFrame>& StreamOutput();
};
```

数据流：`RTSP 网络 ─► avformat ─► Topic<EncodedFrame> ─► IVideoDecoder`
消息类型：`common::EncodedFrame`（含 `codec`、`parameter_sets`、`is_key_frame`、`stream_sequence`）。

## 3. 关键实现点

- **FFmpeg avformat**：`avformat_open_input` + `avformat_find_stream_info`，选项：
  `rtsp_transport`（tcp/udp）、`fflags=nobuffer`、`probesize/analyzeduration` 限制探测时间、
  `stimeout` 超时（微秒）。
- **中断回调**：`format_ctx->interrupt_callback` 的 `opaque` 指向实现对象，回调检查停止标志；
  `Stop()` 置位后阻塞中的 `av_read_frame` 尽快返回（`AVERROR_EXIT`），保证确定性停机。
- **断线重连**：`av_read_frame` 失败 → 关闭连接 → 等待 `reconnect_delay` → 重新 `OpenStream`。
- **参数集消息**：`OpenStream` 成功后若 `codecpar->extradata` 非空，发布一条 `data` 为空、
  `parameter_sets` 填充的消息；解码器在首帧前完成初始化（见 `video_decoder.md`）。
- **码流分帧**：`av_read_frame` 返回的 packet 即一个可送入解码器的访问单元，直接打包发布。

## 4. 日志行为

| 等级 | 场景 |
|------|------|
| INFO | 创建、启动、停止、销毁、连接成功（url/编码/分辨率/连接次数） |
| WARN（节流） | 打开 RTSP 失败、断流（带累计计数，第 1 次 + 每满 100 次） |
| ERROR | 分配上下文失败、未找到视频流 |

逐帧热路径（读包/发布）不打日志。

## 5. 测试方式

- **自动化测试**：暂无（依赖真实 RTSP 源）。核心链路通过 `video_decoder_test` 软解测试覆盖
  （测试中的参数集消息模拟本类的发布行为）。
- **实机验证（香橙派）**：配置真实 `rtsp_url`，运行后观察日志：
  ```
  INFO 摄像头接收器已连接: url=... 编码=H.265 分辨率=... 
  ```
  断网后应出现 `WARN 摄像头接收器断流 ... 准备重连`，恢复后自动重连。

## 6. 排查/修改要点

| 现象 | 排查方向 |
|------|----------|
| 一直 `打开 RTSP 失败` | 地址/端口、`rtsp_transport` 是否与源匹配、`open_timeout` 是否过短、防火墙 |
| 频繁断流重连 | 网络抖动；`fflags=nobuffer` 会放大丢包，必要时调小 `reconnect_delay` 或加大缓冲 |
| 摄像头不支持 RTSP | 替换实现类（保持 `ICameraReceiver` 接口），参考通信与数据定义 §11 厂商协议 |
| 改重连策略 | `reconnect_delay`；如需退避（指数/上限）在 `ReceiveLoop` 中扩展 |
