# UVC 双目相机接收（uvc_camera_receiver）

> 对应实现：`include/video/uvc_camera_receiver.h`、`src/video/uvc_camera_receiver.cpp`
> 接口：`ICameraReceiver`（`camera_receiver.h`，与 RTSP 版 `CameraReceiver` 同接口）
> 下游：解码 `video_decoder.md`；设计：`docs/superpowers/specs/2026-09-21-usb双目接入-design.md`

## 1. 功能职责

V4L2 mmap 直采 USB UVC 双目相机（SBS 同帧单流，如 2560×720@30 MJPG），逐帧构成
`common::EncodedFrame`（`codec=kMjpeg`、`is_key_frame=true`、`parameter_sets` 空）
发布到输出主题；断流/拔线自动关流并按 `reconnect_delay` 重连。

- 与 RTSP 版 `CameraReceiver` 同实现 `ICameraReceiver`，由 `DroneApplication` 按
  `video.camera_source` 配置二选一装配，下游解码/健康监控对具体类型无感。
- 健康上报复用 `health::source_names::kCamera` 源名（计数器接口一致，装配层零改动）。

不做什么：不解码（下游 `VideoDecoder`）、不做左右拆分（下游 `StereoFrameSplitter`）、
不支持双流机型（SBS 同帧单流仅 1 个采集节点，硬件事实已确认）。

## 2. 接口与数据流

```cpp
struct UvcCameraReceiverConfig {
    std::string device = "/dev/video0";      // V4L2 采集节点
    std::uint32_t width = 2560;              // 全幅 SBS 宽（偶数）
    std::uint32_t height = 720;
    std::uint32_t fps = 30;
    std::uint32_t buffer_count = 4;          // V4L2 mmap 缓冲数（>=2）
    std::chrono::milliseconds reconnect_delay{3000};
};

class UvcCameraReceiver final : public ICameraReceiver {
    // Start/Stop/IsRunning；IsConnected/ConnectCount/ReceivedBytes/ErrorCount；
    // StreamOutput() → Topic<EncodedFrame>&（语义同 kCameraStream）
};
```

数据流：`/dev/videoN DQBUF ─► EncodedFrame(MJPG) ─► Topic<EncodedFrame> ─► VideoDecoder`

## 3. 关键实现点

- **打开序列**：`open(O_RDWR|O_NONBLOCK|O_CLOEXEC)` → `VIDIOC_QUERYCAP`（须含单平面
  采集 + Streaming，兼容 `V4L2_CAP_DEVICE_CAPS`）→ `S_FMT(MJPG, w, h)` → **`G_FMT`
  回读校验**（必须拿到配置的分辨率与 MJPG，否则 Start 失败，防固件错档）→
  `S_PARM(1/fps)`（尽力而为，失败仅 WARN）→ `REQBUFS/MMAP/QBUF` → `STREAMON`。
- **采集线程**：`poll(fd, POLLIN, 100ms)` 定 tick（Stop 响应 ≤100ms）；帧就绪
  `DQBUF` → 发布 → `QBUF` 归还。fd 与 mmap 缓冲 RAII，`CloseDevice()` 幂等。
- **断流判定**：无帧累计 1s（10 个 100ms tick，远长于 33ms 帧间隔，容忍瞬时卡顿）
  或 ioctl 报 EIO/其他错误 → `ErrorCount++`、节流 WARN、`CloseDevice()`，按
  `reconnect_delay`（100ms 粒度可被 Stop 打断）重连；重连失败仅计数重试。
- **Start 语义**：首次打开同步执行，设备不存在/格式校验不符返回 false（ERROR）；
  启动后运行期失败走重连循环（WARN），不返回失败。
- **时间戳（不得伪造）**：`capture_time_ms` 与 `header.source_time_ms` 取 v4l2 buffer
  的 `V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC` 时间戳（monotonic/soe，与主机单调时钟同域，
  探测报告实证可用）；无 MONOTONIC 标志留 0。`header.receive_time_ms` 取 DQBUF 后
  主机单调时钟。`stream_sequence` 与 `header.sequence` 同源递增。
- **仅系统头**：只依赖 `linux/videodev2.h` 与 POSIX，无第三方库；开发机无 USB 相机
  时 Start 返回 false 属预期。

## 4. 日志行为

| 等级 | 场景 |
|------|------|
| INFO | 创建（设备/分辨率/缓冲数）、启动、停止、销毁、建连成功（驱动/设备名/格式/缓冲数/累计次数） |
| WARN（节流） | 断流与重连失败（第 1 次 + 每满 100 次）、S_PARM 失败、STREAMOFF 失败 |
| ERROR | Start 首次打开失败（open/QUERYCAP/S_FMT/G_FMT 校验/REQBUFS/mmap/QBUF/STREAMON） |

逐帧热路径零日志。

## 5. 测试方式

`tests/video/uvc_camera_receiver_test.cpp`（WSL2 无 USB 相机，限非采集路径）：

```bash
cmake --build build && cd build && ctest -R UvcCameraReceiver
```

- `RejectsInvalidConfig`：空设备/0 宽/0 高/0 帧率/缓冲数<2/负重连间隔 → 构造抛出。
- `StartFailsWhenDeviceMissing`：不存在节点 → Start false、ErrorCount>0、
  ConnectCount==0；重复 Start 仍 false，Stop 幂等。
- `StopIsIdempotentWithoutStart`：未 Start 调 Stop 幂等；StreamOutput 装配期即可订阅。

采集正确性（格式回读、逐帧 DQBUF、时间戳语义、30 分钟无断流、左右帧率一致）由
香橙派板上验收覆盖（v4l2 行为已被 `tools/stereo_camera_probe.sh` 探测报告实证）。

## 6. 排查/修改要点

| 现象 | 排查方向 |
|------|----------|
| Start false + `open: No such file` | 相机未识别：查 `v4l2-ctl --list-devices` 与 USB 连接；节点号变化改 `video.uvc_device` |
| Start false + `格式回读校验不符` | 固件错档：实际格式打在日志里；与 `v4l2-ctl -d N --list-formats-ext` 对拍，确认配置 `uvc_width/height` 在 MJPG 档位内 |
| 运行期反复断流重连 | 拔线/供电不足/带宽抢占：查 `dmesg` USB 错误；确认无其他进程占用节点；正式程序与探测脚本勿同时开 |
| `capture_time_ms=0` | 驱动未给 MONOTONIC 标志：属预期降级（不伪造），下游以 `receive_time_ms` 为准 |
| 修改分辨率/帧率 | 同步 `video.uvc_width/height/fps` 与（`stereo_split=true` 时）拆分器尺寸——配置解析已把 uvc 尺寸透传给拆分器，无需单独配 |
