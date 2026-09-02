# 图传发送器（VideoSender）与编码后端实现文档

> 对应实现：`include/video_transmission/video_sender.h`、`include/video_transmission/video_encoder.h`
>（实现 `src/video_transmission/video_sender.cpp`、`src/video_transmission/video_encoder.cpp`）

## 功能职责

- **VideoSender**：订阅 `FrameCompositor` 产出的标注帧（`kAnnotatedFrame`，NV12
  `FrameHandle`），编码并推送给图传（当前 RTSP）。**拥塞/失败只丢图传帧，不反压**
  上游摄像头/感知链路。
- **IVideoEncoderBackend**（可替换边界）：封装"编码 + 图传协议输出"。默认实现走
  FFmpeg：香橙派优先 rkmpp 硬编码（`h264_rkmpp`/`hevc_rkmpp`），开发机回退
  `libx264`/`libx265` 软编码，均支持 RTSP 推流；也支持 `mpegts` 文件输出（本地调试）。

**不做**：视频叠加/裁剪等图像处理（那是 `FrameCompositor` 的职责，见
`include/video/frame_compositor.md`）；不做目标识别。

## 接口与数据流

```
kAnnotatedFrame (Topic<video::FrameHandle>, NV12)
   ─► VideoSender ──► IVideoEncoderBackend ──► 图传 RJ45（当前 RTSP）
```

- `IVideoSender`（冻结接口，签名不变，见 `docs/数据接口文档.md` §4.3）：
  `Start/Stop/IsRunning/SetInput(Topic<FrameHandle>&)/SentFrameCount/DroppedFrameCount/ErrorCount`。
- `VideoSender`：具体实现。构造传入 `VideoSenderConfig`。
- `VideoSenderConfig`：
  - `encode`：`EncoderBackendConfig`（URL/编码格式/尺寸/帧率/码率/I 帧间隔/传输协议/
    `output_format`=rtsp|mpegts/`output_url`）。`output_url` 非空时优先于 `url`（本地文件调试用）。
  - `input_queue`：标注帧订阅队列容量（默认 2，`kDropOldest`）。
  - `backend_factory`：编码后端注入工厂（测试注入 Mock；为空用默认 FFmpeg 后端）。
- `IVideoEncoderBackend`：`Start/Stop/IsRunning/EncodeFrame(const FrameHandle&)/
  SentFrameCount/ErrorCount`。工厂 `CreateVideoEncoderBackend(config)` 创建默认实现。

## 关键实现点

### 拥塞不反压
- 订阅队列 `Subscribe(input_queue)` 默认容量 2 + `kDropOldest`：图传处理慢时只丢最新
  标注帧，`SendLoop` 对失败帧直接 `continue`，绝不阻塞或等待上游。
- 消费线程与编码后端同一线程串行推进（单消费者订阅句柄，无需锁），与解码器/合成线程解耦。

### 编码链路（FFmpeg，PIMPL 隔离）
1. `Start`：`PickEncoderName` 按 `codec=h264/h265` 与 `prefer_hardware` 选择
   `h264_rkmpp/hevc_rkmpp`（香橙派）或 `libx264/libx265`（软解回退）；打开编码器，
   `AV_CODEC_FLAG_GLOBAL_HEADER` 使 SPS/PPS 进 `extradata`。
2. 输出格式：
   - **RTSP**：`avformat_alloc_output_context2(nullptr, "rtsp", url)`，
     `avformat_write_header` 内部建立网络会话；`rtsp_transport=tcp/udp`。
   - **mpegts（文件调试）**：需显式 `avio_open(&pb, url, AVIO_FLAG_WRITE)`，
     RTSP 不需要（write_header 自建会话）。文件输出由封装器自理 SPS/PPS，**不手动拼**
     extradata。
3. 帧输入：NV12 帧按行拷入 `nv12_in_`（Y 平面 `hor_stride`，UV 平面偏移
   `hor_stride*height`，每行 `w` 字节）。
   - **硬编（rkmpp）**：rkmpp 编码器直接收 NV12 软件帧（内部自行导入 MPP 缓冲），直接送。
   - **软编（libx264/265）**：需 YUV420P，`sws_scale` NV12→YUV420P 到 `pic_out_` 再送编码器。
4. `EncodeFrame`：`avcodec_send_frame` → `DrainPackets`（`avcodec_receive_packet` →
   `WritePacket`，`av_interleaved_write_frame`）。
5. `WritePacket`：时间戳换算；**RTSP** 关键帧前手动拼 `extradata`（提高中途接入兼容性）。
6. `Stop`：置 `running=false` → `avcodec_send_frame(null)` 冲刷尾帧 → `DrainPackets` →
   `av_write_trailer` → 释放 FFmpeg 资源。**幂等**（`Stop` 由 `running` 判断）。

### 关键坑点（已踩过）
- `DrainPackets` 不能依赖 `running` 标志（冲刷时已为 false）；必须 `for(;;)` 到
  EAGAIN/EOF。
- 软编必须显式 NV12→YUV420P，libx264 直接收 NV12 会报
  "Input picture width is greater than stride" 且不产包。
- 编码 PTS 必须**严格递增**：用独立 `frame_counter_`（非 `sent_count`，后者仅在写包成功后
  递增会导致 PTS 不增）。
- 文件输出必须 `avio_open`，否则 `pb` 为空、写包在 `avio_write` 段错误。
- 重启复用既有后端：`Start` 仅当 `backend==nullptr` 才建后端，避免重复建 RTSP 会话抖动。

### 中断回调
原型用 `NetInterruptCallback`（`interrupt_callback`）让阻塞的网络写尽快返回。当前实现
依赖 `Stop` 在独立线程被调用；开发机文件路径无该问题。若实机出现"推流写包卡死"，需补
中断回调——**留待香橙派实机验证时处理**（标记 TODO）。

## 日志行为

- **INFO**：创建（编码格式/地址/分辨率/帧率）、启动、停止、销毁、后端就绪。
- **ERROR（节流：第 1 次 + 每满 100 次）**：创建/打开后端失败、后端启动失败、送帧失败、
  取包失败、写包失败、输入帧非法/尺寸不匹配、PTS/SPS/PPS 相关异常。
- **不判 WARN**：图传丢帧量大时仅 `DroppedFrameCount` 统计累加，刻意不打 WARN——图传拥塞
  属预期降级，且高频，避免刷屏。

## 测试方式

- **单元测试** `tests/video_transmission/video_sender_test.cpp`：注入 `MockBackend`
  （`backend_factory`）验证线程/生命周期/发送计数/后端失败丢帧不反压/停机/重启/后端启动失败。
- **集成测试** `tests/video_transmission/video_encoder_integration_test.cpp`：走真实
  FFmpeg 后端，开发机软编码（libx264）合成 NV12 → 编码到**本地 mpegts 文件**，验证
  编码器可开、帧可送、文件非空、计数正确。自动清理文件。
  ```bash
  cmake --build build -j$(nproc) && cd build && ctest -R "Encoder|Sender|Compositor"
  ```
- **RTSP 推流 + rkmpp 硬编码**：需真实图传目标/香橙派（`ffmpeg-rockchip`），在
  香橙派实机用 `ffprobe -rtsp_transport tcp ...` 或地面站播放器验收。
  **当前采用 mediamtx 中转架构**：`config.video.output_rtsp` 指向本机 mediamtx。
  为避免多机图传路径冲突，正式配置使用身份占位符
  `rtsp://127.0.0.1:8554/drone_{aircraft_component_id}_{aircraft_system_id}`，配置解析后
  捕网-01会变为`rtsp://127.0.0.1:8554/drone_25_1`，火箭-01会变为
  `rtsp://127.0.0.1:8554/drone_26_1`。香橙派程序推给本机 mediamtx，mediamtx 再对外
  发布供任意图传设备拉取（见 `deploy/mediamtx.yml`、`deploy/install_mediamtx.sh`）。
  换图传设备时不动香橙派代码/配置。

## 排查 / 修改要点

- **推流地址不可达**：`Start` 返回 false，`avformat_write_header` 报
  "图传 RTSP 写头失败"。
  - 若指向本机 mediamtx：确认 mediamtx 服务已启动（`systemctl status mediamtx`）、
    8554 端口在监听、配置含对应身份路径或`all_others: source: publisher`。当前捕网-01
    默认路径为`drone_25_1`。
  - 若目标是远端图传设备：确认地址端口与传输协议（`transport=tcp` 不支持时回 udp）。
- **软编不产包 / stride 报错**：确认未在软编路径误送 NV12；`pic_out_` YUV420P 转换是否生效。
- **文件测试段错误（avio_write）**：多半是文件输出未 `avio_open`，`pb` 空。
- **PTS 警告**：用独立帧计数器，勿用发送计数。
- **改图传设备/协议**：替换 `IVideoEncoderBackend` 实现或工厂（可替换边界），不要改
  `VideoSender` 主流程与冻结接口。
- 改 `msvc/mpegts` 输出记得同步文档与测试。
