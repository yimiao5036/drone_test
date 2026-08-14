# 视频解码（video_decoder）

> 对应实现：`include/video/video_decoder.h`、`src/video/video_decoder.cpp`
> 接口：`IVideoDecoder`（同头文件）；占位实现 `VideoDecoderStub`（`src/video/video_decoder_stub.cpp`）
> 上游：RTSP 接收 `camera_receiver.md`；下游：内存池 `video_frame_pool.md`

## 1. 功能职责

订阅 H.264/H.265 码流块（`common::EncodedFrame`），解码为 **NV12** 帧，从内存池分配
`FrameHandle` 发布（零拷贝共享、最后引用归还内存池）：

- 解码器优先 rkmpp 硬解（香橙派 DRM 路径），不可用时回退 FFmpeg 软解（开发机验证路径）。
- 解码器未创建前不丢弃任何包：首个数据帧（任意类型，含 IDR 前独立到达的 SPS/PPS 小包）创建解码器并送包，之后全量送包，由解码器自行完成参数集同步与关键帧等待（对齐原型 `videoPart/rtsp_yolo_stream`）。
- 池满丢帧不阻塞（WARN 节流）。

不做什么：不负责拉流（上游 `CameraReceiver`）；不负责色彩转换到 RGB（后续 YOLO 用 RGA 转）。

## 2. 接口与数据流

```cpp
struct VideoDecoderConfig {
    std::size_t pool_capacity = 8;     // 帧内存池容量（≥ 输出订阅队列 + 在途）
    std::uint32_t width = 0;           // 预知分辨率（0=首帧懒建池）
    std::uint32_t height = 0;
    std::uint32_t stride_alignment = 64;  // 水平 stride 像素对齐
    bool prefer_hardware = true;       // 优先 rkmpp 硬解
};

class VideoDecoder final : public IVideoDecoder {
    // Start/Stop/IsRunning；SetInput(Topic<EncodedFrame>&)；
    // FrameOutput() → Topic<video::FrameHandle>&；
    // DecodedFrameCount/DroppedFrameCount/ErrorCount
};
```

数据流：`Topic<EncodedFrame> ─► avcodec 解码 ─► NV12 ─► VideoFramePool.Acquire ─► Topic<FrameHandle>`

## 3. 关键实现点

- **解码器选择**：`avcodec_find_decoder_by_name("hevc_rkmpp"/"h264_rkmpp")` 优先；
  硬解打开失败（如无 DRM 设备）回退 `avcodec_find_decoder` 软解。
- **硬解路径**：`av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_DRM)` + 绑定解码器；
  输出 `AV_PIX_FMT_DRM_PRIME` → `av_hwframe_transfer_data` 转存系统内存（NV12）。
- **软解路径**：YUV420P → swscale 转 NV12；`sws_ctx` 按源格式缓存，格式变化时重建。
- **参数集**：`EncodedFrame.parameter_sets` 写入 `codec_ctx->extradata`（RTSP 流级参数集）；
  无参数集时解码器从关键帧内嵌 SPS/PPS 解析。
- **参数集与送包策略（关键）**：不再按“未初始化跳过非关键帧”。本款摄像头 RTSP 的
  SPS/PPS 是 IDR 之前**独立到达的非关键帧小包**；旧逻辑丢弃它们导致 rkmpp 永远收不到
  参数集，随后解析 IDR 报 `invalid pps` 并段错误。修正：解码器未创建时用首个数据帧
  （任意类型）创建并一并送包，此后全量送包，由 rkmpp 自行完成参数同步与关键帧等待。
  若容器头 `extradata` 非空，`CameraReceiver` 仍会先发参数集消息初始化解码器。
- **内存池**：配置给分辨率则 `Start()` 预建池；否则首帧确定尺寸懒建池。
  `hor_stride = align_up(width, 64)`；`buf_size` 由池按 NV12 自动推算。
- **packet 拷贝**：`av_new_packet` + memcpy（骨架期接受拷贝开销，实测不足再优化零拷贝）。

## 4. 日志行为

| 等级 | 场景 |
|------|------|
| INFO | 创建（配置）、启动、停止、销毁、解码器创建（名称/硬解或软解）、池创建（容量/分辨率/stride） |
| WARN | rkmpp 存在但打开失败回退软解、池满丢帧（节流） |
| ERROR（节流） | 送包失败、取帧失败、sws 创建失败、池创建失败、硬件帧转存失败 |

## 5. 测试方式

`tests/video/video_decoder_test.cpp`（2 个用例）：

```bash
cmake --build build && cd build && ctest -R VideoDecoder
```

- `DecodesH264ToNv12Frames`：libx264 软编 10 帧（ultrafast 预设，SPS/PPS 随关键帧）→ 发布 →
  软解 → 断言输出帧数、320x240、NV12 格式、缓冲可写、`DroppedFrameCount==0`。
- `StopsCleanlyWhenIdle`：无输入启停干净（确定性停机）。
- 硬解路径（rkmpp/DRM）需香橙派实机验证。

## 6. 排查/修改要点

| 现象 | 排查方向 |
|------|----------|
| `non-existing PPS 0 referenced` / `h265d: pps invalid` / 空参数集 | 参数集丢失。本款摄像头 SPS/PPS 为 IDR 前独立小包，旧逻辑按“跳过非关键帧”丢弃；已修正为全量送包。若再遇到先确认 `CameraReceiver` 的 extradata/参数集消息是否下发，再看队列容量是否挤包 |
| 解码 0 帧但无错误 | 输入队列容量(8) < 突发帧数挤掉关键帧后无后续关键帧（GOP 过长）；调整队列容量或等关键帧机制确认 |
| `DroppedFrameCount` 增长 | 池容量 < 输出订阅队列 + 在途；调大 `pool_capacity` |
| 硬解失败 | 香橙派需 rkmpp 版 FFmpeg + `/dev/dri` 可用；开发机无 rkmpp 属正常回退软解 |
| 帧率不足 | 软解慢属预期（开发机）；香橙派确认走 `hevc_rkmpp`；`sws` 在格式不变时不会重建 |
| 修改输出格式 | 当前固定 NV12；如需 RGB888 改 `sws` 目标格式与 `PixelFormat` |
