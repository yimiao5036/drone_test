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
  SPS/PPS 是 IDR 之前独立到达的非关键帧小包；旧逻辑丢弃它们导致 rkmpp 收不到参数。
  修正：解码器未创建时用首个数据帧（任意类型）创建并一并送包，此后全量送包。
- **HEVC extradata 陷阱**：容器头 `extradata` 对 HEVC 是 `HEVCDecoderConfigurationRecord`
  （MP4 格式），不能整块塞给 rkmpp 硬解；硬解交给 rkmpp 从码流内嵌 SPS/PPS 自行解析
  （对齐实测成功的原型）；软解才写 extradata。
- **输出转存（关键，本机段错误根因）**：rkmpp 硬解输出为 **NV12**（实测 ffmpeg:
  `wrapped_avframe nv12`）。旧实现用 `sws_scale` 做 NV12→NV12 纯拷贝时，在部分
  linesize/UV 指针组合下踩内存 → `SIGSEGV`（gdb 回栈定位在 `PublishFrame → sws_scale`、
  解码线程）。修复：**NV12 帧不走 sws**，直接按 `frame->linesize` 源 stride 逐行 `memcpy`
  Y/UV 平面到内存池；仅软解 YUV420P 才用 `sws_scale` 转 NV12。硬解识别条件兼看
  `hw_frames_ctx`。
- **本相机实测**：H.265/HEVC Main、1280x720、25fps、LIVE555 RTSP；`ffprobe` 确认；
  rkmpp 硬解经实验确认可正常出帧。
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
| `non-existing PPS 0` / `h265d: pps invalid`（段错误） | ① 若 SPS/PPS 为 IDR 前独立小包：勿按“未初始化跳过非关键帧”（已修复为全量送包）。② HEVC 容器 extradata 勿塞给 rkmpp（已修复）。③ 若已能出帧仍段错误：为输出转存内存问题；纯 NV12 拷贝不再走 sws（已修复），确认源 linesize 与池 stride 匹配 |
| 解码 0 帧但无错误 | 输入队列容量(8) < 突发帧数挤掉关键帧后无后续关键帧（GOP 过长）；调整队列容量或等关键帧机制确认 |
| `DroppedFrameCount` 增长 | 池容量 < 输出订阅队列 + 在途；调大 `pool_capacity` |
| 硬解失败 | 香橙派需 rkmpp 版 FFmpeg + `/dev/dri` 可用；开发机无 rkmpp 属正常回退软解 |
| 帧率不足 | 软解慢属预期（开发机）；香橙派确认走 `hevc_rkmpp`；`sws` 在格式不变时不会重建 |
| 修改输出格式 | 当前固定 NV12；如需 RGB888 改 `sws` 目标格式与 `PixelFormat` |
