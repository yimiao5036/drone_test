# USB 双目摄像头接入（采集/解码/左右拆分）设计

> 日期：2026-09-21
> 关联：`docs/双目深度估计思路.md`（总体路线）、`docs/双目硬件事实确认记录.md`（P3/P4 已关闭的硬件事实）
> 前置：双目模组 23H1328889HV1 已完成板上实证——SBS 同帧单流（单采集节点 /dev/video0）、
> 全部有效档位实测 30fps、buffer 时间戳 monotonic/soe。

## 1. 本轮范围与既定前提

**本轮做到 B 层**：UVC 采集 → MJPG 解码 → RGA 左右拆分 → **左目进现有链路出图**，
右目发独立 Topic 仅统计。不做双推理、不做测距（测距受 P2 标定制约，标定完成前
仅限影子/回放验证，不在本轮）。

既定前提（已与维护者确认）：

- **档位**：2560×720@30 MJPG（SBS，每目 1280×720），与现有链路分辨率一致，
  USB 带宽与解码负载低于 3840×1080 档
- **链路关系**：与现有 RTSP 链路**配置切换**（`camera_source` 字段二选一），
  不并存、不替换；现有 RTSP 链路保持可用作备份
- **验收**：量化指标（见 §5）
- 安全基线不动：`enable_control: false`

## 2. 架构与数据流

拆分落点为**独立组件 StereoFrameSplitter**（方案 1，维护者选定）：VideoDecoder
保持通用只加 MJPG 分支；拆分组件单一职责可独立测试；右目 Topic 出口天然。

```text
【UVC 双目链路（camera_source=uvc, stereo_split=true)】

/dev/video0 (V4L2 mmap, 2560×720@30 MJPG, SBS)
  → UvcCameraReceiver（新）──kCameraStream(EncodedFrame, kMjpeg)──► VideoDecoder(+MJPG 分支)
  ──kDecodedFrame(2560×720 NV12 全幅)──► StereoFrameSplitter（新，RGA 裁剪）
      ├─左目 1280×720──kDecodedFrameLeft──► YoloDetector / FrameCompositor（改订阅此 Topic）
      └─右目 1280×720──kDecodedFrameRight──► 统计订阅（帧率/丢帧核对）

【RTSP 链路（camera_source=rtsp，现状）】完全不动：
  CameraReceiver → kCameraStream → VideoDecoder → kDecodedFrame → YoloDetector / FrameCompositor
```

- 两链路在 `DroneApplication::BuildComponents()` 按配置二选一，接收器共用
  `ICameraReceiver` 接口，下游组件零改动
- 拆分后左目与现有 RTSP 链路的解码帧同为 1280×720 NV12，YOLO 输入侧无差异
- 时间戳：v4l2 buffer 时间戳（monotonic/soe，主机侧接收时刻）写入
  `EncodedFrame.capture_time_ms`；splitter 输出帧时间戳与序号**透传源帧**，
  便于左右目帧率一致性核对

## 3. 组件设计

### 3.1 types.h 扩展

- `VideoCodec` 新增 `kMjpeg`
- Topic 常量新增：
  - `kDecodedFrameLeft`（拆分后左目，`video::FrameHandle`）
  - `kDecodedFrameRight`（右目统计，`video::FrameHandle`）
- `kDecodedFrame` 保留为解码器全幅输出；`EncodedFrame` 结构不动
  （MJPG 一帧=一个访问单元、`is_key_frame=true`、参数集为空，天然适配）

### 3.2 UvcCameraReceiver（新组件，`include/video/uvc_camera_receiver.h`）

实现 `ICameraReceiver` 接口，V4L2 mmap 直采（无第三方依赖）：

- 配置 `UvcCameraReceiverConfig`：device 路径（默认 `/dev/video0`）、
  width=2560、height=720、fps=30、mmap buffer 数（默认 4）、重连间隔
- 采集流程：`S_FMT(MJPG)` → `G_FMT` **回读校验**（必须拿到 2560×720 MJPG，
  否则 Start 失败，防固件错档）→ `REQBUFS`/`MMAP`/`QBUF`/`STREAMON` →
  采集线程 `DQBUF` → 构成 `EncodedFrame` 发布 → `QBUF` 归还
- `capture_time_ms` 取 v4l2 buffer 的 monotonic/soe 时间戳（实证可用）；
  `receive_time_ms` 取主机单调时钟
- 断流/拔线：DQBUF 超时（1s，远长于 33ms 帧间隔，容忍瞬时卡顿）或 EIO →
  计 ErrorCount、关流、按重连间隔重开，语义与 RTSP 版对齐（ConnectCount 计数）
- v4l2 fd 与 mmap 缓冲 RAII 封装；Stop 幂等

### 3.3 VideoDecoder MJPG 分支（扩展现有组件）

- codec 分派新增 `kMjpeg → AV_CODEC_ID_MJPEG`
- 优先试 `mjpeg_rkmpp` 硬解（ffmpeg-rockchip 是否提供，实现期验证），
  不可用时回退 FFmpeg 软解（MJPG 软解负载低，2560×720@30 可控）
- 软解输出经现有 swscale 路径转 NV12 入池，下游无感
- MJPG 帧间无依赖：单帧解码失败计 DroppedFrameCount 继续，
  无 H.26x 的参数集同步问题

### 3.4 StereoFrameSplitter（新组件，`include/video/stereo_frame_splitter.h`）

- 订阅 `kDecodedFrame`（全幅 2560×720 NV12），独立消费线程
  （输入队列容量 1、优先最新帧）
- RGA 裁剪：左目 src rect (0,0,1280,720)、右目 (1280,0,1280,720)；
  左右各一个自有 VideoFramePool（1280×720 NV12，容量默认 8，stride 64 对齐）
- 输入校验：帧宽非偶数或尺寸与配置不符 → 丢弃计错（节流日志）
- x86 开发机无 RGA 时回退逐行 memcpy（NV12 裁剪简单），保证 WSL2 可测；
  RGA 失败亦回退 memcpy 并计 fallback 数
- 输出帧时间戳/序号透传源帧；池满丢帧分左右计数（DroppedFrameCount）
- **左右目对应关系**：SBS 左半幅是否为物理左目未经实证（探测只确认了左右视差）。
  本轮任取左半幅为主用目（不影响出图/统计验收）；测距阶段开始前必须实证
  （如遮挡单侧镜头看画面变化），届时若相反仅交换两个裁剪 rect 即可

### 3.5 配置与装配

config.json `video` 节新增（默认值保持现状零回归）：

```json
"camera_source": "rtsp",        // rtsp | uvc
"uvc_device": "/dev/video0",
"uvc_width": 2560,
"uvc_height": 720,
"uvc_fps": 30,
"stereo_split": false           // uvc 且 true 时装配 StereoFrameSplitter
```

- `BuildComponents()`：`camera_source=uvc` 创建 UvcCameraReceiver，
  否则 CameraReceiver（同接口指针 `ICameraReceiver`）
- `BindTopics()`：`stereo_split=true` 时 YoloDetector/FrameCompositor
  改绑 `kDecodedFrameLeft`；否则维持 `kDecodedFrame`
- `camera_source=uvc` 且 `stereo_split=false` 为过渡态：全幅 2560×720 直进
  YOLO/Compositor（非本轮验收对象，仅调试用）
- 健康监控沿用现有 `register_source` 模式：UVC 接收器复用 `kCamera` 源名，
  splitter 注册新源名
- `enable_control: false` 安全基线不动

## 4. 错误处理与日志

沿用项目硬约束：构造/初始化抛异常，运行时错误码/optional，资源 RAII；
逐帧热路径零日志。

| 场景 | 行为 | 日志 |
|---|---|---|
| UVC Start 失败（设备不存在/格式校验不符） | 返回 false | ERROR |
| DQBUF 超时/EIO（拔线） | 计 ErrorCount，关流重连 | WARN（重连生命周期 INFO/WARN） |
| mjpeg_rkmpp 不可用 | 回退软解 | WARN（开发机无 rkmpp 为正常路径 INFO） |
| MJPG 单帧解码失败 | 计 DroppedFrameCount 继续 | ERROR 节流（1+100） |
| 拆分输入尺寸非法 | 丢弃计错 | ERROR 节流（1+100） |
| 池满丢帧 / RGA 回退 memcpy | 计数不阻塞 | WARN 节流 |

## 5. 测试与验收

### 5.1 WSL2 单测（Google Test，随主工程 ctest）

- **VideoDecoder MJPG 分支**：用探测已抓回的 MJPG 帧（3840×1080，解码器不关心
  分辨率）或 ffmpeg 合成的 2560×720 MJPG 构造 `EncodedFrame` 喂入 → 断言解出
  NV12、尺寸正确、计数正确（x86 软解可测）
- **StereoFrameSplitter**：构造左右半幅不同图案的 2560×720 NV12 假帧 →
  断言左右输出各 1280×720、内容与对应半幅逐字节一致、时间戳/序号透传、
  丢帧计数（x86 memcpy 路径可测）
- **UvcCameraReceiver**：WSL2 无 USB 相机，单测限生命周期（Start/Stop 幂等）、
  错误路径（设备不存在→Start false）、配置校验；采集正确性由板上验收覆盖
  （v4l2 行为已被探测报告实证）
- **配置解析**：`camera_source`/`uvc_*`/`stereo_split` 解析与默认值
- 基线：现有 206 测试全过 + 新增测试全过

### 5.2 板上验收（量化指标）

1. 2560×720@30 连续运行 ≥30 分钟无断流
2. 左目链路端到端延迟 ≤ 现有 RTSP 链路 +50ms（现有延迟统计日志对比）
3. 左右目帧率一致（计数核对）
4. 人工确认回传画面为左目单目 1280×720 正常画面

### 5.3 文档同步（项目强制）

- 新增：`include/video/uvc_camera_receiver.md`、`include/video/stereo_frame_splitter.md`
- 更新：`include/video/video_decoder.md`（MJPG 分支）、`docs/模块索引.md`、
  `docs/项目总览与数据流.md`（UVC 链路）、`docs/开发进度.md`

## 6. 明确不做（YAGNI）

- 双 YOLO 推理（NPU0/NPU1 绑核）——后续轮次
- StereoRanger 视差测距——受 P2 标定制约，标定完成前不实现
- 右目回传/落盘——本轮右目仅统计
- RTSP 链路任何改动
