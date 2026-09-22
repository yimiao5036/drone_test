# FFmpeg 8.1（ffmpeg-rockchip）迁移 + RK3588 硬解硬编落地 实现计划

> 日期：2026-09-22  
> 触发：板上 120s 探针实测端到端 avg=79.7ms / p95=160ms / max=300ms，瓶颈全在 CPU 软路径  
> （MJPG 软解 12.8ms + D5 swscale/拷贝 17.6ms + libx264 软编 avg 26ms/max 252ms）。  
> 环境前提：板上已从源码安装 ffmpeg-rockchip 8.1.2 到 `/usr`（lavc 62），CLI 实测
> mjpeg_rkmpp 硬解 196fps、硬解→h264_rkmpp 全链路 299fps。

## 目标与结构决策

- 同源兼容：**WSL2 开发机 FFmpeg 6.1（编译 0 警告 + ctest 全绿）** 与 **板上 ffmpeg-rockchip 8.1**
  （硬解硬编验收）；FFmpeg 4.4 不再支持。版本差异全部用运行时探测吸收，不引入版本宏
  （唯一预案：测试 YUVJ 修复若 6.1 不接受新写法，才加 `LIBAVUTIL_VERSION_MAJOR>=60` 分支）。
- **NV16→NV12 在解码器内转换**（已定）：mjpeg_rkmpp 输出 DRM_PRIME/NV16 DMA-BUF 后，解码器内部
  RGA `imcvtcolor` 一次转换写入既有解码帧池（2560×720 NV12）；拆分器/FrameHandle/Topic/下游
  （YOLO/叠加/编码）零改动。不做 DMA-BUF 直通拆分器（帧句柄体系不承载 DMA-BUF）。

## FFmpeg 8.1 硬断点（全库仅 4 个 .cpp 碰 FFmpeg，PIMPL 纪律生效）

| 触点 | 位置 | 改法（6.1/8.1 通用） |
|---|---|---|
| `av_new_packet`（8.0 已删） | src/video/video_decoder.cpp | `av_packet_unref` + `av_grow_packet` |
| RTSP `stimeout`（8.0 已删） | src/video/camera_receiver.cpp | 改 `timeout`（5.1+，µs 语义相同） |
| `AV_PIX_FMT_YUVJ420P`（8.0 已删） | tests/video/video_decoder_test.cpp | `AV_PIX_FMT_YUV420P` + `color_range=AVCOL_RANGE_JPEG` |
| 无 `avformat_network_init()` | 全 src/ | main.cpp + video_latency_probe.cpp 启动处各调一次 |
| `sws_getContext` 弃用风险 | decoder / encoder | 创建迁 `sws_alloc_context`+av_opt+`sws_init_context`；`sws_scale` 保留，板上若出 deprecated 警告再迁 `sws_scale_frame` |

## 实施步骤（每步独立提交）

### Step 1：FFmpeg 8.1 同源兼容地基（无行为变化）
上表 5 项。验证：WSL2 全量构建 0 警告 + ctest 全绿。
文档：video_decoder.md、camera_receiver.md、video_sender.md、tools/video_latency_probe.md。

### Step 2：drm_nv12_transfer NV16 泛化
- `IDrmNv12Transfer::CopyToNv12` 保持单方法（目标恒 NV12），内部按 `layout.drm_format` 分派；
  结构体 `DrmNv12Layout` 不改名。
- 头文件导出（无 RGA 可测）：`enum class DrmSpFormat{kUnsupported,kNv12,kNv16}` +
  `ClassifyDrmSpFormat(fourcc)` + `ValidateLinearDrmNv16Layout()`。
- NV16 校验差异：fourcc='NV16'；UV 行数=height（垂直满采样）；object_size ≥ pitch×height×2；
  其余规则同 NV12（modifier=0、单对象、y_offset=0、uv_offset==pitch×height、pitch≥width、偶数尺寸）。
- RGA 分派：NV12→`imcopy`；NV16→`imcvtcolor`（RK_FORMAT_YCbCr_422_SP→420_SP 写入 NV12 池槽位）。
- 禁用语义保持：布局不支持→会话内永久禁用（MPP 布局恒定）；运行时 RGA 失败→只计 fallback。
- 测试 +5 用例（fourcc 分类、NV16 合法布局、NV12 尺寸规则误判 NV16 被拒、坏 uv_offset、非线性
  modifier）。文档：drm_nv12_transfer.md。

### Step 3：解码器 rkmpp 设备探测 + NV16 接入
- `CreateRkHardwareDevice()`：`av_hwdevice_find_type_by_name("rkmpp")` 运行时探测（编译期不引用
  `AV_HWDEVICE_TYPE_RKMMPP`，6.1 头文件无此枚举）→ 存在则 alloc+init（8.1 主路径）；否则回退
  `AV_HWDEVICE_TYPE_DRM`（旧 fork 兼容）。hevc/h264/mjpeg 统一走此函数；open2 失败不做设备重试矩阵。
- TryPublishDrmFrameWithRga 最小改动：结构预检不变，分派在 transfer 内；首次成功 INFO 补格式
  （NV12复制/NV16转NV12）；LogDrmLayoutOnce 的图层 format 补 fourcc 字符解码。
- 回退链零新代码：RGA 拒绝→av_hwframe_transfer_data 出 NV16 CPU 帧→既有 sws 分支→池。
  记账复用现有计数器（D4R/D4/D5），不加接口计数器。
- 验证：WSL2 回归 + 板上出图（颜色/错位人工确认，RGA_DMA成功增长、回退=0）。
  文档：video_decoder.md。

### Step 4：编码器 h264_rkmpp 回退结构 + 私有选项
- `OpenEncoderWithFallback()` 候选列表（prefer_hardware 且 h264_rkmpp 存在→先硬编，libx264 兜底）；
  `TryOpenEncoder` 幂等自清场；open 失败回退对齐解码器模式（区分"无硬编属正常"/"硬编打开失败"）。
- AVDictionary 私有选项仅 rkmpp 分支：`rc_mode=VBR`、`rc_max_rate=bitrate`（硬编码常量，不进
  config.json）；open 后 dict 残留检查打 WARN（启动期护栏）。libx264 路径行为不变。
- RTSP 显式 TCP 已满足（VideoEncoderConfig.transport 默认 "tcp"），仅板上复核。
- 测试：新增 prefer_hardware=true 无 rkmpp 时回退成功用例。文档：video_sender.md（含 hwupload
  预案触发条件——仅当 8.1 拒收 CPU NV12 时启用，不预先实现）。

### Step 5：板上收口 + 部署文档
- 板上全量构建 0 警告（扫 deprecated）；120s 探针验收；600s 长测。
- docs/部署与打包.md 新增 ffmpeg-rockchip 8.1 安装节：来源/前缀、卸载发行版 libavcodec-dev 冲突包、
  **`apt-mark hold ffmpeg libav*`** 防 apt 原生同版本号覆盖（会丢全部 rkmpp）、验收命令
  （`pkg-config --modversion libavcodec`=62.x、`ffmpeg -encoders|grep rkmpp`、
  `ldd build/drone_control|grep avcodec` 指向新库）。
- 顶层文档：开发进度.md、项目总览与数据流.md §6.2、故障排查手册.md、模块索引.md（如有计数器变化）。

依赖：Step1 → {Step2→Step3, Step4} → Step5。Step2 与 Step4 可并行。

## 验收对照（板上 120s 探针）

| 指标 | 改造前 | 目标 |
|---|---|---|
| 02 解码+NV12转存 | avg 29.6ms | <5ms |
| D5 NV12内存池复制 | avg 17.6ms | count=0（硬路径） |
| RGA_DMA成功 | 0 | 持续计数 |
| 12 H264编码+推送 | avg 26ms/max 252ms | <5ms 尾部平坦 |
| 10 编码输入队列 p95 | 59ms | ≈0 |
| 14 入口→RTSP发布 | avg 79.7 / p95 160 | avg 35~45ms / p95 <60ms |

## 主要风险与预案

| 风险 | 缓解 |
|---|---|
| imcvtcolor NV16→NV12 板上 librga 实际支持度 | 回退链（FFmpeg转存+sws）自动生效不断链 |
| h264_rkmpp@8.1 拒收 CPU NV12 | hwupload 预案骨架已备（不预实现） |
| rkmpp 设备探测/open 组合不确定 | 两级探测+DRM 回退；WARN 带设备名可定位 |
| libswscale 9 弃用警告（0 警告硬要求） | 创建路径已迁现代 API；sws_scale 触发即迁 sws_scale_frame |
| 6.1 mjpeg 编码器不认 YUV420P+JPEG range | 首日 WSL2 ctest 验证；预案版本宏分支 |
| 系统/rockchip ffmpeg 库混装符号串扰 | apt-mark hold + 卸载清单 + ldd/pkg-config 验收 |
| ffmpeg-rockchip 头文件警告计入 0 警告 | CMake SYSTEM 包覆 PkgConfig::FFMPEG include（MAVLink 有先例） |

## 零回归约束

- `camera_source=rtsp` 默认链路行为不变（hevc_rkmpp 在新 ffmpeg 下走同一 rkmpp 设备探测，
  属顺带迁移，板上需复测 RTSP 链路一轮）。
- `enable_control=false` 安全基线不动；本任务不触碰控制链路。
