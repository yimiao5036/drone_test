# 视频帧叠加器（FrameCompositor）实现文档

> 对应实现：`include/video/frame_compositor.h`（实现 `src/video/frame_compositor.cpp`）

## 功能职责

在解码帧（NV12）上叠加 YOLO 检测框 + 目标类型/置信度文字，产出自带叠加的标注帧
`kAnnotatedFrame`，供图传 `VideoSender` 编码推流。

数据流（对应 `docs/数据接口文档.md` §5 中 `kAnnotatedFrame` 生产者"感知线程（叠加后）"）：

```
kDecodedFrame(FrameHandle, NV12) ─┐
                                  ├─► FrameCompositor ─► kAnnotatedFrame(FrameHandle, NV12)
kDetection(DetectionResult) ──────┘
```

**不做**：目标检测、光流、融合（那是 `perception` 各模块）；不做编码/推流
（那是 `VideoSender`）。本组件不建 RTSP、不碰 NPU。

非"13 个部件"之一（是 `kAnnotatedFrame` 的产出单元，依附感知链），故不设 `I` 接口/Stub，
但提供与部件一致的 `Start/Stop/SetInputs/输出主题/状态计数` 便于装配与测试。

## 接口与数据流

- `CompositorConfig`：`pool_capacity`（输出池容量）、`stride_alignment`、`box_line_thickness`、
  `draw_text`（是否画类型+置信度）、`text_scale`、`class_names`（数组下标即 `class_id`）、
  `max_detection_frame_lag`（无新检测时旧框最多沿用的解码帧数，默认 10）。
  根 `main.cpp` 从 `yolo.class_names` 与 `yolo.max_detection_frame_lag` 读取后注入；
  配置缺失时类别默认 `UAV/OBS`、旧框最多沿用 10 帧。
- `SetDecodedInput(Topic<FrameHandle>&)`：绑定解码帧（队列容量 4，丢最旧）。
- `SetDetectionInput(Topic<DetectionResult>&)`：绑定检测结果（队列容量 32，缓存供对齐）。
- `AnnotatedOutput()` → `Topic<FrameHandle>&`。
- 状态：`AnnotatedCount/DroppedFrameCount/ErrorCount`。

## 关键实现点

### 异步帧-检测对齐与历史框清理
消费线程 `Run` 每轮先通过 `DrainDetections` 拉取检测消息，再取解码帧合成：
- 只保留最新 `frame_sequence` 对应的一组检测；同一帧的多目标追加到同一组。
- 收到更大的 `frame_sequence` 时先清空上一帧检测，避免目标移动后历史位置框不断累积。
- 晚到的旧帧检测直接丢弃，避免框倒退。
- 如果解码帧序号比最近检测帧领先超过 `max_detection_frame_lag`，清空最后一组检测，
  避免目标消失或连续无检测时最后一个框永久停留。

每张标注帧都从未绘制的解码帧重新复制，框的“删除”通过不再重画历史检测实现，
不需要在 NV12 图像上执行擦除操作。

### 数据竞态防护：输出用独立内存池
不为就地修改 `kDecodedFrame` 的共享 NV12 缓冲（否则被 YOLO/RGA 并发读会读到半画半不画
的脏数据）。而是把输入 NV12 按行拷入**自己的池帧**（`VideoFramePool`），再在副本上叠加，
从机制上杜绝竞态。代价是按帧一次 `memcpy`（3MB@1080p），相对编码耗时可忽略。

### NV12 叠加（零外部依赖，不引 OpenCV/FreeType）
- **检测框与文字颜色**：`DrawBox`/`DrawText` 统一调用 `PlotPixel` 绘制红色像素，
  NV12 近似值为 Y=82、U=90、V=240。
- **文字**：内置 5×7 点阵 ASCII 字模（A-Z、数字 0-9、空格、`. - %`），
  `DrawText` 按 scale 放大逐点画；标签格式 `配置名称 置信度%`（如 `BALLOON 85%`）。
  `ClassToken` 按 `class_id` 查询 `CompositorConfig::class_names`，越界或名称为空回退 `OBJ`；
  小写自动转大写，不支持字符替换为空格，名称最多显示 16 个字符。
  中文需要大字体集，当前未内置；实机如需中文可扩展字模或接入 FreeType。

### 输出池懒建
首帧确定分辨率后按 `pool_capacity` + `stride_alignment` 创建。池满 `Acquire` 返回空句柄
→ 不阻塞、计 `DroppedFrameCount`（WARN 节流）。

### 生命周期与重启
像 `YoloDetector`：`Stop` `Reset` 订阅唤醒线程；`Start` 通过保存的 `decoded_topic_`/
`detection_topic_` 在 `IsOpen()==false` 时重新订阅，支持停机后再启动。

## 日志行为

- **INFO**：创建（池容量/线宽/文字开关）、输出池创建、启动、停止、销毁。
- **WARN（节流）**：输出池满丢帧。
- **ERROR（节流）**：输入帧非法（格式非 NV12/句柄空）、创建输出池失败。

## 测试方式

`tests/video/frame_compositor_test.cpp`（不依赖硬件）：
- 构建合成 NV12 帧 + 检测结果，验证 Start/Stop 幂等、输出标注帧有效、检测应用到帧、
  新检测帧替换历史框、连续无检测时按帧差清除旧框、JSON 类别名称渲染、无效帧计错
  不发布、停机不再消费、重启后恢复。
- 单元测试验证红色 Y 分量、JSON 名称字模、历史框替换与超时清理；香橙派实机已确认
  红色框/文字正常，框随目标动态移动，历史位置不残留，目标消失后旧框可清除。

```bash
cmake --build build -j$(nproc) && cd build && ./frame_compositor_test
```

## 排查 / 修改要点

- **标注帧未发布**：确认解码帧主题有数据、检测主题已绑（`pending_` 空则只画框不画文字，
  且逐帧仍有输出）；确认输出池未被 0 容量覆盖（构造抛异常）。
- **画框但无文字**：检查 `config/config.json` 的 `yolo.class_names` 是否为非空字符串数组、
  `class_id` 是否越界、`draw_text` 是否为 true，或 `text_scale` 是否过大导致文字越界。
- **改标签文本/颜色**：标签只需修改 JSON 的 `yolo.class_names`；颜色修改 `PlotPixel` 的
  Y/U/V 常量；新增非 ASCII 字符需扩展 `GlyphByte` 或接入字体库。
- **想用真字体/中文**：替换 `DrawText` 为 FreeType/二值字库（受"不引入 OpenCV/控制依赖"
  约束，需权衡后问维护者）。
- **历史框重叠**：确认检测消息的 `frame_sequence` 正确递增；叠加器按该字段替换上一组
  检测。如果目标消失后框保留过久，可减小 `max_detection_frame_lag`，但过小会在 YOLO
  推理偶尔变慢时造成框闪烁。
- **线程模型**：单消费线程串行；若要并行化叠加（多目标/多流）需重新设计，勿直接加锁
  在热路径。
