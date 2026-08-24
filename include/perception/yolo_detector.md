# yolo_detector 实现文档

> 对应实现：`src/perception/yolo_detector.cpp`、`src/perception/yolo_postprocess.cpp`、
> `src/perception/rknn_detection_backend.cpp`（香橙派条件编译）
> 更新：2026-08-12

## 功能职责

- **YoloDetector**（`IYoloDetector` 真实实现）：订阅解码帧主题（`Topic<video::FrameHandle>`，
  NV12），逐帧调用推理后端做目标检测，将检测结果转换为 `common::DetectionResult` 发布到
  检测结果主题。独立消费线程运行，与 VideoDecoder 相同的线程模型。
- **yolo_postprocess**（纯函数子模块）：NPU 输出的多分支 INT8 量化张量 → 量化解码 →
  按类别 NMS → letterbox 逆变换，输出裁剪图坐标系检测框。不依赖任何硬件库，开发机可单测。
- **RknnDetectionBackend**（`IDetectionBackend` 实现，仅 `DRONE_HAVE_RKNN=ON` 编译）：
  RGA 预处理（NV12 居中裁剪 16 对齐正方形 → NV12→RGB letterbox 写入模型输入内存）+
  RKNN NPU 推理（3 核上下文）+ 后处理 + 坐标还原原图。

边界：
- 不做：目标跟踪（track_id 保持 0）、类别语义映射（class_id 透传模型类别，反无人机
  专用模型定稿后由配置映射 0=无人机 1=障碍物）、帧级聚合（一帧多目标发布多条消息，
  融合侧按 frame_sequence 聚合并做超时判丢）。
- 不做：图像叠加标注（第 5 步图传链路）、光流（optical_flow_estimator 独立部件）。

## 接口与数据流

```
IVideoDecoder::FrameOutput()
        │  Topic<video::FrameHandle>（订阅队列容量默认 2，丢最旧）
        ▼
YoloDetector::DetectLoop（独立线程）
        │  IDetectionBackend::Detect(frame)   ← 后端依赖注入
        ▼
YoloDetector::DetectionOutput()  Topic<common::DetectionResult>（每目标一条）
        │
        ▼  IPerceptionFusion（第 4.7 步）
```

- 接口签名：见 `include/perception/yolo_detector.h`（IYoloDetector 冻结于
  `docs/数据接口文档.md` §4.4，本实现不改变签名）。
- `YoloDetectorConfig`：`model_path`（RKNN 模型路径，空 = 必须注入后端）、
  `conf_threshold`（默认 0.25）、`nms_threshold`（默认 0.45）、
  `input_queue_capacity`（解码帧订阅队列容量，默认 2）。
- `IDetectionBackend`：`Load() / Unload() / IsLoaded() / Detect(FrameHandle) →
  vector<BackendDetection>`（原图坐标系像素框）。工厂 `CreateDefaultDetectionBackend`
  在 `DRONE_HAVE_RKNN` 编译时返回 RKNN 后端，否则返回 nullptr。
- 发布约定：每条 `DetectionResult` 一个目标；`frame_sequence` = 帧 `Info().sequence`；
  `header.source_time_ms` = 帧单调时间戳；`header.sequence` 检测器内递增；
  `inference_time_ms` 同帧多目标相同；一帧无检测不发布。

## 关键实现点

- **线程模型与停机**：`Start()` 启动 `DetectLoop`（`WaitTakeFor(100ms)` 轮询订阅），
  `Stop()` 置停止标志 → `input_sub.Reset()` 唤醒等待 → `join()`。`Start` 幂等
  （已启动直接返回 true）；停止后重启会重新订阅（`Stop` Reset 了订阅）并重新
  `Load()` 后端。`load_failed` 标志防止加载失败后每帧重试刷屏。
- **后端抽象与双环境**：推理后端经构造参数注入（`std::unique_ptr<IDetectionBackend>`），
  开发机测试注入 Mock；香橙派默认 `RknnDetectionBackend`。两者都不可用时
  `Start()` 返回 false 并打 ERROR，不静默空转。
- **耗时统计**：检测线程对 `Detect()` 调用计时，指数滑动平均（EMA, α=0.1），
  长时间运行不迟钝。热路径不打日志。
- **后处理移植**（`yolo_postprocess`）：移植自原型 `common.cpp` 的 `process_i8` /
  `nms` / `quick_sort_indice_inverse` / `compute_dfl`，参数化类别数与分支结构。
  关键数值逻辑与原型保持一致：量化阈值比较（避免逐网格反量化）、score_sum 快速过滤、
  box 通道 = 4×dfl_len（YOLO26 无 DFL 直接反量化，YOLO11 走 DFL 解码）、
  letterbox 逆变换（去 pad → clamp 到模型尺寸 → 除 scale）。
- **RKNN 后端**（香橙派）：原型 `rknn_model.cpp` 的 3 核上下文
  （`rknn_init` + `rknn_dup_context`×2 + `RKNN_NPU_CORE_ALL`）、输入输出零拷贝内存
  （`rknn_create_mem` / `rknn_set_io_mem`，输入 UINT8 由 NPU 内部 normalize/quantize）、
  输出 NC1HWC2→NCHW 转换（预分配缓冲）。预处理与原型差异：输入是 NV12
  FrameHandle 而非 BGR cv::Mat（不引入 OpenCV），RGA 一次完成裁剪/缩放/色彩转换
  （NV12→RGB888）。模型类别数、输入尺寸、分支结构（2 或 3 输出/分支）在
  `QueryModelInfo` 时从张量属性解析，不写死。
- **分辨率变化**：裁剪缓冲懒分配按需扩容；letterbox 每帧按实际宽高计算，
  解码分辨率变化（断流重连）无需重建后端。

## 日志行为

| 场景 | 等级 | 节流 |
|------|------|------|
| 创建/销毁/启动/停止 | INFO | 否 |
| 后端缺失、加载失败（模型不可读、RKNN 初始化失败等） | ERROR | 否（启动路径一次性） |
| 推理异常（Detect 抛异常）、RGA 失败、RKNN 推理失败 | ERROR | 第 1 次 + 每满 100 次，带累计计数 |
| 无效帧（元数据缺失）、非 NV12 帧 | WARN | 第 1 次 + 每满 100 次，带累计计数 |
| 逐帧检测/推理耗时 | 不打日志 | — |

## 测试方式

- **开发机（WSL2 Ubuntu 24.04）**：
  ```bash
  cmake -S . -B build && cmake --build build -j$(nproc)
  ./build/yolo_postprocess_test   # 后处理纯函数：解码/NMS/letterbox 逆变换，8 用例
  ./build/yolo_detector_test      # Mock 后端注入：线程/发布/统计/错误处理，9 用例
  ctest                           # 全部 60 用例应通过
  ```
  期望结果：`yolo_postprocess_test` 验证量化解码数值、阈值过滤、score_sum 快速过滤、
  同类别 NMS 抑制、异类保留、letterbox 逆变换坐标；`yolo_detector_test` 验证
  Start/Stop 幂等、无后端失败、检测字段完整、帧序号关联、后端故障恢复、无效帧跳过、
  停机后停止消费、停止后重启。
- **香橙派实机**：`cmake -S . -B build -DDRONE_HAVE_RKNN=ON
  -DDRONE_RKNN_API_DIR=<rknn头目录> -DDRONE_RGA_DIR=/usr/include/rga`，需安装
  librknnrt（RKNN runtime）与 librga；验证 RGA 预处理色彩/裁剪、3 核推理耗时。
  2026-08-24 实机首次启用条件编译时修复 `RknnDetectionBackend::Impl` 缺失
  `model_channel_` 成员导致的编译错误；该成员用于记录 NCHW/NHWC 输入通道数并输出模型信息
  （原型实测约 53ms/帧）、检测框与实拍目标对齐、断流重连分辨率变化。

## 排查/修改要点

- **启动失败"无推理后端"**：开发机未注入后端且未开 `DRONE_HAVE_RKNN` 的正常表现；
  香橙派检查 CMake 是否 `DRONE_HAVE_RKNN=ON`、模型路径是否存在。
- **启动失败"后端加载失败"**：模型文件不可读、RKNN 驱动未加载（`dmesg` 查 rknpu）、
  SRAM 初始化失败（尝试去掉 `RKNN_FLAG_ENABLE_SRAM`）。
- **检测结果坐标错位**：先核对 letterbox 参数（`x_pad/y_pad/scale`）与后处理逆变换
  一致性；再核对裁剪偏移叠加；用单目标单色场景在香橙派打点验证。
- **推理耗时异常**：确认 3 核上下文与 `RKNN_NPU_CORE_ALL`；检查是否误用
  `RKNN_FLAG_COLLECT_PERF_MASK`（原型已去除）。
- **RGA 接口差异**：`wrapbuffer_virtualaddr` 带 stride 重载、`im2d.hpp` 常量
  （`INTER_LINEAR`、`IM_STATUS_SUCCESS`）以香橙派实际安装的 librga 版本为准，
  实机联调时核对；`/usr/include/rga` 缺失时 `apt install librga-dev`。
- **修改关联**：`DetectionResult` 字段与 `docs/数据接口文档.md` §2 冻结，改动需同步
  文档并重跑 `yolo_detector_test`；后处理数值逻辑改动需同步原型
  `videoPart/yolo26-rknn`（避免两套实现漂移）；`IDetectionBackend` 新增后端时
  同步 `CreateDefaultDetectionBackend` 工厂与 CMake 条件编译。
