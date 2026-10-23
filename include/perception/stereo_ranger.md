# stereo_ranger 实现文档（v2：稀疏 Census 路线）

> 对应实现：`include/perception/stereo_ranger_core.h`、`src/perception/stereo_ranger_core.cpp`、
> `include/perception/stereo_ranger.h`、`src/perception/stereo_ranger.cpp`
> 对应设计：`docs/superpowers/specs/2026-10-10-双目测距v2-稀疏Census-design.md`（设计冻结 2026-10-10）
> 参考实现（已验证）：`tools/camera_test_ok/{stereo_core.py, distance_app.py}`、`tools/camera_test_ok/C++移植指南.md`
> 取代 v1.0（`2026-09-23-双目测距v1.0-design.md`）：**v1.0 两目内参互换，整链失效**，见「排查要点」。

## 功能职责

- **StereoRangerCore**（纯算法）：输入一帧**基准灰度图 + 辅助灰度图 + 左目检测框**，
  完成 校正（预置矩阵）→ 行带 remap → Shi-Tomasi 关键点 → Census 7×7 →
  粗/精搜索 + 抛物线亚像素 + 唯一性 + 左右一致性 → bbox 近端簇聚合
  （Z/X/Y/欧氏/IQR）→ EMA 平滑。不碰 Topic/线程/日志；线程不安全，由组件壳独占调用。
- **StereoRanger**（组件壳）：订阅左目检测框与左右目原始帧，按 `frame_sequence` 把检测
  成批、按时间窗把左右帧配对，交 core 测距，每个左目检测框发布一条 `StereoTargetDistance`。

### 目序（写死，勿动）

| 角色 | 来源 | 物理身份 |
|---|---|---|
| 基准图（视差/YOLO/3D 坐标系） | `kDecodedFrameLeft`（SBS 右半幅） | **物理左目** |
| 辅助图（只用于匹配） | `kDecodedFrameRight`（SBS 左半幅） | 物理右目 |

标定文件（`config/camera_calibration_uvc_1280x720.json`）的 `left`/`right` 指 **SBS 半幅**、
不是物理目；`T_m[0]=+0.0601>0`（cam1 在 cam2 的 +x 侧）⇒ SBS 左半是物理**右**目。
校正以 cam2（SBS 右半）为基准：`R'=Rᵀ、T'=−RᵀT`，`stereoRectify(K_ref=K2, D2, K1, D1, …, R', T')`；
视差 `d = u_ref − u_aux > 0`，`Z = fxB/d`，`fxB = fx_rect × |T| = 63.57 m·px`。
主工程拆分器（`stereo_frame_splitter`）的左目输出已是 SBS 右半幅，语义天然一致。

### 边界（v2）

- **影子链路组件**：发布 `kStereoTargetDistance`，消费者为 `VisualTrackingShadow`
  （距离通道，可选）与 `TargetEstimator`（视觉融合，需 `enable_visual_position_fusion`）；
  不产生飞行控制输出；不注册健康源。
- **只在基准图跑检测**：无右目检测实例（`yolo_right` 已删除，省约一半 NPU）。
- 不做多目标跟踪（`track_id` 恒 0）、不做 NED 融合、不做饱和点剔除（仅截断告警）。
- 校正矩阵**预置**（Python 一次性导出），C++ 不引入 OpenCV。

## 接口与数据流

```
Topic<DetectionResult> kDetection（基准图检测框，原始左图坐标） ─┐
Topic<FrameHandle> kDecodedFrameLeft （基准图=SBS右半,NV12）   ─┼─► StereoRanger（消费线程）
Topic<FrameHandle> kDecodedFrameRight（辅助图=SBS左半,NV12）   ─┘        │
  帧环形缓冲（按 sequence，深度 frame_buffer_depth）；                    │
  检测按 frame_sequence 成批（新序号或 settle 超时闭合）；                 ▼
  ≤pair_window 时间窗配对左右帧 → core.Process            Topic<StereoTargetDistance>
                                                                        │
                                                                        ▼
                                    VisualTrackingShadow::SetDistanceInput（可选）
                                    valid 且未超龄 → 距离 PID 接管（影子观察）；
                                    无效/未接线 → 回落 no_distance_action=kSlowApproach；
                                    TargetEstimator::SetVisualDistanceInput（可选，
                                    视觉融合开关）→ 局部 NED 观测更新滤波器（影子）
```

`StereoTargetDistance` 字段（`include/common/types.h`）：

| 字段 | v2 语义 |
|---|---|
| `frame_sequence` | 基准（左目）帧序号，取自检测批次的 `frame_sequence` |
| `forward_distance_m` | 前向距离 Z（米，EMA 后）；不可用 NaN |
| `slant_range_m` | 斜距（按平滑后 Z 重算）；不可用 NaN |
| `disparity_px` | 近端簇中位视差（px，校正图坐标系）；不可用 NaN |
| `confidence` | 左目检测置信度 |
| `valid` | 聚合成功（近端簇点数 ≥ `min_points`） |
| `center_pixel_x_mid` | 腐蚀后框中心 x（校正图）；不可用时取原始框中心 x |
| `camera_x_m` / `camera_y_m` | 目标在基准相机系（x 右 / y 下 / z 前）的横向/纵向位置（米，EMA 后）；z 即 `forward_distance_m`；valid=false 时 NaN。供 TargetEstimator 视觉融合（`visual_ned_converter`） |
| `bbox_x/y/w/h` | 左目检测框（**原始左图坐标**，未腐蚀） |
| `header.source_time_ms` | 基准帧的拆分完成时刻（帧元数据 `timestamp_ms`） |

配置（`config.json` 的 `stereo_ranger` 段，默认值即参考实现实测）：

`disp_min_px=8.0`（约 7.9m）、`disp_max_px=160.0`（约 0.40m）、`coarse_stride=2`、
`fine_halfwidth=6`、`uniqueness_ratio=1.05`、`lr_tol_px=1.0`、`census_win=7`、
`max_points_per_target=80`、`grid_cells=4`、`warmup_frames=15`、
`black_frame_mean_threshold=2.0`、`bbox_erode_px=5`、`min_points=30`、`near_pct=90.0`、
`near_tol_px=3.0`、`smooth_alpha=0.4`、`collect_detailed_latency=true`（诊断开关，开发
阶段开、生产定型后置 false，见下）、`input_queue_capacity=2`、`frame_buffer_depth=4`、
`pair_window_ms=20`、`settle_ms=50`、`summary_log_interval_ms=10000`。
`StereoRangerConfig::Validate()` 对非法取值抛 `std::invalid_argument`。

总开关 `runtime.enable_stereo_ranging`（默认 false；依赖链：enable_video +
camera_source=uvc + stereo_split + enable_visual_tracking，config.cpp 校验）。
启用时由 config 模块从 **`config/stereo_rectify_uvc_1280x720.json`**（与 config.json 同目录，
`tools/camera_calibration/export_rectify.py` 生成）加载预置矩阵；缺失/不自洽即启动失败。

Topic 一览：

| Topic 常量 | 消息类型 | 生产者 | 消费者 |
|---|---|---|---|
| `kDetection` | `DetectionResult` | YoloDetector（基准图） | FrameCompositor、StereoRanger |
| `kDecodedFrameLeft/Right` | `video::FrameHandle` | StereoFrameSplitter | YoloDetector/叠加器/图传、StereoRanger |
| `kStereoTargetDistance` | `StereoTargetDistance` | StereoRanger | VisualTrackingShadow（可选）、TargetEstimator（视觉融合，可选） |

## 关键实现点（语义照抄参考实现，逐条可核对）

- **校正映射表**：`P_ref⁻¹` 归一化 → `R_refᵀ` → 正向畸变模型 → `K_ref` 投影，
  得到 rect→source 采样坐标（float32，init 时一次构建，约 14MB）。
  `SourceToRectRef` 为解析逆映射（畸变逆解定点迭代 10 次），用于把检测框从原始坐标
  映射到校正坐标（四角+四边中点包围盒，向外取整）。
- **行带 + 列裁剪**：只在检测框联合区域 ±（采样余量 + Census 半径）的行带上做 remap、
  Census 与提点，行带内再按 `框 ± 2r ± ceil(disp_max)` 裁列（带内结果与全幅逐像素一致：
  四类被访问列——参考窗口/目标窗口/LR 回搜/回搜参考列——均在裁剪区间内或与全幅在
  同一画面边缘处裁剪；Census 只依赖自身邻域）；窄框 vs 全宽框等价性有专门单测压住。
- **诊断开关 `collect_detailed_latency`（开发阶段默认 true）**：控制阶段耗时统计与逐点
  调试快照；**关闭时热路径零诊断**——不读阶段计时、不调 `LatencyStatistics::Add`、
  不留快照（`LastPointDebugForTest` 返回空），测距结果不变（有专门用例守住）。
  与视频链 `collect_detailed_latency` 同一约定：开发阶段保持打开记录数据，**生产
  调优定型后置 false**（对照 `tools/NPU性能分析探针使用方法.md` §6「不在热路径增加
  诊断调用」）。开发机 A/B 实测（bench 开/关各 3 轮、每轮 60 帧）差异在 ±0.5ms 轮间
  噪声内，即当前诊断开销量不出来。跑基准/看阶段占比见
  `tools/双目测距性能基准使用方法.md`。
- **热路径性能（2026-10-10 开发机基准 `stereo_ranger_bench`，400×380 框）**：
  107.2ms → **14.1ms/帧**（remap 3.6 | census 4.5 | 提点 4.7 | 匹配 1.6 | 聚合 0.004）。
  关键手段：Census 源行指针按输出行提升、内部列免裁剪分支、win=7 邻位常量展开；
  remap 四抽头在图内时走无分支快路径；Shi-Tomasi 临时缓冲跨帧复用、最小特征值/
  候选只在掩膜包围盒 ±(block/2+1) 内计算（区域包含全部候选与掩膜像素，结果不变）。
  板上数字约为开发机的 1/2~1/4，可用摘要的「双目测距阶段耗时」行对照。
- **Census**：以中心像素为阈值（`>=` 判据）的 7×7 邻域（跳过中心，48 位 → `uint64`），
  越界按复制延拓。位序自定，但**内部列的常量展开路径与边界列的通用裁剪路径必须位序
  一致**（否则边界附近的关键点会整片失配）——有专门单测逐像素对拍独立规格实现守住；
  改动任一编码路径必须同步改另一处并跑该用例。
- **代价引擎**：基准点窗口预提取一次；候选视差只做**列平移**（`x_R = x_L − d`），
  行不变；越界**按行列分别裁剪**（线性索引裁剪会跨行回绕）。
- **匹配主流程**：粗搜索（stride=2，候选不足 3 个时 9 点等分）→ 精搜索（±6，step=1）
  → 抛物线亚像素（clip ±1）→ 唯一性（峰位 **±2 之外**的次优/最优 ≥ 1.05）
  → 左右一致性（以右图点回搜 `x_L = x_R + d`，**加号**；容差 1.0px）。
- **关键点**：Shi-Tomasi（blockSize=5、quality=0.01、minDistance=8、3×3 局部极大、
  按质量降序 + minDistance 贪心抑制）+ **4×4 网格轮转**取点，每目标 ≤80。
  与 OpenCV `goodFeaturesToTrack` 非逐位一致（见「测试方式」的隔离设计）。
- **聚合（bbox 近端簇）**：框向内腐蚀 5px → 取框内有效匹配点（< `min_points` 判不可用）
  → `near_pct=90` 分位 `d_near` → 聚 `d ≥ d_near − 3px` 的近端簇（簇内不足退回全体）
  → 簇内**中位数** → `Z = fxB/d_med`；X/Y 用腐蚀后框中心与 `P_ref`；
  IQR = p75−p25 作置信度。百分位为 numpy 语义（线性插值）。
- **帧间平滑**：按 `(class_id, x1//40, y1//40)` 分桶 EMA（`smooth_alpha=0.4`）；
  X/Y/欧氏距离按平滑后 Z 重算（`scale = Z_f/Z_raw`），保证自洽。
- **截断告警**：`d_med ≥ disp_max − 2` 置截断标志（摘要计数 + 节流 WARN），
  提示距离可能更近（量程近端 0.4m 顶格）。
- **组件帧/批次**：帧缓冲持有 Topic 消息指针（占用拆分器内存池槽位，深度 4，
  受池容量 8 约束）；批次闭合 = 新 `frame_sequence` 到达或 `settle_ms` 超时；
  等帧上限 200ms，超限/被挤出时该批次每框发一条 valid=false（保持节拍）。
- **线程模型与停机**：`Start()` 订阅三路输入（队列容量 2，丢最旧）并启动消费线程；
  10ms 节拍阻塞等待左目检测驱动 Pump；`Stop()` 置停止标志 → Reset 三路订阅唤醒 →
  join → `core.Reset()`。Start/Stop 幂等。

## 日志行为

| 场景 | 等级 | 节流 |
|------|------|------|
| 创建（fx_rect/基线/fxB/视差区间）、启动、停止（累计计数） | INFO | 否 |
| 摘要：帧数/匹配帧/批次/发布/有效测距/有效占比/距离 min/avg/max/跳过/帧未配/截断/处理耗时 | INFO | `summary_log_interval_ms`；**静默窗口不打印**（见下） |

> **静默抑制（2026-10-12 实施，故障排查手册 §19）**：窗口内**无帧、无发布、无截断、
> 无未配对**（四项累计计数均无增量，即无目标时的全 0 行）⇒ 整段摘要（含阶段耗时行）
> 跳过不打印；启动后首个窗口仍打印一次以确认组件活着。一旦有帧/发布/截断/未配对，
> 摘要照常出现。纯配置兜底仍可调大 `summary_log_interval_ms` 或提高 `log.level`。
| 阶段耗时细分：remap/census/提点/匹配/聚合 的 avg/p95（滚动窗口；**仅 `collect_detailed_latency=true` 时输出**，判读见 `tools/双目测距性能基准使用方法.md`） | INFO | `summary_log_interval_ms` |
| 批次未配对到帧（左/右帧缺失或被挤出） | WARN | 第 1 次 + 每满 100 次 |
| 视差顶到搜索上限（截断） | WARN | 第 1 次 + 每满 100 次 |
| 启动时三路输入未接线 | ERROR | 否（启动路径一次性，Start 返回 false） |
| 逐帧匹配/聚合 | 不打日志 | — |

排查计数：`ProcessedFrameCount()`/`MatchedFrameCount()`/`AvailableCount()`/
`ClippedCount()`/`SkippedFrameCount()`/`BatchCount()`/`FrameMissCount()`/
`ErrorCount()`/`LastMatchMs()`。

## 测试方式

- **开发机（WSL2 Ubuntu 24.04）**：
  ```bash
  cmake -S . -B build && cmake --build build -j$(nproc)
  ./build/stereo_ranger_core_test   # 映射表黄金采样点（对拍 OpenCV）、源↔校正往返、
                                    # 黄金场景端到端对拍 Python 参考实现、
                                    # Census 编码规格对拍（内列常量展开 vs 边界通用
                                    # 裁剪两条路径逐像素对独立规格实现）、
                                    # 边缘关键点跨编码分界匹配、列裁剪等价（窄框 vs
                                    # 全宽框逐位一致）、合成视差场复原/近端簇/截断/EMA、
                                    # 预热与黑帧跳过、配置与矩阵校验
  ./build/stereo_ranger_test        # 组件：接线校验、settle/新序号闭合、每框一条、
                                    # 右帧缺失降级、Stop 幂等与重启、
                                    # 摘要静默抑制（静默不打印/活动恢复打印）
  ctest --test-dir build --output-on-failure     # 基线 604/604
  ./build/stereo_ranger_bench 30    # 性能基准（开发机；输出各阶段 avg/p95，内部自动开诊断）
  ./build/stereo_ranger_bench 30 --no-stats   # 同上但关诊断（=生产默认），用于 A/B 诊断开销
  ```
  黄金数据（`tests/perception/data/`）由 `tests/perception/gen_stereo_golden.py`
  用参考实现生成；Shi-Tomasi 与 OpenCV 的差异用「注入关键点」（`SetKeypointsOverride`）
  在核心单测中隔离，端到端用例（组件测试）走真实提点。
  测试钩子（仅测试用）：`CensusEncodeForTest`（暴露编码，校验两条路径位序）、
  `LastPointDebugForTest`（逐关键点视差/唯一性比值/LR 回搜最优候选/有效性，
  用于定位个别关键点失配）。
- **实机精度验证在独立测试工程完成（主工程不做板上卷尺验收）**：按指南 §9 在
  Python 参考工程（`tools/camera_test_ok/`）做卷尺对照（非周期纹理平面
  0.5/1.0/1.5/2.0m，1m 内 ±2%、2m 处 ±5%，**禁用棋盘格/标定板**）、单调性与
  极线残差验证；结论（`fxB` 绝对精度、基线实测值、参数档位）确认后回填主工程
  配置与本文。主工程侧只在运行中观察摘要日志（有效占比/IQR/距离 min/avg/max/
  匹配耗时）作快速判读。
  **优先级（2026-10-12 维护者决定）**：前期参考工程实测精度可接受（无量化回填）；
  当前距离消费者全为影子（无控制输出依赖精度），卷尺级验收与 `fxB` 对齐降级为
  **阶段四（正式控制接入）前置条件**——距离尺度错误会直接转化为接近速度错误，
  接入控制前必须完成。
  **室外现场使用（2026-10-20 回填）**：目标 **<8m 可正常测距**；近距约 1.9m 实测
  有效占比 **99.2%**（2772/2793），无帧未配、无截断。该结论只说明量程内可用，
  **不替代**卷尺级精度验收与 `fxB` 绝对对齐。
  **遗留**：`fxB` 绝对精度尚未与物理卷尺对齐（指南 §9.1）——若整体偏固定比例，
  卡尺量镜头中心距并回填基线后重跑 `export_rectify.py`；该对齐同样在独立测试工程
  完成后再回填。

## 排查/修改要点

- **v1.0 失效根因（务必牢记）**：标定文件的 left/right 是 **SBS 半幅**语义，而拆分器
  左目 Topic 是 **物理左目**（SBS 右半）。v1.0 把标定 `left` 内参（cx=620.73）用在左目
  Topic（真实 cx=674.76）上 ⇒ 归一化视差恒多出 +0.107 ⇒ Z 恒 0.36~0.56m ⇒ valid 恒 false。
  v2 用预置矩阵（以 cam2 为基准）从根上消除该配对歧义；**任何"按帧坐标直接套内参"的
  改动都会重新引入该类错误**。
- **valid 恒 false**：先看摘要「有效占比」。配对率正常但有效为 0 →
  ① 目标超出量程（`disp_min/max` 决定 7.9m~0.4m）；② 纹理不足（弱纹理/纯色目标
  关键点少）；③ 标定档位与当前分辨率不符（矩阵 `view_width/height` 必须 = 拆分后
  每目尺寸）；④ 检查 `stereo_rectify_uvc_1280x720.json` 是否与当前相机标定同源。
- **现场观察（2026-10-15 室外 0%，已定位 2026-10-20）**：发布正常但有效占比 0%，
  根因是**目标距离超出 `disp_min_px=8` 对应的 ≈7.95m 量程**；目标进入 <8m 后
  近距实测有效占比 99.2%（2772/2793）。室外使用时先确认目标在量程内，再看纹理/
  配对；见 `docs/故障排查手册.md` §11.1。
- **距离系统性偏差**：优先怀疑标定（fx/基线/主点）与实现所用矩阵不同源；
  固定比例偏差 ⇒ 卡尺量基线并重跑导出脚本。
- **帧未配（WARN）**：右帧缺失/被挤出 ⇒ 查拆分器是否丢帧（输出池满）、
  左右目时间戳差是否超 `pair_window_ms`（实测 0~1ms）；帧缓冲深度过小
  （< 检测推理时延/帧周期）也会导致批次拿不到帧。
- **截断告警（WARN）**：目标进入量程下限（≈0.4m，`disp_max` 决定）；需要更近量程
  时增大 `disp_max_px`（代价是搜索变慢）。
- **修改关联**：`StereoTargetDistance` 字段改动需同步 `docs/数据接口文档.md`、
  `include/common/types.h` 注释并重跑三件套测试；标定更新后必须重跑
  `tools/camera_calibration/export_rectify.py` 并同步 `config.json` 的 `stereo_ranger` 段；
  控制律侧距离语义见 `include/control/tracking_control_law.md`。
