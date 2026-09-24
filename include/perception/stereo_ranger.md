# stereo_ranger 实现文档

> 对应实现：`include/perception/stereo_ranger_core.h`、`src/perception/stereo_ranger_core.cpp`、
> `include/perception/stereo_ranger.h`、`src/perception/stereo_ranger.cpp`
> 对应设计：`docs/superpowers/specs/2026-09-23-双目测距v1.0-design.md`（设计冻结 2026-09-23）
> 更新：2026-09-24（v1.0 已实现，影子接入，待板上验证）

## 功能职责

- **StereoRangerCore**（纯算法）：输入一对已时间配准的左右目 `DetectionResult`
  数组，完成 匹配（硬约束 + 代价贪心）→ 去主点视差 → `Z = B/D_norm` 三角测量
  → 有效域判定 → Z 一阶低通滤波与跳变重置 → |Δy| 在线校验统计。不碰
  Topic/线程/日志，线程不安全，由组件壳独占调用。
- **GreedyStereoMatcher**（默认匹配器，`IStereoMatcher` 接口注入可替换）：
  硬约束过滤后按代价升序贪心一一配对。
- **StereoRanger**（组件壳）：双订阅左右目检测 Topic，按
  `header.source_time_ms` 以 ±`pair_window_ms` 时间邻接配对左右帧，交 core
  测距，每个左目目标发布一条 `StereoTargetDistance`（未匹配 valid=false 照常
  发布方位）。独立消费线程，与 YoloDetector 相同的线程模型。

边界（v1.0）：
- **影子链路组件**：发布 `kStereoTargetDistance`，唯一消费者是
  `VisualTrackingShadow` 的距离通道（可选输入）；不产生飞行控制输出；
  不注册健康源（本轮设计决定）。
- **不做 remap 立体校正**：直接使用原始像素与标定内参；校正质量由 |Δy| 在线
  残差统计兜底，持续超门限打 WARN 提示评估升级 remap（v1.1）。
- **v1.0 单目标约定**：`track_id` 恒 0，多目标只做类别 + 最近邻滤波关联，
  全量跟踪留 v1.1。
- 右目无检测帧时 YOLO 不发布消息，走左帧超龄路径（按空右帧处理）。
- 不做 NED 卡尔曼融合、不改控制律数学、方位通道不切换 x_mid（字段照发，
  接入切换留后续）。

## 接口与数据流

```
Topic<DetectionResult> kDetection（左目）       ─┐
                                                 ├─► StereoRanger（消费线程）
Topic<DetectionResult> kDetectionRight（右目）  ─┘        │
        按 header.source_time_ms 分组为帧；               │
        左帧在挂起右帧中找 |Δt|≤pair_window_ms 的最近者   ▼
                                     Topic<StereoTargetDistance> kStereoTargetDistance
                                                │
                                                ▼
                            VisualTrackingShadow::SetDistanceInput（可选）
                            valid 且未超龄 → 距离 PID 接管（影子观察）；
                            无效/未接线 → 回落 no_distance_action=kSlowApproach
```

配对语义（组件壳）：
- 消费线程以 10ms 节拍阻塞等待左目消息驱动 Pump；右目消息只入挂起表。
- 左帧超龄（now−t > pair_window）仍无右帧配对时，按空右帧调 core 并照常发布
  （valid=false，方位字段取左目），保证下游每个左目目标都有节拍。
- 超龄右帧（> 4×pair_window）直接丢弃；左右目无检测帧不发布消息。
- 拆分器左右帧时间差实测 0~1ms，因此用 ±pair_window 邻接配对而非同帧相等。

`StereoTargetDistance` 字段（`include/common/types.h`）：

| 字段 | 含义 |
|---|---|
| `header` | `sequence`/`receive_time_ms` 由组件发布时填写；`source_time_ms` 取左目帧 |
| `frame_sequence` | 左目帧序号 |
| `class_id` / `track_id` | 类别（0=无人机）；v1.0 track_id 恒 0 |
| `forward_distance_m` | 前向距离 Z（米，滤波后）；valid=false 时 NaN |
| `slant_range_m` | 斜距 R（米，参考值）；valid=false 时 NaN |
| `disparity_px` | 归一化视差折算左目像素视差；valid=false 时 NaN |
| `confidence` | 左右目置信度取小；未匹配时为左目置信度 |
| `valid` | 匹配成功且在有效域内（视差≥门限且距离在域内） |
| `center_pixel_x_mid` | 双目中心 x 中值；未匹配时为左目中心 x |
| `bbox_x/y/w/h` | 左目检测框（像素），供交叉校验与调试 |

Topic 一览：

| Topic 常量 | 消息类型 | 生产者 | 消费者 |
|---|---|---|---|
| `kDetection` | `DetectionResult` | YoloDetector（左目） | FrameCompositor、StereoRanger |
| `kDetectionRight` | `DetectionResult` | 右目 YoloDetector | StereoRanger |
| `kStereoTargetDistance` | `StereoTargetDistance` | StereoRanger | VisualTrackingShadow（可选） |

配置（`stereo_ranger` 段，默认值即 2026-09-23 标定实测
`config/camera_calibration_uvc_1280x720.json`，由 config.cpp 解析覆盖）：
`baseline_m=0.060085`、左目 `fx=1007.82/fy=1008.02/cx=620.73/cy=532.07`、
右目 `fx=1006.59/fy=1006.07/cx=674.76/cy=526.70`、`disparity_min_px=3.0`、
`distance_min_m=1.0`、`distance_max_m=11.0`、`tau_y_px=2.0`（⚠初值待板上回填）、
`tau_h_ratio=0.25`、`smooth_alpha=0.3`、`jump_reset_m=5.0`、
`pair_window_ms=20`、`summary_log_interval_ms=10000`。
`StereoRangerConfig::Validate()` 对非法取值抛 `std::invalid_argument`。
右目检测器由 `yolo_right` 段配置（缺省字段全部继承 `yolo`，仅
`npu_core_mode` 独立）。总开关 `runtime.enable_stereo_ranging`（默认 false，
依赖链：enable_video + camera_source=uvc + stereo_split + enable_visual_tracking，
config.cpp 校验）。

## 关键实现点

- **去主点视差（v1.0 最大的坑）**：两目主点 cx 实测差约 54px
  （左 620.73 / 右 674.76），直接用框 x 坐标差算视差会在近距出现符号反转。
  实现先做焦距归一化 `uL = (xL − cx_left)/fx_left`、
  `uR = (xR − cx_right)/fx_right`（uL/uR 无量纲），取归一化视差
  `D_norm = uL − uR`，三角测量 `Z = B / D_norm`（B = baseline_m）。
  折算左目像素视差 `D_px = D_norm·fx_left`（消息 `disparity_px` 字段即此值），
  等价写法 `Z = fx_left·B / D_px`，实测 `fx_left·B ≈ 1007.82 × 0.060085 ≈ 60.57`。
  有专门的近距符号回归用例
  （`NearDistanceSignRegression_PrincipalPointGap54px`）。
- **贪心匹配**（设计 §3.2 原样）：硬约束——类别一致；|Δy|（去 cy）≤
  `tau_y_px`；去主点视差对应距离在 `[distance_min_m, distance_max_m]`；
  框高比 |hL−hR|/max ≤ `tau_h_ratio`。代价
  `cost = |Δy|/tau_y + |Δh|/(tau_h·max)`，按 cost 升序逐一配对，cost<1
  才接受，一一配对。目标数少，O(n²) 足够；匹配器接口注入，可替换匈牙利。
- **有效域 ≤11m**：判据 `valid ⇔ 视差 ≥ disparity_min_px(3px) 且 Z ∈
  [1, 11]m`。11m 上限来自误差模型 δZ ≈ Z²·δD/(F·B) 按实测 F·B=60.57、
  δD=1px 取 δZ≤2m 的边界（思路文档 §5.3 已按实测重算）。
- **滤波与跳变重置**：Z 一阶低通 `Z_f = α·Z + (1−α)·Z_prev`（α=0.3）；
  |ΔZ| > `jump_reset_m`(5m) 直接重置不滤波，抑制单帧抖动又不拖慢真实跳变。
  未匹配不清空滤波器；`ResetFilter()` 由组件 Stop 时调用。
- **|Δy| 在线校验（不 remap 的代价兜底）**：core 对每对匹配维护 |Δy|（去 cy）
  滑窗 P95（无样本 NaN）；组件摘要日志输出，连续 3 个摘要周期超 `tau_y_px`
  打 WARN，提示未 remap 校正质量不足、评估升级 remap（v1.1）。
- **线程模型与停机**：`Start()` 订阅双输入（队列容量 `input_queue_capacity` 默认 2，
  丢最旧）并启动消费线程；`Stop()` 置停止标志 → Reset 双订阅唤醒阻塞等待 →
  join → `core.ResetFilter()`。Start/Stop 幂等。热路径零日志。
- **输出契约**：每个左目目标恰好一条输出；core 只填 `frame_sequence` 与
  `header.source_time_ms`，`header.sequence`（本地单调计数器，跨重启不重置）
  与 `receive_time_ms` 由组件发布时填写。
- **装配**（`DroneApplication`）：`enable_stereo_ranging=true` 时创建右目
  YoloDetector（订阅 `kDecodedFrameRight`）与 StereoRanger（左输入接左目
  `DetectionOutput()`，右输入接右目 `DetectionOutput()`），`DistanceOutput()`
  接 `VisualTrackingShadow::SetDistanceInput()`；右目检测器不接入健康上报
  （本轮设计决定），右目链路降级经 StereoRanger 摘要的配对率/有效域占比
  可观测；StereoRanger 不注册新健康源。

## 日志行为

| 场景 | 等级 | 节流 |
|------|------|------|
| 创建（shadow=true、有效域）、启动、停止（累计计数） | INFO | 否 |
| 摘要：左帧数/匹配对/配对率/有效数/有效域内占比/\|Δy\| P95/测距 avg/max | INFO | `summary_log_interval_ms`（默认 10s） |
| \|Δy\| P95 连续 3 个摘要周期超 tau_y_px | WARN | 摘要节拍（提示评估升级 remap v1.1） |
| 启动时左/右目输入未接线 | ERROR | 否（启动路径一次性，Start 返回 false） |
| 逐帧配对/测距 | 不打日志 | — |

口径说明：摘要中配对率/有效域占比为启动以来累计计数（`ProcessedPairCount()`/
`MatchedPairCount()`/`ValidDistanceCount()` 单调累计），测距耗时 avg/max
按摘要周期重置（每周期清零重新统计）。

排查计数：`ProcessedPairCount()`（已处理左目帧数，含超龄未配对）、
`MatchedPairCount()`、`ValidDistanceCount()`、`ErrorCount()`、
`DeltaYP95Px()`。

## 测试方式

- **开发机（WSL2 Ubuntu 24.04）**：
  ```bash
  cmake -S . -B build && cmake --build build -j$(nproc)
  ./build/stereo_ranger_core_test   # 匹配器硬约束逐条拒绝/贪心一一配对、
                                    # 去主点视差数值（含 cx 差 54px 符号回归）、
                                    # 有效域边界（11m valid / 15m invalid / 视差门限）、
                                    # 低通滤波与跳变重置、无匹配 valid=false、
                                    # |Δy| P95 滑窗、配置校验、Mock 匹配器注入
  ./build/stereo_ranger_test        # 组件：未接线拒绝启动、时间窗配对、
                                    # 右目乱序先到仍配对、左帧超龄 valid=false、
                                    # Stop 幂等与重启
  ./build/visual_tracking_shadow_test --gtest_filter='*Distance*'
                                    # 影子距离接通：valid 距离驱动距离 PID、
                                    # 无效/超龄回落 kSlowApproach
  ctest --test-dir build --output-on-failure   # 基线 333/333
  ```
  配置解析用例（缺段默认、非法值拒绝、依赖链校验）在 `config_test`。
- **香橙派实机（待板上验证，留后续会话）**：静态标靶 3/5/8/11m 手持激光
  测距仪对照、|Δy| P95 实测回填 `tau_y_px`、双推理并行帧率/时延回填
  （思路文档 §9 预算表）、右目绑核 vs `all` 实测、摘要日志观察。

## 排查/修改要点

- **无距离输出（kStereoTargetDistance 无消息）**：先确认
  `enable_stereo_ranging=true` 且依赖链齐全（配置加载期会拒绝缺依赖的
  启用）；再查右目检测器是否启动（订阅 `kDecodedFrameRight`，需
  `camera_source=uvc` + `stereo_split=true`）；再看摘要日志配对率——左帧
  全部超龄未配对说明右目无检测或 `pair_window_ms` 过窄（拆分器左右帧
  时间差实测 0~1ms，默认 20ms 足够，异常时先查右目推理是否运行）。
- **valid 恒 false**：配对率正常但有效域内占比低——查目标距离是否超出
  有效域 [1, 11]m、视差是否低于 `disparity_min_px`(3px)，或标定内参与
  当前相机档位不符（内参对应 1280×720 单目档）。
- **近距距离符号/数值异常**：优先怀疑去主点视差被动过——两目 cx 差约
  54px，任何直接用框 x 坐标差的改动都会在近距翻车；回归用例
  `NearDistanceSignRegression_PrincipalPointGap54px` 必须保持通过。
- **|Δy| P95 摘要持续超 tau_y_px（WARN）**：未 remap 的校正质量不足，
  评估升级 remap（v1.1）；短期可对照标定文件确认内参未漂移。
- **修改关联**：`StereoTargetDistance` 字段改动需同步
  `docs/数据接口文档.md` 与 `include/common/types.h` 注释并重跑
  `stereo_ranger_core_test` / `stereo_ranger_test` /
  `visual_tracking_shadow_test`；标定更新（`tools/camera_calibration/`）
  后必须同步 `config.json` 的 `stereo_ranger` 段与本文默认值；
  控制律侧距离语义见 `include/control/tracking_control_law.md`。
