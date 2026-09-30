# stereo_depth 实现文档

> 对应实现：`stereo_depth/`（独立子工程，静态库 `stereo_depth`）
> 对应设计：`docs/superpowers/specs/2026-09-29-传统双目测距传感器-design.md`
> 状态：v1 已实现（纯 CPU / ROI 全分辨率稠密匹配），开发机单测 33/33 通过；待板上实测精度/帧率回填。

## 功能职责

一个**纯 CPU、不依赖 NPU** 的距离传感器：输入一对已拆分的左右目灰度帧 + 一个
查询矩形（原始左目像素系，与 YOLO 框一致），输出该区域的前向距离（米）+ 视差 +
置信度。核心是**稠密立体匹配**（校正 → Census → 简化 SGM → 亚像素 → 后处理 →
区域聚合），替代现有 `stereo_ranger` 的「框中心去主点视差」，把 12m 处误差从
米级降到分米~半米级。

**做什么**：立体校正 remap、ROI 稠密匹配、亚像素视差、深度换算、输出级滤波。
**不做什么**：目标检测/跟踪、控制量、NPU 推理、MJPG 采集解码（半链路：接收主
项目 `StereoFrameSplitter` 已拆分的 NV12 帧，取 Y 平面）、NED 融合、硬件基线改动。

**与主项目关系**：并列的**可选**距离传感器，不替换 `stereo_ranger`；子库不依赖
主项目任何头文件（Topic/VideoFrame/types.h 均不引入），主项目侧通过薄适配器接入。

## 接口与数据流

```
主项目 StereoFrameSplitter.LeftOutput()/RightOutput()（FrameHandle, NV12）
        │  适配器取 Y 平面(data/hor_stride/timestamp) 组 StereoFramePair
        ▼
StereoDepthSensor::Process(pair)   —— 全分辨率 remap 校正到内部 rect_left_/rect_right_
        │
适配器把 YOLO 框转 QueryRect（原始左目像素系，零转换）
        ▼
StereoDepthSensor::Query(rect)     —— ROI 匹配 + 区域聚合 + 深度 + 输出级滤波
        ▼
DistanceResult{distance_m, disparity_px, confidence, valid, valid_pixel_ratio_x100}
        │  适配器映射到 StereoTargetDistance（字段复用，不改主项目类型）
        ▼
下游 VisualTrackingShadow 距离通道（语义不变）
```

对外类型（`include/stereo_depth/stereo_depth_types.h`）：
- `GrayImage{data,width,height,stride,timestamp_ms}`：单目灰度平面（NV12 的 Y 平面）。
- `StereoFramePair{left,right}`：一对已配准左右目。
- `QueryRect{x,y,w,h,track_id}`：查询区域，**原始左目像素系**；`track_id` 供滤波分轨。
- `DistanceResult`：距离/视差/置信度/有效性/有效占比。

模块划分：
| 文件 | 职责 |
|---|---|
| `stereo_depth_config` | 配置 + 标定结构、JSON 加载、`Validate()` |
| `stereo_rectifier` | stereoRectify 数学、正向 remap LUT、source→rectified 映射、深度换算 |
| `census_transform` | Census 符号串 + Hamming 距离 |
| `sgm_matcher` | `IStereoMatcher` 接口 + 简化 SGM（代价体/多路径聚合/WTA/唯一性） |
| `subpixel_refiner` | `ISubpixelRefiner` 接口 + 抛物线/V型线性两种亚像素估计器 |
| `disparity_postprocess` | 3×3 中值、speckle 去斑、区域鲁棒聚合 |
| `stereo_depth_sensor` | 组件壳：编排全流程 + 输出级滤波 + 计数 |

## 关键实现点

- **ROI 驱动全分辨率匹配（精度/实时性双达标的关键）**：每帧只做全分辨率 remap
  校正（便宜，LUT gather），昂贵的 Census+SGM 只在查询 ROI（目标框外扩
  `roi_margin_px` + 左侧 `max_disparity` 搜索余量）上跑。单目标 ROI 匹配 ~1-2ms，
  全分辨率精度（fx≈1007，非降采样）与 ≥30fps 同时达成。
- **同步实现（对设计 §8 的取舍）**：v1 采用同步 `Process`/`Query`，无后台流水线
  线程——ROI 已使单目标每帧成本远小于 33ms 帧预算。多目标并发或单帧内并行加速
  留待按需引入（见「排查/修改要点」）。
- **校正旋转（Hartley-Sturm，自研）**：`e_x=normalize(T)`（新 x 沿基线）、
  `e_z=normalize([-Tx·Tz,-Ty·Tz,Tx²+Ty²])`（与 T 正交且最接近原前向）、
  `e_y=e_z×e_x`（右手系），`R1=[e_x;e_y;e_z]`、`R2=R1·R`。Build 内校验
  `R1·T≈[B,0,0]`（偏差 >0.1% 抛 `logic_error`）。正确性由单测「理想标定下
  source→rectified 恒等 + 深度换算」验证，而非照抄 OpenCV 符号约定。
- **新内参 K_new**：`f=0.25·(fxL+fyL+fxR+fyR)`、`cx=(W-1)/2`、`cy=(H-1)/2`；
  左右共用，保证视差一致、`Z=f·B/d` 成立。
- **正向 remap LUT**：rectified 每像素经 `P⁻¹→R^T→归一化→施加 5 项畸变→K` 得
  source 采样坐标，运行时双线性采样；越界填 0（rectified 边缘无效区）。
- **source→rectified 直接映射**（无全分辨率逆 LUT）：对 QueryRect 4 角做
  `K⁻¹→畸变逆解(定点迭代 10 次)→R1→K_new`，取包围盒。畸变逆解同 OpenCV
  undistortPoints 定点法。
- **Census 7×7**（去中心 48 位存 uint64）：对光照/左右目独立 AE 亮度差鲁棒
  （硬件记录 §4：左右目曝光独立），是选它而非 SAD/NCC 的关键。
- **简化 SGM**：代价=Census Hamming（uint16，越界记 `kInvalidCost=4096`）；
  8 路径动态规划（P1 连续/P2 跳变，`-minPrev` 防溢出）；路径按 (dr,dc) 决定
  行/列扫描序，前驱已算。全代价体 + 单路径缓冲复用（`cost_`/`agg_`/`path_`）。
- **WTA + 唯一性 + 亚像素**：取最小聚合代价；`best > uniqueness_ratio·second`
  判歧义置 NaN；`ISubpixelRefiner` 细化。**默认 `LinearSubpixel`（V 型代价模型
  第一性推导）**：`r=(C₋−C₊)/(C₋+C₊−2C₀)=δ/(1−|δ|)`，反解 `δ=r/(1+r)`(r≥0)
  / `r/(1−r)`(r<0)，比裸抛物线偏差更小（单测 `LessBiasedThanParabolic` 证明）。
- **离群剔除（替代设计 §7 的 LR）**：v1 不做第二遍右视差 LR 一致性（会使 SGM
  成本翻倍且 ROI 裁剪复杂化），改用 **WTA 唯一性比 + 3×3 中值 + speckle 去斑 +
  区域中位数聚合 + 有效占比门限** 组合，对区域查询传感器已足够鲁棒。
- **区域聚合**：目标子区内取有效视差**中位数**（抗离群），并按有效域
  `[distance_min_m,distance_max_m]` 过滤、统计 `valid_ratio`；
  `Z=f·B/median_disp`。
- **输出级滤波（本库内，按 track_id）**：一阶低通 `Z_f=α·Z+(1−α)·Z_prev` +
  跳变重置 `|ΔZ|>jump_reset_m` 直接重置；`valid=false` 不更新滤波器；
  `ResetFilters()` 清空。
- **精度物理天花板**：`δZ≈Z²·δD/(f·B)`，`f·B≈60.5`。60mm 基线下 12m 处视差仅
  ~5px，δD=0.15~0.3px → δZ≈0.36~0.71m，是硬件基线决定的上限；给 Z(d) 加二次项
  无益（模型精确，二阶项仅 ~δD/D≈6%）。压低 δD（LinearSubpixel + 8 路径 + 7×7
  Census）是唯一软件杠杆，加大基线是唯一硬件杠杆。

## 日志行为

| 场景 | 等级 | 节流 |
|------|------|------|
| Init 完成（rect 尺寸/f/B/f·B/D/paths/census/subpixel/有效域） | INFO | 否 |
| Init 失败（配置/标定加载、分辨率不符、校正构建） | ERROR | 否（返回 false） |
| Process 未初始化 / 帧尺寸非法 | WARN | 第 1 次 + 每满 100 次 |
| 逐帧 Process/Query 热路径 | 不打日志 | — |

排查计数：`ProcessedFrameCount()`、`QueryCount()`、`ValidCount()`、`ErrorCount()`。

## 测试方式

- **开发机（WSL2 Ubuntu）**：
  ```bash
  cd stereo_depth
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build -j$(nproc)
  ./build/stereo_depth_unit_tests          # 33 用例
  ctest --test-dir build --output-on-failure
  ```
  覆盖：配置校验/加载、真实标定加载（含 cx 差 54px 断言）、校正理想恒等 + 深度
  换算 + 真实标定构建、Census 恒等/平移等变/边界、SGM 已知视差复原（命中率 >90%）、
  亚像素两种估计器 + Linear 偏差更小、后处理中值/speckle/区域聚合/有效域门控、
  传感器生命周期 + **合成端到端距离复原（真值 1.0m，容差 0.15m）** + 滤波/Reset。
  > 注：本机 WSL 为 Ubuntu 22.04；AGENTS.md 规定唯一构建环境为 24.04，最终验证须在 24.04。
- **离线 demo**：`./build/stereo_depth_demo left.pgm right.pgm [calib] [config] [x y w h]`
  读一对 PGM 灰度 + 标定跑全流程打印区域距离（脱离主项目验证）。
- **香橙派实机（待验证）**：静态标靶 2/3/5/8/11/12m 手持激光测距仪对照回填精度表
  与真实 δD；实测 30fps/各段耗时/多目标扩展性；据此定 `roi_margin_px`/`sgm_paths`/
  `census_window` 档位。

## 排查/修改要点

- **Query 恒 invalid**：先确认 `Process` 返回 true 且 `HasFrame()`；再看
  `valid_pixel_ratio_x100`——占比低于 `min_valid_ratio` 说明 ROI 弱纹理/超有效域；
  查目标距离是否在 `[distance_min_m,distance_max_m]`、视差是否 ≥`disparity_min_px`。
- **距离系统性偏差**：优先怀疑标定（`f`/`B`/内参与当前相机档位是否一致，标定档位
  须 = `source_width×source_height`）；其次 `subpixel` 估计器与 `penalty_p1/p2`。
- **近距符号/数值异常**：本方案已 remap 校正（不同于 v1.0 不去主点），主点差不再
  影响；若仍异常查 `SourceToRectifiedLeft` 的畸变逆解是否收敛。
- **帧率不足**：减小 `roi_margin_px`、降 `sgm_paths` 8→4、缩 `census_window` 7→5；
  多目标并发时考虑引入 worker 池（当前同步实现，Query 串行）。
- **加多目标并行**：`SgmMatcher` 非线程安全（内部复用缓冲），并行须每 worker 独占
  一个 matcher 实例 + 独立 ROI 缓冲；`Process` 与 `Query` 若跨线程需对 rect 缓冲加锁。
- **修改关联**：`DistanceResult`/`QueryRect` 字段改动需同步适配器与本文；标定更新后
  确认 `source_width/height` 与标定档位一致；接入主项目时配置开关
  `runtime.stereo_distance_source=box_match|dense_roi`（设计 §11，本轮未实现适配器）。
