# 双目测距 v2（稀疏 Census 路线）设计（设计冻结）

> 日期：2026-10-10　状态：已获用户确认
> 参考实现（已验证）：`tools/camera_test_ok/distance_app.py`、`tools/camera_test_ok/stereo_core.py`
> 移植规格：`tools/camera_test_ok/C++移植指南.md`（下称"指南"）
> 取代：`2026-09-23-双目测距v1.0-design.md`（v1.0 整链失效，根因见 §1，相关实现原地重写）

## 1. 背景与 v1.0 失效根因

v1.0（09-23 设计、09-24 影子接入、待板上验证）在实际运行中不可用。代码核对定案：

| 事实 | 出处 |
|---|---|
| 拆分器：**左目 Topic = SBS 右半幅**（物理左目），右目 Topic = SBS 左半幅 | `src/video/stereo_frame_splitter.cpp:224-229`（09-22 实证交换） |
| 标定文件 `left`/`right` 指 **SBS 半幅**（left=SBS 左半=物理右目），不是物理目 | `tools/camera_calibration/capture_stereo_charuco.py:193-194`；`tools/camera_test_ok/calib_charuco.json` note |
| v1.0 把标定 `left` 内参（cx=620.73）用在了左目 Topic（真实 cx=674.76）上，右目同理反置 | `2026-09-23-双目测距v1.0-design.md:118-119`、`config/config.json` |

两目内参互换使归一化视差恒多出 +(674.76−620.73)/1006.59×2 ≈ **+0.107**：
`Z = B/D_norm` 恒在 0.36~0.56 m，永远落在 `distance_min_m=1.0` 之外 → **valid 恒 false**，整链失效。

参考实现（2026-10-09，Windows Python 实测定论）自带同源坑的实测定案：

- OpenCV 约定 `T` 是 cam1 原点在 cam2 系中的位置；`T_m[0]=+0.0601>0` ⇒ cam1（SBS 左半）物理上是**右**目；
- 基准图必须取 **SBS 右半**（= 物理左目 = 主工程左目 Topic）。以左半为基准时视差为负，匹配退化：
  有效视差 6.9%、IQR 106.5px；换成右半为基准后 62.1%、IQR 4.1px（同一帧实测）。
- 视差 `d = u_ref − u_aux > 0`，`Z = fxB_m / d`，`fxB_m = fx_rect × |T| = 63.57`（**不是** fx×T.x，两者差 0.057%）。

本机（cv2 5.0.0）用 `calib_charuco.json` 复算校正矩阵，与指南 §3.2 常数逐项一致：
`fx_rect=1057.33492`、主点 (692.7685, 529.0832)、`B=0.060120`、`fxB=63.57`、`Tb.x=−0.060090`。

## 2. 范围

- **原地重写** `stereo_ranger`：`StereoRangerCore` 换为稀疏 Census 引擎；组件壳输入改为
  「左目检测框 + 左右目灰度帧」。
- **删除 `yolo_right` 右目检测链路**（路线 B 只在基准图检测，右图仅作匹配灰度；省约一半 NPU）。
- **零新依赖**：不引入 OpenCV。校正矩阵由 Python 脚本一次性导出为 JSON 预置入库，
  C++ 自研映射表生成与 remap。
- 仍然**影子**：`runtime.enable_stereo_ranging` 默认 false；不接控制输出；不注册健康源（沿用 v1.0 决定）。
- 不做（指南 §9.1 明列未做项）：饱和点排除（只做截断告警）、多目标跟踪、NED 融合。

## 3. 数据流与接口

```
Topic<DetectionResult> kDetection（左目框，原始左图坐标） ─┐
Topic<FrameHandle> kDecodedFrameLeft （基准图=SBS右半,NV12）─┼─► StereoRanger（消费线程）
Topic<FrameHandle> kDecodedFrameRight（辅助图=SBS左半,NV12）─┘        │
   帧环形缓冲（按 sequence）；左目检测按 frame_sequence 成批            │
   批次闭合（新 sequence 到达，或 settle 超时）→ 取对应左右帧 → core    ▼
                                        Topic<StereoTargetDistance> kStereoTargetDistance
                                                       │
                                                       ▼
                                  VisualTrackingShadow::SetDistanceInput（可选，语义不变）
```

字段映射（`StereoTargetDistance`，类型不变）：

| 字段 | v2 语义 |
|---|---|
| `frame_sequence` | 左目（基准）帧序号（取自检测批次的 frame_sequence） |
| `forward_distance_m` | `z`（EMA 平滑后，米）；不可用 NaN |
| `slant_range_m` | `euclid`（按平滑 z 重算）；不可用 NaN |
| `disparity_px` | 近端簇中位视差 `d_med`（px，基准图坐标系）；不可用 NaN |
| `confidence` | 左目检测置信度 |
| `valid` | 聚合成功（近端簇点数 ≥ min_points 且 bbox 有效） |
| `center_pixel_x_mid` | 腐蚀后 bbox 中心 x（基准图像素系） |
| `bbox_x/y/w/h` | 左目检测框（原始左图坐标，未腐蚀） |

## 4. 算法语义（照抄参考实现，逐条可核对）

### 4.1 校正

以 cam2（SBS 右半）为基准：`R_ref=Rᵀ, T_ref=−RᵀT`（cam2→cam1），
`stereoRectify(K_ref=K2, D_ref=D2, K_aux=K1, D_aux=D1, (1280,720), R_ref, T_ref, alpha=0, CALIB_ZERO_DISPARITY)`
→ `R_r,R_a,P_r,P_a`。映射表按 `initUndistortRectifyMap` 等价实现（rect→source，float32，双线性）：

```
对每个 rect 像素 (u,v)：
  x=(u−cx_r)/fx_r, y=(v−cy_r)/fy_r            # P_r 内参
  [X,Y,Z]=R_rᵀ·[x,y,1]；x'=X/Z, y'=Y/Z
  x''=x'(1+k1r²+k2r⁴+k3r⁶)+2p1x'y'+p2(r²+2x'²)   # 正向畸变模型(K_ref,D_ref)
  u_src=fx_ref·x''+cx_ref, v_src=fy_ref·y''+cy_ref
```

运行时只做 bilinear 采样（读源 Y 平面，带 stride）；`B=|P_a(0,3)/P_a(0,0)|`；
`fx_rect=P_r(0,0)`；`fxB_m=fx_rect·B`（断言与预置 JSON 记录值一致）。

### 4.2 source→rect 框映射

检测框在**原始左图**坐标，聚合与匹配在**校正图**坐标，需逐框四角映射：
`K_ref⁻¹` 归一化 → 畸变逆解（定点迭代 10 次，同 OpenCV undistortPoints）→ `R_r` → `P_r` → 取包围盒（向外取整）。

### 4.3 Census（7×7，48 bit → uint64）

以中心像素为阈值，邻域 `>=` 判据二值化（参考实现 `CMP_GE`）；跳过中心共 48 邻位。
位序自定（匹配只用自身编码），但**左右图与所有帧必须一致**。边界 replicate。

### 4.4 代价引擎

参考图（基准）各点窗口预提取一次（(N,49) uint64）；候选视差只需**列平移**：
`x_R = x_L − d`，行索引不变；越界**按行列分别 clip**（线性索引 clip 会跨行回绕）。
代价 = 49 邻位汉明距离之和（XOR + 字节查表 popcount）。

### 4.5 匹配主流程

1. 粗搜索：`d ∈ [disp_min, disp_max]`，步长 `coarse_stride=2`；候选不足 3 个时取 9 点等分；
2. 精搜索：粗最优 ±`fine_halfwidth=6`，step=1（候选 clip 到 [disp_min,disp_max]）；
3. 抛物线亚像素：`off = 0.5(c₋₁−c₊₁)/(c₋₁−2c₀+c₊₁)`，clip ±1；`disp = d_fine + off`，再 clip 到区间；
4. 唯一性：峰位 **±2 之外**（在精搜索窗内）的次优代价 / 最优代价 ≥ `uniqueness_ratio=1.05`；
5. 左右一致性：以右图点 `x_R = x_L − round(disp)` 为参考反向搜 `x_L' = x_R + d'`（**加号**），
   `d' ∈ disp ± (ceil(lr_tol)+2)`，`|disp − d'_best| ≤ lr_tol=1.0`；
6. `valid = 有限 && 唯一性 && LR`。

### 4.6 关键点

Shi-Tomasi（等价 `goodFeaturesToTrack`：blockSize=5、qualityLevel=0.01、minDistance=8、
maxCorners=max(80×目标数,16)、mask=所有框并集）；再按 **4×4 网格轮转**取点
（按质量序入桶，逐轮每桶取一个），每目标 ≤ `max_points_per_target=80`。
> 与 OpenCV 非逐位一致属已知差异：关键点选择不影响深度正确性（见 §7）。

### 4.7 聚合（bbox 近端簇）

```
腐蚀 bbox 5px（向左上 +erode、右 -erode，clip 到画面）→ 宽高 <8 返回不可用
取落在框内的有效匹配点；点数 < min_points(30) 返回不可用
d_near = percentile(disps, 90)          # numpy 线性插值语义
cluster = {d ≥ d_near − 3.0}；若簇内 <30 退回全体
d_med = median(cluster)
z = fxB_m / d_med
X = (u−cx_r)·z/fx_r, Y = (v−cy_r)·z/fy_r（u,v=腐蚀后 bbox 中心）
euclid = √(X²+Y²+z²)；IQR = p75−p25（簇内，置信度指标）
```

### 4.8 平滑与输出

EMA `z_f = α·z + (1−α)·z_f`，α=`smooth_alpha`(0.4)，键 `(class_id, x1//40, y1//40)`（x1,y1 取原始框）；
X/Y/euclid 按平滑后 z 重算（`scale = z_f/z_raw`）。α=0 关闭。

### 4.9 截断告警

`d_med ≥ disp_max − 2` 时置截断标志（日志/计数，节流），提示距离可能更近。

## 5. 配置与预置数据

**新增** `config/stereo_rectify_uvc_1280x720.json`（预置校正矩阵，入库）：
由 `tools/camera_calibration/export_rectify.py` 从标定 JSON 生成，含
`K_ref/D_ref/R_ref/P_ref/K_aux/D_aux/R_aux/P_aux/Q`、`baseline_m`、`fx_rect_px`、`fxb_m_px`、
来源标定文件与 `calibrated_at`。重新标定后必须重跑该脚本。

**`config/config.json` 的 `stereo_ranger` 段（v2 重写）**：

```json
{
  "_comment": "双目测距 v2（稀疏 Census，基准图=SBS右半=左目Topic）。",
  "rectify_matrices_path": "config/stereo_rectify_uvc_1280x720.json",
  "disp_min_px": 8.0, "disp_max_px": 160.0,
  "coarse_stride": 2, "fine_halfwidth": 6, "uniqueness_ratio": 1.05, "lr_tol_px": 1.0,
  "census_win": 7, "max_points_per_target": 80, "grid_cells": 4,
  "warmup_frames": 15, "black_frame_mean_threshold": 2.0,
  "bbox_erode_px": 5, "min_points": 30, "near_pct": 90.0, "near_tol_px": 3.0,
  "smooth_alpha": 0.4,
  "input_queue_capacity": 2, "frame_buffer_depth": 24,
  "pair_window_ms": 20, "settle_ms": 50, "summary_log_interval_ms": 10000
}
```

**删除** `yolo_right` 段与 `kDetectionRight` Topic（含 config 解析、装配、文档引用）。

## 6. 测试与验收

- 单元测试（开发机 WSL2）：映射表黄金采样点（对拍本机 OpenCV）、Census/代价/亚像素/唯一性/LR
  语义、合成视差场匹配复原（给定关键点，与 Python 黄金值对拍）、百分位/中位数语义、
  近端簇聚合各分支、截断标志、source→rect 框映射。
- 组件测试：帧缓冲与检测批次配对（乱序/缺帧/超龄）、发布语义（每框一条）、Stop 幂等。
- **板上验收（用户执行）**：按指南 §9 —— 卷尺 0.5/1/1.5/2.0m（非周期纹理平面，±2%/±5%）、
  单调性、极线残差、近端截断告警、连续 5 分钟 ≥30fps。
- **遗留必补**：`fxB` 绝对精度尚未与物理卷尺对齐（指南 §9.1），板上必须先做卷尺对齐
  （若整体偏固定比例 → 卡尺量基线并回填 `baseline_m`）。

## 7. 风险与已知差异

1. **Shi-Tomasi 非逐位一致**：角点集合可能与 OpenCV 有差异 → 匹配点数/IQR 略有不同，
   但不影响深度换算正确性；匹配与聚合用「给定关键点」的黄金对拍隔离验证。
2. **饱和错配**（黑色细结构目标钉在视差上限，IQR=0）：指南明列"未做"，v2 同样只做告警。
3. **检测器**：Python 侧 YOLO 在实际场景不可用（conf 0.026），测距以固定框验证；
   主工程 ReLU6 模型已实机验证可用，检测由现有链路承担。
4. **性能**：C++ 侧 census/代价为位运算，预估远快于 Python（118ms→<10ms 量级）；
   板上实测回填。
