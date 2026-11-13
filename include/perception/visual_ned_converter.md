# 视觉观测 → 局部 NED 转换（visual_ned_converter）

> 实现：`include/perception/visual_ned_converter.h` + `src/perception/visual_ned_converter.cpp`
> 设计依据：`docs/物理追踪思路.md` §3.1/§6.3、`docs/archive/早期设计/感知与避障设计.md` §单目标线性卡尔曼
> 消费者：`TargetEstimator` 第二阶段（视觉融合，见 `include/perception/目标估计器.md`）

## 功能职责

把双目测距给出的目标位置（**基准相机系**，米）转换为局部 NED 坐标，
供线性卡尔曼作为位置观测。纯函数：零分配、无状态、不依赖 control 库；
任一输入非有限返回 `std::nullopt`。

## 转换链

```text
相机系（x 右 / y 下 / z 前，来自双目测距校正域，StereoRangeResult.x/y/z_m）
  → ① 固定轴置换到机体 FRD：body = (cam_z, cam_x, cam_y)
  → ② 安装外参：机体系 RPY 旋转（航空 ZYX 顺序）+ 平移（杠杆臂）
  → ③ 机体姿态 RPY 旋转（同一 ZYX 约定，与 PX4 一致）→ NED 方向
  → ④ 加自身局部位置（PX4 LOCAL_POSITION_NED）
```

旋转矩阵约定：`R = Rz(yaw)·Ry(pitch)·Rx(roll)`；roll=pitch=0 时退化为
`control/frame_transform.h` 的 `RotateBodyToNed`（yaw-only），单测中逐数值
交叉验证两套约定一致。

## 关键假设（未验证前不得据此开放控制）

1. **安装外参默认全 0 = 准直安装假设（未标定）**：1° 安装误差在 20m 处引入
   约 0.35m 系统偏差。标定流程为后续任务，标定后改 `target_estimator.visual.mount_*`。
2. **原点假设**：PX4 `LOCAL_POSITION_NED` 的 EKF origin 与 `HOME_POSITION` 同点
   （通常成立但不保证；EKF origin 可在 GPS 切源时静默重置）。室外验证方法见
   `docs/故障排查手册.md`（悬停时对比 GLOBAL_POSITION_INT 转 Home-NED 与
   LOCAL_POSITION_NED）。
3. **时间对齐 v1**：使用最新 PX4 快照（ATTITUDE/LOCAL_POSITION_NED 约 10Hz），
   超龄门禁由 TargetEstimator 执行（`flight_state_max_age_ms`，默认 150ms）。
   最坏 100ms 姿态陈旧：90°/s 偏航角速度下 20m 目标横向误差约 3.1m。
   最近邻快照缓冲/插值为后续优化项。

## 接口

```cpp
struct VisualMountExtrinsics { roll_deg, pitch_deg, yaw_deg, x_m, y_m, z_m };

std::optional<LocalNedCoordinate> VisualCameraToLocalNed(
    double cam_x_m, double cam_y_m, double cam_z_m,   // 相机系，米
    const VisualMountExtrinsics& mount,
    double roll_rad, double pitch_rad, double yaw_rad, // PX4 姿态
    const LocalNedCoordinate& self_position);          // 自身 NED（米）
```

## 日志行为

无日志（纯函数）；拒绝由调用方（TargetEstimator）计数并节流 WARN。

## 测试方式

```bash
./build/visual_ned_converter_test
```

覆盖：轴置换（零姿态恒等）、yaw=90° 前方→+East、pitch=+90° 前方→−Down（仰头）、
roll=+90° 相机右向→+Down（右翼下沉）、mount_yaw 偏置、杠杆臂平移、自身位置叠加、
非有限输入拒绝、与 `RotateBodyToNed` 的约定交叉验证。

## 排查要点

- 视觉观测方向系统性偏转固定角度 → 先查 mount 配置是否还是全 0 假设；
- 目标 NED 整体平移错误 → 查自身 `local_position_valid` 与原点假设
  （EKF origin 是否被重置）；
- 单位/坐标系混淆（度 vs 弧度、相机系 vs 机体系）→ 单测 9 条是权威参照。
