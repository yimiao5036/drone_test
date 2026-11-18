#pragma once

// =============================================================================
// visual_ned_converter.h —— 双目视觉观测（相机系）→ 局部 NED 纯函数转换
//
// 对应设计：docs/物理追踪思路.md §3.1/§6.3
//   双目位置观测必须先结合相机内外参和对应时刻 PX4 姿态转换到局部 NED，
//   再更新正式单目标线性卡尔曼。
//
// 转换链（全部输入为同一观测时刻的快照）：
//   相机系（x 右 / y 下 / z 前，来自双目测距校正域）
//     → 固定轴置换到机体 FRD（body = (cam_z, cam_x, cam_y)）
//     → 相机安装外参（RPY 旋转 + 平移，机体系表达）
//     → 机体姿态 RPY（航空 ZYX 顺序：R = Rz(yaw)·Ry(pitch)·Rx(roll)）
//     → 局部 NED 方向
//     → 加自身局部位置（PX4 LOCAL_POSITION_NED）
//
// 约定与边界：
//   - 安装外参默认值全 0 = **准直安装假设（未标定）**；1° 安装误差在 20m 处
//     引入约 0.35m 系统偏差，标定前不得据此开放控制；
//   - 自身位置原点假设：PX4 LOCAL_POSITION_NED 的 EKF origin 与 HOME_POSITION
//     同点（待室外验证项，见 docs/故障排查手册.md）；
//   - 纯函数：零分配、无状态、不依赖 control 库；非有限输入返回 nullopt。
// =============================================================================

#include <optional>

#include "perception/geodetic_converter.h"  // LocalNedCoordinate

namespace drone::perception {

/// 相机→机体安装外参。旋转为机体系下的 RPY（度，航空 ZYX 顺序）；
/// 平移为相机原点相对机体原点的位置（机体系前/右/下，米）。
/// 默认全 0 = 准直安装假设（未标定）。
struct VisualMountExtrinsics {
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    double x_m = 0.0;
    double y_m = 0.0;
    double z_m = 0.0;
};

/// 双目观测（基准相机系，米）→ 局部 NED（米）。
/// attitude 为机体欧拉角（弧度，FRD/NED 约定，与 PX4 一致）；
/// self_position 为同一时刻自身局部 NED 位置（PX4 LOCAL_POSITION_NED）。
/// 任一输入非有限时返回 nullopt。
std::optional<LocalNedCoordinate> VisualCameraToLocalNed(
    double cam_x_m, double cam_y_m, double cam_z_m,
    const VisualMountExtrinsics& mount, double roll_rad, double pitch_rad,
    double yaw_rad, const LocalNedCoordinate& self_position);

}  // namespace drone::perception
