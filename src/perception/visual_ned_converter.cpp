// =============================================================================
// visual_ned_converter.cpp —— 双目视觉观测 → 局部 NED 转换实现
//
// 旋转约定（与 PX4 一致的航空 NED/FRD）：
//   R(roll,pitch,yaw) = Rz(yaw)·Ry(pitch)·Rx(roll)
//   Rx(φ) = [[1,0,0],[0,cφ,-sφ],[0,sφ,cφ]]
//   Ry(θ) = [[cθ,0,sθ],[0,1,0],[-sθ,0,cθ]]
//   Rz(ψ) = [[cψ,-sψ,0],[sψ,cψ,0],[0,0,1]]
// roll=pitch=0 时退化为 frame_transform.h 的 RotateBodyToNed（yaw-only），
// 单测中交叉验证。
// =============================================================================

#include "perception/visual_ned_converter.h"

#include <cmath>

namespace drone::perception {
namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

bool AllFinite(double a, double b, double c) {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

/// 3×3 矩阵 × 3 向量（行主序）。
void Apply(const double m[9], double x, double y, double z, double& out_x,
           double& out_y, double& out_z) {
    out_x = m[0] * x + m[1] * y + m[2] * z;
    out_y = m[3] * x + m[4] * y + m[5] * z;
    out_z = m[6] * x + m[7] * y + m[8] * z;
}

/// 航空 RPY（ZYX）旋转矩阵，角度弧度。
void RotationRpy(double roll, double pitch, double yaw, double out[9]) {
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    // R = Rz(yaw)·Ry(pitch)·Rx(roll)
    out[0] = cy * cp;
    out[1] = cy * sp * sr - sy * cr;
    out[2] = cy * sp * cr + sy * sr;
    out[3] = sy * cp;
    out[4] = sy * sp * sr + cy * cr;
    out[5] = sy * sp * cr - cy * sr;
    out[6] = -sp;
    out[7] = cp * sr;
    out[8] = cp * cr;
}

}  // namespace

std::optional<LocalNedCoordinate> VisualCameraToLocalNed(
    double cam_x_m, double cam_y_m, double cam_z_m,
    const VisualMountExtrinsics& mount, double roll_rad, double pitch_rad,
    double yaw_rad, const LocalNedCoordinate& self_position) {
    if (!AllFinite(cam_x_m, cam_y_m, cam_z_m) ||
        !AllFinite(roll_rad, pitch_rad, yaw_rad) ||
        !AllFinite(self_position.north_m, self_position.east_m,
                   self_position.down_m) ||
        !AllFinite(mount.roll_deg, mount.pitch_deg, mount.yaw_deg) ||
        !AllFinite(mount.x_m, mount.y_m, mount.z_m)) {
        return std::nullopt;
    }

    // ① 相机系（右/下/前）→ 机体 FRD（前/右/下）固定轴置换
    double bx = cam_z_m;
    double by = cam_x_m;
    double bz = cam_y_m;

    // ② 安装外参：机体系内旋转 + 平移（杠杆臂）
    double r_mount[9];
    RotationRpy(mount.roll_deg * kDegToRad, mount.pitch_deg * kDegToRad,
                mount.yaw_deg * kDegToRad, r_mount);
    double mx, my, mz;
    Apply(r_mount, bx, by, bz, mx, my, mz);
    bx = mx + mount.x_m;
    by = my + mount.y_m;
    bz = mz + mount.z_m;

    // ③ 机体姿态 → 局部 NED 方向
    double r_body[9];
    RotationRpy(roll_rad, pitch_rad, yaw_rad, r_body);
    double nx, ny, nz;
    Apply(r_body, bx, by, bz, nx, ny, nz);

    // ④ 加自身局部位置
    LocalNedCoordinate result;
    result.north_m = self_position.north_m + nx;
    result.east_m = self_position.east_m + ny;
    result.down_m = self_position.down_m + nz;
    if (!AllFinite(result.north_m, result.east_m, result.down_m)) {
        return std::nullopt;
    }
    return result;
}

}  // namespace drone::perception
