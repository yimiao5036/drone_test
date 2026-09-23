#pragma once

// =============================================================================
// line_of_sight.h —— 像素坐标 → 视线角（针孔模型，内参式）
//
// 对应规格：docs/物理追踪思路.md
//   §4.1 水平视线角 β、§5.1 垂直视线角 α
//
// 移植改造（2026-09-23，自 visual_track_control/line_of_sight.h）：
//   原实现 PixelToAngle(size_px, fov_rad) 以 FOV 等效焦距 + 画面中心为偏移
//   基准。2026-09-23 相机标定实测主点偏离画面中心（左目 cx=620.7、
//   cy=532.1 @1280×720，垂直方向偏差约 172px ≈ 9.7°），画面中心假设会
//   引入系统性角度误差。改为内参式：
//       angle = atan( (pixel − c) / f )
//   偏移自主点（cx/cy）量起，f 取标定 fx/fy。
//
// 实现要点：
//   - 畸变不参与本换算（中心区域影响 <0.5°；畸变由双目测距立体校正承担）；
//   - 构造期校验参数，热路径仅一次减法、一次除法 + atan
//     （零分配、无虚函数、无锁）。
// =============================================================================

namespace drone::control {

/// 像素坐标 → 视线角（弧度）转换器；水平/垂直各构造一个实例
class PixelToAngle {
public:
    /// f_px：该方向像素焦距（标定 fx 或 fy，必须 > 0）；
    /// c_px：该方向主点坐标（标定 cx 或 cy，必须 >= 0）。
    /// 非法参数抛 std::invalid_argument（构造期校验）。
    PixelToAngle(double f_px, double c_px);

    /// 像素坐标（水平右正 / 垂直下正）→ 视线角（弧度，同向为正）
    double Convert(double pixel_px) const;

private:
    double f_px_;  // 像素焦距
    double c_px_;  // 主点坐标
};

}  // namespace drone::control
