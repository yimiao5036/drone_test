#pragma once

// =============================================================================
// line_of_sight.h —— 像素偏移 → 视线角（针孔模型）
//
// 对应规格：docs/物理追踪思路.md
//   §4.1 水平视线角 β：
//        针孔模型 β = atan( x_offset · 2·tan(θ_h/2) / W )
//        （线性近似 β ≈ x_offset · ( θ_h / W ) 仅在 |x_offset| 很小时成立）
//   §5.1 垂直视线角 α 同理（θ_v、H）
//
// 实现要点：
//   - 全程使用精确针孔模型：规格中的线性近似与针孔式在切换边界两侧
//     函数值不连续（θ=60°、边界比例 0.2 处约 8% 角度台阶），会向控制
//     环路注入突变；atan 计算代价可忽略，故直接全程采用精确式，
//     linear_ratio 配置项随之移除（§8 待定项收敛）；
//   - 构造期预计算针孔系数，热路径仅一次乘加 + atan
//     （零分配、无虚函数、无锁）。
// =============================================================================

namespace drone::vtc {

/// 像素偏移 → 视线角（弧度）转换器；水平/垂直各构造一个实例
class PixelToAngle {
public:
    /// size_px：画面尺寸（W 或 H，像素，必须 > 0）；
    /// fov_rad：视场角（θ_h 或 θ_v，弧度，必须 ∈ (0, π)）。
    /// 非法参数抛 std::invalid_argument（构造期校验）。
    PixelToAngle(double size_px, double fov_rad);

    /// 偏移（像素，水平右正 / 垂直下正）→ 视线角（弧度，同向为正）
    double Convert(double offset_px) const;

private:
    double pinhole_scale_;    // 2·tan(θ/2) / size：针孔模型系数
};

}  // namespace drone::vtc
