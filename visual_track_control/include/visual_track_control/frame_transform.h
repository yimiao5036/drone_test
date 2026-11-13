#pragma once

// =============================================================================
// frame_transform.h —— 机体系（前/右/下）→ 局部 NED（北/东/地）坐标旋转
//
// 对应规格：docs/物理追踪思路.md
//   §2.3 控制律内部计算统一在机体坐标系；输出的速度意图若采用局部系，
//        必须随姿态完成旋转并显式标注坐标系，禁止混用未标注坐标系的数据。
//
// 实现要点：
//   - 速度设定值只随**航向角 ψ**（绕下轴旋转）旋转：水平速度分量的
//     姿态耦合项由飞控内环处理，这是 PX4 offboard 速度设定值的标准做法；
//     滚转/俯仰不参与（对速度指令为二阶小量）；
//   - 旋转为正交变换：合速度大小不变，因此"限幅在机体系完成、旋转在
//     输出端执行"不破坏 §8 限幅约束；
//   - 无状态纯函数（零分配、无虚函数、无锁），可独立单元测试。
//
// 正方向验证（NED：x=北，y=东，z=地；航向 ψ 自北顺时针为正）：
//   - 机体前向 (1,0) 在 ψ=90°（机头朝东）时 → (0, 1) = 东 ✓
//   - 机体右向 (0,1) 在 ψ=0°（机头朝北）时 → (0, 1) = 东 ✓
// =============================================================================

#include <cmath>

namespace drone::vtc {

/// 机体系速度（前/右/下）→ 局部 NED（北/东/地），绕下轴按航向角 ψ 旋转：
///   v_north =  v_forward·cosψ − v_right·sinψ
///   v_east  =  v_forward·sinψ + v_right·cosψ
///   v_down  =  v_down（不变）
/// 参数以引用传入并就地写出旋转结果。
inline void RotateBodyToNed(double& v_forward, double& v_right, double& v_down,
                            double yaw_rad) {
    (void)v_down;  // 垂直分量不受绕下轴旋转影响；保留参数以保持接口对称
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    const double north = v_forward * c - v_right * s;
    const double east = v_forward * s + v_right * c;
    v_forward = north;
    v_right = east;
    // v_down 不变
}

}  // namespace drone::vtc
