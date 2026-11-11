#pragma once

// =============================================================================
// visual_control_law.h —— 视觉控制段抽象接口（IVisualControlLaw）
//
// 背景（2026-11-11）：捕网、干扰、撞击三类飞机的“接近段”共用同一套制导律，
// 但 S7 视觉伺服之后的纵向行为不同：
//   - 捕网/干扰：到保持距离后停住/伴飞，视觉只负责瞄准与保持；
//   - 撞击：不保持距离，全速穿越，视觉只负责末端修正。
// 因此把 S7 视觉伺服的控制律抽成接口，由装配层按“拦截剖面”选择实现；
// 控制回路不查询飞机身份（身份只用于寻址），避免安全关键行为被身份配置
// 静默改变。
//
// 当前实现状态：
//   - kNetCapture / kJammer：共用 TrackingControlLaw（参数各自来自本机配置）；
//   - kImpact：预留，工厂抛异常（误配撞击机时启动期 fail-fast，不静默回退）。
//
// 边界：本接口只覆盖“给定视觉/距离/姿态快照 → 限幅后的控制输出”；S6/S7
// 进入/退出编排、S9/S10 拦截动作仍由 MissionStateMachine 负责，不在本接口内。
// =============================================================================

#include <cstdint>
#include <memory>

#include "control/tracking_config.h"
#include "control/tracking_types.h"

namespace drone::control {

/// 拦截剖面：决定视觉控制律实现与参数口径。
/// 由本机功能类别（MAVLink component ID）在装配期映射，见 DroneApplication。
enum class VisualProfile : std::uint8_t {
    kNetCapture = 0,  ///< 捕网：保持距离 + 瞄准（当前唯一生产实现）
    kJammer = 1,      ///< 干扰：与捕网共用控制律（保持距离/伴飞），参数不同
    kImpact = 2,      ///< 撞击：预留（全速穿越、不保持距离），尚未实现
};

/// 剖面中文名（日志/诊断用，返回静态字符串）。
const char* VisualProfileName(VisualProfile profile);

/// S7 视觉伺服控制律接口。单线程调用（状态机决策线程），实现不得内部起线程。
class IVisualControlLaw {
public:
    virtual ~IVisualControlLaw() = default;

    /// 单周期控制计算，语义与 TrackingControlLaw::Update 一致：
    /// 超龄判定 → 判丢刹车 → 距离通道 → 机体系→NED 旋转输出。
    virtual ControlOutput Update(const ControlSnapshot& snapshot,
                                 std::int64_t now_ms) = 0;

    /// 停止视觉闭环时清全部状态（积分、限幅器历史、节拍时钟）。
    virtual void Reset() = 0;

    /// 本实例对应的剖面（日志/状态回传；不得用于控制分支）。
    virtual VisualProfile Profile() const = 0;
};

/// 按剖面创建视觉控制律。
/// @param profile 拦截剖面（装配期由飞机功能类别映射）
/// @param config  本机 visual_tracking 配置（参数按剖面各自配置）
/// @throws std::invalid_argument kImpact 预留未实现，或剖面非法
std::unique_ptr<IVisualControlLaw> CreateVisualControlLaw(
    VisualProfile profile, const TrackingConfig& config);

}  // namespace drone::control
