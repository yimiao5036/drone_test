/**
 * @file mission_state_machine.h
 * @brief 任务状态机部件接口（IMissionStateMachine）
 *
 * 属于 drone/state_machine 模块。职责：消费地面站目标、飞行状态与健康快照，
 * 按任务状态机（S0~S16，见 docs/状态机设计.md）决策，输出控制意图
 * （ControlIntent）与任务状态回传（MissionStatus）。
 *
 * 当前正式实现处于安全影子阶段：只验证 GroundStationTarget 能进入任务链路并
 * 发布 MissionStatus，不连接 PX4 控制输出。阶段四起可通过 SetEstimationInputs
 * 绑定估计/视觉/测距输入：绑定后 S5（GPS_APPROACH）发布 kPositionGuide
 * 控制意图（reason_code 带 bit31 控制权请求位）；绑定估计器诊断接口后，
 * 交接判据（S5→S6→S7 进入/退出迟滞，handover 配置节）开始工作；未绑定时
 * 保持原影子行为（停在 S5）。S6 驻留期续发进入时刻冻结的最后位置目标，
 * S7（VISUAL_TRACKING）由内部视觉伺服协作者（复用 TrackingControlLaw）
 * 发布 control_source=1 的视觉意图；视觉状态断流/测距失效/连续丢失等退出
 * 迟滞之外另有热备份年龄核对（节流 WARN，不退出 S7）。
 *
 * 数据流：
 *   common::Topic<GroundStationTarget> ─┐
 *   common::Topic<FlightStateSnapshot> ─┤
 *   common::Topic<HealthStatus> ────────┤
 *   common::Topic<TargetState> ─────────┼──► IMissionStateMachine ──┬─► common::Topic<ControlIntent>
 *   common::Topic<VisualTargetStatus> ──┤   （后三者经              └─► common::Topic<MissionStatus>
 *   common::Topic<StereoTargetDistance> ┘      SetEstimationInputs；
 *   估计器诊断（LastVisualInnovationM）经 SetEstimationDiagnostics）
 * 可替换边界：不依赖具体设备。
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

#include "common/topic.h"
#include "common/types.h"
#include "config/config.h"
#include "control/tracking_config.h"

namespace drone::communication {
class IPx4CommandSender;
}

namespace drone::perception {
class ITargetEstimator;
}

namespace drone::state_machine {

/// MissionStateMachine 配置（阶段四新增）。全部字段可选，缺席回退默认值。
struct MissionStateMachineConfig {
    /// 控制意图有效期（valid_for_ms 与 valid_until_ms 基准）。必须与
    /// FlightController 的 controller.intent_timeout 对齐，由装配层从同一来源传入。
    std::chrono::milliseconds intent_valid_for{300};
    /// 交接缝迟滞参数（handover 配置节同源注入）。关系校验（exit > lock 等）
    /// 由配置加载期 `HandoverConfig::Validate` 强制；本结构不重复校验。
    config::HandoverConfig handover;
    /// 视觉跟踪控制律全量配置（visual_tracking 节整节同源注入）：
    /// `tracking.camera` 供视线角与视场半角换算（S6 进入判据）、
    /// S7 视觉伺服协作者构造 TrackingControlLaw（阈值与增益同源）。
    /// 分辨率与内参绑定，换档位必须重新标定（见 control/tracking_config.h）。
    control::TrackingConfig tracking;
    /// 视觉稳定性状态超龄阈值（visual_tracking.control.visual_stale_ms 同源）。
    /// VisualTargetStatus 头不设置 valid_for_ms，新鲜度按 receive_time + 该阈值判定。
    std::chrono::milliseconds visual_stale{200};
    /// 双目测距超龄阈值（visual_tracking.control.distance_stale_ms 同源）。
    std::chrono::milliseconds distance_stale{500};
};

/// 任务状态机部件抽象接口。
class IMissionStateMachine {
public:
    virtual ~IMissionStateMachine() = default;

    // ---- 生命周期 ----
    /// 启动状态机（启动决策线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止状态机；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;

    // ---- 输入 ----
    /// 绑定地面站目标、飞行状态与健康快照输入主题。
    virtual void SetInputs(common::Topic<common::GroundStationTarget>& ground_target,
                           common::Topic<common::FlightStateSnapshot>& flight,
                           common::Topic<common::HealthStatus>& health) = 0;
    /// 绑定估计/视觉/测距输入（阶段四正式化；未绑定时 S5 不出意图、保持现有影子行为）。
    /// 三个主题必须作为一组同时绑定；必须在 Start 前调用。
    virtual void SetEstimationInputs(
        common::Topic<common::TargetState>& estimated,
        common::Topic<common::VisualTargetStatus>& visual_status,
        common::Topic<common::StereoTargetDistance>& distance) = 0;
    /// 注入 PX4 命令窄接口（仅 enable_control=true 装配期调用）。
    virtual void SetCommandSender(communication::IPx4CommandSender* sender) = 0;
    /// 注入目标估计器诊断接口：交接判据的创新门限读取
    /// `ITargetEstimator::LastVisualInnovationM()`（最近一次视觉观测与卡尔曼预测的
    /// 水平距离）。必须在 Start 前调用；未注入时交接判据不评估（保持 S5 影子行为，
    /// 创新门限无法核验时不放行交接）。
    virtual void SetEstimationDiagnostics(
        const perception::ITargetEstimator* estimator) = 0;

    // ---- 输出 ----
    /// 控制意图输出主题：common::ControlIntent（供 IFlightController 消费）。
    virtual common::Topic<common::ControlIntent>& IntentOutput() = 0;
    /// 任务状态回传输出主题：common::MissionStatus（供地面站回传）。
    virtual common::Topic<common::MissionStatus>& StatusOutput() = 0;

    // ---- 状态查询 ----
    /// 当前任务状态（S0~S16，见 common::MissionState）。
    virtual common::MissionState CurrentState() const = 0;
    /// 累计状态转移次数。
    virtual uint64_t TransitionCount() const = 0;
    /// 累计错误次数。
    virtual uint64_t ErrorCount() const = 0;
};

/// 正式任务状态机实现。
///
/// 默认保持安全影子语义；调用 SetEstimationInputs 绑定估计/视觉/测距输入后，
/// S5（GPS_APPROACH）开始发布 kPositionGuide 控制意图（带 bit31 控制权请求位）；
/// 再经 SetEstimationDiagnostics 注入估计器诊断后，交接判据
/// （handover 配置节：LOCKED 连续帧/测距/视场余量/创新门限 + S6 超时 + S7 退出迟滞）
/// 生效。S6 驻留期续发冻结的最后位置目标（设计 §2.1），S7 由内部视觉伺服
/// 协作者（TrackingControlLaw）发布 control_source=1 的速度航向意图
/// （输出已是局部 NED）。是否真正输出控制由 FlightController 授权链与
/// enable_control 装配门禁决定。
class MissionStateMachine final : public IMissionStateMachine {
public:
    explicit MissionStateMachine(MissionStateMachineConfig config = {});
    ~MissionStateMachine() override;

    MissionStateMachine(const MissionStateMachine&) = delete;
    MissionStateMachine& operator=(const MissionStateMachine&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInputs(common::Topic<common::GroundStationTarget>& ground_target,
                   common::Topic<common::FlightStateSnapshot>& flight,
                   common::Topic<common::HealthStatus>& health) override;
    void SetEstimationInputs(
        common::Topic<common::TargetState>& estimated,
        common::Topic<common::VisualTargetStatus>& visual_status,
        common::Topic<common::StereoTargetDistance>& distance) override;
    void SetCommandSender(communication::IPx4CommandSender* sender) override;
    void SetEstimationDiagnostics(
        const perception::ITargetEstimator* estimator) override;

    common::Topic<common::ControlIntent>& IntentOutput() override;
    common::Topic<common::MissionStatus>& StatusOutput() override;

    common::MissionState CurrentState() const override;
    uint64_t TransitionCount() const override;
    uint64_t ErrorCount() const override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

/// 骨架占位实现：生命周期完整，业务方法返回默认值。
class MissionStateMachineStub final : public IMissionStateMachine {
public:
    MissionStateMachineStub();
    ~MissionStateMachineStub() override;

    MissionStateMachineStub(const MissionStateMachineStub&) = delete;
    MissionStateMachineStub& operator=(const MissionStateMachineStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInputs(common::Topic<common::GroundStationTarget>& ground_target,
                   common::Topic<common::FlightStateSnapshot>& flight,
                   common::Topic<common::HealthStatus>& health) override;
    void SetEstimationInputs(
        common::Topic<common::TargetState>& estimated,
        common::Topic<common::VisualTargetStatus>& visual_status,
        common::Topic<common::StereoTargetDistance>& distance) override;
    void SetCommandSender(communication::IPx4CommandSender* sender) override;
    void SetEstimationDiagnostics(
        const perception::ITargetEstimator* estimator) override;

    common::Topic<common::ControlIntent>& IntentOutput() override;
    common::Topic<common::MissionStatus>& StatusOutput() override;

    common::MissionState CurrentState() const override;
    uint64_t TransitionCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    common::MissionState state_ = common::MissionState::kBoot;
    uint64_t transition_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::ControlIntent> intent_output_;
    common::Topic<common::MissionStatus> status_output_;
};

}  // namespace drone::state_machine
