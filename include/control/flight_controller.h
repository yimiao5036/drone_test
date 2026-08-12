/**
 * @file flight_controller.h
 * @brief 飞行控制器部件接口（IFlightController）
 *
 * 属于 drone/control 模块。职责：消费控制意图（ControlIntent），
 * 将其转换为 PX4 设定值（Px4Setpoint）发布，供 PX4 通信线程发送。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，控制算法（位置/速度/姿态设定值生成）在实现期接入。
 * - FlightControllerStub 为骨架占位实现：生命周期可运行，业务方法
 *   记录"未实现"节流日志并返回默认值。
 *
 * 数据流：
 *   common::Topic<ControlIntent>（状态机）──► IFlightController ──► common::Topic<Px4Setpoint> ──► IPx4Link
 * 可替换边界：控制算法。
 */
#pragma once

#include <cstdint>

#include "common/topic.h"
#include "common/types.h"

namespace drone::control {

/// 飞行控制器部件抽象接口。
class IFlightController {
public:
    virtual ~IFlightController() = default;

    // ---- 生命周期 ----
    /// 启动控制（启动控制周期线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止控制；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;

    // ---- 输入 ----
    /// 绑定控制意图输入主题（IMissionStateMachine::IntentOutput()）。
    virtual void SetInput(common::Topic<common::ControlIntent>& intent) = 0;

    // ---- 输出 ----
    /// PX4 设定值输出主题：common::Px4Setpoint（供 IPx4Link 消费）。
    virtual common::Topic<common::Px4Setpoint>& SetpointOutput() = 0;

    // ---- 状态查询 ----
    /// 累计生成设定值次数。
    virtual uint64_t SetpointCount() const = 0;
    /// 累计错误次数。
    virtual uint64_t ErrorCount() const = 0;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class FlightControllerStub final : public IFlightController {
public:
    FlightControllerStub();
    ~FlightControllerStub() override;

    FlightControllerStub(const FlightControllerStub&) = delete;
    FlightControllerStub& operator=(const FlightControllerStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInput(common::Topic<common::ControlIntent>& intent) override;

    common::Topic<common::Px4Setpoint>& SetpointOutput() override;

    uint64_t SetpointCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t setpoint_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::Px4Setpoint> setpoint_output_;
};

}  // namespace drone::control
