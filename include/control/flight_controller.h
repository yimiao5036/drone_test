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
 * - FlightController 为正式实现（阶段四控制接入 Task 3 骨架）：固定周期
 *   线程消费意图 + 门禁链（有限性/超龄/控制权）；控制权默认 IDLE 不发布
 *   设定值，限幅（Task 4）与控制权三态（Task 5）后续接入。实现文档见
 *   同目录 flight_controller.md。
 *
 * 数据流：
 *   common::Topic<ControlIntent>（状态机）──► IFlightController ──► common::Topic<Px4Setpoint> ──► IPx4Link
 * 可替换边界：控制算法。
 */
#pragma once

#include <cstdint>
#include <memory>

#include "common/topic.h"
#include "common/types.h"

namespace drone::config {
struct FlightControllerConfig;
}

namespace drone::communication {
class IPx4Link;
}

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

/// 控制权状态（spec §4；Task 5 接入授权/降级转移，本任务恒为 kIdle）。
enum class ControlAuthority : uint8_t { kIdle, kActive, kCoast };

/// 正式飞行控制器（spec §3 执行层）：固定周期消费 ControlIntent，
/// 门禁链（有限性/超龄/控制权）+ 限幅 + 无扰衔接后发布 Px4Setpoint。
///
/// 骨架边界（本任务）：
/// - 固定周期线程、意图队列排干（只留最新）、入口门禁（有限性/超龄拒绝计数）、
///   IDLE 门禁（默认 kIdle，不发布任何 setpoint）已实现；
/// - 限幅（Task 4）、控制权三态与 PX4 门禁（Task 5）后续接入；
///   SetPx4LinkSource 接口已预留，本任务仅存指针不做判定。
class FlightController final : public IFlightController {
public:
    /// 配置在 LoadAppConfig 加载期已 Validate() 校验（fail-fast）；
    /// 构造函数另做本地防御：period<=0 直接抛 std::invalid_argument。
    explicit FlightController(const config::FlightControllerConfig& config);
    ~FlightController() override;

    FlightController(const FlightController&) = delete;
    FlightController& operator=(const FlightController&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInput(common::Topic<common::ControlIntent>& intent) override;

    common::Topic<common::Px4Setpoint>& SetpointOutput() override;

    uint64_t SetpointCount() const override;
    uint64_t ErrorCount() const override;

    // ---- 诊断（测试与门禁用） ----
    /// 当前控制权状态。
    ControlAuthority Authority() const;
    /// 门禁拒绝（非有限/超龄）意图累计计数。
    uint64_t RejectedIntentCount() const;
    /// 注入 PX4 链路只读快照源（Task 5 门禁启用；本任务仅保存指针）。
    void SetPx4LinkSource(const communication::IPx4Link* link);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;  // PIMPL：线程/订阅等实现细节隔离在 .cpp
};

}  // namespace drone::control
