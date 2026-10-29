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
 * - FlightController 为正式实现（阶段四控制接入 Task 3~5）：固定周期
 *   线程消费意图 + 门禁链（有限性/超龄/控制权）；限幅流水线（Task 4）
 *   与控制权三态/Offboard 周期核验/Home 相对围栏（Task 5）已接入。
 *   实现文档见同目录 flight_controller.md。
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
    /// 须在 Start() 前调用（Start 后周期线程即开始消费订阅）。
    virtual void SetInput(common::Topic<common::ControlIntent>& intent) = 0;

    /// 注入围栏多边形只读来源（MissionStateMachine 实现，2026-10-29）。
    /// 注入后位置式意图按多边形钳制（含内缩余量）并按区域高度带钳制高度；
    /// 未注入或区域无效（顶点 < 3）时不做钳制（圆形兜底围栏已删除）。
    virtual void SetFencePolygonSource(const common::IFencePolygonSource* source) = 0;

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

    void SetFencePolygonSource(const common::IFencePolygonSource* source) override;

    common::Topic<common::Px4Setpoint>& SetpointOutput() override;

    uint64_t SetpointCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t setpoint_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::Px4Setpoint> setpoint_output_;
};

/// 控制权状态（spec §4；Task 5 接入授权/降级，Task 6 接入自动起飞预授权）：
/// - IDLE：无控制权，停发任何设定值；
/// - BOOTSTRAP：自动起飞预授权态（spec §8），在地面且未 armed 时发布
///   起飞原点位置保持流，为 PX4 进入 Offboard 建立合法 setpoint 流；
///   armed + Offboard(6/0) 后转 ACTIVE；条件失效/超时/意图撤销只回 IDLE，
///   **不进入 COAST**；
/// - ACTIVE：正常 Offboard 控制；
/// - COAST：意图短期超龄时的零速刹车保持。
/// 迁移全部由授权链/门禁周期核验驱动，迁移时打 WARN 级状态变化日志。
enum class ControlAuthority : uint8_t { kIdle, kBootstrap, kActive, kCoast };

/// 正式飞行控制器（spec §3 执行层）：固定周期消费 ControlIntent，
/// 门禁链（有限性/超龄/控制权）+ 限幅流水线 + 无扰衔接后发布 Px4Setpoint。
///
/// 当前边界：
/// - 固定周期线程、意图队列排干（只留最新）、入口门禁（有限性/超龄拒绝计数）；
/// - 限幅流水线（Task 4）：速度幅值等比钳制 → SlewRateLimiter 加速度限幅；
///   来源/类型变化时斜坡起点=上一周期实际输出（无扰衔接）；时钟回退冻结输出；
///   未定义 control_source 按安全语义降级（仅刹停类意图生效）；
/// - 控制权三态与 PX4 门禁（Task 5）：每拍经 SetPx4LinkSource 注入的快照核验
///   connected+armed+Offboard(6/0)，并要求最新意图携带状态机显式请求位
///   （reason_code bit31，见 common::kControlIntentReasonRequestControl）；
///   未注入快照源时授权链恒不通过（影子运行，保持 IDLE 停发）；
///   ACTIVE/COAST 中门禁失效立即发布 valid=false 停发并转 IDLE；
///   意图超龄转 COAST 刹车保持，coast_timeout 超时转 IDLE 停发；
/// - 自动起飞预授权（Task 6，spec §8）：IDLE 下最新意图为 kTakeoff + bit31
///   且未超龄、PX4 connected + landed_state_valid&&landed + !armed +
///   local_position/attitude/home 有效时进入 kBootstrap，每拍发布起飞原点
///   位置保持流（kPosition、valid=true、无前馈），armed+Offboard(6/0) 后转
///   ACTIVE；条件失效/意图撤销/超过 bootstrap_timeout 只回 IDLE 并停流。
///   **门禁不读取任何 RC 字段**（rc_state_valid/rc_connected/
///   manual_override_active）——RC 交由 PX4 自己管理；
/// - 多边形围栏（2026-10-29）：位置式意图按地面站多边形钳制（内缩
///   `fence_margin_m`）、高度按区域高度带上限钳制；home_valid=false 时
///   位置式意图拒绝。圆形兜底围栏（原 Task 5 的 max_radius_m/max_altitude_m）
///   已随维护者裁决删除，围栏只有多边形一种。
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

    void SetFencePolygonSource(const common::IFencePolygonSource* source) override;

    common::Topic<common::Px4Setpoint>& SetpointOutput() override;

    uint64_t SetpointCount() const override;
    uint64_t ErrorCount() const override;

    // ---- 诊断（测试与门禁用） ----
    /// 当前控制权状态。
    ControlAuthority Authority() const;
    /// 门禁拒绝意图累计计数（入口门禁：非有限/到达即超龄，按到达计——
    /// 上游每拍重发同一条违例意图时表现为按拍增长；授权后路径：持有期
    /// 超龄在转入 COAST 那一拍 +1，不再逐拍增长；位置式意图无 Home 基准
    /// 拒绝按拍计，直至意图超龄转 COAST）。
    uint64_t RejectedIntentCount() const;
    /// 限幅触发累计计数（速度幅值等比钳制 + 位置式围栏钳制 +
    /// 位置前馈幅值钳制，每次钳制生效 +1，供 G2 验收核对）。
    uint64_t ClampedCount() const;
    /// 位置前馈因非有限值/未知掩码位被丢弃的轴次累计计数（P3）。
    uint64_t FeedforwardDroppedCount() const;
    /// 注入 PX4 链路只读快照源（控制权门禁与 Home 围栏数据源）；
    /// 未注入时授权链恒不通过，控制器保持 IDLE 影子运行。
    void SetPx4LinkSource(const communication::IPx4Link* link);

    // ---- 测试入口（生产装配不得调用） ----
    /// 强制设置控制权状态（单测入口）。注意授权链每拍重估：门禁不通时
    /// 下一拍即撤到 IDLE——维持 ACTIVE 须同时满足快照门禁与意图请求位。
    void TestForceAuthority(ControlAuthority authority);
    /// 以指定时间戳同步执行一拍控制周期（须在未 Start 状态调用，
    /// 与周期线程互斥；Start 后调用被忽略并节流 WARN——运行时守卫，
    /// 防止与周期线程并发访问 Tick 内部无锁状态）。
    void TestTick(uint64_t now_ms);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;  // PIMPL：线程/订阅等实现细节隔离在 .cpp
};

}  // namespace drone::control
