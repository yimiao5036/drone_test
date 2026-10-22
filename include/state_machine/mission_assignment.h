/**
 * @file mission_assignment.h
 * @brief 任务分配状态助手（MissionAssignmentTracker）
 *
 * 属于 drone/state_machine 模块。纯逻辑助手：不启动线程、不访问 Topic、
 * 不读取系统时钟、不依赖 PX4；所有时间由调用方显式传入。状态机持有一个
 * 实例，用它记录 65014 分配的生命周期、一次上电一次空中任务语义、
 * 按 assignment.target_id 过滤目标，以及重复 assignment_id 的结果重发。
 *
 * 典型迁移：
 *
 *   kIdle --Accept--> kAssigned --BeginTakeoff--> kTakingOff
 *   kTakingOff --TakeoffComplete--> kAirborne --BeginEngage--> kEngaged
 *   kEngaged --BeginReturn--> kReturning --Terminal--> kTerminal
 *
 *   地面中止：kAssigned/kTakingOff --AbortOnGround（尚未离地）--> kIdle
 *   空中中止：已离地 --AbortAirborne--> kReturning --Terminal--> kTerminal
 *
 * MarkAirborne() 只记录“实际 armed && !landed”并消耗任务机会，不推进阶段；
 * 是否已到 S4 高度由 TakeoffComplete() 单独推进。实现文档见同目录
 * mission_assignment.md。
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "common/types.h"

namespace drone::state_machine {

/// 分配生命周期阶段。与 MissionState 无关：状态机负责 S2~S16，
/// 助手只跟踪“这一次分配”走到哪一步。
enum class AssignmentPhase : uint8_t {
    kIdle,       ///< 无活动分配，可接受新分配
    kAssigned,   ///< 已接受，等待进入 S3
    kTakingOff,  ///< S3/S4：解锁、Offboard、爬升
    kAirborne,   ///< 已到达任务高度，等待进入 S5
    kEngaged,    ///< S5~S10 任务执行中
    kReturning,  ///< S11~S13 返航/降落
    kTerminal,   ///< S14/S16 或空中任务结束，重启前不再分配
};

/// 任务分配纯逻辑助手。
///
/// 线程安全：**无内部同步**，只允许在状态机决策线程内调用。
/// 异常：全部接口不抛异常；非法调用返回 false 或保持当前状态。
/// 日志：本类不写日志，状态迁移由调用方按“仅状态变化”原则节流输出。
class MissionAssignmentTracker {
public:
    /// 接受一次分配。仅 kIdle、未消耗空中任务机会且 target_id != 0 时成功；
    /// 成功后记录 assignment_id/target_id/拦截类型并进入 kAssigned。
    /// 拦截类型与本机是否匹配、flags 授权、valid_for_ms 等预检由调用方完成。
    bool Accept(const common::MissionAssignment& assignment, uint64_t now_ms);

    /// kAssigned → kTakingOff，记录起飞原点（局部 NED，向上为负）。
    /// 时刻由调用方用于自己的超时计时；其他阶段调用为空操作。
    void BeginTakeoff(float x, float y, float z, uint64_t now_ms);

    /// 记录“实际 armed && !landed”（PX4 判定，不依赖是否已进入 S5），
    /// 消耗本次上电唯一的空中任务机会。不推进阶段；重复调用幂等。
    /// 无活动分配时为空操作。
    void MarkAirborne(uint64_t now_ms);

    /// S4 到达任务高度：kTakingOff → kAirborne。不改变任务机会消耗状态；
    /// 其他阶段为空操作（已进入 kAirborne 后可安全重复调用）。
    void TakeoffComplete();

    /// kAirborne（或已消耗任务机会且仍在起飞阶段）→ kEngaged。
    void BeginEngage();

    /// kAirborne/kEngaged → kReturning，用于正常结束任务进入返航。
    void BeginReturn();

    /// 进入 kTerminal（S14/S16 或空中任务结束）。终端后 Accept 永久返回 false，
    /// 直到进程重启；重复调用幂等。
    void Terminal();

    /// 起飞前中止：尚未消耗空中任务机会时回到 kIdle 并清空本次分配，
    /// 允许地面站用新的 assignment_id 重新分配；已离地调用为空操作。
    void AbortOnGround();

    /// 已离地中止：已消耗任务机会或处于 kAirborne/kEngaged/kReturning 时
    /// 进入 kReturning；之后由调用方在落地后调用 Terminal()。
    /// 空中中止不得回 kIdle，也不得再次分配。
    void AbortAirborne();

    /// 仅活动分配（kAssigned~kReturning）且 target_id 匹配时返回 true。
    /// 0 永远不匹配；kTerminal/kIdle 返回 false。
    bool AllowsTarget(uint32_t target_id) const;

    /// 映射到 65013/65015 的 availability。kIdle 固定返回 kReady：
    /// 硬件健康、电池、控制路径等可调度性由状态机预检覆盖，助手不判断。
    common::MissionAvailability Availability() const;

    AssignmentPhase Phase() const { return phase_; }

    /// 是否持有活动分配（kAssigned~kReturning）。
    bool HasActiveAssignment() const;

    /// 本次上电是否已实际离地（once-per-boot 语义）。
    bool ConsumedAirborneMission() const { return consumed_; }

    uint32_t AssignmentId() const { return assignment_id_; }
    uint32_t TargetId() const { return target_id_; }
    uint8_t InterceptorType() const { return interceptor_type_; }

    /// 起飞原点（局部 NED）。未调用 BeginTakeoff 或地面中止后为 0。
    float OriginX() const { return origin_x_; }
    float OriginY() const { return origin_y_; }
    float OriginZ() const { return origin_z_; }

    /// 记录某个 assignment_id 的最终回执，供重复 65014 直接重发 65015
    /// 而不重新迁移。保存最近 8 条，同一 id 覆盖为最新结果；超过上限
    /// 覆盖最旧记录，内存有界。
    void RememberResult(uint32_t assignment_id,
                        common::MissionAssignmentResult result,
                        common::MissionAssignmentReason reason);

    /// 查询 assignment_id 是否已处理过。命中时填充 result/reason（指针可空）
    /// 并返回 true；调用方只重发 65015，不得再次 Accept/迁移。
    bool WasHandled(uint32_t assignment_id,
                    common::MissionAssignmentResult* result,
                    common::MissionAssignmentReason* reason) const;

private:
    struct RememberedResult {
        uint32_t assignment_id = 0;
        common::MissionAssignmentResult result =
            common::MissionAssignmentResult::kRejected;
        common::MissionAssignmentReason reason =
            common::MissionAssignmentReason::kNone;
    };

    static constexpr std::size_t kMaxRememberedResults = 8;

    /// 清空本次分配字段（不动 consumed_ 与结果历史）。
    void ClearAssignment();

    AssignmentPhase phase_ = AssignmentPhase::kIdle;
    bool consumed_ = false;  ///< 已观测到 armed && !landed
    uint32_t assignment_id_ = 0;
    uint32_t target_id_ = 0;
    uint8_t interceptor_type_ = 0;
    float origin_x_ = 0.f;
    float origin_y_ = 0.f;
    float origin_z_ = 0.f;

    std::array<RememberedResult, kMaxRememberedResults> results_{};
    std::size_t result_count_ = 0;
    std::size_t result_next_ = 0;  ///< 写满后的覆盖位置
};

}  // namespace drone::state_machine
