#pragma once

// =============================================================================
// staleness_checker.h —— 输入超龄判定（单调毫秒时钟）
//
// 对应规格：docs/物理追踪思路.md
//   §7.1 超龄数据只能触发降级或告警，不能继续生成正常控制指令；
//        超龄判定使用取样时间 + valid_for_ms（MessageHeader 语义）。
//
// 特性：无状态纯函数封装（零分配、无虚函数、无锁）。
// =============================================================================

#include <cstdint>

namespace drone::vtc {

/// 输入超龄判定器（无状态，仅提供静态判定）
class StalenessChecker {
public:
    /// sample_time_ms：取样单调时钟毫秒（<=0 视为从未取样，直接判超龄）；
    /// now_ms：当前单调时钟毫秒；
    /// valid_for_ms：有效期（毫秒，配置给定）。
    /// 返回 true = 已超龄。
    static bool IsStale(std::int64_t sample_time_ms, std::int64_t now_ms,
                        std::int64_t valid_for_ms) {
        if (sample_time_ms <= 0) {
            return true;  // 从未取样：视为超龄（触发降级）
        }
        if (now_ms < sample_time_ms) {
            // 时间戳不一致（时钟源错配/打戳错误）：年龄不可信。
            // 单一单调时钟下不应发生；宁可降级也不信任不可核实的取样。
            return true;
        }
        return (now_ms - sample_time_ms) > valid_for_ms;
    }
};

}  // namespace drone::vtc
