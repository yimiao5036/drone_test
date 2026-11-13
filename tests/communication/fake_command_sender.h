#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <mutex>
#include <vector>

#include "common/types.h"
#include "communication/px4_link.h"

namespace drone::test {

/// MAV_CMD_DO_SET_ACTUATOR（MAVLink common.xml #187）。测试假件不引入
/// MAVLink 头，使用显式常量与实现侧关联字段对应。
inline constexpr uint16_t kDoSetActuatorCommand = 187;

/// 命令窄接口共享假件：记录 SendCommand 调用（顺序/参数/入队结果），
/// ACK/超时/取消均可脚本化，供状态机与执行器相关测试复用。
/// 非 final：需要模拟“ACK 在 SendCommand 返回前到达”等并发时序时，可在
/// 测试内派生并覆写 SendCommand（见 net_actuator_test）。
class FakeCommandSender : public communication::IPx4CommandSender {
public:
    struct CallRecord {
        uint16_t command = 0;
        std::array<float, 7> params{};
        bool queued = false;  // SendCommand 返回值（false=链路未就绪/队列满）
    };

    bool SendCommand(uint16_t mavlink_command, float param1, float param2,
                     float param3, float param4, float param5, float param6,
                     float param7) override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++call_count_;
        last_command_ = mavlink_command;
        last_params_ = {param1, param2, param3, param4, param5, param6, param7};
        const bool queued = fail_calls_remaining_ == 0;
        if (!queued) {
            --fail_calls_remaining_;
        }
        history_.push_back({mavlink_command, last_params_, queued});
        return queued;
    }

    uint64_t AckMatchCount() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return ack_.matched_count;
    }

    uint64_t AckTimeoutCount() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return ack_.timeout_count;
    }

    common::CommandAckStatus AckStatus() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return ack_;
    }

    void CancelPendingCommands() override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++cancel_count_;
    }

    /// 模拟一次 ACK 匹配（更新匹配计数与最近 ACK 快照）。
    /// 187 同时更新专用关联字段，供执行器独立于其它命令判断推进/故障。
    void SimulateAck(uint16_t command, uint8_t result, uint64_t time_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++ack_.matched_count;
        ack_.last_command = command;
        ack_.last_result = result;
        ack_.last_time_ms = time_ms;
        if (command == kDoSetActuatorCommand) {
            ++ack_.do_set_actuator_ack_count;
            ack_.do_set_actuator_last_result = result;
            ack_.do_set_actuator_last_time_ms = time_ms;
        }
    }

    /// 模拟一次 ACK 超时（更新超时计数与最近超时命令）。
    void SimulateTimeout(uint16_t command) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++ack_.timeout_count;
        ack_.last_timeout_command = command;
        if (command == kDoSetActuatorCommand) {
            ++ack_.do_set_actuator_timeout_count;
        }
    }

    /// 令接下来 count 次 SendCommand 返回失败（模拟 PX4 未连接/队列满）。
    void FailNextCalls(uint64_t count) {
        std::lock_guard<std::mutex> lock(mutex_);
        fail_calls_remaining_ = count;
    }

    uint64_t CallCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return call_count_;
    }

    uint64_t CancelCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cancel_count_;
    }

    uint16_t LastCommand() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_command_;
    }

    std::array<float, 7> LastParams() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_params_;
    }

    /// 全部调用记录（按时间顺序的拷贝）。
    std::vector<CallRecord> History() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return history_;
    }

private:
    mutable std::mutex mutex_;
    common::CommandAckStatus ack_;
    uint64_t call_count_ = 0;
    uint64_t cancel_count_ = 0;
    uint64_t fail_calls_remaining_ = 0;
    uint16_t last_command_ = 0;
    std::array<float, 7> last_params_{};
    std::vector<CallRecord> history_;
};

}  // namespace drone::test
