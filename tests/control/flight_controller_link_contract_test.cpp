/**
 * @file flight_controller_link_contract_test.cpp
 * @brief FlightController → Px4Link 跨模块契约回归（PTY 真实链路）
 *
 * 背景（Task 11 fix round 1/5）：`FlightController::Publish()` 曾未标注
 * `header.frame_id`，而 `Px4Link::IsSetpointStructurallyValid` 强制要求
 * `frame_id == 3`（局部 NED）——控制器全部设定值会被 Px4Link 静默拒绝，
 * 控制器 → Px4Link 全链零发送。该缺陷穿过 Task 3~5 的单测（两侧各自都
 * 自洽：控制器侧不校验输出 frame_id，Px4Link 侧只收自己构造的合法帧）。
 *
 * 本用例是该契约的回归网：把控制器输出真正接到真实 `Px4Link`（PTY 伪终端
 * 对端，与 tests/communication/px4_link_test.cpp 同构），授权控制器
 * （armed + Offboard(6/0) 快照 + 带请求位的意图）后断言：
 * 1) `Px4Link::SetpointSendCount()` 随控制周期增长（设定值穿透结构校验
 *    抵达链路线程，而非被静默拒绝）；
 * 2) 线上可解码出 `SET_POSITION_TARGET_LOCAL_NED`：坐标系 LOCAL_NED、
 *    速度通道未被 type_mask 忽略、数值等于指令速度（限幅流水线爬升到位）。
 */
#include "control/flight_controller.h"

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "common/topic.h"
#include "common/types.h"
#include "communication/mavlink_handler.h"
#include "communication/px4_link.h"
#include "config/config.h"

namespace drone::control {
namespace {

using namespace std::chrono_literals;

/// 返回当前进程单调时钟毫秒（与实现侧同源，参照 mission_state_machine.cpp）。
uint64_t MonotonicMs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

/// 轮询等待谓词成立（参照 tests/communication/px4_link_test.cpp）。
bool WaitUntil(const std::function<bool()>& predicate,
               std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(5ms);
    }
    return predicate();
}

/// 伪终端对端（最小版，与 tests/communication/px4_link_test.cpp 同构）：
/// 本用例需要真实串口链路对端才能验证“设定值是否穿透 Px4Link 结构校验”。
class PseudoTerminal final {
public:
    PseudoTerminal() {
        master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (master_fd_ < 0 || ::grantpt(master_fd_) != 0 || ::unlockpt(master_fd_) != 0) {
            if (master_fd_ >= 0) {
                ::close(master_fd_);
            }
            throw std::runtime_error("创建 PX4 测试伪终端失败");
        }
        const char* name = ::ptsname(master_fd_);
        if (name == nullptr) {
            ::close(master_fd_);
            throw std::runtime_error("读取 PX4 测试伪终端名称失败");
        }
        slave_name_ = name;
    }

    ~PseudoTerminal() {
        if (master_fd_ >= 0) {
            ::close(master_fd_);
        }
    }

    PseudoTerminal(const PseudoTerminal&) = delete;
    PseudoTerminal& operator=(const PseudoTerminal&) = delete;

    const std::string& SlaveName() const { return slave_name_; }

    bool Write(const std::vector<uint8_t>& data) const {
        std::size_t written = 0;
        while (written < data.size()) {
            pollfd descriptor{};
            descriptor.fd = master_fd_;
            descriptor.events = POLLOUT;
            if (::poll(&descriptor, 1, 100) <= 0) {
                return false;
            }
            const ssize_t count =
                ::write(master_fd_, data.data() + written, data.size() - written);
            if (count <= 0) {
                return false;
            }
            written += static_cast<std::size_t>(count);
        }
        return true;
    }

    /// 读取并解码直到命中指定消息 ID（解码器跨轮保持，容忍分帧）；
    /// 未命中返回 false。
    bool WaitForMessage(uint32_t message_id, std::chrono::milliseconds timeout,
                        mavlink_message_t* output) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            pollfd descriptor{};
            descriptor.fd = master_fd_;
            descriptor.events = POLLIN;
            if (::poll(&descriptor, 1, 5) <= 0) {
                continue;
            }
            std::array<uint8_t, 512> buffer{};
            const ssize_t count = ::read(master_fd_, buffer.data(), buffer.size());
            if (count <= 0) {
                continue;
            }
            bool found = false;
            decoder_.Feed(buffer.data(), static_cast<std::size_t>(count),
                          [&](const mavlink_message_t& message) {
                              if (!found && message.msgid == message_id) {
                                  found = true;
                                  if (output != nullptr) {
                                      *output = message;
                                  }
                              }
                          });
            if (found) {
                return true;
            }
        }
        return false;
    }

private:
    int master_fd_ = -1;
    std::string slave_name_;
    communication::MavlinkHandler decoder_;
};

drone::communication::Px4LinkConfig MakeLinkConfig(const std::string& device) {
    drone::communication::Px4LinkConfig config;
    config.serial.device = device;
    config.serial.baud_rate = 115200;
    config.serial.read_timeout = 5ms;
    config.serial.write_timeout = 100ms;
    config.onboard_system_id = 1;
    config.onboard_component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    config.target_system_id = 1;
    config.target_component_id = MAV_COMP_ID_AUTOPILOT1;
    config.mavlink_version = 2;
    config.heartbeat_send_interval = 30ms;
    config.heartbeat_timeout = 500ms;
    config.state_publish_interval = 20ms;
    config.reconnect_interval = 100ms;
    config.command_ack_timeout = 100ms;
    config.setpoint_send_interval = 10ms;
    config.setpoint_timeout = 100ms;
    config.setpoint_queue_capacity = 4;
    return config;
}

/// PX4 心跳：armed + Offboard(6/0)（控制器授权链的快照门禁条件）。
std::vector<uint8_t> EncodePx4Heartbeat(communication::MavlinkHandler& encoder, bool armed,
                                        uint8_t main_mode, uint8_t sub_mode) {
    const uint8_t base_mode = static_cast<uint8_t>(
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
        (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0));
    const uint32_t custom_mode = (static_cast<uint32_t>(main_mode) << 16U) |
                                 (static_cast<uint32_t>(sub_mode) << 24U);
    return encoder.Encode(
        [=](mavlink_status_t* status, mavlink_message_t* message) {
            return mavlink_msg_heartbeat_pack_status(
                1, MAV_COMP_ID_AUTOPILOT1, status, message, MAV_TYPE_QUADROTOR,
                MAV_AUTOPILOT_PX4, base_mode, custom_mode, MAV_STATE_ACTIVE);
        });
}

/// 状态机侧速度意图（带 bit31 显式请求位，保持新鲜）。
void PublishVelocityIntent(common::Topic<common::ControlIntent>& topic, float vx) {
    common::ControlIntent intent;
    intent.type = common::ControlIntentType::kVelocityHeading;
    intent.control_source = 1;  // 1=视觉（对应 S7 伺服）
    intent.reason_code = common::kControlIntentReasonRequestControl;
    intent.header.receive_time_ms = MonotonicMs();
    intent.header.frame_id = 3;  // 局部 NED
    intent.target_x = vx;
    intent.valid_until_ms = MonotonicMs() + 500;
    (void)topic.Emplace(intent);
}

/// 跨模块契约：控制器的设定值必须穿透 Px4Link 结构校验并真正上线。
TEST(FlightControllerLinkContractTest, SetpointsReachPx4LinkWire) {
    constexpr float kCommandedNorthMps = 1.5f;
    constexpr double kVelocityToleranceMps = 0.25;
    constexpr uint32_t kMainModeOffboard = 6;

    PseudoTerminal terminal;
    drone::communication::Px4Link link(MakeLinkConfig(terminal.SlaveName()));

    drone::config::FlightControllerConfig controller_config;
    controller_config.period = 20ms;
    controller_config.max_speed_mps = 2.0;
    controller_config.max_accel_mps2 = 2.0;
    controller_config.intent_timeout = 200ms;
    controller_config.coast_timeout = 1000ms;
    FlightController controller(controller_config);

    common::Topic<common::ControlIntent> intent_topic;
    controller.SetInput(intent_topic);
    controller.SetPx4LinkSource(&link);
    link.SetInput(controller.SetpointOutput());
    ASSERT_TRUE(link.Start());
    ASSERT_TRUE(controller.Start());

    communication::MavlinkHandler px4_encoder;
    ASSERT_TRUE(terminal.Write(
        EncodePx4Heartbeat(px4_encoder, true, static_cast<uint8_t>(kMainModeOffboard), 0)));
    ASSERT_TRUE(WaitUntil([&link] { return link.IsConnected(); }, 1s));

    // 驱动循环：持续发布新鲜意图（保持授权与 ACTIVE），同时监听线上设定值。
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    bool setpoint_sent = false;
    bool wire_contract_ok = false;
    while (std::chrono::steady_clock::now() < deadline && !wire_contract_ok) {
        PublishVelocityIntent(intent_topic, kCommandedNorthMps);
        // 周期性心跳维持链路连接与 armed/Offboard 快照（授权链门禁每拍核验）。
        (void)terminal.Write(
            EncodePx4Heartbeat(px4_encoder, true, static_cast<uint8_t>(kMainModeOffboard), 0));
        setpoint_sent = setpoint_sent || link.SetpointSendCount() > 0;

        mavlink_message_t message{};
        if (terminal.WaitForMessage(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, 20ms,
                                    &message)) {
            mavlink_set_position_target_local_ned_t target{};
            mavlink_msg_set_position_target_local_ned_decode(&message, &target);
            // 契约：局部 NED 帧、速度通道未被 type_mask 忽略、数值=指令速度
            // （slew 限幅爬升到位，2m/s² × 20ms 步长，约 0.8 秒到达 1.5m/s）。
            if (target.coordinate_frame == MAV_FRAME_LOCAL_NED &&
                (target.type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) == 0 &&
                std::fabs(static_cast<double>(target.vx) - kCommandedNorthMps) <=
                    kVelocityToleranceMps) {
                wire_contract_ok = true;
            }
        }
        std::this_thread::sleep_for(5ms);
    }

    EXPECT_TRUE(setpoint_sent)
        << "控制器设定值未穿透 Px4Link 结构校验（frame_id 等契约回归）";
    EXPECT_EQ(controller.Authority(), ControlAuthority::kActive);
    EXPECT_TRUE(wire_contract_ok)
        << "线上未解码出等于指令速度的 SET_POSITION_TARGET_LOCAL_NED（LOCAL_NED、"
           "速度通道生效）";

    controller.Stop();
    link.Stop();
}

}  // namespace
}  // namespace drone::control
