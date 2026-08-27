#include "communication/px4_link.h"

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "communication/mavlink_handler.h"

namespace drone::communication {
namespace {

using namespace std::chrono_literals;

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

    bool WaitForMessage(uint32_t message_id, std::chrono::milliseconds timeout,
                        mavlink_message_t* output) const {
        MavlinkHandler decoder;
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            pollfd descriptor{};
            descriptor.fd = master_fd_;
            descriptor.events = POLLIN;
            if (::poll(&descriptor, 1, 10) <= 0) {
                continue;
            }

            std::array<uint8_t, 512> buffer{};
            const ssize_t count = ::read(master_fd_, buffer.data(), buffer.size());
            if (count <= 0) {
                continue;
            }

            bool found = false;
            decoder.Feed(buffer.data(), static_cast<std::size_t>(count),
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
};

Px4LinkConfig MakeConfig(const std::string& device) {
    Px4LinkConfig config;
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
    config.heartbeat_timeout = 100ms;
    config.state_publish_interval = 20ms;
    config.reconnect_interval = 100ms;
    config.setpoint_queue_capacity = 4;
    return config;
}

std::vector<uint8_t> EncodePx4Heartbeat(MavlinkHandler& encoder, uint8_t system_id,
                                        uint8_t component_id, bool armed,
                                        uint8_t main_mode, uint8_t sub_mode) {
    const uint8_t base_mode = static_cast<uint8_t>(
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
        (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0));
    const uint32_t custom_mode =
        (static_cast<uint32_t>(main_mode) << 16U) |
        (static_cast<uint32_t>(sub_mode) << 24U);
    return encoder.Encode(
        [=](mavlink_status_t* status, mavlink_message_t* message) {
            return mavlink_msg_heartbeat_pack_status(
                system_id, component_id, status, message,
                MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, base_mode,
                custom_mode, MAV_STATE_ACTIVE);
        });
}

template <typename Packer>
void AppendEncoded(MavlinkHandler& encoder, std::vector<uint8_t>* output,
                   Packer&& packer) {
    auto frame = encoder.Encode(std::forward<Packer>(packer));
    output->insert(output->end(), frame.begin(), frame.end());
}

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

TEST(Px4LinkTest, ConfigRejectsInvalidIdentityAndVersion) {
    Px4LinkConfig config;
    config.firmware_version.clear();
    EXPECT_THROW(config.Validate(), std::invalid_argument);

    config.firmware_version = "1.17.0";
    config.onboard_component_id = 0;
    EXPECT_THROW(config.Validate(), std::invalid_argument);

    config.onboard_component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    config.mavlink_version = 3;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
}

TEST(Px4LinkTest, StartsAndSendsOnboardHeartbeat) {
    PseudoTerminal terminal;
    Px4Link link(MakeConfig(terminal.SlaveName()));

    ASSERT_TRUE(link.Start());
    mavlink_message_t message{};
    ASSERT_TRUE(terminal.WaitForMessage(MAVLINK_MSG_ID_HEARTBEAT, 500ms, &message));
    EXPECT_EQ(message.sysid, 1);
    EXPECT_EQ(message.compid, MAV_COMP_ID_ONBOARD_COMPUTER);

    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    EXPECT_EQ(heartbeat.type, MAV_TYPE_ONBOARD_CONTROLLER);
    EXPECT_EQ(heartbeat.autopilot, MAV_AUTOPILOT_INVALID);

    link.Stop();
    EXPECT_FALSE(link.IsRunning());
}

TEST(Px4LinkTest, Px4HeartbeatConnectsAndPublishesBasicSnapshot) {
    PseudoTerminal terminal;
    Px4Link link(MakeConfig(terminal.SlaveName()));
    auto subscription = link.StateOutput().Subscribe(16);
    ASSERT_TRUE(link.Start());

    MavlinkHandler px4_encoder;
    ASSERT_TRUE(terminal.Write(EncodePx4Heartbeat(
        px4_encoder, 1, MAV_COMP_ID_AUTOPILOT1, true, 6, 0)));
    ASSERT_TRUE(WaitUntil([&link] { return link.IsConnected(); }, 500ms));

    bool found_connected_snapshot = false;
    common::FlightStateSnapshot connected_snapshot;
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = subscription.WaitTakeFor(50ms);
        if (message && (*message)->connected) {
            connected_snapshot = **message;
            found_connected_snapshot = true;
            break;
        }
    }

    ASSERT_TRUE(found_connected_snapshot);
    EXPECT_TRUE(connected_snapshot.armed);
    EXPECT_EQ(connected_snapshot.base_mode & MAV_MODE_FLAG_SAFETY_ARMED,
              MAV_MODE_FLAG_SAFETY_ARMED);
    EXPECT_EQ(connected_snapshot.flight_mode, 6);
    EXPECT_EQ(connected_snapshot.flight_sub_mode, 0);
    EXPECT_EQ(connected_snapshot.system_status, MAV_STATE_ACTIVE);
    EXPECT_EQ(connected_snapshot.header.source_id, 1);
    EXPECT_EQ(connected_snapshot.header.health, 1);
    EXPECT_GT(link.ReceiveCount(), 0u);

    link.Stop();
}

TEST(Px4LinkTest, HeartbeatTimeoutPublishesDisconnectedState) {
    PseudoTerminal terminal;
    auto config = MakeConfig(terminal.SlaveName());
    config.heartbeat_timeout = 80ms;
    Px4Link link(config);
    auto subscription = link.StateOutput().Subscribe(32);
    ASSERT_TRUE(link.Start());

    MavlinkHandler px4_encoder;
    ASSERT_TRUE(terminal.Write(EncodePx4Heartbeat(
        px4_encoder, 1, MAV_COMP_ID_AUTOPILOT1, false, 3, 0)));
    ASSERT_TRUE(WaitUntil([&link] { return link.IsConnected(); }, 500ms));
    while (subscription.TryTake()) {
        // 丢弃连接建立前和连接状态的旧快照，只等待本次心跳超时事件。
    }
    ASSERT_TRUE(WaitUntil([&link] { return !link.IsConnected(); }, 500ms));

    bool found_disconnected_snapshot = false;
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = subscription.WaitTakeFor(50ms);
        if (message && !(*message)->connected && (*message)->header.sequence > 1) {
            found_disconnected_snapshot = true;
            break;
        }
    }
    EXPECT_TRUE(found_disconnected_snapshot);
    link.Stop();
}

TEST(Px4LinkTest, IgnoresHeartbeatFromUnexpectedSystem) {
    PseudoTerminal terminal;
    Px4Link link(MakeConfig(terminal.SlaveName()));
    ASSERT_TRUE(link.Start());

    MavlinkHandler other_encoder;
    ASSERT_TRUE(terminal.Write(EncodePx4Heartbeat(
        other_encoder, 2, MAV_COMP_ID_AUTOPILOT1, true, 6, 0)));
    std::this_thread::sleep_for(80ms);

    EXPECT_FALSE(link.IsConnected());
    EXPECT_EQ(link.ReceiveCount(), 0u);
    link.Stop();
}

TEST(Px4LinkTest, TelemetryMessagesPopulateFlightStateSnapshot) {
    PseudoTerminal terminal;
    auto config = MakeConfig(terminal.SlaveName());
    config.heartbeat_timeout = 500ms;
    config.telemetry_timeout = 300ms;
    Px4Link link(config);
    auto subscription = link.StateOutput().Subscribe(64);
    ASSERT_TRUE(link.Start());

    MavlinkHandler encoder;
    std::vector<uint8_t> bytes = EncodePx4Heartbeat(
        encoder, 1, MAV_COMP_ID_AUTOPILOT1, true, 6, 0);

    mavlink_extended_sys_state_t extended{};
    extended.landed_state = MAV_LANDED_STATE_ON_GROUND;
    AppendEncoded(encoder, &bytes, [&](mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_extended_sys_state_encode_status(
            1, MAV_COMP_ID_AUTOPILOT1, status, message, &extended);
    });

    mavlink_gps_raw_int_t gps{};
    gps.fix_type = GPS_FIX_TYPE_3D_FIX;
    AppendEncoded(encoder, &bytes, [&](mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_gps_raw_int_encode_status(
            1, MAV_COMP_ID_AUTOPILOT1, status, message, &gps);
    });

    mavlink_global_position_int_t global{};
    global.lat = 311234567;
    global.lon = 1211234567;
    global.alt = 123450;
    global.vx = 120;
    global.vy = -230;
    global.vz = 40;
    AppendEncoded(encoder, &bytes, [&](mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_global_position_int_encode_status(
            1, MAV_COMP_ID_AUTOPILOT1, status, message, &global);
    });

    mavlink_local_position_ned_t local{};
    local.x = 1.25f;
    local.y = -2.5f;
    local.z = -3.75f;
    local.vx = 4.f;
    local.vy = 5.f;
    local.vz = 6.f;
    AppendEncoded(encoder, &bytes, [&](mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_local_position_ned_encode_status(
            1, MAV_COMP_ID_AUTOPILOT1, status, message, &local);
    });

    mavlink_attitude_t attitude{};
    attitude.roll = 0.1f;
    attitude.pitch = -0.2f;
    attitude.yaw = 1.3f;
    AppendEncoded(encoder, &bytes, [&](mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_attitude_encode_status(
            1, MAV_COMP_ID_AUTOPILOT1, status, message, &attitude);
    });

    mavlink_sys_status_t system_status{};
    system_status.voltage_battery = 12000;
    system_status.current_battery = 123;
    system_status.battery_remaining = 80;
    AppendEncoded(encoder, &bytes, [&](mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_sys_status_encode_status(
            1, MAV_COMP_ID_AUTOPILOT1, status, message, &system_status);
    });

    mavlink_battery_status_t battery{};
    for (std::size_t index = 0; index < 10; ++index) {
        battery.voltages[index] = std::numeric_limits<uint16_t>::max();
    }
    battery.id = 0;
    battery.voltages[0] = 16000;
    battery.current_battery = 250;
    battery.battery_remaining = 75;
    AppendEncoded(encoder, &bytes, [&](mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_battery_status_encode_status(
            1, MAV_COMP_ID_AUTOPILOT1, status, message, &battery);
    });

    mavlink_home_position_t home{};
    home.latitude = 312345678;
    home.longitude = 1212345678;
    home.altitude = 100000;
    AppendEncoded(encoder, &bytes, [&](mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_home_position_encode_status(
            1, MAV_COMP_ID_AUTOPILOT1, status, message, &home);
    });

    mavlink_rc_channels_t rc{};
    rc.chancount = 8;
    rc.rssi = 200;
    AppendEncoded(encoder, &bytes, [&](mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_rc_channels_encode_status(
            1, MAV_COMP_ID_AUTOPILOT1, status, message, &rc);
    });

    mavlink_autopilot_version_t version{};
    version.flight_sw_version = (1u << 24U) | (17u << 16U) | (0u << 8U);
    version.capabilities = MAV_PROTOCOL_CAPABILITY_MAVLINK2;
    AppendEncoded(encoder, &bytes, [&](mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_autopilot_version_encode_status(
            1, MAV_COMP_ID_AUTOPILOT1, status, message, &version);
    });

    ASSERT_TRUE(terminal.Write(bytes));

    common::FlightStateSnapshot snapshot;
    bool found = false;
    const auto deadline = std::chrono::steady_clock::now() + 800ms;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = subscription.WaitTakeFor(50ms);
        if (message && (*message)->connected && (*message)->landed_state_valid &&
            (*message)->global_position_valid && (*message)->local_position_valid &&
            (*message)->attitude_valid && (*message)->battery_valid &&
            (*message)->home_valid && (*message)->rc_state_valid &&
            (*message)->autopilot_version_valid) {
            snapshot = **message;
            found = true;
            break;
        }
    }

    ASSERT_TRUE(found);
    EXPECT_TRUE(snapshot.landed);
    EXPECT_TRUE(snapshot.gps_state_valid);
    EXPECT_TRUE(snapshot.gps_fix);
    EXPECT_EQ(snapshot.gps_fix_type, GPS_FIX_TYPE_3D_FIX);
    EXPECT_EQ(snapshot.latitude_1e7, global.lat);
    EXPECT_EQ(snapshot.longitude_1e7, global.lon);
    EXPECT_EQ(snapshot.altitude_mm, global.alt);
    EXPECT_FLOAT_EQ(snapshot.local_x_m, local.x);
    EXPECT_FLOAT_EQ(snapshot.local_y_m, local.y);
    EXPECT_FLOAT_EQ(snapshot.local_z_m, local.z);
    EXPECT_FLOAT_EQ(snapshot.vx_mps, local.vx);
    EXPECT_FLOAT_EQ(snapshot.roll_rad, attitude.roll);
    EXPECT_FLOAT_EQ(snapshot.pitch_rad, attitude.pitch);
    EXPECT_FLOAT_EQ(snapshot.yaw_rad, attitude.yaw);
    EXPECT_FLOAT_EQ(snapshot.battery_voltage_v, 16.f);
    EXPECT_TRUE(snapshot.battery_current_valid);
    EXPECT_FLOAT_EQ(snapshot.battery_current_a, 2.5f);
    EXPECT_TRUE(snapshot.battery_remaining_valid);
    EXPECT_FLOAT_EQ(snapshot.battery_remaining_pct, 75.f);
    EXPECT_EQ(snapshot.home_lat_1e7, home.latitude);
    EXPECT_EQ(snapshot.home_lon_1e7, home.longitude);
    EXPECT_EQ(snapshot.home_altitude_mm, home.altitude);
    EXPECT_TRUE(snapshot.rc_connected);
    EXPECT_EQ(snapshot.rc_rssi, 200);
    EXPECT_EQ(snapshot.firmware_major, 1);
    EXPECT_EQ(snapshot.firmware_minor, 17);
    EXPECT_EQ(snapshot.firmware_patch, 0);
    EXPECT_EQ(snapshot.autopilot_capabilities,
              static_cast<uint64_t>(MAV_PROTOCOL_CAPABILITY_MAVLINK2));
    EXPECT_EQ(snapshot.header.frame_id, 3);

    link.Stop();
}

TEST(Px4LinkTest, TelemetryValidityExpiresWithoutDisconnectingHeartbeat) {
    PseudoTerminal terminal;
    auto config = MakeConfig(terminal.SlaveName());
    config.heartbeat_timeout = 500ms;
    config.telemetry_timeout = 60ms;
    Px4Link link(config);
    auto subscription = link.StateOutput().Subscribe(64);
    ASSERT_TRUE(link.Start());

    MavlinkHandler encoder;
    std::vector<uint8_t> bytes = EncodePx4Heartbeat(
        encoder, 1, MAV_COMP_ID_AUTOPILOT1, false, 3, 0);
    mavlink_local_position_ned_t local{};
    local.x = 10.f;
    AppendEncoded(encoder, &bytes, [&](mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_local_position_ned_encode_status(
            1, MAV_COMP_ID_AUTOPILOT1, status, message, &local);
    });
    ASSERT_TRUE(terminal.Write(bytes));

    ASSERT_TRUE(WaitUntil([&] {
        while (auto message = subscription.TryTake()) {
            if ((*message)->local_position_valid) {
                return true;
            }
        }
        return false;
    }, 500ms));

    bool found_expired = false;
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < deadline) {
        auto message = subscription.WaitTakeFor(50ms);
        if (message && (*message)->connected && !(*message)->local_position_valid) {
            found_expired = true;
            break;
        }
    }
    EXPECT_TRUE(found_expired);
    EXPECT_TRUE(link.IsConnected());
    link.Stop();
}

TEST(Px4LinkTest, CanRestartAfterStop) {
    PseudoTerminal terminal;
    Px4Link link(MakeConfig(terminal.SlaveName()));

    ASSERT_TRUE(link.Start());
    link.Stop();
    ASSERT_TRUE(link.Start());
    EXPECT_TRUE(link.IsRunning());
    link.Stop();
}

TEST(Px4LinkTest, CommandSendingRemainsExplicitlyUnavailableInTelemetryStage) {
    PseudoTerminal terminal;
    Px4Link link(MakeConfig(terminal.SlaveName()));
    EXPECT_FALSE(link.SendCommand(MAV_CMD_COMPONENT_ARM_DISARM,
                                  1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f));
    EXPECT_EQ(link.ErrorCount(), 1u);
}

}  // namespace
}  // namespace drone::communication
