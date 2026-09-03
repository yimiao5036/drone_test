#include "communication/ground_station_link.h"

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
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
        if (master_fd_ < 0 || ::grantpt(master_fd_) != 0 ||
            ::unlockpt(master_fd_) != 0) {
            throw std::runtime_error("创建地面站测试伪终端失败");
        }
        const char* name = ::ptsname(master_fd_);
        if (name == nullptr) {
            throw std::runtime_error("读取地面站测试伪终端名称失败");
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
        return ::write(master_fd_, data.data(), data.size()) ==
               static_cast<ssize_t>(data.size());
    }

    bool WaitFor(const std::function<bool(const mavlink_message_t&)>& predicate,
                 std::chrono::milliseconds timeout,
                 mavlink_message_t* output = nullptr) const {
        MavlinkHandler decoder;
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            pollfd descriptor{master_fd_, POLLIN, 0};
            if (::poll(&descriptor, 1, 10) <= 0) {
                continue;
            }
            std::array<uint8_t, 2048> buffer{};
            const ssize_t count = ::read(master_fd_, buffer.data(), buffer.size());
            if (count <= 0) {
                continue;
            }
            bool found = false;
            decoder.Feed(buffer.data(), static_cast<std::size_t>(count),
                         [&](const mavlink_message_t& message) {
                             if (!found && predicate(message)) {
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

GroundStationLinkConfig MakeConfig(
    const std::string& device, uint8_t aircraft_system_id = 1,
    uint8_t aircraft_component_id = kNetCaptureAircraftComponentId,
    std::string aircraft_type = "net_capture", uint8_t aircraft_number = 1,
    std::string callsign = "捕网-01") {
    GroundStationLinkConfig config;
    config.serial.device = device;
    config.serial.baud_rate = 115200;
    config.serial.data_bits = 8;
    config.serial.stop_bits = 1;
    config.serial.parity = 'N';
    config.serial.read_timeout = 5ms;
    config.serial.write_timeout = 100ms;
    config.aircraft_system_id = aircraft_system_id;
    config.aircraft_component_id = aircraft_component_id;
    config.aircraft_type = std::move(aircraft_type);
    config.aircraft_number = aircraft_number;
    config.callsign = std::move(callsign);
    config.ground_system_id = kGroundStationSystemId;
    config.ground_component_id = kGroundStationComponentId;
    config.mavlink_version = 2;
    config.heartbeat_send_interval = 30ms;
    config.heartbeat_timeout = 80ms;
    config.enable_time_sync = true;
    config.time_sync_acquire_interval = 10ms;
    config.time_sync_steady_interval = 20ms;
    config.time_sync_timeout = 80ms;
    config.time_sync_max_rtt = 40ms;
    config.time_sync_max_offset_jump = 20ms;
    config.time_sync_minimum_samples = 3;
    config.time_sync_window_capacity = 5;
    config.attitude_send_interval = 20ms;
    config.local_position_send_interval = 20ms;
    config.global_position_send_interval = 20ms;
    config.gps_send_interval = 20ms;
    config.extended_state_send_interval = 20ms;
    config.system_status_send_interval = 20ms;
    config.battery_send_interval = 20ms;
    config.home_send_interval = 20ms;
    config.flight_state_queue_capacity = 2;
    return config;
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

std::vector<uint8_t> EncodeGcsHeartbeat(
    uint8_t system_id = kGroundStationSystemId,
    uint8_t component_id = kGroundStationComponentId) {
    MavlinkHandler encoder(MavlinkVersion::kV2);
    return encoder.Encode([system_id, component_id](mavlink_status_t* status,
                                                    mavlink_message_t* message) {
        return mavlink_msg_heartbeat_pack_status(
            system_id, component_id, status, message, MAV_TYPE_GCS,
            MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    });
}

std::vector<uint8_t> EncodeTimeSyncResponse(
    const mavlink_message_t& request, int64_t offset_ns,
    uint8_t source_system = kGroundStationSystemId,
    uint8_t source_component = kGroundStationComponentId,
    uint8_t target_system = 1,
    uint8_t target_component = kNetCaptureAircraftComponentId) {
    mavlink_timesync_t request_payload{};
    mavlink_msg_timesync_decode(&request, &request_payload);
    MavlinkHandler encoder(MavlinkVersion::kV2);
    return encoder.Encode([=](mavlink_status_t* status,
                              mavlink_message_t* message) {
        return mavlink_msg_timesync_pack_status(
            source_system, source_component, status, message,
            request_payload.ts1 + offset_ns, request_payload.ts1,
            target_system, target_component);
    });
}

TEST(GroundStationLinkTest, ConfigRejectsInvalidParameters) {
    GroundStationLinkConfig config;
    config.serial.device = "/dev/ttyS6";
    EXPECT_NO_THROW(config.Validate());

    config.mavlink_version = 3;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
    config.mavlink_version = 1;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
    config.enable_time_sync = false;
    EXPECT_NO_THROW(config.Validate());
    config.enable_time_sync = true;
    config.mavlink_version = 2;
    config.flight_state_queue_capacity = 0;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
    config.flight_state_queue_capacity = 2;
    config.aircraft_component_id = 27;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
    config.aircraft_component_id = kNetCaptureAircraftComponentId;
    config.ground_component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
    config.ground_component_id = kGroundStationComponentId;
    config.time_sync_minimum_samples = 6;
    config.time_sync_window_capacity = 5;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
    config.time_sync_minimum_samples = 3;
    config.time_sync_max_rtt = config.time_sync_timeout;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
}

TEST(GroundStationLinkTest, RequiresFlightStateTopicBeforeStart) {
    PseudoTerminal terminal;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    EXPECT_FALSE(link.Start());
    EXPECT_GT(link.ErrorCount(), 0u);
}

TEST(GroundStationLinkTest, SendsMavlink2HeartbeatWithAircraftIdentity) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName(), 2,
                                      kNetCaptureAircraftComponentId,
                                      "net_capture", 2, "捕网-02"));
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    mavlink_message_t message{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& candidate) {
            return candidate.msgid == MAVLINK_MSG_ID_HEARTBEAT;
        }, 300ms, &message));
    EXPECT_EQ(message.magic, MAVLINK_STX);
    EXPECT_EQ(message.sysid, 2);
    EXPECT_EQ(message.compid, kNetCaptureAircraftComponentId);
    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    EXPECT_EQ(heartbeat.type, MAV_TYPE_ONBOARD_CONTROLLER);
    EXPECT_EQ(heartbeat.autopilot, MAV_AUTOPILOT_INVALID);

    link.Stop();
}

TEST(GroundStationLinkTest, SendsRocketHeartbeatWithSameSystemIdAndDifferentComponent) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName(), 1,
                                      kRocketAircraftComponentId,
                                      "rocket", 1, "火箭-01"));
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    mavlink_message_t message{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& candidate) {
            return candidate.msgid == MAVLINK_MSG_ID_HEARTBEAT;
        }, 300ms, &message));
    EXPECT_EQ(message.sysid, 1);
    EXPECT_EQ(message.compid, kRocketAircraftComponentId);

    link.Stop();
}

TEST(GroundStationLinkTest, EncodesFlightSnapshotAsStandardTelemetry) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    common::FlightStateSnapshot snapshot;
    snapshot.connected = true;
    snapshot.armed = true;
    snapshot.base_mode = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    snapshot.custom_mode = 0x00040000U;
    snapshot.system_status = MAV_STATE_ACTIVE;
    snapshot.landed_state_valid = true;
    snapshot.landed = false;
    snapshot.attitude_valid = true;
    snapshot.roll_rad = 0.1f;
    snapshot.pitch_rad = -0.2f;
    snapshot.yaw_rad = 1.0f;
    snapshot.local_position_valid = true;
    snapshot.local_x_m = 1.f;
    snapshot.local_y_m = 2.f;
    snapshot.local_z_m = -3.f;
    snapshot.vx_mps = 4.f;
    snapshot.vy_mps = 5.f;
    snapshot.vz_mps = -0.5f;
    snapshot.gps_state_valid = true;
    snapshot.gps_fix = true;
    snapshot.gps_fix_type = GPS_FIX_TYPE_3D_FIX;
    snapshot.global_position_valid = true;
    snapshot.latitude_1e7 = 231234567;
    snapshot.longitude_1e7 = 1131234567;
    snapshot.altitude_mm = 123400;
    snapshot.home_valid = true;
    snapshot.home_lat_1e7 = 231230000;
    snapshot.home_lon_1e7 = 1131230000;
    snapshot.home_altitude_mm = 120000;
    snapshot.battery_valid = true;
    snapshot.battery_voltage_v = 15.2f;
    snapshot.battery_current_valid = true;
    snapshot.battery_current_a = 3.4f;
    snapshot.battery_remaining_valid = true;
    snapshot.battery_remaining_pct = 76.f;
    ASSERT_TRUE(flight_state.Publish(
        std::make_shared<const common::FlightStateSnapshot>(snapshot)).accepted);

    mavlink_message_t global_message{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& message) {
            return message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
        }, 500ms, &global_message));
    EXPECT_EQ(global_message.sysid, 1);
    EXPECT_EQ(global_message.compid, kNetCaptureAircraftComponentId);
    mavlink_global_position_int_t global{};
    mavlink_msg_global_position_int_decode(&global_message, &global);
    EXPECT_EQ(global.lat, snapshot.latitude_1e7);
    EXPECT_EQ(global.lon, snapshot.longitude_1e7);
    EXPECT_EQ(global.alt, snapshot.altitude_mm);
    EXPECT_EQ(global.relative_alt, 3400);
    EXPECT_EQ(global.vx, 400);

    mavlink_message_t battery_message{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& message) {
            return message.msgid == MAVLINK_MSG_ID_SYS_STATUS;
        }, 500ms, &battery_message));
    EXPECT_EQ(battery_message.sysid, 1);
    EXPECT_EQ(battery_message.compid, kNetCaptureAircraftComponentId);
    mavlink_sys_status_t system_status{};
    mavlink_msg_sys_status_decode(&battery_message, &system_status);
    EXPECT_EQ(system_status.voltage_battery, 15200);
    EXPECT_EQ(system_status.current_battery, 340);
    EXPECT_EQ(system_status.battery_remaining, 76);
    EXPECT_GT(link.SendCount(), 0u);

    link.Stop();
}

TEST(GroundStationLinkTest, TracksOnlyConfiguredGcsHeartbeatAndTimeout) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    ASSERT_TRUE(terminal.Write(EncodeGcsHeartbeat(254, kGroundStationComponentId)));
    EXPECT_FALSE(WaitUntil([&] { return link.IsConnected(); }, 120ms));

    ASSERT_TRUE(terminal.Write(EncodeGcsHeartbeat()));
    ASSERT_TRUE(WaitUntil([&] { return link.IsConnected(); }, 200ms));
    EXPECT_TRUE(WaitUntil([&] { return !link.IsConnected(); }, 300ms));
    EXPECT_GT(link.ReceiveCount(), 0u);

    link.Stop();
}

TEST(GroundStationLinkTest, EstablishesTimeSyncAndExpiresWithoutResponses) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    auto config = MakeConfig(terminal.SlaveName());
    config.heartbeat_timeout = 500ms;
    GroundStationLink link(config);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    ASSERT_TRUE(terminal.Write(EncodeGcsHeartbeat()));
    ASSERT_TRUE(WaitUntil([&] { return link.IsConnected(); }, 200ms));

    constexpr int64_t kGroundClockOffsetNs = 40LL * 1000LL * 1000LL;
    for (int index = 0; index < 3; ++index) {
        mavlink_message_t request{};
        ASSERT_TRUE(terminal.WaitFor(
            [](const mavlink_message_t& message) {
                return message.msgid == MAVLINK_MSG_ID_TIMESYNC;
            }, 300ms, &request));
        mavlink_timesync_t payload{};
        mavlink_msg_timesync_decode(&request, &payload);
        EXPECT_EQ(request.sysid, 1);
        EXPECT_EQ(request.compid, kNetCaptureAircraftComponentId);
        EXPECT_EQ(payload.tc1, 0);
        EXPECT_EQ(payload.target_system, kGroundStationSystemId);
        EXPECT_EQ(payload.target_component, kGroundStationComponentId);
        ASSERT_TRUE(terminal.Write(
            EncodeTimeSyncResponse(request, kGroundClockOffsetNs)));
    }

    ASSERT_TRUE(WaitUntil(
        [&] {
            return link.GetTimeSyncStatus().state ==
                   GroundStationTimeSyncState::kSynchronized;
        }, 300ms));
    const auto synchronized = link.GetTimeSyncStatus();
    EXPECT_EQ(synchronized.valid_sample_count, 3u);
    EXPECT_EQ(synchronized.response_count, 3u);
    EXPECT_GE(synchronized.request_count, 3u);
    EXPECT_LT(synchronized.round_trip_time_ns, 40ULL * 1000ULL * 1000ULL);
    EXPECT_NEAR(static_cast<double>(synchronized.offset_ns) / 1000000.0,
                40.0, 10.0);

    EXPECT_TRUE(WaitUntil(
        [&] {
            return link.GetTimeSyncStatus().state !=
                   GroundStationTimeSyncState::kSynchronized;
        }, 300ms));
    link.Stop();
}

TEST(GroundStationLinkTest, OffsetJumpImmediatelyDegradesTimeSync) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    auto config = MakeConfig(terminal.SlaveName());
    config.heartbeat_timeout = 500ms;
    GroundStationLink link(config);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    ASSERT_TRUE(terminal.Write(EncodeGcsHeartbeat()));
    ASSERT_TRUE(WaitUntil([&] { return link.IsConnected(); }, 200ms));

    constexpr int64_t kInitialOffsetNs = 30LL * 1000LL * 1000LL;
    for (int index = 0; index < 3; ++index) {
        mavlink_message_t request{};
        ASSERT_TRUE(terminal.WaitFor(
            [](const mavlink_message_t& message) {
                return message.msgid == MAVLINK_MSG_ID_TIMESYNC;
            }, 300ms, &request));
        ASSERT_TRUE(terminal.Write(
            EncodeTimeSyncResponse(request, kInitialOffsetNs)));
    }
    ASSERT_TRUE(WaitUntil(
        [&] {
            return link.GetTimeSyncStatus().state ==
                   GroundStationTimeSyncState::kSynchronized;
        }, 300ms));

    mavlink_message_t jump_request{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& message) {
            return message.msgid == MAVLINK_MSG_ID_TIMESYNC;
        }, 300ms, &jump_request));
    constexpr int64_t kJumpedOffsetNs = 200LL * 1000LL * 1000LL;
    ASSERT_TRUE(terminal.Write(
        EncodeTimeSyncResponse(jump_request, kJumpedOffsetNs)));

    ASSERT_TRUE(WaitUntil(
        [&] {
            return link.GetTimeSyncStatus().state ==
                   GroundStationTimeSyncState::kDegraded;
        }, 200ms));
    const auto degraded = link.GetTimeSyncStatus();
    EXPECT_EQ(degraded.valid_sample_count, 1u);
    EXPECT_GT(degraded.rejected_sample_count, 0u);
    link.Stop();
}

TEST(GroundStationLinkTest, IgnoresTimeSyncResponseWithWrongTarget) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    ASSERT_TRUE(terminal.Write(EncodeGcsHeartbeat()));
    ASSERT_TRUE(WaitUntil([&] { return link.IsConnected(); }, 200ms));

    mavlink_message_t request{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& message) {
            return message.msgid == MAVLINK_MSG_ID_TIMESYNC;
        }, 300ms, &request));
    ASSERT_TRUE(terminal.Write(EncodeTimeSyncResponse(
        request, 0, kGroundStationSystemId, kGroundStationComponentId,
        2, kNetCaptureAircraftComponentId)));
    std::this_thread::sleep_for(30ms);

    const auto status = link.GetTimeSyncStatus();
    EXPECT_EQ(status.response_count, 0u);
    EXPECT_GT(status.rejected_sample_count, 0u);
    EXPECT_NE(status.state, GroundStationTimeSyncState::kSynchronized);
    link.Stop();
}

TEST(GroundStationLinkTest, RespondsToAddressedGroundStationTimeSyncRequest) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    constexpr int64_t kGroundRequestTimeNs = 123456789LL;
    MavlinkHandler encoder(MavlinkVersion::kV2);
    const auto request = encoder.Encode([](mavlink_status_t* status,
                                           mavlink_message_t* message) {
        return mavlink_msg_timesync_pack_status(
            kGroundStationSystemId, kGroundStationComponentId, status, message,
            0, kGroundRequestTimeNs, 1, kNetCaptureAircraftComponentId);
    });
    ASSERT_TRUE(terminal.Write(request));

    mavlink_message_t response{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& message) {
            if (message.msgid != MAVLINK_MSG_ID_TIMESYNC) {
                return false;
            }
            mavlink_timesync_t payload{};
            mavlink_msg_timesync_decode(&message, &payload);
            return payload.tc1 != 0 && payload.ts1 == kGroundRequestTimeNs;
        }, 300ms, &response));
    mavlink_timesync_t payload{};
    mavlink_msg_timesync_decode(&response, &payload);
    EXPECT_EQ(payload.target_system, kGroundStationSystemId);
    EXPECT_EQ(payload.target_component, kGroundStationComponentId);
    link.Stop();
}

TEST(GroundStationLinkTest, StopAndRestartAreIdempotent) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    link.Stop();
    link.Stop();
    EXPECT_FALSE(link.IsRunning());
    EXPECT_TRUE(link.Start());
    EXPECT_TRUE(link.IsRunning());
    link.Stop();
}

}  // namespace
}  // namespace drone::communication
