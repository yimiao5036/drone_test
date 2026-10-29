#include "communication/ground_station_link.h"

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "communication/mavlink_handler.h"

namespace drone::communication {
namespace {

using namespace std::chrono_literals;

uint64_t MonotonicMs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

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

    // 主动关闭主端：从端（链路打开的 /dev/pts/N）随后的读操作会返回错误，
    // 用于模拟串口断线并验证链路的重连状态机。
    void CloseMaster() {
        if (master_fd_ >= 0) {
            ::close(master_fd_);
            master_fd_ = -1;
        }
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

template <typename T>
void WriteLe(std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN>& payload,
             std::size_t offset, T value) {
    std::memcpy(payload.data() + offset, &value, sizeof(T));
}

template <typename T>
T ReadLe(const uint8_t* payload, std::size_t offset) {
    T value{};
    std::memcpy(&value, payload + offset, sizeof(T));
    return value;
}

struct TargetUpdateFixture {
    uint64_t source_time_ms = 0;
    uint32_t boot_id = 42;
    uint32_t update_seq = 1;
    uint32_t target_id = 7;
    uint32_t valid_for_ms = 1000;
    int32_t latitude_1e7 = 231230000;
    int32_t longitude_1e7 = 1131230000;
    int32_t altitude_mm = 120000;
    int16_t velocity_north_cms = 120;
    int16_t velocity_east_cms = -50;
    int16_t velocity_down_cms = 0;
    uint16_t heading_cd = 9000;
    uint16_t horizontal_accuracy_cm = 150;
    uint16_t vertical_accuracy_cm = 250;
    uint16_t flags = 0x007FU;
    uint8_t target_system = 1;
    uint8_t target_component = kNetCaptureAircraftComponentId;
    uint8_t coordinate_frame = kTrackTargetCoordinateFrameWgs84;
    uint8_t protocol_version = kTrackTargetProtocolVersion;
    uint8_t alt_reference = 1;
};

// 65014 MISSION_ASSIGN 测试编码参数；默认是发给本机 1/25 的合法授权。
struct MissionAssignFixture {
    uint8_t protocol_version = kMissionAssignmentProtocolVersion;
    uint32_t assignment_id = 1001;
    uint32_t target_id = 7;
    uint8_t interceptor_type = kNetCaptureAircraftComponentId;
    uint8_t flags = kMissionAssignFlagAuthorized;
    uint32_t valid_for_ms = 5000;
    uint8_t target_system = 1;
    uint8_t target_component = kNetCaptureAircraftComponentId;
    uint8_t frame_length_marker = 1;
};

// protocol_length 用于构造截断短帧：超出该长度的协议字节全部清零，
// MAVLink2 尾部零字节截断后 message.len 会小于 60。
std::vector<uint8_t> EncodeMissionAssign(
    const MissionAssignFixture& assignment, std::size_t protocol_length = 55,
    uint8_t header_system = 1,
    uint8_t header_component = kNetCaptureAircraftComponentId) {
    std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
    WriteLe<uint8_t>(payload, 0, assignment.protocol_version);
    WriteLe<uint32_t>(payload, 1, assignment.assignment_id);
    WriteLe<uint32_t>(payload, 5, assignment.target_id);
    WriteLe<uint8_t>(payload, 9, assignment.interceptor_type);
    WriteLe<uint8_t>(payload, 10, assignment.flags);
    WriteLe<uint32_t>(payload, 11, assignment.valid_for_ms);
    WriteLe<uint8_t>(payload, 50, assignment.target_system);
    WriteLe<uint8_t>(payload, 51, assignment.target_component);
    WriteLe<uint8_t>(payload, 54, assignment.frame_length_marker);
    if (protocol_length < payload.size()) {
        std::fill(payload.begin() + protocol_length, payload.end(), 0);
    }
    MavlinkHandler encoder(MavlinkVersion::kV2);
    return encoder.Encode([&payload, header_system, header_component](
                              mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_v2_extension_pack_status(
            kGroundStationSystemId, kGroundStationComponentId, status, message, 0,
            header_system, header_component, kMissionAssignMessageType,
            payload.data());
    });
}

struct MissionAssignAckMessage {
    mavlink_message_t message{};
    mavlink_v2_extension_t extension{};
};

std::optional<MissionAssignAckMessage> ReadMissionAssignAck(
    PseudoTerminal& terminal) {
    mavlink_message_t ack{};
    if (!terminal.WaitFor(
            [](const mavlink_message_t& message) {
                if (message.msgid != MAVLINK_MSG_ID_V2_EXTENSION) {
                    return false;
                }
                mavlink_v2_extension_t extension{};
                mavlink_msg_v2_extension_decode(&message, &extension);
                return extension.message_type == kMissionAssignAckMessageType;
            }, 300ms, &ack)) {
        return std::nullopt;
    }
    MissionAssignAckMessage result;
    result.message = ack;
    mavlink_msg_v2_extension_decode(&ack, &result.extension);
    return result;
}

// protocol_length 用于构造截断短帧：超出该长度的协议字节全部清零，
// MAVLink2 尾部零字节截断后 message.len 会小于协议要求的最小帧长。
std::vector<uint8_t> EncodeTrackTargetUpdate(const TargetUpdateFixture& update,
                                             std::size_t protocol_length = 55) {
    std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
    WriteLe<uint64_t>(payload, 0, update.source_time_ms);
    WriteLe<uint32_t>(payload, 8, update.boot_id);
    WriteLe<uint32_t>(payload, 12, update.update_seq);
    WriteLe<uint32_t>(payload, 16, update.target_id);
    WriteLe<uint32_t>(payload, 20, update.valid_for_ms);
    WriteLe<int32_t>(payload, 24, update.latitude_1e7);
    WriteLe<int32_t>(payload, 28, update.longitude_1e7);
    WriteLe<int32_t>(payload, 32, update.altitude_mm);
    WriteLe<int16_t>(payload, 36, update.velocity_north_cms);
    WriteLe<int16_t>(payload, 38, update.velocity_east_cms);
    WriteLe<int16_t>(payload, 40, update.velocity_down_cms);
    WriteLe<uint16_t>(payload, 42, update.heading_cd);
    WriteLe<uint16_t>(payload, 44, update.horizontal_accuracy_cm);
    WriteLe<uint16_t>(payload, 46, update.vertical_accuracy_cm);
    WriteLe<uint16_t>(payload, 48, update.flags);
    WriteLe<uint8_t>(payload, 50, update.target_system);
    WriteLe<uint8_t>(payload, 51, update.target_component);
    WriteLe<uint8_t>(payload, 52, update.coordinate_frame);
    WriteLe<uint8_t>(payload, 53, update.protocol_version);
    WriteLe<uint8_t>(payload, 54, update.alt_reference);
    if (protocol_length < payload.size()) {
        std::fill(payload.begin() + static_cast<std::ptrdiff_t>(protocol_length),
                  payload.end(), 0);
    }
    MavlinkHandler encoder(MavlinkVersion::kV2);
    return encoder.Encode([&payload, &update](mavlink_status_t* status,
                                              mavlink_message_t* message) {
        return mavlink_msg_v2_extension_pack_status(
            kGroundStationSystemId, kGroundStationComponentId, status, message, 0,
            update.target_system, update.target_component,
            kTrackTargetUpdateMessageType, payload.data());
    });
}

// 65018 FENCE_ZONE 测试编码参数；默认是发给本机 1/25 的合法多边形区域。
struct FenceZoneFixture {
    uint8_t protocol_version = kFenceZoneProtocolVersion;
    uint16_t zone_id = 3;
    uint16_t version = 1;
    uint8_t type = 0;
    uint8_t action = 1;
    uint8_t altitude_reference = 1;
    int16_t alt_min_dm = 100;
    int16_t alt_max_dm = 500;
    uint8_t vertex_count = 4;
    int32_t ref_lat_1e7 = 231230000;
    int32_t ref_lon_1e7 = 1131230000;
    uint8_t frame_length_marker = 1;
};

std::vector<uint8_t> EncodeFenceZone(
    const FenceZoneFixture& zone, uint8_t header_system = 1,
    uint8_t header_component = kNetCaptureAircraftComponentId) {
    std::array<uint8_t, MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN> payload{};
    WriteLe<uint8_t>(payload, 0, zone.protocol_version);
    WriteLe<uint16_t>(payload, 1, zone.zone_id);
    WriteLe<uint16_t>(payload, 3, zone.version);
    WriteLe<uint8_t>(payload, 5, zone.type);
    WriteLe<uint8_t>(payload, 6, zone.action);
    WriteLe<uint8_t>(payload, 7, zone.altitude_reference);
    WriteLe<int16_t>(payload, 8, zone.alt_min_dm);
    WriteLe<int16_t>(payload, 10, zone.alt_max_dm);
    WriteLe<uint8_t>(payload, 12, zone.vertex_count);
    WriteLe<int32_t>(payload, 13, zone.ref_lat_1e7);
    WriteLe<int32_t>(payload, 17, zone.ref_lon_1e7);
    // 顶点偏移全部取 0：全部落在参考点上，便于断言区域已被接受。
    for (std::size_t index = 0; index < zone.vertex_count; ++index) {
        WriteLe<int16_t>(payload, 21U + index * 4U, 0);
        WriteLe<int16_t>(payload, 23U + index * 4U, 0);
    }
    WriteLe<uint8_t>(payload, 54, zone.frame_length_marker);
    MavlinkHandler encoder(MavlinkVersion::kV2);
    return encoder.Encode([&payload, header_system, header_component](
                              mavlink_status_t* status, mavlink_message_t* message) {
        return mavlink_msg_v2_extension_pack_status(
            kGroundStationSystemId, kGroundStationComponentId, status, message, 0,
            header_system, header_component, kFenceZoneMessageType,
            payload.data());
    });
}

void EstablishTimeSync(PseudoTerminal& terminal, GroundStationLink& link,
                       int64_t offset_ns = 40LL * 1000LL * 1000LL) {
    ASSERT_TRUE(terminal.Write(EncodeGcsHeartbeat()));
    ASSERT_TRUE(WaitUntil([&] { return link.IsConnected(); }, 200ms));
    for (int index = 0; index < 3; ++index) {
        mavlink_message_t request{};
        ASSERT_TRUE(terminal.WaitFor(
            [](const mavlink_message_t& message) {
                return message.msgid == MAVLINK_MSG_ID_TIMESYNC;
            }, 300ms, &request));
        ASSERT_TRUE(terminal.Write(EncodeTimeSyncResponse(request, offset_ns)));
    }
    ASSERT_TRUE(WaitUntil(
        [&] {
            return link.GetTimeSyncStatus().state ==
                   GroundStationTimeSyncState::kSynchronized;
        }, 300ms));
}

std::optional<std::tuple<TrackTargetAckResult, TrackTargetAckReason, uint32_t>>
ReadTrackTargetAck(PseudoTerminal& terminal) {
    mavlink_message_t ack{};
    if (!terminal.WaitFor(
            [](const mavlink_message_t& message) {
                if (message.msgid != MAVLINK_MSG_ID_V2_EXTENSION) {
                    return false;
                }
                mavlink_v2_extension_t extension{};
                mavlink_msg_v2_extension_decode(&message, &extension);
                return extension.message_type == kTrackTargetAckMessageType;
            }, 300ms, &ack)) {
        return std::nullopt;
    }
    mavlink_v2_extension_t extension{};
    mavlink_msg_v2_extension_decode(&ack, &extension);
    return std::make_tuple(
        static_cast<TrackTargetAckResult>(ReadLe<uint8_t>(extension.payload, 22)),
        static_cast<TrackTargetAckReason>(ReadLe<uint16_t>(extension.payload, 20)),
        ReadLe<uint32_t>(extension.payload, 12));
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
    snapshot.gps_satellites_valid = true;
    snapshot.gps_satellites_visible = 17;
    snapshot.gps_accuracy_valid = true;
    snapshot.gps_eph_m = 1.25f;
    snapshot.gps_epv_m = 2.5f;
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

    mavlink_message_t gps_message{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& message) {
            return message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT;
        }, 500ms, &gps_message));
    mavlink_gps_raw_int_t gps{};
    mavlink_msg_gps_raw_int_decode(&gps_message, &gps);
    EXPECT_EQ(gps.fix_type, GPS_FIX_TYPE_3D_FIX);
    EXPECT_EQ(gps.satellites_visible, 17);
    EXPECT_EQ(gps.eph, 125);
    EXPECT_EQ(gps.epv, 250);

    link.Stop();
}

TEST(GroundStationLinkTest, GpsDownlinkKeepsUnknownSatellitesAndAccuracyInvalid) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    common::FlightStateSnapshot snapshot;
    snapshot.connected = true;
    snapshot.gps_state_valid = true;
    snapshot.gps_fix = true;
    snapshot.gps_fix_type = GPS_FIX_TYPE_3D_FIX;
    // 卫星数/精度未知：保持 valid 标志为假
    ASSERT_TRUE(flight_state.Publish(
        std::make_shared<const common::FlightStateSnapshot>(snapshot)).accepted);

    mavlink_message_t gps_message{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& message) {
            return message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT;
        }, 500ms, &gps_message));
    mavlink_gps_raw_int_t gps{};
    mavlink_msg_gps_raw_int_decode(&gps_message, &gps);
    EXPECT_EQ(gps.satellites_visible, UINT8_MAX);
    EXPECT_EQ(gps.eph, UINT16_MAX);
    EXPECT_EQ(gps.epv, UINT16_MAX);

    link.Stop();
}

TEST(GroundStationLinkTest, EncodesHealthAndMissionStatusExtensions) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    common::Topic<common::HealthStatus> health_status;
    common::Topic<common::MissionStatus> mission_status;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    link.SetFlightStateInput(flight_state);
    link.SetHealthInput(health_status);
    link.SetMissionStatusInput(mission_status);
    ASSERT_TRUE(link.Start());

    common::HealthStatus health;
    health.header.sequence = 42;
    health.header.receive_time_ms = 123456;
    health.link_health_bits = 0x13;
    health.device_health_bits = 0x03;
    health.data_freshness_bits = 0x04;
    health.error_bits = 0x80;
    health.cpu_load_pct = 12.34f;
    health.timeout_event_count = 7;
    ASSERT_TRUE(health_status.Publish(
        std::make_shared<const common::HealthStatus>(health)).accepted);

    mavlink_message_t health_message{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& message) {
            if (message.msgid != MAVLINK_MSG_ID_V2_EXTENSION) {
                return false;
            }
            mavlink_v2_extension_t extension{};
            mavlink_msg_v2_extension_decode(&message, &extension);
            return extension.message_type == kHealthStatusMessageType;
        }, 300ms, &health_message));
    mavlink_v2_extension_t health_extension{};
    mavlink_msg_v2_extension_decode(&health_message, &health_extension);
    EXPECT_EQ(health_message.len, 60);
    EXPECT_EQ(health_message.sysid, 1);
    EXPECT_EQ(health_message.compid, kNetCaptureAircraftComponentId);
    // 65012 保持 v1，版本源与 65013 独立。
    EXPECT_EQ(ReadLe<uint8_t>(health_extension.payload, 0), kHealthStatusProtocolVersion);
    EXPECT_EQ(ReadLe<uint32_t>(health_extension.payload, 1), 42U);
    EXPECT_EQ(ReadLe<uint32_t>(health_extension.payload, 5), 0x13U);
    EXPECT_EQ(ReadLe<uint32_t>(health_extension.payload, 9), 0x03U);
    EXPECT_EQ(ReadLe<uint32_t>(health_extension.payload, 13), 0x04U);
    EXPECT_EQ(ReadLe<uint32_t>(health_extension.payload, 17), 0x80U);
    EXPECT_EQ(ReadLe<uint32_t>(health_extension.payload, 21), 1234U);
    EXPECT_EQ(ReadLe<uint32_t>(health_extension.payload, 25), 7U);
    EXPECT_EQ(ReadLe<uint8_t>(health_extension.payload, 31), 0x01U);
    EXPECT_EQ(ReadLe<uint64_t>(health_extension.payload, 32), 123456U);

    health.header.sequence = 43;
    health.cpu_load_pct = std::numeric_limits<float>::quiet_NaN();
    ASSERT_TRUE(health_status.Publish(
        std::make_shared<const common::HealthStatus>(health)).accepted);
    mavlink_message_t invalid_cpu_message{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& message) {
            if (message.msgid != MAVLINK_MSG_ID_V2_EXTENSION) {
                return false;
            }
            mavlink_v2_extension_t extension{};
            mavlink_msg_v2_extension_decode(&message, &extension);
            return extension.message_type == kHealthStatusMessageType &&
                   ReadLe<uint32_t>(extension.payload, 1) == 43U;
        }, 300ms, &invalid_cpu_message));
    mavlink_v2_extension_t invalid_cpu_extension{};
    mavlink_msg_v2_extension_decode(&invalid_cpu_message, &invalid_cpu_extension);
    EXPECT_EQ(ReadLe<uint8_t>(invalid_cpu_extension.payload, 31), 0x00U);
    EXPECT_EQ(ReadLe<uint32_t>(invalid_cpu_extension.payload, 21), 0U);

    common::MissionStatus mission;
    mission.header.sequence = 9;
    mission.header.receive_time_ms = 234567;
    mission.state = common::MissionState::kGpsApproach;
    mission.control_source = 2;
    mission.task_phase = 3;
    mission.active_warning_bits = 0x22;
    mission.state_entered_ms = 200000;
    mission.front_distance_m = 12.345f;
    mission.interception_authorized = true;
    mission.power_status_bits = 0x03;
    mission.assignment_id = 1001;
    mission.target_id = 7;
    mission.availability = common::MissionAvailability::kAssigned;
    mission.interceptor_type = kNetCaptureAircraftComponentId;
    ASSERT_TRUE(mission_status.Publish(
        std::make_shared<const common::MissionStatus>(mission)).accepted);

    mavlink_message_t mission_message{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& message) {
            if (message.msgid != MAVLINK_MSG_ID_V2_EXTENSION) {
                return false;
            }
            mavlink_v2_extension_t extension{};
            mavlink_msg_v2_extension_decode(&message, &extension);
            return extension.message_type == kMissionStatusMessageType;
        }, 300ms, &mission_message));
    mavlink_v2_extension_t mission_extension{};
    mavlink_msg_v2_extension_decode(&mission_message, &mission_extension);
    EXPECT_EQ(mission_message.len, 60);
    // 65013 升级到 v2：旧字段 offset 0~39 不变，扩展字段复用原保留区。
    EXPECT_EQ(ReadLe<uint8_t>(mission_extension.payload, 0),
              kMissionStatusProtocolVersion);
    EXPECT_EQ(ReadLe<uint32_t>(mission_extension.payload, 1), 9U);
    EXPECT_EQ(ReadLe<uint8_t>(mission_extension.payload, 5),
              static_cast<uint8_t>(common::MissionState::kGpsApproach));
    EXPECT_EQ(ReadLe<uint8_t>(mission_extension.payload, 6), 2U);
    EXPECT_EQ(ReadLe<uint8_t>(mission_extension.payload, 7), 3U);
    EXPECT_EQ(ReadLe<uint32_t>(mission_extension.payload, 8), 0x22U);
    EXPECT_EQ(ReadLe<uint64_t>(mission_extension.payload, 12), 200000U);
    EXPECT_EQ(ReadLe<int32_t>(mission_extension.payload, 20), 12345);
    EXPECT_EQ(ReadLe<uint8_t>(mission_extension.payload, 24), 1U);
    EXPECT_EQ(ReadLe<uint8_t>(mission_extension.payload, 25), 3U);
    EXPECT_EQ(ReadLe<uint64_t>(mission_extension.payload, 32), 234567U);
    EXPECT_EQ(ReadLe<uint32_t>(mission_extension.payload, 40), 1001U);
    EXPECT_EQ(ReadLe<uint32_t>(mission_extension.payload, 44), 7U);
    EXPECT_EQ(ReadLe<uint8_t>(mission_extension.payload, 48),
              static_cast<uint8_t>(common::MissionAvailability::kAssigned));
    EXPECT_EQ(ReadLe<uint8_t>(mission_extension.payload, 49),
              kNetCaptureAircraftComponentId);
    EXPECT_EQ(ReadLe<uint8_t>(mission_extension.payload, 54), 1U);

    link.Stop();
}

TEST(GroundStationLinkTest, PublishesMissionAssignmentForMatchingAddress) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto assignment_subscription =
        link.MissionAssignmentOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    MissionAssignFixture assignment;
    ASSERT_TRUE(terminal.Write(EncodeMissionAssign(assignment)));

    auto message = assignment_subscription.WaitTakeFor(300ms);
    ASSERT_TRUE(message.has_value());
    const auto& received = **message;
    EXPECT_EQ(received.protocol_version, kMissionAssignmentProtocolVersion);
    EXPECT_EQ(received.assignment_id, assignment.assignment_id);
    EXPECT_EQ(received.target_id, assignment.target_id);
    EXPECT_EQ(received.interceptor_type, assignment.interceptor_type);
    EXPECT_EQ(received.flags, assignment.flags);
    EXPECT_EQ(received.valid_for_ms, assignment.valid_for_ms);
    EXPECT_EQ(received.header.source_id, kGroundStationSystemId);
    EXPECT_EQ(received.header.valid_for_ms, assignment.valid_for_ms);
    EXPECT_GT(received.header.receive_time_ms, 0u);
    EXPECT_EQ(received.received_ms, received.header.receive_time_ms);
    EXPECT_GT(received.header.sequence, 0u);

    // 链路层不自动回 65015；必须由状态机通过 ACK Topic 触发。
    EXPECT_FALSE(ReadMissionAssignAck(terminal).has_value());
    link.Stop();
}

TEST(GroundStationLinkTest, MissionAssignmentHeaderAddressMismatchIsIgnored) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto assignment_subscription =
        link.MissionAssignmentOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    MissionAssignFixture assignment;
    // payload 指向本机 1/25，但 V2_EXTENSION 帧头指向 2/25。
    ASSERT_TRUE(terminal.Write(EncodeMissionAssign(assignment, 55, 2,
                                                   kNetCaptureAircraftComponentId)));

    EXPECT_FALSE(assignment_subscription.WaitTakeFor(100ms).has_value());
    EXPECT_FALSE(ReadMissionAssignAck(terminal).has_value());
    link.Stop();
}

TEST(GroundStationLinkTest, MissionAssignmentPayloadAddressMismatchIsIgnored) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto assignment_subscription =
        link.MissionAssignmentOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    MissionAssignFixture assignment;
    // 帧头指向本机 1/25，但 payload 地址指向 2/25。
    assignment.target_system = 2;
    ASSERT_TRUE(terminal.Write(EncodeMissionAssign(assignment)));

    EXPECT_FALSE(assignment_subscription.WaitTakeFor(100ms).has_value());
    EXPECT_FALSE(ReadMissionAssignAck(terminal).has_value());
    link.Stop();
}

TEST(GroundStationLinkTest, MissionAssignmentPublishesSemanticFieldsUnchanged) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto assignment_subscription =
        link.MissionAssignmentOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    // 地址匹配后，版本/拦截类型/flags/target_id 的语义校验属于状态机；
    // 链路层必须原样发布，不能自行丢弃。
    MissionAssignFixture assignment;
    assignment.protocol_version = 9;
    assignment.target_id = 0;
    assignment.interceptor_type = 99;
    assignment.flags = 0x80;
    ASSERT_TRUE(terminal.Write(EncodeMissionAssign(assignment)));

    auto message = assignment_subscription.WaitTakeFor(300ms);
    ASSERT_TRUE(message.has_value());
    EXPECT_EQ((**message).protocol_version, 9U);
    EXPECT_EQ((**message).target_id, 0U);
    EXPECT_EQ((**message).interceptor_type, 99U);
    EXPECT_EQ((**message).flags, 0x80U);
    link.Stop();
}

TEST(GroundStationLinkTest, TruncatedMissionAssignmentIsIgnored) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto assignment_subscription =
        link.MissionAssignmentOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    // 只编码前 12 字节，MAVLink2 截断后 message.len < 60，不得解析或崩溃。
    MissionAssignFixture assignment;
    ASSERT_TRUE(terminal.Write(EncodeMissionAssign(assignment, 12)));

    EXPECT_FALSE(assignment_subscription.WaitTakeFor(100ms).has_value());
    EXPECT_FALSE(ReadMissionAssignAck(terminal).has_value());

    // 畸形短帧之后仍能正常处理合法分配。
    ASSERT_TRUE(terminal.Write(EncodeMissionAssign(assignment)));
    EXPECT_TRUE(assignment_subscription.WaitTakeFor(300ms).has_value());
    link.Stop();
}

TEST(GroundStationLinkTest, SendsMissionAssignmentAckToGroundStation) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    common::Topic<common::MissionAssignmentAck> ack_input;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    link.SetFlightStateInput(flight_state);
    link.SetMissionAssignmentAckInput(ack_input);
    ASSERT_TRUE(link.Start());

    common::MissionAssignmentAck ack;
    ack.assignment_id = 1001;
    ack.target_id = 7;
    ack.result = common::MissionAssignmentResult::kAccepted;
    ack.reason = common::MissionAssignmentReason::kNone;
    ack.mission_state = common::MissionState::kArming;
    ack.availability = common::MissionAvailability::kAssigned;
    ASSERT_TRUE(ack_input.Publish(
        std::make_shared<const common::MissionAssignmentAck>(ack)).accepted);

    const auto sent = ReadMissionAssignAck(terminal);
    ASSERT_TRUE(sent.has_value());
    EXPECT_EQ(sent->message.len, 60);
    EXPECT_EQ(sent->message.sysid, 1);
    EXPECT_EQ(sent->message.compid, kNetCaptureAircraftComponentId);
    EXPECT_EQ(sent->extension.target_system, kGroundStationSystemId);
    EXPECT_EQ(sent->extension.target_component, kGroundStationComponentId);
    EXPECT_EQ(ReadLe<uint8_t>(sent->extension.payload, 0),
              kMissionAssignmentProtocolVersion);
    EXPECT_EQ(ReadLe<uint32_t>(sent->extension.payload, 1), 1001U);
    EXPECT_EQ(ReadLe<uint32_t>(sent->extension.payload, 5), 7U);
    EXPECT_EQ(ReadLe<uint8_t>(sent->extension.payload, 9),
              static_cast<uint8_t>(common::MissionAssignmentResult::kAccepted));
    EXPECT_EQ(ReadLe<uint8_t>(sent->extension.payload, 10),
              static_cast<uint8_t>(common::MissionAssignmentReason::kNone));
    EXPECT_EQ(ReadLe<uint8_t>(sent->extension.payload, 11),
              static_cast<uint8_t>(common::MissionState::kArming));
    EXPECT_EQ(ReadLe<uint8_t>(sent->extension.payload, 12),
              static_cast<uint8_t>(common::MissionAvailability::kAssigned));
    EXPECT_EQ(ReadLe<uint8_t>(sent->extension.payload, 26), 1U);
    EXPECT_EQ(ReadLe<uint8_t>(sent->extension.payload, 27),
              kNetCaptureAircraftComponentId);
    EXPECT_EQ(ReadLe<uint8_t>(sent->extension.payload, 54), 1U);
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

TEST(GroundStationLinkTest, PublishesTrackTargetUpdateAndSendsAck) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto target_subscription = link.TargetOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    EstablishTimeSync(terminal, link);

    const int64_t offset_ms = link.GetTimeSyncStatus().offset_ns / 1000000LL;
    TargetUpdateFixture update;
    update.source_time_ms = MonotonicMs() + offset_ms - 100;
    ASSERT_TRUE(terminal.Write(EncodeTrackTargetUpdate(update)));

    auto target = target_subscription.WaitTakeFor(300ms);
    ASSERT_TRUE(target.has_value());
    EXPECT_EQ((**target).ground_station_boot_id, update.boot_id);
    EXPECT_EQ((**target).update_seq, update.update_seq);
    EXPECT_EQ((**target).target_id, update.target_id);
    EXPECT_EQ((**target).latitude_1e7, update.latitude_1e7);
    EXPECT_EQ((**target).longitude_1e7, update.longitude_1e7);
    EXPECT_EQ((**target).altitude_mm, update.altitude_mm);
    EXPECT_EQ((**target).alt_reference, update.alt_reference);
    EXPECT_EQ((**target).protocol_version, kTrackTargetProtocolVersion);
    EXPECT_EQ((**target).validity_flags, update.flags);
    EXPECT_NEAR((**target).velocity_north_mps, 1.20f, 1e-3f);
    EXPECT_NEAR((**target).velocity_east_mps, -0.50f, 1e-3f);
    EXPECT_NEAR((**target).heading_deg, 90.0f, 1e-3f);
    EXPECT_GE((**target).transport_age_ms, 0u);
    EXPECT_LT((**target).header.valid_for_ms, update.valid_for_ms);

    const auto ack = ReadTrackTargetAck(terminal);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(std::get<0>(*ack), TrackTargetAckResult::kAccepted);
    EXPECT_EQ(std::get<1>(*ack), TrackTargetAckReason::kOk);
    EXPECT_GE(std::get<2>(*ack), 0u);
    link.Stop();
}

TEST(GroundStationLinkTest, TrackTargetWrongAddressIsIgnoredWithoutAck) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto target_subscription = link.TargetOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    EstablishTimeSync(terminal, link);

    TargetUpdateFixture update;
    update.target_system = 2;
    update.source_time_ms = MonotonicMs() + link.GetTimeSyncStatus().offset_ns / 1000000LL;
    ASSERT_TRUE(terminal.Write(EncodeTrackTargetUpdate(update)));

    EXPECT_FALSE(target_subscription.WaitTakeFor(100ms).has_value());
    EXPECT_FALSE(ReadTrackTargetAck(terminal).has_value());
    link.Stop();
}

TEST(GroundStationLinkTest, TrackTargetRejectsWithoutTimeSync) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto target_subscription = link.TargetOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    ASSERT_TRUE(terminal.Write(EncodeGcsHeartbeat()));
    ASSERT_TRUE(WaitUntil([&] { return link.IsConnected(); }, 200ms));

    TargetUpdateFixture update;
    update.source_time_ms = MonotonicMs();
    ASSERT_TRUE(terminal.Write(EncodeTrackTargetUpdate(update)));

    EXPECT_FALSE(target_subscription.WaitTakeFor(100ms).has_value());
    const auto ack = ReadTrackTargetAck(terminal);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(std::get<0>(*ack), TrackTargetAckResult::kRejectedTimeSyncUnavailable);
    EXPECT_EQ(std::get<1>(*ack), TrackTargetAckReason::kTimeSyncUnavailable);
    link.Stop();
}

TEST(GroundStationLinkTest, TrackTargetRejectsDuplicateSequence) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto target_subscription = link.TargetOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    EstablishTimeSync(terminal, link);

    TargetUpdateFixture update;
    update.source_time_ms = MonotonicMs() + link.GetTimeSyncStatus().offset_ns / 1000000LL;
    ASSERT_TRUE(terminal.Write(EncodeTrackTargetUpdate(update)));
    ASSERT_TRUE(target_subscription.WaitTakeFor(300ms).has_value());
    ASSERT_TRUE(ReadTrackTargetAck(terminal).has_value());

    update.source_time_ms = MonotonicMs() + link.GetTimeSyncStatus().offset_ns / 1000000LL;
    ASSERT_TRUE(terminal.Write(EncodeTrackTargetUpdate(update)));
    EXPECT_FALSE(target_subscription.WaitTakeFor(100ms).has_value());
    const auto duplicate_ack = ReadTrackTargetAck(terminal);
    ASSERT_TRUE(duplicate_ack.has_value());
    EXPECT_EQ(std::get<0>(*duplicate_ack),
              TrackTargetAckResult::kRejectedStaleOrDuplicate);
    EXPECT_EQ(std::get<1>(*duplicate_ack),
              TrackTargetAckReason::kUpdateSequenceStale);
    link.Stop();
}

TEST(GroundStationLinkTest, TrackTargetRejectsInvalidLatitude) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto target_subscription = link.TargetOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    EstablishTimeSync(terminal, link);

    TargetUpdateFixture update;
    update.latitude_1e7 = 910000000;
    update.source_time_ms = MonotonicMs() + link.GetTimeSyncStatus().offset_ns / 1000000LL;
    ASSERT_TRUE(terminal.Write(EncodeTrackTargetUpdate(update)));

    EXPECT_FALSE(target_subscription.WaitTakeFor(100ms).has_value());
    const auto ack = ReadTrackTargetAck(terminal);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(std::get<0>(*ack), TrackTargetAckResult::kRejectedInvalidField);
    EXPECT_EQ(std::get<1>(*ack), TrackTargetAckReason::kLatitudeOrLongitudeInvalid);
    link.Stop();
}

TEST(GroundStationLinkTest, TrackTargetRejectsExpiredSourceTime) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto target_subscription = link.TargetOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    EstablishTimeSync(terminal, link);

    TargetUpdateFixture update;
    update.valid_for_ms = 500;
    update.source_time_ms = MonotonicMs() + link.GetTimeSyncStatus().offset_ns / 1000000LL - 900;
    ASSERT_TRUE(terminal.Write(EncodeTrackTargetUpdate(update)));

    EXPECT_FALSE(target_subscription.WaitTakeFor(100ms).has_value());
    const auto ack = ReadTrackTargetAck(terminal);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(std::get<0>(*ack), TrackTargetAckResult::kRejectedStaleOrDuplicate);
    EXPECT_EQ(std::get<1>(*ack), TrackTargetAckReason::kTargetExpired);
    link.Stop();
}

TEST(GroundStationLinkTest, TrackTargetBootChangeClearsTimeSync) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto target_subscription = link.TargetOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    EstablishTimeSync(terminal, link);

    TargetUpdateFixture update;
    update.source_time_ms = MonotonicMs() + link.GetTimeSyncStatus().offset_ns / 1000000LL;
    ASSERT_TRUE(terminal.Write(EncodeTrackTargetUpdate(update)));
    ASSERT_TRUE(target_subscription.WaitTakeFor(300ms).has_value());
    ASSERT_TRUE(ReadTrackTargetAck(terminal).has_value());

    TargetUpdateFixture restarted = update;
    restarted.boot_id = 43;
    restarted.source_time_ms = MonotonicMs() + link.GetTimeSyncStatus().offset_ns / 1000000LL;
    ASSERT_TRUE(terminal.Write(EncodeTrackTargetUpdate(restarted)));

    EXPECT_FALSE(target_subscription.WaitTakeFor(100ms).has_value());
    const auto ack = ReadTrackTargetAck(terminal);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(std::get<0>(*ack), TrackTargetAckResult::kRejectedTimeSyncUnavailable);
    EXPECT_EQ(std::get<1>(*ack), TrackTargetAckReason::kBootIdInvalidOrChanged);
    EXPECT_NE(link.GetTimeSyncStatus().state,
              GroundStationTimeSyncState::kSynchronized);
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

// A-01：连续串口读错误必须关闭端口并自动重开，而不是永久停在"计数+sleep"。
TEST(GroundStationLinkTest, RecoversSerialLinkAfterReadErrors) {
    const std::string link_path =
        "/tmp/gs_link_reconnect_" + std::to_string(::getpid());
    ::unlink(link_path.c_str());
    PseudoTerminal first;
    ASSERT_EQ(::symlink(first.SlaveName().c_str(), link_path.c_str()), 0);

    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(link_path));
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    ASSERT_TRUE(first.Write(EncodeGcsHeartbeat()));
    ASSERT_TRUE(WaitUntil([&] { return link.IsConnected(); }, 300ms));

    // 关闭主端：从端读取立即失败，连续错误达阈值后链路应关闭端口并置未连接。
    first.CloseMaster();
    ASSERT_TRUE(WaitUntil([&] { return !link.IsConnected(); }, 1000ms));

    // 换上新的伪终端：重连成功后仍要重新等到地面站心跳才算连接建立。
    PseudoTerminal second;
    ASSERT_EQ(::unlink(link_path.c_str()), 0);
    ASSERT_EQ(::symlink(second.SlaveName().c_str(), link_path.c_str()), 0);
    ASSERT_TRUE(WaitUntil(
        [&] {
            second.Write(EncodeGcsHeartbeat());
            return link.IsConnected();
        }, 3000ms));

    ::unlink(link_path.c_str());
    link.Stop();
}

// A-03：动作码只允许 0/1/2；越界值必须显式拒收整帧，不能静默当返航执行。
TEST(GroundStationLinkTest, FenceZoneRejectsInvalidActionAndAltitudeBand) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto zone_subscription = link.FenceZoneOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());

    FenceZoneFixture zone;
    zone.action = 3;
    ASSERT_TRUE(terminal.Write(EncodeFenceZone(zone)));
    EXPECT_FALSE(zone_subscription.WaitTakeFor(150ms).has_value());

    // 高度带上下限倒置同样应拒收（alt_max_dm=0 表示不限制，不在此列）。
    zone.action = 1;
    zone.alt_min_dm = 500;
    zone.alt_max_dm = 100;
    ASSERT_TRUE(terminal.Write(EncodeFenceZone(zone)));
    EXPECT_FALSE(zone_subscription.WaitTakeFor(150ms).has_value());

    zone.alt_min_dm = 100;
    zone.alt_max_dm = 500;
    zone.action = 2;
    ASSERT_TRUE(terminal.Write(EncodeFenceZone(zone)));
    auto published = zone_subscription.WaitTakeFor(300ms);
    ASSERT_TRUE(published.has_value());
    EXPECT_EQ((**published).action, common::FenceAction::kLand);
    EXPECT_EQ((**published).vertices.size(), 4u);

    link.Stop();
}

// A-05a：65010 允许尾部零字节截断，最小合法总长 59 字节（alt_reference 可缺省为 0）。
TEST(GroundStationLinkTest, TrackTargetAcceptsTruncatedAltReference) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto target_subscription = link.TargetOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    EstablishTimeSync(terminal, link);

    TargetUpdateFixture update;
    update.alt_reference = 0;
    update.source_time_ms =
        MonotonicMs() + link.GetTimeSyncStatus().offset_ns / 1000000LL;
    ASSERT_TRUE(terminal.Write(EncodeTrackTargetUpdate(update)));

    auto target = target_subscription.WaitTakeFor(300ms);
    ASSERT_TRUE(target.has_value());
    EXPECT_EQ((**target).alt_reference, 0);
    const auto ack = ReadTrackTargetAck(terminal);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(std::get<0>(*ack), TrackTargetAckResult::kAccepted);
    link.Stop();
}

// A-05b：帧长不足 59 字节（payload 缺 alt_reference 之外的必需字段）必须拒收并回 ACK。
TEST(GroundStationLinkTest, TrackTargetUpdateRejectsTruncatedFrame) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    auto target_subscription = link.TargetOutput().Subscribe(2);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    EstablishTimeSync(terminal, link);

    TargetUpdateFixture update;
    update.source_time_ms =
        MonotonicMs() + link.GetTimeSyncStatus().offset_ns / 1000000LL;
    // 从 target_component 起截断：payload 只剩 50 字节。
    ASSERT_TRUE(terminal.Write(EncodeTrackTargetUpdate(update, 50)));

    EXPECT_FALSE(target_subscription.WaitTakeFor(150ms).has_value());
    const auto ack = ReadTrackTargetAck(terminal);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(std::get<0>(*ack), TrackTargetAckResult::kRejectedInvalidField);
    EXPECT_EQ(std::get<1>(*ack), TrackTargetAckReason::kInternalError);
    link.Stop();
}

// A-07：周期遥测发送失败时不能推进正常周期，而应按短退避重试（否则要等一整个周期）。
TEST(GroundStationLinkTest, TelemetrySendFailureRetriesWithBackoff) {
    PseudoTerminal terminal;
    // mavlink 1 下扩展帧恒编码失败，用来构造确定性的周期发送失败。
    auto config = MakeConfig(terminal.SlaveName());
    config.mavlink_version = 1;
    config.enable_time_sync = false;
    config.health_status_send_interval = 5000ms;
    config.mission_status_send_interval = 5000ms;

    common::Topic<common::FlightStateSnapshot> flight_state;
    common::Topic<common::HealthStatus> health_status;
    common::Topic<common::MissionStatus> mission_status;
    GroundStationLink link(config);
    link.SetFlightStateInput(flight_state);
    link.SetHealthInput(health_status);
    link.SetMissionStatusInput(mission_status);
    ASSERT_TRUE(link.Start());

    common::HealthStatus health;
    health.header.sequence = 5;
    health.cpu_load_pct = 12.f;
    ASSERT_TRUE(health_status.Publish(
        std::make_shared<const common::HealthStatus>(health)).accepted);
    common::MissionStatus mission;
    mission.header.sequence = 9;
    ASSERT_TRUE(mission_status.Publish(
        std::make_shared<const common::MissionStatus>(mission)).accepted);

    std::this_thread::sleep_for(150ms);
    const uint64_t before = link.ErrorCount();
    std::this_thread::sleep_for(300ms);
    const uint64_t after = link.ErrorCount();
    // 旧实现把 deadline 推到 5s 之后：窗口内只有 1~2 次失败。
    // 新实现按 100ms 退避重试：两路遥测各约 3 次。
    EXPECT_GE(after - before, 4u);
    link.Stop();
}

// A-04：已同步后抖动恶化必须降级（协议 §5 DEGRADED 覆盖 RTT/offset 抖动）。
TEST(GroundStationLinkTest, TimeSyncDegradesWhenJitterExceedsThreshold) {
    PseudoTerminal terminal;
    auto config = MakeConfig(terminal.SlaveName());
    config.serial.read_timeout = 1ms;
    config.time_sync_minimum_samples = 5;
    config.time_sync_window_capacity = 5;
    config.time_sync_max_rtt = 200ms;
    config.time_sync_max_offset_jump = 30ms;
    config.time_sync_timeout = 5000ms;
    config.time_sync_acquire_interval = 10ms;
    config.time_sync_steady_interval = 10ms;

    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(config);
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    ASSERT_TRUE(terminal.Write(EncodeGcsHeartbeat()));
    ASSERT_TRUE(WaitUntil([&] { return link.IsConnected(); }, 200ms));

    // 前 5 个样本用 0 与 -24ms 建立同步（跨度 24ms ≤ 30ms 门限）；
    // 之后 3 个 27ms 样本逐步把中位数推到 27ms，跨度扩大到约 51ms 触发降级，
    // 而逐样本跳跃始终 ≤ 30ms，不会走"offset 突变"分支。
    const std::array<int64_t, 8> offsets_ms{0, 0, 0, 0, -24, 27, 27, 27};
    for (const int64_t offset_ms : offsets_ms) {
        mavlink_message_t request{};
        ASSERT_TRUE(terminal.WaitFor(
            [](const mavlink_message_t& message) {
                return message.msgid == MAVLINK_MSG_ID_TIMESYNC;
            }, 500ms, &request));
        ASSERT_TRUE(terminal.Write(
            EncodeTimeSyncResponse(request, offset_ms * 1000000LL)));
    }

    ASSERT_TRUE(WaitUntil(
        [&] {
            return link.GetTimeSyncStatus().state ==
                   GroundStationTimeSyncState::kDegraded;
        }, 1000ms));
    link.Stop();
}

// A-08：无状态变化时不逐次发布时间同步状态（否则 20ms 采样即 ~50Hz 刷新）。
TEST(GroundStationLinkTest, TimeSyncStatusPublishIsLowFrequency) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    link.SetFlightStateInput(flight_state);
    ASSERT_TRUE(link.Start());
    EstablishTimeSync(terminal, link);

    const auto before = link.GetTimeSyncStatus();
    std::this_thread::sleep_for(150ms);
    const auto after = link.GetTimeSyncStatus();
    // 快照只在状态跃迁或 1Hz 到期时刷新，样本年龄应保持不变。
    EXPECT_EQ(before.sample_age_ms, after.sample_age_ms);
    link.Stop();
}

// A-09：65022 回读时高度带与顶点偏移必须做量程钳制与 NaN 保护，不能回绕。
TEST(GroundStationLinkTest, FenceZoneReportClampsOutOfRangeValues) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    common::Topic<common::FenceZone> report_input;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    link.SetFlightStateInput(flight_state);
    link.SetFenceZoneReportInput(report_input);
    ASSERT_TRUE(link.Start());

    common::FenceZone zone;
    zone.zone_id = 4;
    zone.version = 2;
    zone.action = common::FenceAction::kReturn;
    zone.alt_min_m = std::numeric_limits<double>::quiet_NaN();
    zone.alt_max_m = 1.0e6;
    zone.ref_lat_1e7 = 231230000;
    zone.ref_lon_1e7 = 1131230000;
    zone.vertices.push_back({23.123, 113.123});
    zone.vertices.push_back({23.133, 113.123});  // 约 +1113m，远超 ±327.67m
    zone.vertices.push_back({23.123, 113.133});
    ASSERT_TRUE(report_input.Publish(
        std::make_shared<const common::FenceZone>(zone)).accepted);

    mavlink_message_t report{};
    ASSERT_TRUE(terminal.WaitFor(
        [](const mavlink_message_t& message) {
            if (message.msgid != MAVLINK_MSG_ID_V2_EXTENSION) {
                return false;
            }
            mavlink_v2_extension_t extension{};
            mavlink_msg_v2_extension_decode(&message, &extension);
            return extension.message_type == kFenceZoneReportMessageType;
        }, 500ms, &report));
    mavlink_v2_extension_t extension{};
    mavlink_msg_v2_extension_decode(&report, &extension);
    // NaN 高度下限按 0 处理；超量程上限钳到 int16 最大值。
    EXPECT_EQ(ReadLe<int16_t>(extension.payload, 8), 0);
    EXPECT_EQ(ReadLe<int16_t>(extension.payload, 10),
              std::numeric_limits<int16_t>::max());
    // 顶点 0 落在参考点；顶点 1 的北向偏移约 +1113m（厘米量纲 111300）应钳到上限。
    EXPECT_EQ(ReadLe<int16_t>(extension.payload, 21), 0);
    EXPECT_EQ(ReadLe<int16_t>(extension.payload, 23), 0);
    EXPECT_EQ(ReadLe<int16_t>(extension.payload, 25),
              std::numeric_limits<int16_t>::max());
    link.Stop();
}

// A-11：顶点数超过协议上限时回读必须显式失败（计告警），不能静默丢弃。
TEST(GroundStationLinkTest, FenceZoneReportRejectsTooManyVertices) {
    PseudoTerminal terminal;
    common::Topic<common::FlightStateSnapshot> flight_state;
    common::Topic<common::FenceZone> report_input;
    GroundStationLink link(MakeConfig(terminal.SlaveName()));
    link.SetFlightStateInput(flight_state);
    link.SetFenceZoneReportInput(report_input);
    ASSERT_TRUE(link.Start());

    common::FenceZone zone;
    zone.zone_id = 5;
    zone.version = 1;
    zone.ref_lat_1e7 = 231230000;
    zone.ref_lon_1e7 = 1131230000;
    for (int index = 0; index < 9; ++index) {
        zone.vertices.push_back({23.123 + 0.0001 * index, 113.123});
    }
    ASSERT_TRUE(report_input.Publish(
        std::make_shared<const common::FenceZone>(zone)).accepted);

    EXPECT_FALSE(terminal.WaitFor(
        [](const mavlink_message_t& message) {
            if (message.msgid != MAVLINK_MSG_ID_V2_EXTENSION) {
                return false;
            }
            mavlink_v2_extension_t extension{};
            mavlink_msg_v2_extension_decode(&message, &extension);
            return extension.message_type == kFenceZoneReportMessageType;
        }, 300ms));
    link.Stop();
}

}  // namespace
}  // namespace drone::communication
