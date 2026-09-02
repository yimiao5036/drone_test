#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "config/config.h"

namespace {

using nlohmann::json;

std::string SourceConfigPath() {
    return std::string(TEST_SOURCE_DIR) + "/config/config.json";
}

json ReadSourceConfig() {
    std::ifstream input(SourceConfigPath());
    EXPECT_TRUE(input.is_open());
    return json::parse(input);
}

std::filesystem::path WriteTemporaryConfig(json value, const char* name) {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream output(path);
    output << value.dump(2);
    output.close();
    return path;
}

TEST(ConfigTest, LoadsCurrentProductionConfiguration) {
    const auto config = drone::config::LoadAppConfig(SourceConfigPath(), "/opt/drone");

    EXPECT_TRUE(config.runtime.enable_video);
    EXPECT_TRUE(config.runtime.enable_px4);
    EXPECT_TRUE(config.runtime.enable_ground_station);
    EXPECT_FALSE(config.runtime.enable_control);
    EXPECT_EQ(config.px4.transport, "serial");
    EXPECT_EQ(config.px4.onboard_system_id, 1);
    EXPECT_EQ(config.px4.onboard_component_id, 191);
    EXPECT_EQ(config.px4.target_system_id, 1);
    EXPECT_EQ(config.px4.target_component_id, 1);
    EXPECT_EQ(config.px4.serial.device, "/dev/ttyS1");
    EXPECT_EQ(config.ground_station.aircraft_system_id, 1);
    EXPECT_EQ(config.ground_station.aircraft_component_id, 25);
    EXPECT_EQ(config.ground_station.aircraft_type, "net_capture");
    EXPECT_EQ(config.ground_station.aircraft_number, 1);
    EXPECT_EQ(config.ground_station.callsign, "捕网-01");
    EXPECT_EQ(config.ground_station.ground_system_id, 255);
    EXPECT_EQ(config.ground_station.ground_component_id, 190);
    EXPECT_EQ(config.ground_station.serial.device, "/dev/ttyS6");
    EXPECT_EQ(config.ground_station.serial.baud_rate, 115200);
    EXPECT_EQ(config.ground_station.serial.data_bits, 8);
    EXPECT_EQ(config.ground_station.mavlink_version, 2);
    EXPECT_EQ(config.ground_station.attitude_send_interval.count(), 100);
    EXPECT_EQ(config.yolo.model_path, "/opt/drone/models/yolo26n-drone.rknn");
}

TEST(ConfigTest, RejectsControlBeforeFormalAssemblyIsEnabled) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_control"] = true;
    const auto path = WriteTemporaryConfig(value, "drone_config_control_rejected.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsGroundStationWithoutPx4StateSource) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_px4"] = false;
    const auto path = WriteTemporaryConfig(value, "drone_config_ground_without_px4.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, AllowsMissingGroundStationSectionWhenDisabled) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_ground_station"] = false;
    value.erase("ground_station");
    const auto path = WriteTemporaryConfig(value, "drone_config_ground_disabled.json");

    EXPECT_NO_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"));
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsInvalidGroundStationSendInterval) {
    json value = ReadSourceConfig();
    value["ground_station"]["send_interval_ms"]["gps"] = 0;
    const auto path = WriteTemporaryConfig(value, "drone_config_ground_rate_invalid.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, AcceptsRocketGroundStationIdentity) {
    json value = ReadSourceConfig();
    value["ground_station"]["aircraft_component_id"] = 26;
    value["ground_station"]["aircraft_type"] = "rocket";
    value["ground_station"]["callsign"] = "火箭-01";
    const auto path = WriteTemporaryConfig(value, "drone_config_rocket_identity.json");

    const auto config = drone::config::LoadAppConfig(path.string(), "/opt/drone");
    EXPECT_EQ(config.ground_station.aircraft_system_id, 1);
    EXPECT_EQ(config.ground_station.aircraft_component_id, 26);
    EXPECT_EQ(config.ground_station.aircraft_type, "rocket");
    EXPECT_EQ(config.ground_station.callsign, "火箭-01");
    EXPECT_EQ(config.px4.onboard_component_id, 191);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsGroundStationTypeComponentMismatch) {
    json value = ReadSourceConfig();
    value["ground_station"]["aircraft_component_id"] = 26;
    value["ground_station"]["aircraft_type"] = "net_capture";
    const auto path = WriteTemporaryConfig(value, "drone_config_identity_mismatch.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsReservedGroundStationComponent) {
    json value = ReadSourceConfig();
    value["ground_station"]["aircraft_component_id"] = 27;
    value["ground_station"]["aircraft_type"] = "rocket";
    const auto path = WriteTemporaryConfig(value, "drone_config_reserved_identity.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsGroundStationNumberMismatch) {
    json value = ReadSourceConfig();
    value["ground_station"]["aircraft_system_id"] = 2;
    value["ground_station"]["aircraft_number"] = 1;
    const auto path = WriteTemporaryConfig(value, "drone_config_number_mismatch.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsUnexpectedGroundStationSourceIdentity) {
    json value = ReadSourceConfig();
    value["ground_station"]["ground_system_id"] = 254;
    const auto path = WriteTemporaryConfig(value, "drone_config_gcs_identity_invalid.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsConfigurationWithNoEnabledDataLink) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_video"] = false;
    value["runtime"]["enable_px4"] = false;
    value["runtime"]["enable_ground_station"] = false;
    const auto path = WriteTemporaryConfig(value, "drone_config_empty_runtime.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

}  // namespace
