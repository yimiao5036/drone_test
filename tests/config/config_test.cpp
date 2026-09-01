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
    EXPECT_EQ(config.px4.serial.device, "/dev/ttyS1");
    EXPECT_EQ(config.ground_station.serial.device, "/dev/ttyS6");
    EXPECT_EQ(config.ground_station.serial.baud_rate, 115200);
    EXPECT_EQ(config.ground_station.serial.data_bits, 8);
    EXPECT_EQ(config.ground_station.mavlink_version, 2);
    EXPECT_EQ(config.ground_station.attitude_send_interval.count(), 100);
    EXPECT_EQ(config.yolo.model_path, "/opt/drone/models/yolo26n-int8.rknn");
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
