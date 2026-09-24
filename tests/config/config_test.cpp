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
    EXPECT_TRUE(config.runtime.enable_target_estimator);
    EXPECT_TRUE(config.runtime.enable_visual_monitor);
    EXPECT_FALSE(config.runtime.enable_control);
    EXPECT_EQ(config.health.cpu_sample_period.count(), 1000);
    EXPECT_EQ(config.health.startup_grace_period.count(), 5000);
    EXPECT_EQ(config.health.camera_max_age.count(), 1000);
    EXPECT_EQ(config.health.decoder_max_age.count(), 1000);
    EXPECT_EQ(config.health.yolo_max_age.count(), 1000);
    EXPECT_EQ(config.health.video_max_age.count(), 1000);
    EXPECT_EQ(config.health.px4_max_age.count(), 3000);
    EXPECT_EQ(config.health.ground_station_max_age.count(), 3000);
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
    EXPECT_TRUE(config.ground_station.enable_time_sync);
    EXPECT_EQ(config.ground_station.time_sync_acquire_interval.count(), 200);
    EXPECT_EQ(config.ground_station.time_sync_steady_interval.count(), 1000);
    EXPECT_EQ(config.ground_station.time_sync_minimum_samples, 5u);
    EXPECT_EQ(config.ground_station.time_sync_window_capacity, 10u);
    EXPECT_EQ(config.ground_station.time_sync_timeout.count(), 5000);
    EXPECT_EQ(config.ground_station.time_sync_max_rtt.count(), 300);
    EXPECT_EQ(config.ground_station.time_sync_max_offset_jump.count(), 100);
    EXPECT_EQ(config.ground_station.target_minimum_valid_for.count(), 100);
    EXPECT_EQ(config.ground_station.target_maximum_valid_for.count(), 5000);
    EXPECT_EQ(config.ground_station.target_minimum_remaining_valid.count(), 100);
    EXPECT_EQ(config.ground_station.target_maximum_transport_delay.count(), 1000);
    EXPECT_EQ(config.ground_station.target_future_tolerance.count(), 200);
    EXPECT_EQ(config.ground_station.attitude_send_interval.count(), 100);
    EXPECT_EQ(config.ground_station.health_status_send_interval.count(), 1000);
    EXPECT_EQ(config.ground_station.mission_status_send_interval.count(), 500);
    EXPECT_EQ(config.target_estimator.ground_target_queue_capacity, 4U);
    EXPECT_EQ(config.target_estimator.flight_state_queue_capacity, 2U);
    EXPECT_EQ(config.target_estimator.publish_interval.count(), 100);
    EXPECT_DOUBLE_EQ(config.target_estimator.process_acceleration_std_mps2, 8.0);
    EXPECT_DOUBLE_EQ(config.target_estimator.default_horizontal_accuracy_m, 10.0);
    EXPECT_EQ(config.visual_monitor.input_queue_capacity, 2U);
    EXPECT_EQ(config.visual_monitor.publish_interval.count(), 100);
    EXPECT_EQ(config.visual_monitor.observation_timeout.count(), 500);
    EXPECT_EQ(config.visual_monitor.lock_frames, 5);
    EXPECT_EQ(config.visual_monitor.lost_frames, 10);
    EXPECT_DOUBLE_EQ(config.visual_monitor.smoothing_alpha, 0.3);
    EXPECT_EQ(config.visual_monitor.transition_log_interval.count(), 1000);
    EXPECT_EQ(config.visual_monitor.status_log_interval.count(), 5000);
    EXPECT_TRUE(config.decoder.prefer_rga_dma_transfer);
    // 生产配置已切换 USB 双目 UVC 链路（提交 cd1b4eb），RTSP 为保留备份
    EXPECT_EQ(config.video_source.camera_source, "uvc");
    EXPECT_TRUE(config.video_source.stereo_split);
    EXPECT_EQ(config.uvc_camera.device, "/dev/video0");
    EXPECT_EQ(config.uvc_camera.width, 2560u);
    EXPECT_EQ(config.uvc_camera.height, 720u);
    EXPECT_EQ(config.uvc_camera.fps, 30u);
    EXPECT_EQ(config.stereo_splitter.width, 2560u);
    EXPECT_EQ(config.stereo_splitter.height, 720u);
    // 生产配置已切换 384x640 矩形输入模型（提交 cdde006）
    EXPECT_EQ(config.yolo.model_path,
              "/opt/drone/models/yolov26_ReLU6_09_13-rk3588_384_640.rknn");
    EXPECT_EQ(config.yolo.input_queue_capacity, 1u);
    EXPECT_EQ(config.yolo.npu_core_mode, "all");
    EXPECT_FALSE(config.yolo.collect_npu_internal_perf);
    EXPECT_FALSE(config.yolo.collect_npu_perf_detail);
    EXPECT_EQ(config.video_sender.encode.url,
              "rtsp://127.0.0.1:8554/drone_25_1");
    // 视觉跟踪控制律（影子）：camera 组为 2026-09-23 左目标定实测内参
    EXPECT_TRUE(config.runtime.enable_visual_tracking);
    EXPECT_DOUBLE_EQ(config.visual_tracking.camera.fx_px, 1007.82);
    EXPECT_DOUBLE_EQ(config.visual_tracking.camera.fy_px, 1008.02);
    EXPECT_DOUBLE_EQ(config.visual_tracking.camera.cx_px, 620.73);
    EXPECT_DOUBLE_EQ(config.visual_tracking.camera.cy_px, 532.07);
    EXPECT_EQ(config.visual_tracking.camera.image_width, 1280);
    EXPECT_EQ(config.visual_tracking.camera.image_height, 720);
    EXPECT_TRUE(config.visual_tracking_shadow.virtual_attitude_when_absent);
}

TEST(ConfigTest, DefaultsVisualTrackingWhenSectionMissing) {
    json value = ReadSourceConfig();
    value.erase("visual_tracking");
    const auto path = WriteTemporaryConfig(value, "drone_config_vt_default.json");

    const auto config = drone::config::LoadAppConfig(path.string(), "/opt/drone");
    // 段缺席回退默认值（即 2026-09-23 标定实测内参）
    EXPECT_DOUBLE_EQ(config.visual_tracking.camera.fx_px, 1007.82);
    EXPECT_DOUBLE_EQ(config.visual_tracking.camera.cy_px, 532.07);
    EXPECT_EQ(config.visual_tracking.heading.mode,
              drone::control::HeadingMode::kRate);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsInvalidVisualTrackingHeadingMode) {
    json value = ReadSourceConfig();
    value["visual_tracking"]["heading"]["mode"] = "invalid";
    const auto path = WriteTemporaryConfig(value, "drone_config_vt_mode_invalid.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsVisualTrackingPrincipalPointOutOfFrame) {
    json value = ReadSourceConfig();
    value["visual_tracking"]["camera"]["cy_px"] = 5000.0;  // 超出画面高 720
    const auto path = WriteTemporaryConfig(value, "drone_config_vt_cy_invalid.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsVisualTrackingWithoutVisualMonitor) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_visual_monitor"] = false;
    const auto path = WriteTemporaryConfig(value, "drone_config_vt_no_monitor.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, DefaultsRgaDmaTransferToDisabledWhenFieldMissing) {
    json value = ReadSourceConfig();
    value["video"].erase("prefer_rga_dma_transfer");
    const auto path = WriteTemporaryConfig(value, "drone_config_rga_dma_default.json");

    const auto config = drone::config::LoadAppConfig(path.string(), "/opt/drone");
    EXPECT_FALSE(config.decoder.prefer_rga_dma_transfer);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsInvalidYoloNpuCoreMode) {
    json value = ReadSourceConfig();
    value["yolo"]["npu_core_mode"] = "invalid";
    const auto path = WriteTemporaryConfig(value, "drone_config_yolo_core_invalid.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, DefaultsVideoSourceWhenFieldsMissing) {
    json value = ReadSourceConfig();
    value["video"].erase("camera_source");
    value["video"].erase("stereo_split");
    value["video"].erase("uvc_device");
    value["video"].erase("uvc_width");
    value["video"].erase("uvc_height");
    value["video"].erase("uvc_fps");
    const auto path =
        WriteTemporaryConfig(value, "drone_config_video_source_default.json");

    const auto config = drone::config::LoadAppConfig(path.string(), "/opt/drone");
    EXPECT_EQ(config.video_source.camera_source, "rtsp");
    EXPECT_FALSE(config.video_source.stereo_split);
    EXPECT_EQ(config.uvc_camera.device, "/dev/video0");
    EXPECT_EQ(config.uvc_camera.width, 2560u);
    EXPECT_EQ(config.uvc_camera.height, 720u);
    EXPECT_EQ(config.uvc_camera.fps, 30u);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsInvalidCameraSource) {
    json value = ReadSourceConfig();
    value["video"]["camera_source"] = "usb";
    const auto path =
        WriteTemporaryConfig(value, "drone_config_camera_source_invalid.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsStereoSplitWithoutUvcSource) {
    json value = ReadSourceConfig();
    value["video"]["camera_source"] = "rtsp";
    value["video"]["stereo_split"] = true;
    const auto path =
        WriteTemporaryConfig(value, "drone_config_split_without_uvc.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsOddUvcWidthForUvcSource) {
    json value = ReadSourceConfig();
    value["video"]["camera_source"] = "uvc";
    value["video"]["uvc_width"] = 2561;
    const auto path =
        WriteTemporaryConfig(value, "drone_config_uvc_odd_width.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, ParsesUvcStereoConfiguration) {
    json value = ReadSourceConfig();
    value["video"]["camera_source"] = "uvc";
    value["video"]["stereo_split"] = true;
    value["video"]["uvc_width"] = 3840;
    value["video"]["uvc_height"] = 1080;
    const auto path = WriteTemporaryConfig(value, "drone_config_uvc_stereo.json");

    const auto config = drone::config::LoadAppConfig(path.string(), "/opt/drone");
    EXPECT_EQ(config.video_source.camera_source, "uvc");
    EXPECT_TRUE(config.video_source.stereo_split);
    EXPECT_EQ(config.uvc_camera.width, 3840u);
    EXPECT_EQ(config.uvc_camera.height, 1080u);
    EXPECT_EQ(config.stereo_splitter.width, 3840u);
    EXPECT_EQ(config.stereo_splitter.height, 1080u);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsControlBeforeFormalAssemblyIsEnabled) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_control"] = true;
    const auto path = WriteTemporaryConfig(value, "drone_config_control_rejected.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsTargetEstimatorWithoutGroundStation) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_ground_station"] = false;
    const auto path = WriteTemporaryConfig(
        value, "drone_config_estimator_without_ground.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsVisualMonitorWithoutVideo) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_video"] = false;
    const auto path = WriteTemporaryConfig(
        value, "drone_config_visual_monitor_without_video.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsInvalidVisualMonitorThreshold) {
    json value = ReadSourceConfig();
    value["visual_monitor"]["lock_frames"] = 0;
    const auto path = WriteTemporaryConfig(
        value, "drone_config_visual_monitor_threshold_invalid.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsInvalidVisualMonitorLogInterval) {
    json value = ReadSourceConfig();
    value["visual_monitor"]["status_log_interval_ms"] = 0;
    const auto path = WriteTemporaryConfig(
        value, "drone_config_visual_monitor_log_interval_invalid.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsInvalidTargetEstimatorNoise) {
    json value = ReadSourceConfig();
    value["target_estimator"]["process_acceleration_std_mps2"] = 0.0;
    const auto path = WriteTemporaryConfig(
        value, "drone_config_estimator_noise_invalid.json");

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
    value["runtime"]["enable_target_estimator"] = false;
    value.erase("ground_station");
    value["video"]["output_rtsp"] = "rtsp://127.0.0.1:8554/drone_out";
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

TEST(ConfigTest, RejectsInvalidTimeSyncWindow) {
    json value = ReadSourceConfig();
    value["ground_station"]["time_sync"]["minimum_samples"] = 11;
    value["ground_station"]["time_sync"]["window_capacity"] = 10;
    const auto path = WriteTemporaryConfig(value, "drone_config_timesync_window.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsTimeSyncRttNotBelowTimeout) {
    json value = ReadSourceConfig();
    value["ground_station"]["time_sync"]["max_rtt_ms"] = 5000;
    value["ground_station"]["time_sync"]["sync_timeout_ms"] = 5000;
    const auto path = WriteTemporaryConfig(value, "drone_config_timesync_timeout.json");

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
    EXPECT_EQ(config.video_sender.encode.url,
              "rtsp://127.0.0.1:8554/drone_26_1");
    EXPECT_EQ(config.px4.onboard_component_id, 191);
    std::filesystem::remove(path);
}

TEST(ConfigTest, AllowsExplicitVideoOutputWithoutIdentityPlaceholder) {
    json value = ReadSourceConfig();
    value["video"]["output_rtsp"] = "rtsp://127.0.0.1:8554/debug";
    const auto path = WriteTemporaryConfig(value, "drone_config_explicit_video.json");

    const auto config = drone::config::LoadAppConfig(path.string(), "/opt/drone");
    EXPECT_EQ(config.video_sender.encode.url, "rtsp://127.0.0.1:8554/debug");
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

TEST(ConfigTest, RejectsNonPositiveStatusSendInterval) {
    json value = ReadSourceConfig();
    value["ground_station"]["status_send_interval_ms"]["health"] = 0;
    const auto path = WriteTemporaryConfig(value, "drone_config_status_interval_invalid.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsNegativeHealthTimeout) {
    json value = ReadSourceConfig();
    value["health"]["cpu_sample_period_ms"] = 0;
    const auto path = WriteTemporaryConfig(value, "drone_config_health_timeout_invalid.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, StereoRangerDefaultsWhenSectionAbsent) {
    json value = ReadSourceConfig();
    value.erase("stereo_ranger");
    value.erase("yolo_right");
    value["runtime"].erase("enable_stereo_ranging");
    const auto path = WriteTemporaryConfig(value, "drone_config_stereo_default.json");

    const auto config = drone::config::LoadAppConfig(path.string(), "/opt/drone");
    // 段缺席回退默认值（即 2026-09-23 标定实测内参/基线）
    EXPECT_DOUBLE_EQ(config.stereo_ranger.baseline_m, 0.060085);
    EXPECT_DOUBLE_EQ(config.stereo_ranger.cx_right_px, 674.76);
    EXPECT_DOUBLE_EQ(config.stereo_ranger.distance_max_m, 11.0);
    EXPECT_FALSE(config.runtime.enable_stereo_ranging);
    // yolo_right 段缺席时默认继承 yolo 全部值
    EXPECT_EQ(config.yolo_right.model_path, config.yolo.model_path);
    EXPECT_EQ(config.yolo_right.npu_core_mode, config.yolo.npu_core_mode);
    std::filesystem::remove(path);
}

TEST(ConfigTest, StereoRangingRequiresUvcSplitAndVisualTracking) {
    // 依赖链：enable_video + camera_source=uvc + stereo_split + 视觉跟踪影子，
    // 任一环节缺失都必须拒绝。
    {
        json value = ReadSourceConfig();
        value["runtime"]["enable_stereo_ranging"] = true;
        value["video"]["stereo_split"] = false;
        const auto path =
            WriteTemporaryConfig(value, "drone_config_stereo_no_split.json");
        EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                     std::invalid_argument);
        std::filesystem::remove(path);
    }
    {
        json value = ReadSourceConfig();
        value["runtime"]["enable_stereo_ranging"] = true;
        value["video"]["camera_source"] = "rtsp";
        value["video"]["stereo_split"] = false;
        const auto path =
            WriteTemporaryConfig(value, "drone_config_stereo_rtsp.json");
        EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                     std::invalid_argument);
        std::filesystem::remove(path);
    }
    {
        json value = ReadSourceConfig();
        value["runtime"]["enable_stereo_ranging"] = true;
        value["runtime"]["enable_visual_tracking"] = false;
        const auto path =
            WriteTemporaryConfig(value, "drone_config_stereo_no_tracking.json");
        EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                     std::invalid_argument);
        std::filesystem::remove(path);
    }
}

TEST(ConfigTest, YoloRightOverridesOnlySpecifiedFields) {
    json value = ReadSourceConfig();
    value["yolo_right"] = json::object({{"npu_core_mode", "core01"}});
    const auto path = WriteTemporaryConfig(value, "drone_config_yolo_right.json");

    const auto config = drone::config::LoadAppConfig(path.string(), "/opt/drone");
    // 仅覆盖 npu_core_mode，其余字段全部继承左目 yolo
    EXPECT_EQ(config.yolo_right.npu_core_mode, "core01");
    EXPECT_EQ(config.yolo.npu_core_mode, "all");
    EXPECT_EQ(config.yolo_right.model_path, config.yolo.model_path);
    EXPECT_FLOAT_EQ(config.yolo_right.conf_threshold, config.yolo.conf_threshold);
    EXPECT_FLOAT_EQ(config.yolo_right.nms_threshold, config.yolo.nms_threshold);
    EXPECT_EQ(config.yolo_right.input_queue_capacity,
              config.yolo.input_queue_capacity);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsInvalidYoloRightNpuCoreMode) {
    json value = ReadSourceConfig();
    value["yolo_right"] = json::object({{"npu_core_mode", "invalid"}});
    const auto path =
        WriteTemporaryConfig(value, "drone_config_yolo_right_invalid.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, StereoRangerRejectsInvalidValues) {
    {
        json value = ReadSourceConfig();
        value["stereo_ranger"]["disparity_min_px"] = 0.0;
        const auto path =
            WriteTemporaryConfig(value, "drone_config_stereo_disparity_invalid.json");
        EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                     std::invalid_argument);
        std::filesystem::remove(path);
    }
    {
        json value = ReadSourceConfig();
        value["stereo_ranger"]["distance_min_m"] = 11.0;
        value["stereo_ranger"]["distance_max_m"] = 11.0;
        const auto path =
            WriteTemporaryConfig(value, "drone_config_stereo_range_invalid.json");
        EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                     std::invalid_argument);
        std::filesystem::remove(path);
    }
}

TEST(ConfigTest, VisualTrackingStaleFieldsAreParsed) {
    // 回归：visual_stale_ms/distance_stale_ms/attitude_stale_ms 此前在
    // config.json 声明但 config.cpp 未解析，静默失效。
    json value = ReadSourceConfig();
    value["visual_tracking"]["control"]["distance_stale_ms"] = 250;
    const auto path = WriteTemporaryConfig(value, "drone_config_vt_stale.json");

    const auto config = drone::config::LoadAppConfig(path.string(), "/opt/drone");
    EXPECT_EQ(config.visual_tracking.control.distance_stale_ms, 250);
    EXPECT_EQ(config.visual_tracking.control.visual_stale_ms, 200);
    EXPECT_EQ(config.visual_tracking.control.attitude_stale_ms, 300);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsConfigurationWithNoEnabledDataLink) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_video"] = false;
    value["runtime"]["enable_px4"] = false;
    value["runtime"]["enable_ground_station"] = false;
    value["runtime"]["enable_target_estimator"] = false;
    value["runtime"]["enable_visual_monitor"] = false;
    const auto path = WriteTemporaryConfig(value, "drone_config_empty_runtime.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

}  // namespace
