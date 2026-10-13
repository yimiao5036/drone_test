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

void CopyRectifyMatricesBeside(const std::filesystem::path& config_path) {
    // 预置校正矩阵与 config.json 同目录解析；临时配置也要有伴生矩阵文件
    // （仅 enable_stereo_ranging=true 的用例会读取）。
    const auto source = std::filesystem::path(TEST_SOURCE_DIR) / "config" /
                        "stereo_rectify_uvc_1280x720.json";
    if (!std::filesystem::exists(source)) {
        return;
    }
    std::filesystem::copy_file(
        source, config_path.parent_path() / "stereo_rectify_uvc_1280x720.json",
        std::filesystem::copy_options::overwrite_existing);
}

std::filesystem::path WriteTemporaryConfig(json value, const char* name) {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream output(path);
    output << value.dump(2);
    output.close();
    CopyRectifyMatricesBeside(path);
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
    // 本用例刻意移除 uvc/stereo_split，会破坏双目测距依赖链（另有专门用例
    // 覆盖该拒绝），故先关掉测距，聚焦视频源默认值。
    value["runtime"]["enable_stereo_ranging"] = false;
    // 视觉融合依赖测距，一并关闭（依赖链另有专门用例覆盖）
    value["runtime"]["enable_visual_position_fusion"] = false;
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

// 阶段四装配门禁：enable_control=true 在依赖链（PX4+地面站）齐备且
// controller 配置 Validate 通过时允许加载（正式绑定仍是装配期行为，
// 仓库生产 config.json 永远保持 false）。
TEST(ConfigTest, AllowsControlWhenAssemblyPreconditionsMet) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_control"] = true;
    const auto path = WriteTemporaryConfig(value, "drone_config_control_allowed.json");

    const auto config = drone::config::LoadAppConfig(path.string(), "/opt/drone");
    EXPECT_TRUE(config.runtime.enable_control);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsControlWithoutGroundStation) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_control"] = true;
    value["runtime"]["enable_ground_station"] = false;
    // 摘掉依赖地面站的链路，确保拒绝原因只剩 enable_control 门禁本身。
    value["runtime"]["enable_target_estimator"] = false;
    value["runtime"]["enable_visual_position_fusion"] = false;
    const auto path =
        WriteTemporaryConfig(value, "drone_config_control_without_ground.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsControlWithInvalidControllerConfig) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_control"] = true;
    // coast_timeout 必须大于 intent_timeout（默认 300ms），违反则 Validate 拒绝。
    value["controller"]["coast_timeout_ms"] = 100;
    const auto path = WriteTemporaryConfig(
        value, "drone_config_control_invalid_controller.json");

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
    // 视觉融合依赖目标估计器，一并关闭
    value["runtime"]["enable_visual_position_fusion"] = false;
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
    value["runtime"].erase("enable_stereo_ranging");
    // 视觉融合依赖测距，一并关闭
    value["runtime"]["enable_visual_position_fusion"] = false;
    const auto path = WriteTemporaryConfig(value, "drone_config_stereo_default.json");

    const auto config = drone::config::LoadAppConfig(path.string(), "/opt/drone");
    // 段缺席回退默认值（即参考实现实测配置）
    EXPECT_DOUBLE_EQ(config.stereo_ranger.disp_min_px, 8.0);
    EXPECT_DOUBLE_EQ(config.stereo_ranger.disp_max_px, 160.0);
    EXPECT_EQ(config.stereo_ranger.coarse_stride, 2);
    EXPECT_EQ(config.stereo_ranger.census_win, 7);
    EXPECT_DOUBLE_EQ(config.stereo_ranger.near_pct, 90.0);
    EXPECT_DOUBLE_EQ(config.stereo_ranger.smooth_alpha, 0.4);
    EXPECT_FALSE(config.runtime.enable_stereo_ranging);
    // 未启用测距时不加载校正矩阵（禁用路径不依赖伴生文件）
    EXPECT_DOUBLE_EQ(config.stereo_rectify.fx_rect_px, 0.0);
    std::filesystem::remove(path);
}

TEST(ConfigTest, StereoRangerLoadsRectifyMatricesWhenEnabled) {
    // 生产配置启用测距：应从 config.json 同目录加载预置矩阵（复算值对拍）
    const auto config =
        drone::config::LoadAppConfig(SourceConfigPath(),
                                     std::string(TEST_SOURCE_DIR));
    ASSERT_TRUE(config.runtime.enable_stereo_ranging);
    EXPECT_EQ(config.stereo_rectify.view_width, 1280);
    EXPECT_EQ(config.stereo_rectify.view_height, 720);
    EXPECT_NEAR(config.stereo_rectify.fx_rect_px, 1057.3349, 1e-3);
    EXPECT_NEAR(config.stereo_rectify.baseline_m, 0.060120, 1e-6);
    EXPECT_NEAR(config.stereo_rectify.fxb_m_px, 63.5665, 1e-3);
    // 基准内参（SBS 右半 = 物理左目）主点应大于辅助（两目 cx 差约 54px）
    EXPECT_GT(config.stereo_rectify.k_ref[2], config.stereo_rectify.k_aux[2]);
}

TEST(ConfigTest, StereoRangerRejectsMissingRectifyMatricesWhenEnabled) {
    json value = ReadSourceConfig();
    value["runtime"]["enable_stereo_ranging"] = true;
    const auto path =
        WriteTemporaryConfig(value, "drone_config_stereo_no_matrices.json");
    // 删除伴生矩阵文件：启用测距时必须启动失败而非静默降级
    std::filesystem::remove(path.parent_path() /
                            "stereo_rectify_uvc_1280x720.json");
    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    CopyRectifyMatricesBeside(path);  // 恢复，避免影响其他用例
    std::filesystem::remove(path);
}

TEST(ConfigTest, VisualFusionParsesAndCopiesFxb) {
    // 生产配置已启用融合：fxb 由校正矩阵装配拷贝（单一真源，不进 JSON）
    const auto config = drone::config::LoadAppConfig(SourceConfigPath(),
                                                     std::string(TEST_SOURCE_DIR));
    ASSERT_TRUE(config.runtime.enable_visual_position_fusion);
    EXPECT_TRUE(config.target_estimator.visual.enabled);
    EXPECT_NEAR(config.target_estimator.visual.fxb_m_px, 63.5665, 1e-3);
    // 子组默认值（生产配置显式给出 mount 全 0 = 准直安装假设）
    EXPECT_DOUBLE_EQ(config.target_estimator.visual.disparity_std_px, 0.5);
    EXPECT_DOUBLE_EQ(config.target_estimator.visual.mount.yaw_deg, 0.0);
}

TEST(ConfigTest, VisualFusionRequiresEstimatorRangingAndPx4) {
    // 依赖链：目标估计器 + 双目测距 + PX4，任一缺失都必须拒绝
    {
        json value = ReadSourceConfig();
        value["runtime"]["enable_target_estimator"] = false;
        const auto path =
            WriteTemporaryConfig(value, "drone_config_fusion_no_estimator.json");
        EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                     std::invalid_argument);
        std::filesystem::remove(path);
    }
    {
        json value = ReadSourceConfig();
        value["runtime"]["enable_stereo_ranging"] = false;
        const auto path =
            WriteTemporaryConfig(value, "drone_config_fusion_no_stereo.json");
        EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                     std::invalid_argument);
        std::filesystem::remove(path);
    }
    {
        json value = ReadSourceConfig();
        value["runtime"]["enable_px4"] = false;
        const auto path =
            WriteTemporaryConfig(value, "drone_config_fusion_no_px4.json");
        EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                     std::invalid_argument);
        std::filesystem::remove(path);
    }
}

TEST(ConfigTest, VisualFusionRejectsInvalidValues) {
    {
        json value = ReadSourceConfig();
        value["target_estimator"]["visual"]["disparity_std_px"] = 0.0;
        const auto path =
            WriteTemporaryConfig(value, "drone_config_fusion_disp_std.json");
        EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                     std::invalid_argument);
        std::filesystem::remove(path);
    }
    {
        json value = ReadSourceConfig();
        value["target_estimator"]["visual"]["min_confidence"] = 1.5;
        const auto path =
            WriteTemporaryConfig(value, "drone_config_fusion_conf_invalid.json");
        EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                     std::invalid_argument);
        std::filesystem::remove(path);
    }
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

TEST(ConfigTest, StereoRangerRejectsInvalidValues) {
    {
        json value = ReadSourceConfig();
        value["stereo_ranger"]["disp_min_px"] = 0.0;
        const auto path =
            WriteTemporaryConfig(value, "drone_config_stereo_dispmin_invalid.json");
        EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                     std::invalid_argument);
        std::filesystem::remove(path);
    }
    {
        json value = ReadSourceConfig();
        value["stereo_ranger"]["disp_max_px"] = 8.0;  // 必须大于 disp_min
        const auto path =
            WriteTemporaryConfig(value, "drone_config_stereo_dispmax_invalid.json");
        EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                     std::invalid_argument);
        std::filesystem::remove(path);
    }
    {
        json value = ReadSourceConfig();
        value["stereo_ranger"]["census_win"] = 4;  // 仅支持 3/5/7
        const auto path =
            WriteTemporaryConfig(value, "drone_config_stereo_census_invalid.json");
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

TEST(ConfigTest, ControllerAndHandoverDefaultsParse) {
    // 生产配置尚未声明 controller/handover 节：两节缺席时必须回退默认值
    const auto config = drone::config::LoadAppConfig(SourceConfigPath(), "/opt/drone");
    EXPECT_EQ(config.controller.period, std::chrono::milliseconds(50));
    EXPECT_DOUBLE_EQ(config.controller.max_speed_mps, 2.0);
    EXPECT_EQ(config.handover.lock_frames, 10u);
    EXPECT_EQ(config.handover.exit_lost_frames, 30u);
    EXPECT_EQ(config.handover.distance_lost_ms, std::chrono::milliseconds(3000));
}

TEST(ConfigTest, RejectsNonPositiveHandoverDistanceLost) {
    // S7 测距失效退出计时门限必须为正：0 → 拒绝（否则进 S7 立即超时退出）
    json value = ReadSourceConfig();
    value["handover"]["distance_lost_ms"] = 0;
    const auto path =
        WriteTemporaryConfig(value, "drone_config_handover_distance_bad.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsHandoverExitNotGreaterThanEnter) {
    // 迟滞关系强制：lock_frames=20、exit_lost_frames=10 → 拒绝
    json value = ReadSourceConfig();
    value["handover"]["lock_frames"] = 20;
    value["handover"]["exit_lost_frames"] = 10;
    const auto path = WriteTemporaryConfig(value, "drone_config_handover_bad.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
    std::filesystem::remove(path);
}

TEST(ConfigTest, RejectsNonPositiveControllerLimits) {
    // max_speed_mps=0 → 拒绝
    json value = ReadSourceConfig();
    value["controller"]["max_speed_mps"] = 0.0;
    const auto path = WriteTemporaryConfig(value, "drone_config_controller_bad.json");

    EXPECT_THROW((void)drone::config::LoadAppConfig(path.string(), "/opt/drone"),
                 std::invalid_argument);
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
