/**
 * @file drone_application_test.cpp
 * @brief 主程序集成（DroneApplication）装配回归测试
 *
 * 不启动线程、不触碰硬件，只验证不同运行时开关组合下的构造装配：
 * - 探针式裁剪配置：视频+双目测距开启、视觉监控/视觉跟踪关闭时，
 *   BindTopics 不得解引用未装配的视觉跟踪影子（历史段错误回归：
 *   video_latency_probe 裁剪 visual_monitor 后启动即 SIGSEGV）。
 * - 完整影子配置：视觉监控+视觉跟踪开启时，距离通道正常接线。
 *
 * 真实摄像头/解码/推理/推流行为在香橙派实机验证，不在本测试范围。
 */
#include <chrono>

#include <gtest/gtest.h>

#include "../common/log_capture.h"
#include "application/drone_application.h"
#include "config/config.h"

namespace drone::application {
namespace {

/// 自洽的最小校正矩阵（单位校正，1280×720）：装配测试不跑匹配，
/// 只需通过 StereoRangerCore 构造期的矩阵校验。
perception::StereoRectifyMatrices MakeTestMatrices() {
    perception::StereoRectifyMatrices m;
    m.view_width = 1280;
    m.view_height = 720;
    const double fx = 1000.0;
    const double cx = 639.5;
    const double cy = 359.5;
    const double baseline = 0.06;
    m.k_ref[0] = fx; m.k_ref[4] = fx; m.k_ref[2] = cx; m.k_ref[5] = cy; m.k_ref[8] = 1.0;
    m.k_aux[0] = fx; m.k_aux[4] = fx; m.k_aux[2] = cx; m.k_aux[5] = cy; m.k_aux[8] = 1.0;
    m.r_ref[0] = m.r_ref[4] = m.r_ref[8] = 1.0;
    m.r_aux[0] = m.r_aux[4] = m.r_aux[8] = 1.0;
    m.p_ref[0] = fx; m.p_ref[5] = fx; m.p_ref[2] = cx; m.p_ref[6] = cy; m.p_ref[10] = 1.0;
    m.p_aux[0] = fx; m.p_aux[5] = fx; m.p_aux[2] = cx; m.p_aux[6] = cy; m.p_aux[10] = 1.0;
    m.p_aux[3] = -fx * baseline;
    m.baseline_m = baseline;
    m.fx_rect_px = fx;
    m.fxb_m_px = fx * baseline;
    return m;
}

/// 探针式裁剪配置：仅视频链路 + 双目测距，无视觉监控/视觉跟踪/PX4/地面站。
config::AppConfig MakeProbeLikeConfig() {
    config::AppConfig config;
    config.runtime.enable_video = true;
    config.runtime.enable_px4 = false;
    config.runtime.enable_ground_station = false;
    config.runtime.enable_target_estimator = false;
    config.runtime.enable_visual_monitor = false;
    config.runtime.enable_visual_tracking = false;
    config.runtime.enable_stereo_ranging = true;
    config.runtime.enable_control = false;
    config.video_source.camera_source = "uvc";
    config.video_source.stereo_split = true;
    config.stereo_rectify = MakeTestMatrices();
    return config;
}

// 回归：enable_stereo_ranging=true 而视觉跟踪未装配时，构造不得段错误
// （BindTopics 距离通道曾无条件解引用 visual_tracking_shadow_）。
TEST(DroneApplicationTest, StereoRangingWithoutVisualTrackingConstructs) {
    EXPECT_NO_THROW({
        DroneApplication application(MakeProbeLikeConfig());
    });
}

// 完整影子配置：视觉监控+视觉跟踪开启，距离通道接入视觉跟踪影子。
TEST(DroneApplicationTest, StereoRangingWithVisualTrackingConstructs) {
    auto config = MakeProbeLikeConfig();
    config.runtime.enable_visual_monitor = true;
    config.runtime.enable_visual_tracking = true;
    EXPECT_NO_THROW({
        DroneApplication application(std::move(config));
    });
}

/// 控制面装配配置：PX4+地面站开启（状态机与飞行控制器因此装配），视频链路
/// 关闭；只验证构造装配接线，不启动线程、不触碰硬件。
config::AppConfig MakeControlAssemblyConfig(bool enable_control) {
    config::AppConfig config;
    config.runtime.enable_video = false;
    config.runtime.enable_px4 = true;
    config.runtime.enable_ground_station = true;
    config.runtime.enable_target_estimator = false;
    config.runtime.enable_visual_monitor = false;
    config.runtime.enable_visual_tracking = false;
    config.runtime.enable_stereo_ranging = false;
    config.runtime.enable_control = enable_control;
    return config;
}

// 阶段四装配门禁：enable_control=false 时飞行控制器影子运行——已创建并接入
// 状态机意图（授权链因未注入 PX4 快照源恒不通过），但不绑定 Px4Setpoint、
// 不注入命令接口，且无「正式控制输出已启用」WARN。
TEST(DroneApplicationTest, EnableControlFalseDoesNotBindSetpoint) {
    test::LogCapture capture;
    auto config = MakeControlAssemblyConfig(false);
    config.takeoff.bootstrap_timeout = std::chrono::milliseconds(9000);
    DroneApplication application(std::move(config));
    const auto snapshot = application.ControlAssembly();
    EXPECT_TRUE(snapshot.flight_controller_created);
    EXPECT_TRUE(snapshot.intent_input_bound);
    // 分配输入/回执与 enable_control 无关，装配即接线。
    EXPECT_TRUE(snapshot.assignment_input_bound);
    EXPECT_TRUE(snapshot.assignment_ack_bound);
    EXPECT_FALSE(snapshot.estimation_inputs_bound);
    EXPECT_FALSE(snapshot.px4_snapshot_source_injected);
    EXPECT_FALSE(snapshot.setpoint_bound_to_px4);
    EXPECT_FALSE(snapshot.command_sender_injected);
    // bootstrap 兜底时长来自 takeoff 节，而非 controller 默认值。
    EXPECT_EQ(snapshot.controller_bootstrap_timeout, std::chrono::milliseconds(9000));
    EXPECT_EQ(capture.Count("正式控制输出已启用"), 0);
}

// enable_control=true：完成 PX4 快照源注入、Px4Setpoint 绑定与命令接口注入；
// bootstrap 超时同样取 takeoff 节。
TEST(DroneApplicationTest, EnableControlTrueBindsSetpointAndCommandSender) {
    auto config = MakeControlAssemblyConfig(true);
    config.takeoff.bootstrap_timeout = std::chrono::milliseconds(12000);
    DroneApplication application(std::move(config));
    const auto snapshot = application.ControlAssembly();
    EXPECT_TRUE(snapshot.flight_controller_created);
    EXPECT_TRUE(snapshot.intent_input_bound);
    EXPECT_TRUE(snapshot.px4_snapshot_source_injected);
    EXPECT_TRUE(snapshot.setpoint_bound_to_px4);
    EXPECT_TRUE(snapshot.command_sender_injected);
    EXPECT_EQ(snapshot.controller_bootstrap_timeout, std::chrono::milliseconds(12000));
}

// enable_control=true：装配期打 WARN「正式控制输出已启用」（运行时显性标记）。
TEST(DroneApplicationTest, EnableControlTrueLogsWarnAtStart) {
    test::LogCapture capture;
    DroneApplication application(MakeControlAssemblyConfig(true));
    EXPECT_GE(capture.Count("正式控制输出已启用"), 1);
}

/// 估计输入全装配配置：视频+双目测距+视觉监控+目标估计+PX4+地面站同时开启，
/// 验证 SetEstimationInputs 三主题成组接线。不启动线程、不触碰硬件。
config::AppConfig MakeEstimationAssemblyConfig() {
    auto config = MakeProbeLikeConfig();
    config.runtime.enable_px4 = true;
    config.runtime.enable_ground_station = true;
    config.runtime.enable_target_estimator = true;
    config.runtime.enable_visual_monitor = true;
    config.runtime.enable_control = false;
    return config;
}

// 估计器/视觉稳定性/双目测距三者同时装配时，S5 位置引导输入成组绑定；
// 分配输入/回执同样接线；enable_control=false 仍不注入命令接口。
TEST(DroneApplicationTest, EstimationInputsBoundWhenAllSourcesPresent) {
    DroneApplication application(MakeEstimationAssemblyConfig());
    const auto snapshot = application.ControlAssembly();
    EXPECT_TRUE(snapshot.estimation_inputs_bound);
    EXPECT_TRUE(snapshot.assignment_input_bound);
    EXPECT_TRUE(snapshot.assignment_ack_bound);
    EXPECT_FALSE(snapshot.command_sender_injected);
    EXPECT_FALSE(snapshot.setpoint_bound_to_px4);
}

}  // namespace
}  // namespace drone::application
