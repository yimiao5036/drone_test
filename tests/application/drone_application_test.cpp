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
#include <gtest/gtest.h>

#include "application/drone_application.h"
#include "config/config.h"

namespace drone::application {
namespace {

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

}  // namespace
}  // namespace drone::application
