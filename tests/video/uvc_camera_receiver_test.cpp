/**
 * @file uvc_camera_receiver_test.cpp
 * @brief USB UVC 双目相机接收器（UvcCameraReceiver）单元测试
 *
 * WSL2 开发机无 USB 相机，本测试只覆盖：
 * - 配置校验：非法配置构造抛出
 * - 错误路径：设备不存在时 Start 返回 false、计数与状态正确
 * - 生命周期：Stop 幂等、Start 失败后可重试且不崩
 *
 * 采集正确性（格式回读、DQBUF 逐帧、时间戳语义）由香橙派板上验收覆盖
 * （v4l2 行为已被 stereo_camera_probe 探测报告实证）。
 */
#include <chrono>

#include <gtest/gtest.h>

#include "video/uvc_camera_receiver.h"

namespace drone::video {
namespace {

TEST(UvcCameraReceiverTest, RejectsInvalidConfig) {
    UvcCameraReceiverConfig config;
    config.device.clear();
    EXPECT_THROW(UvcCameraReceiver receiver(config), std::invalid_argument);

    config.device = "/dev/video0";
    config.width = 0;
    EXPECT_THROW(UvcCameraReceiver receiver(config), std::invalid_argument);

    config.width = 2560;
    config.height = 0;
    EXPECT_THROW(UvcCameraReceiver receiver(config), std::invalid_argument);

    config.height = 720;
    config.fps = 0;
    EXPECT_THROW(UvcCameraReceiver receiver(config), std::invalid_argument);

    config.fps = 30;
    config.buffer_count = 1;  // mmap 双缓冲下限
    EXPECT_THROW(UvcCameraReceiver receiver(config), std::invalid_argument);

    config.buffer_count = 4;
    config.reconnect_delay = std::chrono::milliseconds(-1);
    EXPECT_THROW(UvcCameraReceiver receiver(config), std::invalid_argument);
}

TEST(UvcCameraReceiverTest, StartFailsWhenDeviceMissing) {
    UvcCameraReceiverConfig config;
    config.device = "/dev/nonexistent_uvc_node_xyz";
    UvcCameraReceiver receiver(config);

    EXPECT_FALSE(receiver.IsRunning());
    EXPECT_FALSE(receiver.IsConnected());
    EXPECT_FALSE(receiver.Start());  // 设备不存在：Start 失败
    EXPECT_FALSE(receiver.IsRunning());
    EXPECT_FALSE(receiver.IsConnected());
    EXPECT_GT(receiver.ErrorCount(), 0u);
    EXPECT_EQ(receiver.ConnectCount(), 0u);
    EXPECT_EQ(receiver.ReceivedBytes(), 0u);

    // Start 失败后可安全重试与停止，不崩不漏
    EXPECT_FALSE(receiver.Start());
    receiver.Stop();
    EXPECT_FALSE(receiver.IsRunning());
    receiver.Stop();  // 幂等
}

TEST(UvcCameraReceiverTest, StopIsIdempotentWithoutStart) {
    UvcCameraReceiverConfig config;
    config.device = "/dev/nonexistent_uvc_node_xyz";
    UvcCameraReceiver receiver(config);
    receiver.Stop();  // 未 Start 调 Stop：幂等
    EXPECT_FALSE(receiver.IsRunning());
    EXPECT_EQ(receiver.ErrorCount(), 0u);
    // 输出主题可用（装配期即暴露）
    auto sub = receiver.StreamOutput().Subscribe(1);
    EXPECT_TRUE(sub.IsOpen());
}

}  // namespace
}  // namespace drone::video
