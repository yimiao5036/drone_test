/**
 * @file stub_smoke_test.cpp
 * @brief 骨架冒烟测试：验证各部件接口抽象与占位实现
 *
 * 骨架期各部件只有接口与 Stub 占位实现（无业务逻辑）。
 * 本测试验证：所有 Stub 可构造/析构、可 Start/Stop、输出主题可访问、
 * 未实现方法按预期返回默认值，为后续装配联调提供基线。
 *
 * 对应 docs/数据接口文档.md：本测试不验证业务行为（业务逻辑阶段接入）。
 */
#include <gtest/gtest.h>

#include "common/types.h"
#include "communication/ground_station_link.h"
#include "communication/px4_link.h"
#include "communication/serial_port.h"
#include "control/flight_controller.h"
#include "health/health_manager.h"
#include "perception/laser_range_finder.h"
#include "perception/optical_flow_estimator.h"
#include "perception/perception_fusion.h"
#include "perception/target_estimator.h"
#include "perception/yolo_detector.h"
#include "state_machine/mission_state_machine.h"
#include "video/camera_receiver.h"
#include "video/video_decoder.h"
#include "video_transmission/video_sender.h"

namespace {

using namespace drone;

// 生命周期：构造 → Start → Stop → 析构，全部可运行且幂等
TEST(StubSmokeTest, CameraReceiverLifecycle) {
    video::CameraReceiverStub stub;
    EXPECT_FALSE(stub.IsRunning());
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    EXPECT_FALSE(stub.IsConnected());  // 骨架期未实现建连
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
    stub.Stop();  // 幂等
}

TEST(StubSmokeTest, VideoDecoderLifecycle) {
    video::VideoDecoderStub stub;
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    EXPECT_EQ(stub.DecodedFrameCount(), 0u);
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
}

TEST(StubSmokeTest, VideoSenderLifecycle) {
    video_transmission::VideoSenderStub stub;
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
}

TEST(StubSmokeTest, YoloDetectorLifecycle) {
    perception::YoloDetectorStub stub;
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    EXPECT_EQ(stub.InferenceTimeMsAvg(), 0.f);
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
}

TEST(StubSmokeTest, OpticalFlowEstimatorLifecycle) {
    perception::OpticalFlowEstimatorStub stub;
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
}

TEST(StubSmokeTest, LaserRangeFinderLifecycle) {
    perception::LaserRangeFinderStub stub;
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    EXPECT_EQ(stub.LastDistanceM(), 0.f);
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
}

TEST(StubSmokeTest, PerceptionFusionLifecycle) {
    perception::PerceptionFusionStub stub;
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
}

TEST(StubSmokeTest, TargetEstimatorLifecycle) {
    perception::TargetEstimatorStub stub;
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
}

TEST(StubSmokeTest, Px4LinkLifecycleAndUnimplemented) {
    communication::Px4LinkStub stub;
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    EXPECT_FALSE(stub.IsConnected());
    // 命令发送未实现：返回失败
    EXPECT_FALSE(stub.SendCommand(400, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f));
    EXPECT_GT(stub.ErrorCount(), 0u);
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
}

TEST(StubSmokeTest, GroundStationLinkLifecycle) {
    communication::GroundStationLinkStub stub;
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    EXPECT_FALSE(stub.IsConnected());
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
}

TEST(StubSmokeTest, MissionStateMachineLifecycle) {
    state_machine::MissionStateMachineStub stub;
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    EXPECT_EQ(stub.CurrentState(), common::MissionState::kBoot);
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
}

TEST(StubSmokeTest, FlightControllerLifecycle) {
    control::FlightControllerStub stub;
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
}

TEST(StubSmokeTest, HealthManagerLifecycleAndUnimplemented) {
    health::HealthManagerStub stub;
    EXPECT_TRUE(stub.Start());
    EXPECT_TRUE(stub.IsRunning());
    // 数据源注册未实现：返回失败
    EXPECT_FALSE(stub.RegisterSource("px4", 1000));
    EXPECT_GT(stub.ErrorCount(), 0u);
    stub.Stop();
    EXPECT_FALSE(stub.IsRunning());
}

// 输出主题可访问（装配验证的基础）
TEST(StubSmokeTest, OutputTopicsAreAccessible) {
    video::CameraReceiverStub camera;
    video::VideoDecoderStub decoder;
    perception::YoloDetectorStub yolo;
    perception::LaserRangeFinderStub laser;
    communication::Px4LinkStub px4;
    state_machine::MissionStateMachineStub sm;

    // 发布默认消息不抛异常（返回值仅用于验证可发布）
    (void)camera.StreamOutput().Emplace();
    (void)decoder.FrameOutput().Emplace();
    (void)yolo.DetectionOutput().Emplace();
    (void)laser.RangeOutput().Emplace();
    (void)px4.StateOutput().Emplace();
    (void)sm.IntentOutput().Emplace();
    (void)sm.StatusOutput().Emplace();
}

// 公共消息类型：默认构造可编译、字段语义正确
TEST(StubSmokeTest, CommonTypesDefaultValues) {
    common::ControlIntent intent;
    EXPECT_EQ(intent.type, common::ControlIntentType::kNone);

    common::FlightStateSnapshot snapshot;
    EXPECT_FALSE(snapshot.landed);
    EXPECT_FALSE(snapshot.landed_state_valid);
    EXPECT_FALSE(snapshot.connected);

    common::MissionStatus status;
    EXPECT_EQ(status.state, common::MissionState::kBoot);

    // Topic 名称常量非空
    EXPECT_STRNE(common::topics::kCameraStream, "");
    EXPECT_STRNE(common::topics::kDecodedFrame, "");
}

// SerialPort 配置校验（不触碰真实硬件）
TEST(StubSmokeTest, SerialPortConfigValidation) {
    communication::SerialPortConfig valid;
    valid.device = "/dev/ttyS0";
    valid.baud_rate = 115200;
    EXPECT_NO_THROW(valid.Validate());

    communication::SerialPortConfig bad_baud = valid;
    bad_baud.baud_rate = 0;
    EXPECT_THROW(bad_baud.Validate(), std::invalid_argument);

    communication::SerialPortConfig bad_parity = valid;
    bad_parity.parity = 'X';
    EXPECT_THROW(bad_parity.Validate(), std::invalid_argument);

    communication::SerialPortConfig bad_data_bits = valid;
    bad_data_bits.data_bits = 9;
    EXPECT_THROW(bad_data_bits.Validate(), std::invalid_argument);
}

}  // namespace
