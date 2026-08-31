// =============================================================================
// mavlink_command_gateway_test.cpp —— MavlinkCommandGateway 单元测试
//
// 覆盖（物理追踪思路 §2.2、§4.2，A2 掩码三分支）：
//   ID 84 速度模式的 type_mask 选择（角速率非零 → 速率掩码；航向角非零 →
//   航向掩码；两者均零 → 速率掩码配零角速率，保持航向）、
//   速度/航向字段编码、刹车消息、坐标基准、传输失败传播。
//
// 构建约束：本文件**不得**包含 common/mavlink.h（MAVLINK_HELPER 默认空宏，
// 多 TU 包含会多重定义），故按 MAVLink v2 线格式手工解码：
//   帧头 10 字节（STX=0xFD, len, flags×2, seq, sys, comp, msgid×3 LE），
//   载荷按字段尺寸降序排列（ID 84：time,x,y,z,vx,vy,vz,afx,afy,afz,
//   yaw,yaw_rate 均 4 字节在前，随后 u16 type_mask，再 target_system、
//   target_component、coordinate_frame 三个 u8）。
// =============================================================================

#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "visual_track_control/i_command_transport.h"
#include "visual_track_control/mavlink_command_gateway.h"
#include "visual_track_control/vtc_types.h"

namespace {

using drone::vtc::ControlOutput;
using drone::vtc::ICommandTransport;
using drone::vtc::MavlinkCommandGateway;

// ---- MAVLink v2 帧布局常量（见文件头说明） ----
constexpr std::uint8_t kMavlinkV2Stx = 0xFD;
constexpr std::size_t kHeaderLen = 10;
constexpr std::size_t kMsgId84PayloadLen = 53;

// ---- ID 84 载荷字段偏移（相对载荷起点） ----
constexpr std::size_t kOffVx = 16;
constexpr std::size_t kOffVy = 20;
constexpr std::size_t kOffVz = 24;
constexpr std::size_t kOffYaw = 40;
constexpr std::size_t kOffYawRate = 44;
constexpr std::size_t kOffTypeMask = 48;
// u8 三字段线序（生成头 mavlink_msg_set_position_target_local_ned.h 为准）
constexpr std::size_t kOffTargetSystem = 50;
constexpr std::size_t kOffTargetComponent = 51;
constexpr std::size_t kOffCoordFrame = 52;

// ---- 预期掩码值（与实现中常量一致：置位 = 忽略） ----
// 基础 0x01C7（忽略位置与加速度）| 航向相关位
constexpr std::uint16_t kExpectMaskYaw = 0x01C7 | 0x0800;      // 忽略 yaw_rate
constexpr std::uint16_t kExpectMaskYawRate = 0x01C7 | 0x0400;  // 忽略 yaw
constexpr std::uint16_t kExpectMaskBrake = 0x01C7 | 0x0C00;    // 两者均忽略

/// 捕获式传输：记录最后一次发送的字节流，可注入失败
class CaptureTransport : public ICommandTransport {
public:
    bool Send(const std::uint8_t* data, std::size_t len) override {
        ++send_count_;
        last_.assign(data, data + len);
        return succeed_;
    }

    void SetSucceed(bool ok) { succeed_ = ok; }
    int send_count() const { return send_count_; }
    const std::vector<std::uint8_t>& last() const { return last_; }

private:
    bool succeed_ = true;
    int send_count_ = 0;
    std::vector<std::uint8_t> last_;
};

float ReadFloat(const std::vector<std::uint8_t>& buf, std::size_t payload_off) {
    float v;
    std::memcpy(&v, buf.data() + kHeaderLen + payload_off, sizeof(v));
    return v;
}

std::uint16_t ReadU16(const std::vector<std::uint8_t>& buf, std::size_t payload_off) {
    return static_cast<std::uint16_t>(buf[kHeaderLen + payload_off]) |
           (static_cast<std::uint16_t>(buf[kHeaderLen + payload_off + 1]) << 8);
}

void ExpectId84Header(const std::vector<std::uint8_t>& buf) {
    ASSERT_GE(buf.size(), kHeaderLen + kMsgId84PayloadLen + 2);  // +CRC
    EXPECT_EQ(buf[0], kMavlinkV2Stx);
    EXPECT_EQ(buf[1], kMsgId84PayloadLen);
    // msgid 24 位小端（字节 7-9）= 84
    EXPECT_EQ(buf[7], 84);
    EXPECT_EQ(buf[8], 0);
    EXPECT_EQ(buf[9], 0);
}

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

TEST(MavlinkGatewayTest, YawRateModeSelectsRateMask) {
    CaptureTransport transport;
    MavlinkCommandGateway gateway(transport, 1, 2, 3, 4);

    ControlOutput out;
    out.mode = drone::vtc::ControlMode::kVelocityHeading;
    out.vx_mps = 1.5;
    out.vy_mps = -0.5;
    out.vz_mps = 0.25;
    out.yaw_rate_dps = 5.0;
    out.yaw_deg = 0.0;
    ASSERT_TRUE(gateway.SendVelocityHeading(out, 12345));

    const auto& buf = transport.last();
    ExpectId84Header(buf);
    EXPECT_EQ(ReadU16(buf, kOffTypeMask), kExpectMaskYawRate);
    EXPECT_FLOAT_EQ(ReadFloat(buf, kOffVx), 1.5F);
    EXPECT_FLOAT_EQ(ReadFloat(buf, kOffVy), -0.5F);
    EXPECT_FLOAT_EQ(ReadFloat(buf, kOffVz), 0.25F);
    EXPECT_FLOAT_EQ(ReadFloat(buf, kOffYawRate), static_cast<float>(5.0 * kDegToRad));
    EXPECT_EQ(buf[kHeaderLen + kOffCoordFrame], 1);       // MAV_FRAME_LOCAL_NED
    EXPECT_EQ(buf[kHeaderLen + kOffTargetSystem], 3);     // target_system
    EXPECT_EQ(buf[kHeaderLen + kOffTargetComponent], 4);  // target_component
}

TEST(MavlinkGatewayTest, YawOnlySelectsYawMask) {
    CaptureTransport transport;
    MavlinkCommandGateway gateway(transport, 1, 2, 3, 4);

    ControlOutput out;
    out.mode = drone::vtc::ControlMode::kVelocityHeading;
    out.yaw_deg = 10.0;
    out.yaw_rate_dps = 0.0;
    ASSERT_TRUE(gateway.SendVelocityHeading(out, 1));

    const auto& buf = transport.last();
    ExpectId84Header(buf);
    EXPECT_EQ(ReadU16(buf, kOffTypeMask), kExpectMaskYaw);
    EXPECT_FLOAT_EQ(ReadFloat(buf, kOffYaw), static_cast<float>(10.0 * kDegToRad));
}

TEST(MavlinkGatewayTest, BothZeroKeepsHeadingViaRateMask) {
    // A2 回归：旧实现在零角速率时落到航向掩码，等于误发绝对航向 0（正北）。
    // 正确行为：速率掩码 + 零角速率 = 保持当前航向。
    CaptureTransport transport;
    MavlinkCommandGateway gateway(transport, 1, 2, 3, 4);

    ControlOutput out;
    out.mode = drone::vtc::ControlMode::kVelocityHeading;
    out.vx_mps = 1.0;
    ASSERT_TRUE(gateway.SendVelocityHeading(out, 1));

    const auto& buf = transport.last();
    ExpectId84Header(buf);
    EXPECT_EQ(ReadU16(buf, kOffTypeMask), kExpectMaskYawRate);
    EXPECT_FLOAT_EQ(ReadFloat(buf, kOffYawRate), 0.0F);
}

TEST(MavlinkGatewayTest, BrakeHoverSendsZeroVelocityAllIgnored) {
    CaptureTransport transport;
    MavlinkCommandGateway gateway(transport, 1, 2, 3, 4);
    ASSERT_TRUE(gateway.SendBrakeHover(777));

    const auto& buf = transport.last();
    ExpectId84Header(buf);
    EXPECT_EQ(ReadU16(buf, kOffTypeMask), kExpectMaskBrake);
    EXPECT_FLOAT_EQ(ReadFloat(buf, kOffVx), 0.0F);
    EXPECT_FLOAT_EQ(ReadFloat(buf, kOffVy), 0.0F);
    EXPECT_FLOAT_EQ(ReadFloat(buf, kOffVz), 0.0F);
}

TEST(MavlinkGatewayTest, TransportFailurePropagates) {
    CaptureTransport transport;
    transport.SetSucceed(false);
    MavlinkCommandGateway gateway(transport, 1, 2, 3, 4);

    ControlOutput out;
    out.mode = drone::vtc::ControlMode::kVelocityHeading;
    EXPECT_FALSE(gateway.SendVelocityHeading(out, 1));
    EXPECT_FALSE(gateway.SendBrakeHover(1));
    EXPECT_EQ(transport.send_count(), 2);
}

}  // namespace
