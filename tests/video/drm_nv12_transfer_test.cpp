#include <gtest/gtest.h>

#include <cstddef>
#include <string>

#include "video/drm_nv12_transfer.h"

namespace drone::video {
namespace {

constexpr std::uint32_t MakeFourcc(char a, char b, char c, char d) {
    return static_cast<std::uint32_t>(static_cast<unsigned char>(a)) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(b)) << 8U) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(c)) << 16U) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(d)) << 24U);
}

DrmNv12Layout MakeValidLayout() {
    DrmNv12Layout layout;
    layout.fd = 41;
    layout.object_size = 1658880;
    layout.format_modifier = 0;
    layout.drm_format = MakeFourcc('N', 'V', '1', '2');
    layout.width = 1280;
    layout.height = 720;
    layout.y_object_index = 0;
    layout.y_offset = 0;
    layout.y_pitch = 1280;
    layout.uv_object_index = 0;
    layout.uv_offset = 921600;
    layout.uv_pitch = 1280;
    return layout;
}

TEST(DrmNv12TransferTest, AcceptsMeasuredOrangePiLayout) {
    std::string error;
    EXPECT_TRUE(ValidateLinearDrmNv12Layout(MakeValidLayout(), &error));
    EXPECT_TRUE(error.empty());
}

TEST(DrmNv12TransferTest, RejectsNonLinearModifier) {
    auto layout = MakeValidLayout();
    layout.format_modifier = 1;
    EXPECT_FALSE(ValidateLinearDrmNv12Layout(layout));
}

TEST(DrmNv12TransferTest, RejectsUnexpectedUvOffset) {
    auto layout = MakeValidLayout();
    layout.uv_offset += 64;
    EXPECT_FALSE(ValidateLinearDrmNv12Layout(layout));
}

TEST(DrmNv12TransferTest, RejectsTooSmallObject) {
    auto layout = MakeValidLayout();
    layout.object_size = 1000;
    EXPECT_FALSE(ValidateLinearDrmNv12Layout(layout));
}

}  // namespace
}  // namespace drone::video
