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

/// NV16（YCbCr422SP）合法布局：UV 面行数 = 高度，对象容量按 pitch×height×2。
DrmNv12Layout MakeValidNv16Layout() {
    auto layout = MakeValidLayout();
    layout.drm_format = MakeFourcc('N', 'V', '1', '6');
    layout.object_size = 1280 * 720 * 2;  // 1843200
    return layout;
}

TEST(DrmNv12TransferTest, ClassifiesDrmSpFormatFourcc) {
    EXPECT_EQ(ClassifyDrmSpFormat(MakeFourcc('N', 'V', '1', '2')),
              DrmSpFormat::kNv12);
    EXPECT_EQ(ClassifyDrmSpFormat(MakeFourcc('N', 'V', '1', '6')),
              DrmSpFormat::kNv16);
    EXPECT_EQ(ClassifyDrmSpFormat(MakeFourcc('Y', 'U', 'Y', 'V')),
              DrmSpFormat::kUnsupported);
    EXPECT_EQ(ClassifyDrmSpFormat(0), DrmSpFormat::kUnsupported);
}

TEST(DrmNv12TransferTest, AcceptsValidNv16Layout) {
    std::string error;
    EXPECT_TRUE(ValidateLinearDrmNv16Layout(MakeValidNv16Layout(), &error));
    EXPECT_TRUE(error.empty());
}

TEST(DrmNv12TransferTest, Nv16RejectsNv12SizeRule) {
    // NV12 的容量规则（pitch×height×1.5）用于 NV16 会拒绝满采样对象：
    // 双方校验函数各自按自身规则工作，互不通用。
    auto layout = MakeValidNv16Layout();
    EXPECT_FALSE(ValidateLinearDrmNv12Layout(layout));  // fourcc 不符
    layout.object_size = 1280 * 720 * 3 / 2;            // NV12 容量对 NV16 不足
    EXPECT_FALSE(ValidateLinearDrmNv16Layout(layout));
}

TEST(DrmNv12TransferTest, Nv16RejectsUnexpectedUvOffset) {
    auto layout = MakeValidNv16Layout();
    layout.uv_offset += 64;
    EXPECT_FALSE(ValidateLinearDrmNv16Layout(layout));
}

TEST(DrmNv12TransferTest, Nv16RejectsNonLinearModifier) {
    auto layout = MakeValidNv16Layout();
    layout.format_modifier = 1;
    EXPECT_FALSE(ValidateLinearDrmNv16Layout(layout));
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
