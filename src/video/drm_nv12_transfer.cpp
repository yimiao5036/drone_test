#include "video/drm_nv12_transfer.h"

#include <limits>
#include <utility>

#ifdef DRONE_HAVE_RGA_DMA
#include <rga/im2d.hpp>
#include <rga/rga.h>
#endif

namespace drone::video {
namespace {

constexpr std::uint32_t MakeFourcc(char a, char b, char c, char d) {
    return static_cast<std::uint32_t>(static_cast<unsigned char>(a)) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(b)) << 8U) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(c)) << 16U) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(d)) << 24U);
}

constexpr std::uint32_t kDrmFormatNv12 = MakeFourcc('N', 'V', '1', '2');
constexpr std::uint64_t kDrmFormatModifierLinear = 0;

void SetError(std::string* output, std::string message) {
    if (output != nullptr) {
        *output = std::move(message);
    }
}

class UnavailableDrmNv12Transfer final : public IDrmNv12Transfer {
public:
    bool IsAvailable() const override { return false; }

    DrmNv12TransferStatus CopyToNv12(
        const DrmNv12Layout&, std::byte*, std::uint32_t,
        std::string* error_message) override {
        SetError(error_message, "当前构建未启用RGA DMA-BUF转存");
        return DrmNv12TransferStatus::kUnavailable;
    }
};

#ifdef DRONE_HAVE_RGA_DMA
class RgaDrmNv12Transfer final : public IDrmNv12Transfer {
public:
    bool IsAvailable() const override { return true; }

    DrmNv12TransferStatus CopyToNv12(
        const DrmNv12Layout& source, std::byte* destination,
        std::uint32_t destination_stride,
        std::string* error_message) override {
        if (!ValidateLinearDrmNv12Layout(source, error_message)) {
            return DrmNv12TransferStatus::kUnsupportedLayout;
        }
        if (destination == nullptr || destination_stride < source.width) {
            SetError(error_message, "RGA DMA目标缓冲为空或stride小于图像宽度");
            return DrmNv12TransferStatus::kUnsupportedLayout;
        }

        rga_buffer_t source_buffer = wrapbuffer_fd(
            source.fd, static_cast<int>(source.width),
            static_cast<int>(source.height), RK_FORMAT_YCbCr_420_SP,
            static_cast<int>(source.y_pitch), static_cast<int>(source.height));
        rga_buffer_t destination_buffer = wrapbuffer_virtualaddr(
            reinterpret_cast<void*>(destination), static_cast<int>(source.width),
            static_cast<int>(source.height), RK_FORMAT_YCbCr_420_SP,
            static_cast<int>(destination_stride), static_cast<int>(source.height));

        const IM_STATUS status = imcopy(source_buffer, destination_buffer);
        if (status != IM_STATUS_SUCCESS) {
            SetError(error_message,
                     std::string("RGA DMA-BUF复制失败: ") + imStrError(status));
            return DrmNv12TransferStatus::kTransferFailed;
        }
        if (error_message != nullptr) {
            error_message->clear();
        }
        return DrmNv12TransferStatus::kSuccess;
    }
};
#endif

}  // namespace

bool ValidateLinearDrmNv12Layout(const DrmNv12Layout& layout,
                                 std::string* error_message) {
    if (layout.fd < 0 || layout.width == 0 || layout.height == 0 ||
        (layout.width % 2U) != 0U || (layout.height % 2U) != 0U) {
        SetError(error_message, "DMA-BUF fd或NV12可见尺寸非法");
        return false;
    }
    if (layout.drm_format != kDrmFormatNv12) {
        SetError(error_message, "DRM图层不是NV12格式");
        return false;
    }
    if (layout.format_modifier != kDrmFormatModifierLinear) {
        SetError(error_message, "当前只支持线性DRM modifier");
        return false;
    }
    if (layout.y_object_index != 0 || layout.uv_object_index != 0 ||
        layout.y_offset != 0) {
        SetError(error_message, "当前只支持Y/UV位于同一对象且Y offset为0");
        return false;
    }
    if (layout.y_pitch < static_cast<std::ptrdiff_t>(layout.width) ||
        layout.uv_pitch != layout.y_pitch) {
        SetError(error_message, "NV12 Y/UV pitch不一致或小于可见宽度");
        return false;
    }

    const auto pitch = static_cast<std::size_t>(layout.y_pitch);
    if (layout.height > std::numeric_limits<std::size_t>::max() / pitch) {
        SetError(error_message, "NV12平面尺寸计算溢出");
        return false;
    }
    const std::size_t expected_uv_offset = pitch * layout.height;
    if (layout.uv_offset < 0 ||
        static_cast<std::size_t>(layout.uv_offset) != expected_uv_offset) {
        SetError(error_message, "UV offset不等于Y pitch乘可见高度");
        return false;
    }

    const std::size_t uv_rows = layout.height / 2U;
    if (uv_rows > (std::numeric_limits<std::size_t>::max() - expected_uv_offset) /
                      pitch) {
        SetError(error_message, "NV12对象大小计算溢出");
        return false;
    }
    const std::size_t required_size = expected_uv_offset + pitch * uv_rows;
    if (layout.object_size < required_size) {
        SetError(error_message, "DMA-BUF对象大小不足以容纳可见NV12平面");
        return false;
    }

    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

std::unique_ptr<IDrmNv12Transfer> CreateDrmNv12Transfer() {
#ifdef DRONE_HAVE_RGA_DMA
    return std::make_unique<RgaDrmNv12Transfer>();
#else
    return std::make_unique<UnavailableDrmNv12Transfer>();
#endif
}

}  // namespace drone::video
