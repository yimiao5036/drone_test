#include "video/drm_nv12_transfer.h"

#include <cstring>
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
constexpr std::uint32_t kDrmFormatNv16 = MakeFourcc('N', 'V', '1', '6');
constexpr std::uint64_t kDrmFormatModifierLinear = 0;

void SetError(std::string* output, std::string message) {
    if (output != nullptr) {
        *output = std::move(message);
    }
}

/// 单对象、线性、连续半规划 YUV 布局的公共校验。
/// uv_rows 为 UV 面行数：NV12=height/2，NV16=height（垂直满采样）。
bool ValidateLinearDrmSpLayoutCommon(const DrmNv12Layout& layout,
                                     std::uint32_t expected_fourcc,
                                     const char* format_name,
                                     std::uint32_t uv_rows,
                                     std::string* error_message) {
    if (layout.fd < 0 || layout.width == 0 || layout.height == 0 ||
        (layout.width % 2U) != 0U || (layout.height % 2U) != 0U) {
        SetError(error_message,
                 std::string("DMA-BUF fd或") + format_name + "可见尺寸非法");
        return false;
    }
    if (layout.drm_format != expected_fourcc) {
        SetError(error_message,
                 std::string("DRM图层不是") + format_name + "格式");
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
        SetError(error_message,
                 std::string(format_name) + " Y/UV pitch不一致或小于可见宽度");
        return false;
    }

    const auto pitch = static_cast<std::size_t>(layout.y_pitch);
    if (layout.height > std::numeric_limits<std::size_t>::max() / pitch) {
        SetError(error_message, std::string(format_name) + "平面尺寸计算溢出");
        return false;
    }
    const std::size_t expected_uv_offset = pitch * layout.height;
    if (layout.uv_offset < 0 ||
        static_cast<std::size_t>(layout.uv_offset) != expected_uv_offset) {
        SetError(error_message, "UV offset不等于Y pitch乘可见高度");
        return false;
    }

    if (uv_rows > (std::numeric_limits<std::size_t>::max() - expected_uv_offset) /
                      pitch) {
        SetError(error_message, std::string(format_name) + "对象大小计算溢出");
        return false;
    }
    const std::size_t required_size = expected_uv_offset + pitch * uv_rows;
    if (layout.object_size < required_size) {
        SetError(error_message,
                 std::string("DMA-BUF对象大小不足以容纳可见") + format_name +
                     "平面");
        return false;
    }

    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
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
        const DrmSpFormat format = ClassifyDrmSpFormat(source.drm_format);
        const char* operation = nullptr;
        switch (format) {
            case DrmSpFormat::kNv12:
                if (!ValidateLinearDrmNv12Layout(source, error_message)) {
                    return DrmNv12TransferStatus::kUnsupportedLayout;
                }
                operation = "复制";
                break;
            case DrmSpFormat::kNv16:
                if (!ValidateLinearDrmNv16Layout(source, error_message)) {
                    return DrmNv12TransferStatus::kUnsupportedLayout;
                }
                operation = "NV16转NV12";
                break;
            default:
                SetError(error_message,
                         "DRM图层格式不支持RGA直传：仅NV12/NV16");
                return DrmNv12TransferStatus::kUnsupportedLayout;
        }
        if (destination == nullptr || destination_stride < source.width) {
            SetError(error_message, "RGA DMA目标缓冲为空或stride小于图像宽度");
            return DrmNv12TransferStatus::kUnsupportedLayout;
        }

        const int rga_source_format =
            (format == DrmSpFormat::kNv16) ? RK_FORMAT_YCbCr_422_SP
                                           : RK_FORMAT_YCbCr_420_SP;
        rga_buffer_t source_buffer = wrapbuffer_fd(
            source.fd, static_cast<int>(source.width),
            static_cast<int>(source.height), rga_source_format,
            static_cast<int>(source.y_pitch), static_cast<int>(source.height));
        rga_buffer_t destination_buffer = wrapbuffer_virtualaddr(
            reinterpret_cast<void*>(destination), static_cast<int>(source.width),
            static_cast<int>(source.height), RK_FORMAT_YCbCr_420_SP,
            static_cast<int>(destination_stride), static_cast<int>(source.height));

        // NV12 同格式纯拷贝；NV16 需 422SP→420SP 色度降采样转换。
        // im2d 同步语义：返回即完成，之后源 DMA-BUF 帧方可释放。
        const IM_STATUS status =
            (format == DrmSpFormat::kNv16)
                ? imcvtcolor(source_buffer, destination_buffer,
                             RK_FORMAT_YCbCr_422_SP, RK_FORMAT_YCbCr_420_SP)
                : imcopy(source_buffer, destination_buffer);
        if (status != IM_STATUS_SUCCESS) {
            SetError(error_message, std::string("RGA DMA-BUF") + operation +
                                        "失败: " + imStrError(status));
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

DrmSpFormat ClassifyDrmSpFormat(std::uint32_t drm_format_fourcc) {
    switch (drm_format_fourcc) {
        case kDrmFormatNv12:
            return DrmSpFormat::kNv12;
        case kDrmFormatNv16:
            return DrmSpFormat::kNv16;
        default:
            return DrmSpFormat::kUnsupported;
    }
}

bool ValidateLinearDrmNv12Layout(const DrmNv12Layout& layout,
                                 std::string* error_message) {
    return ValidateLinearDrmSpLayoutCommon(layout, kDrmFormatNv12, "NV12",
                                           layout.height / 2U, error_message);
}

bool ValidateLinearDrmNv16Layout(const DrmNv12Layout& layout,
                                 std::string* error_message) {
    return ValidateLinearDrmSpLayoutCommon(layout, kDrmFormatNv16, "NV16",
                                           layout.height, error_message);
}

std::unique_ptr<IDrmNv12Transfer> CreateDrmNv12Transfer() {
#ifdef DRONE_HAVE_RGA_DMA
    return std::make_unique<RgaDrmNv12Transfer>();
#else
    return std::make_unique<UnavailableDrmNv12Transfer>();
#endif
}

}  // namespace drone::video
