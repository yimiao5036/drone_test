#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace drone::video {

/// 从FFmpeg AVDRMFrameDescriptor提取的单对象半规划YUV DMA-BUF布局。
/// 支持 NV12（YCbCr420SP）与 NV16（YCbCr422SP）两种源格式；
/// NV16 时 UV 面行数 = 高度（垂直满采样），对象容量按 pitch×height×2 计。
struct DrmNv12Layout {
    int fd = -1;
    std::size_t object_size = 0;
    std::uint64_t format_modifier = 0;
    std::uint32_t drm_format = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    int y_object_index = -1;
    std::ptrdiff_t y_offset = -1;
    std::ptrdiff_t y_pitch = 0;
    int uv_object_index = -1;
    std::ptrdiff_t uv_offset = -1;
    std::ptrdiff_t uv_pitch = 0;
};

enum class DrmNv12TransferStatus : std::uint8_t {
    kSuccess = 0,
    kUnavailable,
    kUnsupportedLayout,
    kTransferFailed,
};

/// DRM 图层格式分类：RGA 直传支持的半规划 YUV 子集。
enum class DrmSpFormat : std::uint8_t {
    kUnsupported = 0,
    kNv12,
    kNv16,
};

/// 按 DRM fourcc 分类；不支持的分派到 kUnsupported。纯函数，无硬件依赖。
DrmSpFormat ClassifyDrmSpFormat(std::uint32_t drm_format_fourcc);

/// DRM DMA-BUF到CPU可访问NV12目标缓冲的硬件复制/转换接口。
/// 目标恒为 NV12；源支持 NV12（纯拷贝）与 NV16（422SP→420SP 转换），
/// 按 source.drm_format 内部分派。
class IDrmNv12Transfer {
public:
    virtual ~IDrmNv12Transfer() = default;

    virtual bool IsAvailable() const = 0;
    virtual DrmNv12TransferStatus CopyToNv12(
        const DrmNv12Layout& source,
        std::byte* destination,
        std::uint32_t destination_stride,
        std::string* error_message) = 0;
};

/// 校验是否为当前RGA适配器支持的单对象、线性、连续NV12布局。
bool ValidateLinearDrmNv12Layout(const DrmNv12Layout& layout,
                                 std::string* error_message = nullptr);

/// 校验单对象、线性、连续NV16（YCbCr422SP）布局。
/// 与 NV12 规则的差异：UV 面行数 = 高度（垂直满采样），
/// object_size 需 >= pitch × height × 2。
bool ValidateLinearDrmNv16Layout(const DrmNv12Layout& layout,
                                 std::string* error_message = nullptr);

/// 创建RGA DMA-BUF复制后端；未编译RGA支持时返回可查询但不可用的后端。
std::unique_ptr<IDrmNv12Transfer> CreateDrmNv12Transfer();

}  // namespace drone::video
