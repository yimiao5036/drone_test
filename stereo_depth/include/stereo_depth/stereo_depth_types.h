/**
 * @file stereo_depth_types.h
 * @brief 传统双目测距传感器的公共 POD 类型
 *
 * 属于 stereo_depth 独立子工程，不依赖主项目任何头文件（Topic/VideoFrame/
 * types.h 均不引入），以保持与主项目解耦。主项目侧通过薄适配器把已拆分的
 * 左右目 NV12 帧（Y 平面）与 YOLO 框映射到本文件类型。
 *
 * 坐标系约定：
 * - GrayImage：单目灰度平面（NV12 的 Y 平面即满足：1 字节/像素 + 行 stride）。
 * - QueryRect：查询区域，坐标在【原始左目图像像素系】（与 YOLO 框一致，
 *   未经立体校正），由传感器内部经逆坐标 LUT 映射到 rectified 系再匹配。
 */
#pragma once

#include <cmath>
#include <cstdint>

namespace stereo_depth {

/// 单目灰度平面输入（只读视图，不持有内存）。
///
/// NV12 帧的 Y 平面即 1 字节/像素的连续灰度，行跨距为 stride（>= width）。
struct GrayImage {
    const std::uint8_t* data = nullptr;  ///< 灰度平面首地址（Y 平面）
    std::uint32_t width = 0;             ///< 有效像素宽
    std::uint32_t height = 0;            ///< 有效像素高
    std::uint32_t stride = 0;            ///< 字节行跨距（>= width）
    std::int64_t timestamp_ms = 0;       ///< 帧时间戳（单调毫秒），用于同步/超龄

    /// 视图是否可用（非空且尺寸齐全，stride 不小于 width）。
    [[nodiscard]] bool Valid() const noexcept {
        return data != nullptr && width > 0 && height > 0 && stride >= width;
    }

    /// 取 (x, y) 像素灰度值；越界不做保护（调用方保证在 [0,width)×[0,height) 内）。
    [[nodiscard]] std::uint8_t At(std::uint32_t x, std::uint32_t y) const noexcept {
        return data[static_cast<std::size_t>(y) * stride + x];
    }
};

/// 一对已时间配准的左右目灰度帧。
struct StereoFramePair {
    GrayImage left;   ///< 左目（物理左目，SBS 拆分后已交换到位）
    GrayImage right;  ///< 右目

    /// 左右目尺寸一致且均可用。
    [[nodiscard]] bool Valid() const noexcept {
        return left.Valid() && right.Valid() && left.width == right.width &&
               left.height == right.height;
    }
};

/// 查询区域（原始左目像素系，与 YOLO 框一致）。
struct QueryRect {
    std::int32_t x = 0;        ///< 左上角 x（像素）
    std::int32_t y = 0;        ///< 左上角 y（像素）
    std::int32_t w = 0;        ///< 宽（像素）
    std::int32_t h = 0;        ///< 高（像素）
    std::int32_t track_id = 0; ///< 目标标识，供输出级按目标维护滤波器状态

    /// 区域是否非退化（宽高为正）。
    [[nodiscard]] bool Valid() const noexcept { return w > 0 && h > 0; }
};

/// 距离查询结果。
struct DistanceResult {
    double distance_m = NAN;    ///< 前向距离 Z（米，滤波后）；无效为 NaN
    double disparity_px = NAN;  ///< 左目原始像素系等效视差（含亚像素）；无效为 NaN
    float confidence = 0.0F;    ///< 置信度 [0,1]：有效占比 + 纹理 + LR 一致性合成
    bool valid = false;         ///< 匹配成功且在有效域内
    std::uint32_t valid_pixel_ratio_x100 = 0;  ///< 有效视差像素占比 ×100（调试用）
};

}  // namespace stereo_depth
