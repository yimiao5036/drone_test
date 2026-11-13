/**
 * @file stereo_rectifier.h
 * @brief 立体校正：stereoRectify 数学 + 正向 remap LUT + source→rectified 映射
 *
 * 全自研，不依赖 OpenCV。职责：
 * - 由标定内外参（K_l/K_r/D_l/D_r/R/T）计算校正旋转 R1、R2 与共用的新内参
 *   K_new（焦距 f、主点 cx/cy），并给出基线 B（= |T|，校正后平移为 [B,0,0]）。
 * - 构建正向 remap LUT（rectified 像素 → source 采样坐标，含 5 项畸变），
 *   运行时对左右目灰度平面各做一次双线性采样，得到行对齐的 rectified 灰度图。
 * - 提供 source→rectified 的直接映射（畸变逆解 + R1 + K_new），供把 QueryRect
 *   从原始左目像素系映射到 rectified 系（无需全分辨率逆 LUT）。
 * - 深度换算 Z = f·B / disparity。
 *
 * 校正旋转构造（Hartley-Sturm，几何第一性原理）：
 *   e_x = normalize(T)                       —— 新 x 轴沿基线
 *   e_z = normalize([-Tx·Tz, -Ty·Tz, Tx²+Ty²]) —— 与 T 正交且最接近原前向(+Z)
 *   e_y = e_z × e_x                          —— 右手系（图像 y 向下）
 *   R1 = [e_x; e_y; e_z]（行）, R2 = R1·R
 * 正确性由单测「共极行水平 + 深度符号为正」验证。
 *
 * 线程模型：Build() 在 Init 期一次性完成；Rectify 与 SourceToRectified 系列为只读，
 * 可被多线程并发调用（内部不修改状态）。
 */
#pragma once

#include <cstdint>
#include <vector>

#include "stereo_depth/stereo_depth_config.h"
#include "stereo_depth/stereo_depth_types.h"

namespace stereo_depth {

/// 单目 remap LUT：rectified 每个像素对应的 source 采样坐标（浮点）。
struct RemapLut {
    std::uint32_t width = 0;   ///< rectified 宽
    std::uint32_t height = 0;  ///< rectified 高
    std::vector<float> map_x;  ///< size = width*height，source x 坐标
    std::vector<float> map_y;  ///< size = width*height，source y 坐标
};

/// 立体校正器。
class StereoRectifier {
public:
    StereoRectifier() = default;

    /// 由标定构建校正参数与左右目 remap LUT。
    /// @param cal 双目标定数据
    /// @param rect_width  rectified 输出宽（全分辨率时 = source 宽）
    /// @param rect_height rectified 输出高
    void Build(const StereoCalibration& cal, std::uint32_t rect_width,
               std::uint32_t rect_height);

    /// 是否已构建。
    [[nodiscard]] bool Ready() const noexcept { return ready_; }

    /// 把左目 source 灰度平面 remap 为 rectified 灰度（双线性）。
    /// @param src  左目原始 NV12 Y 平面
    /// @param out  输出缓冲，容量须 >= rect_width*rect_height
    void RectifyLeft(const GrayImage& src, std::uint8_t* out) const;
    /// 把右目 source 灰度平面 remap 为 rectified 灰度（双线性）。
    void RectifyRight(const GrayImage& src, std::uint8_t* out) const;

    /// 把原始【左目】像素坐标映射到 rectified 坐标（畸变逆解 + R1 + K_new）。
    /// 用于 QueryRect 角点变换。
    void SourceToRectifiedLeft(double src_x, double src_y, double* rect_u,
                               double* rect_v) const;

    // ---- 参数查询 ----
    [[nodiscard]] double Focal() const noexcept { return focal_; }
    [[nodiscard]] double Baseline() const noexcept { return baseline_; }
    [[nodiscard]] double CxNew() const noexcept { return cx_new_; }
    [[nodiscard]] double CyNew() const noexcept { return cy_new_; }
    [[nodiscard]] std::uint32_t RectWidth() const noexcept { return rect_width_; }
    [[nodiscard]] std::uint32_t RectHeight() const noexcept { return rect_height_; }

    /// 由 rectified 视差（像素）算前向距离 Z（米）：Z = f·B / disp。
    /// disp<=0 返回 NaN。
    [[nodiscard]] double DepthFromDisparity(double disp_px) const;
    /// f·B 常量（米·像素），供误差分析。
    [[nodiscard]] double FocalTimesBaseline() const noexcept {
        return focal_ * baseline_;
    }

private:
    /// 双线性采样 source 到 out[y*width+x]。
    void RemapBilinear(const GrayImage& src, const RemapLut& lut,
                       std::uint8_t* out) const;

    bool ready_ = false;
    std::uint32_t rect_width_ = 0, rect_height_ = 0;
    double focal_ = 0.0, baseline_ = 0.0, cx_new_ = 0.0, cy_new_ = 0.0;
    Matrix3 R1_{};              ///< 左目校正旋转
    CameraIntrinsics cal_left_; ///< 左目内参/畸变（SourceToRectified 用）
    RemapLut left_, right_;
};

}  // namespace stereo_depth
