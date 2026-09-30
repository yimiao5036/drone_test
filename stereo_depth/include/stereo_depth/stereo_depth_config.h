/**
 * @file stereo_depth_config.h
 * @brief 传统双目测距传感器配置与标定数据结构
 *
 * 配置分两部分：
 * - StereoDepthConfig：可调处理/匹配/查询/滤波参数，从 config/stereo_depth.json
 *   加载（nlohmann），Validate() 对非法值抛 std::invalid_argument。
 * - StereoCalibration：双目内参/畸变/外参，从主项目产出的标定 JSON 加载，
 *   供 StereoRectifier 构建校正与 remap LUT。
 *
 * 加载器只在启动期使用 nlohmann，不进入热路径。
 */
#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace stereo_depth {

/// 3×3 行主序矩阵（相机内参 / 旋转矩阵）。
using Matrix3 = std::array<double, 9>;

/// 单目内参 + 畸变。
struct CameraIntrinsics {
    Matrix3 camera_matrix{};   ///< [fx 0 cx; 0 fy cy; 0 0 1] 行主序
    std::array<double, 5> dist_coeffs{};  ///< [k1 k2 p1 p2 k3]（OpenCV 5 项）
    double rms = 0.0;          ///< 标定重投影 RMS（像素），仅记录用

    [[nodiscard]] double Fx() const noexcept { return camera_matrix[0]; }
    [[nodiscard]] double Fy() const noexcept { return camera_matrix[4]; }
    [[nodiscard]] double Cx() const noexcept { return camera_matrix[2]; }
    [[nodiscard]] double Cy() const noexcept { return camera_matrix[5]; }
};

/// 双目标定数据（对应主项目 camera_calibration_uvc_1280x720.json）。
struct StereoCalibration {
    std::uint32_t image_width = 0;   ///< 标定档位单目宽（如 1280）
    std::uint32_t image_height = 0;  ///< 标定档位单目高（如 720）
    CameraIntrinsics left;           ///< 左目内参/畸变
    CameraIntrinsics right;          ///< 右目内参/畸变
    Matrix3 R{};                     ///< 左→右旋转（3×3 行主序）
    std::array<double, 3> T_m{};     ///< 左→右平移（米）
    double baseline_m = 0.0;         ///< 基线（米），= |T_m|

    /// 标定是否齐全（尺寸、基线为正）。
    [[nodiscard]] bool Valid() const noexcept {
        return image_width > 0 && image_height > 0 && baseline_m > 0.0;
    }
};

/// 传感器可调参数（默认值对应设计文档 §9）。
struct StereoDepthConfig {
    // ---- 分辨率与视差域 ----
    std::uint32_t source_width = 1280;   ///< 输入单目宽（须与标定档位一致）
    std::uint32_t source_height = 720;   ///< 输入单目高
    std::int32_t max_disparity = 96;     ///< 视差搜索上限 D（像素）
    double distance_min_m = 0.8;         ///< 有效域近界（米）
    double distance_max_m = 12.0;        ///< 有效域远界（米）
    double disparity_min_px = 1.5;       ///< 最小可信视差（像素），低于则视为超远/无效

    // ---- ROI 与匹配 ----
    std::int32_t roi_margin_px = 48;     ///< QueryRect 外扩边距（给 SGM 路径聚合上下文）
    std::int32_t census_window = 7;      ///< Census 窗口边长（奇数，5 或 7）
    std::int32_t sgm_paths = 8;          ///< SGM 路径数（4 或 8）
    double penalty_p1 = 8.0;             ///< SGM 连续视差惩罚 P1
    double penalty_p2 = 32.0;            ///< SGM 视差跳变惩罚 P2
    double uniqueness_ratio = 0.9;       ///< WTA 唯一性：best <= ratio*second 才保留
    std::int32_t speckle_max_size = 32;  ///< speckle 去噪最大连通域（像素数），<=0 关闭

    // ---- 查询聚合与亚像素 ----
    bool query_use_median = true;        ///< 区域聚合用中位数（true）/ 均值（false）
    double min_valid_ratio = 0.35;       ///< 有效视差像素占比门限，低于则 valid=false
    std::string subpixel = "linear";     ///< 亚像素估计器："linear"（V型代价模型，默认）/ "parabolic"（裸抛物线）

    // ---- 输出级滤波 ----
    double smooth_alpha = 0.3;           ///< 一阶低通系数 α
    double jump_reset_m = 5.0;           ///< |ΔZ| 超此值直接重置滤波（米）

    // ---- 标定文件 ----
    std::string calibration_path;        ///< 标定 JSON 路径（为空则须由 Init 显式传入）

    /// 校验参数合法性；非法抛 std::invalid_argument。
    void Validate() const;
};

/// 从 JSON 文件加载标定数据。失败抛 std::runtime_error。
StereoCalibration LoadStereoCalibration(const std::string& json_path);

/// 从 JSON 文件加载配置。缺字段用默认值；非法值由 Validate() 拒绝。
/// 失败抛 std::runtime_error，非法抛 std::invalid_argument。
StereoDepthConfig LoadStereoDepthConfig(const std::string& json_path);

}  // namespace stereo_depth
