/**
 * @file stereo_depth_config.cpp
 * @brief 配置/标定校验与 JSON 加载实现
 */
#include "stereo_depth/stereo_depth_config.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "nlohmann/json.hpp"

namespace stereo_depth {
namespace {

using nlohmann::json;

/// 读取整个文件为 json 对象；失败抛 runtime_error。
json ReadJsonFile(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        throw std::runtime_error("无法打开 JSON 文件: " + path);
    }
    try {
        return json::parse(ifs);
    } catch (const std::exception& e) {
        throw std::runtime_error("JSON 解析失败: " + path + " : " + e.what());
    }
}

/// 解析 3×3 嵌套数组为行主序 Matrix3。
Matrix3 ParseMatrix3(const json& j) {
    Matrix3 m{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            m[static_cast<std::size_t>(r * 3 + c)] = j[r][c].get<double>();
        }
    }
    return m;
}

/// 解析单目内参段。
CameraIntrinsics ParseIntrinsics(const json& j) {
    CameraIntrinsics ci;
    ci.camera_matrix = ParseMatrix3(j.at("camera_matrix"));
    const auto& d = j.at("dist_coeffs");
    for (int i = 0; i < 5; ++i) {
        ci.dist_coeffs[static_cast<std::size_t>(i)] =
            (i < static_cast<int>(d.size())) ? d[i].get<double>() : 0.0;
    }
    if (j.contains("rms")) ci.rms = j.at("rms").get<double>();
    return ci;
}

/// 取字段，缺省返回默认值。
template <typename T>
T GetOr(const json& j, const char* key, T def) {
    return (j.contains(key)) ? j.at(key).get<T>() : def;
}

}  // namespace

void StereoDepthConfig::Validate() const {
    auto require = [](bool ok, const char* msg) {
        if (!ok) throw std::invalid_argument(msg);
    };
    require(source_width > 0 && source_height > 0, "source_width/height 必须为正");
    require(max_disparity > 1, "max_disparity 必须 > 1");
    require(distance_min_m > 0.0, "distance_min_m 必须 > 0");
    require(distance_max_m > distance_min_m, "distance_max_m 必须 > distance_min_m");
    require(disparity_min_px > 0.0, "disparity_min_px 必须 > 0");
    require(roi_margin_px >= 0, "roi_margin_px 必须 >= 0");
    require(census_window == 5 || census_window == 7,
            "census_window 仅支持 5 或 7");
    require(sgm_paths == 4 || sgm_paths == 8, "sgm_paths 仅支持 4 或 8");
    require(penalty_p1 > 0.0 && penalty_p2 > penalty_p1,
            "penalty 需满足 0 < P1 < P2");
    require(uniqueness_ratio > 0.0 && uniqueness_ratio <= 1.0,
            "uniqueness_ratio 必须在 (0,1]");
    require(min_valid_ratio >= 0.0 && min_valid_ratio <= 1.0,
            "min_valid_ratio 必须在 [0,1]");
    require(subpixel == "linear" || subpixel == "parabolic",
            "subpixel 仅支持 linear 或 parabolic");
    require(smooth_alpha > 0.0 && smooth_alpha <= 1.0, "smooth_alpha 必须在 (0,1]");
    require(jump_reset_m > 0.0, "jump_reset_m 必须 > 0");
}

StereoCalibration LoadStereoCalibration(const std::string& json_path) {
    json j = ReadJsonFile(json_path);
    StereoCalibration cal;
    cal.image_width = j.at("image_width").get<std::uint32_t>();
    cal.image_height = j.at("image_height").get<std::uint32_t>();
    cal.left = ParseIntrinsics(j.at("left"));
    cal.right = ParseIntrinsics(j.at("right"));

    const json& s = j.at("stereo");
    cal.R = ParseMatrix3(s.at("R"));
    for (int i = 0; i < 3; ++i) {
        cal.T_m[static_cast<std::size_t>(i)] = s.at("T_m")[i].get<double>();
    }
    cal.baseline_m = GetOr<double>(s, "baseline_m", 0.0);
    if (cal.baseline_m <= 0.0) {
        // 缺 baseline_m 时由 T_m 模长兜底。
        double tx = cal.T_m[0], ty = cal.T_m[1], tz = cal.T_m[2];
        cal.baseline_m = std::sqrt(tx * tx + ty * ty + tz * tz);
    }
    if (!cal.Valid()) {
        throw std::runtime_error("标定数据不完整: " + json_path);
    }
    return cal;
}

StereoDepthConfig LoadStereoDepthConfig(const std::string& json_path) {
    json j = ReadJsonFile(json_path);
    StereoDepthConfig c;  // 默认值即设计文档 §9

    c.source_width = GetOr<std::uint32_t>(j, "source_width", c.source_width);
    c.source_height = GetOr<std::uint32_t>(j, "source_height", c.source_height);
    c.max_disparity = GetOr<std::int32_t>(j, "max_disparity", c.max_disparity);
    c.distance_min_m = GetOr<double>(j, "distance_min_m", c.distance_min_m);
    c.distance_max_m = GetOr<double>(j, "distance_max_m", c.distance_max_m);
    c.disparity_min_px = GetOr<double>(j, "disparity_min_px", c.disparity_min_px);
    c.roi_margin_px = GetOr<std::int32_t>(j, "roi_margin_px", c.roi_margin_px);
    c.census_window = GetOr<std::int32_t>(j, "census_window", c.census_window);
    c.sgm_paths = GetOr<std::int32_t>(j, "sgm_paths", c.sgm_paths);
    c.penalty_p1 = GetOr<double>(j, "penalty_p1", c.penalty_p1);
    c.penalty_p2 = GetOr<double>(j, "penalty_p2", c.penalty_p2);
    c.uniqueness_ratio = GetOr<double>(j, "uniqueness_ratio", c.uniqueness_ratio);
    c.speckle_max_size = GetOr<std::int32_t>(j, "speckle_max_size", c.speckle_max_size);
    c.query_use_median = GetOr<bool>(j, "query_use_median", c.query_use_median);
    c.min_valid_ratio = GetOr<double>(j, "min_valid_ratio", c.min_valid_ratio);
    c.subpixel = GetOr<std::string>(j, "subpixel", c.subpixel);
    c.smooth_alpha = GetOr<double>(j, "smooth_alpha", c.smooth_alpha);
    c.jump_reset_m = GetOr<double>(j, "jump_reset_m", c.jump_reset_m);
    c.calibration_path = GetOr<std::string>(j, "calibration_path", c.calibration_path);

    c.Validate();
    return c;
}

}  // namespace stereo_depth
