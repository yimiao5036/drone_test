// =============================================================================
// vtc_config.cpp —— 配置加载器实现（启动期一次性解析）
//
// 对应规格：docs/物理追踪思路.md §8（阈值集中配置化，NFR-007）。
//
// 实现要点：
//   - 仅在启动期调用一次：缺字段 / 类型错误 / 非法取值抛
//     std::runtime_error，异常消息携带完整字段名（如 "heading.gain"）；
//   - 热路径绝不触碰 JSON：解析结果拷贝进 TrackingConfig 后即释放；
//   - 本文件是全库唯一包含 nlohmann/json 的 TU。
// =============================================================================

#include "visual_track_control/vtc_config.h"

#include <fstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace drone::vtc {
namespace {

using Json = nlohmann::json;

/// 读取并解析 JSON 文件；失败抛异常并携带路径
Json ParseConfigFile(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs) {
        throw std::runtime_error("配置文件打开失败: " + path);
    }
    try {
        return Json::parse(ifs);
    } catch (const std::exception& e) {
        throw std::runtime_error("配置文件解析失败: " + path + " (" + e.what() + ")");
    }
}

/// 取必需的对象分组；缺失或类型错抛异常
const Json& RequireSection(const Json& root, const char* section) {
    const auto it = root.find(section);
    if (it == root.end() || !it->is_object()) {
        throw std::runtime_error(std::string("配置缺失分组或类型错误: ") + section);
    }
    return *it;
}

/// 取必需的数值字段；缺失/类型错抛异常并携带字段名
double RequireNumber(const Json& section, const char* section_name, const char* key) {
    const auto it = section.find(key);
    if (it == section.end() || !it->is_number()) {
        throw std::runtime_error(std::string("配置缺失字段或类型错误: ") +
                                 section_name + "." + key);
    }
    return it->get<double>();
}

/// 取必需的整数字段
std::int64_t RequireInteger(const Json& section, const char* section_name,
                            const char* key) {
    const auto it = section.find(key);
    if (it == section.end() || !it->is_number_integer()) {
        throw std::runtime_error(std::string("配置缺失字段或类型错误(需整数): ") +
                                 section_name + "." + key);
    }
    return it->get<std::int64_t>();
}

/// 取必需的字符串字段
std::string RequireString(const Json& section, const char* section_name,
                          const char* key) {
    const auto it = section.find(key);
    if (it == section.end() || !it->is_string()) {
        throw std::runtime_error(std::string("配置缺失字段或类型错误(需字符串): ") +
                                 section_name + "." + key);
    }
    return it->get<std::string>();
}

/// 校验字段为正数；非法抛异常并携带字段名
double RequirePositive(double value, const char* field_name) {
    if (!(value > 0.0)) {
        throw std::runtime_error(std::string("配置非法(必须为正): ") + field_name);
    }
    return value;
}

/// 校验字段为非负数
double RequireNonNegative(double value, const char* field_name) {
    if (value < 0.0) {
        throw std::runtime_error(std::string("配置非法(不得为负): ") + field_name);
    }
    return value;
}

/// 校验字段在开区间 (lo, hi)
double RequireRange(double value, double lo, double hi, const char* field_name) {
    if (!(value > lo && value < hi)) {
        throw std::runtime_error(std::string("配置非法(超出范围): ") + field_name);
    }
    return value;
}

ControlGroupConfig LoadControl(const Json& root) {
    const Json& s = RequireSection(root, "control");
    ControlGroupConfig c;
    c.frequency_hz = RequirePositive(RequireNumber(s, "control", "frequency_hz"),
                                     "control.frequency_hz");
    c.visual_stale_ms = RequireInteger(s, "control", "visual_stale_ms");
    c.radar_stale_ms = RequireInteger(s, "control", "radar_stale_ms");
    c.attitude_stale_ms = RequireInteger(s, "control", "attitude_stale_ms");
    if (c.visual_stale_ms <= 0 || c.radar_stale_ms <= 0 || c.attitude_stale_ms <= 0) {
        throw std::runtime_error("配置非法: control.*_stale_ms 必须为正");
    }
    return c;
}

CameraGroupConfig LoadCamera(const Json& root) {
    const Json& s = RequireSection(root, "camera");
    CameraGroupConfig c;
    c.image_width = static_cast<int>(
        RequirePositive(RequireInteger(s, "camera", "image_width"), "camera.image_width"));
    c.image_height = static_cast<int>(
        RequirePositive(RequireInteger(s, "camera", "image_height"), "camera.image_height"));
    c.fov_h_deg = RequireRange(RequireNumber(s, "camera", "fov_h_deg"), 0.0, 180.0,
                               "camera.fov_h_deg");
    c.fov_v_deg = RequireRange(RequireNumber(s, "camera", "fov_v_deg"), 0.0, 180.0,
                               "camera.fov_v_deg");
    return c;
}

HeadingGroupConfig LoadHeading(const Json& root) {
    const Json& s = RequireSection(root, "heading");
    HeadingGroupConfig h;
    const std::string mode = RequireString(s, "heading", "mode");
    if (mode == "position") {
        h.mode = HeadingMode::kPosition;
    } else if (mode == "rate") {
        h.mode = HeadingMode::kRate;
    } else {
        throw std::runtime_error("配置非法: heading.mode 仅支持 position / rate");
    }
    h.gain = RequireNonNegative(RequireNumber(s, "heading", "gain"), "heading.gain");
    h.yaw_rate_limit_dps = RequirePositive(
        RequireNumber(s, "heading", "yaw_rate_limit_dps"), "heading.yaw_rate_limit_dps");
    h.yaw_slew_limit_dps2 = RequirePositive(
        RequireNumber(s, "heading", "yaw_slew_limit_dps2"), "heading.yaw_slew_limit_dps2");
    return h;
}

VerticalGroupConfig LoadVertical(const Json& root) {
    const Json& s = RequireSection(root, "vertical");
    VerticalGroupConfig v;
    v.vz_gain_mps_per_deg = RequireNonNegative(
        RequireNumber(s, "vertical", "vz_gain_mps_per_deg"), "vertical.vz_gain_mps_per_deg");
    v.vz_limit_mps = RequirePositive(RequireNumber(s, "vertical", "vz_limit_mps"),
                                     "vertical.vz_limit_mps");
    return v;
}

DistanceGroupConfig LoadDistance(const Json& root) {
    const Json& s = RequireSection(root, "distance");
    DistanceGroupConfig d;
    d.d_exp_m = RequirePositive(RequireNumber(s, "distance", "d_exp_m"), "distance.d_exp_m");
    d.kp = RequireNonNegative(RequireNumber(s, "distance", "kp"), "distance.kp");
    d.ki = RequireNonNegative(RequireNumber(s, "distance", "ki"), "distance.ki");
    d.kd = RequireNonNegative(RequireNumber(s, "distance", "kd"), "distance.kd");
    d.integral_limit = RequireNonNegative(RequireNumber(s, "distance", "integral_limit"),
                                          "distance.integral_limit");
    d.derivative_filter_coef = RequireRange(
        RequireNumber(s, "distance", "derivative_filter_coef"), -0.000001, 1.0,
        "distance.derivative_filter_coef");
    if (d.derivative_filter_coef < 0.0) {
        d.derivative_filter_coef = 0.0;  // 容许 -0.0
    }
    d.approach_velocity_limit_mps = RequirePositive(
        RequireNumber(s, "distance", "approach_velocity_limit_mps"),
        "distance.approach_velocity_limit_mps");
    d.retreat_velocity_limit_mps = RequirePositive(
        RequireNumber(s, "distance", "retreat_velocity_limit_mps"),
        "distance.retreat_velocity_limit_mps");
    const std::string action = RequireString(s, "distance", "no_distance_action");
    if (action == "hold") {
        d.no_distance_action = NoDistanceAction::kHold;
    } else if (action == "slow_approach") {
        d.no_distance_action = NoDistanceAction::kSlowApproach;
    } else if (action == "exit") {
        d.no_distance_action = NoDistanceAction::kExit;
    } else {
        throw std::runtime_error(
            "配置非法: distance.no_distance_action 仅支持 hold / slow_approach / exit");
    }
    d.no_distance_approach_limit_mps = RequireNonNegative(
        RequireNumber(s, "distance", "no_distance_approach_limit_mps"),
        "distance.no_distance_approach_limit_mps");
    return d;
}

AccelLimitConfig LoadAccelLimit(const Json& root) {
    const Json& s = RequireSection(root, "accel_limit");
    AccelLimitConfig a;
    a.ax_mps2 = RequirePositive(RequireNumber(s, "accel_limit", "ax_mps2"),
                                "accel_limit.ax_mps2");
    a.ay_mps2 = RequirePositive(RequireNumber(s, "accel_limit", "ay_mps2"),
                                "accel_limit.ay_mps2");
    a.az_mps2 = RequirePositive(RequireNumber(s, "accel_limit", "az_mps2"),
                                "accel_limit.az_mps2");
    return a;
}

}  // namespace

TrackingConfig TrackingConfig::LoadFromJson(const std::string& path) {
    const Json root = ParseConfigFile(path);
    TrackingConfig cfg;
    cfg.control = LoadControl(root);
    cfg.camera = LoadCamera(root);
    cfg.heading = LoadHeading(root);
    cfg.vertical = LoadVertical(root);
    cfg.distance = LoadDistance(root);
    cfg.accel_limit = LoadAccelLimit(root);
    return cfg;
}

}  // namespace drone::vtc
