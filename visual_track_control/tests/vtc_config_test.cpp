// =============================================================================
// vtc_config_test.cpp —— TrackingConfig::LoadFromJson 单元测试
//
// 覆盖（物理追踪思路 §8、需求分析 NFR-007）：
//   真实配置文件加载成功与取值正确（含 A1/A2 修改后的字段结构）、
//   缺字段 / 缺分组 / 非法枚举 / 非法数值 / 文件缺失均抛带字段名的异常。
//
// 依赖编译期宏：
//   VTC_TEST_CONFIG   —— config/track_control.json 绝对路径
//   VTC_TEST_TMP_DIR  —— 测试临时文件输出目录（构建目录）
// =============================================================================

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "visual_track_control/vtc_config.h"

namespace {

using drone::vtc::HeadingMode;
using drone::vtc::NoDistanceAction;
using drone::vtc::TrackingConfig;

/// 与真实配置同构的完整合法 JSON（用于派生各非法变体）
const char* kFullJson = R"({
    "control": {
        "frequency_hz": 20.0,
        "visual_stale_ms": 200,
        "radar_stale_ms": 500,
        "attitude_stale_ms": 300
    },
    "camera": {
        "image_width": 1920,
        "image_height": 1080,
        "fov_h_deg": 60.0,
        "fov_v_deg": 35.0
    },
    "heading": {
        "mode": "rate",
        "gain": 1.0,
        "yaw_rate_limit_dps": 30.0,
        "yaw_slew_limit_dps2": 60.0
    },
    "vertical": {
        "vz_gain_mps_per_deg": 0.1,
        "vz_limit_mps": 2.0
    },
    "distance": {
        "d_exp_m": 10.0,
        "kp": 0.5,
        "ki": 0.05,
        "kd": 0.1,
        "integral_limit": 2.0,
        "derivative_filter_coef": 0.2,
        "approach_velocity_limit_mps": 3.0,
        "retreat_velocity_limit_mps": 1.0,
        "no_distance_action": "hold",
        "no_distance_approach_limit_mps": 0.5
    },
    "accel_limit": {
        "ax_mps2": 2.0,
        "ay_mps2": 2.0,
        "az_mps2": 1.5
    }
})";

/// 在 json 中把首个 from 替换为 to（未找到即测试失败）
void ReplaceOnce(std::string& json, const std::string& from, const std::string& to) {
    const auto pos = json.find(from);
    ASSERT_NE(pos, std::string::npos) << "未找到待替换片段: " << from;
    json.replace(pos, from.size(), to);
}

/// 把 JSON 文本写入临时文件并返回路径（调用方用完后删除）
std::string WriteTempJson(const std::string& content, const char* name) {
    const std::string path = std::string(VTC_TEST_TMP_DIR) + "/" + name;
    std::ofstream ofs(path);
    EXPECT_TRUE(static_cast<bool>(ofs)) << "无法写入临时配置: " << path;
    ofs << content;
    return path;
}

TEST(VtcConfigTest, LoadRealConfigSucceeds) {
    const TrackingConfig cfg = TrackingConfig::LoadFromJson(VTC_TEST_CONFIG);
    EXPECT_DOUBLE_EQ(cfg.control.frequency_hz, 20.0);
    EXPECT_EQ(cfg.control.visual_stale_ms, 200);
    EXPECT_EQ(cfg.camera.image_width, 1920);
    EXPECT_EQ(cfg.camera.image_height, 1080);
    EXPECT_DOUBLE_EQ(cfg.camera.fov_h_deg, 60.0);
    EXPECT_EQ(cfg.heading.mode, HeadingMode::kRate);  // A2：默认速率式
    EXPECT_DOUBLE_EQ(cfg.heading.yaw_rate_limit_dps, 30.0);
    EXPECT_DOUBLE_EQ(cfg.vertical.vz_gain_mps_per_deg, 0.1);
    EXPECT_DOUBLE_EQ(cfg.vertical.vz_limit_mps, 2.0);
    EXPECT_DOUBLE_EQ(cfg.distance.d_exp_m, 10.0);
    EXPECT_EQ(cfg.distance.no_distance_action, NoDistanceAction::kHold);
    EXPECT_DOUBLE_EQ(cfg.accel_limit.ax_mps2, 2.0);
}

TEST(VtcConfigTest, FullJsonRoundTripMatchesExpected) {
    // 内嵌合法 JSON 与真实配置文件行为一致
    const std::string path = WriteTempJson(kFullJson, "vtc_cfg_full.json");
    const TrackingConfig cfg = TrackingConfig::LoadFromJson(path);
    EXPECT_EQ(cfg.heading.mode, HeadingMode::kRate);
    EXPECT_DOUBLE_EQ(cfg.distance.kp, 0.5);
    std::remove(path.c_str());
}

TEST(VtcConfigTest, PositionModeAccepted) {
    std::string json = kFullJson;
    ReplaceOnce(json, "\"mode\": \"rate\"", "\"mode\": \"position\"");
    const std::string path = WriteTempJson(json, "vtc_cfg_position.json");
    const TrackingConfig cfg = TrackingConfig::LoadFromJson(path);
    EXPECT_EQ(cfg.heading.mode, HeadingMode::kPosition);
    std::remove(path.c_str());
}

TEST(VtcConfigTest, MissingFileThrowsWithPath) {
    try {
        TrackingConfig::LoadFromJson("/nonexistent/vtc_no_such_file.json");
        FAIL() << "应当抛出异常";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("vtc_no_such_file.json"),
                  std::string::npos);
    }
}

TEST(VtcConfigTest, MalformedJsonThrows) {
    const std::string path = WriteTempJson("{ this is not json", "vtc_cfg_bad.json");
    EXPECT_THROW(TrackingConfig::LoadFromJson(path), std::runtime_error);
    std::remove(path.c_str());
}

TEST(VtcConfigTest, MissingFieldThrowsWithFieldName) {
    std::string json = kFullJson;
    ReplaceOnce(json, "\"gain\": 1.0,\n", "");
    const std::string path = WriteTempJson(json, "vtc_cfg_missing.json");
    try {
        TrackingConfig::LoadFromJson(path);
        FAIL() << "应当抛出异常";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("heading.gain"), std::string::npos);
    }
    std::remove(path.c_str());
}

TEST(VtcConfigTest, MissingSectionThrows) {
    std::string json = kFullJson;
    ReplaceOnce(json,
                ",\n    \"accel_limit\": {\n"
                "        \"ax_mps2\": 2.0,\n"
                "        \"ay_mps2\": 2.0,\n"
                "        \"az_mps2\": 1.5\n"
                "    }",
                "");
    const std::string path = WriteTempJson(json, "vtc_cfg_nosection.json");
    try {
        TrackingConfig::LoadFromJson(path);
        FAIL() << "应当抛出异常";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("accel_limit"), std::string::npos);
    }
    std::remove(path.c_str());
}

TEST(VtcConfigTest, InvalidHeadingModeThrows) {
    std::string json = kFullJson;
    ReplaceOnce(json, "\"mode\": \"rate\"", "\"mode\": \"magic\"");
    const std::string path = WriteTempJson(json, "vtc_cfg_badmode.json");
    try {
        TrackingConfig::LoadFromJson(path);
        FAIL() << "应当抛出异常";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("heading.mode"), std::string::npos);
    }
    std::remove(path.c_str());
}

TEST(VtcConfigTest, InvalidNoDistanceActionThrows) {
    std::string json = kFullJson;
    ReplaceOnce(json, "\"no_distance_action\": \"hold\"",
                "\"no_distance_action\": \"zoom\"");
    const std::string path = WriteTempJson(json, "vtc_cfg_badaction.json");
    try {
        TrackingConfig::LoadFromJson(path);
        FAIL() << "应当抛出异常";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("no_distance_action"),
                  std::string::npos);
    }
    std::remove(path.c_str());
}

TEST(VtcConfigTest, NegativeFrequencyThrows) {
    std::string json = kFullJson;
    ReplaceOnce(json, "\"frequency_hz\": 20.0", "\"frequency_hz\": -1.0");
    const std::string path = WriteTempJson(json, "vtc_cfg_negfreq.json");
    try {
        TrackingConfig::LoadFromJson(path);
        FAIL() << "应当抛出异常";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("control.frequency_hz"),
                  std::string::npos);
    }
    std::remove(path.c_str());
}

TEST(VtcConfigTest, FovOutOfRangeThrows) {
    std::string json = kFullJson;
    ReplaceOnce(json, "\"fov_h_deg\": 60.0", "\"fov_h_deg\": 200.0");
    const std::string path = WriteTempJson(json, "vtc_cfg_badfov.json");
    try {
        TrackingConfig::LoadFromJson(path);
        FAIL() << "应当抛出异常";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("camera.fov_h_deg"), std::string::npos);
    }
    std::remove(path.c_str());
}

}  // namespace
