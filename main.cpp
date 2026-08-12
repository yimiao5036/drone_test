#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "common/logger.h"
#include "common/topic.h"

namespace {

using nlohmann::json;

/// 从 config/config.json 加载日志配置（目录与等级）。
/// 配置文件缺失或解析失败时返回默认值（logs/ + info），不抛异常，
/// 保证系统在配置缺失时仍可启动并输出告警日志。
void LoadLogConfig(std::string& log_dir, spdlog::level::level_enum& level) {
    constexpr char kConfigPath[] = "config/config.json";

    std::ifstream config_file(kConfigPath);
    if (!config_file.is_open()) {
        std::cerr << "[启动] 未找到配置文件 " << kConfigPath
                  << "，使用默认日志配置 (logs/ + info)" << std::endl;
        return;
    }

    try {
        const json config = json::parse(config_file);
        const json log_config = config.value("log", json::object());
        log_dir = log_config.value("dir", log_dir);
        level = spdlog::level::from_str(
            log_config.value("level", "info"));
    } catch (const std::exception& error) {
        std::cerr << "[启动] 配置文件解析失败: " << error.what()
                  << "，使用默认日志配置 (logs/ + info)" << std::endl;
    }
}

}  // namespace

int main() {
    // 1. 加载 JSON 配置文件，读取日志等级（配置缺失时用默认值兜底）
    std::string log_dir = "logs/";
    auto log_level = spdlog::level::info;
    LoadLogConfig(log_dir, log_level);

    // 2. 初始化全局异步文件日志（生成新日志文件，不覆盖历史）
    drone::common::InitializeAsyncLogger(log_dir, log_level);
    SPDLOG_INFO("已加载日志配置: 目录={} 等级={}", log_dir,
                spdlog::level::to_string_view(log_level));

    // 根入口只负责组合和启动各模块，具体业务逻辑由独立模块实现。
    SPDLOG_INFO("系统启动");
    return 0;
}
