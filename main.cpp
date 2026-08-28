#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <sys/types.h>
#include <unistd.h>

#include <spdlog/spdlog.h>

#include "application/drone_application.h"
#include "common/logger.h"
#include "config/config.h"

namespace {

std::atomic<bool> g_stop{false};

void OnSignal(int) {
    g_stop.store(true, std::memory_order_release);
}

std::string ExecDir() {
    char buffer[4096];
    const ssize_t length = ::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (length <= 0) {
        return ".";
    }
    buffer[length] = '\0';
    std::string path(buffer);
    const auto separator = path.find_last_of('/');
    return separator == std::string::npos ? "." : path.substr(0, separator);
}

std::string ParseRequestedConfig(int argc, char** argv) {
    if (argc == 1) {
        return {};
    }
    if (argc == 3 && std::string(argv[1]) == "--config") {
        return argv[2];
    }
    throw std::invalid_argument("用法: drone_control [--config <config.json>]");
}

std::string ResolveLogDirectory(const std::string& configured,
                                const std::string& executable_directory) {
    if (configured.empty() || configured.front() == '/') {
        return configured;
    }
    return executable_directory + '/' + configured;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::string executable_directory = ExecDir();
        const std::string config_path = drone::config::ResolveConfigPath(
            executable_directory, ParseRequestedConfig(argc, argv));
        drone::config::AppConfig config =
            drone::config::LoadAppConfig(config_path, executable_directory);

        const auto log_level = spdlog::level::from_str(config.log.level);
        drone::common::InitializeAsyncLogger(
            ResolveLogDirectory(config.log.directory, executable_directory), log_level);
        SPDLOG_INFO("正式主程序加载配置: path={} log_level={}", config_path,
                    spdlog::level::to_string_view(log_level));

        std::signal(SIGINT, OnSignal);
        std::signal(SIGTERM, OnSignal);

        drone::application::DroneApplication application(std::move(config));
        if (!application.Start()) {
            SPDLOG_ERROR("正式主程序启动失败：没有可运行模块");
            return 2;
        }

        SPDLOG_INFO("正式主程序运行中，按Ctrl+C停止");
        while (!g_stop.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        application.Stop();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "正式主程序启动失败: " << error.what() << std::endl;
        return 1;
    }
}
