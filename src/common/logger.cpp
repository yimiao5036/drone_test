#include "common/logger.h"

#include <chrono>
#include <cstddef>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <random>
#include <regex>
#include <sstream>
#include <string>

#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

namespace drone::common {
namespace {

// 日志格式：时间(毫秒) / 级别(带颜色) / 线程 ID / 消息
constexpr char kLogPattern[] = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [t:%t] %v";

// 异步线程池参数：队列容量 8192 条，工作线程 1 个
constexpr std::size_t kAsyncQueueSize = 8192;
constexpr std::size_t kAsyncThreadCount = 1;

// 定期冲刷间隔（秒），避免进程异常退出时丢失过多日志
constexpr std::size_t kFlushIntervalSeconds = 3;

constexpr char kLoggerName[] = "drone_async_logger";

// 系统时间早于 2020-01-01 UTC 视为不可信（RTC 未初始化或时间明显错误），
// 此时日志文件名不写时间，改用随机字符
constexpr std::time_t kTrustedTimeEpoch = 1577836800;

// 随机后缀长度（hex 字符个数）
constexpr std::size_t kRandomSuffixLength = 12;

/// 系统时间可信时返回 "yyyyMMdd_HHmmss"，否则返回空。
std::optional<std::string> FormatNowIfTrusted() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time_now = std::chrono::system_clock::to_time_t(now);
    if (time_now < kTrustedTimeEpoch) {
        return std::nullopt;
    }

    std::tm local_time{};
#if defined(_WIN32)
    localtime_s(&local_time, &time_now);
#else
    localtime_r(&time_now, &local_time);
#endif

    std::ostringstream stream;
    stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return stream.str();
}

/// 生成指定长度的随机 hex 字符（用于系统时间不可信时的文件名后缀）。
std::string GenerateRandomHex(std::size_t length) {
    static constexpr char kHexDigits[] = "0123456789abcdef";
    std::random_device random_device;
    std::uniform_int_distribution<std::size_t> distribution(0, 15);

    std::string result;
    result.reserve(length);
    for (std::size_t i = 0; i < length; ++i) {
        result.push_back(kHexDigits[distribution(random_device)]);
    }
    return result;
}

/// 扫描日志目录，返回本次运行应使用的编号（已有最大编号 + 1，无则从 1 起）。
/// 编号取文件名开头 4 位数字，如 "0001_xxx.log" -> 1。
int FindNextRunNumber(const std::filesystem::path& dir) {
    int max_number = 0;
    std::error_code error;
    if (!std::filesystem::exists(dir, error)) {
        return 1;
    }

    const std::regex number_pattern(R"(^(\d{4})_)");
    for (const auto& entry : std::filesystem::directory_iterator(dir, error)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const std::string file_name = entry.path().filename().string();
        std::smatch match;
        if (std::regex_search(file_name, match, number_pattern)) {
            max_number = std::max(max_number, std::stoi(match[1]));
        }
    }
    return max_number + 1;
}

/// 组装本次运行的日志文件路径：<log_dir>/<编号>_<时间或随机字符>.log
std::filesystem::path BuildLogFilePath(const std::string& log_dir) {
    const std::filesystem::path dir(log_dir);
    std::error_code error;
    std::filesystem::create_directories(dir, error);
    if (error) {
        throw std::runtime_error("创建日志目录失败: " + log_dir + " (" +
                                 error.message() + ")");
    }

    const int run_number = FindNextRunNumber(dir);
    const auto time_text = FormatNowIfTrusted();
    const std::string suffix =
        time_text.value_or(GenerateRandomHex(kRandomSuffixLength));

    std::ostringstream file_name;
    file_name << std::setw(4) << std::setfill('0') << run_number << '_'
              << suffix << ".log";
    return dir / file_name.str();
}

}  // namespace

std::shared_ptr<spdlog::logger> InitializeAsyncLogger(
    const std::string& log_dir) {
    // 幂等：已经初始化过则直接返回既有实例
    if (const auto existing = spdlog::get(kLoggerName)) {
        return existing;
    }

    const auto file_path = BuildLogFilePath(log_dir);

    // 初始化全局异步线程池（有界队列 + 独立工作线程）
    spdlog::init_thread_pool(kAsyncQueueSize, kAsyncThreadCount);

    // 创建异步文件 logger；truncate 为 false，绝不截断已有文件
    auto logger = spdlog::create_async<spdlog::sinks::basic_file_sink_mt>(
        kLoggerName, file_path.string(), false);
    logger->set_pattern(kLogPattern);
    logger->set_level(spdlog::level::debug);  // 级别后续由配置文件控制

    // 设为全局默认 logger，业务代码直接用 SPDLOG_* 宏
    spdlog::set_default_logger(logger);

    // 定期冲刷，防止崩溃时日志丢失过多
    spdlog::flush_every(std::chrono::seconds(kFlushIntervalSeconds));

    SPDLOG_INFO("异步文件日志已初始化: {}", file_path.string());
    return logger;
}

}  // namespace drone::common
