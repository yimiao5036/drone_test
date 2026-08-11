/**
 * @file logger.h
 * @brief 全局异步文件日志初始化
 *
 * 属于 drone/common 模块，封装 spdlog 异步日志。
 * 初始化后业务代码直接使用 SPDLOG_DEBUG / SPDLOG_INFO / SPDLOG_WARN /
 * SPDLOG_ERROR 等全局宏即可，无需传递 logger 对象。
 *
 * 日志文件命名规则（每次运行生成新文件，不覆盖历史）：
 *   logs/0001_20260805_103025.log   系统时间可信（yyyyMMdd_HHmmss）
 *   logs/0002_9f3a2c71d4e6.log      系统时间不可信（随机 hex 字符）
 * 其中 0001/0002 为本次运行的日志编号，启动时自动取已有最大编号 + 1。
 */
#ifndef DRONE_COMMON_LOGGER_H_
#define DRONE_COMMON_LOGGER_H_

#include <memory>
#include <string>

#include <spdlog/spdlog.h>

namespace drone::common {

/// 初始化全局异步文件日志。
///
/// 使用 spdlog 异步机制：独立工作线程从有界队列取日志并写入文件，
/// 业务线程只做入队，不阻塞在磁盘 I/O 上，适合高频日志场景。
///
/// @param log_dir 日志输出目录，不存在时自动创建，默认 "logs/"
/// @param level 日志等级（info / debug / warn / error 等），由 main 从
///              JSON 配置文件读取后传入，默认 debug（兜底值）
/// @return 全局异步 logger；重复调用返回同一个实例（幂等）
std::shared_ptr<spdlog::logger> InitializeAsyncLogger(
    const std::string& log_dir = "logs/",
    spdlog::level::level_enum level = spdlog::level::debug);

}  // namespace drone::common

#endif  // DRONE_COMMON_LOGGER_H_
