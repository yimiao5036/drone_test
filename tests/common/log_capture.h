#pragma once

// =============================================================================
// log_capture.h —— 测试用日志捕获（spdlog 默认 logger 内存 sink）
//
// 用途：断言组件的日志行为（如“静默期不打印摘要”，故障排查手册 §19）：
// 在启动被测组件前构造、停止组件后析构，用 Lines()/Count() 检查期间输出。
//
// 线程约束：非线程安全——构造/析构（增删 sink、改等级）必须在被测组件
// 线程未运行时进行（spdlog 的 logger->sinks() 不受内部锁保护）。
//
// 只捕获 payload（格式化后的消息体，不含时间/等级/logger 名前缀）；
// 断言请用足够具体的子串，避免误匹配同进程其它日志。
// =============================================================================

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>

namespace drone::test {

/// 内存 sink：逐条保存格式化后的消息体。
class MemoryLogSink final : public spdlog::sinks::base_sink<std::mutex> {
public:
    /// 已捕获行的线程安全快照。
    std::vector<std::string> Lines() const {
        std::lock_guard<std::mutex> lock(lines_mutex_);
        return lines_;
    }

    void Clear() {
        std::lock_guard<std::mutex> lock(lines_mutex_);
        lines_.clear();
    }

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        std::lock_guard<std::mutex> lock(lines_mutex_);
        lines_.emplace_back(msg.payload.data(), msg.payload.size());
    }

    void flush_() override {}

private:
    // 独立锁：base_sink 的 mutex_ 由 spdlog 在 sink_it_ 外层持有
    mutable std::mutex lines_mutex_;
    std::vector<std::string> lines_;
};

/// RAII 日志捕获：向默认 logger 附加内存 sink，析构时移除并恢复日志等级。
class LogCapture {
public:
    explicit LogCapture(spdlog::level::level_enum level = spdlog::level::info)
        : sink_(std::make_shared<MemoryLogSink>()),
          logger_(spdlog::default_logger()),
          previous_level_(logger_->level()) {
        logger_->set_level(level);
        logger_->sinks().push_back(sink_);
    }

    LogCapture(const LogCapture&) = delete;
    LogCapture& operator=(const LogCapture&) = delete;

    ~LogCapture() {
        auto& sinks = logger_->sinks();
        sinks.erase(std::remove(sinks.begin(), sinks.end(), sink_), sinks.end());
        logger_->set_level(previous_level_);
    }

    /// 当前已捕获的全部行。
    std::vector<std::string> Lines() const { return sink_->Lines(); }

    /// 含指定子串的行数。
    int Count(const std::string& needle) const {
        int count = 0;
        for (const std::string& line : Lines()) {
            if (line.find(needle) != std::string::npos) {
                ++count;
            }
        }
        return count;
    }

    /// 丢弃已捕获内容（如跳过启动阶段日志后再开始断言）。
    void Clear() { sink_->Clear(); }

private:
    std::shared_ptr<MemoryLogSink> sink_;
    std::shared_ptr<spdlog::logger> logger_;
    spdlog::level::level_enum previous_level_;
};

}  // namespace drone::test
