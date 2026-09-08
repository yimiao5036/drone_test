/**
 * @file health_manager.cpp
 * @brief HealthManager 真实实现与骨架占位实现
 *
 * HealthManager 只负责数据源登记、新鲜度检查和 HealthStatus 汇总，不负责
 * 硬件读写、链路重连、任务状态转移或 PX4 控制输出。
 */
#include "health/health_manager.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::health {

namespace {

constexpr std::uint32_t kLinkCameraBit = 1U << 0U;
constexpr std::uint32_t kLinkPx4Bit = 1U << 1U;
constexpr std::uint32_t kLinkGroundStationBit = 1U << 2U;
constexpr std::uint32_t kLinkLaserRangeBit = 1U << 3U;
constexpr std::uint32_t kLinkVideoBit = 1U << 4U;

constexpr std::uint32_t kDeviceDecoderBit = 1U << 0U;
constexpr std::uint32_t kDeviceYoloBit = 1U << 1U;
constexpr std::uint32_t kDevicePowerABit = 1U << 2U;
constexpr std::uint32_t kDevicePowerBBit = 1U << 3U;

constexpr std::chrono::milliseconds kMonitorPeriod{100};

struct SourceSpec {
    bool is_device = false;
    std::uint32_t health_bit = 0;
    std::uint32_t error_bit = 0;
};

std::optional<SourceSpec> FindSourceSpec(const std::string& name) {
    if (name == source_names::kCamera) {
        return SourceSpec{false, kLinkCameraBit, error_bits::kCamera};
    }
    if (name == source_names::kPx4) {
        return SourceSpec{false, kLinkPx4Bit, error_bits::kPx4};
    }
    if (name == source_names::kGroundStation) {
        return SourceSpec{false, kLinkGroundStationBit, error_bits::kGroundStation};
    }
    if (name == source_names::kLaserRange) {
        return SourceSpec{false, kLinkLaserRangeBit, error_bits::kLaserRange};
    }
    if (name == source_names::kVideo) {
        return SourceSpec{false, kLinkVideoBit, error_bits::kVideo};
    }
    if (name == source_names::kVideoDecoder) {
        return SourceSpec{true, kDeviceDecoderBit, error_bits::kVideoDecoder};
    }
    if (name == source_names::kYolo) {
        return SourceSpec{true, kDeviceYoloBit, error_bits::kYolo};
    }
    if (name == source_names::kPowerA) {
        return SourceSpec{true, kDevicePowerABit, error_bits::kPower};
    }
    if (name == source_names::kPowerB) {
        return SourceSpec{true, kDevicePowerBBit, error_bits::kPower};
    }
    return std::nullopt;
}

std::uint64_t SteadyNowMs() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

/// 异常日志节流：第 1 次与每满 100 次才打印，避免高频异常刷屏。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

}  // namespace

ProcStatCpuLoadProvider::ProcStatCpuLoadProvider(std::string path)
    : path_(std::move(path)) {
    if (path_.empty()) {
        throw std::invalid_argument("CPU负载采样路径不能为空");
    }
}

CpuLoadSample ProcStatCpuLoadProvider::Sample() {
    std::ifstream input(path_);
    std::string line;
    if (!input.is_open() || !std::getline(input, line)) {
        return {CpuLoadSampleStatus::kError, 0.f};
    }

    std::istringstream parser(line);
    std::string label;
    uint64_t user = 0;
    uint64_t nice = 0;
    uint64_t system = 0;
    uint64_t idle = 0;
    uint64_t iowait = 0;
    uint64_t irq = 0;
    uint64_t softirq = 0;
    uint64_t steal = 0;
    if (!(parser >> label >> user >> nice >> system >> idle) || label != "cpu") {
        return {CpuLoadSampleStatus::kError, 0.f};
    }
    // 旧内核可能缺少尾部字段；缺失项按0处理，前四项必须存在。
    if (!(parser >> iowait)) {
        parser.clear();
    }
    if (!(parser >> irq)) {
        parser.clear();
    }
    if (!(parser >> softirq)) {
        parser.clear();
    }
    if (!(parser >> steal)) {
        parser.clear();
    }

    const uint64_t idle_total = idle + iowait;
    const uint64_t non_idle_total = user + nice + system + irq + softirq + steal;
    const uint64_t total = idle_total + non_idle_total;
    if (!have_previous_) {
        previous_total_ = total;
        previous_idle_ = idle_total;
        have_previous_ = true;
        return {CpuLoadSampleStatus::kWarmup, 0.f};
    }
    if (total < previous_total_ || idle_total < previous_idle_) {
        previous_total_ = total;
        previous_idle_ = idle_total;
        return {CpuLoadSampleStatus::kError, 0.f};
    }

    const uint64_t total_delta = total - previous_total_;
    const uint64_t idle_delta = idle_total - previous_idle_;
    previous_total_ = total;
    previous_idle_ = idle_total;
    if (total_delta == 0 || idle_delta > total_delta) {
        return {CpuLoadSampleStatus::kError, 0.f};
    }

    const double load = 100.0 * static_cast<double>(total_delta - idle_delta) /
                        static_cast<double>(total_delta);
    return {CpuLoadSampleStatus::kValid,
            static_cast<float>(std::clamp(load, 0.0, 100.0))};
}

void ProcStatCpuLoadProvider::Reset() {
    previous_total_ = 0;
    previous_idle_ = 0;
    have_previous_ = false;
}

HealthManager::HealthManager(
    std::unique_ptr<ICpuLoadProvider> cpu_load_provider,
    std::chrono::milliseconds cpu_sample_period,
    std::chrono::milliseconds startup_grace_period)
    : cpu_load_provider_(std::move(cpu_load_provider)),
      cpu_sample_period_(cpu_sample_period),
      startup_grace_period_(startup_grace_period) {
    if (cpu_sample_period_.count() <= 0 || startup_grace_period_.count() <= 0) {
        throw std::invalid_argument("CPU采样周期和健康启动宽限期必须为正数");
    }
    if (!cpu_load_provider_) {
        cpu_load_provider_ = std::make_unique<ProcStatCpuLoadProvider>();
    }
    SPDLOG_INFO("健康管理部件创建: cpu_sample_period_ms={} startup_grace_ms={}",
                cpu_sample_period_.count(), startup_grace_period_.count());
}

HealthManager::~HealthManager() {
    Stop();
    SPDLOG_INFO("健康管理部件销毁");
}

bool HealthManager::Start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
        return true;
    }

    stop_requested_ = false;
    snapshot_requested_ = false;
    last_cpu_sample_ms_ = 0;
    monitor_start_ms_ = SteadyNowMs();
    cpu_load_pct_ = 0.f;
    cpu_load_valid_ = false;
    cpu_load_provider_->Reset();
    running_ = true;
    try {
        monitor_thread_ = std::thread(&HealthManager::MonitorLoop, this);
    } catch (...) {
        running_ = false;
        stop_requested_ = true;
        throw;
    }

    SPDLOG_INFO("健康管理部件启动: source_count={}", sources_.size());
    return true;
}

void HealthManager::Stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ && !monitor_thread_.joinable()) {
            return;
        }
        running_ = false;
        stop_requested_ = true;
    }
    condition_.notify_all();

    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
    SPDLOG_INFO("健康管理部件停止");
}

bool HealthManager::IsRunning() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
}

bool HealthManager::RegisterSource(const std::string& name,
                                   std::uint64_t max_age_ms) {
    const auto source_spec = FindSourceSpec(name);
    std::lock_guard<std::mutex> lock(mutex_);

    if (running_ || name.empty() || max_age_ms == 0 || !source_spec.has_value() ||
        sources_.find(name) != sources_.end()) {
        RecordError("RegisterSource", name);
        return false;
    }

    sources_.emplace(name, SourceState{source_spec->is_device,
                                       source_spec->health_bit,
                                       source_spec->error_bit,
                                       max_age_ms,
                                       0,
                                       false,
                                       false,
                                       false,
                                       0});
    return true;
}

void HealthManager::ReportData(const std::string& name,
                               std::uint64_t receive_time_ms) {
    const std::uint64_t now_ms = SteadyNowMs();
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iterator = sources_.find(name);
    if (iterator == sources_.end() || receive_time_ms == 0 || receive_time_ms > now_ms) {
        RecordError("ReportData", name);
        return;
    }

    SourceState& source = iterator->second;
    const bool was_timed_out = source.timed_out;
    const bool recovered_from_error =
        source.error_active && receive_time_ms > source.last_error_time_ms;
    source.last_receive_time_ms = receive_time_ms;
    source.has_data = true;
    source.timed_out = false;
    if (recovered_from_error) {
        source.error_active = false;
    }
    snapshot_requested_ = true;
    if (was_timed_out) {
        SPDLOG_INFO("健康数据源恢复: {}", name);
    }
    if (recovered_from_error) {
        SPDLOG_INFO("健康数据源活动错误恢复: {}", name);
    }
    condition_.notify_one();
}

void HealthManager::ReportError(const std::string& name,
                                std::uint64_t error_time_ms) {
    const std::uint64_t now_ms = SteadyNowMs();
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iterator = sources_.find(name);
    if (iterator == sources_.end() || error_time_ms == 0 || error_time_ms > now_ms) {
        RecordError("ReportError", name);
        return;
    }

    SourceState& source = iterator->second;
    const bool newly_active = !source.error_active;
    source.error_active = true;
    source.last_error_time_ms = std::max(source.last_error_time_ms, error_time_ms);
    snapshot_requested_ = true;
    if (newly_active) {
        SPDLOG_WARN("健康数据源活动错误: {}", name);
    }
    condition_.notify_one();
}

common::Topic<common::HealthStatus>& HealthManager::Output() {
    return output_;
}

std::uint64_t HealthManager::TimeoutEventCount() const {
    return timeout_event_count_.load(std::memory_order_relaxed);
}

std::uint64_t HealthManager::ErrorCount() const {
    return error_count_.load(std::memory_order_relaxed);
}

void HealthManager::MonitorLoop() {
    PublishSnapshot(SteadyNowMs());

    std::unique_lock<std::mutex> lock(mutex_);
    while (!stop_requested_) {
        condition_.wait_for(lock, kMonitorPeriod,
                            [this] { return stop_requested_ || snapshot_requested_; });
        if (stop_requested_) {
            break;
        }
        snapshot_requested_ = false;
        lock.unlock();
        PublishSnapshot(SteadyNowMs());
        lock.lock();
    }
}

void HealthManager::PublishSnapshot(std::uint64_t now_ms) {
    if (last_cpu_sample_ms_ == 0 ||
        now_ms - last_cpu_sample_ms_ >=
            static_cast<uint64_t>(cpu_sample_period_.count())) {
        const CpuLoadSample sample = cpu_load_provider_->Sample();
        last_cpu_sample_ms_ = now_ms;
        if (sample.status == CpuLoadSampleStatus::kValid) {
            cpu_load_pct_ = std::clamp(sample.load_pct, 0.f, 100.f);
            cpu_load_valid_ = true;
        } else if (sample.status == CpuLoadSampleStatus::kError) {
            cpu_load_valid_ = false;
            RecordError("CpuLoadSample", "/proc/stat");
        }
    }

    common::HealthStatus snapshot;
    std::vector<std::string> timeout_sources;
    std::vector<std::string> recovered_sources;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        bool all_sources_healthy = !sources_.empty();

        for (auto& [name, source] : sources_) {
            const bool fresh = source.has_data && now_ms >= source.last_receive_time_ms &&
                               now_ms - source.last_receive_time_ms <= source.max_age_ms;
            // 启动后给每个数据源一个自身max_age的首包宽限期，避免线程刚启动就误报全源超时。
            const uint64_t first_packet_grace_ms = std::max<uint64_t>(
                source.max_age_ms,
                static_cast<uint64_t>(startup_grace_period_.count()));
            const bool waiting_first_data =
                !source.has_data && now_ms >= monitor_start_ms_ &&
                now_ms - monitor_start_ms_ <= first_packet_grace_ms;
            const bool was_timed_out = source.timed_out;
            source.timed_out = !fresh && !waiting_first_data;

            if (source.timed_out) {
                all_sources_healthy = false;
                if (!was_timed_out) {
                    timeout_event_count_.fetch_add(1, std::memory_order_relaxed);
                    timeout_sources.push_back(name);
                }
            } else if (was_timed_out) {
                recovered_sources.push_back(name);
            }

            if (fresh) {
                if (source.is_device) {
                    snapshot.device_health_bits |= source.health_bit;
                } else {
                    snapshot.link_health_bits |= source.health_bit;
                }
            } else if (source.timed_out && !source.is_device) {
                // 数据源仍在首包宽限期时不置超时位；宽限结束后才进入正式超时。
                snapshot.data_freshness_bits |= source.health_bit;
            }
            if (source.error_active) {
                snapshot.error_bits |= source.error_bit;
            }
        }

        snapshot.header.sequence = ++output_sequence_;
        snapshot.header.receive_time_ms = now_ms;
        snapshot.header.health = sources_.empty() ? 0 : (all_sources_healthy ? 1 : 2);
        snapshot.cpu_load_pct = cpu_load_valid_
                                    ? cpu_load_pct_
                                    : std::numeric_limits<float>::quiet_NaN();
        snapshot.timeout_event_count = static_cast<std::uint32_t>(
            std::min<std::uint64_t>(timeout_event_count_.load(std::memory_order_relaxed),
                                    std::numeric_limits<std::uint32_t>::max()));
    }

    for (const auto& name : timeout_sources) {
        SPDLOG_WARN("健康数据源超时: {}", name);
    }
    for (const auto& name : recovered_sources) {
        SPDLOG_INFO("健康数据源恢复: {}", name);
    }

    // 只有已经订阅的消费者才会接收快照；Topic 本身负责处理队列满策略。
    const auto publish_result =
        output_.Publish(std::make_shared<const common::HealthStatus>(std::move(snapshot)));
    (void)publish_result;
}

void HealthManager::RecordError(const char* operation, const std::string& name) {
    const std::uint64_t count = error_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (ShouldLogThrottled(count)) {
        SPDLOG_WARN("健康管理部件 {} 参数或状态非法: name='{}', 累计错误 {}",
                    operation, name, count);
    }
}

HealthManagerStub::HealthManagerStub() {
    SPDLOG_INFO("健康管理部件骨架创建");
}

HealthManagerStub::~HealthManagerStub() {
    SPDLOG_INFO("健康管理部件骨架销毁");
}

bool HealthManagerStub::Start() {
    running_ = true;
    SPDLOG_INFO("健康管理部件骨架启动");
    return true;
}

void HealthManagerStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("健康管理部件骨架停止");
}

bool HealthManagerStub::IsRunning() const {
    return running_;
}

bool HealthManagerStub::RegisterSource(const std::string& /*name*/,
                                       uint64_t /*max_age_ms*/) {
    ++error_count_;
    if (ShouldLogThrottled(error_count_)) {
        SPDLOG_WARN("健康管理部件 RegisterSource 未实现（骨架占位），累计调用 {}", error_count_);
    }
    return false;
}

void HealthManagerStub::ReportData(const std::string& /*name*/,
                                   uint64_t /*receive_time_ms*/) {
    ++error_count_;
    if (ShouldLogThrottled(error_count_)) {
        SPDLOG_WARN("健康管理部件 ReportData 未实现（骨架占位），累计调用 {}", error_count_);
    }
}

void HealthManagerStub::ReportError(const std::string& /*name*/,
                                    uint64_t /*error_time_ms*/) {
    ++error_count_;
    if (ShouldLogThrottled(error_count_)) {
        SPDLOG_WARN("健康管理部件 ReportError 未实现（骨架占位），累计调用 {}", error_count_);
    }
}

common::Topic<common::HealthStatus>& HealthManagerStub::Output() {
    return output_;
}

uint64_t HealthManagerStub::TimeoutEventCount() const {
    return timeout_event_count_;
}

uint64_t HealthManagerStub::ErrorCount() const {
    return error_count_;
}

}  // namespace drone::health
