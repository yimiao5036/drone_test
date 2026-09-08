/**
 * @file health_manager.h
 * @brief 健康管理部件接口（IHealthManager）
 *
 * 属于 drone/health 模块。职责：监控各链路与设备的数据新鲜度与健康状态，
 * 汇总发布 HealthStatus，供状态机决策与地面站回传使用。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，健康规则（新鲜度阈值、告警位定义）在实现期接入。
 * - HealthManagerStub 为骨架占位实现：生命周期可运行，业务方法
 *   记录"未实现"节流日志并返回默认值。
 *
 * 数据流：各数据源上报（RegisterSource/ReportData）──► IHealthManager ──► common::Topic<HealthStatus>
 * 可替换边界：健康规则。
 */
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "common/topic.h"
#include "common/types.h"

namespace drone::health {

/// HealthStatus.error_bits 活动错误位定义。
namespace error_bits {
inline constexpr uint32_t kCamera = 1U << 0U;
inline constexpr uint32_t kVideoDecoder = 1U << 1U;
inline constexpr uint32_t kYolo = 1U << 2U;
inline constexpr uint32_t kPx4 = 1U << 3U;
inline constexpr uint32_t kGroundStation = 1U << 4U;
inline constexpr uint32_t kVideo = 1U << 5U;
inline constexpr uint32_t kLaserRange = 1U << 6U;
inline constexpr uint32_t kPower = 1U << 7U;
}  // namespace error_bits

/// HealthManager 使用的标准数据源名称。
/// 名称与 HealthStatus 中的位位置一一对应，调用方不得自行拼写其他别名。
namespace source_names {
inline constexpr char kCamera[] = "camera";
inline constexpr char kPx4[] = "px4";
inline constexpr char kGroundStation[] = "ground_station";
inline constexpr char kLaserRange[] = "laser_range";
inline constexpr char kVideo[] = "video";
inline constexpr char kVideoDecoder[] = "video_decoder";
inline constexpr char kYolo[] = "yolo";
inline constexpr char kPowerA[] = "power_a";
inline constexpr char kPowerB[] = "power_b";
}  // namespace source_names

/// CPU负载采样结果状态。
enum class CpuLoadSampleStatus : uint8_t {
    kValid = 0,
    kWarmup,  ///< 仅取得首个累计值，尚不能计算差分负载。
    kError,
};

struct CpuLoadSample {
    CpuLoadSampleStatus status = CpuLoadSampleStatus::kWarmup;
    float load_pct = 0.f;
};

/// CPU负载采样抽象，便于测试注入和后续替换平台实现。
class ICpuLoadProvider {
public:
    virtual ~ICpuLoadProvider() = default;
    virtual CpuLoadSample Sample() = 0;
    virtual void Reset() = 0;
};

/// Linux /proc/stat CPU总负载采样器；使用相邻两次累计时间差计算百分比。
class ProcStatCpuLoadProvider final : public ICpuLoadProvider {
public:
    explicit ProcStatCpuLoadProvider(std::string path = "/proc/stat");
    CpuLoadSample Sample() override;
    void Reset() override;

private:
    std::string path_;
    uint64_t previous_total_ = 0;
    uint64_t previous_idle_ = 0;
    bool have_previous_ = false;
};

/// 健康管理部件抽象接口。
class IHealthManager {
public:
    virtual ~IHealthManager() = default;

    // ---- 生命周期 ----
    /// 启动健康监控（启动监控周期线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止健康监控；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;

    // ---- 监控 ----
    /// 注册监控数据源。
    /// @param name 数据源名称（与 HealthStatus 位定义对应）。
    /// @param max_age_ms 数据超时阈值；超过即视为数据超时。
    /// @return 是否注册成功（重名或非法参数返回 false）。
    virtual bool RegisterSource(const std::string& name, uint64_t max_age_ms) = 0;

    /// 上报数据源最新数据时间（单调时钟毫秒）。
    /// @param name 已注册的数据源名称。
    /// @param receive_time_ms 最近一次数据到达时间。
    virtual void ReportData(const std::string& name, uint64_t receive_time_ms) = 0;

    /// 上报数据源发生错误的单调时钟时间。错误保持激活，直到收到时间更新的有效数据。
    virtual void ReportError(const std::string& name, uint64_t error_time_ms) = 0;

    // ---- 输出 ----
    /// 健康状态输出主题：common::HealthStatus。
    virtual common::Topic<common::HealthStatus>& Output() = 0;

    // ---- 状态查询 ----
    /// 累计数据超时事件次数。
    virtual uint64_t TimeoutEventCount() const = 0;
    /// 累计错误次数。
    virtual uint64_t ErrorCount() const = 0;
};

/// 真实健康管理实现：按单调时钟检查已注册数据源的新鲜度并发布 HealthStatus。
///
/// 数据源必须在 Start() 前注册；ReportData() 可以在启动前预置首个时间戳，
/// 运行期间由各生产模块在收到有效数据后调用。实现不负责硬件读写、重连或控制决策。
class HealthManager final : public IHealthManager {
public:
    explicit HealthManager(
        std::unique_ptr<ICpuLoadProvider> cpu_load_provider = {},
        std::chrono::milliseconds cpu_sample_period = std::chrono::milliseconds(1000));
    ~HealthManager() override;

    HealthManager(const HealthManager&) = delete;
    HealthManager& operator=(const HealthManager&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    bool RegisterSource(const std::string& name, uint64_t max_age_ms) override;
    void ReportData(const std::string& name, uint64_t receive_time_ms) override;
    void ReportError(const std::string& name, uint64_t error_time_ms) override;

    common::Topic<common::HealthStatus>& Output() override;

    uint64_t TimeoutEventCount() const override;
    uint64_t ErrorCount() const override;

private:
    struct SourceState {
        bool is_device = false;
        uint32_t health_bit = 0;
        uint32_t error_bit = 0;
        uint64_t max_age_ms = 0;
        uint64_t last_receive_time_ms = 0;
        bool has_data = false;
        bool timed_out = false;
        bool error_active = false;
        uint64_t last_error_time_ms = 0;
    };

    void MonitorLoop();
    void PublishSnapshot(uint64_t now_ms);
    void RecordError(const char* operation, const std::string& name);

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::unordered_map<std::string, SourceState> sources_;
    common::Topic<common::HealthStatus> output_;
    std::thread monitor_thread_;
    bool running_ = false;
    bool stop_requested_ = false;
    bool snapshot_requested_ = false;
    uint64_t output_sequence_ = 0;
    std::unique_ptr<ICpuLoadProvider> cpu_load_provider_;
    std::chrono::milliseconds cpu_sample_period_;
    uint64_t last_cpu_sample_ms_ = 0;
    float cpu_load_pct_ = 0.f;
    bool cpu_load_valid_ = false;
    std::atomic<uint64_t> timeout_event_count_{0};
    std::atomic<uint64_t> error_count_{0};
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class HealthManagerStub final : public IHealthManager {
public:
    HealthManagerStub();
    ~HealthManagerStub() override;

    HealthManagerStub(const HealthManagerStub&) = delete;
    HealthManagerStub& operator=(const HealthManagerStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    bool RegisterSource(const std::string& name, uint64_t max_age_ms) override;
    void ReportData(const std::string& name, uint64_t receive_time_ms) override;
    void ReportError(const std::string& name, uint64_t error_time_ms) override;

    common::Topic<common::HealthStatus>& Output() override;

    uint64_t TimeoutEventCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t timeout_event_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::HealthStatus> output_;
};

}  // namespace drone::health
