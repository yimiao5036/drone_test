#pragma once

// =============================================================================
// visual_target_monitor.h —— 单目标视觉稳定性判定（影子）
//
// 对应任务：单目标视觉观测帧级链路。
//
// 职责：
//   消费 YoloDetector 的帧级单目标观测（VisualTargetObservation），按帧节拍维护
//   连续检测/连续丢失计数、平滑后的目标像素中心与置信度，并周期发布
//   VisualTargetStatus（SEARCHING / ACQUIRING / LOCKED / LOST）。
//
// 边界（当前阶段）：
//   - 只做单目标，不做多目标ID关联、IoU匹配或主目标选举；
//   - 不做像素到空间坐标的转换（需要相机标定与双目深度）；
//   - 不做NED滤波，不消费双目、光流或PX4姿态；
//   - 不产生 ControlIntent / Px4Setpoint，纯影子输出；
//   - 不修改 target_tracker/ 独立库。
//
// 数据流：
//   Topic<VisualTargetObservation> ──► VisualTargetMonitor ──► Topic<VisualTargetStatus>
//
// 线程模型：独立消费线程。Stop() 先置停止标志、再 Reset 订阅唤醒等待、最后 join。
// 仅 last_status_ 由互斥锁保护供跨线程读取，其余状态为线程独占。
// =============================================================================

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>

#include "common/topic.h"
#include "common/types.h"

namespace drone::perception {

/// 视觉稳定性判定配置。门限为影子阶段初值，必须由实拍数据复核后再用于门禁。
struct VisualTargetMonitorConfig {
    std::size_t input_queue_capacity = 2;      ///< 观测订阅队列容量（丢最旧）
    std::chrono::milliseconds publish_interval{100};   ///< 状态发布周期
    /// 观测超时：超过该时间未收到任何观测（含无目标观测）判定为数据中断。
    /// 用于区分“本帧无目标”与“YOLO线程已停止或卡死”。
    std::chrono::milliseconds observation_timeout{500};
    int lock_frames = 5;           ///< 连续检测达到该帧数判定为锁定
    int lost_frames = 10;          ///< 连续丢失达到该帧数判定为丢失
    /// 像素中心与置信度的指数滑动平均系数，范围(0,1]；1.0 表示不平滑。
    double smoothing_alpha = 0.3;

    void Validate() const;
};

/// 单目标视觉稳定性判定部件（影子阶段，不驱动控制）。
class VisualTargetMonitor final {
public:
    explicit VisualTargetMonitor(
        VisualTargetMonitorConfig config = VisualTargetMonitorConfig{});
    ~VisualTargetMonitor();

    VisualTargetMonitor(const VisualTargetMonitor&) = delete;
    VisualTargetMonitor& operator=(const VisualTargetMonitor&) = delete;

    /// 启动消费线程。未绑定输入时返回 false，不静默空转。
    bool Start();
    /// 停止并 join；幂等。
    void Stop();
    bool IsRunning() const;

    /// 绑定帧级观测输入主题（IYoloDetector::ObservationOutput()）。须在 Start 前调用。
    void SetInput(common::Topic<common::VisualTargetObservation>& input);

    /// 稳定性状态输出主题。
    common::Topic<common::VisualTargetStatus>& StatusOutput();
    /// 最近一次发布的稳定性状态（线程安全拷贝）。
    common::VisualTargetStatus LastStatus() const;

    /// 累计消费的观测数（等于YOLO成功推理帧数）。
    uint64_t ObservationCount() const;
    /// 累计错误次数（当前为观测队列溢出丢帧等异常）。
    uint64_t ErrorCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace drone::perception
