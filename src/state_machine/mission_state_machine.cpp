/**
 * @file mission_state_machine.cpp
 * @brief MissionStateMachine 安全影子实现（阶段四：位置意图 + 交接判据 + S7 视觉伺服 + 捕网执行器）
 *
 * 默认只接入地面站目标与飞行状态，发布 MissionStatus 供链路验证；
 * 经 SetEstimationInputs 绑定估计/视觉/测距输入后，GPS_APPROACH 状态发布
 * kPositionGuide 控制意图（带 bit31 控制权请求位）；再经
 * SetEstimationDiagnostics 注入估计器诊断后，交接判据（handover 配置节）生效：
 * S5→S6（LOCKED 连续帧/测距有效/视场余量/创新门限）→ S7 验证、S6 超时回 S5、
 * S7 退出迟滞。S6 驻留期续发进入时刻冻结的最后位置目标（防意图超龄 →
 * 控制器刹车 → 再起步）；S7 由内部视觉伺服协作者（复用 TrackingControlLaw）
 * 发布 control_source=1 的速度意图（输出已是局部 NED），短期 LOST 按控制律
 * 判丢语义映射为零速刹车意图。是否真正输出由 FlightController 授权链与
 * enable_control 装配门禁决定。
 * 阶段四 Task 10：注入命令窄接口后，进入 S10（INTERCEPT_EXECUTING）经
 * MAV_CMD_DO_SET_ACTUATOR 发一次捕网触发命令、离开拦截序列发复位行程
 * （actuator 配置节，PWM 微秒 → -1..1 归一化）。
 */
#include "state_machine/mission_state_machine.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

#include "communication/px4_link.h"
#include "control/line_of_sight.h"
#include "control/staleness_checker.h"
#include "control/tracking_control_law.h"
#include "control/tracking_types.h"
#include "perception/target_estimator.h"

namespace drone::state_machine {
namespace {

// 当前实现默认是安全影子状态机：只消费目标/飞行/健康快照并发布 MissionStatus，
// 不生成真实 ControlIntent，也不连接 PX4 控制输出。所有决策运行在独立线程中。
// 绑定估计/视觉/测距输入后，S5 开始发布位置引导意图（EvaluatePositionIntent），
// S6 续发冻结位置目标，S7 由伺服协作者发布速度意图
// （EvaluateVisualServoIntent → BuildVisualIntent）。

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

/// 状态机工作周期。当前影子阶段只做状态/新鲜度判断，100ms足够且不会造成CPU压力。
constexpr auto kDecisionPeriod = 100ms;
/// MissionStatus周期回传间隔。状态切换会立即发布，周期发布用于地面站持续显示。
constexpr uint64_t kStatusPublishIntervalMs = 500;
/// MissionStatus有效期，地面站可据此判断状态回传是否过期。
constexpr uint64_t kStatusValidForMs = 1000;
/// 告警位：尚未收到满足GPS引导要求的PX4导航快照。
constexpr uint32_t kWarningNavigationUnavailable = 1U << 0U;
/// 告警位：已经进入GPS_APPROACH影子状态，但最新目标超过剩余有效期。
constexpr uint32_t kWarningGroundTargetStale = 1U << 1U;
/// 告警位：VISUAL_TRACKING 期间热备份断粮（卡尔曼状态年龄超窗，设计 §2.4）。
/// 只提示"假备份"风险，不触发退出（退出由 S7 四条收口路径决定）。
constexpr uint32_t kWarningHotBackupStarved = 1U << 2U;
/// 告警位：VISUAL_TRACKING 期间本拍未发出视觉伺服意图——伺服尚未启动
/// （S6/S7 过渡期仍在续发冻结位置意图）或控制律降级停发（姿态/视觉超龄，
/// 控制器意图超时门禁接手刹停）。标注"S7 报告状态与实际控制输出背离"，
/// 供地面站观测（65013 全量下发；Web UI 渲染待补。T9 评审裁决 I2，
/// 阶段四 Task 12 落地）。
constexpr uint32_t kWarningServoSuspended = 1U << 3U;
/// 意图重发上限（10Hz）：意图是周期语义，FlightController 50ms 周期消费；
/// 内容无变化时按该间隔重发，内容变化立即发。
constexpr uint64_t kIntentRepublishIntervalMs = 100;
/// 位置引导意图低位原因码：地面站任务下发（docs/数据接口文档.md §3.5 编码 7），
/// 与 bit31 控制权请求位正交叠加。
constexpr uint32_t kReasonGroundStationTask = 7;
/// 视觉伺服意图低位原因码（复用 100 段视觉跟踪语义，docs/数据接口文档.md §3.5
/// 编码 100/102）：100=正常跟踪，102=判丢零速刹车。主链路无预测坐标
/// （is_predicted 恒 false），101（Coast 预测跟踪）在本链路不可达；短时丢失
/// 按控制律判丢语义映射为 102。
constexpr uint32_t kReasonVisualTracking = 100;
constexpr uint32_t kReasonVisualBrake = 102;
/// 局部 NED 坐标系编码（types.h 文件头约定：3=局部NED）。
/// 视觉伺服输出（机体系→NED 旋转在控制律内完成）按该坐标系标注。
constexpr uint8_t kFrameLocalNed = 3;
/// 热备份年龄核对周期（设计 §2.4「S7 期间每秒核对」）。
constexpr uint64_t kHotBackupCheckIntervalMs = 1000;
/// MAV_CMD_DO_SET_ACTUATOR（MAVLink common.xml #187，Task 10 捕网触发）。
/// 参数语义（third_party/c_library_v2 common.xml）：p1..p6 = Actuator Set 1..6
/// 归一化值（-1..1，NaN=忽略不改动）；p7 = 执行器组序号（PX4 仅处理 0）。
/// 状态机不引 MAVLink 头，命令号以常量携带（与 px4_link 的数值约定一致）。
constexpr uint16_t kMavCmdDoSetActuator = 187;
/// 弧度 → 度换算（视线角与视场半角比较统一用度，与配置项单位一致）。
constexpr double kRadToDeg = 57.29577951308232;
/// 交接状态迁移原因（状态转移日志用，中文可读）。
constexpr const char* kReasonEnterHandover =
    "交接进入判据满足（连续锁定/测距有效/视场余量/创新门限）";
constexpr const char* kReasonHandoverVerified =
    "交接验证通过（视觉链持续推进），进入VISUAL_TRACKING";
constexpr const char* kReasonHandoverRevoked =
    "交接验证失败（进入判据失效），撤销回GPS_APPROACH";
constexpr const char* kReasonHandoverTimeout =
    "S6驻留超时（首条视觉指令未生效），回GPS_APPROACH";
constexpr const char* kReasonVisualTrackingExit =
    "视觉连续丢失达退出迟滞门限，回退GPS_APPROACH";
constexpr const char* kReasonVisionStreamLost =
    "视觉状态断流超龄（监视器/链路中断），退出VISUAL_TRACKING回GPS_APPROACH";
constexpr const char* kReasonDistanceLostExit =
    "双目测距失效达距离丢失门限，退出VISUAL_TRACKING回GPS_APPROACH";
constexpr const char* kReasonServoNotStartedTimeout =
    "S7伺服未启动超时（首条视觉伺服意图未生效），回GPS_APPROACH";

/// 交接进入判据输入（阶段四 Task 8；纯数据，供判定函数与 Task 9 复用）。
struct HandoverInputs {
    common::VisualTargetStatus visual;  ///< 最新视觉稳定性状态（state/置信度/中心）
    bool distance_valid = false;        ///< 双目距离有效（StereoTargetDistance.valid）
    bool distance_fresh = false;        ///< 双目距离未超龄（receive_time + 超龄阈值）
    double los_azimuth_deg = 0.0;       ///< 视线方位角（像素中心 + 标定内参）
    double los_elevation_deg = 0.0;     ///< 视线俯仰角（下正）
    double fow_half_azimuth_deg = 0.0;  ///< 视场水平半角（内参推）
    double fow_half_elevation_deg = 0.0;///< 视场垂直半角（内参推）
    double innovation_distance_m =
        std::numeric_limits<double>::quiet_NaN();  ///< 视觉观测与预测水平距离
};

/// 交接进入判据（S5→S6 的必要条件，全部满足才放行；连续帧迟滞由调用方计数）。
/// 纯函数便于单测与 Task 9 复用；不满足即返回 false，不区分原因（原因由调用方
/// 的诊断计数与日志承担）。
bool HandoverEntryConditionsMet(const HandoverInputs& in,
                                const config::HandoverConfig& cfg) {
    // 视觉必须处于稳定锁定（LOCKED），且测距有效且未超龄
    if (in.visual.state != common::VisualTargetState::kLocked) {
        return false;
    }
    if (!in.distance_valid || !in.distance_fresh) {
        return false;
    }
    // 视线角余量：目标视线角距视场边缘至少 fow_margin_deg
    // （内参式视线角以标定主点为基准，视场按主点对称半角建模：
    //  垂直半角取 (H−cy) 侧，对上侧偏保守，宁可晚交接不可交接即丢目标）
    if (std::fabs(in.los_azimuth_deg) + cfg.fow_margin_deg >
        in.fow_half_azimuth_deg) {
        return false;
    }
    if (std::fabs(in.los_elevation_deg) + cfg.fow_margin_deg >
        in.fow_half_elevation_deg) {
        return false;
    }
    // 创新门限：NaN（无有效视觉创新）不得当作"一致"放行
    if (!std::isfinite(in.innovation_distance_m) ||
        in.innovation_distance_m > cfg.innovation_gate_m) {
        return false;
    }
    return true;
}

/// 返回当前进程单调时钟毫秒，用于内部Topic时间戳和新鲜度判断。
uint64_t MonotonicMs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now().time_since_epoch())
            .count());
}

/// 异常/诊断日志节流规则：第1次与每满100次打印一次，避免周期线程刷屏。
bool ShouldLogThrottled(uint64_t count) {
    return count == 1 || count % 100 == 0;
}

/// 浮点内容比较：NaN 与 NaN 视为相等（高度无效时 pos_z_m 恒为 NaN，
/// 不得因此被当成"内容变化"每拍立即重发）。
bool FloatContentEqual(float a, float b) {
    return a == b || (std::isnan(a) && std::isnan(b));
}

/// 任务状态枚举转中文可读名称，供状态转移日志定位。
const char* MissionStateName(common::MissionState state) {
    switch (state) {
        case common::MissionState::kBoot:
            return "BOOT";
        case common::MissionState::kSelfCheck:
            return "SELF_CHECK";
        case common::MissionState::kReady:
            return "READY";
        case common::MissionState::kArming:
            return "ARMING";
        case common::MissionState::kTakeoff:
            return "TAKEOFF";
        case common::MissionState::kGpsApproach:
            return "GPS_APPROACH";
        case common::MissionState::kVisualHandover:
            return "VISUAL_HANDOVER";
        case common::MissionState::kVisualTracking:
            return "VISUAL_TRACKING";
        case common::MissionState::kObstacleHold:
            return "OBSTACLE_HOLD";
        case common::MissionState::kInterceptReady:
            return "INTERCEPT_READY";
        case common::MissionState::kInterceptExecuting:
            return "INTERCEPT_EXECUTING";
        case common::MissionState::kPostIntercept:
            return "POST_INTERCEPT";
        case common::MissionState::kReturnHome:
            return "RETURN_HOME";
        case common::MissionState::kLanding:
            return "LANDING";
        case common::MissionState::kDisarmed:
            return "DISARMED";
        case common::MissionState::kManualOverride:
            return "MANUAL_OVERRIDE";
        case common::MissionState::kFailsafe:
            return "FAILSAFE";
        case common::MissionState::kCount:
            return "COUNT";
    }
    return "UNKNOWN";
}

/// 把任务状态映射为简洁阶段号，便于地面站还没有完整枚举表时显示。
uint8_t TaskPhase(common::MissionState state) {
    return static_cast<uint8_t>(state);
}

/// 控制律降级原因中文名（仅降级停发日志用）。
const char* DegradedReasonName(control::DegradedReason reason) {
    switch (reason) {
        case control::DegradedReason::kNone:
            return "无";
        case control::DegradedReason::kAttitudeStale:
            return "姿态超龄";
        case control::DegradedReason::kVisionStale:
            return "视觉超龄";
        case control::DegradedReason::kNoDistanceExit:
            return "持续无距离退出";
    }
    return "未知";
}

/// 控制意图类型中文名（仅意图切换日志用）。
const char* IntentTypeName(common::ControlIntentType type) {
    switch (type) {
        case common::ControlIntentType::kNone:
            return "无";
        case common::ControlIntentType::kPositionGuide:
            return "位置引导";
        case common::ControlIntentType::kVelocityHeading:
            return "速度航向";
        case common::ControlIntentType::kBrakeHover:
            return "零速刹车";
        case common::ControlIntentType::kTakeoff:
            return "起飞";
        case common::ControlIntentType::kLand:
            return "降落";
        case common::ControlIntentType::kRtl:
            return "返航";
        case common::ControlIntentType::kManualOverride:
            return "人工接管";
    }
    return "未知";
}

}  // namespace

/// MissionStateMachine的实际状态与线程实现，隐藏在PIMPL中保持头文件稳定。
class MissionStateMachine::Impl final {
public:
    explicit Impl(const MissionStateMachineConfig& config)
        : intent_valid_for_ms_(
              static_cast<uint64_t>(config.intent_valid_for.count())),
          handover_(config.handover),
          handover_timeout_ms_(static_cast<uint64_t>(config.handover.timeout.count())),
          visual_stale_ms_(config.visual_stale.count()),
          distance_stale_ms_(config.distance_stale.count()),
          tracking_config_(config.tracking),
          tracking_law_(config.tracking),
          actuator_config_(config.actuator),
          azimuth_converter_(config.tracking.camera.fx_px,
                             config.tracking.camera.cx_px),
          elevation_converter_(config.tracking.camera.fy_px,
                               config.tracking.camera.cy_px) {
        // 视场半角由内参推（主点非画面中心，见 control/tracking_config.h）：
        // 水平取 (W−cx) 侧、垂直取 (H−cy) 侧，构造期固化。
        const double width = static_cast<double>(config.tracking.camera.image_width);
        const double height =
            static_cast<double>(config.tracking.camera.image_height);
        fow_half_azimuth_deg_ =
            std::atan((width - config.tracking.camera.cx_px) /
                      config.tracking.camera.fx_px) *
            kRadToDeg;
        fow_half_elevation_deg_ =
            std::atan((height - config.tracking.camera.cy_px) /
                      config.tracking.camera.fy_px) *
            kRadToDeg;
        SPDLOG_INFO(
            "任务状态机创建: shadow=true control_output=false intent_valid_for_ms={} "
            "handover(lock_frames={} exit_lost_frames={} timeout_ms={} "
            "fow_margin_deg={:.1f} innovation_gate_m={:.1f}) "
            "fov_half(az={:.2f}deg el={:.2f}deg) servo=TrackingControlLaw({:.1f}Hz)",
            intent_valid_for_ms_, handover_.lock_frames, handover_.exit_lost_frames,
            handover_timeout_ms_, handover_.fow_margin_deg,
            handover_.innovation_gate_m, fow_half_azimuth_deg_,
            fow_half_elevation_deg_, tracking_config_.control.frequency_hz);
    }

    ~Impl() {
        Stop();
        SPDLOG_INFO("任务状态机销毁");
    }

    /// 启动状态机工作线程。输入Topic必须已绑定，避免运行期出现空订阅。
    bool Start() {
        if (running_.load(std::memory_order_acquire)) {
            return true;
        }
        if (ground_target_topic_ == nullptr || flight_topic_ == nullptr ||
            health_topic_ == nullptr) {
            RecordError("任务状态机启动失败: 输入Topic未绑定");
            return false;
        }

        // 估计/视觉/测距三个输入是一组：部分绑定视为装配错误，按未绑定处理
        // （保持影子行为），避免 S5 意图只依据部分观测链。
        const bool any_estimation_bound = estimated_topic_ != nullptr ||
                                          visual_status_topic_ != nullptr ||
                                          distance_topic_ != nullptr;
        const bool all_estimation_bound = estimated_topic_ != nullptr &&
                                          visual_status_topic_ != nullptr &&
                                          distance_topic_ != nullptr;
        if (any_estimation_bound && !all_estimation_bound) {
            RecordError("估计/视觉/测距输入必须三者同时绑定；本次按未绑定处理（保持影子）");
        }
        // 交接判据的创新门限需要估计器诊断接口（LastVisualInnovationM）。
        // 未注入时 S5 位置意图照常发布，但交接不放行（保持 GPS_APPROACH）：
        // 无法核验"视觉锁定与卡尔曼航迹是否同一目标"就不允许切进视觉末段。
        if (all_estimation_bound && estimator_diagnostics_ == nullptr) {
            RecordError(
                "未注入估计器诊断接口：交接判据不评估，保持GPS_APPROACH（防错锁伪目标）");
        }

        try {
            ground_target_subscription_ = ground_target_topic_->Subscribe(4);
            flight_subscription_ = flight_topic_->Subscribe(2);
            health_subscription_ = health_topic_->Subscribe(2);
            if (all_estimation_bound) {
                estimated_subscription_ = estimated_topic_->Subscribe(2);
                visual_status_subscription_ = visual_status_topic_->Subscribe(2);
                distance_subscription_ = distance_topic_->Subscribe(2);
                estimation_inputs_active_ = true;
            }
        } catch (const std::exception& error) {
            RecordError(std::string("任务状态机订阅输入失败: ") + error.what());
            return false;
        }

        const uint64_t now_ms = MonotonicMs();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_ = common::MissionState::kBoot;
            state_entered_ms_ = now_ms;
            transition_count_ = 0;
            active_warning_bits_ = 0;
        }
        // 交接层迟滞计数复位（决策线程尚未启动，无并发）；去重记忆一并清空，
        // 重启后同一锁定帧不得沿用上一轮的计数。
        ResetHandoverHysteresis();
        last_counted_locked_frame_.reset();
        // 捕网执行器标志复位：重启后不得沿用上一轮触发记忆（机构实际位置
        // 由 G4 台架/人工确认，见实现文档"排查/修改要点"）。
        net_fired_ = false;
        net_sequence_active_ = false;
        net_reset_pending_ = false;
        PublishStatus(now_ms);

        running_.store(true, std::memory_order_release);
        worker_ = std::thread([this] { WorkerLoop(); });
        SPDLOG_INFO("任务状态机启动: shadow=true");
        return true;
    }

    /// 停止工作线程并释放订阅。幂等，不关闭输出Topic，便于主程序按统一顺序停机。
    void Stop() {
        if (!running_.exchange(false, std::memory_order_acq_rel)) {
            return;
        }
        ground_target_subscription_.Reset();
        flight_subscription_.Reset();
        health_subscription_.Reset();
        estimated_subscription_.Reset();
        visual_status_subscription_.Reset();
        distance_subscription_.Reset();
        if (worker_.joinable()) {
            worker_.join();
        }
        SPDLOG_INFO("任务状态机停止");
    }

    bool IsRunning() const { return running_.load(std::memory_order_acquire); }

    /// 绑定输入Topic。必须在Start前调用；运行中重新绑定会破坏订阅一致性，因此拒绝。
    void SetInputs(common::Topic<common::GroundStationTarget>& ground_target,
                   common::Topic<common::FlightStateSnapshot>& flight,
                   common::Topic<common::HealthStatus>& health) {
        if (running_.load(std::memory_order_acquire)) {
            RecordError("任务状态机运行中不允许重新绑定输入Topic");
            return;
        }
        ground_target_topic_ = &ground_target;
        flight_topic_ = &flight;
        health_topic_ = &health;
    }

    /// 绑定估计/视觉/测距输入（三者一组）。必须在Start前调用；未绑定时
    /// GPS_APPROACH 不出控制意图，保持影子行为。
    void SetEstimationInputs(common::Topic<common::TargetState>& estimated,
                             common::Topic<common::VisualTargetStatus>& visual_status,
                             common::Topic<common::StereoTargetDistance>& distance) {
        if (running_.load(std::memory_order_acquire)) {
            RecordError("任务状态机运行中不允许重新绑定估计输入Topic");
            return;
        }
        estimated_topic_ = &estimated;
        visual_status_topic_ = &visual_status;
        distance_topic_ = &distance;
    }

    /// 注入 PX4 命令窄接口。仅 enable_control=true 装配期调用；当前阶段只持有，
    /// 供后续 ARM/执行机构命令使用，不参与意图发布判定。
    void SetCommandSender(communication::IPx4CommandSender* sender) {
        if (running_.load(std::memory_order_acquire)) {
            RecordError("任务状态机运行中不允许注入 PX4 命令接口");
            return;
        }
        command_sender_ = sender;
        SPDLOG_INFO("任务状态机 PX4 命令接口已注入: sender={}", static_cast<const void*>(sender));
    }

    /// 注入目标估计器诊断接口（交接创新门限的数据来源）。必须在Start前调用；
    /// 未注入时交接判据不评估（保持影子 S5 行为）。
    void SetEstimationDiagnostics(const perception::ITargetEstimator* estimator) {
        if (running_.load(std::memory_order_acquire)) {
            RecordError("任务状态机运行中不允许注入估计器诊断接口");
            return;
        }
        estimator_diagnostics_ = estimator;
        SPDLOG_INFO("任务状态机估计器诊断接口已注入: estimator={}",
                    static_cast<const void*>(estimator));
    }

    common::Topic<common::ControlIntent>& IntentOutput() { return intent_output_; }
    common::Topic<common::MissionStatus>& StatusOutput() { return status_output_; }

    /// 测试入口：强制提交一次状态迁移（与生产路径共用 TransitionTo 统一挂点；
    /// 进入/离开动作由决策线程在 Evaluate 末段评估）。生产装配不得调用。
    void TestForceState(common::MissionState state) {
        TransitionTo(state, "测试入口强制迁移");
    }

    common::MissionState CurrentState() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_;
    }

    uint64_t TransitionCount() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return transition_count_;
    }

    uint64_t ErrorCount() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return error_count_;
    }

private:
    /// 状态机主循环：先进入SELF_CHECK，再按周期抽取最新输入并执行影子状态推进。
    // 工作线程启动后先离开 BOOT 进入 SELF_CHECK；随后每个周期使用最新输入做一次决策。
    void WorkerLoop() {
        TransitionTo(common::MissionState::kSelfCheck, "初始化完成");
        while (running_.load(std::memory_order_acquire)) {
            DrainInputs();
            Evaluate(MonotonicMs());
            std::this_thread::sleep_for(kDecisionPeriod);
        }
    }

    /// 抽干各输入队列，只保留最新快照。状态判断必须基于同一周期的不可变快照。
    // 每个决策周期先抽干输入队列，只保留最新快照，避免旧目标或旧飞行状态积压。
    void DrainInputs() {
        while (auto message = ground_target_subscription_.TryTake()) {
            latest_ground_target_ = **message;
            const uint64_t count = ++target_receive_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_INFO("状态机收到地面站目标: boot={} seq={} target_id={} valid_for_ms={} age_ms={} count={}",
                            latest_ground_target_->ground_station_boot_id,
                            latest_ground_target_->update_seq,
                            latest_ground_target_->target_id,
                            latest_ground_target_->header.valid_for_ms,
                            latest_ground_target_->transport_age_ms, count);
            }
        }
        while (auto message = flight_subscription_.TryTake()) {
            latest_flight_ = **message;
        }
        while (auto message = health_subscription_.TryTake()) {
            latest_health_ = **message;
        }
        while (auto message = estimated_subscription_.TryTake()) {
            latest_estimated_ = **message;
        }
        while (auto message = visual_status_subscription_.TryTake()) {
            latest_visual_status_ = **message;
            UpdateVisualHysteresis(**message);
        }
        while (auto message = distance_subscription_.TryTake()) {
            latest_distance_ = **message;
        }
    }

    /// 根据当前状态、飞行快照和目标新鲜度推进影子状态，并周期发布MissionStatus。
    // 影子决策：SELF_CHECK 等待导航就绪，READY 等待新鲜目标，GPS_APPROACH 只做告警维护。
    // 这里刻意不生成控制意图，真实 ARM/TAKEOFF/导航流程需经过后续安全门禁。
    void Evaluate(uint64_t now_ms) {
        uint32_t warnings = 0;
        if (!HasNavigationForGps()) {
            warnings |= kWarningNavigationUnavailable;
        }

        const bool target_fresh = HasFreshGroundTarget(now_ms);
        const auto state = CurrentState();
        if (state == common::MissionState::kSelfCheck && HasNavigationForGps()) {
            TransitionTo(common::MissionState::kReady, "PX4导航快照满足GPS引导前置条件");
        } else if (state == common::MissionState::kReady && target_fresh &&
                   HasNavigationForGps()) {
            // 安全影子阶段只验证GPS目标进入状态链路；不执行ARMING/TAKEOFF，不输出控制。
            TransitionTo(common::MissionState::kGpsApproach,
                         "收到有效地面站目标，进入GPS_APPROACH影子验证");
        } else if (state == common::MissionState::kGpsApproach && !target_fresh) {
            warnings |= kWarningGroundTargetStale;
            const uint64_t count = ++target_stale_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("GPS_APPROACH影子状态目标过期: count={}", count);
            }
        }

        // 交接评估先于意图发布：S5→S6→S7 迁移当拍即改变位置/视觉意图分支
        // （S5 位置引导 → S6 冻结续发 → S7 视觉伺服，见各函数的当拍收口）。
        EvaluateHandover(now_ms);
        EvaluatePositionIntent(now_ms);
        EvaluateVisualServoIntent(now_ms);
        // 告警位：S7 报告状态与实际控制输出背离标注（T9 评审裁决 I2）——本拍
        // 未发出视觉伺服意图：伺服尚未启动（S6/S7 过渡期仍在续发冻结位置意图）
        // 或控制律降级停发（姿态/视觉超龄 → 控制器意图超时门禁接手刹停）。
        // 状态按当拍（迁移后）判定；离开 S7 即不置位。
        if (CurrentState() == common::MissionState::kVisualTracking &&
            !servo_intent_active_) {
            warnings |= kWarningServoSuspended;
        }
        // 热备份核对（Task 9）：S7 期间每秒核对卡尔曼状态年龄，断粮置告警位。
        warnings |= EvaluateHotBackup(now_ms);
        // 捕网执行器联动（Task 10）：本拍迁移后立即评估（进入 S10 当拍触发、
        // 离开拦截序列当拍复位；入队失败下拍重试，误差 <1 决策周期）。
        EvaluateNetActuator();

        SetWarnings(warnings);
        if (now_ms - last_status_publish_ms_ >= kStatusPublishIntervalMs) {
            PublishStatus(now_ms);
        }
    }

    /// 位置类意图发布总入口（阶段四 Task 7/9）：
    /// - S5（GPS_APPROACH）：估计新鲜 → 发布/重发 kPositionGuide 意图；估计失效
    ///   → 停发，超龄收口由 FlightController 意图超时门禁（intent_timeout）接管；
    /// - S6（VISUAL_HANDOVER）：逐拍续发进入时刻冻结的最后位置目标
    ///   （valid_until 滚动续期，设计 §2.1 冻结 + §2.5 无扰衔接：避免交接期意图
    ///   超龄 → 控制器 COAST 刹车 → 再起步的"交接点先刹停"）；
    /// - S7：首条视觉速度意图发出前仍续发冻结意图（伺服首拍降级时不留意图
    ///   缺口），发出后停发（视觉链接管）。
    void EvaluatePositionIntent(uint64_t now_ms) {
        if (!estimation_inputs_active_) {
            return;
        }
        switch (CurrentState()) {
            case common::MissionState::kGpsApproach:
                EvaluateEstimateGuidedIntent(now_ms);
                return;
            case common::MissionState::kVisualHandover:
                PublishFrozenHandoverIntent(now_ms);
                return;
            case common::MissionState::kVisualTracking:
                if (!s7_servo_started_ && frozen_handover_target_.has_value()) {
                    PublishFrozenHandoverIntent(now_ms);
                    return;
                }
                StopPositionIntent("视觉伺服接续");
                return;
            default:
                StopPositionIntent("离开位置引导段");
                return;
        }
    }

    /// S5 估计引导意图：估计新鲜有效 → 发布/重发（内容变化立即发，内容不变
    /// 按 10Hz 上限续期）；估计失效或超龄 → 停发（不补发安全类意图，
    /// 刹停语义归 FlightController 意图超时门禁）。
    void EvaluateEstimateGuidedIntent(uint64_t now_ms) {
        if (HasFreshEstimate(now_ms)) {
            PublishEstimateGuidedIntent(*latest_estimated_, now_ms);
            return;
        }
        if (position_intent_source_ != PositionIntentSource::kNone) {
            position_intent_source_ = PositionIntentSource::kNone;
            last_intent_target_.reset();
            const uint64_t count = ++intent_stale_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("GPS_APPROACH 位置引导意图停止: 估计无效或超龄 count={}",
                            count);
            }
        }
    }

    /// 冻结最后位置目标（S5→S6 进入时调用，设计 §2.1）：取进入时刻的估计
    /// 目标坐标（此刻的 TargetState 值），交接期不再随估计更新改变。
    void FreezeHandoverTarget() {
        frozen_handover_target_ = latest_estimated_;
        SPDLOG_INFO("交接冻结最后位置目标: pos=({:.2f},{:.2f},{:.2f}) frame_id={}",
                    static_cast<double>(frozen_handover_target_->pos_x_m),
                    static_cast<double>(frozen_handover_target_->pos_y_m),
                    static_cast<double>(frozen_handover_target_->pos_z_m),
                    frozen_handover_target_->header.frame_id);
    }

    /// S6/S7 过渡期冻结位置意图续发：valid_until 逐拍滚动（不做内容节流，
    /// 续期必须发生在每拍），内容固定为进入时刻的冻结目标。
    void PublishFrozenHandoverIntent(uint64_t now_ms) {
        if (!frozen_handover_target_.has_value()) {
            return;  // 防御：未冻结（不应发生，进入 S6 必有估计快照）
        }
        if (position_intent_source_ != PositionIntentSource::kFrozen) {
            position_intent_source_ = PositionIntentSource::kFrozen;
            last_intent_target_.reset();  // 内容节流记忆失效（来源已切换）
            SPDLOG_INFO("交接冻结位置意图续发: 滚动续期直至视觉伺服接续");
        }
        const auto& frozen = *frozen_handover_target_;
        PublishPositionIntent(frozen.pos_x_m, frozen.pos_y_m, frozen.pos_z_m,
                              frozen.header.frame_id, now_ms);
    }

    /// 停止位置类意图发布（离开位置引导段/交接段）；幂等。
    void StopPositionIntent(const char* reason) {
        if (position_intent_source_ == PositionIntentSource::kNone) {
            return;
        }
        position_intent_source_ = PositionIntentSource::kNone;
        last_intent_target_.reset();
        SPDLOG_INFO("位置引导意图停止: {}", reason);
    }

    /// 交接层独立迟滞计数（与 VisualTargetMonitor 内部 lock_frames 判据是两套
    /// 口径，不得混用：监视器判"视觉是否稳定看到"，这里判"交接是否允许切换"）。
    ///
    /// 计数口径（每条视觉状态消息在抽干时计数一次）：
    /// - LOCKED：仅当 frame_sequence 推进（新的锁定帧）才 +1——监视器会周期重发
    ///   同一帧的状态消息，重发不得重复计数（否则"连续帧"被时间稀释）；
    /// - LOST：每条 LOST 状态消息 +1——观测断流（无新帧）时帧序冻结，
    ///   靠状态消息本身表达"仍未见目标"，按帧序去重会让退出永不触发；
    /// - ACQUIRING/SEARCHING：其锁定计数清零、丢失计数清零（既非锁定也非判丢）。
    void UpdateVisualHysteresis(const common::VisualTargetStatus& status) {
        if (status.state == common::VisualTargetState::kLocked) {
            consecutive_lost_frames_ = 0;
            if (!last_counted_locked_frame_.has_value() ||
                *last_counted_locked_frame_ != status.frame_sequence) {
                last_counted_locked_frame_ = status.frame_sequence;
                ++consecutive_lock_frames_;
            }
            return;
        }
        consecutive_lock_frames_ = 0;
        if (status.state == common::VisualTargetState::kLost) {
            ++consecutive_lost_frames_;
        } else {
            consecutive_lost_frames_ = 0;
        }
    }

    /// 复位交接迟滞计数（不清去重记忆：回退后重发的旧锁定帧不得立即重新起算，
    /// 重新进入 S6 必须依赖新的锁定帧）。同时作为**交接段收口**：冻结目标、
    /// 视觉伺服标志、控制律历史与热备份核对状态随段结束清空（S6/S7 回退与
    /// Start 均经此函数；S6→S7 不调用，段内继续）。
    void ResetHandoverHysteresis() {
        consecutive_lock_frames_ = 0;
        consecutive_lost_frames_ = 0;
        s6_lock_baseline_ = 0;
        s6_entered_ms_ = 0;
        s7_entered_ms_ = 0;
        distance_lost_since_ms_.reset();
        if (frozen_handover_target_.has_value()) {
            SPDLOG_INFO("交接段收口: 停止冻结位置续发/视觉伺服意图");
        }
        frozen_handover_target_.reset();
        s7_servo_started_ = false;
        servo_intent_active_ = false;  // 交接段收口：背离告警位随段结束清空
        hot_backup_check_ms_ = 0;
        hot_backup_starved_ = false;
        tracking_law_.Reset();
    }

    /// 交接判据总入口（S5→S6→S7 推进与 S7 退出迟滞）。
    /// 仅在估计/视觉/测距输入已绑定且估计器诊断已注入时工作，否则保持 S5 影子。
    void EvaluateHandover(uint64_t now_ms) {
        if (!estimation_inputs_active_ || estimator_diagnostics_ == nullptr) {
            return;
        }
        switch (CurrentState()) {
            case common::MissionState::kGpsApproach:
                TryEnterVisualHandover(now_ms);
                break;
            case common::MissionState::kVisualHandover:
                EvaluateVisualHandover(now_ms);
                break;
            case common::MissionState::kVisualTracking:
                EvaluateVisualTrackingExit(now_ms);
                break;
            default:
                break;
        }
    }

    /// S5→S6：进入条件全部满足且 LOCKED 连续帧数达 handover.lock_frames。
    /// 进入当拍冻结最后位置目标（设计 §2.1，取进入时刻的 TargetState 值），
    /// 此后 S6 驻留期由 EvaluatePositionIntent 逐拍续发该冻结位置意图
    /// （valid_until 滚动），直到首条视觉速度意图发出（S7，Task 9）——
    /// 避免位置意图在交接期超龄 → 控制器刹车 → 再起步（设计 §2.5）。
    void TryEnterVisualHandover(uint64_t now_ms) {
        if (consecutive_lock_frames_ < handover_.lock_frames) {
            return;
        }
        if (!HandoverEntrySatisfied(now_ms)) {
            return;
        }
        s6_lock_baseline_ = consecutive_lock_frames_;
        s6_entered_ms_ = now_ms;
        consecutive_lost_frames_ = 0;
        FreezeHandoverTarget();
        TransitionTo(common::MissionState::kVisualHandover, kReasonEnterHandover);
    }

    /// S6（VISUAL_HANDOVER）驻留验证：
    /// - 进入条件任一致失效 → 撤销回 S5（不得带着失效条件推进交接）；
    /// - 条件持续满足且再收到一条新的 LOCKED 帧（视觉链在交接期继续推进）
    ///   → S7 VISUAL_TRACKING；
    /// - 驻留超 handover.timeout_ms（首条视觉指令未生效）→ 回 S5 防卡死。
    /// 注：设计 §2.2 的"位置误差/地速/首条视觉指令经限幅生效"三项验证量
    /// 依赖控制侧反馈与尚未冻结的阈值配置，归 Task 9/T12 补齐；本任务用
    /// "条件持续满足 + 视觉链继续推进 + 驻留超时"作为等价的最小验证。
    void EvaluateVisualHandover(uint64_t now_ms) {
        if (!HandoverEntrySatisfied(now_ms)) {
            ResetHandoverHysteresis();
            TransitionTo(common::MissionState::kGpsApproach, kReasonHandoverRevoked);
            return;
        }
        if (consecutive_lock_frames_ > s6_lock_baseline_) {
            consecutive_lost_frames_ = 0;
            s7_entered_ms_ = now_ms;  // S7 伺服未启动超时计时基准（见退出路径④）
            TransitionTo(common::MissionState::kVisualTracking,
                         kReasonHandoverVerified);
            return;
        }
        if (now_ms - s6_entered_ms_ >= handover_timeout_ms_) {
            // 超时回退同样复位计数：重新进入 S6 需要新的连续锁定证据，
            // 不得因陈旧计数在下一拍立即回跳。
            ResetHandoverHysteresis();
            TransitionTo(common::MissionState::kGpsApproach, kReasonHandoverTimeout);
        }
    }

    /// S7 退出（四条独立路径，任一成立即回退 S5）：
    /// ① 视觉状态断流：最近一条 VisualTargetStatus 的接收年龄超过 S6 进入判据
    ///    所用的同一视觉新鲜度阈值（监视器线程死亡/链路中断时无 LOST 消息可计数、
    ///    帧序冻结，按超龄直接收口）；
    /// ② 测距失效连续计时达 handover.distance_lost_ms（valid=false 或超龄；
    ///    恢复有效即清零）——覆盖"测距丢失但视觉仍锁定"的收口；
    /// ③ 视觉 LOST 连续帧数达 handover.exit_lost_frames（进入 lock_frames 与
    ///    退出 exit_lost_frames 门限不同防抖动）；
    /// ④ 伺服未启动超时：S7 驻留超 handover.timeout_ms 仍未见首条有效视觉意图
    ///    （控制律持续降级，如姿态超龄）——冻结位置续发不得无限续期（S6 有
    ///    超时封顶，S7 需等价封顶），收口回 S5 重新走交接判据。
    /// 未达门限保持 S7，短时丢失的图像空间 coast 由 Task 9 的视觉伺服承担
    /// （监视器按 publish_interval≈100ms 发布，健康流不会触及 200ms 超龄阈值）。
    void EvaluateVisualTrackingExit(uint64_t now_ms) {
        if (VisualStatusStale(now_ms)) {
            ResetHandoverHysteresis();
            TransitionTo(common::MissionState::kGpsApproach, kReasonVisionStreamLost);
            return;
        }
        UpdateDistanceLostTimer(now_ms);
        if (distance_lost_since_ms_.has_value() &&
            now_ms - *distance_lost_since_ms_ >=
                static_cast<uint64_t>(handover_.distance_lost_ms.count())) {
            ResetHandoverHysteresis();
            TransitionTo(common::MissionState::kGpsApproach,
                         kReasonDistanceLostExit);
            return;
        }
        if (consecutive_lost_frames_ >= handover_.exit_lost_frames) {
            ResetHandoverHysteresis();
            TransitionTo(common::MissionState::kGpsApproach,
                         kReasonVisualTrackingExit);
            return;
        }
        // 防御：s7_entered_ms_ 由 S6→S7 迁移当拍赋值，正常路径进入 S7 必有值
        //（未赋值时不评估，避免误判 0 为"已驻留超时"当拍即退）。
        if (!s7_servo_started_ && s7_entered_ms_ != 0 &&
            now_ms - s7_entered_ms_ >= handover_timeout_ms_) {
            // 超时回退同样复位计数：重新进入 S6 需要新的连续锁定证据，
            // 不得因陈旧计数在下一拍立即回跳。
            ResetHandoverHysteresis();
            TransitionTo(common::MissionState::kGpsApproach,
                         kReasonServoNotStartedTimeout);
        }
    }

    /// 测距失效计时（S7 专用）：invalid 或超龄即累计起始时刻，恢复有效清零。
    void UpdateDistanceLostTimer(uint64_t now_ms) {
        if (DistanceLost(now_ms)) {
            if (!distance_lost_since_ms_.has_value()) {
                distance_lost_since_ms_ = now_ms;
            }
            return;
        }
        distance_lost_since_ms_.reset();
    }

    /// 测距是否失效：无测距快照、valid=false 或接收超龄（阈值与进入判据同源）。
    bool DistanceLost(uint64_t now_ms) const {
        if (!latest_distance_) {
            return true;
        }
        if (!latest_distance_->valid) {
            return true;
        }
        return !DistanceFresh(now_ms);
    }

    /// 测距接收是否新鲜（存在且未超龄；valid 单独判定，供进入判据两字段使用）。
    bool DistanceFresh(uint64_t now_ms) const {
        if (!latest_distance_) {
            return false;
        }
        return !control::StalenessChecker::IsStale(
            static_cast<int64_t>(latest_distance_->header.receive_time_ms),
            static_cast<int64_t>(now_ms), distance_stale_ms_);
    }

    /// 视觉状态是否断流：无状态消息或接收超龄（阈值与进入判据同源）。
    bool VisualStatusStale(uint64_t now_ms) const {
        if (!latest_visual_status_) {
            return true;
        }
        return control::StalenessChecker::IsStale(
            static_cast<int64_t>(latest_visual_status_->header.receive_time_ms),
            static_cast<int64_t>(now_ms), visual_stale_ms_);
    }

    /// 进入判据整体核验：视觉状态新鲜 + 卡尔曼估计存活 + 四项判据（纯函数）。
    /// 视觉/测距状态消息不携带 valid_for_ms（生产者未设置），新鲜度按
    /// receive_time + 注入的超龄阈值判定；估计新鲜是创新门限的前提
    /// （卡尔曼必须存活，预测才有意义）。
    bool HandoverEntrySatisfied(uint64_t now_ms) const {
        if (VisualStatusStale(now_ms)) {
            return false;
        }
        if (!HasFreshEstimate(now_ms)) {
            return false;
        }
        return HandoverEntryConditionsMet(BuildHandoverInputs(now_ms), handover_);
    }

    /// 组装交接判据输入：视线角由平滑像素中心经标定内参换算（PixelToAngle），
    /// 视场半角为构造期固化的内参推断值，创新值取自估计器诊断接口。
    HandoverInputs BuildHandoverInputs(uint64_t now_ms) const {
        HandoverInputs in;
        in.fow_half_azimuth_deg = fow_half_azimuth_deg_;
        in.fow_half_elevation_deg = fow_half_elevation_deg_;
        if (latest_visual_status_) {
            in.visual = *latest_visual_status_;
            in.los_azimuth_deg =
                azimuth_converter_.Convert(
                    static_cast<double>(in.visual.center_pixel_x)) *
                kRadToDeg;
            in.los_elevation_deg =
                elevation_converter_.Convert(
                    static_cast<double>(in.visual.center_pixel_y)) *
                kRadToDeg;
        }
        if (latest_distance_) {
            in.distance_valid = latest_distance_->valid;
            in.distance_fresh = DistanceFresh(now_ms);
        }
        if (estimator_diagnostics_ != nullptr) {
            in.innovation_distance_m =
                estimator_diagnostics_->LastVisualInnovationM();
        }
        return in;
    }

    /// 估计目标是否新鲜有效（valid 标志 + 剩余有效期，与 TargetEstimator 输出语义一致）。
    bool HasFreshEstimate(uint64_t now_ms) const {
        if (!latest_estimated_) {
            return false;
        }
        const auto& target = *latest_estimated_;
        if (!target.valid) {
            return false;
        }
        if (target.header.receive_time_ms == 0 || target.header.valid_for_ms == 0) {
            return false;
        }
        return now_ms <= target.header.receive_time_ms + target.header.valid_for_ms;
    }

    /// 发布 S5 估计引导位置意图。节流纪律：内容无变化按 10Hz 上限重发
    /// （意图是周期语义，FlightController 50ms 周期消费）；内容变化立即发。
    void PublishEstimateGuidedIntent(const common::TargetState& target,
                                     uint64_t now_ms) {
        const bool content_changed = !last_intent_target_.has_value() ||
            !FloatContentEqual(last_intent_target_->pos_x_m, target.pos_x_m) ||
            !FloatContentEqual(last_intent_target_->pos_y_m, target.pos_y_m) ||
            !FloatContentEqual(last_intent_target_->pos_z_m, target.pos_z_m) ||
            last_intent_target_->valid != target.valid;
        if (position_intent_source_ == PositionIntentSource::kEstimate &&
            !content_changed &&
            now_ms - last_intent_publish_ms_ < kIntentRepublishIntervalMs) {
            return;
        }
        if (position_intent_source_ != PositionIntentSource::kEstimate) {
            SPDLOG_INFO(
                "GPS_APPROACH 位置引导意图启动: target_id={} pos=({:.2f},{:.2f},{:.2f})",
                target.target_id, static_cast<double>(target.pos_x_m),
                static_cast<double>(target.pos_y_m),
                static_cast<double>(target.pos_z_m));
            position_intent_source_ = PositionIntentSource::kEstimate;
        }
        PublishPositionIntent(target.pos_x_m, target.pos_y_m, target.pos_z_m,
                              target.header.frame_id, now_ms);
        last_intent_target_ = target;
    }

    /// 组装并发布位置引导意图（S5 估计引导与 S6 冻结续发共用）。
    /// 高度无效时 pos_z 为 NaN（合法语义）：原样透传，由 FlightController
    /// 有限性门禁拒绝，状态机侧不得改成 0。
    void PublishPositionIntent(float x, float y, float z, uint8_t frame_id,
                               uint64_t now_ms) {
        common::ControlIntent intent;
        intent.header.sequence = ++intent_sequence_;
        intent.header.source_time_ms = now_ms;
        intent.header.receive_time_ms = now_ms;
        intent.header.valid_for_ms = intent_valid_for_ms_;
        intent.header.source_id = 1;
        intent.header.health = 1;
        intent.header.frame_id = frame_id;
        intent.type = common::ControlIntentType::kPositionGuide;
        intent.control_source = 0;  // 位置链
        intent.target_x = x;
        intent.target_y = y;
        intent.target_z = z;
        // bit31 显式请求控制权（FlightController 授权链必要条件）+
        // 低位 7=地面站任务下发。
        intent.reason_code =
            common::kControlIntentReasonRequestControl | kReasonGroundStationTask;
        intent.valid_until_ms = now_ms + intent_valid_for_ms_;
        (void)intent_output_.Publish(
            std::make_shared<const common::ControlIntent>(intent));
        last_intent_publish_ms_ = now_ms;
    }

    // ---- S7 视觉伺服（Task 9：TrackingControlLaw 迁入状态机） ----

    /// S7 视觉伺服意图发布入口。仅在估计输入已绑定且处于 S7 时工作；
    /// 意图逐拍发布（决策周期 10Hz；控制律按实际时钟差推算 dt，配置频率
    /// 仅作首拍兜底）。S7 去留由 EvaluateVisualTrackingExit 四条路径决定，
    /// 本函数不迁移状态。
    void EvaluateVisualServoIntent(uint64_t now_ms) {
        if (!estimation_inputs_active_) {
            return;
        }
        if (CurrentState() != common::MissionState::kVisualTracking) {
            return;  // 离开 S7 的收口在 EvaluateHandover 内完成（含标志复位）
        }
        servo_intent_active_ = false;  // 本拍默认停发（S7 背离告警位按拍评估）
        const auto intent = BuildVisualIntent(now_ms);
        if (!intent.has_value()) {
            // 控制律降级保持（output.valid=false，姿态/视觉超龄）：不发布意图。
            // 安全语义——控制律已明确"禁止正常控制"，补发零速会与其矛盾；
            // 收口交 FlightController 意图超时门禁（intent_timeout）刹停。
            const uint64_t count = ++servo_degraded_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("S7 视觉伺服停发意图: 控制律降级（{}）count={}",
                            DegradedReasonName(last_degraded_reason_), count);
            }
            return;
        }
        if (!s7_servo_started_) {
            s7_servo_started_ = true;
            SPDLOG_INFO(
                "S7 视觉伺服意图启动: type={} vx={:.2f} vy={:.2f} vz={:.2f} "
                "yaw_rate={:.1f}dps",
                IntentTypeName(intent->type),
                static_cast<double>(intent->target_x),
                static_cast<double>(intent->target_y),
                static_cast<double>(intent->target_z),
                static_cast<double>(intent->yaw_rate_dps));
        }
        servo_intent_active_ = true;
        (void)intent_output_.Publish(
            std::make_shared<const common::ControlIntent>(*intent));
    }

    /// 组装控制快照并计算 S7 视觉伺服意图（本任务核心）：
    /// ① 快照：视觉（`LOCKED`→tracked；主链路无预测坐标 → is_predicted 恒
    ///    false）、双目距离（valid 且接收时间有效才透传，超龄判定在控制律内）、
    ///    真实姿态（正式路径不做虚拟姿态兜底——姿态缺失按控制律原语义降级）；
    /// ② TrackingControlLaw::Update（机体系增益/限幅 + 机体系→NED 旋转）；
    /// ③ 输出映射：kVelocityHeading 填 NED 速度与航向通道；kBrakeHover 为零速
    ///    安全意图（判丢刹车，§6.4）；二者均 control_source=1、带 bit31 控制权
    ///    请求位（刹车保持控制权比停发更安全）；降级保持返回 nullopt。
    std::optional<common::ControlIntent> BuildVisualIntent(uint64_t now_ms) {
        control::ControlSnapshot snapshot;
        if (latest_visual_status_.has_value() &&
            latest_visual_status_->header.receive_time_ms > 0) {
            snapshot.track.tracked =
                latest_visual_status_->state == common::VisualTargetState::kLocked;
            snapshot.track.is_predicted = false;  // 主链路无预测坐标
            snapshot.track.center_x =
                static_cast<double>(latest_visual_status_->center_pixel_x);
            snapshot.track.center_y =
                static_cast<double>(latest_visual_status_->center_pixel_y);
            snapshot.track_time_ms = static_cast<int64_t>(
                latest_visual_status_->header.receive_time_ms);
        }
        if (latest_distance_.has_value() && latest_distance_->valid &&
            latest_distance_->header.receive_time_ms > 0) {
            snapshot.target_distance_m =
                static_cast<double>(latest_distance_->forward_distance_m);
            snapshot.distance_valid = true;
            snapshot.distance_time_ms = static_cast<int64_t>(
                latest_distance_->header.receive_time_ms);
        }
        if (latest_flight_.has_value()) {
            snapshot.attitude_present = latest_flight_->attitude_valid &&
                latest_flight_->header.receive_time_ms > 0;
            snapshot.attitude_time_ms = static_cast<int64_t>(
                latest_flight_->header.receive_time_ms);
            snapshot.roll_rad = static_cast<double>(latest_flight_->roll_rad);
            snapshot.pitch_rad = static_cast<double>(latest_flight_->pitch_rad);
            snapshot.yaw_rad = static_cast<double>(latest_flight_->yaw_rad);
        }

        const auto output =
            tracking_law_.Update(snapshot, static_cast<int64_t>(now_ms));
        last_degraded_reason_ = output.degraded_reason;
        if (!output.valid) {
            return std::nullopt;
        }

        common::ControlIntent intent;
        intent.header.sequence = ++intent_sequence_;
        intent.header.source_time_ms = now_ms;
        intent.header.receive_time_ms = now_ms;
        intent.header.valid_for_ms = intent_valid_for_ms_;
        intent.header.source_id = 1;
        intent.header.health = 1;
        // 速度输出已是局部 NED（机体系→NED 旋转在控制律内完成），
        // FlightController 不再做二次旋转。
        intent.header.frame_id = kFrameLocalNed;
        intent.control_source = 1;  // 1=视觉
        intent.valid_until_ms = now_ms + intent_valid_for_ms_;
        switch (output.mode) {
            case control::ControlMode::kVelocityHeading:
                intent.type = common::ControlIntentType::kVelocityHeading;
                intent.target_x = static_cast<float>(output.vx_mps);
                intent.target_y = static_cast<float>(output.vy_mps);
                intent.target_z = static_cast<float>(output.vz_mps);
                intent.yaw_deg = static_cast<float>(output.yaw_deg);
                intent.yaw_rate_dps = static_cast<float>(output.yaw_rate_dps);
                intent.reason_code = common::kControlIntentReasonRequestControl |
                                     kReasonVisualTracking;
                return intent;
            case control::ControlMode::kBrakeHover:
                // 判丢零速刹车（§6.4）：速度分量保持零（ControlIntent 默认值）
                intent.type = common::ControlIntentType::kBrakeHover;
                intent.reason_code = common::kControlIntentReasonRequestControl |
                                     kReasonVisualBrake;
                return intent;
            case control::ControlMode::kDegradedHold:
                break;  // valid=true 时不应出现；防御：不发布
        }
        return std::nullopt;
    }

    /// 热备份核对（设计 §2.4）：S7 期间每秒核对卡尔曼状态年龄
    /// （`TargetState.header.receive_time_ms` 年龄是否超出其自报有效期窗
    /// `valid_for_ms`——生产者声明的"数据允许使用的最长时间"，与 S5 意图门禁
    /// `HasFreshEstimate` 同一口径；无快照、valid=false 亦视为断粮）。
    /// 进入 S7 后立即核对一次，此后每秒一次；断粮 → 节流 WARN + 返回告警位
    /// （不退出 S7：S7 去留由四条收口路径决定，热备份告警只提示"假备份"风险）。
    uint32_t EvaluateHotBackup(uint64_t now_ms) {
        if (CurrentState() != common::MissionState::kVisualTracking) {
            hot_backup_starved_ = false;
            return 0;
        }
        if (hot_backup_check_ms_ != 0 &&
            now_ms - hot_backup_check_ms_ < kHotBackupCheckIntervalMs) {
            return hot_backup_starved_ ? kWarningHotBackupStarved : 0;
        }
        hot_backup_check_ms_ = now_ms;
        hot_backup_starved_ = !HasFreshEstimate(now_ms);
        if (hot_backup_starved_) {
            const uint64_t count = ++hot_backup_starved_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN(
                    "S7 热备份断粮: 卡尔曼状态年龄超窗（估计无效/超龄/未收到），"
                    "不退出 S7 count={}",
                    count);
            }
        }
        return hot_backup_starved_ ? kWarningHotBackupStarved : 0;
    }

    // ---- 捕网执行器（Task 10：MAV_CMD_DO_SET_ACTUATOR 命令窄接口） ----

    /// 捕网执行器联动（决策线程每拍，本函数内状态仅决策线程访问）。语义：
    /// - 进入 S10（INTERCEPT_EXECUTING）：发一次触发命令；net_fired_ 防重复，
    ///   入队失败不置位、下拍重试（PX4 未连接/命令队列满时自动补发）；
    /// - 离开拦截序列（S9/S10 → 回退/返航/POST_INTERCEPT）：发复位行程并清
    ///   触发标志（机构归位到待命行程）；复位入队失败同样下拍重试，且复位
    ///   未入队前不发触发（先复位、后触发的次序不变）。
    /// S9（INTERCEPT_READY）只标记序列开始，不触发（设计：S10 才调用执行机构）。
    /// 未注入命令接口（enable_control=false 装配语义）→ 零调用（物理缺席）；
    /// 触发/复位同为控制输出，仅经 enable_control 门禁注入的窄接口可达。
    /// 注：本函数按"状态驻留"而非"迁移边沿"判定——生产路径同一决策拍内完成
    /// 迁移与评估（Evaluate 末段调用），S10 驻留远长于决策周期，不存在漏触发；
    /// 无生产入口的拦截序列经 TestForceState 驱动时同样满足该时序约束。
    void EvaluateNetActuator() {
        if (command_sender_ == nullptr) {
            return;  // 物理缺席：不产生任何命令
        }
        const auto state = CurrentState();
        const bool in_intercept_sequence =
            state == common::MissionState::kInterceptReady ||
            state == common::MissionState::kInterceptExecuting;
        if (net_sequence_active_ && !in_intercept_sequence) {
            // 离开拦截序列（回退/返航/POST_INTERCEPT）
            net_sequence_active_ = false;
            net_fired_ = false;
            net_reset_pending_ = true;
        }
        if (in_intercept_sequence) {
            net_sequence_active_ = true;
        }
        if (net_reset_pending_) {
            // 复位行程：入队成功才清除，失败下拍重试
            net_reset_pending_ =
                !SendNetActuatorStroke(actuator_config_.net_idle_pwm, "复位行程");
            return;
        }
        if (state == common::MissionState::kInterceptExecuting && !net_fired_) {
            net_fired_ =
                SendNetActuatorStroke(actuator_config_.net_fire_pwm, "触发");
        }
    }

    /// 发送一条捕网行程命令（触发/复位共用）。参数按 MAV_CMD_DO_SET_ACTUATOR
    /// 规范组装：仅本通道对应的参数位填归一化值（PWM 微秒按输出映射区间换算
    /// 为 -1..1），其余参数位 NaN（规范语义"忽略"，不得填 0——0 为中位行程，
    /// 会误动其他执行器组）；p7=执行器组序号固定 0（PX4 仅处理组 0）。
    /// @return 是否成功入队（false=链路未就绪/队列满，由调用方决定重试）。
    bool SendNetActuatorStroke(double pwm_us, const char* action) {
        const int32_t channel = actuator_config_.net_channel;
        const double normalized = actuator_config_.NormalizePwm(pwm_us);
        std::array<float, 7> params;
        params.fill(std::numeric_limits<float>::quiet_NaN());
        params[static_cast<std::size_t>(channel - 1)] =
            static_cast<float>(normalized);
        params[6] = 0.0F;
        const bool queued = command_sender_->SendCommand(
            kMavCmdDoSetActuator, params[0], params[1], params[2], params[3],
            params[4], params[5], params[6]);
        if (queued) {
            SPDLOG_WARN("捕网执行器{}命令已入队: channel={} pwm={:.0f}us norm={:.2f}",
                        action, channel, pwm_us, normalized);
            return true;
        }
        const uint64_t count = ++net_command_fail_count_;
        if (ShouldLogThrottled(count)) {
            SPDLOG_WARN("捕网执行器{}命令入队失败（下拍重试）: channel={} count={}",
                        action, channel, count);
        }
        return false;
    }

    /// GPS位置引导的最低导航前置条件。后续接入控制前还需增加模式/高度/电池等门禁。
    // GPS 引导最低门禁：连接、3D fix、全局位置和 Home 必须同时有效。
    bool HasNavigationForGps() const {
        if (!latest_flight_) {
            return false;
        }
        const auto& flight = *latest_flight_;
        return flight.connected && flight.gps_state_valid && flight.gps_fix &&
               flight.global_position_valid && flight.home_valid;
    }

    /// 判断地面站目标是否仍在飞机端计算出的剩余有效期内。
    // 目标有效期使用 GroundStationLink 校正后的 receive_time + remaining valid_for 判断。
    bool HasFreshGroundTarget(uint64_t now_ms) const {
        if (!latest_ground_target_) {
            return false;
        }
        const auto& target = *latest_ground_target_;
        if (target.header.receive_time_ms == 0 || target.header.valid_for_ms == 0) {
            return false;
        }
        return now_ms <= target.header.receive_time_ms + target.header.valid_for_ms;
    }

    /// 执行状态转移并立即发布状态。所有转移统一经过此函数，便于计数和日志审计。
    void TransitionTo(common::MissionState next_state, const char* reason) {
        const uint64_t now_ms = MonotonicMs();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (state_ == next_state) {
                return;
            }
            SPDLOG_INFO("任务状态转移: {} -> {} reason={}", MissionStateName(state_),
                        MissionStateName(next_state), reason);
            state_ = next_state;
            state_entered_ms_ = now_ms;
            ++transition_count_;
        }
        PublishStatus(now_ms);
    }

    /// 更新当前告警位。状态发布线程读取同一份告警快照。
    void SetWarnings(uint32_t warnings) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        active_warning_bits_ = warnings;
    }

    /// 发布任务状态回传。影子阶段control_source固定为0，表示未取得真实控制权。
    // 发布不可变任务状态快照；影子阶段固定 control_source=0 且禁止拦截授权。
    void PublishStatus(uint64_t now_ms) {
        common::MissionStatus status;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status.header.sequence = ++status_sequence_;
            status.state = state_;
            status.state_entered_ms = state_entered_ms_;
            status.task_phase = TaskPhase(state_);
            status.active_warning_bits = active_warning_bits_;
        }
        status.header.source_time_ms = now_ms;
        status.header.receive_time_ms = now_ms;
        status.header.valid_for_ms = kStatusValidForMs;
        status.header.source_id = 1;
        status.header.health = 1;
        status.control_source = 0;
        status.interception_authorized = false;
        (void)status_output_.Publish(std::make_shared<const common::MissionStatus>(status));
        last_status_publish_ms_ = now_ms;
    }

    /// 记录状态机错误并按节流规则打印，避免启动失败或运行期错误刷屏。
    void RecordError(const std::string& message) {
        uint64_t count = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            count = ++error_count_;
        }
        if (ShouldLogThrottled(count)) {
            SPDLOG_ERROR("{} count={}", message, count);
        }
    }

    common::Topic<common::GroundStationTarget>* ground_target_topic_ = nullptr;
    common::Topic<common::FlightStateSnapshot>* flight_topic_ = nullptr;
    common::Topic<common::HealthStatus>* health_topic_ = nullptr;
    common::Topic<common::TargetState>* estimated_topic_ = nullptr;
    common::Topic<common::VisualTargetStatus>* visual_status_topic_ = nullptr;
    common::Topic<common::StereoTargetDistance>* distance_topic_ = nullptr;
    /// PX4 命令窄接口（仅 enable_control=true 装配期注入）。当前消费点：
    /// 捕网执行器触发/复位（Task 10，EvaluateNetActuator）；未注入时零调用。
    communication::IPx4CommandSender* command_sender_ = nullptr;
    /// 估计器诊断接口（只读；交接创新门限的数据来源，Start 前注入）。
    const perception::ITargetEstimator* estimator_diagnostics_ = nullptr;

    common::Topic<common::GroundStationTarget>::Subscription ground_target_subscription_;
    common::Topic<common::FlightStateSnapshot>::Subscription flight_subscription_;
    common::Topic<common::HealthStatus>::Subscription health_subscription_;
    common::Topic<common::TargetState>::Subscription estimated_subscription_;
    common::Topic<common::VisualTargetStatus>::Subscription visual_status_subscription_;
    common::Topic<common::StereoTargetDistance>::Subscription distance_subscription_;

    std::optional<common::GroundStationTarget> latest_ground_target_;
    std::optional<common::FlightStateSnapshot> latest_flight_;
    std::optional<common::HealthStatus> latest_health_;
    std::optional<common::TargetState> latest_estimated_;
    std::optional<common::VisualTargetStatus> latest_visual_status_;
    std::optional<common::StereoTargetDistance> latest_distance_;

    std::atomic<bool> running_{false};
    std::thread worker_;
    mutable std::mutex state_mutex_;
    common::MissionState state_ = common::MissionState::kBoot;
    uint64_t state_entered_ms_ = 0;
    uint64_t transition_count_ = 0;
    uint64_t error_count_ = 0;
    uint64_t status_sequence_ = 0;
    uint64_t last_status_publish_ms_ = 0;
    uint32_t active_warning_bits_ = 0;
    uint64_t target_receive_count_ = 0;
    uint64_t target_stale_count_ = 0;

    /// 控制意图有效期（构造注入，与 controller.intent_timeout 对齐）。
    const uint64_t intent_valid_for_ms_;
    /// 交接判据配置与阈值（构造注入，全部来自 config.json handover/visual_tracking 节）。
    const config::HandoverConfig handover_;
    const uint64_t handover_timeout_ms_;
    const int64_t visual_stale_ms_;
    const int64_t distance_stale_ms_;
    /// 视觉跟踪控制律配置拷贝与主控律（Task 9；仅决策线程访问，遵循控制律
    /// 单线程约定。控制律内部超龄阈值取 tracking.control.*，生产与状态机侧
    /// visual_stale/distance_stale 同源注入）。
    const control::TrackingConfig tracking_config_;
    control::TrackingControlLaw tracking_law_;
    /// 标定内参换算器与视场半角（构造期固化；热路径零分配）。
    control::PixelToAngle azimuth_converter_;
    control::PixelToAngle elevation_converter_;
    double fow_half_azimuth_deg_ = 0.0;
    double fow_half_elevation_deg_ = 0.0;
    /// 交接层独立迟滞计数（仅决策线程访问；与 VisualTargetMonitor 内部
    /// lock_frames 判据是两套口径，不得混用）。
    uint64_t consecutive_lock_frames_ = 0;
    uint64_t consecutive_lost_frames_ = 0;
    std::optional<uint64_t> last_counted_locked_frame_;  ///< 去重：最近计数的锁定帧序号
    uint64_t s6_lock_baseline_ = 0;  ///< S6 进入时的锁定计数（S6→S7 需再推进）
    uint64_t s6_entered_ms_ = 0;     ///< S6 进入时刻（驻留超时基准）
    uint64_t s7_entered_ms_ = 0;     ///< S7 进入时刻（伺服未启动超时基准）
    /// S7 测距失效计时起始时刻（nullopt=测距未失效；恢复有效即清零）
    std::optional<uint64_t> distance_lost_since_ms_;
    bool estimation_inputs_active_ = false;  ///< 估计/视觉/测距输入组已成功订阅
    /// 位置类意图来源（Task 9 起含 S6 冻结续发与 S7 首拍过渡续发）：
    /// kNone=未发布，kEstimate=S5 估计引导，kFrozen=S6/S7 冻结续发。
    enum class PositionIntentSource { kNone, kEstimate, kFrozen };
    PositionIntentSource position_intent_source_ = PositionIntentSource::kNone;
    /// S5→S6 冻结的最后位置目标（进入时刻的估计快照；交接段结束清空）。
    std::optional<common::TargetState> frozen_handover_target_;
    /// S7 段内是否已发出首条视觉意图（冻结续发的收口条件）。
    bool s7_servo_started_ = false;
    /// S7 本拍是否实际发出视觉伺服意图（背离告警位 kWarningServoSuspended
    /// 的判定依据；EvaluateVisualServoIntent 每拍置位，离开 S7 不再更新）。
    bool servo_intent_active_ = false;
    /// 视觉伺服降级停发计数（节流日志）与最近降级原因。
    uint64_t servo_degraded_count_ = 0;
    control::DegradedReason last_degraded_reason_ = control::DegradedReason::kNone;
    /// 热备份核对（Task 9）：上次核对时刻、断粮状态与断粮计数（节流日志）。
    uint64_t hot_backup_check_ms_ = 0;
    bool hot_backup_starved_ = false;
    uint64_t hot_backup_starved_count_ = 0;
    /// 捕网执行器配置（构造注入，actuator 配置节同源；装机确认前为默认值）。
    const config::ActuatorConfig actuator_config_;
    /// 捕网触发命令已入队（防重复触发；离开拦截序列清除，再进入可重新触发）。
    bool net_fired_ = false;
    /// 处于拦截序列（S9/S10）——离开时的复位行程检测基准（决策线程独占）。
    bool net_sequence_active_ = false;
    /// 复位行程待入队（离开拦截序列置位；入队成功清除，失败下拍重试）。
    bool net_reset_pending_ = false;
    /// 捕网命令入队失败累计计数（节流日志用）。
    uint64_t net_command_fail_count_ = 0;
    uint64_t intent_sequence_ = 0;
    uint64_t last_intent_publish_ms_ = 0;
    uint64_t intent_stale_count_ = 0;
    std::optional<common::TargetState> last_intent_target_;  ///< 上次发布意图对应的目标内容

    common::Topic<common::ControlIntent> intent_output_;
    common::Topic<common::MissionStatus> status_output_;
};

MissionStateMachine::MissionStateMachine(MissionStateMachineConfig config)
    : impl_(std::make_unique<Impl>(config)) {}
MissionStateMachine::~MissionStateMachine() = default;
bool MissionStateMachine::Start() { return impl_->Start(); }
void MissionStateMachine::Stop() { impl_->Stop(); }
bool MissionStateMachine::IsRunning() const { return impl_->IsRunning(); }
void MissionStateMachine::SetInputs(common::Topic<common::GroundStationTarget>& ground_target,
                                    common::Topic<common::FlightStateSnapshot>& flight,
                                    common::Topic<common::HealthStatus>& health) {
    impl_->SetInputs(ground_target, flight, health);
}
void MissionStateMachine::SetEstimationInputs(
    common::Topic<common::TargetState>& estimated,
    common::Topic<common::VisualTargetStatus>& visual_status,
    common::Topic<common::StereoTargetDistance>& distance) {
    impl_->SetEstimationInputs(estimated, visual_status, distance);
}
void MissionStateMachine::SetCommandSender(communication::IPx4CommandSender* sender) {
    impl_->SetCommandSender(sender);
}
void MissionStateMachine::SetEstimationDiagnostics(
    const perception::ITargetEstimator* estimator) {
    impl_->SetEstimationDiagnostics(estimator);
}
common::Topic<common::ControlIntent>& MissionStateMachine::IntentOutput() {
    return impl_->IntentOutput();
}
common::Topic<common::MissionStatus>& MissionStateMachine::StatusOutput() {
    return impl_->StatusOutput();
}
common::MissionState MissionStateMachine::CurrentState() const {
    return impl_->CurrentState();
}
uint64_t MissionStateMachine::TransitionCount() const {
    return impl_->TransitionCount();
}
uint64_t MissionStateMachine::ErrorCount() const { return impl_->ErrorCount(); }
void MissionStateMachine::TestForceState(common::MissionState state) {
    impl_->TestForceState(state);
}

MissionStateMachineStub::MissionStateMachineStub() {
    SPDLOG_INFO("任务状态机部件骨架创建");
}

MissionStateMachineStub::~MissionStateMachineStub() {
    SPDLOG_INFO("任务状态机部件骨架销毁");
}

bool MissionStateMachineStub::Start() {
    running_ = true;
    SPDLOG_INFO("任务状态机部件骨架启动");
    return true;
}

void MissionStateMachineStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("任务状态机部件骨架停止");
}

bool MissionStateMachineStub::IsRunning() const {
    return running_;
}

void MissionStateMachineStub::SetInputs(
    common::Topic<common::GroundStationTarget>& /*ground_target*/,
    common::Topic<common::FlightStateSnapshot>& /*flight*/,
    common::Topic<common::HealthStatus>& /*health*/) {
    // 骨架实现仅保留接口兼容性；正式逻辑见 MissionStateMachine。
}

void MissionStateMachineStub::SetEstimationInputs(
    common::Topic<common::TargetState>& /*estimated*/,
    common::Topic<common::VisualTargetStatus>& /*visual_status*/,
    common::Topic<common::StereoTargetDistance>& /*distance*/) {
    // 骨架实现仅保留接口兼容性；正式逻辑见 MissionStateMachine。
}

void MissionStateMachineStub::SetCommandSender(
    communication::IPx4CommandSender* /*sender*/) {
    // 骨架实现仅保留接口兼容性；正式逻辑见 MissionStateMachine。
}

void MissionStateMachineStub::SetEstimationDiagnostics(
    const perception::ITargetEstimator* /*estimator*/) {
    // 骨架实现仅保留接口兼容性；正式逻辑见 MissionStateMachine。
}

common::Topic<common::ControlIntent>& MissionStateMachineStub::IntentOutput() {
    return intent_output_;
}

common::Topic<common::MissionStatus>& MissionStateMachineStub::StatusOutput() {
    return status_output_;
}

common::MissionState MissionStateMachineStub::CurrentState() const {
    return state_;
}

uint64_t MissionStateMachineStub::TransitionCount() const {
    return transition_count_;
}

uint64_t MissionStateMachineStub::ErrorCount() const {
    return error_count_;
}

}  // namespace drone::state_machine
