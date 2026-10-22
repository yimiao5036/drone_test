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
 * INetActuator 抽象接口触发一次，复位由执行器在触发 ACK 后自行计时完成
 * （离开拦截序列不补发复位）；当前 single_pwm 实现
 * 发送 MAV_CMD_DO_SET_ACTUATOR（PWM 微秒 → -1..1 归一化），后续多发实现
 * 通过 actuator.type 工厂切换，状态机不感知通道/弹仓结构。
 */
#include "state_machine/mission_state_machine.h"

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

#include "communication/ground_station_link.h"
#include "communication/px4_link.h"
#include "control/line_of_sight.h"
#include "control/staleness_checker.h"
#include "control/tracking_control_law.h"
#include "control/tracking_types.h"
#include "perception/geodetic_converter.h"
#include "perception/target_estimator.h"
#include "state_machine/mission_assignment.h"
#include "state_machine/net_actuator.h"

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
/// 告警位：捕网执行器故障（ACK 超时/非 ACCEPTED/PX4 断连/入队超时）。
/// 状态机进入 S16 FAILSAFE：停发一切控制意图、取消待发命令、不重试/不补发
/// 复位，交 PX4/遥控处置。地面站 65013 全量下发，Web UI 渲染待补。
constexpr uint32_t kWarningActuatorFault = 1U << 4U;
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
/// 拦截后返航位置意图低位原因码（8 在 0~7 规划语义之外；S11~S14 V1 口径）。
constexpr uint32_t kReasonReturnHome = 8;
/// MAVLink 命令号/模式编码（状态机不引 MAVLink 头，与 Px4Link/工具同源）。
constexpr uint16_t kMavCmdDoSetMode = 176;          // MAV_CMD_DO_SET_MODE
constexpr uint16_t kMavCmdComponentArmDisarm = 400; // MAV_CMD_COMPONENT_ARM_DISARM
constexpr float kMavModeFlagCustomModeEnabled = 1.0F;
constexpr float kPx4MainModeOffboard = 6.0F;
constexpr float kPx4SubModeOffboard = 0.0F;
constexpr float kPx4MainModeAuto = 4.0F;
constexpr float kPx4SubModeAutoLand = 6.0F;
constexpr uint8_t kPx4FlightMainModeOffboard = 6;
constexpr uint8_t kPx4FlightSubModeOffboard = 0;
constexpr uint8_t kPx4FlightMainModeAuto = 4;
constexpr uint8_t kPx4FlightSubModeAutoLand = 6;
/// 局部 NED 坐标系编码（types.h 文件头约定：3=局部NED）。
/// 视觉伺服输出（机体系→NED 旋转在控制律内完成）按该坐标系标注。
constexpr uint8_t kFrameLocalNed = 3;
/// 热备份年龄核对周期（设计 §2.4「S7 期间每秒核对」）。
constexpr uint64_t kHotBackupCheckIntervalMs = 1000;
/// MAV_CMD_DO_SET_ACTUATOR（MAVLink common.xml #187，Task 10 捕网触发）。
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
          interception_(config.interception),
          return_home_(config.return_home),
          takeoff_(config.takeoff),
          expected_interceptor_type_(config.expected_interceptor_type),
          test_hooks_enabled_(config.test_hooks_enabled),
          azimuth_converter_(config.tracking.camera.fx_px,
                             config.tracking.camera.cx_px),
          elevation_converter_(config.tracking.camera.fy_px,
                               config.tracking.camera.cy_px) {
        // 执行器配置构造期兜底校验：single_pwm 实现最终以
        // `params[net_channel-1]` 索引固定 7 元素数组，程序化构造的越界
        // channel（如 8）即 UB。配置加载期（`LoadAppConfig`）已校验，此处
        // 对程序化构造 fail-fast，先例见 FlightController 构造期 period 防御。
        config.actuator.Validate();
        // 拦截入口与返航配置构造期兜底（程序化构造非法值 fail-fast）。
        config.interception.Validate();
        config.return_home.Validate();
        // 自动起飞时序配置构造期兜底（takeoff 节，2026-10-22）：
        // 高度/爬升率/各超时关系非法时直接抛异常，不允许带病起飞。
        config.takeoff.Validate();
        if (config.interception.enabled && !config.return_home.enabled) {
            throw std::invalid_argument(
                "interception.enabled=true 时 return_home.enabled 必须为 true"
                "（发射后必须有返航收口）");
        }
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
            // 任务分配输入可选：未绑定时不订阅（保持既有影子行为，
            // 不影响任何既有测试/装配）。
            if (assignment_topic_ != nullptr) {
                assignment_subscription_ = assignment_topic_->Subscribe(4);
            }
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
        // 任务故障标志复位：重启后不得沿用上一轮 S16 记忆（执行器本体若已
        // 处于终态故障仍由执行器自身拒绝动作，需人工/维护处置）。
        mission_faulted_ = false;
        // 任务分配状态复位：一次上电一次空中任务语义以 Start 为界，
        // 重启后重新允许分配（分配历史与原点不得沿用上一轮）。
        assignment_tracker_ = MissionAssignmentTracker{};
        latest_assignment_.reset();
        accepted_assignment_.reset();
        assignment_receive_count_ = 0;
        assignment_reject_count_ = 0;
        assignment_duplicate_count_ = 0;
        // 自动起飞时序复位（Task 5）：per-boot 语义，重启后重新允许调度。
        takeoff_stage_ = TakeoffStage::kNone;
        takeoff_stage_ms_ = 0;
        takeoff_last_tick_ms_ = 0;
        takeoff_stable_since_ms_ = 0;
        takeoff_z_cmd_ = 0.f;
        takeoff_disarm_sent_ = false;
        takeoff_ack_baseline_matched_ = 0;
        takeoff_unavailable_latched_ = false;
        // 电池守卫复位（Task 8）：per-boot 语义，中止锁存与防抖计时清空。
        battery_abort_latched_ = false;
        battery_low_since_ms_.reset();
        // 目标等待/丢失看门狗与地面站取消（Task 10）：per-boot 复位。
        mission_abort_latched_ = false;
        target_wait_started_ms_ = 0;
        assigned_target_acquired_ = false;
        target_lost_since_ms_.reset();
        last_cancel_action_ms_.reset();
        // 拦截后返航段状态复位：重启不得沿用上一轮发射/返航记忆。
        interception_fired_ = false;
        return_climb_phase_ = true;
        return_px4_takeover_ = false;
        return_timeout_warned_ = false;
        return_home_wait_logged_ = false;
        arrival_stable_since_ms_ = 0;
        landing_command_sent_ = false;
        landing_command_ms_ = 0;
        landing_takeover_ = false;
        landing_command_unavailable_logged_ = false;
        landing_command_fail_count_ = 0;
        disarm_command_sent_ = false;
        disarm_command_ms_ = 0;
        disarm_command_fail_count_ = 0;
        mission_end_logged_ = false;
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
        assignment_subscription_.Reset();
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
        net_actuator_.reset();
        if (sender != nullptr) {
            net_actuator_ = CreateNetActuator(actuator_config_, sender);
        }
        SPDLOG_INFO("任务状态机 PX4 命令接口已注入: sender={} net_type={}",
                    static_cast<const void*>(sender),
                    net_actuator_ ? net_actuator_->Type() : "disabled");
    }

    /// 绑定任务分配输入 Topic（65014）。必须在 Start 前调用；未绑定时
    /// 不订阅、不处理分配，既有影子行为保持不变。
    void SetAssignmentInput(common::Topic<common::MissionAssignment>& assignment) {
        if (running_.load(std::memory_order_acquire)) {
            RecordError("任务状态机运行中不允许重新绑定任务分配输入Topic");
            return;
        }
        assignment_topic_ = &assignment;
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
    common::Topic<common::MissionAssignmentAck>& AssignmentAckOutput() {
        return assignment_ack_output_;
    }

    /// 测试入口：强制提交一次状态迁移（与生产路径共用 TransitionTo 统一挂点；
    /// 进入/离开动作由决策线程在 Evaluate 末段评估）。
    /// 门控：`test_hooks_enabled=false`（默认，生产装配语义）时拒绝并记 ERROR——
    /// 强推到拦截状态会在 enable_control=true 装配下发出不可逆的捕网触发命令。
    void ForceStateForTest(common::MissionState state) {
        if (!test_hooks_enabled_) {
            RecordError(
                "未启用测试钩子（MissionStateMachineConfig::test_hooks_enabled=false）："
                "拒绝强制状态迁移");
            return;
        }
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
        // 任务分配（65014）：队列里只保留最新一条，由 EvaluateAssignmentEntry
        // 在 S2 处理一次后清空（每条消息只回一条 65015，不按决策周期重发）。
        while (auto message = assignment_subscription_.TryTake()) {
            latest_assignment_ = **message;
            const uint64_t count = ++assignment_receive_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_INFO("状态机收到任务分配: assignment_id={} target_id={} "
                            "type={} flags=0x{:02x} valid_for_ms={} count={}",
                            latest_assignment_->assignment_id,
                            latest_assignment_->target_id,
                            latest_assignment_->interceptor_type,
                            latest_assignment_->flags,
                            latest_assignment_->valid_for_ms, count);
            }
        }
    }

    /// 根据当前状态、飞行快照和目标新鲜度推进影子状态，并周期发布MissionStatus。
    // 影子决策：SELF_CHECK 等待导航就绪，READY 等待新鲜目标，GPS_APPROACH 只做告警维护。
    // 这里刻意不生成控制意图，真实 ARM/TAKEOFF/导航流程需经过后续安全门禁。
    void Evaluate(uint64_t now_ms) {
        uint32_t warnings = 0;
        // 捕网执行器每拍推进（ACK/保持/复位/稳定时序全在执行器内）。
        // 故障或已处于 S16：只维护告警位与状态发布，本拍跳过一切任务控制
        // 逻辑（位置/伺服/返航/降落意图全部停发）。
        if (UpdateNetActuator(now_ms)) {
            SetWarnings(kWarningActuatorFault);
            if (now_ms - last_status_publish_ms_ >= kStatusPublishIntervalMs) {
                PublishStatus(now_ms);
            }
            return;
        }

        if (!HasNavigationForGps()) {
            warnings |= kWarningNavigationUnavailable;
        }
        const bool target_fresh = HasFreshGroundTarget(now_ms);
        const auto state = CurrentState();
        if (state == common::MissionState::kSelfCheck && HasNavigationForGps()) {
            TransitionTo(common::MissionState::kReady, "PX4导航快照满足GPS引导前置条件");
        } else if (state == common::MissionState::kReady && target_fresh &&
                   HasNavigationForGps() && command_sender_ == nullptr) {
            // 安全影子阶段（未注入命令接口）只验证GPS目标进入状态链路；
            // 不执行ARMING/TAKEOFF，不输出控制。注入命令接口后 65010 目标
            // 不得单独把 S2 推到 S5：必须由 65014 分配经 S2 预检进入 S3。
            TransitionTo(common::MissionState::kGpsApproach,
                         "收到有效地面站目标，进入GPS_APPROACH影子验证");
        } else if (state == common::MissionState::kGpsApproach && !target_fresh) {
            warnings |= kWarningGroundTargetStale;
            const uint64_t count = ++target_stale_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("GPS_APPROACH影子状态目标过期: count={}", count);
            }
        }

        // S2 READY → S3 分配入口（2026-10-22）：只在 kReady 处理 65014。
        // 迁移当拍即生效，后续交接/意图评估按新状态（ARMING）执行。
        EvaluateAssignmentEntry(now_ms);

        // S3 解锁/Offboard 与 S4 自动爬升（Task 5，2026-10-22）：放在分配入口
        // 之后、交接/目标/拦截逻辑之前。S3/S4 期间上述逻辑按状态判定均不参与
        // （交接只认 S5/S6/S7，拦截只认 S7/S9/S10）；中止当拍即停止发布起飞
        // 意图，不留下持续控制流。
        EvaluateTakeoffSequence(now_ms);

        // 电池守卫（Task 8）：放在起飞时序之后、交接/拦截之前。飞行中低电/
        // 遥测失效当拍中止到 S12，后续拦截入口（S7/S9/S10）不再有机会发射。
        EvaluateBatteryGuard(now_ms);

        // S5 目标等待/丢失看门狗（Task 10，设计 §7.5）：只在控制任务 S5 生效。
        // 超时当拍中止到 S12 并停发位置引导；后续状态判定自然跳过交接/拦截。
        EvaluateTargetWatchdog(now_ms);

        // 交接评估先于意图发布：S5→S6→S7 迁移当拍即改变位置/视觉意图分支
        // （S5 位置引导 → S6 冻结续发 → S7 视觉伺服，见各函数的当拍收口）。
        EvaluateHandover(now_ms);
        // S9/S10 入口（2026-10-19）：S7 条件满足→S9；自动授权→S10。
        // 放在位置/伺服意图之前，迁移当拍即可按新状态发布（S9/S10 继续伺服）。
        EvaluateInterceptSequence(now_ms);
        // S9→S10 迁移当拍触发捕网（有连接证据才触发；入队失败下拍重试，
        // 超过 fire_timeout 判故障）。
        MaybeTriggerNetActuator(now_ms);
        if (mission_faulted_) {
            SetWarnings(kWarningActuatorFault);
            if (now_ms - last_status_publish_ms_ >= kStatusPublishIntervalMs) {
                PublishStatus(now_ms);
            }
            return;
        }

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
        // 单发拦截结束（S10→S11→S12）与返航/降落（S11~S14，2026-10-19）。
        // 顺序：先完成拦截判定，再执行返航/降落；S12 的位置意图在本拍最后发布。
        EvaluateInterceptionCompletion();
        EvaluateReturnHome(now_ms);
        EvaluateLandingAndDisarm(now_ms);

        // 本拍任一评估触发的故障：保留 bit4（其余告警不再覆盖故障位）。
        if (mission_faulted_) {
            warnings |= kWarningActuatorFault;
        }
        SetWarnings(warnings);
        if (now_ms - last_status_publish_ms_ >= kStatusPublishIntervalMs) {
            PublishStatus(now_ms);
        }
    }

    /// PX4 低电参数是否可用于自动起飞门禁（设计 §9.2）：
    /// BAT_LOW_THR 有效且落在 (0,100)，COM_LOW_BAT_ACT 有效且为 0
    /// （Warning Only；其余动作由 PX4 自行执行，机载不再叠加处置）。
    static bool BatteryParametersReady(
        const common::FlightStateSnapshot& flight) {
        return flight.battery_low_threshold_valid &&
               flight.low_battery_action_valid &&
               flight.battery_low_threshold_pct > 0.f &&
               flight.battery_low_threshold_pct < 100.f &&
               flight.low_battery_action == 0;
    }

    /// 电池遥测是否可用：电压/剩余量有效且百分比为有限值（不伪造 0）。
    static bool BatteryTelemetryReady(
        const common::FlightStateSnapshot& flight) {
        return flight.battery_valid && flight.battery_remaining_valid &&
               std::isfinite(flight.battery_remaining_pct);
    }

    /// 剩余电量是否严格高于 PX4 低电阈值。调用方须先确认参数与遥测有效。
    static bool BatteryAboveThreshold(
        const common::FlightStateSnapshot& flight) {
        return BatteryTelemetryReady(flight) &&
               flight.battery_remaining_pct > flight.battery_low_threshold_pct;
    }

    /// 自动分配/起飞的总电池门禁：参数可判定 + 遥测有效 + 高于阈值。
    static bool BatteryGuardReady(const common::FlightStateSnapshot& flight) {
        return BatteryParametersReady(flight) &&
               BatteryTelemetryReady(flight) &&
               BatteryAboveThreshold(flight);
    }

    /// 65014 分配字段预检（不改变任何状态）。返回 kNone 表示字段与当前
    /// 预检条件全部通过；否则返回 65015 的具体 reason（设计 §5.4/§7.2）。
    /// 注意：调用方必须先处理重复 assignment_id（WasHandled），再做本检查。
    common::MissionAssignmentReason ValidateAssignment(
        const common::MissionAssignment& assignment, uint64_t now_ms) const {
        // 协议版本：当前固定 1。
        if (assignment.protocol_version !=
            communication::kMissionAssignmentProtocolVersion) {
            return common::MissionAssignmentReason::kUnsupportedVersion;
        }
        // flags：只允许已知位，且必须带 bit0（授权自动起飞）。
        if ((assignment.flags &
             static_cast<uint8_t>(~communication::kMissionAssignKnownFlagsMask)) !=
            0U) {
            return common::MissionAssignmentReason::kInvalidMessage;
        }
        if ((assignment.flags & communication::kMissionAssignFlagAuthorized) == 0U) {
            return common::MissionAssignmentReason::kInvalidMessage;
        }
        if (assignment.target_id == 0U) {
            return common::MissionAssignmentReason::kInvalidMessage;
        }
        // 拦截功能类别必须与本机一致（25=捕网，26=火箭）。
        if (assignment.interceptor_type != expected_interceptor_type_) {
            return common::MissionAssignmentReason::kUnsupportedInterceptorType;
        }
        // 分配有效期：按本机收到时刻 + valid_for_ms 判定；0 或已过期均非法。
        if (assignment.valid_for_ms == 0U || assignment.received_ms == 0U ||
            now_ms > assignment.received_ms + assignment.valid_for_ms) {
            return common::MissionAssignmentReason::kInvalidMessage;
        }
        // 控制路径：未注入命令接口（影子装配）或估计输入未绑定时不得起飞。
        if (command_sender_ == nullptr) {
            return common::MissionAssignmentReason::kControlDisabled;
        }
        if (!estimation_inputs_active_) {
            return common::MissionAssignmentReason::kPreflightFailed;
        }
        // 起飞中止后 PX4 状态不确定（断连/离地状态不明等）：置一次性
        // 不可调度标志，新分配回 not_ready，不允许错误地回到 ready。
        if (takeoff_unavailable_latched_) {
            return common::MissionAssignmentReason::kNotReady;
        }
        // PX4 连接与地面/未 armed/导航前置条件（设计 §7.2）。
        if (!latest_flight_.has_value() || !latest_flight_->connected) {
            return common::MissionAssignmentReason::kPx4Disconnected;
        }
        const auto& flight = *latest_flight_;
        if (!flight.landed_state_valid || !flight.landed || flight.armed) {
            return common::MissionAssignmentReason::kPreflightFailed;
        }
        if (!flight.local_position_valid || !flight.attitude_valid ||
            !flight.home_valid) {
            return common::MissionAssignmentReason::kPreflightFailed;
        }
        // 电池门禁（设计 §9.2，Task 8）：唯一来源是 PX4 BAT_LOW_THR /
        // COM_LOW_BAT_ACT 参数与电池遥测；机载不设本地百分比阈值。
        // 参数缺失/非法与遥测失效分开回执，便于地面站定位。
        if (!BatteryParametersReady(flight)) {
            return common::MissionAssignmentReason::kBatteryConfigUnknown;
        }
        if (!BatteryTelemetryReady(flight)) {
            return common::MissionAssignmentReason::kBatteryTelemetryInvalid;
        }
        if (!BatteryAboveThreshold(flight)) {
            return common::MissionAssignmentReason::kBatteryLow;
        }
        // 设备故障：任务故障终态或捕网执行器故障时不允许接受新分配。
        if (mission_faulted_ ||
            (net_actuator_ != nullptr && net_actuator_->Faulted())) {
            return common::MissionAssignmentReason::kMissionFault;
        }
        return common::MissionAssignmentReason::kNone;
    }

    /// 任务分配入口（2026-10-22）。
    /// 每条 65014 只消费一次（DrainInputs 保存最新一条，处理前清空，避免按
    /// 决策周期重复回 65015）。重复 assignment_id 在任何阶段都重发原 65015；
    /// 新分配只在 S2 预检，非 S2 一律拒绝且不重试。
    void EvaluateAssignmentEntry(uint64_t now_ms) {
        if (!latest_assignment_.has_value()) {
            return;
        }
        const common::MissionAssignment assignment = *latest_assignment_;
        latest_assignment_.reset();

        // 取消当前分配（flags.bit1，设计 §5.1/§7.5）：必须先于重复/普通预检
        // 处理——取消消息不要求 bit0，且活动分配在 tracker 中已记住 accepted，
        // 若先走 WasHandled 会把取消误当作重复分配重发 accepted。
        if ((assignment.flags & communication::kMissionAssignFlagCancel) != 0U) {
            HandleAssignmentCancel(assignment, now_ms);
            return;
        }

        // 重复 assignment_id：只重发原 65015，不再预检/迁移（幂等）。
        common::MissionAssignmentResult prior_result =
            common::MissionAssignmentResult::kRejected;
        common::MissionAssignmentReason prior_reason =
            common::MissionAssignmentReason::kNone;
        bool duplicate = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            duplicate = assignment_tracker_.WasHandled(
                assignment.assignment_id, &prior_result, &prior_reason);
        }
        if (duplicate) {
            PublishAssignmentAck(assignment, prior_result, prior_reason);
            const uint64_t count = ++assignment_duplicate_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_INFO("重复任务分配只重发65015: assignment_id={} result={} "
                            "reason={} count={}",
                            assignment.assignment_id,
                            static_cast<int>(prior_result),
                            static_cast<int>(prior_reason), count);
            }
            return;
        }

        // 任务已开始（S3~S16）时收到新 assignment_id：拒绝并记录结果，
        // 不打断当前任务；重复 id 已在上面重发原回执。
        if (CurrentState() != common::MissionState::kReady) {
            const auto reason = common::MissionAssignmentReason::kAlreadyAssigned;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                assignment_tracker_.RememberResult(
                    assignment.assignment_id,
                    common::MissionAssignmentResult::kRejected, reason);
            }
            PublishAssignmentAck(assignment,
                                 common::MissionAssignmentResult::kRejected,
                                 reason);
            const uint64_t count = ++assignment_reject_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("任务执行中拒绝新分配: assignment_id={} state={} count={}",
                            assignment.assignment_id,
                            MissionStateName(CurrentState()), count);
            }
            return;
        }

        auto reason = ValidateAssignment(assignment, now_ms);
        bool accepted = false;
        if (reason == common::MissionAssignmentReason::kNone) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // 一次上电一次空中任务：非 kIdle 或已消耗机会时拒绝。
            if (!assignment_tracker_.Accept(assignment, now_ms)) {
                reason = common::MissionAssignmentReason::kAlreadyAssigned;
            } else {
                assignment_tracker_.RememberResult(
                    assignment.assignment_id,
                    common::MissionAssignmentResult::kAccepted,
                    common::MissionAssignmentReason::kNone);
                // 保存已接受的分配（id/target）：S3/S4 中止时 latest_assignment_
                // 早已清空，65015 aborted 必须用这份副本。
                accepted_assignment_ = assignment;
                // 新任务重新武装电池守卫（地面低电中止后恢复再分配的路径）。
                battery_abort_latched_ = false;
                battery_low_since_ms_.reset();
                // 目标看门狗与中止锁存按任务复位（一次任务只中止一次）。
                mission_abort_latched_ = false;
                target_wait_started_ms_ = 0;
                assigned_target_acquired_ = false;
                target_lost_since_ms_.reset();
                // 记录起飞原点（局部 NED），保证 tracker 阶段与状态机一致。
                const auto& flight = *latest_flight_;
                assignment_tracker_.BeginTakeoff(
                    flight.local_x_m, flight.local_y_m, flight.local_z_m, now_ms);
                accepted = true;
            }
        }

        if (!accepted) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                assignment_tracker_.RememberResult(
                    assignment.assignment_id,
                    common::MissionAssignmentResult::kRejected, reason);
            }
            PublishAssignmentAck(assignment,
                                 common::MissionAssignmentResult::kRejected,
                                 reason);
            const uint64_t count = ++assignment_reject_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("任务分配被拒绝: assignment_id={} target_id={} "
                            "reason={} state={} count={}",
                            assignment.assignment_id, assignment.target_id,
                            static_cast<int>(reason),
                            MissionStateName(CurrentState()), count);
            }
            return;  // 保持 S2；不自动重试、不消耗任务机会
        }

        // 成功：先回 accepted（tracker 已进入 kAssigned/kTakingOff，
        // availability=assigned），再迁移 S3。S3 入口动作（176/400、
        // 每拍 kTakeoff 原点意图）由 Task 5 实现。
        PublishAssignmentAck(assignment,
                             common::MissionAssignmentResult::kAccepted,
                             common::MissionAssignmentReason::kNone);
        SPDLOG_INFO("任务分配已接受: assignment_id={} target_id={} origin=({:.2f},"
                    "{:.2f},{:.2f}) → S3 ARMING",
                    assignment.assignment_id, assignment.target_id,
                    static_cast<double>(assignment_tracker_.OriginX()),
                    static_cast<double>(assignment_tracker_.OriginY()),
                    static_cast<double>(assignment_tracker_.OriginZ()));
        TransitionTo(common::MissionState::kArming, "接受任务分配");
    }

    // ---- 65014 取消（flags.bit1，Task 10，设计 §5.1/§7.5） ----

    /// 取消动作最小间隔：同一取消在 500ms 内不重复下发命令（幂等重发回执）。
    static constexpr uint64_t kCancelCommandMinIntervalMs = 500;

    /// 任务执行阶段（已离地）：S5~S13。S14/S16 为终态，不再重启返航链。
    static bool IsAirborneMissionState(common::MissionState state) {
        switch (state) {
            case common::MissionState::kGpsApproach:
            case common::MissionState::kVisualHandover:
            case common::MissionState::kVisualTracking:
            case common::MissionState::kObstacleHold:
            case common::MissionState::kInterceptReady:
            case common::MissionState::kInterceptExecuting:
            case common::MissionState::kPostIntercept:
            case common::MissionState::kReturnHome:
            case common::MissionState::kLanding:
                return true;
            default:
                return false;
        }
    }

    /// 本机是否已实际离地：tracker 已消耗本次空中任务机会，或当前飞行快照
    /// armed && !landed。取消处置以此决定"地面中止"还是"空中返航"。
    bool AssignmentAirborne() const {
        if (assignment_tracker_.ConsumedAirborneMission()) {
            return true;
        }
        return latest_flight_.has_value() && latest_flight_->armed &&
               !latest_flight_->landed;
    }

    /// 取消消息被拒绝：记住结果（重复取消重发同一拒绝回执）并发布 65015。
    void RejectAssignmentCancel(const common::MissionAssignment& assignment,
                                common::MissionAssignmentReason reason) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            assignment_tracker_.RememberResult(
                assignment.assignment_id,
                common::MissionAssignmentResult::kRejected, reason);
        }
        PublishAssignmentAck(assignment,
                             common::MissionAssignmentResult::kRejected, reason);
        const uint64_t count = ++assignment_reject_count_;
        if (ShouldLogThrottled(count)) {
            SPDLOG_WARN("任务取消被拒绝: assignment_id={} reason={} state={} "
                        "count={}",
                        assignment.assignment_id, static_cast<int>(reason),
                        MissionStateName(CurrentState()), count);
        }
    }

    /// 处理 65014 flags.bit1。协议字段只做版本/未知位校验（取消不要求 bit0）：
    /// - 活动分配 id 匹配：地面 → AbortTakeoff/AbortOnGround 回 READY；空中 →
    ///   AbortAirborne + S12，65015 cancelled/aborted(cancelled_by_gcs)；
    /// - 重复取消：重发最近结果（cancelled 结果不被 accepted 覆盖）；
    /// - 无活动分配且 id 从未处理：rejected(not_ready)；
    /// - 活动分配是其他 id：rejected(already_assigned)。
    /// 取消不重启已进入 S12/S13 的返航/降落链，只记录并回执。
    void HandleAssignmentCancel(const common::MissionAssignment& assignment,
                                uint64_t now_ms) {
        if (assignment.protocol_version !=
            communication::kMissionAssignmentProtocolVersion) {
            RejectAssignmentCancel(
                assignment,
                common::MissionAssignmentReason::kUnsupportedVersion);
            return;
        }
        if ((assignment.flags &
             static_cast<uint8_t>(~communication::kMissionAssignKnownFlagsMask)) !=
            0U) {
            RejectAssignmentCancel(
                assignment,
                common::MissionAssignmentReason::kInvalidMessage);
            return;
        }

        common::MissionAssignmentResult prior_result =
            common::MissionAssignmentResult::kRejected;
        common::MissionAssignmentReason prior_reason =
            common::MissionAssignmentReason::kNone;
        bool handled = false;
        bool has_active = false;
        bool active_id_match = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            handled = assignment_tracker_.WasHandled(
                assignment.assignment_id, &prior_result, &prior_reason);
            has_active = assignment_tracker_.HasActiveAssignment();
            active_id_match = has_active &&
                assignment_tracker_.AssignmentId() == assignment.assignment_id;
        }

        if (active_id_match) {
            // 该分配的取消已生效（空中取消后 tracker 仍 kReturning）：直接
            // 重发最近结果，不再迁移/下发命令。
            if (handled &&
                prior_result != common::MissionAssignmentResult::kAccepted) {
                PublishAssignmentAck(assignment, prior_result, prior_reason);
                return;
            }
            // 500ms 内已执行过取消动作：不重复下发命令（含 DISARM），重发
            // 当前取消结论。
            if (last_cancel_action_ms_.has_value() &&
                ElapsedMs(now_ms, *last_cancel_action_ms_) <
                    kCancelCommandMinIntervalMs) {
                PublishAssignmentAck(
                    assignment,
                    common::MissionAssignmentResult::kCancelledOrAborted,
                    common::MissionAssignmentReason::kCancelledByGcs);
                return;
            }
            last_cancel_action_ms_ = now_ms;
            ExecuteAssignmentCancel(assignment);
            return;
        }

        if (handled) {
            // 已处理过：重发最终结果。已被接受但已不在活动阶段（S14/S16
            // 终态）的分配按协议回 rejected(not_ready)（§5.5）。
            if (prior_result != common::MissionAssignmentResult::kAccepted) {
                PublishAssignmentAck(assignment, prior_result, prior_reason);
                return;
            }
            RejectAssignmentCancel(
                assignment, common::MissionAssignmentReason::kNotReady);
            return;
        }

        // 未匹配当前活动分配：有活动分配说明是其他 id（already_assigned），
        // 否则无分配/终态（not_ready）。
        RejectAssignmentCancel(
            assignment, has_active
                            ? common::MissionAssignmentReason::kAlreadyAssigned
                            : common::MissionAssignmentReason::kNotReady);
    }

    /// 执行已匹配活动分配的取消（调用方已确认 id 匹配并记录 500ms 动作时刻）。
    void ExecuteAssignmentCancel(const common::MissionAssignment& assignment) {
        const auto state = CurrentState();
        const bool airborne =
            AssignmentAirborne() || IsAirborneMissionState(state);

        if (airborne) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                assignment_tracker_.AbortAirborne();
            }
            RememberTakeoffAbort(
                common::MissionAssignmentReason::kCancelledByGcs);
            SPDLOG_WARN("地面站取消任务（已离地）: assignment_id={} state={} → "
                        "中止任务并返航",
                        assignment.assignment_id, MissionStateName(state));
            // 已在返航/降落链（S12~S14）：只记录并回执，不把 S13 重置回 S12。
            if (state == common::MissionState::kReturnHome ||
                state == common::MissionState::kLanding ||
                state == common::MissionState::kDisarmed) {
                PublishTakeoffAbortAck(
                    common::MissionAssignmentReason::kCancelledByGcs);
                return;
            }
            // 中止当拍停发位置引导，S12 在本拍后续接管返航意图。
            StopPositionIntent("地面站取消任务（已离地）");
            TransitionTo(common::MissionState::kReturnHome,
                         "地面站取消（已离地），进入返航");
            PublishTakeoffAbortAck(
                common::MissionAssignmentReason::kCancelledByGcs);
            return;
        }

        if (state == common::MissionState::kArming ||
            state == common::MissionState::kTakeoff) {
            // 地面 S3/S4：复用统一起飞中止（停流、必要时一次 DISARM、
            // tracker AbortOnGround、回 READY、65015 cancelled/aborted）。
            AbortTakeoff(common::MissionAssignmentReason::kCancelledByGcs,
                         "地面站取消");
            return;
        }

        // 防御路径：S2 已接受但尚未进入 S3（正常路径接受当拍即迁移 S3）。
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            assignment_tracker_.AbortOnGround();
        }
        RememberTakeoffAbort(
            common::MissionAssignmentReason::kCancelledByGcs);
        SPDLOG_WARN("地面站取消任务（地面）: assignment_id={} state={} → 保持 "
                    "READY",
                    assignment.assignment_id, MissionStateName(state));
        PublishTakeoffAbortAck(
            common::MissionAssignmentReason::kCancelledByGcs);
    }

    // ---- S3 解锁/Offboard 与 S4 自动爬升（Task 5，2026-10-22） ----

    /// 单调时钟差值（无符号回绕保护：刚迁移时 now 可能早于基准）。
    static uint64_t ElapsedMs(uint64_t now_ms, uint64_t since_ms) {
        return now_ms >= since_ms ? now_ms - since_ms : 0;
    }

    /// 是否处于 PX4 Offboard（6/0，实际心跳模式）。
    static bool IsPx4Offboard(const common::FlightStateSnapshot& flight) {
        return flight.flight_mode == kPx4FlightMainModeOffboard &&
               flight.flight_sub_mode == kPx4FlightSubModeOffboard;
    }

    /// S3/S4 起飞时序入口：每决策拍调用一次（10Hz）。
    ///
    /// 节拍顺序：连接/快照检查 → 实际离地记录（MarkAirborne，幂等）→
    /// S3 模式/解锁时序或 S4 爬升推进 → 仍在起飞段则发布 kTakeoff 意图。
    /// 中止/迁移发生在意图发布之前：故障当拍不再发布任何起飞意图。
    void EvaluateTakeoffSequence(uint64_t now_ms) {
        const auto state = CurrentState();
        if (state != common::MissionState::kArming &&
            state != common::MissionState::kTakeoff) {
            return;
        }
        if (command_sender_ == nullptr) {
            // 分配预检已要求命令接口，但强制状态/装配异常仍需兜底：不得
            // 在无命令通道时继续起飞，也不得空指针解引用。
            AbortTakeoff(common::MissionAssignmentReason::kControlDisabled,
                         "S3/S4 缺少 PX4 命令接口");
            return;
        }
        if (!latest_flight_.has_value()) {
            AbortTakeoff(common::MissionAssignmentReason::kPx4Disconnected,
                         "S3/S4 缺少飞行状态快照");
            return;
        }
        const auto& flight = *latest_flight_;
        if (!flight.connected) {
            AbortTakeoff(common::MissionAssignmentReason::kPx4Disconnected,
                         "S3/S4 期间 PX4 断连");
            return;
        }
        // 实际离地即消耗本次上电唯一的空中任务机会（PX4 判定，不依赖
        // 已进入 S5）。S3/S4 每拍调用，tracker 内部幂等。
        if (flight.armed && !flight.landed) {
            assignment_tracker_.MarkAirborne(now_ms);
        }

        if (state == common::MissionState::kArming) {
            EvaluateArmingSequence(flight, now_ms);
        } else {
            EvaluateClimbSequence(flight, now_ms);
        }

        // 迁移（S3→S4、S4→S5）或中止后按当拍新状态决定是否继续发意图：
        // - 仍在 S3：原点保持（z=z0）；
        // - 已进入 S4：起飞意图目标 z 为逐拍推进的爬升值；
        // - 已中止（S2/S12/S13）：停发，不留下持续控制流。
        const auto after = CurrentState();
        if (after == common::MissionState::kArming) {
            PublishTakeoffIntent(assignment_tracker_.OriginZ(), now_ms);
        } else if (after == common::MissionState::kTakeoff) {
            PublishTakeoffIntent(takeoff_z_cmd_, now_ms);
        }
    }

    // ---- 飞行中电池守卫（Task 8，2026-10-22） ----

    /// 电池守卫生效阶段：S3/S4 与 S5~S10。S11 起由拦截完成/返航链处置；
    /// S12/S13/S14/S16 不重复触发（一次任务只中止一次）。
    static bool BatteryGuardActiveState(common::MissionState state) {
        switch (state) {
            case common::MissionState::kArming:
            case common::MissionState::kTakeoff:
            case common::MissionState::kGpsApproach:
            case common::MissionState::kVisualHandover:
            case common::MissionState::kVisualTracking:
            case common::MissionState::kObstacleHold:
            case common::MissionState::kInterceptReady:
            case common::MissionState::kInterceptExecuting:
                return true;
            default:
                return false;
        }
    }

    /// 飞行中电池守卫（每决策拍）：遥测失效立即中止；电池有效且
    /// pct <= BAT_LOW_THR 连续 `takeoff.battery_low_debounce` 后中止。
    /// 恢复高于阈值即清零防抖计时。机载不设本地百分比阈值。
    void EvaluateBatteryGuard(uint64_t now_ms) {
        // 影子装配（未注入命令接口）没有真实任务与控制权：S5 影子验证
        // 路径保持既有语义，电池守卫不中止/不迁移。
        if (command_sender_ == nullptr) {
            battery_low_since_ms_.reset();
            return;
        }
        if (!BatteryGuardActiveState(CurrentState())) {
            battery_low_since_ms_.reset();
            return;
        }
        if (battery_abort_latched_) {
            return;
        }
        if (!latest_flight_.has_value()) {
            // 无飞行快照无法确认电池状态：按遥测失效处理（S3/S4 的断连
            // 路径已先于本拍收口，这里主要覆盖 S5~S10）。
            TriggerBatteryAbort(
                common::MissionAssignmentReason::kBatteryTelemetryInvalid,
                "飞行中缺少飞行状态快照");
            return;
        }
        const auto& flight = *latest_flight_;
        if (!BatteryTelemetryReady(flight)) {
            TriggerBatteryAbort(
                common::MissionAssignmentReason::kBatteryTelemetryInvalid,
                "电池遥测失效（电压/剩余量无效）");
            return;
        }
        // 参数在飞行中不可判定时不做低电比较（避免把未知阈值当 0），
        // 参数有效性由 S2 预检在起飞前保证。
        const bool low = BatteryParametersReady(flight) &&
                         flight.battery_remaining_pct <=
                             flight.battery_low_threshold_pct;
        if (!low) {
            battery_low_since_ms_.reset();
            return;
        }
        if (!battery_low_since_ms_.has_value()) {
            battery_low_since_ms_ = now_ms;
        }
        if (ElapsedMs(now_ms, *battery_low_since_ms_) <
            static_cast<uint64_t>(takeoff_.battery_low_debounce.count())) {
            return;
        }
        TriggerBatteryAbort(common::MissionAssignmentReason::kBatteryLow,
                            "电池剩余电量不高于 PX4 BAT_LOW_THR");
    }

    /// 电池中止统一处置（一次任务只执行一次）：
    /// - S3/S4：复用 AbortTakeoff（已离地 → S12；地面 → 停流/必要时
    ///   DISARM → READY）；
    /// - S5~S10：tracker AbortAirborne + 65015 aborted → S12 机载位置返航；
    ///   不走 PX4 RTL，不触发新的拦截/发射。
    void TriggerBatteryAbort(common::MissionAssignmentReason reason,
                             const char* detail) {
        if (battery_abort_latched_) {
            return;
        }
        battery_abort_latched_ = true;
        battery_low_since_ms_.reset();
        const auto state = CurrentState();
        if (state == common::MissionState::kArming ||
            state == common::MissionState::kTakeoff) {
            AbortTakeoff(reason, detail);
            return;
        }
        assignment_tracker_.AbortAirborne();
        RememberTakeoffAbort(reason);
        SPDLOG_WARN("飞行中电池中止: reason={} detail={} state={} → S12 返航",
                    static_cast<int>(reason), detail, MissionStateName(state));
        TransitionTo(common::MissionState::kReturnHome,
                     "电池低电/遥测失效，中止任务并返航");
        PublishTakeoffAbortAck(reason);
    }

    // ---- S5 目标等待/丢失看门狗（Task 10，设计 §7.5） ----

    /// S5 入口初始化（TransitionTo 调用）：仅在控制任务下启用（影子模式
    /// 保持既有 S5 行为，不因目标缺失自动中止）。
    void BeginTargetWatchdog(uint64_t now_ms) {
        target_lost_since_ms_.reset();
        if (command_sender_ == nullptr) {
            target_wait_started_ms_ = 0;
            assigned_target_acquired_ = false;
            return;
        }
        target_wait_started_ms_ = now_ms;
        assigned_target_acquired_ = false;
        SPDLOG_INFO("S5 目标看门狗启动: target_id={} wait_timeout={}ms "
                    "lost_timeout={}ms",
                    assignment_tracker_.TargetId(),
                    takeoff_.target_wait_timeout.count(),
                    takeoff_.target_lost_timeout.count());
    }

    /// 每决策拍（仅 S5、仅控制任务）：未获得匹配目标超过 target_wait_timeout，
    /// 或已获得后匹配目标持续失效超过 target_lost_timeout → 中止分配、S12 返航、
    /// 65015 aborted(target_wait_timeout/target_lost)。一次任务只中止一次。
    /// S6/S7 的交接/视觉丢失逻辑不受本函数影响。
    void EvaluateTargetWatchdog(uint64_t now_ms) {
        if (command_sender_ == nullptr) {
            return;  // 影子模式不启用
        }
        if (CurrentState() != common::MissionState::kGpsApproach) {
            return;
        }
        if (mission_abort_latched_) {
            return;
        }
        if (HasFreshAssignedTarget(now_ms)) {
            assigned_target_acquired_ = true;
            target_lost_since_ms_.reset();
            return;
        }
        if (!assigned_target_acquired_) {
            if (ElapsedMs(now_ms, target_wait_started_ms_) >=
                static_cast<uint64_t>(
                    takeoff_.target_wait_timeout.count())) {
                AbortForTargetWatchdog(
                    common::MissionAssignmentReason::kTargetWaitTimeout,
                    "起飞后等待分配目标超时");
            }
            return;
        }
        if (!target_lost_since_ms_.has_value()) {
            target_lost_since_ms_ = now_ms;
        }
        if (ElapsedMs(now_ms, *target_lost_since_ms_) >=
            static_cast<uint64_t>(takeoff_.target_lost_timeout.count())) {
            AbortForTargetWatchdog(
                common::MissionAssignmentReason::kTargetLost,
                "已获得目标持续失效超时");
        }
    }

    /// 目标看门狗中止统一处置：停发位置引导（S12 当拍接管返航意图）、
    /// tracker AbortAirborne、65015 aborted → S12；不触发拦截/187。
    void AbortForTargetWatchdog(common::MissionAssignmentReason reason,
                                const char* detail) {
        if (mission_abort_latched_) {
            return;
        }
        mission_abort_latched_ = true;
        target_lost_since_ms_.reset();
        StopPositionIntent("目标等待/丢失超时中止任务");
        assignment_tracker_.AbortAirborne();
        RememberTakeoffAbort(reason);
        SPDLOG_WARN("S5 目标看门狗中止: reason={} detail={} → S12 返航",
                    static_cast<int>(reason), detail);
        TransitionTo(common::MissionState::kReturnHome,
                     "目标等待/丢失超时，中止任务并返航");
        PublishTakeoffAbortAck(reason);
    }

    /// S3：每拍原点保持意图已由入口发布；等待 bootstrap_lead 后只发一次
    /// DO_SET_MODE(176)，等待实际 6/0 后只发一次 ARM_DISARM(400)，再等待
    /// 实际 armed。超时/ACK 非 ACCEPTED/入队失败一律中止，不重试。
    void EvaluateArmingSequence(const common::FlightStateSnapshot& flight,
                                uint64_t now_ms) {
        switch (takeoff_stage_) {
            case TakeoffStage::kOriginHold: {
                // 先让 FlightController 的 kBootstrap 有若干拍合法原点保持流
                // （Task 6 线程侧实现；本任务只保证意图持续发布）。
                const uint64_t held_ms = ElapsedMs(now_ms, state_entered_ms_);
                if (held_ms <
                    static_cast<uint64_t>(takeoff_.bootstrap_lead.count())) {
                    return;
                }
                const bool queued = command_sender_->SendCommand(
                    kMavCmdDoSetMode, kMavModeFlagCustomModeEnabled,
                    kPx4MainModeOffboard, kPx4SubModeOffboard, 0.f, 0.f, 0.f,
                    0.f);
                if (!queued) {
                    // 命令入队失败同样中止，不重试（不原地循环重发）。
                    AbortTakeoff(common::MissionAssignmentReason::kBootstrapTimeout,
                                 "176 DO_SET_MODE 入队失败");
                    return;
                }
                takeoff_stage_ = TakeoffStage::kWaitOffboard;
                takeoff_stage_ms_ = now_ms;
                SPDLOG_INFO("S3 起飞模式命令已入队: DO_SET_MODE(176) p1={} p2={} "
                            "p3={}（本次起飞仅一次）",
                            static_cast<double>(kMavModeFlagCustomModeEnabled),
                            static_cast<double>(kPx4MainModeOffboard),
                            static_cast<double>(kPx4SubModeOffboard));
                return;
            }
            case TakeoffStage::kWaitOffboard: {
                if (TakeoffAckRejected(kMavCmdDoSetMode)) {
                    AbortTakeoff(common::MissionAssignmentReason::kModeTimeout,
                                 "176 ACK 非 ACCEPTED");
                    return;
                }
                if (IsPx4Offboard(flight)) {
                    // 实际 6/0 才发解锁命令（ACK 只是过程反馈）。
                    const bool queued = command_sender_->SendCommand(
                        kMavCmdComponentArmDisarm, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f,
                        0.f);
                    if (!queued) {
                        AbortTakeoff(common::MissionAssignmentReason::kArmTimeout,
                                     "400 ARM_DISARM 入队失败");
                        return;
                    }
                    takeoff_stage_ = TakeoffStage::kWaitArmed;
                    takeoff_stage_ms_ = now_ms;
                    SPDLOG_INFO("S3 已进入 Offboard(6/0)，解锁命令已入队: "
                                "ARM_DISARM(400) p1=1（本次起飞仅一次）");
                    return;
                }
                if (ElapsedMs(now_ms, takeoff_stage_ms_) >=
                    static_cast<uint64_t>(takeoff_.mode_timeout.count())) {
                    AbortTakeoff(common::MissionAssignmentReason::kModeTimeout,
                                 "等待实际 Offboard(6/0) 超时");
                }
                return;
            }
            case TakeoffStage::kWaitArmed: {
                // ACK 非 ACCEPTED（0）且非 IN_PROGRESS（5）立即中止；
                // 实际 armed 是最终判据。
                if (TakeoffAckRejected(kMavCmdComponentArmDisarm)) {
                    AbortTakeoff(common::MissionAssignmentReason::kArmTimeout,
                                 "400 ACK 非 ACCEPTED");
                    return;
                }
                if (TakeoffAckRejected(kMavCmdDoSetMode)) {
                    AbortTakeoff(common::MissionAssignmentReason::kModeTimeout,
                                 "176 ACK 非 ACCEPTED（等待解锁期间）");
                    return;
                }
                if (flight.armed && IsPx4Offboard(flight)) {
                    // armed + Offboard 是 S3 的最终成功判据；S4 由
                    // TransitionTo 初始化爬升时序。
                    TransitionTo(common::MissionState::kTakeoff,
                                 "PX4 已解锁且处于 Offboard");
                    return;
                }
                if (!IsPx4Offboard(flight)) {
                    AbortTakeoff(common::MissionAssignmentReason::kModeTimeout,
                                 "等待解锁期间 PX4 离开 Offboard");
                    return;
                }
                if (ElapsedMs(now_ms, takeoff_stage_ms_) >=
                    static_cast<uint64_t>(takeoff_.arm_timeout.count())) {
                    AbortTakeoff(common::MissionAssignmentReason::kArmTimeout,
                                 "等待实际 armed 超时");
                }
                return;
            }
            case TakeoffStage::kClimb:
            case TakeoffStage::kNone:
                return;
        }
    }

    /// S4：水平保持原点，z 目标按 climb_rate 逐拍推进到 z0-height（NED 向上
    /// 为负），到达判据（z/xy/vz）持续 stable_time 后进 S5。
    void EvaluateClimbSequence(const common::FlightStateSnapshot& flight,
                               uint64_t now_ms) {
        // S4 只在 PX4 仍 armed + Offboard 时继续爬升。离开 Offboard（人工/
        // failsafe/RTL）或空中失锁必须当拍停止输出：空中转 S13 仅监控；
        // 地面走统一地面中止。
        if (!flight.armed || !IsPx4Offboard(flight)) {
            if (!flight.landed) {
                assignment_tracker_.AbortAirborne();
                RememberTakeoffAbort(
                    common::MissionAssignmentReason::kModeTimeout);
                if (!flight.armed) {
                    takeoff_unavailable_latched_ = true;
                }
                return_px4_takeover_ = true;
                SPDLOG_WARN("S4 中 PX4 离开 Offboard 或失锁: mode={}/{} armed={} "
                            "landed={}，停止爬升并让权（S13 仅监控）",
                            flight.flight_mode, flight.flight_sub_mode,
                            flight.armed, flight.landed);
                TransitionTo(common::MissionState::kLanding,
                             "S4 PX4 接管/离开 Offboard");
                PublishTakeoffAbortAck(
                    common::MissionAssignmentReason::kModeTimeout);
            } else {
                AbortTakeoff(common::MissionAssignmentReason::kModeTimeout,
                             "S4 仍在地面但未 armed/Offboard");
            }
            return;
        }
        const double goal_z =
            static_cast<double>(assignment_tracker_.OriginZ()) - takeoff_.height_m;
        // 逐拍推进，不阶跃；首拍 dt=0。时间回绕/重启双保险钳到 0。
        const uint64_t dt_ms =
            now_ms >= takeoff_last_tick_ms_ ? now_ms - takeoff_last_tick_ms_ : 0;
        takeoff_last_tick_ms_ = now_ms;
        if (dt_ms > 0) {
            const double step = takeoff_.climb_rate_mps *
                                static_cast<double>(dt_ms) / 1000.0;
            const double candidate = static_cast<double>(takeoff_z_cmd_) - step;
            takeoff_z_cmd_ =
                static_cast<float>(candidate <= goal_z ? goal_z : candidate);
        }

        // 空中超时：PX4 仍在 Offboard → S13 发 AUTO.LAND（现有
        // EvaluateLandingAndDisarm）；已离开 Offboard → 停发让权，S13 仅监控。
        // 地面超时：停流 + 必要时一次 DISARM + 回 S2（状态不明则 unavailable）。
        if (ElapsedMs(now_ms, state_entered_ms_) >=
            static_cast<uint64_t>(takeoff_.height_timeout.count())) {
            HandleTakeoffHeightTimeout(flight);
            return;
        }

        if (!flight.local_position_valid) {
            takeoff_stable_since_ms_ = 0;
            return;
        }
        const double z_error =
            std::fabs(static_cast<double>(flight.local_z_m) - goal_z);
        const double xy_error = std::hypot(
            static_cast<double>(flight.local_x_m) -
                static_cast<double>(assignment_tracker_.OriginX()),
            static_cast<double>(flight.local_y_m) -
                static_cast<double>(assignment_tracker_.OriginY()));
        const double vertical_speed =
            std::fabs(static_cast<double>(flight.vz_mps));
        const bool inside = z_error <= takeoff_.z_tolerance_m &&
                            xy_error <= takeoff_.xy_tolerance_m &&
                            vertical_speed <= takeoff_.max_vz_mps;
        if (!inside) {
            takeoff_stable_since_ms_ = 0;
            return;
        }
        if (takeoff_stable_since_ms_ == 0) {
            takeoff_stable_since_ms_ = now_ms;
        }
        if (ElapsedMs(now_ms, takeoff_stable_since_ms_) <
            static_cast<uint64_t>(takeoff_.stable_time.count())) {
            return;
        }
        assignment_tracker_.TakeoffComplete();
        assignment_tracker_.BeginEngage();
        SPDLOG_INFO("起飞到达任务高度: z={:.2f}（原点 z0={:.2f}，高度 {:.2f}m），"
                    "进入 GPS_APPROACH",
                    static_cast<double>(flight.local_z_m),
                    static_cast<double>(assignment_tracker_.OriginZ()),
                    takeoff_.height_m);
        TransitionTo(common::MissionState::kGpsApproach, "起飞到达任务高度");
    }

    /// S3/S4 期间任务故障（S16）的起飞收口：按实际是否离地推进 tracker，
    /// 记 aborted(mission_fault) 并回执；状态迁移由 HandleMissionFault 统一
    /// 进入 S16（故障终态，不可调度）。
    void AbortTakeoffForMissionFault(const char* detail) {
        const auto state = CurrentState();
        if (state != common::MissionState::kArming &&
            state != common::MissionState::kTakeoff) {
            return;
        }
        if (!accepted_assignment_.has_value()) {
            return;
        }
        const bool airborne = latest_flight_.has_value() &&
                              latest_flight_->armed && !latest_flight_->landed;
        if (airborne) {
            assignment_tracker_.AbortAirborne();
        } else {
            assignment_tracker_.AbortOnGround();
        }
        takeoff_unavailable_latched_ = true;
        RememberTakeoffAbort(common::MissionAssignmentReason::kMissionFault);
        SPDLOG_WARN("起飞段任务故障，65015 aborted(mission_fault): {}", detail);
        PublishTakeoffAbortAck(common::MissionAssignmentReason::kMissionFault);
    }

    /// S4 高度超时处置（空中：S13 / 地面：S2）。
    void HandleTakeoffHeightTimeout(
        const common::FlightStateSnapshot& flight) {
        if (!(flight.armed && !flight.landed)) {
            // 尚未实际离地：按地面中止收口（停流/必要时 DISARM/回 S2）。
            AbortTakeoff(common::MissionAssignmentReason::kTakeoffTimeout,
                         "S4 等待到达任务高度超时（仍在地面）");
            return;
        }
        const bool offboard = IsPx4Offboard(flight);
        assignment_tracker_.AbortAirborne();
        RememberTakeoffAbort(common::MissionAssignmentReason::kTakeoffTimeout);
        // S13 入口按 return_px4_takeover_ 决定发 AUTO.LAND 还是仅监控：
        // PX4 已离开 Offboard 时停发让权，不再争控。
        return_px4_takeover_ = !offboard;
        SPDLOG_WARN("起飞爬升超时（height_timeout={}ms）：{}",
                    takeoff_.height_timeout.count(),
                    offboard ? "PX4 仍在 Offboard，进入 S13 发 AUTO.LAND"
                             : "PX4 已离开 Offboard，停发让权并仅监控（S13）");
        TransitionTo(common::MissionState::kLanding,
                     offboard ? "起飞爬升超时，AUTO.LAND"
                              : "起飞爬升超时，PX4 已接管");
        PublishTakeoffAbortAck(common::MissionAssignmentReason::kTakeoffTimeout);
    }

    /// 176/400 的 ACK 是否明确失败。只认 S3 入口之后新到的 ACK
    /// （matched_count 基线），避免上一次任务/命令的陈旧 ACK 误伤本次起飞。
    /// MAV_RESULT：0=ACCEPTED，5=IN_PROGRESS，其余（含 4=FAILED）视为失败。
    bool TakeoffAckRejected(uint16_t command) const {
        if (command_sender_ == nullptr) {
            return false;
        }
        const auto ack = command_sender_->AckStatus();
        if (ack.matched_count <= takeoff_ack_baseline_matched_) {
            return false;
        }
        return ack.last_command == command && ack.last_result != 0 &&
               ack.last_result != 5;
    }

    /// S3/S4 中止统一处置：
    /// - 已实际离地（armed && !landed）：tracker AbortAirborne → S12，
    ///   **不空中 disarm**；
    /// - 地面：停发意图；若 armed+landed 只发一次 DISARM；tracker
    ///   AbortOnGround → S2 READY；
    /// - 状态不确定（无快照/断连/离地状态不明）：置 per-boot 不可调度标志，
    ///   65013/65015 availability=unavailable，不错误回到 ready。
    /// 65015 aborted 使用已保存的 accepted_assignment_（latest_assignment_
    /// 在处理 65014 时已清空）。
    void AbortTakeoff(common::MissionAssignmentReason reason,
                      const char* detail) {
        const bool have_flight = latest_flight_.has_value();
        const bool armed = have_flight && latest_flight_->armed;
        const bool landed = have_flight && latest_flight_->landed;
        const bool landed_known = have_flight && latest_flight_->landed_state_valid;
        const bool connected = have_flight && latest_flight_->connected;
        const bool airborne = have_flight && armed && !landed;
        const bool uncertain = !have_flight || !connected || !landed_known;

        SPDLOG_WARN("自动起飞中止: reason={} detail={} airborne={} uncertain={}",
                    static_cast<int>(reason), detail, airborne, uncertain);
        if (uncertain) {
            takeoff_unavailable_latched_ = true;
        }
        if (airborne) {
            assignment_tracker_.AbortAirborne();
            RememberTakeoffAbort(reason);
            // 空中中止交返航链（S12），不在此处 disarm；回执按新状态发布。
            TransitionTo(common::MissionState::kReturnHome,
                         "起飞中止（已离地），进入返航");
            PublishTakeoffAbortAck(reason);
            return;
        }
        if (landed_known && landed && armed) {
            SendTakeoffDisarmOnce();
        }
        assignment_tracker_.AbortOnGround();
        RememberTakeoffAbort(reason);
        TransitionTo(common::MissionState::kReady, "起飞中止（地面），回到 READY");
        PublishTakeoffAbortAck(reason);
    }

    /// 地面中止的 DISARM：只尝试一次，不重试（本次飞行解锁流程已失败）。
    void SendTakeoffDisarmOnce() {
        if (takeoff_disarm_sent_ || command_sender_ == nullptr) {
            return;
        }
        takeoff_disarm_sent_ = true;
        const bool queued = command_sender_->SendCommand(
            kMavCmdComponentArmDisarm, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
        if (queued) {
            SPDLOG_INFO("地面中止 DISARM 命令已入队（仅一次）");
        } else {
            SPDLOG_WARN("地面中止 DISARM 入队失败（不重试，交 PX4/人工处置）");
        }
    }

    /// 记录本次分配最终结果为 aborted（重复 65014 重发最新结果）。
    void RememberTakeoffAbort(common::MissionAssignmentReason reason) {
        if (!accepted_assignment_.has_value()) {
            return;
        }
        assignment_tracker_.RememberResult(
            accepted_assignment_->assignment_id,
            common::MissionAssignmentResult::kCancelledOrAborted, reason);
    }

    /// 发布 65015 aborted（已接受分配的 id/target + 具体 reason）。
    void PublishTakeoffAbortAck(common::MissionAssignmentReason reason) {
        if (!accepted_assignment_.has_value()) {
            return;
        }
        PublishAssignmentAck(*accepted_assignment_,
                             common::MissionAssignmentResult::kCancelledOrAborted,
                             reason);
    }

    /// 发布 S3/S4 起飞意图（type=kTakeoff，**不用 kPositionGuide 代替**）：
    /// 目标 x/y=tracker 起飞原点，z 为 S3 原点值或 S4 逐拍爬升值；reason_code
    /// 带 bit31 控制权请求位；航向尽量保持当前姿态 yaw。每决策拍发布，
    /// 不走内容节流——控制器需要持续的原点保持/爬升流。
    void PublishTakeoffIntent(float z_target, uint64_t now_ms) {
        common::ControlIntent intent;
        intent.header.sequence = ++intent_sequence_;
        intent.header.source_time_ms = now_ms;
        intent.header.receive_time_ms = now_ms;
        intent.header.valid_for_ms = intent_valid_for_ms_;
        intent.header.source_id = 1;
        intent.header.health = 1;
        intent.header.frame_id = kFrameLocalNed;
        intent.type = common::ControlIntentType::kTakeoff;
        intent.control_source = 0;  // 位置链（自动起飞）
        intent.target_x = assignment_tracker_.OriginX();
        intent.target_y = assignment_tracker_.OriginY();
        intent.target_z = z_target;
        if (latest_flight_.has_value() && latest_flight_->attitude_valid) {
            intent.yaw_deg = static_cast<float>(
                static_cast<double>(latest_flight_->yaw_rad) * kRadToDeg);
        }
        // bit31 显式请求控制权（FlightController kBootstrap 必要条件）+
        // 低位 7（地面站任务下发）。
        intent.reason_code =
            common::kControlIntentReasonRequestControl | kReasonGroundStationTask;
        intent.valid_until_ms = now_ms + intent_valid_for_ms_;
        (void)intent_output_.Publish(
            std::make_shared<const common::ControlIntent>(intent));
        last_intent_publish_ms_ = now_ms;
    }

    /// S3 入口初始化（TransitionTo 调用）：记录时序基准与 ACK 基线。
    void BeginArmingSequence(uint64_t now_ms) {
        takeoff_stage_ = TakeoffStage::kOriginHold;
        takeoff_stage_ms_ = now_ms;
        takeoff_last_tick_ms_ = now_ms;
        takeoff_stable_since_ms_ = 0;
        takeoff_z_cmd_ = assignment_tracker_.OriginZ();
        takeoff_disarm_sent_ = false;
        takeoff_ack_baseline_matched_ =
            command_sender_ != nullptr ? command_sender_->AckStatus().matched_count
                                       : 0;
        SPDLOG_INFO("S3 自动起飞开始: origin=({:.2f},{:.2f},{:.2f}) "
                    "bootstrap_lead={}ms mode_timeout={}ms arm_timeout={}ms",
                    static_cast<double>(assignment_tracker_.OriginX()),
                    static_cast<double>(assignment_tracker_.OriginY()),
                    static_cast<double>(assignment_tracker_.OriginZ()),
                    takeoff_.bootstrap_lead.count(),
                    takeoff_.mode_timeout.count(), takeoff_.arm_timeout.count());
    }

    /// S4 入口初始化（TransitionTo 调用）：爬升目标从原点开始逐拍推进。
    void BeginClimbSequence(uint64_t now_ms) {
        takeoff_stage_ = TakeoffStage::kClimb;
        takeoff_stage_ms_ = now_ms;
        takeoff_last_tick_ms_ = now_ms;
        takeoff_stable_since_ms_ = 0;
        takeoff_z_cmd_ = assignment_tracker_.OriginZ();
        SPDLOG_INFO("S4 自动爬升开始: 高度 {:.2f}m 爬升率 {:.2f}m/s "
                    "到达判据 z±{:.2f}m xy±{:.2f}m vz≤{:.2f}m/s 稳定 {}ms",
                    takeoff_.height_m, takeoff_.climb_rate_mps,
                    takeoff_.z_tolerance_m, takeoff_.xy_tolerance_m,
                    takeoff_.max_vz_mps, takeoff_.stable_time.count());
    }

    /// 计算 65013/65015 的 availability。影子模式（未注入命令接口）固定
    /// unavailable：不可被调度；有控制路径时用 tracker 阶段映射。
    /// 调用方必须已持有 state_mutex_。
    common::MissionAvailability ComputeAvailabilityLocked() const {
        if (command_sender_ == nullptr || mission_faulted_ ||
            (net_actuator_ != nullptr && net_actuator_->Faulted()) ||
            takeoff_unavailable_latched_) {
            // takeoff_unavailable_latched_：起飞中止后 PX4 状态不确定
            // （断连/armed/离地状态不明），per-boot 锁存直到重启，避免
            // 错误回到 ready 被再次调度。
            return common::MissionAvailability::kUnavailable;
        }
        if (assignment_tracker_.Phase() != AssignmentPhase::kIdle) {
            return assignment_tracker_.Availability();
        }
        // 已消耗空中任务机会但阶段已清空：本次上电不可再调度。
        if (assignment_tracker_.ConsumedAirborneMission()) {
            return common::MissionAvailability::kUnavailable;
        }
        if (!latest_flight_.has_value() || !latest_flight_->connected) {
            return common::MissionAvailability::kUnavailable;
        }
        // 电池门禁（Task 8）：地面参数缺失/遥测失效/不高于 BAT_LOW_THR 时
        // 不可调度。未执行空中任务，电量恢复且参数有效后自然回到 ready
        // （不锁存为 terminal）。
        if (!BatteryGuardReady(*latest_flight_)) {
            return common::MissionAvailability::kUnavailable;
        }
        return common::MissionAvailability::kReady;
    }

    common::MissionAvailability ComputeAvailability() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return ComputeAvailabilityLocked();
    }

    /// 发布 65015 任务分配回执（assignment_id/target_id/result/reason/
    /// 当前状态/availability）。低速率控制面消息，每条 65014 一条。
    void PublishAssignmentAck(const common::MissionAssignment& assignment,
                              common::MissionAssignmentResult result,
                              common::MissionAssignmentReason reason) {
        const uint64_t now_ms = MonotonicMs();
        common::MissionAssignmentAck ack;
        ack.header.sequence = ++assignment_ack_sequence_;
        ack.header.source_time_ms = now_ms;
        ack.header.receive_time_ms = now_ms;
        ack.header.valid_for_ms = kStatusValidForMs;
        ack.header.source_id = 1;
        ack.header.health = 1;
        ack.assignment_id = assignment.assignment_id;
        ack.target_id = assignment.target_id;
        ack.result = result;
        ack.reason = reason;
        ack.mission_state = CurrentState();
        ack.availability = ComputeAvailability();
        (void)assignment_ack_output_.Publish(
            std::make_shared<const common::MissionAssignmentAck>(ack));
    }

    /// S9/S10 入口判据（interception 配置节，2026-10-19）。
    /// 条件：视觉 LOCKED 连续帧 + 测距窗口 + PX4 Offboard/armed/姿态/位置有效
    /// + 命令接口就绪 + 机构未触发且 Ready()（无在途触发/复位时序）。
    bool InterceptEntrySatisfied(uint64_t now_ms) const {
        if (!interception_.enabled) {
            return false;
        }
        if (!latest_flight_) {
            return false;
        }
        const auto& flight = *latest_flight_;
        if (!flight.connected || !flight.armed || !flight.attitude_valid ||
            !flight.local_position_valid) {
            return false;
        }
        if (interception_.min_battery_percent > 0.0) {
            if (!flight.battery_remaining_valid ||
                static_cast<double>(flight.battery_remaining_pct) <
                    interception_.min_battery_percent) {
                return false;
            }
        }
        if (flight.flight_mode != kPx4FlightMainModeOffboard ||
            flight.flight_sub_mode != kPx4SubModeOffboard) {
            return false;
        }
        if (command_sender_ == nullptr || net_actuator_ == nullptr ||
            mission_faulted_ || net_actuator_->Faulted() ||
            net_actuator_->Fired() || !net_actuator_->Ready()) {
            return false;  // 命令接口未注入 / 已触发 / 时序未回到 Ready / 故障
        }
        if (VisualStatusStale(now_ms) || !latest_visual_status_ ||
            latest_visual_status_->state != common::VisualTargetState::kLocked) {
            return false;
        }
        if (consecutive_lock_frames_ < interception_.lock_frames) {
            return false;
        }
        if (!latest_distance_ || !latest_distance_->valid ||
            !DistanceFresh(now_ms)) {
            return false;
        }
        const double range_m =
            static_cast<double>(latest_distance_->forward_distance_m);
        return std::isfinite(range_m) && range_m >= interception_.min_range_m &&
               range_m <= interception_.max_range_m;
    }

    /// S7→S9（入口条件满足）、S9 条件失效→S7、S9 自动授权→S10 /
    /// 非自动授权超时→S12。S10 的触发由 MaybeTriggerNetActuator 处理，
    /// 触发入队（`Fired()`）后由 EvaluateInterceptionCompletion 进入 S11。
    void EvaluateInterceptSequence(uint64_t now_ms) {
        if (!interception_.enabled) {
            return;
        }
        const auto state = CurrentState();
        if (state == common::MissionState::kVisualTracking) {
            if (!InterceptEntrySatisfied(now_ms)) {
                return;
            }
            const double range_m = latest_distance_
                ? static_cast<double>(latest_distance_->forward_distance_m)
                : std::numeric_limits<double>::quiet_NaN();
            SPDLOG_WARN("拦截入口条件满足: LOCKED 帧={} 测距={:.2f}m（窗口 "
                        "{:.1f}~{:.1f}m）→ S9",
                        consecutive_lock_frames_, range_m,
                        interception_.min_range_m, interception_.max_range_m);
            TransitionTo(common::MissionState::kInterceptReady,
                         "拦截入口条件满足");
            return;
        }
        if (state == common::MissionState::kInterceptReady) {
            if (!InterceptEntrySatisfied(now_ms)) {
                SPDLOG_WARN("拦截条件失效，退出 S9 返回视觉跟踪");
                TransitionTo(common::MissionState::kVisualTracking,
                             "拦截条件失效");
                return;
            }
            if (interception_.auto_authorize) {
                SPDLOG_WARN("拦截自动授权通过 → S10");
                TransitionTo(common::MissionState::kInterceptExecuting,
                             "自动授权通过");
                return;
            }
            const uint64_t wait_ms =
                now_ms >= state_entered_ms_ ? now_ms - state_entered_ms_ : 0;
            if (wait_ms >= static_cast<uint64_t>(
                               interception_.authorization_timeout.count())) {
                SPDLOG_WARN("拦截授权超时，进入返航");
                TransitionTo(common::MissionState::kReturnHome,
                             "拦截授权超时");
            }
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
        PositionIntentTarget position_target;
        position_target.x = frozen.pos_x_m;
        position_target.y = frozen.pos_y_m;
        position_target.z = frozen.pos_z_m;
        position_target.frame_id = frozen.header.frame_id;
        // P3 决策：S6 保持冻结语义，不带速度前馈（即使冻结快照有速度位）。
        // 交接后由 S7 速度伺服接管，避免固定位置目标 + 持续前馈造成越过冻结点。
        PublishPositionIntent(position_target, now_ms);
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

    /// 控制任务下是否按当前分配的 target_id 过滤目标（设计 §7.5）。
    /// 影子装配（未注入命令接口）保持既有行为，不过滤。
    bool AssignmentTargetFilterActive() const {
        return command_sender_ != nullptr &&
               assignment_tracker_.HasActiveAssignment();
    }

    /// 目标编号是否允许进入本机任务链。无过滤条件时全部放行；控制任务下
    /// 只允许 `target_id == assignment.target_id`（0 永远不匹配）。
    bool TargetMatchesAssignment(uint32_t target_id) const {
        if (!AssignmentTargetFilterActive()) {
            return true;
        }
        return assignment_tracker_.AllowsTarget(target_id);
    }

    /// 估计目标是否新鲜有效（valid 标志 + 剩余有效期，与 TargetEstimator 输出语义一致）。
    /// 控制任务下非本分配 target_id 的估计一律视为不可用（不进入 S5 引导、
    /// 不参与交接、不满足目标看门狗）。
    bool HasFreshEstimate(uint64_t now_ms) const {
        if (!latest_estimated_) {
            return false;
        }
        const auto& target = *latest_estimated_;
        if (!target.valid) {
            return false;
        }
        if (!TargetMatchesAssignment(target.target_id)) {
            return false;
        }
        if (target.header.receive_time_ms == 0 || target.header.valid_for_ms == 0) {
            return false;
        }
        return now_ms <= target.header.receive_time_ms + target.header.valid_for_ms;
    }

    /// S5 目标看门狗判据：只看匹配当前分配的估计目标。65010 只是上游位置
    /// 输入，S5 的位置引导实际消耗 TargetEstimator 输出；若地面站持续发
    /// 65010 但估计器不出目标，不能把“已收到 65010”误当成可执行目标，
    /// 否则 S5 会无引导地悬停且 target_wait_timeout 永不触发。
    /// 控制路径的 target_id 过滤在 HasFreshEstimate 内生效。
    bool HasFreshAssignedTarget(uint64_t now_ms) const {
        return HasFreshEstimate(now_ms);
    }

    /// 位置型意图的组装输入（S5 估计引导 / S6 冻结续发共用）。
    /// 前馈字段只对 S5 生效；S6 构造时保持 mask=0、前馈值=0（P3 冻结决策）。
    struct PositionIntentTarget {
        float x = 0.f;
        float y = 0.f;
        float z = 0.f;
        uint8_t frame_id = 0;
        float feedforward_vx_mps = 0.f;
        float feedforward_vy_mps = 0.f;
        float feedforward_vz_mps = 0.f;
        uint8_t feedforward_valid_mask = 0;
    };

    /// 发布 S5 估计引导位置意图。节流纪律：内容无变化按 10Hz 上限重发
    /// （意图是周期语义，FlightController 50ms 周期消费）；内容变化立即发。
    void PublishEstimateGuidedIntent(const common::TargetState& target,
                                     uint64_t now_ms) {
        // 前馈逐轴归一化：只保留“有效位 + 值有限”的轴；其余轴清零，
        // 避免无效 KF 值污染内容比较或下发。S6 不走本函数（冻结段无前馈）。
        PositionIntentTarget position_target;
        position_target.x = target.pos_x_m;
        position_target.y = target.pos_y_m;
        position_target.z = target.pos_z_m;
        position_target.frame_id = target.header.frame_id;
        const uint8_t target_ff_mask =
            target.velocity_valid_mask & common::kVelocityValidAll;
        if ((target_ff_mask & common::kVelocityValidNorth) != 0 &&
            std::isfinite(target.vel_x_mps)) {
            position_target.feedforward_vx_mps = target.vel_x_mps;
            position_target.feedforward_valid_mask |= common::kVelocityValidNorth;
        }
        if ((target_ff_mask & common::kVelocityValidEast) != 0 &&
            std::isfinite(target.vel_y_mps)) {
            position_target.feedforward_vy_mps = target.vel_y_mps;
            position_target.feedforward_valid_mask |= common::kVelocityValidEast;
        }
        if ((target_ff_mask & common::kVelocityValidDown) != 0 &&
            std::isfinite(target.vel_z_mps)) {
            position_target.feedforward_vz_mps = target.vel_z_mps;
            position_target.feedforward_valid_mask |= common::kVelocityValidDown;
        }

        const bool content_changed = !last_intent_target_.has_value() ||
            !FloatContentEqual(last_intent_target_->pos_x_m, target.pos_x_m) ||
            !FloatContentEqual(last_intent_target_->pos_y_m, target.pos_y_m) ||
            !FloatContentEqual(last_intent_target_->pos_z_m, target.pos_z_m) ||
            last_intent_target_->valid != target.valid ||
            last_intent_target_->velocity_valid_mask != target_ff_mask ||
            ((target_ff_mask & common::kVelocityValidNorth) != 0 &&
             !FloatContentEqual(last_intent_target_->vel_x_mps, target.vel_x_mps)) ||
            ((target_ff_mask & common::kVelocityValidEast) != 0 &&
             !FloatContentEqual(last_intent_target_->vel_y_mps, target.vel_y_mps)) ||
            ((target_ff_mask & common::kVelocityValidDown) != 0 &&
             !FloatContentEqual(last_intent_target_->vel_z_mps, target.vel_z_mps));
        if (position_intent_source_ == PositionIntentSource::kEstimate &&
            !content_changed &&
            now_ms - last_intent_publish_ms_ < kIntentRepublishIntervalMs) {
            return;
        }
        if (position_intent_source_ != PositionIntentSource::kEstimate) {
            SPDLOG_INFO(
                "GPS_APPROACH 位置引导意图启动: target_id={} pos=({:.2f},{:.2f},{:.2f}) "
                "ff_mask=0x{:02x} ff=({:.2f},{:.2f},{:.2f})",
                target.target_id, static_cast<double>(target.pos_x_m),
                static_cast<double>(target.pos_y_m),
                static_cast<double>(target.pos_z_m),
                position_target.feedforward_valid_mask,
                static_cast<double>(position_target.feedforward_vx_mps),
                static_cast<double>(position_target.feedforward_vy_mps),
                static_cast<double>(position_target.feedforward_vz_mps));
            position_intent_source_ = PositionIntentSource::kEstimate;
        }
        PublishPositionIntent(position_target, now_ms);
        last_intent_target_ = target;
    }

    /// 组装并发布位置引导意图（S5/S6 引导与 S12 返航共用）。
    /// 高度无效时 pos_z 为 NaN（合法语义）：原样透传，由 FlightController
    /// 有限性门禁拒绝，状态机侧不得改成 0。
    /// reason_low：低位原因码（S5/S6=7 地面站任务，S12=8 返航）。
    void PublishPositionIntent(const PositionIntentTarget& target,
                               uint64_t now_ms,
                               uint32_t reason_low = kReasonGroundStationTask) {
        common::ControlIntent intent;
        intent.header.sequence = ++intent_sequence_;
        intent.header.source_time_ms = now_ms;
        intent.header.receive_time_ms = now_ms;
        intent.header.valid_for_ms = intent_valid_for_ms_;
        intent.header.source_id = 1;
        intent.header.health = 1;
        intent.header.frame_id = target.frame_id;
        intent.type = common::ControlIntentType::kPositionGuide;
        intent.control_source = 0;  // 位置链
        intent.target_x = target.x;
        intent.target_y = target.y;
        intent.target_z = target.z;
        intent.feedforward_vx_mps = target.feedforward_vx_mps;
        intent.feedforward_vy_mps = target.feedforward_vy_mps;
        intent.feedforward_vz_mps = target.feedforward_vz_mps;
        intent.feedforward_valid_mask = target.feedforward_valid_mask;
        // bit31 显式请求控制权（FlightController 授权链必要条件）+
        // 低位原因码（S5/S6=7 地面站任务；S12=8 返航）。
        intent.reason_code =
            common::kControlIntentReasonRequestControl | reason_low;
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
        const auto state = CurrentState();
        // S7 正常跟踪；S9/S10 期间继续输出伺服意图（保持瞄准/稳定，避免
        // 拦截准备段因意图停发触发控制器 COAST 刹车）。S11/S12 起返航接管。
        if (state != common::MissionState::kVisualTracking &&
            state != common::MissionState::kInterceptReady &&
            state != common::MissionState::kInterceptExecuting) {
            return;
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

    /// 组装执行器每拍反馈：PX4 连接证据（最新飞行快照）与命令 ACK 快照。
    /// 执行器内部时序只认本次 Update 的反馈，状态机不复制计时逻辑。
    ActuatorFeedback BuildActuatorFeedback(uint64_t now_ms) const {
        ActuatorFeedback feedback;
        feedback.now_ms = now_ms;
        feedback.px4_connected =
            latest_flight_.has_value() && latest_flight_->connected;
        if (command_sender_ != nullptr) {
            feedback.ack = command_sender_->AckStatus();
        }
        return feedback;
    }

    /// 每拍推进捕网执行器并处理其故障。返回 true 表示已进入/处于故障态，
    /// 本拍应跳过任务控制逻辑（S16 只维护告警与状态发布）。
    bool UpdateNetActuator(uint64_t now_ms) {
        if (mission_faulted_) {
            return true;
        }
        if (net_actuator_ == nullptr) {
            return false;  // 物理缺席（影子/未开控制）：不产生任何命令
        }
        net_actuator_->Update(BuildActuatorFeedback(now_ms));
        if (net_actuator_->Faulted()) {
            HandleMissionFault(net_actuator_->FaultReason());
            return true;
        }
        return false;
    }

    /// S10 内触发一次捕网。语义：
    /// - 无飞行快照 = 没有连接证据（启动早期/测试未注入）→ 不触发；
    /// - 快照在线才触发；快照存在但 connected=false → 故障；
    /// - 入队失败由执行器保持 Ready、下拍重试；超过 return_home.fire_timeout
    ///   仍未入队 → 故障（不再“按失败结果返航”）；
    /// - 触发成功后由执行器维护 ACK 保持/复位时序，状态机不重复触发。
    void MaybeTriggerNetActuator(uint64_t now_ms) {
        if (net_actuator_ == nullptr || mission_faulted_ ||
            CurrentState() != common::MissionState::kInterceptExecuting) {
            return;
        }
        if (net_actuator_->Fired()) {
            return;
        }
        const uint64_t held_ms =
            now_ms >= state_entered_ms_ ? now_ms - state_entered_ms_ : 0;
        if (held_ms >= static_cast<uint64_t>(return_home_.fire_timeout.count())) {
            HandleMissionFault("触发命令未入队（发射超时）");
            return;
        }
        if (!latest_flight_.has_value()) {
            return;  // 启动早期/测试未注入：没有连接证据，不触发
        }
        if (!latest_flight_->connected) {
            HandleMissionFault("S10 期间 PX4 连接断开");
            return;
        }
        if (net_actuator_->Ready() && net_actuator_->RemainingShots() > 0) {
            if (net_actuator_->Trigger()) {
                interception_fired_ = true;
            }
        }
    }

    /// 捕网执行器故障 → S16 FAILSAFE：置 bit4、取消待发命令（恰好一次）、
    /// 停发一切控制意图，交 PX4/遥控处置；不重试、不补发复位。
    void HandleMissionFault(const char* reason) {
        if (mission_faulted_) {
            return;
        }
        mission_faulted_ = true;
        // 起飞段（S3/S4）发生任务故障：先按实际是否离地收口 tracker 并回
        // 65015 aborted(mission_fault)，再迁移 S16（故障为终态，控制意图停发）。
        AbortTakeoffForMissionFault(reason);
        SetWarnings(kWarningActuatorFault);
        if (command_sender_ != nullptr) {
            command_sender_->CancelPendingCommands();
        }
        SPDLOG_ERROR("捕网执行器故障，停止任务控制并交 PX4/遥控: {}", reason);
        TransitionTo(common::MissionState::kFailsafe, reason);
    }

    // ---- 拦截后返航/降落（S11~S14，2026-10-19 V1 单发口径） ----

    /// Home 在 EKF 锚点坐标系下的局部 NED（返航目标）。
    /// 使用 Home 经纬度 + 锚点换算，避免依赖 HOME_POSITION.x/y/z（MAVLink
    /// 报文未携带 PX4 的 valid_lpos 标志）；home/锚点/当前位置任一无效返回 nullopt。
    std::optional<perception::LocalNedCoordinate> HomeLocalNed() const {
        if (!latest_flight_.has_value()) {
            return std::nullopt;
        }
        const auto& flight = *latest_flight_;
        if (!flight.home_valid || !flight.ned_ref_valid ||
            !flight.local_position_valid) {
            return std::nullopt;
        }
        const perception::GeodeticCoordinate anchor{
            static_cast<double>(flight.ned_ref_lat_1e7) * 1e-7,
            static_cast<double>(flight.ned_ref_lon_1e7) * 1e-7,
            static_cast<double>(flight.ned_ref_alt_mm) * 1e-3};
        const perception::GeodeticCoordinate home{
            static_cast<double>(flight.home_lat_1e7) * 1e-7,
            static_cast<double>(flight.home_lon_1e7) * 1e-7,
            static_cast<double>(flight.home_altitude_mm) * 1e-3};
        return perception::GeodeticToLocalNed(anchor, home);
    }

    /// 发布返航位置意图（低位原因码 8；不带目标速度前馈）。
    void PublishReturnHomeIntent(float x, float y, float z, uint64_t now_ms) {
        PositionIntentTarget target;
        target.x = x;
        target.y = y;
        target.z = z;
        target.frame_id = kFrameLocalNed;
        PublishPositionIntent(target, now_ms, kReasonReturnHome);
    }

    /// 单发拦截完成：S10 触发命令已入队（`Fired()`，不等 ACK）→ S11 → S12。
    /// V1 不判断命中；摄像头状态不参与返航判断。发射超时/断连已由
    /// MaybeTriggerNetActuator/执行器判故障进入 S16，不再走“失败返航”。
    void EvaluateInterceptionCompletion() {
        if (!return_home_.enabled || mission_faulted_) {
            return;
        }
        const auto state = CurrentState();
        if (state == common::MissionState::kInterceptExecuting) {
            if (net_actuator_ != nullptr && net_actuator_->Fired()) {
                interception_fired_ = true;
                SPDLOG_WARN("单发拦截命令已入队，进入 POST_INTERCEPT（V1 不判命中）");
                TransitionTo(common::MissionState::kPostIntercept,
                             "单发拦截命令已入队（不判命中）");
            }
            return;
        }
        if (state == common::MissionState::kPostIntercept) {
            TransitionTo(common::MissionState::kReturnHome,
                         interception_fired_ ? "单发拦截已发射，进入返航"
                                             : "单发拦截未发射，进入返航");
        }
    }

    /// S12 返航：低于 5m 先原地爬升，再位置设定值飞 Home；到达后进 S13。
    /// PX4 离开 Offboard（failsafe/人工）→ 停止争控，转 S13 仅监控降落。
    void EvaluateReturnHome(uint64_t now_ms) {
        if (!return_home_.enabled ||
            CurrentState() != common::MissionState::kReturnHome) {
            return;
        }
        if (!latest_flight_.has_value()) {
            return;
        }
        const auto& flight = *latest_flight_;
        if (flight.flight_mode != kPx4FlightMainModeOffboard ||
            flight.flight_sub_mode != kPx4FlightSubModeOffboard) {
            return_px4_takeover_ = true;
            SPDLOG_WARN("返航中 PX4 已离开 Offboard（mode={}/{}），停止返航设定值",
                        flight.flight_mode, flight.flight_sub_mode);
            TransitionTo(common::MissionState::kLanding, "PX4 接管返航/降落");
            return;
        }
        if (!flight.local_position_valid) {
            if (!return_home_wait_logged_) {
                SPDLOG_WARN("返航等待 PX4 局部位置有效");
                return_home_wait_logged_ = true;
            }
            return;
        }
        // 同一拍刚进入 S12 时 state_entered_ms_ 可能晚于本拍 now_ms，unsigned
        // 相减会回绕；钳到 0，避免"刚进返航就超时"。
        const uint64_t return_elapsed_ms =
            now_ms >= state_entered_ms_ ? now_ms - state_entered_ms_ : 0;
        if (return_elapsed_ms >=
            static_cast<uint64_t>(return_home_.timeout.count())) {
            if (!return_timeout_warned_) {
                SPDLOG_WARN("返航超时，原地悬停等待地面站/人工处置");
                return_timeout_warned_ = true;
            }
            PublishReturnHomeIntent(flight.local_x_m, flight.local_y_m,
                                    flight.local_z_m, now_ms);
            return;
        }
        const auto home = HomeLocalNed();
        if (!home.has_value()) {
            if (!return_home_wait_logged_) {
                SPDLOG_WARN("返航等待 Home/EKF 锚点有效（无局部返航基准）");
                return_home_wait_logged_ = true;
            }
            // 无 Home 基准时原地悬停，不盲目返航。
            PublishReturnHomeIntent(flight.local_x_m, flight.local_y_m,
                                    flight.local_z_m, now_ms);
            return;
        }

        // 返航高度：低于 5m 先原地爬升；≥5m 保持当前高度。
        const double relative_alt_m =
            home->down_m - static_cast<double>(flight.local_z_m);
        double target_z = static_cast<double>(flight.local_z_m);
        if (relative_alt_m < return_home_.altitude_m) {
            target_z = home->down_m - return_home_.altitude_m;
        } else {
            return_climb_phase_ = false;
        }
        if (return_climb_phase_) {
            const bool climbed =
                std::fabs(static_cast<double>(flight.local_z_m) - target_z) <=
                    return_home_.arrival_vertical_m &&
                std::fabs(static_cast<double>(flight.vz_mps)) <=
                    return_home_.arrival_speed_mps;
            if (climbed) {
                return_climb_phase_ = false;
                SPDLOG_INFO("返航爬升完成（相对 Home {:.1f}m），开始水平返航",
                            return_home_.altitude_m);
            }
        }
        const double target_x = return_climb_phase_
                                    ? static_cast<double>(flight.local_x_m)
                                    : home->north_m;
        const double target_y = return_climb_phase_
                                    ? static_cast<double>(flight.local_y_m)
                                    : home->east_m;
        PublishReturnHomeIntent(static_cast<float>(target_x),
                                static_cast<float>(target_y),
                                static_cast<float>(target_z), now_ms);

        if (return_climb_phase_) {
            return;
        }
        const double horizontal_m = std::hypot(
            home->north_m - static_cast<double>(flight.local_x_m),
            home->east_m - static_cast<double>(flight.local_y_m));
        const double vertical_m =
            std::fabs(target_z - static_cast<double>(flight.local_z_m));
        const double horizontal_speed_mps =
            std::hypot(static_cast<double>(flight.vx_mps),
                       static_cast<double>(flight.vy_mps));
        const bool inside = horizontal_m <= return_home_.arrival_radius_m &&
                            vertical_m <= return_home_.arrival_vertical_m &&
                            horizontal_speed_mps <= return_home_.arrival_speed_mps;
        if (!inside) {
            arrival_stable_since_ms_ = 0;
            return;
        }
        if (arrival_stable_since_ms_ == 0) {
            arrival_stable_since_ms_ = now_ms;
        }
        const uint64_t arrival_held_ms =
            now_ms >= arrival_stable_since_ms_
                ? now_ms - arrival_stable_since_ms_
                : 0;
        if (arrival_held_ms >=
            static_cast<uint64_t>(return_home_.arrival_hold.count())) {
            TransitionTo(common::MissionState::kLanding,
                         "到达 Home 附近，进入降落");
        }
    }

    /// S13 发 AUTO.LAND（4/6），S14 前发 DISARM；PX4 接管时只监控不争控。
    void EvaluateLandingAndDisarm(uint64_t now_ms) {
        if (!return_home_.enabled) {
            return;
        }
        const auto state = CurrentState();
        if (state == common::MissionState::kLanding) {
            if (!latest_flight_.has_value()) {
                return;
            }
            const auto& flight = *latest_flight_;
            if (flight.landed_state_valid && flight.landed) {
                if (!flight.armed) {
                    TransitionTo(common::MissionState::kDisarmed,
                                 "PX4 已确认 landed + disarmed，任务结束");
                    return;
                }
                if (command_sender_ == nullptr) {
                    if (!landing_command_unavailable_logged_) {
                        SPDLOG_ERROR("S13 无法 DISARM：命令接口未注入");
                        landing_command_unavailable_logged_ = true;
                    }
                    return;
                }
                if (now_ms - disarm_command_ms_ >=
                    static_cast<uint64_t>(return_home_.command_retry.count())) {
                    const bool queued = command_sender_->SendCommand(
                        kMavCmdComponentArmDisarm, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
                        0.f);
                    disarm_command_ms_ = now_ms;
                    if (queued) {
                        if (!disarm_command_sent_) {
                            disarm_command_sent_ = true;
                            SPDLOG_INFO("S14 前 DISARM 命令已入队");
                        }
                    } else {
                        const uint64_t count = ++disarm_command_fail_count_;
                        if (ShouldLogThrottled(count)) {
                            SPDLOG_WARN("DISARM 入队失败（下拍重试）累计={}", count);
                        }
                    }
                }
                return;
            }

            if (landing_takeover_) {
                return;  // PX4 主导返航/降落，仅等待 landed/disarmed
            }
            if (command_sender_ == nullptr) {
                if (!landing_command_unavailable_logged_) {
                    SPDLOG_ERROR("S13 无法发 AUTO.LAND：命令接口未注入");
                    landing_command_unavailable_logged_ = true;
                }
                return;
            }
            const bool offboard =
                flight.flight_mode == kPx4FlightMainModeOffboard &&
                flight.flight_sub_mode == kPx4FlightSubModeOffboard;
            if (!offboard) {
                const bool auto_land =
                    flight.flight_mode == kPx4FlightMainModeAuto &&
                    flight.flight_sub_mode == kPx4FlightSubModeAutoLand;
                if (!auto_land) {
                    landing_takeover_ = true;
                    SPDLOG_WARN(
                        "S13 PX4 已离开 Offboard 且非 AUTO.LAND（mode={}/{}），"
                        "停止命令重试，等待降落",
                        flight.flight_mode, flight.flight_sub_mode);
                }
                return;
            }
            if (now_ms - landing_command_ms_ >=
                static_cast<uint64_t>(return_home_.command_retry.count())) {
                const bool queued = command_sender_->SendCommand(
                    kMavCmdDoSetMode, kMavModeFlagCustomModeEnabled,
                    kPx4MainModeAuto, kPx4SubModeAutoLand, 0.f, 0.f, 0.f, 0.f);
                landing_command_ms_ = now_ms;
                if (queued) {
                    if (!landing_command_sent_) {
                        landing_command_sent_ = true;
                        SPDLOG_INFO("S13 AUTO.LAND 命令已入队");
                    }
                } else {
                    const uint64_t count = ++landing_command_fail_count_;
                    if (ShouldLogThrottled(count)) {
                        SPDLOG_WARN("AUTO.LAND 入队失败（下拍重试）累计={}", count);
                    }
                }
            }
            return;
        }
        if (state == common::MissionState::kDisarmed && !mission_end_logged_) {
            mission_end_logged_ = true;
            SPDLOG_INFO("任务结束: S14 DISARMED，拦截结果={}",
                        interception_fired_ ? "已发射" : "未发射");
        }
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
        // 控制任务下非本分配 target_id 的 65010 直接忽略（GroundStationLink
        // 仍可照常做链路 ACK，状态机不进入任务链/不更新引导）。
        if (!TargetMatchesAssignment(target.target_id)) {
            return false;
        }
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
            // 拦截后返航/降落段入口状态初始化（S11~S14，2026-10-19）。
            if (next_state == common::MissionState::kReturnHome) {
                return_climb_phase_ = true;
                return_px4_takeover_ = false;
                return_timeout_warned_ = false;
                return_home_wait_logged_ = false;
                arrival_stable_since_ms_ = 0;
                SPDLOG_INFO("返航开始: Home 局部NED 由 Home+EKF锚点换算，"
                            "返航高度 {:.1f}m，到达半径 {:.1f}m",
                            return_home_.altitude_m, return_home_.arrival_radius_m);
            } else if (next_state == common::MissionState::kLanding) {
                landing_command_sent_ = false;
                landing_command_ms_ = 0;
                landing_takeover_ = return_px4_takeover_;
                landing_command_unavailable_logged_ = false;
                landing_command_fail_count_ = 0;
                disarm_command_sent_ = false;
                disarm_command_ms_ = 0;
                disarm_command_fail_count_ = 0;
                SPDLOG_INFO("降落阶段: {}",
                            landing_takeover_
                                ? "PX4 已接管，停止发设定值并等待 landed/disarmed"
                                : "发 AUTO.LAND（4/6）");
            } else if (next_state == common::MissionState::kDisarmed) {
                mission_end_logged_ = false;
                // 任务结束（Task 8）：分配进入 terminal，本次上电不再分配。
                assignment_tracker_.Terminal();
            } else if (next_state == common::MissionState::kFailsafe) {
                // 故障终态同样不再分配（S16）。
                assignment_tracker_.Terminal();
            }
            // 自动起飞时序入口（Task 5）：S3 记录 bootstrap/mode/arm 基准与
            // ACK 基线；S4 初始化逐拍爬升目标。中止迁移由统一挂点保证
            // 下一拍不再发布起飞意图。
            if (next_state == common::MissionState::kArming) {
                BeginArmingSequence(now_ms);
            } else if (next_state == common::MissionState::kTakeoff) {
                BeginClimbSequence(now_ms);
            } else if (next_state == common::MissionState::kGpsApproach) {
                // S5 目标等待计时从进入任务高度起算（仅控制任务）。
                BeginTargetWatchdog(now_ms);
            }
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
            // 65013 v2（2026-10-22）：分配编号/目标/功能类别与可用状态。
            // 无分配时 id 为 0；影子模式 availability 固定 unavailable。
            status.assignment_id = assignment_tracker_.AssignmentId();
            status.target_id = assignment_tracker_.TargetId();
            status.interceptor_type = assignment_tracker_.InterceptorType();
            status.availability = ComputeAvailabilityLocked();
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
    /// 任务分配输入（65014，可选绑定；未绑定时不订阅、不处理）。
    common::Topic<common::MissionAssignment>* assignment_topic_ = nullptr;
    /// PX4 命令窄接口（仅 enable_control=true 装配期注入）。当前消费点：
    /// S13 AUTO.LAND / S14 DISARM；未注入时零调用。
    communication::IPx4CommandSender* command_sender_ = nullptr;
    /// 捕网执行器抽象（随命令接口注入创建；actuator.type 工厂选择单发/未来多发）。
    /// 为空=影子/未开控制，触发/复位物理缺席。
    std::unique_ptr<INetActuator> net_actuator_;
    /// 估计器诊断接口（只读；交接创新门限的数据来源，Start 前注入）。
    const perception::ITargetEstimator* estimator_diagnostics_ = nullptr;

    common::Topic<common::GroundStationTarget>::Subscription ground_target_subscription_;
    common::Topic<common::FlightStateSnapshot>::Subscription flight_subscription_;
    common::Topic<common::HealthStatus>::Subscription health_subscription_;
    common::Topic<common::TargetState>::Subscription estimated_subscription_;
    common::Topic<common::VisualTargetStatus>::Subscription visual_status_subscription_;
    common::Topic<common::StereoTargetDistance>::Subscription distance_subscription_;
    common::Topic<common::MissionAssignment>::Subscription assignment_subscription_;

    std::optional<common::GroundStationTarget> latest_ground_target_;
    std::optional<common::FlightStateSnapshot> latest_flight_;
    std::optional<common::HealthStatus> latest_health_;
    std::optional<common::TargetState> latest_estimated_;
    std::optional<common::VisualTargetStatus> latest_visual_status_;
    std::optional<common::StereoTargetDistance> latest_distance_;
    /// 最新一条 65014（DrainInputs 写入，EvaluateAssignmentEntry 处理后清空）。
    std::optional<common::MissionAssignment> latest_assignment_;
    /// 已接受的分配副本（Task 5）：S3/S4 中止时 latest_assignment_ 已清空，
    /// 65015 aborted 必须用这份 id/target。下一次接受时覆盖。
    std::optional<common::MissionAssignment> accepted_assignment_;

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
    uint64_t assignment_receive_count_ = 0;
    uint64_t assignment_reject_count_ = 0;
    uint64_t assignment_duplicate_count_ = 0;
    uint64_t assignment_ack_sequence_ = 0;

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
    /// 拦截入口/授权与返航配置（构造注入；JSON interception/return_home 节）。
    const config::InterceptionConfig interception_;
    const config::ReturnHomeConfig return_home_;
    /// 自动起飞配置（takeoff 节；Task 4 只做构造期校验与状态一致性，
    /// S3 时序/S4 爬升在 Task 5 使用）。
    const config::TakeoffConfig takeoff_;
    /// 本机拦截功能类别（ground_station.aircraft_component_id 同源）。
    const uint8_t expected_interceptor_type_;
    /// 任务分配生命周期助手（一次上电一次任务、重复 id 回执、原点记录）。
    /// 只由决策线程访问；状态发布读取时受 state_mutex_ 保护。
    MissionAssignmentTracker assignment_tracker_;
    /// 自动起飞（S3/S4）时序阶段。只由决策线程推进；仅在 S3/S4 状态内有效。
    enum class TakeoffStage : uint8_t {
        kNone,          ///< 未在起飞时序中
        kOriginHold,    ///< S3：发布原点保持意图，等待 bootstrap_lead 后发 176
        kWaitOffboard,  ///< S3：已发 176，等待实际 6/0
        kWaitArmed,     ///< S3：已发 400，等待实际 armed
        kClimb,         ///< S4：逐拍爬升到任务高度
    };
    TakeoffStage takeoff_stage_ = TakeoffStage::kNone;
    uint64_t takeoff_stage_ms_ = 0;        ///< 当前阶段起点（mode/arm 超时基准）
    uint64_t takeoff_last_tick_ms_ = 0;    ///< S4 爬升 dt 基准
    uint64_t takeoff_stable_since_ms_ = 0; ///< S4 到达判据连续成立起点
    float takeoff_z_cmd_ = 0.f;            ///< S4 逐拍推进的 z 目标（局部 NED）
    bool takeoff_disarm_sent_ = false;     ///< 地面中止 DISARM 已尝试（仅一次）
    uint64_t takeoff_ack_baseline_matched_ = 0;  ///< S3 入口 ACK 匹配计数基线
    /// 起飞中止后 PX4 状态不确定（断连/离地状态不明）：per-boot 锁存，
    /// availability=unavailable 且新分配回 not_ready，直到 Start() 复位。
    bool takeoff_unavailable_latched_ = false;
    /// 飞行中电池中止锁存（Task 8）：一次任务只中止/回 65015 一次；
    /// 接受新分配或 Start() 时复位。
    bool battery_abort_latched_ = false;
    /// 低电防抖起点（nullopt=当前不低电/不在守卫生效阶段）。
    std::optional<uint64_t> battery_low_since_ms_;
    /// 目标看门狗（Task 10）：一次任务只中止一次；Accept/Start 复位。
    bool mission_abort_latched_ = false;
    uint64_t target_wait_started_ms_ = 0;  ///< S5 入口时刻（等待目标基准）
    bool assigned_target_acquired_ = false;  ///< 本次 S5 是否已获得匹配目标
    std::optional<uint64_t> target_lost_since_ms_;  ///< 目标持续失效起点
    /// 最近一次执行的取消动作时刻（500ms 内不重复下发命令）。
    std::optional<uint64_t> last_cancel_action_ms_;
    /// 测试钩子开关（构造注入，默认 false）：门控 ForceStateForTest。
    const bool test_hooks_enabled_;
    /// 任务故障（S16 FAILSAFE）：捕网执行器 ACK 超时/非 ACCEPTED/PX4 断连/
    /// 入队超时。置位后本拍只维护告警与状态发布，停发一切控制意图；
    /// 故障为终态（需人工/维护处置），Start() 复位。
    bool mission_faulted_ = false;
    // ---- 拦截后返航（S11~S14，2026-10-19）----
    bool interception_fired_ = false;       ///< 单发触发命令已入队（S10→S11 依据）
    bool return_climb_phase_ = true;        ///< S12：先爬升到 5m（若不足）
    bool return_px4_takeover_ = false;      ///< S12 中 PX4 离开 Offboard（停止争控）
    bool return_timeout_warned_ = false;    ///< 返航超时告警已打印
    bool return_home_wait_logged_ = false;  ///< Home/锚点无效告警已打印
    uint64_t arrival_stable_since_ms_ = 0;  ///< 到达判据连续成立起点
    bool landing_command_sent_ = false;     ///< AUTO.LAND 曾成功入队
    uint64_t landing_command_ms_ = 0;       ///< 最近一次 AUTO.LAND 入队尝试时刻
    bool landing_takeover_ = false;         ///< S13 由 PX4 接管（不再发 AUTO.LAND）
    bool landing_command_unavailable_logged_ = false;  ///< 命令接口未注入只报一次
    uint64_t landing_command_fail_count_ = 0;          ///< AUTO.LAND 入队失败计数
    bool disarm_command_sent_ = false;      ///< DISARM 曾成功入队
    uint64_t disarm_command_ms_ = 0;        ///< 最近一次 DISARM 入队尝试时刻
    uint64_t disarm_command_fail_count_ = 0;///< DISARM 入队失败计数
    bool mission_end_logged_ = false;       ///< S14 结束日志只打一次
    uint64_t intent_sequence_ = 0;
    uint64_t last_intent_publish_ms_ = 0;
    uint64_t intent_stale_count_ = 0;
    std::optional<common::TargetState> last_intent_target_;  ///< 上次发布意图对应的目标内容

    common::Topic<common::ControlIntent> intent_output_;
    common::Topic<common::MissionStatus> status_output_;
    common::Topic<common::MissionAssignmentAck> assignment_ack_output_;
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
void MissionStateMachine::SetAssignmentInput(
    common::Topic<common::MissionAssignment>& assignment) {
    impl_->SetAssignmentInput(assignment);
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
common::Topic<common::MissionAssignmentAck>&
MissionStateMachine::AssignmentAckOutput() {
    return impl_->AssignmentAckOutput();
}
common::MissionState MissionStateMachine::CurrentState() const {
    return impl_->CurrentState();
}
uint64_t MissionStateMachine::TransitionCount() const {
    return impl_->TransitionCount();
}
uint64_t MissionStateMachine::ErrorCount() const { return impl_->ErrorCount(); }
void MissionStateMachine::ForceStateForTest(common::MissionState state) {
    impl_->ForceStateForTest(state);
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

void MissionStateMachineStub::SetAssignmentInput(
    common::Topic<common::MissionAssignment>& /*assignment*/) {
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

common::Topic<common::MissionAssignmentAck>&
MissionStateMachineStub::AssignmentAckOutput() {
    return assignment_ack_output_;
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
