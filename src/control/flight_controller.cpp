/**
 * @file flight_controller.cpp
 * @brief IFlightController 骨架占位实现 + FlightController 正式实现
 *
 * 骨架期仅保证生命周期可运行与装配验证；业务逻辑（控制意图 → PX4
 * 设定值转换、控制算法）在实现期接入。
 *
 * FlightController（阶段四控制接入 Task 3~5）：固定周期线程消费
 * ControlIntent，门禁链（有限性/超龄/控制权）通过后经限幅流水线
 * （速度幅值钳制 → slew 加速度限幅）与无扰衔接发布 Px4Setpoint。
 * 控制权三态（Task 5）：每拍核验 PX4 快照（connected+armed+Offboard 6/0）
 * 与状态机显式请求位（reason_code bit31）；失效立即停发（valid=false）
 * 转 IDLE；意图超龄转 COAST 刹车保持，coast_timeout 超时转 IDLE 停发；
 * 位置式意图经 Home 相对围栏钳制。
 */
#include "control/flight_controller.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

#include "communication/px4_link.h"
#include "config/config.h"
#include "control/rate_limiter.h"
#include "control/staleness_checker.h"

namespace drone::control {
namespace {

using Clock = std::chrono::steady_clock;

/// 已定义控制来源上限（common::ControlIntent::control_source 注释：
/// 0=位置 1=视觉 2=安全 3=遥控器 4=PX4 保护）；超出按安全语义降级。
constexpr uint8_t kMaxDefinedControlSource = 4;

/// PX4 1.17.0 模式编码：Offboard = custom main mode 6 / sub mode 0。
/// 与 px4_link.cpp 心跳解码侧编码一致，控制权门禁每拍核验。
constexpr uint8_t kPx4MainModeOffboard = 6;
constexpr uint8_t kPx4SubModeOffboard = 0;

/// 控制权状态名（日志用）。
const char* AuthorityName(ControlAuthority authority) {
    switch (authority) {
        case ControlAuthority::kIdle:
            return "IDLE";
        case ControlAuthority::kActive:
            return "ACTIVE";
        case ControlAuthority::kCoast:
            return "COAST";
    }
    return "UNKNOWN";
}

/// 返回当前进程单调时钟毫秒，用于超龄判定与输出时间戳。
uint64_t MonotonicMs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now().time_since_epoch())
            .count());
}

/// 拒绝/异常日志节流规则：第1次与每满100次打印一次，避免周期线程刷屏。
bool ShouldLogThrottled(uint64_t count) {
    return count == 1 || count % 100 == 0;
}

/// 意图字段有限性检查：数值通道全部 std::isfinite，NaN/Inf 直接拒绝。
bool IsFiniteIntent(const common::ControlIntent& intent) {
    return std::isfinite(intent.target_x) && std::isfinite(intent.target_y) &&
           std::isfinite(intent.target_z) && std::isfinite(intent.yaw_deg) &&
           std::isfinite(intent.yaw_rate_dps) &&
           std::isfinite(intent.max_speed_mps);
}

}  // namespace

// 数据流：ControlIntent Topic → 控制算法 → Px4Setpoint Topic。
// Stub 仅暴露生命周期与主题接口，用于装配验证；真实控制算法在实现期接入。

FlightControllerStub::FlightControllerStub() {
    SPDLOG_INFO("飞行控制器部件骨架创建");
}

FlightControllerStub::~FlightControllerStub() {
    SPDLOG_INFO("飞行控制器部件骨架销毁");
}

// 启动：骨架无真实后端，仅置位运行标志（幂等语义由调用方 Start 前置判断保证）。
bool FlightControllerStub::Start() {
    running_ = true;
    SPDLOG_INFO("飞行控制器部件骨架启动");
    return true;
}

// 停止：幂等，仅在已启动时置位，未启动直接返回。
void FlightControllerStub::Stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    SPDLOG_INFO("飞行控制器部件骨架停止");
}

bool FlightControllerStub::IsRunning() const {
    return running_;
}

// 骨架期忽略输入绑定；实现期保存订阅并启动控制周期。
void FlightControllerStub::SetInput(common::Topic<common::ControlIntent>& /*intent*/) {
}

common::Topic<common::Px4Setpoint>& FlightControllerStub::SetpointOutput() {
    return setpoint_output_;
}

uint64_t FlightControllerStub::SetpointCount() const {
    return setpoint_count_;
}

uint64_t FlightControllerStub::ErrorCount() const {
    return error_count_;
}

// =============================================================================
// FlightController —— 正式实现（骨架阶段）
//
// 线程模型：单周期线程按 config.period 执行 Tick()；意图经 Topic 有界订阅
// 队列传入，Tick 开头排干只留最新。诊断计数用原子量，跨线程只读安全。
// 日志纪律：控制周期热路径不打日志；拒绝事件节流（第1次与每满100次）。
// =============================================================================

class FlightController::Impl {
public:
    explicit Impl(const config::FlightControllerConfig& config) : config_(config) {
        // 配置在 LoadAppConfig 加载期已 Validate() 校验（fail-fast）；
        // 此处不调用 drone::config::Validate()（避免 drone_control_core →
        // drone_config 链接依赖反转），但对周期做本地防御：period<=0 会让
        // Run() 的 sleep_until 立即返回、周期线程退化为忙等空转。
        if (config_.period.count() <= 0) {
            throw std::invalid_argument("controller.period_ms 必须为正");
        }
    }

    ~Impl() { Stop(); }

    bool Start() {
        bool expected = false;
        if (!running_.compare_exchange_strong(expected, true)) {
            return true;  // 幂等：已启动直接成功
        }
        worker_ = std::thread([this] { Run(); });
        SPDLOG_INFO("飞行控制器启动: period={}ms intent_timeout={}ms coast_timeout={}ms",
                    config_.period.count(), config_.intent_timeout.count(),
                    config_.coast_timeout.count());
        return true;
    }

    void Stop() {
        if (!running_.exchange(false)) {
            return;  // 幂等：未启动直接返回
        }
        if (worker_.joinable()) {
            worker_.join();
        }
        SPDLOG_INFO("飞行控制器停止: setpoint_count={} rejected_intent_count={}",
                    setpoint_count_.load(), rejected_intent_count_.load());
    }

    bool IsRunning() const { return running_.load(); }

    void SetInput(common::Topic<common::ControlIntent>& intent) {
        // 队列有界、优先最新：周期线程排干时只保留最后一条意图。
        intent_subscription_ = intent.Subscribe(kIntentQueueCapacity);
    }

    common::Topic<common::Px4Setpoint>& SetpointOutput() { return setpoint_output_; }

    uint64_t SetpointCount() const { return setpoint_count_.load(); }
    uint64_t ErrorCount() const { return error_count_.load(); }
    uint64_t RejectedIntentCount() const { return rejected_intent_count_.load(); }
    uint64_t ClampedCount() const { return clamped_count_.load(); }

    ControlAuthority Authority() const {
        return authority_.load(std::memory_order_acquire);
    }

    void SetPx4LinkSource(const communication::IPx4Link* link) {
        // 控制权门禁与 Home 围栏的只读快照源；只允许装配期（Start 前）注入。
        px4_link_ = link;
    }

    /// 测试入口：强制控制权状态。授权链每拍重估，门禁不通时下一拍即撤到
    /// IDLE——单测维持 ACTIVE 须配合 MockPx4Link 快照与意图请求位。
    void TestForceAuthority(ControlAuthority authority) {
        authority_.store(authority, std::memory_order_release);
    }

    /// 测试入口：以指定时间戳同步执行一拍。须在未 Start 状态调用；
    /// 运行时守卫：Start 后调用与周期线程并发访问 Tick 内部无锁状态，
    /// 直接忽略并节流 WARN。
    void TestTick(uint64_t now_ms) {
        if (running_.load(std::memory_order_acquire)) {
            const uint64_t count = ++error_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("飞行控制器 TestTick 在运行状态被调用，已忽略（须先 Stop） 累计={}",
                            count);
            }
            return;
        }
        Tick(now_ms);
    }

private:
    /// 意图订阅队列容量：周期消费，4 条足够吸收突发，溢出丢旧保新。
    static constexpr std::size_t kIntentQueueCapacity = 4;

    /// 固定周期线程：每 period 执行一次 Tick()。
    void Run() {
        auto next = Clock::now();
        while (running_.load(std::memory_order_acquire)) {
            next += config_.period;
            Tick(MonotonicMs());
            // Tick 超时停摆后钳制下一拍起点：不连发补偿拍（追赶无意义，
            // 上游意图只留最新，限幅器以固定 dt 推进）。
            if (next < Clock::now()) {
                next = Clock::now();
            }
            std::this_thread::sleep_until(next);
        }
    }

    void Tick(uint64_t now_ms) {
        // 1) 排干意图队列，只留最新（入口门禁在排干时执行，拒绝即计数；
        //    最新通过门禁的意图同步刷新"状态机显式请求"锁存）
        DrainIntents(now_ms);
        // 2) 控制权门禁：PX4 快照（connected+armed+Offboard 6/0）+
        //    状态机显式请求位，每拍核验（非一次性）
        const bool gate = AuthorityGateOpen();
        switch (authority_.load(std::memory_order_acquire)) {
            case ControlAuthority::kIdle:
                // IDLE→ACTIVE 须授权链全通且持有新鲜请求位意图：请求锁存
                // 不随时间过期，陈旧请求不得事后激活授权（评审 fix round 2）。
                if (!gate || !FreshIntentAvailable(now_ms)) {
                    return;  // IDLE 停发：不发布任何 setpoint
                }
                TransitionTo(ControlAuthority::kActive, "授权链全通");
                break;
            case ControlAuthority::kActive:
                if (!gate) {
                    // 控制权失效（心跳断/非 Offboard/未 armed/请求撤销）：
                    // 立即停发并转 IDLE——离开即撤，不做迟滞（模式抖动不抖动）
                    TransitionTo(ControlAuthority::kIdle, "授权链断开");
                    PublishInvalidStop(now_ms);
                    return;
                }
                break;
            case ControlAuthority::kCoast:
                if (!gate) {
                    TransitionTo(ControlAuthority::kIdle, "COAST 中授权链断开");
                    PublishInvalidStop(now_ms);
                    return;
                }
                if (static_cast<int64_t>(now_ms) -
                        static_cast<int64_t>(coast_since_ms_) >
                    config_.coast_timeout.count()) {
                    TransitionTo(ControlAuthority::kIdle, "COAST 超时");
                    PublishInvalidStop(now_ms);
                    return;
                }
                if (!FreshIntentAvailable(now_ms)) {
                    PublishBrake(now_ms);  // COAST 刹车保持
                    return;
                }
                TransitionTo(ControlAuthority::kActive, "意图恢复");
                break;
        }
        // ACTIVE 路径：意图缺失或持有期超龄 → 转 COAST 刹车保持
        if (!latest_intent_.has_value()) {
            EnterCoast(now_ms, "无意图");
            PublishBrake(now_ms);
            return;
        }
        // ① 持有期再超龄（意图在消费间隔内过期）→ 拒绝并转 COAST 刹车。
        //    转入 COAST 后走 kCoast 分支不再经过此处，拒绝计数仅本拍 +1。
        if (IntentStale(*latest_intent_, now_ms)) {
            RejectIntent("意图持有期超龄");
            EnterCoast(now_ms, "意图持有期超龄");
            PublishBrake(now_ms);
            return;
        }
        PublishFromIntent(*latest_intent_, now_ms);
    }

    /// 授权链（本控制器侧部分）：快照源已注入 + PX4 connected+armed+
    /// Offboard(6/0) + 状态机显式请求位。门禁判据只取 LatestState() 快照的
    /// connected 字段（断开方向有滞后窗口：链路侧先清原子标志后发布快照，
    /// 不得与 IsConnected() 交叉推理）。
    bool AuthorityGateOpen() const {
        if (px4_link_ == nullptr) {
            return false;  // 未注入（影子模式/未装配）：永不授权
        }
        const auto state = px4_link_->LatestState();
        const bool offboard = state.connected && state.armed &&
                              state.flight_mode == kPx4MainModeOffboard &&
                              state.flight_sub_mode == kPx4SubModeOffboard;
        return offboard && state_machine_requested_;
    }

    /// 控制权迁移：WARN 级状态变化日志；进入 IDLE 时清零"状态机显式请求"
    /// 锁存——回 ACTIVE 必须重新满足授权链（含显式请求），防模式抖动抖动。
    void TransitionTo(ControlAuthority next, const char* reason) {
        const ControlAuthority previous =
            authority_.load(std::memory_order_acquire);
        if (previous == next) {
            return;
        }
        authority_.store(next, std::memory_order_release);
        if (next == ControlAuthority::kIdle) {
            state_machine_requested_ = false;
        }
        SPDLOG_WARN("飞行控制器控制权迁移: {} -> {} ({})", AuthorityName(previous),
                    AuthorityName(next), reason);
    }

    /// 意图超龄判定（valid_until 截止 + receive_time 年龄超 intent_timeout）。
    bool IntentStale(const common::ControlIntent& intent, uint64_t now_ms) const {
        return now_ms >= intent.valid_until_ms ||
               StalenessChecker::IsStale(
                   static_cast<int64_t>(intent.header.receive_time_ms),
                   static_cast<int64_t>(now_ms), config_.intent_timeout.count());
    }

    /// COAST 恢复判据：存在未超龄意图。
    bool FreshIntentAvailable(uint64_t now_ms) const {
        return latest_intent_.has_value() && !IntentStale(*latest_intent_, now_ms);
    }

    /// 进入 COAST：记录刹车保持起点（重复进入不刷新计时——超时以首次进入为准）。
    void EnterCoast(uint64_t now_ms, const char* reason) {
        if (authority_.load(std::memory_order_acquire) !=
            ControlAuthority::kCoast) {
            coast_since_ms_ = now_ms;
        }
        TransitionTo(ControlAuthority::kCoast, reason);
    }

    /// 排干意图队列，只保留通过入口门禁的最新一条。
    void DrainIntents(uint64_t now_ms) {
        while (auto message = intent_subscription_.TryTake()) {
            common::ControlIntent intent = **message;
            // 控制器是意图的接收边界：上游未打戳时补 receive_time_ms，
            // 保证超龄判定有可信取样时间。
            if (intent.header.receive_time_ms == 0) {
                intent.header.receive_time_ms = now_ms;
            }
            // ③ 有限性（NaN/Inf 意图字段直接拒绝）
            if (!IsFiniteIntent(intent)) {
                RejectIntent("意图字段非有限(NaN/Inf)");
                continue;
            }
            // ① 超龄（含到达即超龄）
            if (now_ms >= intent.valid_until_ms ||
                StalenessChecker::IsStale(
                    static_cast<int64_t>(intent.header.receive_time_ms),
                    static_cast<int64_t>(now_ms), config_.intent_timeout.count())) {
                RejectIntent("意图超龄");
                continue;
            }
            // 状态机显式请求锁存：跟随最新通过入口门禁的意图（约定：
            // reason_code bit31，见 common::kControlIntentReasonRequestControl；
            // 被拒绝的意图不刷新锁存）。
            state_machine_requested_ =
                (intent.reason_code & common::kControlIntentReasonRequestControl) !=
                0;
            latest_intent_ = std::move(intent);
        }
    }

    /// 拒绝计数 + 节流日志（第1次与每满100次）。
    void RejectIntent(const char* reason) {
        const uint64_t count = ++rejected_intent_count_;
        if (ShouldLogThrottled(count)) {
            SPDLOG_WARN("飞行控制器拒绝意图({}): 累计={}", reason, count);
        }
    }

    /// 发布零速刹停设定值：速度三分量为 0、valid=true。
    /// 刹车后 slew 状态归零，保证恢复跟踪后指令从零渐变
    /// （SlewRateLimiter::Reset 语义，S6 首条指令突变抑制）。
    void PublishBrake(uint64_t now_ms) {
        common::Px4Setpoint setpoint;
        setpoint.type = common::SetpointType::kVelocity;
        setpoint.x = 0.f;
        setpoint.y = 0.f;
        setpoint.z = 0.f;
        setpoint.valid = true;
        vx_slew_.Reset(0.0);
        vy_slew_.Reset(0.0);
        vz_slew_.Reset(0.0);
        last_output_ = setpoint;  // 零速也是速度式输出，供无扰衔接起点判定
        Publish(std::move(setpoint), now_ms);
    }

    /// 发布停发设定值：valid=false（Px4Link 收到后立即停止向 PX4 发送）。
    /// 用于控制权失效/COAST 超时转 IDLE。衔接状态归零：恢复授权后
    /// 速度式输出从 0 斜坡起步（last_output_ 非速度式 → 从 0 起步路径）。
    void PublishInvalidStop(uint64_t now_ms) {
        common::Px4Setpoint setpoint;
        setpoint.type = common::SetpointType::kNone;
        setpoint.valid = false;
        vx_slew_.Reset(0.0);
        vy_slew_.Reset(0.0);
        vz_slew_.Reset(0.0);
        last_output_ = setpoint;
        Publish(std::move(setpoint), now_ms);
    }

    /// 限幅流水线入口：未知来源安全降级 → 按类型分发。
    void PublishFromIntent(const common::ControlIntent& intent, uint64_t now_ms) {
        // 未定义 control_source（>4）：节流 WARN + 按 kSafety(2) 语义降级——
        // 只允许刹停类意图生效，其余抑制输出（不崩溃不直通）。
        if (intent.control_source > kMaxDefinedControlSource) {
            const uint64_t count = ++error_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("飞行控制器收到未定义控制来源: source={} 按安全语义降级 累计={}",
                            intent.control_source, count);
            }
            if (intent.type != common::ControlIntentType::kBrakeHover) {
                return;
            }
        }
        switch (intent.type) {
            case common::ControlIntentType::kVelocityHeading:
                PublishVelocity(intent, now_ms);
                return;
            case common::ControlIntentType::kBrakeHover:
                PublishBrake(now_ms);
                return;
            case common::ControlIntentType::kPositionGuide:
                PublishPosition(intent, now_ms);
                return;
            default: {
                // 起飞/降落/返航/人工接管等类型暂不支持，拒绝计数。
                const uint64_t count = ++error_count_;
                if (ShouldLogThrottled(count)) {
                    SPDLOG_WARN("飞行控制器暂不支持的意图类型: type={} 累计={}",
                                static_cast<int>(intent.type), count);
                }
                return;
            }
        }
    }

    /// 速度式限幅流水线（热路径零分配，限幅器为成员跨周期保持状态）：
    /// 速度幅值等比钳制 → 无扰衔接（来源/类型变化时斜坡起点=上一周期实际
    /// 输出）→ SlewRateLimiter 加速度限幅。
    void PublishVelocity(const common::ControlIntent& intent, uint64_t now_ms) {
        double vx = intent.target_x, vy = intent.target_y, vz = intent.target_z;
        // ① 速度幅值限幅（等比钳制，方向不变）
        const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (speed > config_.max_speed_mps) {
            const double scale = config_.max_speed_mps / speed;
            vx *= scale;
            vy *= scale;
            vz *= scale;
            const uint64_t count = ++clamped_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("飞行控制器速度幅值限幅: |v|={:.2f} > max={:.2f} 累计={}",
                            speed, config_.max_speed_mps, count);
            }
        }
        // ② 无扰衔接：来源/类型变化时斜坡起点=上一周期实际输出；上一输出
        //    非速度式（位置/首拍）时量纲不同，从 0 起步（防止位置值被当成
        //    速度初值引发跳变）。
        if (intent.control_source != last_source_ || intent.type != last_type_) {
            const bool from_velocity =
                last_output_.type == common::SetpointType::kVelocity;
            vx_slew_ = SlewRateLimiter(config_.max_accel_mps2,
                                       from_velocity ? last_output_.x : 0.0);
            vy_slew_ = SlewRateLimiter(config_.max_accel_mps2,
                                       from_velocity ? last_output_.y : 0.0);
            vz_slew_ = SlewRateLimiter(config_.max_accel_mps2,
                                       from_velocity ? last_output_.z : 0.0);
        }
        // ③ slew 加速度限幅：dt<=0（时钟回退/首拍）时 SlewRateLimiter 冻结
        //    输出——既有语义，勿改。
        double dt_s = static_cast<double>(config_.period.count()) / 1000.0;
        if (now_ms <= last_output_ms_) {
            dt_s = 0.0;  // 时钟回退：冻结输出，保持上一值
        }
        common::Px4Setpoint setpoint;
        setpoint.type = common::SetpointType::kVelocity;
        setpoint.x = static_cast<float>(vx_slew_.Limit(vx, dt_s));
        setpoint.y = static_cast<float>(vy_slew_.Limit(vy, dt_s));
        setpoint.z = static_cast<float>(vz_slew_.Limit(vz, dt_s));
        // 航向通道透传（互斥约束沿用 Px4Link 现有校验：use_yaw 与
        // use_yaw_rate 不得同时为 true，违例由 Px4Link 拒发）。
        setpoint.use_yaw = intent.yaw_deg != 0.f;
        setpoint.yaw_deg = intent.yaw_deg;
        setpoint.use_yaw_rate = intent.yaw_rate_dps != 0.f;
        setpoint.yaw_rate_dps = intent.yaw_rate_dps;
        setpoint.valid = true;
        last_source_ = intent.control_source;
        last_type_ = intent.type;
        last_output_ = setpoint;  // 仅 type/x/y/z 参与衔接判定，header 由 Publish 填充
        Publish(std::move(setpoint), now_ms);
    }

    /// 位置式（S5 远端引导）：透传位置目标，不做 slew——位置阶跃由 PX4
    /// 内部轨迹规划平滑，本控制器只管速度式连续性；速度上限写入由 Px4Link
    /// 侧 type mask 语义保证。完整联调测试归 Task 7。
    ///
    /// Home 相对围栏（Task 5）：意图目标为局部 NED，项目坐标约定局部 NED
    /// 原点即 PX4 Home（docs/数据接口文档.md、include/perception/目标估计器.md），
    /// 故目标值本身即 Home 相对量；`home_valid=false` 时该约定无基准，
    /// 位置式意图拒绝输出（无基准不可围栏）。
    ///
    /// 注意（2026-10-16 结论）：该约定是近似——PX4 局部帧原点实为 EKF 锚点，
    /// Home 是另一点，两者相差 H（地面阶段 Home 贴住飞机故 H≤1~2m；解锁/起飞后
    /// Home 冻结，H = 起飞时刻飞机的局部坐标，位置意图与围栏随之偏移 H）。
    /// 影响与处置见 include/control/flight_controller.md「Home 漂移对控制的
    /// 影响」一节。
    void PublishPosition(const common::ControlIntent& intent, uint64_t now_ms) {
        const auto state = px4_link_ != nullptr
                               ? px4_link_->LatestState()
                               : common::FlightStateSnapshot{};
        if (!state.home_valid) {
            RejectIntent("位置意图无 Home 基准(home_valid=false)，拒绝");
            return;
        }
        double target_x = intent.target_x;
        double target_y = intent.target_y;
        double target_z = intent.target_z;
        // 水平半径：模长超 max_radius_m 等比钳制（方向不变）
        const double radius = std::hypot(target_x, target_y);
        if (radius > config_.max_radius_m) {
            const double scale = config_.max_radius_m / radius;
            target_x *= scale;
            target_y *= scale;
            const uint64_t count = ++clamped_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("飞行控制器围栏水平半径钳制: r={:.2f} > max={:.2f} 累计={}",
                            radius, config_.max_radius_m, count);
            }
        }
        // 高度：NED z 向下为正，相对 Home 高度 = -z；超 max_altitude_m 钳制
        const double altitude_m = -target_z;
        if (altitude_m > config_.max_altitude_m) {
            target_z = -config_.max_altitude_m;
            const uint64_t count = ++clamped_count_;
            if (ShouldLogThrottled(count)) {
                SPDLOG_WARN("飞行控制器围栏高度钳制: h={:.2f} > max={:.2f} 累计={}",
                            altitude_m, config_.max_altitude_m, count);
            }
        }
        common::Px4Setpoint setpoint;
        setpoint.type = common::SetpointType::kPosition;
        setpoint.x = static_cast<float>(target_x);
        setpoint.y = static_cast<float>(target_y);
        setpoint.z = static_cast<float>(target_z);
        // 航向通道透传（互斥约束同速度式，沿用 Px4Link 现有校验）。
        setpoint.use_yaw = intent.yaw_deg != 0.f;
        setpoint.yaw_deg = intent.yaw_deg;
        setpoint.use_yaw_rate = intent.yaw_rate_dps != 0.f;
        setpoint.yaw_rate_dps = intent.yaw_rate_dps;
        setpoint.valid = true;
        last_source_ = intent.control_source;
        last_type_ = intent.type;
        last_output_ = setpoint;
        Publish(std::move(setpoint), now_ms);
    }

    void Publish(common::Px4Setpoint setpoint, uint64_t now_ms) {
        setpoint.header.sequence = ++setpoint_sequence_;
        setpoint.header.receive_time_ms = now_ms;
        setpoint.header.valid_for_ms =
            static_cast<uint64_t>(config_.period.count()) * 2;
        // 坐标系契约：本控制器输出的位置/速度设定值统一为局部 NED
        // （frame_id=3，编码见 include/common/types.h）。Px4Link 的
        // IsSetpointStructurallyValid 强制核验该值——缺失会使全部设定值被
        // 静默拒绝（stage4 门禁实现期代码核对发现：控制器 → Px4Link 全链零发送；
        // 回归网见 tests/control/flight_controller_link_contract_test.cpp）。
        setpoint.header.frame_id = 3;
        (void)setpoint_output_.Publish(
            std::make_shared<const common::Px4Setpoint>(std::move(setpoint)));
        ++setpoint_count_;
        last_output_ms_ = now_ms;
    }

    const config::FlightControllerConfig config_;
    std::atomic<bool> running_{false};
    std::thread worker_;
    common::Topic<common::ControlIntent>::Subscription intent_subscription_;
    common::Topic<common::Px4Setpoint> setpoint_output_;
    const communication::IPx4Link* px4_link_ = nullptr;  // 只读门禁源（快照/围栏）

    // 以下仅周期线程访问（诊断接口经原子量暴露）。
    std::optional<common::ControlIntent> latest_intent_;
    uint64_t setpoint_sequence_ = 0;
    bool state_machine_requested_ = false;  ///< 状态机显式请求锁存（reason_code bit31）
    uint64_t coast_since_ms_ = 0;           ///< 本次 COAST 进入时刻
    std::atomic<ControlAuthority> authority_{ControlAuthority::kIdle};
    std::atomic<uint64_t> setpoint_count_{0};
    std::atomic<uint64_t> rejected_intent_count_{0};
    std::atomic<uint64_t> error_count_{0};
    std::atomic<uint64_t> clamped_count_{0};

    // 限幅流水线状态（仅周期线程访问；成员声明在 config_ 之后，
    // 保证初始化顺序正确）。slew 初值 0：天然实现首条指令突变抑制（S6）。
    SlewRateLimiter vx_slew_{config_.max_accel_mps2};
    SlewRateLimiter vy_slew_{config_.max_accel_mps2};
    SlewRateLimiter vz_slew_{config_.max_accel_mps2};
    uint8_t last_source_ = 0;  ///< 上一拍意图控制来源（无扰衔接判定）
    common::ControlIntentType last_type_ = common::ControlIntentType::kNone;
    common::Px4Setpoint last_output_{};   ///< 上一拍实际输出（衔接斜坡起点）
    uint64_t last_output_ms_ = 0;         ///< 上一拍输出时刻（时钟回退检测）
};

FlightController::FlightController(const config::FlightControllerConfig& config)
    : impl_(std::make_unique<Impl>(config)) {
    SPDLOG_INFO("飞行控制器创建（默认 IDLE 不发布设定值；限幅流水线+控制权三态+Home 围栏已接入）");
}

FlightController::~FlightController() = default;

bool FlightController::Start() { return impl_->Start(); }

void FlightController::Stop() { impl_->Stop(); }

bool FlightController::IsRunning() const { return impl_->IsRunning(); }

void FlightController::SetInput(common::Topic<common::ControlIntent>& intent) {
    impl_->SetInput(intent);
}

common::Topic<common::Px4Setpoint>& FlightController::SetpointOutput() {
    return impl_->SetpointOutput();
}

uint64_t FlightController::SetpointCount() const { return impl_->SetpointCount(); }

uint64_t FlightController::ErrorCount() const { return impl_->ErrorCount(); }

ControlAuthority FlightController::Authority() const { return impl_->Authority(); }

uint64_t FlightController::RejectedIntentCount() const {
    return impl_->RejectedIntentCount();
}

uint64_t FlightController::ClampedCount() const { return impl_->ClampedCount(); }

void FlightController::SetPx4LinkSource(const communication::IPx4Link* link) {
    impl_->SetPx4LinkSource(link);
}

void FlightController::TestForceAuthority(ControlAuthority authority) {
    impl_->TestForceAuthority(authority);
}

void FlightController::TestTick(uint64_t now_ms) { impl_->TestTick(now_ms); }

}  // namespace drone::control
