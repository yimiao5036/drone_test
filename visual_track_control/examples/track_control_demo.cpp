// =============================================================================
// track_control_demo.cpp —— 视觉跟踪控制律演示程序
//
// 数据流（模拟单线程整合的完整链路）：
//   合成运动检测框 → TargetTracker::Update()（模拟推理线程每帧调用）
//   → LastResult() + 打戳 → ControlSnapshot → TrackingControlLaw::Update()
//   → MavlinkCommandGateway → LoggingTransport（字节数 + 十六进制摘要）
//
// 依次演示三种分支（对应物理追踪思路 §6.4）：
//   1. 正常跟踪：持续喂入合成检测框，输出速度航向意图（垂直/距离通道
//      统一为速度分量）；
//   2. Coast：停止喂检测 → is_predicted==true → 清积分，继续预测跟踪；
//   3. 判丢刹车：连续丢帧超过追踪库阈值 → tracked==false → 零速刹车。
//
// 运行方式：
//   track_control_demo [配置文件路径]   # 缺省使用编译期内置路径
// =============================================================================

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <spdlog/spdlog.h>
#include <target_tracker/target_tracker.h>

#include "visual_track_control/i_command_transport.h"
#include "visual_track_control/mavlink_command_gateway.h"
#include "visual_track_control/tracking_control_law.h"
#include "visual_track_control/vtc_config.h"
#include "visual_track_control/vtc_types.h"

#ifndef VTC_DEMO_DEFAULT_CONFIG
#define VTC_DEMO_DEFAULT_CONFIG "config/track_control.json"
#endif

namespace {

// ---- 演示时长（秒）：总时长有限，自动退出 ----
constexpr int kNormalPhaseSeconds = 6;   // 阶段1：正常跟踪
constexpr int kCoastPhaseSeconds = 4;    // 阶段2：Coast（停止喂检测）
constexpr int kLostPhaseSeconds = 3;     // 阶段3：判丢刹车

// 追踪库判丢阈值放大：默认 8 帧在 20Hz 下 Coast 仅 0.4 秒，不便于观察；
// 演示专用放宽到 60 帧（约 3 秒），正式参数以追踪库规格为准。
constexpr int kDemoMaxLostFrames = 60;

const char* ModeName(drone::vtc::ControlMode mode) {
    switch (mode) {
        case drone::vtc::ControlMode::kVelocityHeading: return "速度航向";
        case drone::vtc::ControlMode::kBrakeHover:      return "零速刹车";
        case drone::vtc::ControlMode::kDegradedHold:    return "降级保持";
    }
    return "未知";
}

/// 演示用传输实现：统计字节数并打印十六进制摘要（按秒级节流，防止刷屏）
class LoggingTransport : public drone::vtc::ICommandTransport {
public:
    bool Send(const std::uint8_t* data, std::size_t len) override {
        ++msg_count_;
        total_bytes_ += len;

        const auto now_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        if (now_ms - last_print_ms_ >= 1000) {
            // 十六进制摘要：取包头前 12 字节
            std::string hex;
            char buf[4];
            const std::size_t n = len < 12 ? len : 12;
            for (std::size_t i = 0; i < n; ++i) {
                std::snprintf(buf, sizeof(buf), "%02X ", data[i]);
                hex += buf;
            }
            spdlog::info("[Transport] 累计 {} 帧 / {} 字节; 本帧 {} 字节, 包头: {}",
                         msg_count_, total_bytes_, len, hex);
            last_print_ms_ = now_ms;
        }
        return true;
    }

private:
    std::int64_t last_print_ms_ = 0;
    std::uint64_t msg_count_ = 0;
    std::uint64_t total_bytes_ = 0;
};

/// 合成运动检测框：从画面右下向中心缓慢漂移（模拟目标被持续锁定）
std::vector<drone::tracker::Detection> SynthesizeDetections(double elapsed_s,
                                                            int width, int height) {
    const double center_x = width * 0.5 + 160.0 - 20.0 * elapsed_s;
    const double center_y = height * 0.5 + 80.0 - 10.0 * elapsed_s;
    const double half_w = 70.0;
    const double half_h = 50.0;

    drone::tracker::Detection det;
    det.x1 = center_x - half_w;
    det.y1 = center_y - half_h;
    det.x2 = center_x + half_w;
    det.y2 = center_y + half_h;
    det.conf = 0.9;
    det.cls_id = 0;
    return {det};
}

}  // namespace

int main(int argc, char** argv) {
    // ---- 启动期：一次性加载配置（缺字段/非法值直接抛异常终止） ----
    const std::string config_path = argc > 1 ? argv[1] : VTC_DEMO_DEFAULT_CONFIG;
    drone::vtc::TrackingConfig cfg;
    try {
        cfg = drone::vtc::TrackingConfig::LoadFromJson(config_path);
    } catch (const std::exception& e) {
        spdlog::error("配置加载失败: {}", e.what());
        return 1;
    }
    spdlog::info("配置加载成功: {} (控制频率 {} Hz)", config_path,
                 cfg.control.frequency_hz);

    // ---- 构造：追踪库 / 控制律 / 传输 / 网关 ----
    drone::tracker::TargetTracker::Config tracker_cfg;
    tracker_cfg.max_lost_frames = kDemoMaxLostFrames;  // 演示专用判丢阈值
    drone::tracker::TargetTracker tracker(tracker_cfg);

    drone::vtc::TrackingControlLaw control_law(cfg);
    LoggingTransport transport;
    drone::vtc::MavlinkCommandGateway gateway(transport, /*sys=*/1, /*comp=*/1,
                                              /*target_sys=*/1, /*target_comp=*/1);

    const int width = cfg.camera.image_width;
    const int height = cfg.camera.image_height;
    const int period_ms = static_cast<int>(1000.0 / cfg.control.frequency_hz);
    const int normal_ticks = kNormalPhaseSeconds * static_cast<int>(cfg.control.frequency_hz);
    const int coast_ticks = kCoastPhaseSeconds * static_cast<int>(cfg.control.frequency_hz);
    const int lost_ticks = kLostPhaseSeconds * static_cast<int>(cfg.control.frequency_hz);
    const int total_ticks = normal_ticks + coast_ticks + lost_ticks;

    spdlog::info("演示开始: 正常跟踪 {}s → Coast {}s → 判丢刹车 {}s (共 {} 拍, {} ms/拍)",
                 kNormalPhaseSeconds, kCoastPhaseSeconds, kLostPhaseSeconds,
                 total_ticks, period_ms);

    drone::tracker::FrameShape frame_shape;
    frame_shape.height = height;
    frame_shape.width = width;

    const char* current_phase = nullptr;

    // ---- 主循环：按配置频率固定节拍（§7.1） ----
    for (int tick = 0; tick < total_ticks; ++tick) {
        const double elapsed_s = static_cast<double>(tick) / cfg.control.frequency_hz;
        // 模拟单调时钟：1 秒偏移保证取样时间戳有效
        const std::int64_t now_ms = 1000 + static_cast<std::int64_t>(tick) * period_ms;

        // ---- 阶段划分 ----
        const char* phase = tick < normal_ticks ? "正常跟踪"
                            : tick < normal_ticks + coast_ticks ? "Coast"
                                                                : "判丢刹车";
        if (phase != current_phase) {
            spdlog::info("===== 阶段切换: {} =====", phase);
            current_phase = phase;
        }

        // ---- 模拟推理线程：仅正常跟踪阶段喂入检测框 ----
        // 阶段2/3 停止喂检测：先 Coast（is_predicted），后判丢（tracked==false）
        std::vector<drone::tracker::Detection> detections;
        if (tick < normal_ticks) {
            detections = SynthesizeDetections(elapsed_s, width, height);
        }
        tracker.Update(detections, frame_shape);

        // ---- 控制线程：LastResult() + 打戳（TrackResult 无时间戳，§7.2） ----
        drone::vtc::ControlSnapshot snapshot;
        snapshot.track = tracker.LastResult();
        snapshot.track_time_ms = now_ms;

        // 雷达：正常阶段关联成立；Coast/刹车阶段取消关联（演示 §6.3 降级路径）
        snapshot.radar_distance_m = 40.0 - 3.0 * elapsed_s;
        snapshot.associated_to_target = (tick < normal_ticks) &&
                                        snapshot.radar_distance_m > 0.0;
        snapshot.radar_time_ms = now_ms;

        // 自机姿态：演示中始终新鲜有效（接入主工程时来自 FlightStateSnapshot）
        snapshot.attitude_present = true;
        snapshot.attitude_time_ms = now_ms;

        // ---- 控制律单周期计算 ----
        const drone::vtc::ControlOutput output = control_law.Update(snapshot, now_ms);

        // ---- 按模式经网关发送 ----
        const auto time_boot_ms = static_cast<std::uint32_t>(now_ms);
        switch (output.mode) {
            case drone::vtc::ControlMode::kVelocityHeading:
                gateway.SendVelocityHeading(output, time_boot_ms);
                break;
            case drone::vtc::ControlMode::kBrakeHover:
                gateway.SendBrakeHover(time_boot_ms);
                break;
            case drone::vtc::ControlMode::kDegradedHold:
                break;  // 降级保持：不发送指令（仅告警，§7.1）
        }

        // ---- 半秒一次的摘要打印（演示观测用，非生产热路径） ----
        if (tick % 10 == 0) {
            spdlog::info(
                "[t={:5.1f}s] 阶段={} 模式={}{} tracked={} vx={:+.2f} vy={:+.2f} "
                "vz={:+.2f} yaw_rate={:+.1f}dps d={:5.1f}m 关联={}",
                elapsed_s, phase, ModeName(output.mode),
                output.coast_active ? "(Coast)" : "", snapshot.track.tracked,
                output.vx_mps, output.vy_mps, output.vz_mps, output.yaw_rate_dps,
                snapshot.radar_distance_m, snapshot.associated_to_target);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }

    control_law.Reset();  // 停止视觉闭环：清全部状态（§6.4）
    spdlog::info("演示结束");
    return 0;
}
