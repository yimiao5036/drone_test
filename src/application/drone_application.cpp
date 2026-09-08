#include "application/drone_application.h"

#include <chrono>
#include <exception>
#include <memory>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

#include "communication/ground_station_link.h"
#include "communication/px4_link.h"
#include "perception/detection_backend.h"
#include "perception/yolo_detector.h"
#include "state_machine/mission_state_machine.h"
#include "video/camera_receiver.h"
#include "video/frame_compositor.h"
#include "video/video_decoder.h"
#include "video_transmission/video_sender.h"

namespace drone::application {

// ============================================================================
// DroneApplication —— 正式进程的组件所有者与装配根。
//
// 负责根据 AppConfig 创建组件、在启动前完成 Topic 接线，并按数据依赖顺序
// 启动/停止各模块。main.cpp 不承载业务装配逻辑，只负责配置、日志、信号和等待。
// 当前正式程序仍处于安全阶段：不绑定 Px4Setpoint，不发送飞行控制命令。
// ============================================================================

DroneApplication::DroneApplication(config::AppConfig config)
    : config_(std::move(config)) {
    BuildComponents();
    BindTopics();
    SPDLOG_INFO("主程序集成创建: video={} px4={} ground_station={} control={}",
                config_.runtime.enable_video, config_.runtime.enable_px4,
                config_.runtime.enable_ground_station, config_.runtime.enable_control);
}

DroneApplication::~DroneApplication() {
    Stop();
    SPDLOG_INFO("主程序集成销毁");
}

// 根据 runtime 开关创建组件。这里只负责所有权，不在此处启动线程或连接设备。
void DroneApplication::BuildComponents() {
    if (config_.runtime.enable_video) {
        camera_ = std::make_unique<video::CameraReceiver>(config_.camera);
        decoder_ = std::make_unique<video::VideoDecoder>(config_.decoder);
        detector_ = std::make_unique<perception::YoloDetector>(config_.yolo);
        compositor_ = std::make_unique<video::FrameCompositor>(config_.compositor);
        video_sender_ =
            std::make_unique<video_transmission::VideoSender>(config_.video_sender);
    }

    if (config_.runtime.enable_px4) {
        px4_link_ = std::make_unique<communication::Px4Link>(config_.px4);
    }
    if (config_.runtime.enable_ground_station) {
        ground_station_link_ = std::make_unique<communication::GroundStationLink>(
            config_.ground_station);
    }

    // 健康管理器独立于具体数据链路；只注册实际创建的生产模块。
    health_manager_ = std::make_unique<health::HealthManager>(
        std::unique_ptr<health::ICpuLoadProvider>{}, config_.health.cpu_sample_period);
    RegisterHealthSources();

    // 影子状态机只在PX4状态源与地面站目标源同时存在时创建；本阶段不创建控制器。
    if (px4_link_ != nullptr && ground_station_link_ != nullptr) {
        mission_state_machine_ = std::make_unique<state_machine::MissionStateMachine>();
    }
}

// 注册运行期健康源。阈值全部来自 AppConfig，禁止在装配层写死业务超时。
void DroneApplication::RegisterHealthSources() {
    const auto register_source = [this](const char* name,
                                         std::chrono::milliseconds max_age) {
        if (!health_manager_->RegisterSource(name,
                                             static_cast<std::uint64_t>(max_age.count()))) {
            SPDLOG_ERROR("主程序健康源注册失败: {}", name);
        }
    };

    if (camera_ != nullptr) {
        register_source(health::source_names::kCamera, config_.health.camera_max_age);
    }
    if (decoder_ != nullptr) {
        register_source(health::source_names::kVideoDecoder,
                       config_.health.decoder_max_age);
    }
    if (detector_ != nullptr) {
        register_source(health::source_names::kYolo, config_.health.yolo_max_age);
    }
    if (video_sender_ != nullptr) {
        register_source(health::source_names::kVideo, config_.health.video_max_age);
    }
    if (px4_link_ != nullptr) {
        register_source(health::source_names::kPx4, config_.health.px4_max_age);
    }
    if (ground_station_link_ != nullptr) {
        register_source(health::source_names::kGroundStation,
                       config_.health.ground_station_max_age);
    }
}

// 在所有组件 Start 前完成 Topic 接线；生产者/消费者的启动顺序由 Start 控制。
void DroneApplication::BindTopics() {
    if (config_.runtime.enable_video) {
        decoder_->SetInput(camera_->StreamOutput());
        detector_->SetInput(decoder_->FrameOutput());
        compositor_->SetDecodedInput(decoder_->FrameOutput());
        compositor_->SetDetectionInput(detector_->DetectionOutput());
        video_sender_->SetInput(compositor_->AnnotatedOutput());
    }

    if (ground_station_link_ != nullptr && px4_link_ != nullptr) {
        ground_station_link_->SetFlightStateInput(px4_link_->StateOutput());
    }

    if (ground_station_link_ != nullptr && health_manager_ != nullptr) {
        ground_station_link_->SetHealthInput(health_manager_->Output());
    }

    if (mission_state_machine_ != nullptr && ground_station_link_ != nullptr &&
        px4_link_ != nullptr && health_manager_ != nullptr) {
        mission_state_machine_->SetInputs(ground_station_link_->TargetOutput(),
                                          px4_link_->StateOutput(),
                                          health_manager_->Output());
        ground_station_link_->SetMissionStatusInput(
            mission_state_machine_->StatusOutput());
    }

    // 当前正式主程序不绑定Px4Setpoint输入，也不发送控制命令。
    // 后续只有在状态机、控制器和真实拆桨台架门禁全部完成后才允许连接控制Topic。
}

// 启动独立模块。单个模块失败时记录降级并继续尝试其他模块；只要有一个模块
// 成功启动就返回 true，避免视频、遥测等相互独立的数据链路被连带阻断。
bool DroneApplication::Start() {
    if (running_) {
        return true;
    }

    bool any_started = false;
    bool degraded = false;

    if (video_sender_ != nullptr) {
        if (video_sender_->Start()) {
            any_started = true;
        } else {
            degraded = true;
            SPDLOG_ERROR("主程序图传发送启动失败，视频链路降级");
        }
        if (!compositor_->Start()) {
            degraded = true;
            SPDLOG_ERROR("主程序叠加器启动失败，视频链路降级");
        } else {
            any_started = true;
        }
        try {
            if (!detector_->Start()) {
                degraded = true;
                SPDLOG_ERROR("主程序YOLO启动失败，检测链路降级");
            } else {
                any_started = true;
            }
        } catch (const std::exception& error) {
            degraded = true;
            SPDLOG_ERROR("主程序YOLO启动异常: {}", error.what());
        }
        if (!decoder_->Start()) {
            degraded = true;
            SPDLOG_ERROR("主程序视频解码启动失败，视频链路降级");
        } else {
            any_started = true;
        }
        if (!camera_->Start()) {
            degraded = true;
            SPDLOG_ERROR("主程序摄像头接收启动失败，视频链路降级");
        } else {
            any_started = true;
        }
    }

    if (mission_state_machine_ != nullptr) {
        if (!mission_state_machine_->Start()) {
            degraded = true;
            SPDLOG_ERROR("主程序任务状态机启动失败，任务状态回传降级");
        } else {
            any_started = true;
        }
    }

    // 地面站是PX4状态消费者，必须先于PX4生产者启动。
    if (ground_station_link_ != nullptr) {
        if (!ground_station_link_->Start()) {
            degraded = true;
            SPDLOG_ERROR("主程序地面站链路启动失败，地面站遥测下行不可用");
        } else {
            any_started = true;
        }
    }

    if (health_manager_ != nullptr) {
        if (!health_manager_->Start()) {
            degraded = true;
            SPDLOG_ERROR("主程序健康管理启动失败，健康状态汇总不可用");
        } else {
            StartHealthReporter();
        }
    }

    if (px4_link_ != nullptr) {
        if (!px4_link_->Start()) {
            degraded = true;
            SPDLOG_ERROR("主程序PX4链路启动失败，飞行状态保持不可用，控制未启用");
        } else {
            any_started = true;
        }
    }

    running_ = any_started;
    if (!any_started) {
        StopHealthReporter();
        if (health_manager_ != nullptr) {
            health_manager_->Stop();
        }
        SPDLOG_ERROR("主程序没有任何模块成功启动");
        return false;
    }
    SPDLOG_INFO("主程序启动完成: degraded={}", degraded);
    return true;
}

void DroneApplication::StartHealthReporter() {
    if (health_report_running_.exchange(true)) {
        return;
    }
    health_report_thread_ = std::thread([this] { HealthReportLoop(); });
}

void DroneApplication::StopHealthReporter() {
    if (!health_report_running_.exchange(false)) {
        return;
    }
    if (health_report_thread_.joinable()) {
        health_report_thread_.join();
    }
}

void DroneApplication::HealthReportLoop() {
    std::uint64_t last_camera_bytes = 0;
    std::uint64_t last_decoder_frames = 0;
    std::uint64_t last_yolo_frames = 0;
    std::uint64_t last_video_frames = 0;
    std::uint64_t last_px4_messages = 0;
    std::uint64_t last_ground_station_messages = 0;
    std::uint64_t last_camera_errors = 0;
    std::uint64_t last_decoder_errors = 0;
    std::uint64_t last_yolo_errors = 0;
    std::uint64_t last_video_errors = 0;
    std::uint64_t last_px4_errors = 0;
    std::uint64_t last_ground_station_errors = 0;

    const auto now_ms = [] {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
    };

    while (health_report_running_.load()) {
        const std::uint64_t timestamp_ms = now_ms();
        const auto report_if_changed = [this, timestamp_ms](const char* source,
                                                             std::uint64_t value,
                                                             std::uint64_t& last) {
            if (value != last) {
                last = value;
                health_manager_->ReportData(source, timestamp_ms);
            }
        };
        const auto report_error_increase = [this, timestamp_ms](
                                               const char* source,
                                               std::uint64_t value,
                                               std::uint64_t& last) {
            if (value > last) {
                health_manager_->ReportError(source, timestamp_ms);
            }
            last = value;
        };

        if (camera_ != nullptr) {
            report_if_changed(health::source_names::kCamera,
                              camera_->ReceivedBytes(), last_camera_bytes);
            report_error_increase(health::source_names::kCamera,
                                  camera_->ErrorCount(), last_camera_errors);
        }
        if (decoder_ != nullptr) {
            report_if_changed(health::source_names::kVideoDecoder,
                              decoder_->DecodedFrameCount(), last_decoder_frames);
            report_error_increase(health::source_names::kVideoDecoder,
                                  decoder_->ErrorCount(), last_decoder_errors);
        }
        if (detector_ != nullptr) {
            report_if_changed(health::source_names::kYolo,
                              detector_->ProcessedFrameCount(), last_yolo_frames);
            report_error_increase(health::source_names::kYolo,
                                  detector_->ErrorCount(), last_yolo_errors);
        }
        if (video_sender_ != nullptr) {
            report_if_changed(health::source_names::kVideo,
                              video_sender_->SentFrameCount(), last_video_frames);
            report_error_increase(health::source_names::kVideo,
                                  video_sender_->ErrorCount(), last_video_errors);
        }
        if (px4_link_ != nullptr) {
            report_if_changed(health::source_names::kPx4,
                              px4_link_->ReceiveCount(), last_px4_messages);
            report_error_increase(health::source_names::kPx4,
                                  px4_link_->ErrorCount(), last_px4_errors);
        }
        if (ground_station_link_ != nullptr) {
            report_if_changed(health::source_names::kGroundStation,
                              ground_station_link_->ReceiveCount(),
                              last_ground_station_messages);
            report_error_increase(health::source_names::kGroundStation,
                                  ground_station_link_->ErrorCount(),
                                  last_ground_station_errors);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 停止顺序与数据流相反：先停止状态生产者，再停止消费者，最后关闭视频链路。
// Stop 幂等，析构阶段可以安全重复调用。
void DroneApplication::Stop() {
    if (!running_) {
        return;
    }

    SPDLOG_INFO("主程序开始停止");
    // 先停止PX4和视频等数据生产者，避免健康上报线程读取正在析构的组件。
    if (px4_link_ != nullptr) {
        px4_link_->Stop();
    }
    if (camera_ != nullptr) {
        camera_->Stop();
        decoder_->Stop();
        detector_->Stop();
        compositor_->Stop();
        video_sender_->Stop();
    }

    StopHealthReporter();
    if (health_manager_ != nullptr) {
        health_manager_->Stop();
    }

    // 健康生产者停止后再停止其消费者。
    if (mission_state_machine_ != nullptr) {
        mission_state_machine_->Stop();
    }
    if (ground_station_link_ != nullptr) {
        ground_station_link_->Stop();
    }

    running_ = false;
    SPDLOG_INFO("主程序停止完成");
}

bool DroneApplication::IsRunning() const {
    return running_;
}

}  // namespace drone::application
