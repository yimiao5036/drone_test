#include "application/drone_application.h"

#include <exception>
#include <memory>
#include <utility>

#include <spdlog/spdlog.h>

#include "communication/ground_station_link.h"
#include "communication/px4_link.h"
#include "perception/detection_backend.h"
#include "perception/yolo_detector.h"
#include "video/camera_receiver.h"
#include "video/frame_compositor.h"
#include "video/video_decoder.h"
#include "video_transmission/video_sender.h"

namespace drone::application {

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
}

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

    // 当前正式主程序不绑定Px4Setpoint输入，也不发送控制命令。
    // 后续只有在状态机、控制器和真实拆桨台架门禁全部完成后才允许连接控制Topic。
}

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

    // 地面站是PX4状态消费者，必须先于PX4生产者启动。
    if (ground_station_link_ != nullptr) {
        if (!ground_station_link_->Start()) {
            degraded = true;
            SPDLOG_ERROR("主程序地面站链路启动失败，地面站遥测下行不可用");
        } else {
            any_started = true;
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
        SPDLOG_ERROR("主程序没有任何模块成功启动");
        return false;
    }
    SPDLOG_INFO("主程序启动完成: degraded={}", degraded);
    return true;
}

void DroneApplication::Stop() {
    if (!running_) {
        return;
    }

    SPDLOG_INFO("主程序开始停止");
    // 先停止PX4状态生产者，再停止地面站消费者。
    if (px4_link_ != nullptr) {
        px4_link_->Stop();
    }
    if (ground_station_link_ != nullptr) {
        ground_station_link_->Stop();
    }

    if (camera_ != nullptr) {
        camera_->Stop();
        decoder_->Stop();
        detector_->Stop();
        compositor_->Stop();
        video_sender_->Stop();
    }

    running_ = false;
    SPDLOG_INFO("主程序停止完成");
}

bool DroneApplication::IsRunning() const {
    return running_;
}

}  // namespace drone::application
