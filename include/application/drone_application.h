#pragma once

#include <memory>

#include "common/topic.h"
#include "common/types.h"
#include "config/config.h"

namespace drone::communication {
class GroundStationLink;
class Px4Link;
}
namespace drone::perception {
class YoloDetector;
}
namespace drone::state_machine {
class MissionStateMachine;
}
namespace drone::video {
class CameraReceiver;
class FrameCompositor;
class VideoDecoder;
}
namespace drone::video_transmission {
class VideoSender;
}

namespace drone::application {

/// 正式进程的组件所有者与装配根。main.cpp只负责配置、信号和进程生命周期。
class DroneApplication final {
public:
    explicit DroneApplication(config::AppConfig config);
    ~DroneApplication();

    DroneApplication(const DroneApplication&) = delete;
    DroneApplication& operator=(const DroneApplication&) = delete;

    /// 启动已配置的数据链路；单模块失败时记录降级并继续启动其他模块。
    bool Start();
    /// 按数据流逆序停止全部模块；幂等。
    void Stop();
    bool IsRunning() const;

private:
    void BuildComponents();
    void BindTopics();

    config::AppConfig config_;
    bool running_ = false;

    std::unique_ptr<video::CameraReceiver> camera_;
    std::unique_ptr<video::VideoDecoder> decoder_;
    std::unique_ptr<perception::YoloDetector> detector_;
    std::unique_ptr<video::FrameCompositor> compositor_;
    std::unique_ptr<video_transmission::VideoSender> video_sender_;
    std::unique_ptr<communication::Px4Link> px4_link_;
    std::unique_ptr<communication::GroundStationLink> ground_station_link_;
    std::unique_ptr<state_machine::MissionStateMachine> mission_state_machine_;

    // 健康管理器尚未接入正式实现，先保留一个主程序级Topic作为状态机输入占位。
    // 后续接入 HealthManager 后替换为真实 HealthStatus 发布源。
    common::Topic<common::HealthStatus> health_status_topic_;
};

}  // namespace drone::application
