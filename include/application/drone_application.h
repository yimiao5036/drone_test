#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include "common/latency_statistics.h"
#include "common/topic.h"
#include "common/types.h"
#include "config/config.h"
#include "health/health_manager.h"

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

struct VideoPipelineLatencySnapshot {
    common::VideoCodec input_codec = common::VideoCodec::kUnknown;
    bool hardware_decoder = false;
    std::uint64_t encoded_frame_count = 0;
    std::uint64_t encoded_bytes = 0;
    std::uint64_t key_frame_count = 0;
    common::LatencySummary decode_queue;
    common::LatencySummary decode;
    common::LatencySummary ingress_to_decoded;
    // 最近最多256帧的解码细分，专用于定位rkmpp长尾。
    common::LatencySummary packet_prepare;
    common::LatencySummary send_packet;
    common::LatencySummary receive_frame;
    common::LatencySummary hardware_transfer;
    common::LatencySummary frame_copy;
    common::LatencySummary yolo_queue;
    common::LatencySummary yolo_inference;
    common::LatencySummary ingress_to_inference;
    common::LatencySummary compositor_queue;
    common::LatencySummary compositor;
    common::LatencySummary ingress_to_annotated;
    common::LatencySummary sender_queue;
    common::LatencySummary frame_prepare;
    common::LatencySummary encode_and_push;
    common::LatencySummary packet_write;
    common::LatencySummary ingress_to_rtsp;
};

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
    VideoPipelineLatencySnapshot VideoLatencySnapshot() const;

private:
    void BuildComponents();
    void BindTopics();
    void RegisterHealthSources();
    void StartHealthReporter();
    void StopHealthReporter();
    void HealthReportLoop();

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
    std::unique_ptr<health::HealthManager> health_manager_;

    std::atomic<bool> health_report_running_{false};
    std::thread health_report_thread_;
};

}  // namespace drone::application
