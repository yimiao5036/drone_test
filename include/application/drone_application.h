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
namespace drone::control {
class FlightController;
class VisualTrackingShadow;
}
namespace drone::perception {
class StereoRanger;
class TargetEstimator;
class VisualTargetMonitor;
class YoloDetector;
}
namespace drone::state_machine {
class MissionStateMachine;
}
namespace drone::video {
class ICameraReceiver;
class FrameCompositor;
class StereoFrameSplitter;
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
    std::uint64_t hardware_transfer_buffer_build_count = 0;
    std::uint64_t rga_dma_transfer_count = 0;
    std::uint64_t rga_dma_fallback_count = 0;
    common::LatencySummary decode_queue;
    common::LatencySummary decode;
    common::LatencySummary ingress_to_decoded;
    // 最近最多256帧的解码细分，专用于定位rkmpp长尾。
    common::LatencySummary packet_prepare;
    common::LatencySummary send_packet;
    common::LatencySummary receive_frame;
    common::LatencySummary hardware_transfer_prepare;
    common::LatencySummary hardware_transfer;
    common::LatencySummary rga_dma_transfer;
    common::LatencySummary frame_copy;
    common::LatencySummary yolo_queue;
    common::LatencySummary yolo_inference;
    common::LatencySummary ingress_to_inference;
    common::LatencySummary yolo_preprocess_total;
    common::LatencySummary yolo_rga_resize_color;
    common::LatencySummary yolo_letterbox_copy;
    common::LatencySummary yolo_npu_run;
    common::LatencySummary yolo_npu_internal_run;
    common::LatencySummary yolo_npu_wall_overhead;
    common::LatencySummary yolo_npu_perf_query;
    common::LatencySummary yolo_output_layout;
    common::LatencySummary yolo_postprocess;
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

    /// 控制面装配诊断快照（阶段四装配门禁断言与排查用）。
    /// enable_control=false：控制器已创建并接入状态机意图（影子运行，
    /// 授权链因未注入 PX4 快照源恒不通过），但不绑定 Px4Setpoint、不注入命令接口；
    /// enable_control=true：四个字段全 true。
    struct ControlAssemblySnapshot {
        bool flight_controller_created = false;    ///< 真实 FlightController 已创建
        bool intent_input_bound = false;           ///< 意图输入已接 MissionStateMachine::IntentOutput()
        bool px4_snapshot_source_injected = false; ///< SetPx4LinkSource 已注入（仅 enable_control=true）
        bool setpoint_bound_to_px4 = false;        ///< Px4Link::SetInput 已绑定（仅 enable_control=true）
    };
    /// 装配完成后（构造返回后）的控制面接线快照。
    ControlAssemblySnapshot ControlAssembly() const { return control_assembly_; }

private:
    void BuildComponents();
    void BindTopics();
    void RegisterHealthSources();
    void StartHealthReporter();
    void StopHealthReporter();
    void HealthReportLoop();

    config::AppConfig config_;
    bool running_ = false;

    std::unique_ptr<video::ICameraReceiver> camera_;
    std::unique_ptr<video::VideoDecoder> decoder_;
    std::unique_ptr<video::StereoFrameSplitter> stereo_splitter_;
    std::unique_ptr<perception::YoloDetector> detector_;
    std::unique_ptr<perception::StereoRanger> stereo_ranger_;   ///< 双目测距影子
    std::unique_ptr<perception::VisualTargetMonitor> visual_monitor_;
    std::unique_ptr<control::VisualTrackingShadow> visual_tracking_shadow_;
    std::unique_ptr<video::FrameCompositor> compositor_;
    std::unique_ptr<video_transmission::VideoSender> video_sender_;
    std::unique_ptr<communication::Px4Link> px4_link_;
    std::unique_ptr<communication::GroundStationLink> ground_station_link_;
    std::unique_ptr<perception::TargetEstimator> target_estimator_;
    std::unique_ptr<state_machine::MissionStateMachine> mission_state_machine_;
    std::unique_ptr<control::FlightController> flight_controller_;  ///< 阶段四正式实现，影子/正式由 enable_control 门禁
    std::unique_ptr<health::HealthManager> health_manager_;

    ControlAssemblySnapshot control_assembly_;

    std::atomic<bool> health_report_running_{false};
    std::thread health_report_thread_;
};

}  // namespace drone::application
