/**
 * @file yolo_detector.cpp
 * @brief YOLO 检测器实现（YoloDetector）
 *
 * 订阅解码帧（video::FrameHandle，NV12），经推理后端逐帧检测，
 * 将结果转换为 common::DetectionResult 发布。
 *
 * 设计要点：
 * - PIMPL 隔离推理后端细节，接口头文件不依赖 RKNN/RGA。
 * - 推理后端经 IDetectionBackend 依赖注入：香橙派部署默认使用
 *   RknnDetectionBackend（DRONE_HAVE_RKNN 编译时创建），开发机测试
 *   注入 Mock 后端；两者都不可用时 Start() 返回 false 并打 ERROR 日志，
 *   不静默降级为空转。
 * - 检测线程与 Topic 订阅解耦：SetInput 绑定订阅，Stop 通过 Reset 订阅
 *   唤醒等待并 join，确定性停机。
 * - 推理平均耗时用指数滑动平均（EMA, alpha=0.1），对长时间运行友好。
 */
#include "perception/yolo_detector.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "perception/detection_backend.h"

#ifdef DRONE_HAVE_RKNN
#include "perception/rknn_detection_backend.h"
#endif

#include <spdlog/spdlog.h>

namespace drone::perception {

namespace {

/// 异常日志节流：第 1 次与每满 100 次才打印，避免高频异常刷屏。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

/// 单调时钟当前毫秒。
std::int64_t SteadyNowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

// 默认后端工厂（条件编译）：
// - DRONE_HAVE_RKNN 且模型路径非空 → RKNN 后端（香橙派）
// - 否则返回 nullptr，由调用方注入或 Start 失败
std::unique_ptr<IDetectionBackend> CreateDefaultDetectionBackend(
    const std::string& model_path, float conf_threshold, float nms_threshold) {
#ifdef DRONE_HAVE_RKNN
    if (!model_path.empty()) {
        return std::make_unique<RknnDetectionBackend>(model_path, conf_threshold,
                                                      nms_threshold);
    }
#else
    (void)model_path;
    (void)conf_threshold;
    (void)nms_threshold;
#endif
    return nullptr;
}

/// YoloDetector 实现细节（PIMPL）：推理后端、订阅、检测线程与统计。
struct YoloDetector::Impl {
    explicit Impl(YoloDetectorConfig config, std::unique_ptr<IDetectionBackend> backend)
        : config(std::move(config)), backend(std::move(backend)) {
        if (this->config.input_queue_capacity == 0) {
            throw std::invalid_argument("YOLO 解码帧订阅队列容量必须大于 0");
        }
        if (this->config.conf_threshold < 0.f || this->config.conf_threshold > 1.f) {
            throw std::invalid_argument("YOLO 置信度阈值必须在 [0,1]");
        }
        if (this->config.nms_threshold < 0.f || this->config.nms_threshold > 1.f) {
            throw std::invalid_argument("YOLO NMS 阈值必须在 [0,1]");
        }
    }

    ~Impl() {
        Stop();
        if (backend != nullptr) {
            backend->Unload();
        }
    }

    YoloDetectorConfig config;
    std::unique_ptr<IDetectionBackend> backend;
    std::atomic<bool> stop_requested{false};
    std::thread thread;
    bool load_failed = false;  // 后端加载失败后不再重复尝试（防刷屏）

    common::Topic<video::FrameHandle>::Subscription input_sub;
    common::Topic<video::FrameHandle>* input_topic = nullptr;  // SetInput 记录，重启时重新订阅
    common::Topic<common::DetectionResult> detection_output;

    // 统计
    std::atomic<uint64_t> processed_count{0};
    std::atomic<uint64_t> error_count{0};
    std::atomic<uint64_t> sequence{0};
    std::atomic<float> avg_inference_ms{0.f};

    /// 更新推理平均耗时（EMA）。
    void UpdateAvg(float elapsed_ms) {
        constexpr float kAlpha = 0.1f;
        float current = avg_inference_ms.load();
        float next = current == 0.f ? elapsed_ms
                                    : current * (1.f - kAlpha) + elapsed_ms * kAlpha;
        avg_inference_ms.store(next);
    }

    /// 检测线程主循环。
    void DetectLoop() {
        while (!stop_requested.load()) {
            auto message = input_sub.WaitTakeFor(std::chrono::milliseconds(100));
            if (!message) {
                continue;  // 超时或主题关闭；循环顶检查停止标志
            }
            const auto& frame = **message;

            // 帧元数据校验（Topic 只发布有效句柄，防御性检查）
            if (!frame.Info().Valid()) {
                const uint64_t errors = error_count.fetch_add(1) + 1;
                if (ShouldLogThrottled(errors)) {
                    SPDLOG_WARN("YOLO 检测收到无效帧（元数据缺失）跳过，累计 {}",
                                errors);
                }
                continue;
            }

            // 推理
            std::vector<BackendDetection> detections;
            const auto start = std::chrono::steady_clock::now();
            try {
                detections = backend->Detect(frame);
            } catch (const std::exception& e) {
                const uint64_t errors = error_count.fetch_add(1) + 1;
                if (ShouldLogThrottled(errors)) {
                    SPDLOG_ERROR("YOLO 推理异常: {}，累计 {}", e.what(), errors);
                }
                continue;
            }
            const auto end = std::chrono::steady_clock::now();
            const float elapsed_ms =
                std::chrono::duration<float, std::milli>(end - start).count();
            UpdateAvg(elapsed_ms);
            ++processed_count;

            // 发布检测结果：每个目标一条消息，共享帧序号与推理耗时
            const std::uint64_t frame_sequence = frame.Info().sequence;
            const std::int64_t source_time_ms = frame.Info().timestamp_ms;
            for (const auto& d : detections) {
                common::DetectionResult result;
                result.header.sequence = sequence.fetch_add(1) + 1;
                result.header.source_time_ms =
                    static_cast<std::uint64_t>(source_time_ms);
                result.header.receive_time_ms =
                    static_cast<std::uint64_t>(SteadyNowMs());
                result.frame_sequence = frame_sequence;
                result.class_id = static_cast<std::uint32_t>(d.class_id);
                result.confidence = d.confidence;
                result.bbox_x = d.x1;
                result.bbox_y = d.y1;
                result.bbox_w = d.x2 - d.x1;
                result.bbox_h = d.y2 - d.y1;
                result.center_pixel_x = (d.x1 + d.x2) * 0.5f;
                result.center_pixel_y = (d.y1 + d.y2) * 0.5f;
                result.inference_time_ms = elapsed_ms;
                (void)detection_output.Emplace(std::move(result));
            }
        }
    }

    void Stop() {
        if (!thread.joinable()) {
            return;
        }
        stop_requested = true;
        input_sub.Reset();  // 唤醒 WaitTakeFor 中的等待
        thread.join();
    }
};

YoloDetector::YoloDetector(YoloDetectorConfig config,
                           std::unique_ptr<IDetectionBackend> backend) {
    if (backend == nullptr) {
        backend = CreateDefaultDetectionBackend(config.model_path,
                                                config.conf_threshold,
                                                config.nms_threshold);
    }
    impl_ = std::make_unique<Impl>(std::move(config), std::move(backend));
    SPDLOG_INFO("YOLO 检测器创建: 模型={} 置信度阈值={} NMS阈值={} 订阅队列={} 后端={}",
                impl_->config.model_path.empty() ? "(注入后端)" : impl_->config.model_path,
                impl_->config.conf_threshold, impl_->config.nms_threshold,
                impl_->config.input_queue_capacity,
                impl_->backend != nullptr ? "已配置" : "缺失(启动将失败)");
}

YoloDetector::~YoloDetector() {
    if (impl_ != nullptr) {
        impl_->Stop();
    }
    SPDLOG_INFO("YOLO 检测器销毁");
}

bool YoloDetector::Start() {
    if (impl_->thread.joinable()) {
        return true;  // 已启动，幂等
    }
    if (impl_->backend == nullptr) {
        ++impl_->error_count;
        SPDLOG_ERROR("YOLO 检测器启动失败: 无推理后端（未启用 RKNN 编译或未注入后端）");
        return false;
    }
    if (impl_->load_failed) {
        ++impl_->error_count;
        SPDLOG_ERROR("YOLO 检测器启动失败: 后端加载此前已失败，不再重试");
        return false;
    }
    if (!impl_->backend->Load()) {
        impl_->load_failed = true;
        ++impl_->error_count;
        SPDLOG_ERROR("YOLO 检测器启动失败: 后端加载失败，模型路径={}",
                     impl_->config.model_path.empty() ? "(注入后端)" : impl_->config.model_path);
        return false;
    }

    // Stop 会 Reset 输入订阅，重启时重新订阅（幂等：已打开则不重复）
    if (!impl_->input_sub.IsOpen() && impl_->input_topic != nullptr) {
        impl_->input_sub =
            impl_->input_topic->Subscribe(impl_->config.input_queue_capacity);
    }

    impl_->stop_requested = false;
    impl_->thread = std::thread(&Impl::DetectLoop, impl_.get());
    SPDLOG_INFO("YOLO 检测器启动");
    return true;
}

void YoloDetector::Stop() {
    impl_->Stop();
    SPDLOG_INFO("YOLO 检测器停止");
}

bool YoloDetector::IsRunning() const {
    return impl_->thread.joinable();
}

void YoloDetector::SetInput(common::Topic<video::FrameHandle>& input) {
    // 队列容量默认 2（丢最旧）：解码 25fps，推理慢时只处理最新帧，
    // 与 docs/数据接口文档.md kDecodedFrame 建议一致
    impl_->input_topic = &input;
    impl_->input_sub = input.Subscribe(impl_->config.input_queue_capacity);
}

common::Topic<common::DetectionResult>& YoloDetector::DetectionOutput() {
    return impl_->detection_output;
}

uint64_t YoloDetector::ProcessedFrameCount() const {
    return impl_->processed_count.load();
}

float YoloDetector::InferenceTimeMsAvg() const {
    return impl_->avg_inference_ms.load();
}

uint64_t YoloDetector::ErrorCount() const {
    return impl_->error_count.load();
}

}  // namespace drone::perception
