#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>  // ssize_t
#include <unistd.h>     // readlink

#include <nlohmann/json.hpp>

#include "common/logger.h"
#include "video/camera_receiver.h"
#include "video/frame_compositor.h"
#include "video/video_decoder.h"
#include "perception/yolo_detector.h"
#include "video_transmission/video_sender.h"
#include "perception/detection_backend.h"  // IDetectionBackend 完整类型（YoloDetector 析构）

namespace {

using nlohmann::json;

// 全局停止标志：Ctrl+C 置位后各成员析构（逆序）停机。
std::atomic<bool> g_stop{false};

void OnSignal(int) { g_stop = true; }

/// 依 /proc/self/exe 定位可执行文件所在目录（Linux 标准做法）。
/// 失败时返回 std::filesystem 替代：取当前工作目录。
std::string ExecDir() {
    char buf[4096];
    const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) {
        return ".";
    }
    buf[n] = '\0';
    std::string path(buf);
    const auto pos = path.find_last_of('/');
    if (pos == std::string::npos) {
        return ".";
    }
    return path.substr(0, pos);
}

/// 候选配置查找：程序目录旁 config.json → 当前工作目录 config.json。
/// 返回实际加载到的路径；全部未找到返回空串。
std::string ResolveConfigPath() {
    const std::string app_dir = ExecDir();
    const std::string candidate1 = app_dir + "/config/config.json";
    const std::string candidate2 = "./config/config.json";
    std::ifstream f1(candidate1);
    if (f1.is_open()) {
        f1.close();
        return candidate1;
    }
    std::ifstream f2(candidate2);
    if (f2.is_open()) {
        f2.close();
        return candidate2;
    }
    return "";
}

/// 加载 JSON 配置（log / yolo / video 配置节）。文件缺失/解析失败时
/// 用默认值兜底并打告警，保证系统可启动。
json LoadConfig() {
    const std::string path = ResolveConfigPath();
    if (path.empty()) {
        std::cerr << "[启动] 未找到配置文件 config/config.json（程序目录旁或当前目录），"
                     "使用内置默认值"
                  << std::endl;
        return json::object();
    }
    try {
        std::ifstream f(path);
        return json::parse(f);
    } catch (const std::exception& e) {
        std::cerr << "[启动] 配置文件解析失败: " << e.what() << "，使用内置默认值"
                  << std::endl;
        return json::object();
    }
}

// 封装 YoloDetector::Start：失败不抛异常（部署初 RKNN 环境不齐时便于排查）。
/// 返回 true 表示 YOLO 正常启动。
bool YoloStart(drone::perception::YoloDetector* detector) {
    try {
        return detector->Start();
    } catch (const std::exception& e) {
        SPDLOG_ERROR("YOLO 检测器启动异常: {}", e.what());
        return false;
    }
}

}  // namespace

int main() {
    // 1. 加载配置
    const json config = LoadConfig();

    // 2. 初始化全局异步文件日志
    {
        const json log_cfg = config.value("log", json::object());
        std::string log_dir = log_cfg.value("dir", std::string("logs/"));
        // 日志目录相对程序目录解析，保证任意位置可运行
        if (log_dir.find('/') != 0 && log_dir != "logs/") {
            log_dir = ExecDir() + "/" + log_dir;
        }
        const auto level =
            spdlog::level::from_str(log_cfg.value("level", std::string("info")));
        drone::common::InitializeAsyncLogger(log_dir, level);
        SPDLOG_INFO("已加载日志配置: 目录={} 等级={}", log_dir,
                    spdlog::level::to_string_view(level));
    }

    // 3. 解析视频链路配置
    const json video_cfg = config.value("video", json::object());
    const std::string input_url =
        video_cfg.value("input_rtsp", std::string("rtsp://192.168.1.100:8554/live"));
    const std::string output_url =
        video_cfg.value("output_rtsp", std::string("rtsp://127.0.0.1:8554/drone_out"));
    const int fps = video_cfg.value("fps", 25);
    const std::int64_t bitrate = video_cfg.value("bitrate", 6LL * 1024 * 1024);
    // 编码尺寸需在启动前确定（与摄像头分辨率一致），配置缺省时按 1080p。
    const int enc_w = video_cfg.value("width", 1920);
    const int enc_h = video_cfg.value("height", 1080);

    const json yolo_cfg = config.value("yolo", json::object());
    // 模型路径相对程序目录解析
    std::string model_path = yolo_cfg.value("model_path", std::string("models/yolo26n_int8.rknn"));
    if (!model_path.empty() && model_path.find_first_of('/') != 0) {
        model_path = ExecDir() + "/" + model_path;
    }
    const float conf_threshold = yolo_cfg.value("conf_threshold", 0.25f);
    const float nms_threshold = yolo_cfg.value("nms_threshold", 0.45f);
    const std::uint64_t max_detection_frame_lag =
        yolo_cfg.value("max_detection_frame_lag", 10ULL);
    std::vector<std::string> class_names{"UAV", "OBS"};
    const auto class_names_it = yolo_cfg.find("class_names");
    if (class_names_it != yolo_cfg.end()) {
        if (class_names_it->is_array()) {
            std::vector<std::string> configured_names;
            configured_names.reserve(class_names_it->size());
            bool valid = true;
            for (const auto& item : *class_names_it) {
                if (!item.is_string()) {
                    valid = false;
                    break;
                }
                configured_names.push_back(item.get<std::string>());
            }
            if (valid && !configured_names.empty()) {
                class_names = std::move(configured_names);
            } else {
                SPDLOG_WARN("YOLO class_names 必须是非空字符串数组，使用默认类别名称");
            }
        } else {
            SPDLOG_WARN("YOLO class_names 不是数组，使用默认类别名称");
        }
    }

    SPDLOG_INFO("视频链路配置: 输入={} 输出={} 帧率={} 码率={}", input_url, output_url,
                fps, bitrate);
    SPDLOG_INFO("YOLO 配置: 模型={} conf={} nms={} 类别名称数={} 旧框保留={}帧",
                model_path, conf_threshold, nms_threshold, class_names.size(),
                max_detection_frame_lag);

    // 4. 装配视频链路（依赖注入绑定输入、输出主题）
    //    摄像头拉流 → 解码 → YOLO 识别 → 叠加 → 编码推流
    auto camera = std::make_unique<drone::video::CameraReceiver>(
        drone::video::CameraReceiverConfig{input_url, "tcp",
                                           std::chrono::milliseconds(5000),
                                           std::chrono::milliseconds(3000)});

    drone::video::VideoDecoderConfig decoder_cfg;
    // 解码帧内存池容量：≥ 各订阅队列容量之和 + 在途帧数。
    // 订阅者：YOLO(2) + 叠加(4) + 图传(2)；另曾在途若干，取 16 稳健。
    decoder_cfg.pool_capacity = 16;
    decoder_cfg.stride_alignment = 64;
    decoder_cfg.prefer_hardware = true;  // 香橙派上 rkmpp 硬解
    auto decoder = std::make_unique<drone::video::VideoDecoder>(decoder_cfg);

    drone::perception::YoloDetectorConfig yolo;
    yolo.model_path = model_path;
    yolo.conf_threshold = conf_threshold;
    yolo.nms_threshold = nms_threshold;
    auto detector = std::make_unique<drone::perception::YoloDetector>(yolo);

    drone::video::CompositorConfig compose_cfg;
    compose_cfg.pool_capacity = 8;
    compose_cfg.draw_text = true;
    compose_cfg.class_names = class_names;
    compose_cfg.max_detection_frame_lag = max_detection_frame_lag;
    auto compositor = std::make_unique<drone::video::FrameCompositor>(compose_cfg);

    drone::video_transmission::VideoSenderConfig sender_cfg;
    sender_cfg.encode.url = output_url;
    sender_cfg.encode.codec = "h264";          // rkmpp 硬编（香橙派）或软编回退
    sender_cfg.encode.prefer_hardware = true;  // 优先 rkmpp 硬编码
    sender_cfg.encode.width = static_cast<std::uint32_t>(enc_w);
    sender_cfg.encode.height = static_cast<std::uint32_t>(enc_h);
    sender_cfg.encode.fps = fps;
    sender_cfg.encode.bitrate = bitrate;
    sender_cfg.encode.gop = fps * 2;
    auto sender = std::make_unique<drone::video_transmission::VideoSender>(
        std::move(sender_cfg));

    // 5. 接线（直接订阅各部件真实输出 Topic，避免装配到无生产者的孤立 Topic）
    decoder->SetInput(camera->StreamOutput());
    detector->SetInput(decoder->FrameOutput());
    compositor->SetDecodedInput(decoder->FrameOutput());
    compositor->SetDetectionInput(detector->DetectionOutput());
    sender->SetInput(compositor->AnnotatedOutput());

    // 6. 注册 Ctrl+C / SIGTERM 停机
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    // 7. 按依赖顺序启动（逆序停机：析构顺序与启动相反）
    if (!sender->Start()) {
        SPDLOG_ERROR("图传发送启动失败");
    }
    // 注意：下面各部件启动若失败，仅记录 ERROR 而不整体退出，
    // 便于现场排查（RTSP 源不可达等常见于部署初期）。
    if (!compositor->Start()) {
        SPDLOG_ERROR("叠加器启动失败");
    }
    // YOLO 依赖 RKNN 后端（DRONE_HAVE_RKNN=ON 且在香橙派），失败会返回 false
    std::atomic<bool> yolo_ok{false};
    if (YoloStart(detector.get())) {
        yolo_ok = true;
    }
    if (!decoder->Start()) {
        SPDLOG_ERROR("视频解码器启动失败");
    }
    if (!camera->Start()) {
        SPDLOG_ERROR("摄像头接收启动失败");
    }

    SPDLOG_INFO("视频链路已启动，按 Ctrl+C 停止");
    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 8. 逆序停机：stop 各线程（shared_ptr 在 main 作用域末尾按声明逆序析构）
    SPDLOG_INFO("收到停止信号，正在停机...");
    camera->Stop();
    decoder->Stop();
    if (yolo_ok) {
        detector->Stop();
    }
    compositor->Stop();
    sender->Stop();
    SPDLOG_INFO("已停机，再见");
    return 0;
}
