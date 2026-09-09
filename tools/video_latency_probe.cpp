#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <sys/types.h>
#include <unistd.h>

#include <spdlog/spdlog.h>

#include "application/drone_application.h"
#include "common/logger.h"
#include "config/config.h"

namespace {

std::atomic<bool> g_stop{false};

void OnSignal(int) { g_stop.store(true, std::memory_order_release); }

std::string ExecDir() {
    char buffer[4096];
    const ssize_t length = ::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (length <= 0) return ".";
    buffer[length] = '\0';
    const std::string path(buffer);
    const auto separator = path.find_last_of('/');
    return separator == std::string::npos ? "." : path.substr(0, separator);
}

struct Options {
    std::string config_path;
    int duration_seconds = 60;
    int interval_seconds = 10;
};

Options ParseOptions(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--config" && index + 1 < argc) {
            options.config_path = argv[++index];
        } else if (arg == "--duration" && index + 1 < argc) {
            options.duration_seconds = std::stoi(argv[++index]);
        } else if (arg == "--interval" && index + 1 < argc) {
            options.interval_seconds = std::stoi(argv[++index]);
        } else {
            throw std::invalid_argument(
                "用法: video_latency_probe [--config path] [--duration 秒] [--interval 秒]");
        }
    }
    if (options.duration_seconds <= 0 || options.interval_seconds <= 0) {
        throw std::invalid_argument("duration和interval必须为正数");
    }
    return options;
}

void PrintSummary(const char* name, const drone::common::LatencySummary& value) {
    std::cout << std::left << std::setw(26) << name
              << " count=" << std::setw(8) << value.total_count
              << " win=" << std::setw(5) << value.window_count
              << " avg=" << std::fixed << std::setprecision(2) << std::setw(8)
              << value.average_ms << " p50=" << std::setw(8) << value.p50_ms
              << " p95=" << std::setw(8) << value.p95_ms
              << " p99=" << std::setw(8) << value.p99_ms
              << " max=" << std::setw(8) << value.maximum_ms << " ms\n";
}

const char* CodecName(drone::common::VideoCodec codec) {
    switch (codec) {
        case drone::common::VideoCodec::kH264:
            return "H.264/AVC";
        case drone::common::VideoCodec::kH265:
            return "H.265/HEVC";
        default:
            return "未知";
    }
}

void PrintSnapshot(const drone::application::VideoPipelineLatencySnapshot& s,
                   int elapsed_seconds,
                   const drone::application::VideoPipelineLatencySnapshot& previous,
                   int previous_elapsed_seconds) {
    const int interval_seconds = elapsed_seconds - previous_elapsed_seconds;
    const std::uint64_t interval_frames =
        s.encoded_frame_count >= previous.encoded_frame_count
            ? s.encoded_frame_count - previous.encoded_frame_count
            : 0;
    const std::uint64_t interval_bytes =
        s.encoded_bytes >= previous.encoded_bytes
            ? s.encoded_bytes - previous.encoded_bytes
            : 0;
    const std::uint64_t interval_key_frames =
        s.key_frame_count >= previous.key_frame_count
            ? s.key_frame_count - previous.key_frame_count
            : 0;
    const double interval_fps = interval_seconds > 0
                                    ? static_cast<double>(interval_frames) /
                                          static_cast<double>(interval_seconds)
                                    : 0.0;
    const double interval_mbps = interval_seconds > 0
                                     ? static_cast<double>(interval_bytes) * 8.0 /
                                           static_cast<double>(interval_seconds) / 1000000.0
                                     : 0.0;
    const std::string decode_name =
        std::string("02 ") + CodecName(s.input_codec) + "解码+NV12转存";

    std::cout << "\n========== 机载视频延迟统计 elapsed=" << elapsed_seconds
              << "s（滑动窗口最多2048样本）==========\n"
              << "输入编码=" << CodecName(s.input_codec)
              << " 解码模式=" << (s.hardware_decoder ? "rkmpp硬解" : "软件解码")
              << " 区间输入=" << std::fixed << std::setprecision(2) << interval_fps
              << " FPS/" << interval_mbps << " Mbps"
              << " 区间关键帧=" << interval_key_frames
              << " 累计访问单元=" << s.encoded_frame_count
              << " DRM转存缓冲构建=" << s.hardware_transfer_buffer_build_count << '\n';
    PrintSummary("01 解码输入队列", s.decode_queue);
    PrintSummary(decode_name.c_str(), s.decode);
    PrintSummary("03 入口→解码输出", s.ingress_to_decoded);
    PrintSummary("04 YOLO输入队列", s.yolo_queue);
    PrintSummary("05 YOLO/RKNN推理", s.yolo_inference);
    PrintSummary("06 入口→YOLO完成", s.ingress_to_inference);
    PrintSummary("07 叠加输入队列", s.compositor_queue);
    PrintSummary("08 NV12复制+框文字叠加", s.compositor);
    PrintSummary("09 入口→标注输出", s.ingress_to_annotated);
    PrintSummary("10 编码输入队列", s.sender_queue);
    PrintSummary("11 H264输入帧准备", s.frame_prepare);
    PrintSummary("12 H264编码+RTSP推送调用", s.encode_and_push);
    PrintSummary("13 RTSP单包写入", s.packet_write);
    PrintSummary("14 入口→本地RTSP发布完成", s.ingress_to_rtsp);

    std::cout << "--- 解码细分（最近最多256样本，约10秒）---\n";
    PrintSummary("D1 AVPacket分配+码流复制", s.packet_prepare);
    PrintSummary("D2 avcodec_send_packet", s.send_packet);
    PrintSummary("D3 avcodec_receive_frame", s.receive_frame);
    PrintSummary("D4 DRM硬件帧转存(缓冲复用)", s.hardware_transfer);
    PrintSummary("D5 NV12内存池复制", s.frame_copy);
    std::cout << "说明：D4计数为0表示当前输出不经过av_hwframe_transfer_data；"
                 "不包含摄像头曝光/编码/网络到机载入口，也不包含HM30传输、Web转码和浏览器显示。\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = ParseOptions(argc, argv);
        const std::string executable_directory = ExecDir();
        const std::string config_path = drone::config::ResolveConfigPath(
            executable_directory, options.config_path);
        auto config = drone::config::LoadAppConfig(config_path, executable_directory);
        // 延迟探针只运行视频链路，避免未连接PX4/地面站干扰测试和日志。
        config.runtime.enable_video = true;
        config.runtime.enable_px4 = false;
        config.runtime.enable_ground_station = false;
        config.runtime.enable_control = false;
        // 仅探针启用慢帧关联日志；正式程序默认0，不增加运行期告警。
        config.decoder.slow_frame_threshold_ms = 10.0;

        drone::common::InitializeAsyncLogger(executable_directory + "/logs",
                                              spdlog::level::info);
        std::signal(SIGINT, OnSignal);
        std::signal(SIGTERM, OnSignal);

        drone::application::DroneApplication application(std::move(config));
        if (!application.Start()) {
            std::cerr << "视频延迟探针启动失败\n";
            return 2;
        }

        const auto start = std::chrono::steady_clock::now();
        auto next_report = start + std::chrono::seconds(options.interval_seconds);
        auto previous_snapshot = application.VideoLatencySnapshot();
        int previous_elapsed = 0;
        while (!g_stop.load(std::memory_order_acquire)) {
            const auto now = std::chrono::steady_clock::now();
            const int elapsed = static_cast<int>(
                std::chrono::duration_cast<std::chrono::seconds>(now - start).count());
            if (elapsed >= options.duration_seconds) break;
            if (now >= next_report) {
                auto snapshot = application.VideoLatencySnapshot();
                PrintSnapshot(snapshot, elapsed, previous_snapshot, previous_elapsed);
                previous_snapshot = std::move(snapshot);
                previous_elapsed = elapsed;
                next_report = now + std::chrono::seconds(options.interval_seconds);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        const int elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count());
        PrintSnapshot(application.VideoLatencySnapshot(), elapsed,
                      previous_snapshot, previous_elapsed);
        std::cout.flush();  // 保证最终统计先于MPP停止阶段stderr告警显示
        application.Stop();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "视频延迟探针失败: " << error.what() << '\n';
        return 1;
    }
}
