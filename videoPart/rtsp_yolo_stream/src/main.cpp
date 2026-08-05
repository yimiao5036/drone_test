/**
 * RTSP 拉流 -> YOLO26 目标检测 -> RTSP 推流 流水线程序（RK3588 / 香橙派 5 Plus）
 *
 * 数据流：
 *   RTSP(H.265) --rkmpp硬解码--> cv::Mat(BGR) --YOLO26Detector推理-->
 *   绘制检测框 --h264_rkmpp硬编码--> RTSP 输出
 *
 * 健壮性设计：
 *   - 拉流/推流均挂接 FFmpeg 中断回调，Ctrl+C 或看门狗超时可立即唤醒阻塞 I/O；
 *   - 看门狗线程监视帧间隔，超过阈值判定网络僵死，触发中断；
 *   - 输入/输出任一链路断开后自动重连，7x24 持续运行；
 *   - 每帧统计各阶段耗时，退出时生成性能日志文件。
 *
 * 延迟优化：
 *   - RTP 重排缓冲禁用（reorder_queue_size=0），降低 RTSP 固有延迟；
 *   - 解码器低延迟模式（AV_CODEC_FLAG_LOW_DELAY），减少帧缓存；
 *   - 小 TCP 缓冲 + 小探测数据量，减少启动积压。
 *
 * 用法：
 *   ./rtsp_yolo_stream <输入RTSP地址> <rknn模型路径> <输出RTSP地址> [选项]
 * 选项：
 *   --codec h264|hevc  推流编码器（默认 h264_rkmpp）
 *   --fps N            推流帧率（默认使用输入帧率）
 *   --log <路径>        性能日志输出路径（默认 rtsp_yolo_stream.log）
 * 示例：
 *   ./rtsp_yolo_stream rtsp://192.168.1.100:8554/live ../yolo26n_int8.rknn \
 *                      rtsp://127.0.0.1:8554/detect --fps 20
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "interrupt_flag.h"
#include "perf_stats.h"
#include "rtsp_decoder.h"
#include "rtsp_encoder.h"
#include "yolo26_detector.h"

namespace {

// Ctrl+C 优雅退出标志
volatile std::sig_atomic_t g_running = 1;

// 看门狗参数
constexpr int64_t kWatchdogTimeoutMs = 10000; // 超过 10 秒无新帧判定网络僵死
constexpr int kReconnectDelaySec = 3;         // 重连间隔

void SignalHandler(int) {
    g_running = 0;
    rtsp_stream::g_net_interrupt = true; // 唤醒可能阻塞在网络 I/O 上的主线程
}

// 当前 steady_clock 时间（毫秒）
int64_t NowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// 当前 steady_clock 时间（微秒，用于高精度计时）
int64_t NowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

int64_t g_start_ms = 0;

// 在画面上绘制检测框与标签
void DrawDetections(cv::Mat& frame, const std::vector<Detection>& detections) {
    for (const auto& det : detections) {
        cv::rectangle(frame, cv::Point(det.x1, det.y1), cv::Point(det.x2, det.y2),
                      cv::Scalar(0, 255, 0), 2);

        char label[64];
        snprintf(label, sizeof(label), "id:%d %.2f", det.class_id, det.confidence);
        cv::putText(frame, label, cv::Point(det.x1, det.y1 > 14 ? det.y1 - 6 : 14),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    }
}

void PrintUsage(const char* program) {
    printf("用法: %s <输入RTSP> <模型路径> <输出RTSP> [选项]\n", program);
    printf("  --codec h264|hevc  推流编码器（默认 h264_rkmpp）\n");
    printf("  --fps N            推流帧率（默认使用输入帧率）\n");
    printf("  --log <路径>        性能日志输出路径（默认 rtsp_yolo_stream.log）\n");
}

// 向日志文件写入一行
void LogToFile(FILE* log_file, const char* fmt, ...) {
    if (log_file == nullptr) return;
    va_list args;
    va_start(args, fmt);
    vfprintf(log_file, fmt, args);
    va_end(args);
    fflush(log_file);
}

// 写性能汇总表头
void WritePerfHeader(FILE* f, const char* title) {
    fprintf(f, "\n--- %s ---\n", title);
    fprintf(f, "  %-12s %10s %10s %10s %12s\n",
            "阶段", "平均(ms)", "最大(ms)", "最小(ms)", "总耗时(ms)");
    fprintf(f, "  %-12s %10s %10s %10s %12s\n",
            "----------", "--------", "--------", "--------", "----------");
}

// 写性能汇总一行
void WritePerfLine(FILE* f, const PerfStats& s) {
    if (s.count == 0) {
        fprintf(f, "  %-12s %10s %10s %10s %12s\n",
                s.name, "-", "-", "-", "-");
        return;
    }
    fprintf(f, "  %-12s %10.2f %10.2f %10.2f %12.2f\n",
            s.name, s.avg_ms(), s.max_ms(), s.min_ms(), s.total_ms());
}

} // namespace

// 跨模块共享的中断标志（定义见 interrupt_flag.h）
namespace rtsp_stream {
std::atomic<bool> g_net_interrupt{ false };
std::atomic<int64_t> g_last_frame_ms{ -1 };
} // namespace rtsp_stream

int main(int argc, char* argv[]) {
    if (argc < 4) {
        PrintUsage(argv[0]);
        return -1;
    }

    const std::string input_url = argv[1];
    const std::string model_path = argv[2];
    const std::string output_url = argv[3];

    // 解析可选参数
    std::string codec_name = "h264_rkmpp";
    int fps_override = 0;
    std::string log_path = "rtsp_yolo_stream.log";
    for (int i = 4; i < argc; ++i) {
        if (strcmp(argv[i], "--codec") == 0 && i + 1 < argc) {
            codec_name = std::string(argv[i + 1]) == "hevc" ? "hevc_rkmpp" : "h264_rkmpp";
            ++i;
        } else if (strcmp(argv[i], "--fps") == 0 && i + 1 < argc) {
            fps_override = atoi(argv[i + 1]);
            ++i;
        } else if (strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
            log_path = argv[i + 1];
            ++i;
        }
    }

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    printf("输入: %s\n模型: %s\n输出: %s (编码器 %s)\n日志: %s\n",
           input_url.c_str(), model_path.c_str(), output_url.c_str(),
           codec_name.c_str(), log_path.c_str());

    g_start_ms = NowMs();

    try {
        // 加载 YOLO26 模型（内部完成 RKNN 初始化，只加载一次，重连不重复加载）
        YOLO26Detector detector(model_path);

        // 重连统计
        ReconnectStats reconnect_input, reconnect_output;
        reconnect_input.type = "输入重连";
        reconnect_output.type = "输出重连";
        int watchdog_trigger_count = 0;

        // 看门狗线程：监视帧间隔，超过阈值则触发网络中断，让阻塞的 I/O 返回
        std::thread watchdog([&watchdog_trigger_count]() {
            while (g_running) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                const int64_t last = rtsp_stream::g_last_frame_ms.load();
                if (last < 0) {
                    continue; // 尚未成功读到任何帧，不判定
                }
                if (NowMs() - last > kWatchdogTimeoutMs) {
                    fprintf(stderr, "[看门狗] 超过 %lld ms 无新帧，触发网络中断\n",
                            static_cast<long long>(kWatchdogTimeoutMs));
                    rtsp_stream::g_net_interrupt = true;
                    watchdog_trigger_count++;
                }
            }
        });

        std::unique_ptr<RtspDecoder> decoder;
        std::unique_ptr<RtspEncoder> encoder;
        cv::Mat frame;
        int64_t processed = 0;
        bool first_start = true;

        // 性能统计
        PerfStats stats_decode, stats_detect, stats_draw, stats_encode, stats_total;
        stats_decode.name = "解码";
        stats_detect.name = "推理";
        stats_draw.name = "绘制";
        stats_encode.name = "编码推流";
        stats_total.name = "总帧耗时";

        // 打开日志文件（追加模式，保留历史）
        FILE* log_file = fopen(log_path.c_str(), "w");
        if (log_file == nullptr) {
            fprintf(stderr, "[警告] 无法创建日志文件 %s，将仅输出到终端\n", log_path.c_str());
        } else {
            fprintf(log_file, "=== rtsp_yolo_stream 性能日志 ===\n");
            fprintf(log_file, "输入: %s\n模型: %s\n输出: %s (编码器 %s)\n启动时间: T+0s\n\n",
                    input_url.c_str(), model_path.c_str(), output_url.c_str(),
                    codec_name.c_str());
            fflush(log_file);
        }

        // 主循环：解码 -> 推理 -> 绘制 -> 编码推流，任一链路断开自动重连
        while (g_running) {
            // 输入未就绪（初次启动或输入重连失败）时，重建输入 + 输出
            if (!decoder) {
                if (!g_running) break;

                if (!first_start) {
                    fprintf(stderr, "[重连] %d 秒后重连输入流 %s ...\n", kReconnectDelaySec,
                            input_url.c_str());
                    LogToFile(log_file, "[重连] %d 秒后重连输入流 %s ...\n", kReconnectDelaySec,
                              input_url.c_str());
                    std::this_thread::sleep_for(std::chrono::seconds(kReconnectDelaySec));
                    if (!g_running) break;
                }
                first_start = false;
                rtsp_stream::g_net_interrupt = false;
                rtsp_stream::g_last_frame_ms = -1;
                try {
                    decoder = std::make_unique<RtspDecoder>(input_url);
                    const int fps = fps_override > 0 ? fps_override : decoder->fps();
                    encoder = std::make_unique<RtspEncoder>(output_url, decoder->width(),
                                                            decoder->height(), fps, codec_name);
                    fprintf(stderr, "[重连] 输入/输出链路已重建\n");
                    LogToFile(log_file, "[重连] 输入/输出链路已重建\n");
                    // 记录重连（首次启动不计入重连）
                    if (processed > 0) {
                        reconnect_input.Record(NowMs());
                    }
                } catch (const std::exception& e) {
                    fprintf(stderr, "[重连] 失败: %s\n", e.what());
                    LogToFile(log_file, "[重连] 失败: %s\n", e.what());
                    decoder.reset();
                    encoder.reset();
                }
                continue;
            }

            // 输出未就绪（仅推流失败）时，只重建输出链路，输入保持
            if (!encoder) {
                if (!g_running) break;

                fprintf(stderr, "[重连] %d 秒后重建推流链路 ...\n", kReconnectDelaySec);
                LogToFile(log_file, "[重连] %d 秒后重建推流链路 ...\n", kReconnectDelaySec);
                std::this_thread::sleep_for(std::chrono::seconds(kReconnectDelaySec));
                if (!g_running) break;
                rtsp_stream::g_net_interrupt = false;
                try {
                    const int fps = fps_override > 0 ? fps_override : decoder->fps();
                    encoder = std::make_unique<RtspEncoder>(output_url, decoder->width(),
                                                            decoder->height(), fps, codec_name);
                    fprintf(stderr, "[重连] 推流链路已重建\n");
                    LogToFile(log_file, "[重连] 推流链路已重建\n");
                    reconnect_output.Record(NowMs());
                } catch (const std::exception& e) {
                    fprintf(stderr, "[重连] 失败: %s\n", e.what());
                    LogToFile(log_file, "[重连] 失败: %s\n", e.what());
                    encoder.reset();
                }
                continue;
            }

            // ----- 核心流水线：解码 -> 推理 -> 绘制 -> 编码推流 -----
            int64_t t_frame_start = NowUs();

            // 阶段1: 解码
            int64_t t0 = NowUs();
            if (!decoder->ReadFrame(frame)) {
                fprintf(stderr, "[重连] 输入流读取失败（断开或被中断），准备重连\n");
                LogToFile(log_file, "[重连] 输入流读取失败，准备重连\n");
                decoder.reset();
                encoder.reset();
                rtsp_stream::g_last_frame_ms = -1;
                continue;
            }
            stats_decode.Record(NowUs() - t0);
            rtsp_stream::g_last_frame_ms = NowMs();

            // 阶段2: 推理
            int64_t t1 = NowUs();
            std::vector<Detection> detections = detector.detect(frame);
            stats_detect.Record(NowUs() - t1);

            // 阶段3: 绘制
            int64_t t2 = NowUs();
            DrawDetections(frame, detections);
            stats_draw.Record(NowUs() - t2);

            // 阶段4: 编码推流
            int64_t t3 = NowUs();
            if (!encoder->WriteFrame(frame)) {
                fprintf(stderr, "[重连] 推流失败（写入被中断或对端关闭），准备重连\n");
                LogToFile(log_file, "[重连] 推流失败，准备重连\n");
                encoder.reset();
                rtsp_stream::g_last_frame_ms = -1;
                continue;
            }
            stats_encode.Record(NowUs() - t3);

            // 总帧耗时
            stats_total.Record(NowUs() - t_frame_start);

            processed++;

            // 每 100 帧输出一次性能摘要到终端
            if (processed % 100 == 0) {
                printf("[性能] 已处理 %lld 帧 | 解码 %.1fms | 推理 %.1fms | 绘制 %.1fms | 编码 %.1fms | 合计 %.1fms\n",
                       static_cast<long long>(processed),
                       stats_decode.avg_ms(), stats_detect.avg_ms(),
                       stats_draw.avg_ms(), stats_encode.avg_ms(),
                       stats_total.avg_ms());

                // 也写入日志
                LogToFile(log_file, "[性能 T+%llds] 已处理 %lld 帧 | 解码 %.1fms | 推理 %.1fms | 绘制 %.1fms | 编码 %.1fms | 合计 %.1fms\n",
                          static_cast<long long>((NowMs() - g_start_ms) / 1000),
                          static_cast<long long>(processed),
                          stats_decode.avg_ms(), stats_detect.avg_ms(),
                          stats_draw.avg_ms(), stats_encode.avg_ms(),
                          stats_total.avg_ms());
            }
        }

        // ----- 程序退出，写性能总结到日志文件 -----
        if (log_file) {
            int64_t run_ms = NowMs() - g_start_ms;
            fprintf(log_file, "\n\n=== 运行总结 ===\n");
            fprintf(log_file, "运行时间: %lld 秒\n", static_cast<long long>(run_ms / 1000));
            fprintf(log_file, "处理帧数: %lld\n", static_cast<long long>(processed));
            fprintf(log_file, "平均帧率: %.1f fps\n", run_ms > 0 ? processed * 1000.0 / run_ms : 0.0);

            WritePerfHeader(log_file, "各阶段耗时");
            WritePerfLine(log_file, stats_decode);
            WritePerfLine(log_file, stats_detect);
            WritePerfLine(log_file, stats_draw);
            WritePerfLine(log_file, stats_encode);
            WritePerfLine(log_file, stats_total);

            fprintf(log_file, "\n--- 重连记录 ---\n");
            fprintf(log_file, "  输入重连: %d 次", reconnect_input.count);
            if (reconnect_input.count > 0) {
                fprintf(log_file, " (首次 T+%llds, 末次 T+%llds)",
                        static_cast<long long>((reconnect_input.first_ms - g_start_ms) / 1000),
                        static_cast<long long>((reconnect_input.last_ms - g_start_ms) / 1000));
            }
            fprintf(log_file, "\n");
            fprintf(log_file, "  输出重连: %d 次", reconnect_output.count);
            if (reconnect_output.count > 0) {
                fprintf(log_file, " (首次 T+%llds, 末次 T+%llds)",
                        static_cast<long long>((reconnect_output.first_ms - g_start_ms) / 1000),
                        static_cast<long long>((reconnect_output.last_ms - g_start_ms) / 1000));
            }
            fprintf(log_file, "\n");
            fprintf(log_file, "  看门狗触发: %d 次\n", watchdog_trigger_count);
            fprintf(log_file, "==============================\n");
            fclose(log_file);
            printf("[日志] 性能详情已写入 %s\n", log_path.c_str());
        }

        // 优雅收尾
        if (encoder) {
            encoder->Flush();
        }
        watchdog.join();

        printf("程序退出，共处理 %lld 帧，运行 %lld 秒\n",
               static_cast<long long>(processed),
               static_cast<long long>((NowMs() - g_start_ms) / 1000));
    } catch (const std::exception& e) {
        fprintf(stderr, "初始化失败: %s\n", e.what());
        return -1;
    }

    return 0;
}