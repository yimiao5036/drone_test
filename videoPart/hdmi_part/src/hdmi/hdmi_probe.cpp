/**
 * @file hdmi_probe.cpp
 * @brief HDMI 直显带外自检工具（不依赖视频链路与 FFmpeg）
 *
 * 用途：在香橙派上分步验证 HDMI 输出链路，把"图传不亮"问题拆成可独立判断的三步，
 * 避免在整条视频链路里瞎猜。
 *
 * 三个子命令：
 * - `modes`：只读枚举 DRM 资源与模式列表，并给出目标模式的匹配结论。
 *   不获取 master、不做 modeset，可在正式程序运行前安全执行。
 *   这是排查"EDID 是否提供 720p30"的第一手段。
 * - `test`：生成 NV12 彩条并连续上屏，验证 DRM/GBM/EGL/GL 全链路与页面翻转。
 * - `nv12`：把裸 NV12 文件连续上屏，用于验证真实帧的 stride 与色彩是否正确。
 *
 * 彩条画面附一根随帧号移动的白色竖线：竖线在动说明上屏在持续推进（而非停在
 * 冻结帧），这是判断"图传发射机画面静止"最直接的依据。
 *
 * 日志刻意输出到控制台而不是走 drone/common 的异步文件日志：本工具是上板排查
 * 手段，需要即时可见的输出；正式程序仍使用项目统一日志。
 *
 * 构建与用法见同目录 hdmi_display_backend.md。
 */
#include "hdmi/hdmi_display_backend.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace {

using drone::video_transmission::HdmiColorSpace;
using drone::video_transmission::HdmiDisplayConfig;
using drone::video_transmission::HdmiSubmitResult;
using drone::video_transmission::IHdmiDisplay;

/// 命令行选项。
struct Options {
    std::string command;                 ///< modes / test / nv12
    std::string device = "/dev/dri/card0";
    std::string connector;               ///< 空 = 自动选择已连接连接器
    std::string nv12_path;               ///< nv12 子命令的输入文件
    std::uint32_t width = 1280;
    std::uint32_t height = 720;
    std::uint32_t stride = 0;            ///< 0 表示等于 width
    int refresh_hz = 30;
    int frames = 90;
    bool bt601 = false;
    bool full_range = false;
    bool strict = false;
    bool blank_on_stop = true;
};

void PrintUsage(const char* program) {
    std::cout
        << "HDMI 直显带外自检工具\n\n"
        << "用法:\n"
        << "  " << program << " modes [选项]            枚举 DRM 资源与模式匹配结论（只读）\n"
        << "  " << program << " test  [选项]            显示 NV12 彩条若干帧后退出\n"
        << "  " << program << " nv12 <文件> [选项]      显示裸 NV12 文件若干帧后退出\n\n"
        << "选项:\n"
        << "  --device <节点>      DRM 设备节点，默认 /dev/dri/card0\n"
        << "  --connector <名称>   连接器名称，如 HDMI-A-1；默认自动选已连接者\n"
        << "  --width <像素>       输出宽，默认 1280\n"
        << "  --height <像素>      输出高，默认 720\n"
        << "  --refresh <Hz>       输出刷新率，默认 30\n"
        << "  --stride <像素>      nv12 模式源行 stride，默认等于 width\n"
        << "  --frames <帧数>      test/nv12 上屏帧数，默认 90\n"
        << "  --bt601              使用 BT.601 色彩矩阵（默认 BT.709）\n"
        << "  --full-range         使用全范围（默认有限范围 16-235）\n"
        << "  --strict             严格模式：只接受精确匹配或合成的输出模式\n"
        << "  --keep-on-stop       停止时不点黑场，恢复进入前的 CRTC 状态\n"
        << "  -h, --help           显示本帮助\n\n"
        << "注意：modes 查询 /dev/dri/card0 需要 root 或 CAP_SYS_ADMIN。\n"
        << "      test/nv12 会独占 DRM master 并改变 HDMI 输出，请先停止正式程序。\n";
}

/// 参数解析结果：区分"可执行""用户请求帮助""参数错误"，
/// 使 main 能给出正确的进程退出码（供脚本/CI 判定）。
enum class ParseOutcome {
    kOk = 0,   ///< 参数有效，可以执行
    kHelp,     ///< 已打印帮助，属正常退出
    kError,    ///< 参数错误
};

/// 解析命令行。
ParseOutcome ParseArgs(int argc, char** argv, Options* out) {
    if (argc < 2) {
        PrintUsage(argv[0]);
        return ParseOutcome::kError;  // 缺少子命令
    }
    out->command = argv[1];
    if (out->command == "-h" || out->command == "--help") {
        PrintUsage(argv[0]);
        return ParseOutcome::kHelp;
    }
    if (out->command != "modes" && out->command != "test" && out->command != "nv12") {
        std::cerr << "未知子命令: " << out->command << "\n\n";
        PrintUsage(argv[0]);
        return ParseOutcome::kError;
    }

    const auto need_value = [&](int& i) -> const char* {
        if (i + 1 >= argc) {
            std::cerr << "选项 " << argv[i] << " 缺少参数值\n";
            return nullptr;
        }
        return argv[++i];
    };

    // 数字参数解析：std::stoul/std::stoi 在非法输入时会抛异常，这里统一收敛为错误信息。
    const auto parse_uint = [](const char* text, std::uint32_t* out) -> bool {
        try {
            *out = static_cast<std::uint32_t>(std::stoul(text));
            return true;
        } catch (const std::exception&) {
            std::cerr << "无符号数字参数非法: " << text << "\n";
            return false;
        }
    };
    const auto parse_int = [](const char* text, int* out) -> bool {
        try {
            *out = std::stoi(text);
            return true;
        } catch (const std::exception&) {
            std::cerr << "整数参数非法: " << text << "\n";
            return false;
        }
    };

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            PrintUsage(argv[0]);
            return ParseOutcome::kHelp;
        }
        if (arg == "--bt601") {
            out->bt601 = true;
        } else if (arg == "--full-range") {
            out->full_range = true;
        } else if (arg == "--strict") {
            out->strict = true;
        } else if (arg == "--keep-on-stop") {
            out->blank_on_stop = false;
        } else if (arg == "--device") {
            const char* value = need_value(i);
            if (value == nullptr) return ParseOutcome::kError;
            out->device = value;
        } else if (arg == "--connector") {
            const char* value = need_value(i);
            if (value == nullptr) return ParseOutcome::kError;
            out->connector = value;
        } else if (arg == "--width") {
            const char* value = need_value(i);
            if (value == nullptr || !parse_uint(value, &out->width)) {
                return ParseOutcome::kError;
            }
        } else if (arg == "--height") {
            const char* value = need_value(i);
            if (value == nullptr || !parse_uint(value, &out->height)) {
                return ParseOutcome::kError;
            }
        } else if (arg == "--stride") {
            const char* value = need_value(i);
            if (value == nullptr || !parse_uint(value, &out->stride)) {
                return ParseOutcome::kError;
            }
        } else if (arg == "--refresh") {
            const char* value = need_value(i);
            if (value == nullptr || !parse_int(value, &out->refresh_hz)) {
                return ParseOutcome::kError;
            }
        } else if (arg == "--frames") {
            const char* value = need_value(i);
            if (value == nullptr || !parse_int(value, &out->frames)) {
                return ParseOutcome::kError;
            }
        } else if (arg.rfind("--", 0) == 0) {
            std::cerr << "未知选项: " << arg << "\n";
            return ParseOutcome::kError;
        } else if (out->command == "nv12" && out->nv12_path.empty()) {
            out->nv12_path = arg;  // nv12 子命令的首个位置参数是文件路径
        } else {
            std::cerr << "多余的位置参数: " << arg << "\n";
            return ParseOutcome::kError;
        }
    }

    if (out->stride == 0) {
        out->stride = out->width;
    }
    if (out->command == "nv12" && out->nv12_path.empty()) {
        std::cerr << "nv12 子命令需要提供 NV12 文件路径\n";
        return ParseOutcome::kError;
    }
    if (out->frames <= 0) {
        std::cerr << "--frames 必须为正数\n";
        return ParseOutcome::kError;
    }
    if (out->width == 0 || out->height == 0) {
        std::cerr << "--width/--height 必须为正数\n";
        return ParseOutcome::kError;
    }
    if (out->refresh_hz <= 0) {
        std::cerr << "--refresh 必须为正数（帧间隔按刷新率计算）\n";
        return ParseOutcome::kError;
    }
    if (out->stride < out->width) {
        std::cerr << "--stride 不能小于 --width（NV12 行必须容纳整行像素）\n";
        return ParseOutcome::kError;
    }
    return ParseOutcome::kOk;
}

HdmiDisplayConfig ToConfig(const Options& options) {
    HdmiDisplayConfig config;
    config.width = options.width;
    config.height = options.height;
    config.refresh_hz = options.refresh_hz;
    config.strict_mode = options.strict;
    config.drm_device = options.device;
    config.connector_name = options.connector;
    config.color_space =
        options.bt601 ? HdmiColorSpace::kBt601 : HdmiColorSpace::kBt709;
    config.full_range = options.full_range;
    config.blank_on_stop = options.blank_on_stop;
    return config;
}

/// 一像素的 NV12 色度取值。
struct Nv12Yuv {
    std::uint8_t y = 0;
    std::uint8_t u = 128;
    std::uint8_t v = 128;
};

/// RGB → NV12 的 Y/Cb/Cr。系数取 BT.601/BT.709 的有限范围或全范围标准值，
/// 与着色器所用矩阵互补，彩条经上屏后应还原为原始颜色。
Nv12Yuv RgbToNv12Yuv(std::uint8_t r, std::uint8_t g, std::uint8_t b, bool bt601,
                     bool full_range) {
    const double rd = r;
    const double gd = g;
    const double bd = b;
    double y = 0.0;
    double cb = 0.0;
    double cr = 0.0;
    if (bt601) {
        y = 16.0 + (65.481 * rd + 128.553 * gd + 24.966 * bd) / 255.0;
        cb = 128.0 + (-37.797 * rd - 74.203 * gd + 112.0 * bd) / 255.0;
        cr = 128.0 + (112.0 * rd - 93.786 * gd - 18.214 * bd) / 255.0;
    } else {
        y = 16.0 + (46.742 * rd + 157.243 * gd + 15.874 * bd) / 255.0;
        cb = 128.0 + (-25.805 * rd - 86.336 * gd + 112.141 * bd) / 255.0;
        cr = 128.0 + (112.0 * rd - 102.56 * gd - 9.44 * bd) / 255.0;
    }
    if (full_range) {
        // 全范围：去掉 16-235 偏置，直接按 0-255 归一化
        y = (bt601 ? (65.481 * rd + 128.553 * gd + 24.966 * bd)
                   : (46.742 * rd + 157.243 * gd + 15.874 * bd)) /
            255.0;
        cb = 128.0 + (bt601 ? (-37.797 * rd - 74.203 * gd + 112.0 * bd)
                            : (-25.805 * rd - 86.336 * gd + 112.141 * bd)) /
                         255.0;
        cr = 128.0 + (bt601 ? (112.0 * rd - 93.786 * gd - 18.214 * bd)
                            : (112.0 * rd - 102.56 * gd - 9.44 * bd)) /
                         255.0;
    }
    const auto clamp_u8 = [](double value) -> std::uint8_t {
        const double clamped = std::min(255.0, std::max(0.0, value));
        return static_cast<std::uint8_t>(clamped + 0.5);
    };
    return Nv12Yuv{clamp_u8(y), clamp_u8(cb), clamp_u8(cr)};
}

/// 填充 NV12 彩条：8 条标准彩条 + 随帧号移动的白色竖线。
void FillColorBars(std::uint8_t* y_plane, std::uint8_t* uv_plane, const Options& opt,
                   int frame_index) {
    // 彩条基色（8 条，等宽）
    static const std::uint8_t kBarColors[8][3] = {
        {235, 235, 235}, {235, 235, 16}, {16, 235, 235}, {16, 235, 16},
        {235, 16, 235},  {235, 16, 16},  {16, 16, 235},  {16, 16, 16},
    };
    const std::uint32_t width = opt.width;
    const std::uint32_t height = opt.height;
    const std::uint32_t stride = opt.stride;
    // 竖线横移一周约 120 帧，肉眼可辨"画面是否在刷新"
    const std::uint32_t marker_x = static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(frame_index) * (width / 120 + 1)) % width);
    constexpr std::uint32_t kMarkerWidth = 8;

    for (std::uint32_t y = 0; y < height; ++y) {
        std::uint8_t* row = y_plane + static_cast<std::size_t>(y) * stride;
        for (std::uint32_t x = 0; x < width; ++x) {
            const bool is_marker = x >= marker_x && x < marker_x + kMarkerWidth;
            if (is_marker) {
                row[x] = 235;  // 白线亮度
            } else {
                const std::uint32_t bar =
                    std::min<std::uint32_t>(7, x * 8 / width);
                row[x] = RgbToNv12Yuv(kBarColors[bar][0], kBarColors[bar][1],
                                      kBarColors[bar][2], opt.bt601,
                                      opt.full_range)
                             .y;
            }
        }
    }
    // 色度半分辨率：每 2x2 取左上像素的颜色。
    // NV12 色度平面的行 stride 与亮度平面一致（同为 stride 字节），
    // 每行前 width 字节为有效 UV 对，其后是填充；此处按字节计，不能写成 stride/2。
    for (std::uint32_t y = 0; y < height / 2; ++y) {
        std::uint8_t* row = uv_plane + static_cast<std::size_t>(y) * stride;
        for (std::uint32_t x = 0; x < width / 2; ++x) {
            const std::uint32_t sx = x * 2;
            const bool is_marker =
                sx >= marker_x && sx < marker_x + kMarkerWidth;
            Nv12Yuv yuv{};
            if (is_marker) {
                yuv = RgbToNv12Yuv(235, 235, 235, opt.bt601, opt.full_range);
            } else {
                const std::uint32_t bar =
                    std::min<std::uint32_t>(7, sx * 8 / width);
                yuv = RgbToNv12Yuv(kBarColors[bar][0], kBarColors[bar][1],
                                   kBarColors[bar][2], opt.bt601, opt.full_range);
            }
            row[x * 2] = yuv.u;
            row[x * 2 + 1] = yuv.v;
        }
    }
}

/// 创建显示设备：优先 DRM 实现；未编译 DRM 支持时回退占位实现并明确提示。
std::unique_ptr<IHdmiDisplay> MakeDisplay(bool* using_stub) {
    auto display = drone::video_transmission::CreateHdmiDrmDisplay();
    if (display != nullptr) {
        *using_stub = false;
        return display;
    }
    *using_stub = true;
    return drone::video_transmission::CreateHdmiDisplayStub();
}

int RunModes(const Options& options) {
    bool device_opened = false;
    std::cout << drone::video_transmission::DescribeHdmiDrmResources(
                     options.device, options.width, options.height,
                     options.refresh_hz, &device_opened);
    // 退出码反映"是否真的枚举到了资源"，供脚本/CI 判断：
    // 无设备节点、无权限、未编译 DRM 支持时均返回非 0。
    return device_opened ? 0 : 1;
}

int RunFrames(const Options& options, bool from_file) {
    bool using_stub = false;
    auto display = MakeDisplay(&using_stub);
    if (using_stub) {
        // 仅提示，不终止：开发机上仍可验证参数校验与计数逻辑
        spdlog::warn("当前构建未启用 DRM/KMS，改用占位实现（不会产生真实 HDMI 输出）");
    }

    // 准备帧缓冲
    std::vector<std::uint8_t> frame;
    if (from_file) {
        std::ifstream input(options.nv12_path, std::ios::binary | std::ios::ate);
        if (!input) {
            spdlog::error("打开 NV12 文件失败: {}", options.nv12_path);
            return 1;
        }
        const std::streamsize size = input.tellg();
        input.seekg(0, std::ios::beg);
        frame.resize(static_cast<std::size_t>(size));
        if (!input.read(reinterpret_cast<char*>(frame.data()), size)) {
            spdlog::error("读取 NV12 文件失败: {}", options.nv12_path);
            return 1;
        }
        const std::size_t expected = static_cast<std::size_t>(options.stride) *
                                     options.height * 3 / 2;
        if (frame.size() < expected) {
            spdlog::error("NV12 文件过小: 实际 {} 字节, 按 {}x{} stride={} 至少需要 {} 字节",
                          frame.size(), options.width, options.height, options.stride,
                          expected);
            return 1;
        }
        spdlog::info("已载入 NV12 文件 {} ({} 字节), 按 {}x{} stride={} 显示",
                     options.nv12_path, frame.size(), options.width, options.height,
                     options.stride);
    } else {
        frame.resize(static_cast<std::size_t>(options.stride) * options.height * 3 / 2,
                     0);
        spdlog::info("生成 NV12 彩条 {}x{} stride={} @{}Hz, 色彩空间={}, 范围={}",
                     options.width, options.height, options.stride, options.refresh_hz,
                     options.bt601 ? "BT.601" : "BT.709",
                     options.full_range ? "全范围" : "有限范围");
    }

    if (!display->Open(ToConfig(options))) {
        spdlog::error("打开 HDMI 显示设备失败，详见上方日志");
        return 1;
    }
    spdlog::info("输出模式: {}", display->ActiveModeName());

    const auto frame_interval = std::chrono::microseconds(1000000 / options.refresh_hz);
    // 按绝对时刻排程，使节拍严格对齐标称刷新率：
    // 若把 deadline 放在每帧生成之后计算，单帧耗时会累加进节拍（彩条生成约 20ms@720p），
    // 探针就跑不到标称帧率，"源与输出同速"这一前提也就不成立了。
    auto next_deadline = std::chrono::steady_clock::now();
    const auto loop_start = next_deadline;
    std::uint64_t submitted = 0;
    std::uint64_t busy = 0;
    std::uint64_t failed = 0;
    for (int index = 0; index < options.frames; ++index) {
        next_deadline += frame_interval;
        if (!from_file) {
            std::uint8_t* y_plane = frame.data();
            std::uint8_t* uv_plane =
                frame.data() + static_cast<std::size_t>(options.stride) * options.height;
            FillColorBars(y_plane, uv_plane, options, index);
        }
        const HdmiSubmitResult result =
            display->Submit(frame.data(), options.width, options.height,
                            options.stride);
        if (result == HdmiSubmitResult::kSubmitted) {
            ++submitted;
        } else if (result == HdmiSubmitResult::kBusy) {
            ++busy;
        } else {
            ++failed;
        }
        // 生成+提交超过一个帧间隔时 sleep_until 立即返回，自动退化为"尽力而为"
        std::this_thread::sleep_until(next_deadline);
    }

    const auto flip = display->FlipLatency();
    const auto prepare = display->PrepareLatency();
    // 实际达成帧率：明显低于标称刷新率说明 GPU 或 DRM 提交成为瓶颈，
    // 图传发射机端会表现为输出帧率不足。
    const double elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start)
            .count();
    const double achieved_fps =
        elapsed_s > 0.0 ? static_cast<double>(submitted + busy + failed) / elapsed_s
                        : 0.0;
    spdlog::info("上屏结束: 提交 {} 帧, 繁忙丢弃 {} 帧, 失败 {} 次; "
                 "耗时 {:.2f}s, 实际提交 {:.1f}fps(标称 {}Hz)",
                 submitted, busy, failed, elapsed_s, achieved_fps,
                 options.refresh_hz);
    spdlog::info("准备耗时 P50={:.2f}ms P99={:.2f}ms; 翻转提交 P50={:.2f}ms P99={:.2f}ms",
                 prepare.p50_ms, prepare.p99_ms, flip.p50_ms, flip.p99_ms);
    if (busy > 0) {
        spdlog::warn("出现 {} 帧繁忙丢弃：说明源帧率高于 HDMI 输出刷新率或 GPU 来不及，"
                     "属预期降级；若比例过高需检查 GPU 负载或降低源帧率",
                     busy);
    }

    display->Close();
    if (failed > 0) {
        spdlog::error("存在 {} 次提交失败，需排查上方 ERROR 日志", failed);
        return 1;
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    // 带外自检工具：输出到控制台，便于上板即时观察
    auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    sink->set_pattern("%H:%M:%S.%e [%^%l%$] %v");
    auto logger = std::make_shared<spdlog::logger>("hdmi_probe", sink);
    logger->set_level(spdlog::level::info);
    logger->flush_on(spdlog::level::info);
    spdlog::set_default_logger(logger);

    Options options;
    switch (ParseArgs(argc, argv, &options)) {
        case ParseOutcome::kHelp:
            return 0;  // 用户显式请求帮助属正常退出
        case ParseOutcome::kError:
            return 2;
        case ParseOutcome::kOk:
            break;
    }
    if (options.command == "modes") {
        return RunModes(options);
    }
    return RunFrames(options, options.command == "nv12");
}
