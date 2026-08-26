/**
 * @file frame_compositor.cpp
 * @brief 视频帧叠加器实现（FrameCompositor）
 *
 * 订阅解码帧（NV12）与检测结果，输出叠加检测框 + 类型/置信度文字的标注帧。
 * 详细说明见 include/video/frame_compositor.h 与 frame_compositor.md。
 *
 * 绘制实现（零外部依赖）：
 * - 检测框：NV12 上按行描彩色矩形边框（边框像素改 Y → 亮度，改对应 U/V → 颜色）。
 * - 文字：内置 5×7 点阵 ASCII 字模（数字、大写字母、'.'、'%'、' '、'-'），
 *   用于渲染"类型标记 + 置信度%"（如 "UAV 85%"、"OBS 63%"）。
 *
 * 归并策略：逐帧先应用上次抽样后累积的检测结果（按 receive_time 顺序），
 * 即"取到一帧时，把截至当前收到的最新检测画上去"，实现帧-检测异步对齐。
 */
#include "video/frame_compositor.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "video/video_frame_pool.h"

#include <spdlog/spdlog.h>

namespace drone::video {

namespace {

/// 异常日志节流：第 1 次与每满 100 次才打印。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

/// 像素对齐（向上取整）。
std::uint32_t AlignUp(std::uint32_t value, std::uint32_t alignment) {
    if (alignment == 0) {
        return value;
    }
    return (value + alignment - 1) / alignment * alignment;
}

// ---------------------------------------------------------------------------
// NV12 像素写入辅助
// ---------------------------------------------------------------------------
struct Nv12Surface {
    std::byte* data = nullptr;          // 帧缓冲首地址
    std::uint32_t hor_stride = 0;       // 水平 stride（像素）
    std::uint32_t height = 0;

    std::byte* Y(uint32_t x, uint32_t y) const {
        return data + static_cast<size_t>(y) * hor_stride + x;
    }
    // NV12 UV 平面：偏移 = hor_stride*height，UV 交替存储。
    // 像素 (x,y) 对应 UV 采样 (x/2, y/2)，每采样 2 字节(U,V)。
    // 注意 NV12 第 1 字节为 U(Cb)、第 2 字节为 V(Cr)。
    std::byte* U(uint32_t x, uint32_t y) const {
        const size_t uv_base = static_cast<size_t>(hor_stride) * height;
        const size_t off = static_cast<size_t>(y / 2) * hor_stride + (x / 2) * 2;
        return data + uv_base + off;
    }
    std::byte* V(uint32_t x, uint32_t y) const {
        return U(x, y) + 1;
    }
};

/// 绘制一个像素（含色）：Y 亮度 + 对应采样 U/V 置为彩色。
/// box 以亮黄绿高对比色：Y=230，U/V 选为明显区别于无色的绿色调。
inline void PlotPixel(Nv12Surface& s, uint32_t x, uint32_t y) {
    if (x >= s.hor_stride || y >= s.height) {
        return;
    }
    // 亮绿色边框：Y 高亮，U/V 取绿色域（NV12 YUV：绿色 ≈ U=149,V=43 区段）
    *s.Y(x, y) = static_cast<std::byte>(230);
    *s.U(x, y) = static_cast<std::byte>(150);
    *s.V(x, y) = static_cast<std::byte>(44);
}

// ---------------------------------------------------------------------------
// 5×7 点阵 ASCII 字模
// ---------------------------------------------------------------------------
// 使用函数而非庞大表：仅当 label 用到时才查，降低出错面。
// 每字符 7 行，每行低 5 位（bit4..bit0）表示左→右 5 点。
// 覆盖：空格、数字0-9、'.'、'%'、'-'、大写字母（类型简写用）。
std::uint8_t GlyphByte(char ch, int row) {
    // 行号 row ∈ [0,7)。字符基础字形定义。
    switch (ch) {
        case ' ': return 0x00;
        case '0': { const std::uint8_t g[7]={0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; return g[row]; }
        case '1': { const std::uint8_t g[7]={0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
        case '2': { const std::uint8_t g[7]={0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}; return g[row]; }
        case '3': { const std::uint8_t g[7]={0x0E,0x11,0x01,0x0E,0x01,0x11,0x0E}; return g[row]; }
        case '4': { const std::uint8_t g[7]={0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}; return g[row]; }
        case '5': { const std::uint8_t g[7]={0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}; return g[row]; }
        case '6': { const std::uint8_t g[7]={0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}; return g[row]; }
        case '7': { const std::uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x08,0x08}; return g[row]; }
        case '8': { const std::uint8_t g[7]={0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}; return g[row]; }
        case '9': { const std::uint8_t g[7]={0x0E,0x11,0x11,0x0F,0x01,0x01,0x0E}; return g[row]; }
        case '.': { const std::uint8_t g[7]={0x00,0x00,0x00,0x00,0x0C,0x0C,0x00}; return g[row]; }
        case '%': { const std::uint8_t g[7]={0x11,0x12,0x04,0x08,0x10,0x11,0x01}; return g[row]; }
        case '-': { const std::uint8_t g[7]={0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; return g[row]; }
        case 'U': { const std::uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
        case 'A': { const std::uint8_t g[7]={0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
        case 'V': { const std::uint8_t g[7]={0x11,0x11,0x11,0x11,0x0A,0x0A,0x04}; return g[row]; }
        case 'B': { const std::uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}; return g[row]; }
        case 'C': { const std::uint8_t g[7]={0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}; return g[row]; }
        case 'D': { const std::uint8_t g[7]={0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}; return g[row]; }
        case 'E': { const std::uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}; return g[row]; }
        case 'F': { const std::uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}; return g[row]; }
        case 'G': { const std::uint8_t g[7]={0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}; return g[row]; }
        case 'H': { const std::uint8_t g[7]={0x11,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
        case 'I': { const std::uint8_t g[7]={0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}; return g[row]; }
        case 'J': { const std::uint8_t g[7]={0x07,0x02,0x02,0x02,0x12,0x12,0x0C}; return g[row]; }
        case 'K': { const std::uint8_t g[7]={0x11,0x12,0x14,0x18,0x14,0x12,0x11}; return g[row]; }
        case 'L': { const std::uint8_t g[7]={0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return g[row]; }
        case 'M': { const std::uint8_t g[7]={0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; return g[row]; }
        case 'N': { const std::uint8_t g[7]={0x11,0x19,0x15,0x13,0x11,0x11,0x11}; return g[row]; }
        case 'O': { const std::uint8_t g[7]={0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
        case 'P': { const std::uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return g[row]; }
        case 'Q': { const std::uint8_t g[7]={0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}; return g[row]; }
        case 'R': { const std::uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return g[row]; }
        case 'S': { const std::uint8_t g[7]={0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; return g[row]; }
        case 'T': { const std::uint8_t g[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return g[row]; }
        case 'W': { const std::uint8_t g[7]={0x11,0x11,0x11,0x15,0x15,0x15,0x0A}; return g[row]; }
        case 'X': { const std::uint8_t g[7]={0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}; return g[row]; }
        case 'Y': { const std::uint8_t g[7]={0x11,0x11,0x0A,0x04,0x04,0x04,0x04}; return g[row]; }
        case 'Z': { const std::uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}; return g[row]; }
        default: return 0x00;
    }
}

/// 在 NV12 表面绘制一行文字。
/// @param s 表面
/// @param x0 起点像素 x（左）
/// @param y0 起点像素 y（文字顶行）
/// @param text UTF-8 文本（此处仅用 ASCII）
/// @param scale 放大倍数（每字符占 5*scale 宽 × 7*scale 高）
void DrawText(Nv12Surface& s, int x0, int y0, const std::string& text, int scale) {
    if (scale <= 0) {
        scale = 1;
    }
    const std::uint32_t w = s.hor_stride;
    const std::uint32_t h = s.height;
    int cursor = x0;
    for (char ch : text) {
        for (int row = 0; row < 7; ++row) {
            const std::uint8_t bits = GlyphByte(ch, row);
            for (int col = 0; col < 5; ++col) {
                if (bits & (0x10u >> col)) {
                    for (int dy = 0; dy < scale; ++dy) {
                        for (int dx = 0; dx < scale; ++dx) {
                            const std::uint32_t px = static_cast<std::uint32_t>(
                                cursor + col * scale + dx);
                            const std::uint32_t py = static_cast<std::uint32_t>(
                                y0 + row * scale + dy);
                            if (px < w && py < h) {
                                PlotPixel(s, px, py);
                            }
                        }
                    }
                }
            }
        }
        cursor += 6 * scale;  // 字符间距 1 列
    }
}

/// 绘制 NV12 矩形框。
void DrawBox(Nv12Surface& s, float bx, float by, float bw, float bh, int thick) {
    if (thick <= 0) {
        thick = 1;
    }
    const std::uint32_t w = s.hor_stride;
    const std::uint32_t h = s.height;
    const int x0 = std::max(0, static_cast<int>(bx));
    const int y0 = std::max(0, static_cast<int>(by));
    const int x1 = std::min(static_cast<int>(w) - 1, static_cast<int>(bx + bw));
    const int y1 = std::min(static_cast<int>(h) - 1, static_cast<int>(by + bh));
    if (x1 <= x0 || y1 <= y0) {
        return;
    }
    for (int t = 0; t < thick; ++t) {
        // 上下横边
        for (int x = x0; x <= x1; ++x) {
            int yy0 = y0 + t;
            int yy1 = y1 - t;
            if (yy0 >= 0 && yy0 < static_cast<int>(h)) PlotPixel(s, static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(yy0));
            if (yy1 >= 0 && yy1 < static_cast<int>(h) && yy1 != yy0) PlotPixel(s, static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(yy1));
        }
        // 左右竖边
        for (int y = y0; y <= y1; ++y) {
            int xx0 = x0 + t;
            int xx1 = x1 - t;
            if (xx0 >= 0 && xx0 < static_cast<int>(w)) PlotPixel(s, static_cast<std::uint32_t>(xx0), static_cast<std::uint32_t>(y));
            if (xx1 >= 0 && xx1 < static_cast<int>(w) && xx1 != xx0) PlotPixel(s, static_cast<std::uint32_t>(xx1), static_cast<std::uint32_t>(y));
        }
    }
}

/// 将 JSON 类别名称规范成点阵可显示的 ASCII，并限制长度避免标签越界过长。
std::string NormalizeClassLabel(const std::string& label) {
    constexpr std::size_t kMaxLabelLength = 16;
    std::string normalized;
    bool has_visible_character = false;
    normalized.reserve(std::min(label.size(), kMaxLabelLength));
    for (char ch : label) {
        if (normalized.size() >= kMaxLabelLength) {
            break;
        }
        if (ch >= 'a' && ch <= 'z') {
            ch = static_cast<char>(ch - 'a' + 'A');
        }
        const bool supported = (ch >= 'A' && ch <= 'Z') ||
                               (ch >= '0' && ch <= '9') || ch == ' ' ||
                               ch == '.' || ch == '-' || ch == '%';
        const char output = supported ? ch : ' ';
        normalized.push_back(output);
        has_visible_character = has_visible_character || output != ' ';
    }
    return has_visible_character ? normalized : "OBJ";
}

/// 类别 ID → JSON 配置名称；越界或空名称回退 OBJ。
std::string ClassToken(const std::vector<std::string>& class_names,
                       std::uint32_t class_id) {
    if (class_id >= class_names.size() || class_names[class_id].empty()) {
        return "OBJ";
    }
    return NormalizeClassLabel(class_names[class_id]);
}

}  // namespace

/// FrameCompositor 实现细节（PIMPL）：两路订阅、输出池与消费线程。
struct FrameCompositor::Impl {
    explicit Impl(CompositorConfig cfg) : config(std::move(cfg)) {
        if (config.pool_capacity == 0) {
            throw std::invalid_argument("叠加器输出池容量必须大于 0");
        }
    }

    CompositorConfig config;
    std::atomic<bool> stop_requested{false};
    std::thread thread;

    common::Topic<FrameHandle>::Subscription decoded_sub;
    common::Topic<common::DetectionResult>::Subscription detection_sub;
    common::Topic<FrameHandle> annotated_output;

    // 主题指针：Stop 会 Reset 订阅，重启时据此重新订阅（幂等）
    common::Topic<FrameHandle>* decoded_topic = nullptr;
    common::Topic<common::DetectionResult>* detection_topic = nullptr;

    // 检测结果跨线程暂存：检测订阅在消费线程读取，无需锁（单独线程）。
    // 但 SetDetectionInput 与等待循环同线程，这里存放"取帧时尚未处理的最新结果"。
    // 只保留最新检测帧的一组目标；同一 frame_sequence 的多目标一起保留。
    std::vector<common::DetectionResult> pending_detections;
    std::uint64_t latest_detection_frame_sequence = 0;
    bool has_detection_frame_sequence = false;

    std::shared_ptr<VideoFramePool> pool;  // 输出标注帧池（懒建）
    std::atomic<uint64_t> annotated_count{0};
    std::atomic<uint64_t> dropped_count{0};
    std::atomic<uint64_t> error_count{0};

    void CreatePool(std::uint32_t width, std::uint32_t height) {
        VideoFrameInfo tmpl;
        tmpl.width = width;
        tmpl.height = height;
        tmpl.hor_stride = AlignUp(width, config.stride_alignment);
        tmpl.ver_stride = height;
        tmpl.format = PixelFormat::kYuv420SpNv12;
        pool = std::make_shared<VideoFramePool>(config.pool_capacity, tmpl);
        SPDLOG_INFO("叠加器输出池创建: 容量={} 分辨率={}x{} 水平stride={} 槽位={}B",
                    config.pool_capacity, tmpl.width, tmpl.height, tmpl.hor_stride,
                    pool->SlotSize());
    }

    /// 拉取检测结果，只保留最新 frame_sequence 的整组目标。
    /// 旧实现持续 push_back 且从不清空，导致历史位置的框被每帧重复绘制。
    void DrainDetections() {
        for (;;) {
            auto msg = detection_sub.TryTake();
            if (!msg) {
                break;
            }
            const auto& detection = **msg;
            if (!has_detection_frame_sequence ||
                detection.frame_sequence > latest_detection_frame_sequence) {
                pending_detections.clear();
                latest_detection_frame_sequence = detection.frame_sequence;
                has_detection_frame_sequence = true;
            }
            if (detection.frame_sequence == latest_detection_frame_sequence) {
                pending_detections.push_back(detection);
            }
            // 晚到的旧帧检测直接丢弃，避免框倒退到历史位置。
        }
    }

    /// 连续无检测时按帧序号清除旧框，避免目标消失后最后一个框永久停留。
    void ExpireStaleDetections(std::uint64_t current_frame_sequence) {
        if (pending_detections.empty() || !has_detection_frame_sequence ||
            current_frame_sequence <= latest_detection_frame_sequence) {
            return;
        }
        if (current_frame_sequence - latest_detection_frame_sequence >
            config.max_detection_frame_lag) {
            pending_detections.clear();
            latest_detection_frame_sequence = 0;
            has_detection_frame_sequence = false;
        }
    }

    /// 消费线程主循环。
    void Run() {
        while (!stop_requested.load()) {
            DrainDetections();
            auto message = decoded_sub.WaitTakeFor(std::chrono::milliseconds(100));
            if (!message) {
                continue;  // 超时或主题关闭；循环顶检查停止标志
            }
            const auto& in_handle = **message;
            if (!in_handle.Valid() ||
                in_handle.Info().format != PixelFormat::kYuv420SpNv12) {
                ++error_count;
                if (ShouldLogThrottled(error_count)) {
                    SPDLOG_ERROR("叠加器输入帧非法(格式/句柄)，累计 {}",
                                 error_count.load());
                }
                continue;
            }
            // 检测 Topic 不会唤醒 decoded_sub 的等待；取到帧后再拉一次，确保等待
            // 期间到达的新检测在本帧立即生效，而不是延迟到下一帧。
            DrainDetections();
            ExpireStaleDetections(in_handle.Info().sequence);
            Compose(in_handle, pending_detections);
        }
    }

    /// 拷贝输入帧 → 新池帧 → 叠加 → 发布。
    void Compose(const FrameHandle& in,
                 const std::vector<common::DetectionResult>& dets) {
        const VideoFrameInfo& src = in.Info();
        const std::uint32_t w = src.width;
        const std::uint32_t h = src.height;
        if (pool == nullptr) {
            try {
                CreatePool(w, h);
            } catch (const std::exception& e) {
                ++error_count;
                if (ShouldLogThrottled(error_count)) {
                    SPDLOG_ERROR("叠加器创建输出池失败: {}，累计 {}", e.what(),
                                 error_count.load());
                }
                return;
            }
        }

        auto out = pool->Acquire();
        if (!out.Valid()) {
            ++dropped_count;
            if (ShouldLogThrottled(dropped_count)) {
                SPDLOG_WARN("叠加器输出池满丢帧，累计 {}", dropped_count.load());
            }
            return;
        }

        // NV12 拷贝：Y 与 UV 平面按行（源 stride 可能 ≠ 输出 stride）
        const VideoFrameInfo& dst_info = out.Info();
        const unsigned out_stride = dst_info.hor_stride;
        const unsigned src_stride = src.hor_stride;
        {
            std::byte* dst = out.Data();
            const std::byte* s = in.Data();
            for (std::uint32_t row = 0; row < h; ++row) {
                std::memcpy(dst + static_cast<size_t>(row) * out_stride,
                            s + static_cast<size_t>(row) * src_stride, w);
            }
            // UV 平面
            const std::byte* src_uv = s + static_cast<size_t>(src_stride) * h;
            std::byte* dst_uv = dst + static_cast<size_t>(out_stride) * h;
            for (std::uint32_t row = 0; row < h / 2; ++row) {
                std::memcpy(dst_uv + static_cast<size_t>(row) * out_stride,
                            src_uv + static_cast<size_t>(row) * src_stride, w);
            }
        }

        // 叠加检测框 + 文字
        Nv12Surface surface{out.Data(), out_stride, h};
        for (const auto& d : dets) {
            DrawBox(surface, d.bbox_x, d.bbox_y, d.bbox_w, d.bbox_h,
                    config.box_line_thickness);
            if (config.draw_text) {
                const int conf_pct = static_cast<int>(d.confidence * 100 + 0.5f);
                std::string text = ClassToken(config.class_names, d.class_id) + " " +
                                   std::to_string(conf_pct) + "%";
                // 文字画在框左上角外侧（避免遮目标本身）
                int ty = static_cast<int>(d.bbox_y) - 7 * config.text_scale;
                if (ty < 0) {
                    ty = static_cast<int>(d.bbox_y) + static_cast<int>(d.bbox_h) + 2;
                }
                DrawText(surface, static_cast<int>(d.bbox_x), ty, text,
                         config.text_scale);
            }
        }

        (void)annotated_output.Emplace(std::move(out));
        ++annotated_count;
    }

    void Stop() {
        if (!thread.joinable()) {
            return;
        }
        stop_requested = true;
        decoded_sub.Reset();  // 唤醒 WaitTakeFor
        thread.join();
    }
};

FrameCompositor::FrameCompositor(CompositorConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
    SPDLOG_INFO("叠加器创建: 池容量={} 线宽={} 文字={}",
                impl_->config.pool_capacity, impl_->config.box_line_thickness,
                impl_->config.draw_text ? "开" : "关");
}

FrameCompositor::~FrameCompositor() {
    if (impl_ != nullptr) {
        impl_->Stop();
    }
    SPDLOG_INFO("叠加器销毁");
}

bool FrameCompositor::Start() {
    if (impl_->thread.joinable()) {
        return true;  // 幂等
    }
    impl_->stop_requested = false;
    // Stop 会 Reset 订阅，重启时重新订阅（幂等：已打开则不重复）
    if (!impl_->decoded_sub.IsOpen() && impl_->decoded_topic != nullptr) {
        impl_->decoded_sub = impl_->decoded_topic->Subscribe(4);
    }
    if (!impl_->detection_sub.IsOpen() && impl_->detection_topic != nullptr) {
        impl_->detection_sub = impl_->detection_topic->Subscribe(32);
    }
    impl_->thread = std::thread(&Impl::Run, impl_.get());
    SPDLOG_INFO("叠加器启动");
    return true;
}

void FrameCompositor::Stop() {
    impl_->Stop();
    SPDLOG_INFO("叠加器停止");
}

bool FrameCompositor::IsRunning() const {
    return impl_->thread.joinable();
}

void FrameCompositor::SetDecodedInput(common::Topic<FrameHandle>& decoded) {
    impl_->decoded_topic = &decoded;
    impl_->decoded_sub = decoded.Subscribe(4);  // 丢最旧，落后只丢重叠加帧
}

void FrameCompositor::SetDetectionInput(
    common::Topic<common::DetectionResult>& detection) {
    impl_->detection_topic = &detection;
    impl_->detection_sub = detection.Subscribe(32);  // 缓存检测结果供对齐
}

common::Topic<FrameHandle>& FrameCompositor::AnnotatedOutput() {
    return impl_->annotated_output;
}

std::uint64_t FrameCompositor::AnnotatedCount() const {
    return impl_->annotated_count.load();
}

std::uint64_t FrameCompositor::DroppedFrameCount() const {
    return impl_->dropped_count.load();
}

std::uint64_t FrameCompositor::ErrorCount() const {
    return impl_->error_count.load();
}

}  // namespace drone::video
