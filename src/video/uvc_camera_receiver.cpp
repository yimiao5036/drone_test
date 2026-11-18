/**
 * @file uvc_camera_receiver.cpp
 * @brief USB UVC 双目相机接收器实现（V4L2 mmap 直采）
 *
 * 打开序列：open(O_NONBLOCK) → QUERYCAP（须含采集+流能力）→ S_FMT(MJPG)
 * → G_FMT 回读校验（防固件错档）→ S_PARM(1/fps，尽力而为) → REQBUFS/MMAP
 * → QBUF 全部缓冲 → STREAMON。采集线程 poll(100ms) 定 tick，DQBUF 构成
 * EncodedFrame 发布后 QBUF 归还；无帧累计 1s 或 ioctl 报错视为断流，
 * 关流后按 reconnect_delay 重连直到 Stop。
 *
 * 仅使用 Linux 系统头（linux/videodev2.h），无第三方依赖；开发机无 USB
 * 相机时 Start 返回 false，采集正确性由香橙派板上验收覆盖。
 */
#include "video/uvc_camera_receiver.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <spdlog/spdlog.h>

namespace drone::video {

namespace {

// UvcCameraReceiver 数据流：V4L2 DQBUF → EncodedFrame(MJPG) Topic；
// 本模块只负责采集与重连，不解码、不拆分（拆分为 StereoFrameSplitter 职责）。

/// 单调时钟毫秒（与消息头时间戳约定一致）。
std::uint64_t SteadyNowMs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

/// 异常日志节流：第 1 次与每满 100 次才打印，避免高频异常刷屏。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

/// ioctl 包装：EINTR 自动重试。
int Xioctl(int fd, unsigned long request, void* arg) {
    int ret;
    do {
        ret = ::ioctl(fd, request, arg);
    } while (ret < 0 && errno == EINTR);
    return ret;
}

}  // namespace

/// UvcCameraReceiver 实现细节（PIMPL）：V4L2 fd、mmap 缓冲与采集线程。
struct UvcCameraReceiver::Impl {
    explicit Impl(UvcCameraReceiverConfig config) : config(std::move(config)) {
        if (this->config.device.empty()) {
            throw std::invalid_argument("UVC设备路径不能为空");
        }
        if (this->config.width == 0 || this->config.height == 0 ||
            this->config.fps == 0) {
            throw std::invalid_argument("UVC采集宽高与帧率必须大于 0");
        }
        if (this->config.buffer_count < 2) {
            throw std::invalid_argument("UVC mmap 缓冲数必须 >= 2");
        }
        if (this->config.reconnect_delay.count() < 0) {
            throw std::invalid_argument("重连间隔不能为负");
        }
    }

    ~Impl() { Stop(); }

    struct MmapBuffer {
        void* start = nullptr;
        std::size_t length = 0;
    };

    UvcCameraReceiverConfig config;
    std::atomic<bool> stop_requested{false};
    std::thread thread;

    int fd = -1;                       // 仅采集线程/Start 访问
    std::vector<MmapBuffer> buffers;   // mmap 缓冲，关流时统一 munmap

    // 状态计数（原子，跨线程可读）
    std::atomic<bool> connected{false};
    std::atomic<uint64_t> connect_count{0};
    std::atomic<uint64_t> received_bytes{0};
    std::atomic<uint64_t> error_count{0};
    std::atomic<uint64_t> sequence{0};

    common::Topic<common::EncodedFrame> stream_output;

    /// 打开设备并完成格式校验、缓冲映射与流启动；失败已清理并计数。
    // 从未成功建连时的失败打 ERROR（Start 失败语义），重连期间的失败打
    // WARN（断流重连是运行期常态，如插拔）。
    bool OpenDevice() {
        const auto fail = [this](const std::string& reason) {
            CloseDevice();
            const uint64_t count = error_count.fetch_add(1) + 1;
            if (ShouldLogThrottled(count)) {
                if (connect_count.load() == 0) {
                    SPDLOG_ERROR("UVC摄像头打开失败: {} {}，累计 {}", config.device,
                                 reason, count);
                } else {
                    SPDLOG_WARN("UVC摄像头重连失败: {} {}，累计 {}", config.device,
                                reason, count);
                }
            }
            return false;
        };

        fd = ::open(config.device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            return fail(std::string("open: ") + std::strerror(errno));
        }

        v4l2_capability cap{};
        if (Xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
            return fail(std::string("QUERYCAP: ") + std::strerror(errno));
        }
        const std::uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
                                       ? cap.device_caps
                                       : cap.capabilities;
        if ((caps & V4L2_CAP_VIDEO_CAPTURE) == 0 ||
            (caps & V4L2_CAP_STREAMING) == 0) {
            return fail("节点不具备单平面采集+流能力");
        }

        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = config.width;
        fmt.fmt.pix.height = config.height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;
        if (Xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            return fail(std::string("S_FMT(MJPG): ") + std::strerror(errno));
        }
        // G_FMT 回读校验：必须拿到配置的分辨率与 MJPG 格式，防固件错档。
        v4l2_format actual{};
        actual.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (Xioctl(fd, VIDIOC_G_FMT, &actual) < 0) {
            return fail(std::string("G_FMT: ") + std::strerror(errno));
        }
        if (actual.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG ||
            actual.fmt.pix.width != config.width ||
            actual.fmt.pix.height != config.height) {
            return fail("格式回读校验不符: 实际=" +
                        std::to_string(actual.fmt.pix.width) + "x" +
                        std::to_string(actual.fmt.pix.height) + " fourcc=" +
                        std::to_string(actual.fmt.pix.pixelformat) +
                        "（要求 MJPG " + std::to_string(config.width) + "x" +
                        std::to_string(config.height) + "）");
        }

        // 帧率尽力而为：部分驱动不支持 S_PARM，失败仅记录不阻断采集。
        v4l2_streamparm parm{};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = config.fps;
        if (Xioctl(fd, VIDIOC_S_PARM, &parm) < 0) {
            SPDLOG_WARN("UVC摄像头设置帧率失败（尽力而为，继续）: {} {}",
                        config.device, std::strerror(errno));
        }

        v4l2_requestbuffers req{};
        req.count = config.buffer_count;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (Xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            return fail(std::string("REQBUFS: ") + std::strerror(errno));
        }
        if (req.count < 2) {
            return fail("驱动可用 mmap 缓冲数 < 2");
        }

        buffers.clear();
        buffers.reserve(req.count);
        for (std::uint32_t index = 0; index < req.count; ++index) {
            v4l2_buffer buffer{};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = index;
            if (Xioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0) {
                return fail(std::string("QUERYBUF: ") + std::strerror(errno));
            }
            void* start = ::mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd, buffer.m.offset);
            if (start == MAP_FAILED) {
                return fail(std::string("mmap: ") + std::strerror(errno));
            }
            buffers.push_back(MmapBuffer{start, buffer.length});
        }

        for (std::uint32_t index = 0; index < req.count; ++index) {
            v4l2_buffer buffer{};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = index;
            if (Xioctl(fd, VIDIOC_QBUF, &buffer) < 0) {
                return fail(std::string("QBUF: ") + std::strerror(errno));
            }
        }

        v4l2_buf_type stream_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (Xioctl(fd, VIDIOC_STREAMON, &stream_type) < 0) {
            return fail(std::string("STREAMON: ") + std::strerror(errno));
        }

        connected.store(true);
        const uint64_t count = connect_count.fetch_add(1) + 1;
        SPDLOG_INFO("UVC摄像头建连: {} 驱动={} 设备={} {}x{} MJPG 缓冲={}（累计第 {} 次）",
                    config.device, reinterpret_cast<const char*>(cap.driver),
                    reinterpret_cast<const char*>(cap.card), config.width,
                    config.height, req.count, count);
        return true;
    }

    /// 关流并释放 fd 与 mmap 缓冲（幂等）。
    void CloseDevice() {
        if (fd >= 0) {
            v4l2_buf_type stream_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (Xioctl(fd, VIDIOC_STREAMOFF, &stream_type) < 0) {
                SPDLOG_WARN("UVC摄像头关流失败: {} {}", config.device,
                            std::strerror(errno));
            }
            for (auto& buffer : buffers) {
                if (buffer.start != nullptr && buffer.length > 0) {
                    ::munmap(buffer.start, buffer.length);
                }
            }
            ::close(fd);
            fd = -1;
        }
        buffers.clear();
        connected.store(false);
    }

    /// 断流/硬件错误处理：计数、节流日志、关流（由采集线程随后重连）。
    void HandleStreamError(const std::string& reason) {
        const uint64_t count = error_count.fetch_add(1) + 1;
        if (ShouldLogThrottled(count)) {
            SPDLOG_WARN("UVC摄像头断流: {} {}，按 {}ms 重连，累计 {}",
                        config.device, reason, config.reconnect_delay.count(),
                        count);
        }
        CloseDevice();
    }

    /// DQBUF 一帧并发布，随后 QBUF 归还。返回 false 表示发生断流级错误
    /// （已走 HandleStreamError）；EAGAIN（非阻塞暂无帧）返回 true。
    bool DequeueAndPublish() {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (Xioctl(fd, VIDIOC_DQBUF, &buffer) < 0) {
            if (errno == EAGAIN) {
                return true;
            }
            HandleStreamError(std::string("DQBUF: ") + std::strerror(errno));
            return false;
        }
        const std::uint64_t receive_time_ms = SteadyNowMs();

        if (buffer.index >= buffers.size()) {
            HandleStreamError("DQBUF 返回越界缓冲序号");
            return false;
        }
        const MmapBuffer& slot = buffers[buffer.index];
        const std::size_t used =
            std::min<std::size_t>(buffer.bytesused, slot.length);

        common::EncodedFrame frame;
        const std::uint64_t seq = sequence.fetch_add(1) + 1;
        frame.header.sequence = seq;
        frame.header.receive_time_ms = receive_time_ms;
        // v4l2 monotonic/soe 时间戳与主机单调时钟同域（探测报告实证可用）；
        // 无 MONOTONIC 标志时保持 0，不伪造采集时刻。
        if (buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
            const std::uint64_t capture_ms =
                static_cast<std::uint64_t>(buffer.timestamp.tv_sec) * 1000ULL +
                static_cast<std::uint64_t>(buffer.timestamp.tv_usec) / 1000ULL;
            frame.capture_time_ms = capture_ms;
            frame.header.source_time_ms = capture_ms;
        }
        frame.codec = common::VideoCodec::kMjpeg;
        frame.stream_sequence = static_cast<std::uint32_t>(seq);
        frame.is_key_frame = true;  // MJPG 每帧独立，无帧间依赖
        const auto* begin = static_cast<const std::uint8_t*>(slot.start);
        frame.data.assign(begin, begin + used);
        received_bytes.fetch_add(used);
        (void)stream_output.Emplace(std::move(frame));

        if (Xioctl(fd, VIDIOC_QBUF, &buffer) < 0) {
            HandleStreamError(std::string("QBUF归还: ") + std::strerror(errno));
            return false;
        }
        return true;
    }

    /// 可被打断的重连等待：按 100ms 粒度睡眠，返回 true 表示收到停止请求。
    bool ReconnectWaitInterrupted() {
        auto remaining = config.reconnect_delay;
        while (!stop_requested.load() && remaining.count() > 0) {
            const auto step = std::min(remaining, std::chrono::milliseconds(100));
            std::this_thread::sleep_for(step);
            remaining -= step;
        }
        return stop_requested.load();
    }

    /// 采集线程主循环。poll 以 100ms 定 tick（Stop 响应 ≤100ms）；无帧
    /// 累计 1s（10 tick）视为断流，关流后按 reconnect_delay 重连。
    void CaptureLoop() {
        int idle_ticks = 0;
        while (!stop_requested.load()) {
            if (fd < 0) {
                if (ReconnectWaitInterrupted()) {
                    break;
                }
                if (!OpenDevice()) {
                    continue;  // 重连失败：下一轮继续等待后重试
                }
                idle_ticks = 0;
            }

            pollfd pfd{};
            pfd.fd = fd;
            pfd.events = POLLIN;
            const int poll_ret = ::poll(&pfd, 1, 100);
            if (stop_requested.load()) {
                break;
            }
            if (poll_ret < 0) {
                if (errno == EINTR) {
                    continue;
                }
                HandleStreamError(std::string("poll: ") + std::strerror(errno));
                idle_ticks = 0;
                continue;
            }
            if (poll_ret == 0) {
                if (++idle_ticks >= 10) {
                    HandleStreamError("超过 1s 无帧");
                    idle_ticks = 0;
                }
                continue;
            }
            idle_ticks = 0;
            if ((pfd.revents & POLLIN) == 0) {
                if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                    HandleStreamError("poll 返回错误事件");
                }
                continue;
            }
            if (!DequeueAndPublish()) {
                idle_ticks = 0;
            }
        }
    }

    /// 启动：先同步完成首次打开（失败即 Start 失败），再建采集线程。
    bool Start() {
        if (thread.joinable()) {
            return true;  // 幂等
        }
        stop_requested = false;
        if (!OpenDevice()) {
            return false;  // 设备不存在/格式校验不符：Start 失败
        }
        thread = std::thread(&Impl::CaptureLoop, this);
        return true;
    }

    /// 停止：置标志（poll 100ms 内响应）→ join → 关设备；幂等。
    void Stop() {
        if (thread.joinable()) {
            stop_requested = true;
            thread.join();
        }
        CloseDevice();
    }
};

UvcCameraReceiver::UvcCameraReceiver(UvcCameraReceiverConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
    SPDLOG_INFO("UVC摄像头接收器创建: {} {}x{}@{} 缓冲={}",
                impl_->config.device, impl_->config.width, impl_->config.height,
                impl_->config.fps, impl_->config.buffer_count);
}

UvcCameraReceiver::~UvcCameraReceiver() {
    if (impl_ != nullptr) {
        impl_->Stop();
    }
    SPDLOG_INFO("UVC摄像头接收器销毁");
}

bool UvcCameraReceiver::Start() {
    if (!impl_->Start()) {
        return false;
    }
    SPDLOG_INFO("UVC摄像头接收器启动");
    return true;
}

void UvcCameraReceiver::Stop() {
    impl_->Stop();
    SPDLOG_INFO("UVC摄像头接收器停止");
}

bool UvcCameraReceiver::IsRunning() const {
    return impl_->thread.joinable();
}

bool UvcCameraReceiver::IsConnected() const {
    return impl_->connected.load();
}

uint64_t UvcCameraReceiver::ConnectCount() const {
    return impl_->connect_count.load();
}

uint64_t UvcCameraReceiver::ReceivedBytes() const {
    return impl_->received_bytes.load();
}

uint64_t UvcCameraReceiver::ErrorCount() const {
    return impl_->error_count.load();
}

common::Topic<common::EncodedFrame>& UvcCameraReceiver::StreamOutput() {
    return impl_->stream_output;
}

}  // namespace drone::video
