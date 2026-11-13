#include "communication/communication_transport.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <spdlog/spdlog.h>

namespace drone::communication {
namespace {

// ============================================================================
// communication_transport —— 串口 / UDP 的统一字节传输边界。
//
// 为 Px4Link 与地面站链路提供一致的 Read/Write 抽象，业务链路不因物理设备而变。
//   - SerialTransport：复用 SerialPort，用于香橙派 /dev/ttyS1；
//   - UdpTransport：非阻塞 UDP + poll() 毫秒级超时，用于 PX4 SITL / 网络数传。
// 本层不负责 MAVLink 解析、线程、自动重连与业务消息转换。
//
// 线程模型：每个 transport 由一条 Link 线程独占，内部不加业务锁。
// 日志纪律：只记录打开/关闭/学习对端与关键错误，错误按第 1 次和每满 100 次节流。
// ============================================================================

// 异常日志节流：第 1 次与每满 100 次打印，防止高频错误刷屏。
bool ShouldLogThrottled(uint64_t count) {
    return count == 1 || count % 100 == 0;
}

// 串口传输：将 SerialPort 适配为统一传输接口，纯转发、无额外状态维护。
class SerialTransport final : public ICommunicationTransport {
public:
    explicit SerialTransport(SerialPortConfig config) : port_(std::move(config)) {}
    void Open() override { port_.Open(); }
    void Close() override { port_.Close(); }
    bool IsOpen() const override { return port_.IsOpen(); }
    std::ptrdiff_t Read(uint8_t* buffer, std::size_t size) override {
        return port_.Read(buffer, size);
    }
    bool Write(const uint8_t* data, std::size_t size) override {
        return port_.Write(data, size);
    }
    void Flush() override { port_.Flush(); }
    uint64_t ErrorCount() const override { return port_.ErrorCount(); }
    std::string Description() const override {
        return "serial:" + port_.Config().device + "@" +
               std::to_string(port_.Config().baud_rate);
    }

private:
    SerialPort port_;
};

class UdpTransport final : public ICommunicationTransport {
public:
    explicit UdpTransport(UdpTransportConfig config) : config_(std::move(config)) {
        config_.Validate();
    }
    ~UdpTransport() override { Close(); }

    // 打开：创建非阻塞 UDP socket、绑定本地地址端口，并按配置预知远端。
    void Open() override {
        if (fd_ >= 0) {
            return;  // 已打开，幂等返回
        }
        // SOCK_NONBLOCK 配合下面 poll() 实现毫秒级超时；SOCK_CLOEXEC 防止 fork/exec 泄漏。
        fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) {
            throw std::system_error(errno, std::generic_category(), "创建 UDP socket 失败");
        }
        // SO_REUSEADDR 仅便于测试快速重建，不允许多个活跃进程同时占用同一 SITL 端口。
        int reuse = 1;
        (void)::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = htons(config_.bind_port);
        if (::inet_pton(AF_INET, config_.bind_address.c_str(), &local.sin_addr) != 1) {
            Close();
            throw std::invalid_argument("UDP bind_address 不是有效 IPv4 地址");
        }
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
            const int error = errno;
            Close();
            throw std::system_error(error, std::generic_category(), "绑定 UDP socket 失败");
        }

        // 若配置了远端地址，则直接记下对端；否则等待首个入站数据报学习对端（peer_known_ = false）。
        if (!config_.remote_address.empty()) {
            peer_.sin_family = AF_INET;
            peer_.sin_port = htons(config_.remote_port);
            if (::inet_pton(AF_INET, config_.remote_address.c_str(), &peer_.sin_addr) != 1) {
                Close();
                throw std::invalid_argument("UDP remote_address 不是有效 IPv4 地址");
            }
            peer_known_ = true;
        }
        SPDLOG_INFO("UDP 传输打开: {}", Description());
    }

    void Close() override {
        if (fd_ < 0) {
            return;  // 已关闭，幂等返回
        }
        ::close(fd_);
        fd_ = -1;
        // 保留已配置的远端；仅学习到的远端在重开时失效。
        peer_known_ = !config_.remote_address.empty();
        SPDLOG_INFO("UDP 传输关闭: {}", Description());
    }

    bool IsOpen() const override { return fd_ >= 0; }

    // 读取一个 UDP 数据报。poll 超时返回 0；一个 Read 对应一个数据报。
    // MAVLink 半包/一帧拆多包仍交给上层 MavlinkHandler 处理。
    std::ptrdiff_t Read(uint8_t* buffer, std::size_t size) override {
        if (fd_ < 0 || buffer == nullptr || size == 0) {
            return -1;
        }
        pollfd descriptor{fd_, POLLIN, 0};
        const int ready = ::poll(&descriptor, 1,
                                 static_cast<int>(config_.read_timeout.count()));
        if (ready == 0) {
            return 0;  // 超时，无数据
        }
        if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            return RecordError("UDP 等待读取失败");
        }

        // 记录来源地址，便于在未配置远端时学习对端。
        sockaddr_in source{};
        socklen_t source_length = sizeof(source);
        const ssize_t count = ::recvfrom(fd_, buffer, size, 0,
                                         reinterpret_cast<sockaddr*>(&source),
                                         &source_length);
        if (count < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;  // 被信号打断或暂时无数据，视为正常空闲
            }
            return RecordError("UDP 读取失败");
        }
        // 首个数据报到来时学习对端，之后回复走原路（peer_）。
        if (!peer_known_) {
            peer_ = source;
            peer_known_ = true;
            SPDLOG_INFO("UDP 已学习 PX4 对端: {}:{}", inet_ntoa(peer_.sin_addr),
                        ntohs(peer_.sin_port));
        }
        return count;
    }

    // 向已学习/已配置的对端发送一个数据报。UDP 无连接语义，sendto 显式指定 peer。
    bool Write(const uint8_t* data, std::size_t size) override {
        if (fd_ < 0 || data == nullptr || size == 0 || !peer_known_) {
            return false;
        }
        pollfd descriptor{fd_, POLLOUT, 0};
        const int ready = ::poll(&descriptor, 1,
                                 static_cast<int>(config_.write_timeout.count()));
        if (ready <= 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            RecordError("UDP 等待写入失败");
            return false;
        }
        const ssize_t count = ::sendto(fd_, data, size, 0,
                                       reinterpret_cast<const sockaddr*>(&peer_),
                                       sizeof(peer_));
        if (count != static_cast<ssize_t>(size)) {
            RecordError("UDP 写入失败");
            return false;
        }
        return true;
    }

    // 排空接收缓冲区：非阻塞读取直到无数据，用于关机/重连前清残留。
    void Flush() override {
        if (fd_ < 0) {
            return;
        }
        uint8_t buffer[2048];
        while (::recv(fd_, buffer, sizeof(buffer), MSG_DONTWAIT) > 0) {
        }
    }

    uint64_t ErrorCount() const override { return error_count_; }
    std::string Description() const override {
        return "udp:" + config_.bind_address + ":" +
               std::to_string(config_.bind_port);
    }

private:
    // 统一错误记账：递增计数，按第 1 次与每满 100 次节流打印，避免高频刷屏。
    std::ptrdiff_t RecordError(const char* context) {
        ++error_count_;
        if (ShouldLogThrottled(error_count_)) {
            SPDLOG_ERROR("{}: errno={} ({})，累计 {}", context, errno,
                         std::strerror(errno), error_count_);
        }
        return -1;
    }

    UdpTransportConfig config_;
    int fd_ = -1;
    sockaddr_in peer_{};
    bool peer_known_ = false;
    uint64_t error_count_ = 0;
};

}  // namespace

// 配置校验：绑定地址/端口与读写超时必须有效；远端地址与端口必须成对出现，
// 避免只配一个导致学习对端逻辑状态不一致。
void UdpTransportConfig::Validate() const {
    if (bind_address.empty() || bind_port == 0 || read_timeout.count() <= 0 ||
        write_timeout.count() <= 0) {
        throw std::invalid_argument("UDP bind 地址/端口和读写超时必须有效");
    }
    if ((!remote_address.empty() && remote_port == 0) ||
        (remote_address.empty() && remote_port != 0)) {
        throw std::invalid_argument("UDP remote_address 和 remote_port 必须同时配置或同时留空");
    }
}

std::unique_ptr<ICommunicationTransport> CreateSerialTransport(SerialPortConfig config) {
    return std::make_unique<SerialTransport>(std::move(config));
}

std::unique_ptr<ICommunicationTransport> CreateUdpTransport(UdpTransportConfig config) {
    return std::make_unique<UdpTransport>(std::move(config));
}

}  // namespace drone::communication
