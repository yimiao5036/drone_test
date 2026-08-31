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

bool ShouldLogThrottled(uint64_t count) {
    return count == 1 || count % 100 == 0;
}

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

    void Open() override {
        if (fd_ >= 0) {
            return;
        }
        fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) {
            throw std::system_error(errno, std::generic_category(), "创建 UDP socket 失败");
        }
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
            return;
        }
        ::close(fd_);
        fd_ = -1;
        peer_known_ = !config_.remote_address.empty();
        SPDLOG_INFO("UDP 传输关闭: {}", Description());
    }

    bool IsOpen() const override { return fd_ >= 0; }

    std::ptrdiff_t Read(uint8_t* buffer, std::size_t size) override {
        if (fd_ < 0 || buffer == nullptr || size == 0) {
            return -1;
        }
        pollfd descriptor{fd_, POLLIN, 0};
        const int ready = ::poll(&descriptor, 1,
                                 static_cast<int>(config_.read_timeout.count()));
        if (ready == 0) {
            return 0;
        }
        if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            return RecordError("UDP 等待读取失败");
        }

        sockaddr_in source{};
        socklen_t source_length = sizeof(source);
        const ssize_t count = ::recvfrom(fd_, buffer, size, 0,
                                         reinterpret_cast<sockaddr*>(&source),
                                         &source_length);
        if (count < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;
            }
            return RecordError("UDP 读取失败");
        }
        if (!peer_known_) {
            peer_ = source;
            peer_known_ = true;
            SPDLOG_INFO("UDP 已学习 PX4 对端: {}:{}", inet_ntoa(peer_.sin_addr),
                        ntohs(peer_.sin_port));
        }
        return count;
    }

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
