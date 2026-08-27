/**
 * @file serial_port.cpp
 * @brief 串口封装实现（Linux termios）
 *
 * 打开/关闭基于 POSIX termios；fd 使用非阻塞模式，读写通过 poll()
 * 和单调时钟实现毫秒级超时。所有 PX4、地面站电台、激光雷达串口链路复用本类。
 */
#include "communication/serial_port.h"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <climits>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <spdlog/spdlog.h>

namespace drone::communication {

namespace {

/// 波特率 → termios 常量映射；不支持的返回 B0。
speed_t ToBaudConstant(int baud) {
    switch (baud) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 921600:
            return B921600;
        default:
            return B0;
    }
}

/// 异常日志节流：第 1 次与每满 100 次才打印，避免高频错误刷屏。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

/// 使用单调时钟等待 fd 就绪，避免 termios VTIME 只能按 100ms 取整。
/// 返回 1=就绪，0=超时，-1=错误；EINTR 会在剩余超时内继续等待。
int WaitForFd(int fd, short events, std::chrono::milliseconds timeout,
              short* returned_events) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const auto remaining = now >= deadline
                                   ? std::chrono::milliseconds{0}
                                   : std::chrono::duration_cast<std::chrono::milliseconds>(
                                         deadline - now);
        const auto remaining_count = remaining.count();
        const int timeout_ms = remaining_count > INT_MAX
                                   ? INT_MAX
                                   : static_cast<int>(remaining_count);

        pollfd descriptor{};
        descriptor.fd = fd;
        descriptor.events = events;
        const int result = ::poll(&descriptor, 1, timeout_ms);
        if (result > 0) {
            if (returned_events != nullptr) {
                *returned_events = descriptor.revents;
            }
            return 1;
        }
        if (result == 0) {
            return 0;
        }
        if (errno != EINTR) {
            return -1;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            return 0;
        }
    }
}

}  // namespace

void SerialPortConfig::Validate() const {
    if (device.empty()) {
        throw std::invalid_argument("串口设备路径不能为空");
    }
    if (baud_rate <= 0) {
        throw std::invalid_argument("串口波特率必须为正数");
    }
    if (data_bits < 5 || data_bits > 8) {
        throw std::invalid_argument("串口数据位必须是 5~8");
    }
    if (stop_bits != 1 && stop_bits != 2) {
        throw std::invalid_argument("串口停止位必须是 1 或 2");
    }
    if (parity != 'N' && parity != 'E' && parity != 'O') {
        throw std::invalid_argument("串口校验位必须是 N/E/O");
    }
    if (read_timeout.count() < 0) {
        throw std::invalid_argument("串口读取超时不能为负");
    }
    if (write_timeout.count() < 0) {
        throw std::invalid_argument("串口写入超时不能为负");
    }
}

SerialPort::SerialPort(SerialPortConfig config) : config_(std::move(config)) {
    config_.Validate();
    SPDLOG_INFO("串口对象创建: device={} baud={} 数据位={} 停止位={} 校验={}",
                config_.device, config_.baud_rate, config_.data_bits,
                config_.stop_bits, config_.parity);
}

SerialPort::~SerialPort() {
    if (fd_ >= 0) {
        Close();
    }
    SPDLOG_INFO("串口对象销毁: device={}", config_.device);
}

SerialPort::SerialPort(SerialPort&& other) noexcept
    : config_(std::move(other.config_)),
      fd_(other.fd_),
      read_bytes_(other.read_bytes_),
      write_bytes_(other.write_bytes_),
      error_count_(other.error_count_) {
    other.fd_ = -1;
}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
    if (this != &other) {
        if (fd_ >= 0) {
            Close();
        }
        config_ = std::move(other.config_);
        fd_ = other.fd_;
        read_bytes_ = other.read_bytes_;
        write_bytes_ = other.write_bytes_;
        error_count_ = other.error_count_;
        other.fd_ = -1;
    }
    return *this;
}

void SerialPort::Open() {
    if (fd_ >= 0) {
        return;  // 已打开，幂等
    }

    fd_ = ::open(config_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
        const int err = errno;
        throw std::system_error(err, std::generic_category(),
                                "打开串口失败: " + config_.device);
    }

    termios options{};
    if (::tcgetattr(fd_, &options) != 0) {
        const int err = errno;
        CloseFd();
        throw std::system_error(err, std::generic_category(),
                                "读取串口属性失败: " + config_.device);
    }

    ::cfmakeraw(&options);
    options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
#ifdef CRTSCTS
    options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    options.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));

    const speed_t baud = ToBaudConstant(config_.baud_rate);
    if (baud == B0) {
        CloseFd();
        throw std::invalid_argument("不支持的波特率: " + std::to_string(config_.baud_rate));
    }
    ::cfsetispeed(&options, baud);
    ::cfsetospeed(&options, baud);

    // 数据位
    options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    switch (config_.data_bits) {
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
        default:
            options.c_cflag |= CS8;
            break;
    }

    // 校验位
    if (config_.parity == 'N') {
        options.c_cflag &= static_cast<tcflag_t>(~PARENB);
    } else if (config_.parity == 'E') {
        options.c_cflag |= PARENB;
        options.c_cflag &= static_cast<tcflag_t>(~PARODD);
    } else {  // 'O'
        options.c_cflag |= PARENB;
        options.c_cflag |= PARODD;
    }

    // 停止位
    if (config_.stop_bits == 2) {
        options.c_cflag |= CSTOPB;
    } else {
        options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    }

    // fd 使用非阻塞模式，读取/写入超时统一由 poll() 精确控制。
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (::tcsetattr(fd_, TCSANOW, &options) != 0) {
        const int err = errno;
        CloseFd();
        throw std::system_error(err, std::generic_category(),
                                "配置串口属性失败: " + config_.device);
    }

    SPDLOG_INFO("串口打开成功: device={} baud={}", config_.device, config_.baud_rate);
}

void SerialPort::Close() {
    if (fd_ < 0) {
        return;
    }
    CloseFd();
    SPDLOG_INFO("串口关闭: device={}", config_.device);
}

bool SerialPort::IsOpen() const {
    return fd_ >= 0;
}

std::ptrdiff_t SerialPort::Read(uint8_t* buffer, std::size_t size) {
    if (fd_ < 0 || buffer == nullptr || size == 0) {
        return -1;
    }

    short events = 0;
    const int wait_result = WaitForFd(fd_, POLLIN, config_.read_timeout, &events);
    if (wait_result == 0) {
        return 0;
    }
    if (wait_result < 0 || (events & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        ++error_count_;
        if (ShouldLogThrottled(error_count_)) {
            SPDLOG_ERROR("串口等待读取失败: device={} events={} errno={} ({})，累计 {}",
                         config_.device, events, errno, std::strerror(errno), error_count_);
        }
        return -1;
    }

    while (true) {
        const ssize_t n = ::read(fd_, buffer, size);
        if (n > 0) {
            read_bytes_ += static_cast<uint64_t>(n);
            return n;
        }
        if (n == 0) {
            return 0;
        }
        if (errno == EINTR) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }

        ++error_count_;
        if (ShouldLogThrottled(error_count_)) {
            SPDLOG_ERROR("串口读取错误: device={} errno={} ({})，累计 {}",
                         config_.device, errno, std::strerror(errno), error_count_);
        }
        return -1;
    }
}

bool SerialPort::Write(const uint8_t* data, std::size_t size) {
    if (fd_ < 0 || (data == nullptr && size > 0)) {
        return false;
    }
    if (size == 0) {
        return true;
    }

    const auto deadline = std::chrono::steady_clock::now() + config_.write_timeout;
    std::size_t written = 0;
    while (written < size) {
        const auto now = std::chrono::steady_clock::now();
        const auto remaining = now >= deadline
                                   ? std::chrono::milliseconds{0}
                                   : std::chrono::duration_cast<std::chrono::milliseconds>(
                                         deadline - now);
        short events = 0;
        const int wait_result = WaitForFd(fd_, POLLOUT, remaining, &events);
        if (wait_result <= 0 || (events & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            ++error_count_;
            if (ShouldLogThrottled(error_count_)) {
                if (wait_result == 0) {
                    SPDLOG_ERROR("串口写入超时: device={} 已写入={}/{}，累计 {}",
                                 config_.device, written, size, error_count_);
                } else {
                    SPDLOG_ERROR("串口等待写入失败: device={} events={} errno={} ({})，累计 {}",
                                 config_.device, events, errno, std::strerror(errno), error_count_);
                }
            }
            return false;
        }

        const ssize_t n = ::write(fd_, data + written, size - written);
        if (n > 0) {
            written += static_cast<std::size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;
        }

        ++error_count_;
        if (ShouldLogThrottled(error_count_)) {
            SPDLOG_ERROR("串口写入错误: device={} errno={} ({})，累计 {}",
                         config_.device, errno, std::strerror(errno), error_count_);
        }
        return false;
    }

    write_bytes_ += written;
    return true;
}

void SerialPort::Flush() {
    if (fd_ < 0) {
        return;
    }
    ::tcflush(fd_, TCIOFLUSH);
}

void SerialPort::CloseFd() noexcept {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

}  // namespace drone::communication
