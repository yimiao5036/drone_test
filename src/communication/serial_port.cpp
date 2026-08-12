/**
 * @file serial_port.cpp
 * @brief 串口封装实现（Linux termios）
 *
 * 打开/关闭/读写均基于 POSIX termios；读取使用 VMIN=0 + VTIME 实现
 * 超时读（阻塞模式）。所有 PX4、地面站电台、激光雷达串口链路复用本类。
 */
#include "communication/serial_port.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
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

    fd_ = ::open(config_.device.c_str(), O_RDWR | O_NOCTTY);
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

    // 读取超时：VMIN=0 + VTIME（单位 0.1 秒），阻塞读最多等待该时长。
    // 骨架期简化实现；精度不足时后续替换为 poll/epoll 精确超时。
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] =
        static_cast<cc_t>(config_.read_timeout.count() / 100);

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
    if (fd_ < 0 || size == 0) {
        return -1;
    }
    const ssize_t n = ::read(fd_, buffer, size);
    if (n < 0) {
        ++error_count_;
        if (ShouldLogThrottled(error_count_)) {
            SPDLOG_ERROR("串口读取错误: device={} errno={} ({})，累计 {}",
                         config_.device, errno, std::strerror(errno), error_count_);
        }
        return -1;
    }
    if (n > 0) {
        read_bytes_ += static_cast<uint64_t>(n);
    }
    return n;  // 0 = 超时无数据
}

bool SerialPort::Write(const uint8_t* data, std::size_t size) {
    if (fd_ < 0 || (data == nullptr && size > 0)) {
        return false;
    }
    std::size_t written = 0;
    while (written < size) {
        const ssize_t n = ::write(fd_, data + written, size - written);
        if (n < 0) {
            ++error_count_;
            if (ShouldLogThrottled(error_count_)) {
                SPDLOG_ERROR("串口写入错误: device={} errno={} ({})，累计 {}",
                             config_.device, errno, std::strerror(errno), error_count_);
            }
            return false;
        }
        if (n == 0) {
            ++error_count_;
            if (ShouldLogThrottled(error_count_)) {
                SPDLOG_ERROR("串口写入零字节: device={}，累计 {}", config_.device, error_count_);
            }
            return false;
        }
        written += static_cast<std::size_t>(n);
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
