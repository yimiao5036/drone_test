#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "communication/serial_port.h"

namespace drone::communication {

/// UDP 传输配置，主要用于 PX4 SITL；接收首个数据报后自动记住对端并原路回复。
struct UdpTransportConfig {
    std::string bind_address = "127.0.0.1";
    uint16_t bind_port = 14540;
    std::string remote_address;  ///< 可选；为空时等待首个入站数据报学习对端
    uint16_t remote_port = 0;
    std::chrono::milliseconds read_timeout{20};
    std::chrono::milliseconds write_timeout{100};

    void Validate() const;
};

/// 串口/UDP 的统一字节传输边界。每个 Link 线程独占一个实例。
class ICommunicationTransport {
public:
    virtual ~ICommunicationTransport() = default;
    virtual void Open() = 0;
    virtual void Close() = 0;
    virtual bool IsOpen() const = 0;
    virtual std::ptrdiff_t Read(uint8_t* buffer, std::size_t size) = 0;
    virtual bool Write(const uint8_t* data, std::size_t size) = 0;
    virtual void Flush() = 0;
    virtual uint64_t ErrorCount() const = 0;
    virtual std::string Description() const = 0;
};

std::unique_ptr<ICommunicationTransport> CreateSerialTransport(SerialPortConfig config);
std::unique_ptr<ICommunicationTransport> CreateUdpTransport(UdpTransportConfig config);

}  // namespace drone::communication
