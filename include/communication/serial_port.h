/**
 * @file serial_port.h
 * @brief 串口封装（SerialPort）
 *
 * 属于 drone/communication 模块。所有串口链路（PX4、地面站电台、
 * 激光雷达）统一通过本类收发，内部使用 Linux termios。
 *
 * 错误处理约定（对应 AGENTS.md）：
 * - 构造与 Open 失败：抛 std::system_error，由上层决定如何处理。
 * - 运行时读写错误：返回错误码/读取字节数，通过 ErrorCount() 监控，
 *   不抛异常（避免影响实时性）。
 * - 关键路径日志：创建/销毁/打开成功（INFO），读写错误（ERROR 节流）。
 *
 * 线程安全：本类为单线程使用模型（每个串口链路由一个线程独占），
 * 内部不加锁；多线程共享时由调用方串行化。
 */
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace drone::communication {

/// 串口参数配置。
struct SerialPortConfig {
    std::string device = "/dev/ttyS0";       ///< 串口设备路径
    int baud_rate = 115200;                  ///< 波特率
    uint8_t data_bits = 8;                   ///< 数据位（5/6/7/8）
    uint8_t stop_bits = 1;                   ///< 停止位（1/2）
    char parity = 'N';                       ///< 校验位：N=无 E=偶 O=奇
    std::chrono::milliseconds read_timeout{100};  ///< 单次读取超时

    /// 参数合法性校验；非法时抛出 std::invalid_argument。
    void Validate() const;
};

/// 串口封装（RAII：析构自动关闭）。
class SerialPort {
public:
    explicit SerialPort(SerialPortConfig config);
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&& other) noexcept;
    SerialPort& operator=(SerialPort&& other) noexcept;

    /// 打开串口并配置 termios。
    /// @throw std::system_error 打开或配置失败。
    void Open();

    /// 关闭串口；幂等。析构自动调用。
    void Close();

    /// 是否已打开。
    [[nodiscard]] bool IsOpen() const;

    /// 读取最多 size 字节。
    /// @return 实际读取字节数；0 表示超时无数据；-1 表示错误（记入 ErrorCount）。
    [[nodiscard]] std::ptrdiff_t Read(uint8_t* buffer, std::size_t size);

    /// 写入全部字节。
    /// @return 是否全部写入成功；失败时记入 ErrorCount。
    bool Write(const uint8_t* data, std::size_t size);

    /// 丢弃收发缓冲区中的残留数据（用于重同步）。
    void Flush();

    /// 累计读取字节数。
    [[nodiscard]] uint64_t ReadBytes() const { return read_bytes_; }
    /// 累计写入字节数。
    [[nodiscard]] uint64_t WriteBytes() const { return write_bytes_; }
    /// 累计错误次数。
    [[nodiscard]] uint64_t ErrorCount() const { return error_count_; }

    /// 当前配置（只读）。
    [[nodiscard]] const SerialPortConfig& Config() const { return config_; }

private:
    /// 关闭底层 fd；不更新日志与状态（供移动语义使用）。
    void CloseFd() noexcept;

    SerialPortConfig config_;
    int fd_ = -1;
    uint64_t read_bytes_ = 0;
    uint64_t write_bytes_ = 0;
    uint64_t error_count_ = 0;
};

}  // namespace drone::communication
