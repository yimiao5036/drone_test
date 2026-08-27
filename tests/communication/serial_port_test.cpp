#include "communication/serial_port.h"

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace drone::communication {
namespace {

class PseudoTerminal final {
public:
    PseudoTerminal() {
        master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (master_fd_ < 0 || ::grantpt(master_fd_) != 0 || ::unlockpt(master_fd_) != 0) {
            const int fd = master_fd_;
            master_fd_ = -1;
            if (fd >= 0) {
                ::close(fd);
            }
            throw std::runtime_error("创建伪终端失败");
        }

        const char* name = ::ptsname(master_fd_);
        if (name == nullptr) {
            ::close(master_fd_);
            master_fd_ = -1;
            throw std::runtime_error("读取伪终端从设备名失败");
        }
        slave_name_ = name;
    }

    ~PseudoTerminal() {
        if (master_fd_ >= 0) {
            ::close(master_fd_);
        }
    }

    PseudoTerminal(const PseudoTerminal&) = delete;
    PseudoTerminal& operator=(const PseudoTerminal&) = delete;

    const std::string& SlaveName() const { return slave_name_; }

    bool WriteMaster(const uint8_t* data, std::size_t size) const {
        std::size_t written = 0;
        while (written < size) {
            const ssize_t count = ::write(master_fd_, data + written, size - written);
            if (count <= 0) {
                return false;
            }
            written += static_cast<std::size_t>(count);
        }
        return true;
    }

    std::vector<uint8_t> ReadMaster(std::size_t expected_size,
                                    std::chrono::milliseconds timeout) const {
        std::vector<uint8_t> result;
        result.reserve(expected_size);
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        while (result.size() < expected_size && std::chrono::steady_clock::now() < deadline) {
            pollfd descriptor{};
            descriptor.fd = master_fd_;
            descriptor.events = POLLIN;
            const int wait_result = ::poll(&descriptor, 1, 10);
            if (wait_result <= 0) {
                continue;
            }

            std::array<uint8_t, 256> buffer{};
            const ssize_t count = ::read(master_fd_, buffer.data(), buffer.size());
            if (count > 0) {
                result.insert(result.end(), buffer.begin(),
                              buffer.begin() + static_cast<std::ptrdiff_t>(count));
            }
        }
        return result;
    }

private:
    int master_fd_ = -1;
    std::string slave_name_;
};

SerialPortConfig MakeConfig(const std::string& device) {
    SerialPortConfig config;
    config.device = device;
    config.baud_rate = 115200;
    config.data_bits = 8;
    config.stop_bits = 1;
    config.parity = 'N';
    config.read_timeout = std::chrono::milliseconds{30};
    config.write_timeout = std::chrono::milliseconds{100};
    return config;
}

TEST(SerialPortTest, OpensAndClosesPseudoTerminalIdempotently) {
    PseudoTerminal terminal;
    SerialPort port(MakeConfig(terminal.SlaveName()));

    EXPECT_FALSE(port.IsOpen());
    port.Open();
    EXPECT_TRUE(port.IsOpen());
    port.Open();
    EXPECT_TRUE(port.IsOpen());
    port.Close();
    EXPECT_FALSE(port.IsOpen());
    port.Close();
}

TEST(SerialPortTest, ReadsBytesWrittenByPeer) {
    PseudoTerminal terminal;
    SerialPort port(MakeConfig(terminal.SlaveName()));
    port.Open();

    const std::array<uint8_t, 6> expected{0xFD, 0x01, 0x02, 0x03, 0x04, 0x05};
    ASSERT_TRUE(terminal.WriteMaster(expected.data(), expected.size()));

    std::array<uint8_t, 32> actual{};
    const auto count = port.Read(actual.data(), actual.size());
    ASSERT_EQ(count, static_cast<std::ptrdiff_t>(expected.size()));
    EXPECT_TRUE(std::equal(expected.begin(), expected.end(), actual.begin()));
    EXPECT_EQ(port.ReadBytes(), expected.size());
}

TEST(SerialPortTest, WritesCompleteBufferToPeer) {
    PseudoTerminal terminal;
    SerialPort port(MakeConfig(terminal.SlaveName()));
    port.Open();

    const std::array<uint8_t, 8> expected{0xFD, 0x06, 0x07, 0x08,
                                          0x09, 0x0A, 0x0B, 0x0C};
    ASSERT_TRUE(port.Write(expected.data(), expected.size()));

    const auto actual = terminal.ReadMaster(expected.size(), std::chrono::milliseconds{200});
    EXPECT_EQ(actual, std::vector<uint8_t>(expected.begin(), expected.end()));
    EXPECT_EQ(port.WriteBytes(), expected.size());
}

TEST(SerialPortTest, ReadReturnsZeroAfterConfiguredTimeout) {
    PseudoTerminal terminal;
    auto config = MakeConfig(terminal.SlaveName());
    config.read_timeout = std::chrono::milliseconds{30};
    SerialPort port(config);
    port.Open();

    std::array<uint8_t, 8> buffer{};
    const auto started = std::chrono::steady_clock::now();
    const auto count = port.Read(buffer.data(), buffer.size());
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started);

    EXPECT_EQ(count, 0);
    EXPECT_GE(elapsed.count(), 15);
    EXPECT_LT(elapsed.count(), 500);
    EXPECT_EQ(port.ErrorCount(), 0u);
}

TEST(SerialPortTest, SupportsRestartAfterClose) {
    PseudoTerminal terminal;
    SerialPort port(MakeConfig(terminal.SlaveName()));

    port.Open();
    port.Close();
    port.Open();

    const std::array<uint8_t, 3> expected{0x01, 0x02, 0x03};
    ASSERT_TRUE(terminal.WriteMaster(expected.data(), expected.size()));
    std::array<uint8_t, 3> actual{};
    ASSERT_EQ(port.Read(actual.data(), actual.size()),
              static_cast<std::ptrdiff_t>(expected.size()));
    EXPECT_EQ(actual, expected);
}

TEST(SerialPortTest, RejectsNegativeWriteTimeout) {
    SerialPortConfig config;
    config.write_timeout = std::chrono::milliseconds{-1};
    EXPECT_THROW(config.Validate(), std::invalid_argument);
}

}  // namespace
}  // namespace drone::communication
