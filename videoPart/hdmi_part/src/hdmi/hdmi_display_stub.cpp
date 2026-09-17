/**
 * @file hdmi_display_stub.cpp
 * @brief HDMI 显示设备占位实现（无 DRM 环境可用）
 *
 * 用途：在无 /dev/dri 的开发机（如 WSL2 Ubuntu）上让 HDMI 直显后端可以完整
 * 走完 Start / EncodeFrame / Stop 生命周期，从而验证：
 * - VideoSender 与后端的接线、线程与计数是否正确；
 * - 输入帧校验与尺寸不匹配的丢帧路径是否生效；
 * - 延迟统计接口是否有值。
 *
 * 不做：不接触任何图形设备，不产生真实 HDMI 输出。上机必须使用
 * hdmi_drm_display.cpp（编译期由 DRONE_HAVE_HDMI_KMS 决定）。
 *
 * 与 video_decoder_stub.cpp / camera_receiver_stub.cpp 的约定保持一致：
 * 生命周期方法记录 INFO 日志，业务方法直接返回成功并累加计数。
 */
#include "video_transmission/hdmi/hdmi_display_backend.h"

#include <cstdint>
#include <memory>
#include <string>

#include <spdlog/spdlog.h>

namespace drone::video_transmission {

namespace {

/// 占位显示器：只维护状态与计数，不做任何真实输出。
class HdmiDisplayStub final : public IHdmiDisplay {
public:
    bool Open(const HdmiDisplayConfig& config) override {
        if (open_) {
            return true;  // 幂等
        }
        config_ = config;
        open_ = true;
        SPDLOG_INFO("HDMI 显示占位实现打开: {}x{}@{}Hz（开发机无 DRM 设备，不会真实输出）",
                    config.width, config.height, config.refresh_hz);
        return true;
    }

    void Close() override {
        if (!open_) {
            return;
        }
        open_ = false;
        SPDLOG_INFO("HDMI 显示占位实现关闭: 累计上屏 {} 帧, 丢弃 {} 帧",
                    submitted_count_, skipped_count_);
    }

    bool IsOpen() const override { return open_; }

    HdmiSubmitResult Submit(const std::uint8_t* data, std::uint32_t width,
                            std::uint32_t height,
                            std::uint32_t hor_stride) override {
        if (!open_) {
            return HdmiSubmitResult::kFailed;
        }
        // 与真实实现一致地校验入参，保证开发机也能暴露调用方的参数错误。
        if (data == nullptr || width == 0 || height == 0 || hor_stride < width) {
            ++invalid_count_;
            return HdmiSubmitResult::kFailed;
        }
        ++submitted_count_;
        return HdmiSubmitResult::kSubmitted;
    }

    common::LatencySummary FlipLatency() const override { return flip_latency_.Snapshot(); }

    common::LatencySummary PrepareLatency() const override {
        return prepare_latency_.Snapshot();
    }

    std::uint64_t SkippedFrameCount() const override { return skipped_count_; }

    std::string ActiveModeName() const override {
        return std::to_string(config_.width) + "x" + std::to_string(config_.height) +
               "@" + std::to_string(config_.refresh_hz) + "(占位)";
    }

private:
    HdmiDisplayConfig config_;
    bool open_ = false;
    std::uint64_t submitted_count_ = 0;
    std::uint64_t skipped_count_ = 0;
    std::uint64_t invalid_count_ = 0;
    common::LatencyStatistics flip_latency_;
    common::LatencyStatistics prepare_latency_;
};

}  // namespace

std::unique_ptr<IHdmiDisplay> CreateHdmiDisplayStub() {
    return std::make_unique<HdmiDisplayStub>();
}

}  // namespace drone::video_transmission
