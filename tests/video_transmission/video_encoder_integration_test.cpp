/**
 * @file video_encoder_integration_test.cpp
 * @brief 编码后端（FFmpeg）开发机集成验证
 *
 * 在开发机（无 rkmpp）用软件编码（libx264）将合成 NV12 帧编码到本地
 * MPEG-TS 文件，验证：编码器可打开、帧送入/取出、文件产出有效码流、计数正确。
 *
 * 说明：
 * - 本测试需要 FFmpeg 带 libx264 编码器（开发机已具备）。
 * - RTSP 推流、rkmpp 硬编码需真实图传目标/香橙派，留待实机验证。
 * - 输出文件写入 /tmp，测试自清理。
 */
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "video/video_frame_pool.h"
#include "video_transmission/video_encoder.h"

namespace drone::video_transmission {
namespace {

using testing::Test;

class VideoEncoderIntegrationTest : public Test {
protected:
    void SetUp() override {
        // 编码输入帧池（NV12，128x128）
        video::VideoFrameInfo tmpl;
        tmpl.width = 128;
        tmpl.height = 128;
        tmpl.hor_stride = 128;
        tmpl.ver_stride = 128;
        tmpl.format = video::PixelFormat::kYuv420SpNv12;
        pool_ = std::make_shared<video::VideoFramePool>(4, tmpl);
    }

    std::string OutputPath() const { return "/tmp/video_encoder_itest.ts"; }

    std::shared_ptr<video::VideoFramePool> pool_;
};

TEST_F(VideoEncoderIntegrationTest, EncodesSyntheticFramesToTsFile) {
    EncoderBackendConfig config;
    config.url = OutputPath();
    config.output_url = OutputPath();
    config.output_format = "mpegts";   // 文件输出（开发机无 RTSP 服务器）
    config.codec = "h264";
    config.width = 128;
    config.height = 128;
    config.fps = 25;
    config.prefer_hardware = false;    // 开发机：软编码
    config.gop = 25;

    auto backend = CreateVideoEncoderBackend(config);
    ASSERT_NE(backend, nullptr);
    ASSERT_TRUE(backend->Start());
    EXPECT_TRUE(backend->IsRunning());

    // 送 20 帧合成 NV12
    for (int i = 0; i < 20; ++i) {
        auto handle = pool_->Acquire();
        ASSERT_TRUE(handle.Valid());
        std::memset(handle.Data(), static_cast<int>((i * 13) % 256),
                    pool_->SlotSize());
        EXPECT_TRUE(backend->EncodeFrame(handle));
    }

    backend->Stop();
    EXPECT_GT(backend->SentFrameCount(), 0u);
    EXPECT_EQ(backend->ErrorCount(), 0u);

    // 文件已生成且非空
    std::FILE* f = std::fopen(OutputPath().c_str(), "rb");
    ASSERT_NE(f, nullptr);
    std::fseek(f, 0, SEEK_END);
    long size = std::ftell(f);
    std::fclose(f);
    std::remove(OutputPath().c_str());
    EXPECT_GT(size, 0L);
}

}  // namespace
}  // namespace drone::video_transmission
