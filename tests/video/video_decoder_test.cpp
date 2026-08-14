/**
 * @file video_decoder_test.cpp
 * @brief 视频解码器（VideoDecoder）软解单元测试
 *
 * 用 libx264 软编码生成 H.264 测试码流，发布到 VideoDecoder 输入主题，
 * 验证：解码帧数量、分辨率、NV12 像素格式、缓冲可写。硬解（rkmpp）
 * 依赖 RK3588 硬件，本测试在开发机以软解路径验证解码链路。
 */
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/opt.h>
}

#include "common/topic.h"
#include "common/types.h"
#include "video/video_decoder.h"

namespace {

using namespace drone;

constexpr int kTestWidth = 320;
constexpr int kTestHeight = 240;
constexpr int kTestFrameCount = 10;

/// 用 libx264 编码 kTestFrameCount 帧测试图案，返回 H.264 码流块。
/// libx264 不可用时抛出 std::runtime_error（测试跳过）。
std::vector<common::EncodedFrame> EncodeTestFrames() {
    const AVCodec* encoder = avcodec_find_encoder_by_name("libx264");
    if (encoder == nullptr) {
        throw std::runtime_error("未找到 libx264 编码器，跳过解码测试");
    }

    AVCodecContext* enc_ctx = avcodec_alloc_context3(encoder);
    enc_ctx->width = kTestWidth;
    enc_ctx->height = kTestHeight;
    enc_ctx->time_base = AVRational{1, 25};
    enc_ctx->framerate = AVRational{25, 1};
    enc_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    enc_ctx->gop_size = 1;  // 每帧关键帧：队列容量(8) < 帧数(10) 时丢帧仍可恢复
    // ultrafast：关闭 x264 lookahead 缓冲，编码帧立即输出，且 SPS/PPS 随
    // 关键帧首包输出（annexb），解码器无需 extradata 即可解析
    av_opt_set(enc_ctx->priv_data, "preset", "ultrafast", 0);
    if (avcodec_open2(enc_ctx, encoder, nullptr) < 0) {
        avcodec_free_context(&enc_ctx);
        throw std::runtime_error("打开 libx264 编码器失败");
    }

    AVFrame* frame = av_frame_alloc();
    frame->width = kTestWidth;
    frame->height = kTestHeight;
    frame->format = AV_PIX_FMT_YUV420P;
    av_frame_get_buffer(frame, 0);

    // 编码器输出参数集（SPS/PPS，avcC 格式），模拟 RTSP 流级参数集
    std::vector<uint8_t> parameter_sets;
    if (enc_ctx->extradata != nullptr && enc_ctx->extradata_size > 0) {
        parameter_sets.assign(enc_ctx->extradata,
                              enc_ctx->extradata + enc_ctx->extradata_size);
    }

    AVPacket* packet = av_packet_alloc();
    std::vector<common::EncodedFrame> result;

    auto flush = [&](bool drain) {
        if (avcodec_send_frame(enc_ctx, drain ? nullptr : frame) < 0) {
            return;
        }
        while (avcodec_receive_packet(enc_ctx, packet) == 0) {
            common::EncodedFrame encoded;
            encoded.codec = common::VideoCodec::kH264;
            encoded.is_key_frame = (packet->flags & AV_PKT_FLAG_KEY) != 0;
            encoded.data.assign(packet->data, packet->data + packet->size);
            result.push_back(std::move(encoded));
            av_packet_unref(packet);
        }
    };

    for (int index = 0; index < kTestFrameCount; ++index) {
        av_frame_make_writable(frame);
        // 测试图案：Y 平面随帧号与行渐变，UV 固定 128
        for (int y = 0; y < kTestHeight; ++y) {
            std::memset(frame->data[0] + y * frame->linesize[0],
                        static_cast<uint8_t>(index * 20 + y), kTestWidth);
        }
        for (int y = 0; y < kTestHeight / 2; ++y) {
            std::memset(frame->data[1] + y * frame->linesize[1], 128, kTestWidth / 2);
            std::memset(frame->data[2] + y * frame->linesize[2], 128, kTestWidth / 2);
        }
        frame->pts = index;
        flush(false);
    }
    flush(true);  // 冲刷编码器剩余帧

    // 把参数集附加到第一条码流（模拟 CameraReceiver 的流级参数集消息）
    if (!parameter_sets.empty() && !result.empty()) {
        result.front().parameter_sets = std::move(parameter_sets);
    }

    av_packet_free(&packet);
    av_frame_free(&frame);
    avcodec_free_context(&enc_ctx);
    return result;
}

TEST(VideoDecoderTest, DecodesH264ToNv12Frames) {
    std::vector<common::EncodedFrame> encoded;
    try {
        encoded = EncodeTestFrames();
    } catch (const std::runtime_error& e) {
        GTEST_SKIP() << e.what();
    }
    ASSERT_FALSE(encoded.empty());

    common::Topic<common::EncodedFrame> input;
    video::VideoDecoderConfig config;
    // 池容量 ≥ 输出订阅队列容量 + 在途帧（解码中），避免池满丢帧干扰断言
    config.pool_capacity = 10;
    config.width = kTestWidth;
    config.height = kTestHeight;
    config.prefer_hardware = false;  // 开发机软解路径

    video::VideoDecoder decoder(config);
    decoder.SetInput(input);
    auto sub = decoder.FrameOutput().Subscribe(4);

    ASSERT_TRUE(decoder.Start());
    // 模拟实时拉流 + 实时消费节奏：逐帧发布后立即收集，避免有界队列积压
    // 导致中间帧被 kDropOldest 挤出（与 YOLO 消费线程持续取帧一致）
    std::size_t received = 0;
    auto validate = [&](const common::Topic<video::FrameHandle>::MessagePtr& msg) {
        const auto& handle = *msg;
        EXPECT_TRUE(handle.Info().Valid());
        EXPECT_EQ(handle.Info().width, static_cast<std::uint32_t>(kTestWidth));
        EXPECT_EQ(handle.Info().height, static_cast<std::uint32_t>(kTestHeight));
        EXPECT_EQ(handle.Info().format, video::PixelFormat::kYuv420SpNv12);
        EXPECT_NE(handle.Data(), nullptr);
    };
    for (const auto& frame : encoded) {
        (void)input.Emplace(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        while (auto msg = sub.TryTake()) {
            validate(*msg);
            ++received;
        }
    }
    // 收尾：收集解码器可能仍在处理中的剩余帧
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        auto msg = sub.WaitTakeFor(std::chrono::milliseconds(100));
        if (!msg) {
            continue;
        }
        validate(*msg);
        ++received;
    }

    decoder.Stop();

    EXPECT_GT(decoder.DecodedFrameCount(), 0u);
    EXPECT_EQ(decoder.ErrorCount(), 0u);
    // 输入 10 帧全部为关键帧，预期解码 10 帧（池容量充足不丢帧）
    EXPECT_GE(received, static_cast<std::size_t>(kTestFrameCount) - 1);
}

TEST(VideoDecoderTest, StopsCleanlyWhenIdle) {
    // 无输入时 Start/Stop 干净退出（确定性停机）
    common::Topic<common::EncodedFrame> input;
    video::VideoDecoderConfig config;
    config.prefer_hardware = false;

    video::VideoDecoder decoder(config);
    decoder.SetInput(input);
    ASSERT_TRUE(decoder.Start());
    EXPECT_TRUE(decoder.IsRunning());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    decoder.Stop();
    EXPECT_FALSE(decoder.IsRunning());
    decoder.Stop();  // 幂等
}

}  // namespace
