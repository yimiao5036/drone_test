// =============================================================================
// stereo_ranger_test —— 双目测距 v2 组件壳单测
//
// 覆盖：三路输入接线校验、帧/检测批次配对（settle 闭合与新序号闭合）、
//   每框一条发布、右帧缺失降级（valid=false）、字段映射（帧序号/框/置信度）、
//   Stop 幂等与可重启、摘要静默抑制（静默窗口不打印，活动恢复照常打印）。
//
// 帧源用真实内存池（video::VideoFramePool）；输入图用黄金场景渲染图
// （tests/perception/data/stereo_*.bin），端到端期望值与参考实现一致。
// =============================================================================

#include "perception/stereo_ranger.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "../common/log_capture.h"
#include "common/topic.h"
#include "data/stereo_golden.h"
#include "video/video_frame_pool.h"

namespace drone::perception {
namespace {

using namespace std::chrono_literals;
using testdata::kViewHeight;
using testdata::kViewWidth;

constexpr double kExpectedZ = 0.988;  // 黄金场景（参考实现聚合值）

/// 轮询等待条件成立（最长 timeout_ms）。
template <typename F>
bool WaitFor(F&& pred, int timeout_ms = 2000) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
}

StereoRangerConfig TestConfig() {
    StereoRangerConfig cfg;
    cfg.disp_min_px = 4.0;
    cfg.disp_max_px = 40.0;
    cfg.warmup_frames = 0;            // 单帧用例不做预热丢弃
    cfg.pair_window = 50ms;
    cfg.settle = 30ms;                // 缩短批次闭合等待
    cfg.summary_log_interval = 24h;   // 测试不打摘要
    return cfg;
}

std::vector<std::uint8_t> LoadBin(const std::string& name) {
    const std::string path =
        std::string(TEST_SOURCE_DIR) + "/tests/perception/data/" + name;
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        throw std::runtime_error("无法打开黄金数据: " + path);
    }
    std::vector<std::uint8_t> data(
        static_cast<std::size_t>(kViewWidth) * kViewHeight);
    input.read(reinterpret_cast<char*>(data.data()),
               static_cast<std::streamsize>(data.size()));
    return data;
}

video::VideoFrameInfo FrameTemplate() {
    video::VideoFrameInfo info;
    info.width = kViewWidth;
    info.height = kViewHeight;
    info.hor_stride = kViewWidth;
    info.ver_stride = kViewHeight;
    info.format = video::PixelFormat::kYuv420SpNv12;
    return info;
}

common::DetectionResult MakeBox(double x0, double y0, double x1, double y1,
                                std::uint32_t class_id = 0) {
    common::DetectionResult box;
    box.class_id = class_id;
    box.confidence = 0.85f;
    box.bbox_x = static_cast<float>(x0);
    box.bbox_y = static_cast<float>(y0);
    box.bbox_w = static_cast<float>(x1 - x0);
    box.bbox_h = static_cast<float>(y1 - y0);
    return box;
}

struct Fixture {
    StereoRangerConfig cfg = TestConfig();
    common::Topic<common::DetectionResult> detection_topic;
    common::Topic<video::FrameHandle> left_topic;
    common::Topic<video::FrameHandle> right_topic;
    std::shared_ptr<video::VideoFramePool> left_pool;
    std::shared_ptr<video::VideoFramePool> right_pool;
    StereoRanger ranger;

    explicit Fixture(StereoRangerConfig config = TestConfig())
        : cfg(std::move(config)),
          left_pool(std::make_shared<video::VideoFramePool>(6, FrameTemplate())),
          right_pool(std::make_shared<video::VideoFramePool>(6, FrameTemplate())),
          ranger(cfg, testdata::Matrices()) {
        ranger.SetDetectionInput(detection_topic);
        ranger.SetLeftFrameInput(left_topic);
        ranger.SetRightFrameInput(right_topic);
    }

    /// 发布一帧（Y 平面填充给定灰度；其余平面留 0）。
    std::uint64_t PublishFrame(const std::shared_ptr<video::VideoFramePool>& pool,
                               common::Topic<video::FrameHandle>& topic,
                               const std::vector<std::uint8_t>& gray,
                               std::int64_t timestamp_ms) {
        auto handle = pool->Acquire(0);
        if (!handle.Valid()) {
            throw std::runtime_error("测试内存池耗尽");
        }
        std::uint8_t* data = reinterpret_cast<std::uint8_t*>(handle.Data());
        if (gray.size() != static_cast<std::size_t>(kViewWidth) * kViewHeight) {
            throw std::runtime_error("测试灰度图尺寸不符");
        }
        std::copy(gray.begin(), gray.end(), data);
        handle.SetTiming(timestamp_ms, 0);
        const std::uint64_t sequence = handle.Info().sequence;
        (void)topic.Emplace(std::move(handle));
        return sequence;
    }
};

// ---------------------------------------------------------------------------
// 启动接线校验
// ---------------------------------------------------------------------------
TEST(StereoRangerTest, StartRequiresAllThreeInputs) {
    StereoRangerConfig cfg = TestConfig();
    common::Topic<common::DetectionResult> detection_topic;
    common::Topic<video::FrameHandle> left_topic;
    StereoRanger ranger(cfg, testdata::Matrices());
    ranger.SetDetectionInput(detection_topic);
    ranger.SetLeftFrameInput(left_topic);  // 右目未接线
    EXPECT_FALSE(ranger.Start());
    EXPECT_FALSE(ranger.IsRunning());
    EXPECT_GE(ranger.ErrorCount(), 1u);
}

// ---------------------------------------------------------------------------
// 端到端：帧 + 检测 → 每框一条有效测距（settle 超时闭合批次）
// ---------------------------------------------------------------------------
TEST(StereoRangerTest, PublishesValidDistancePerBoxAfterSettle) {
    Fixture f;
    auto out_sub = f.ranger.DistanceOutput().Subscribe(4);
    ASSERT_TRUE(f.ranger.Start());

    const auto ref = LoadBin("stereo_ref.bin");
    const auto aux = LoadBin("stereo_aux.bin");
    const std::int64_t ts = 1000;
    const std::uint64_t left_seq = f.PublishFrame(f.left_pool, f.left_topic, ref, ts);
    (void)f.PublishFrame(f.right_pool, f.right_topic, aux, ts + 1);

    common::DetectionResult box = MakeBox(20, 20, kViewWidth - 15, kViewHeight - 5);
    box.frame_sequence = left_seq;  // 检测的帧序号 = 左目帧序号
    (void)f.detection_topic.Emplace(box);

    const auto msg = out_sub.WaitTakeFor(2s);
    ASSERT_TRUE(msg.has_value());
    EXPECT_TRUE((*msg)->valid);
    EXPECT_NEAR((*msg)->forward_distance_m, kExpectedZ, 0.08);
    EXPECT_GT((*msg)->disparity_px, 10.0f);
    EXPECT_LT((*msg)->disparity_px, 16.0f);
    // 相机系三维位置：z=forward_distance_m，x/y 有限且量级合理（黄金场景近距）
    EXPECT_TRUE(std::isfinite((*msg)->camera_x_m));
    EXPECT_TRUE(std::isfinite((*msg)->camera_y_m));
    EXPECT_NEAR((*msg)->camera_x_m, 0.0f, (*msg)->forward_distance_m);
    EXPECT_NEAR((*msg)->camera_y_m, 0.0f, (*msg)->forward_distance_m);
    EXPECT_EQ((*msg)->frame_sequence, left_seq);
    EXPECT_FLOAT_EQ((*msg)->bbox_x, 20.f);
    EXPECT_FLOAT_EQ((*msg)->bbox_w, static_cast<float>(kViewWidth - 35));
    EXPECT_FLOAT_EQ((*msg)->confidence, 0.85f);
    EXPECT_EQ((*msg)->header.sequence, 1u);
    EXPECT_GT((*msg)->header.receive_time_ms, 0u);
    EXPECT_EQ((*msg)->header.source_time_ms, static_cast<std::uint64_t>(ts));

    f.ranger.Stop();
    EXPECT_EQ(f.ranger.BatchCount(), 1u);
    EXPECT_GE(f.ranger.AvailableCount(), 1u);
    EXPECT_EQ(f.ranger.FrameMissCount(), 0u);
}

// ---------------------------------------------------------------------------
// 每框一条：同帧多个检测框 → 多条消息
// ---------------------------------------------------------------------------
TEST(StereoRangerTest, PublishesOneMessagePerBox) {
    Fixture f;
    auto out_sub = f.ranger.DistanceOutput().Subscribe(8);
    ASSERT_TRUE(f.ranger.Start());

    const auto ref = LoadBin("stereo_ref.bin");
    const auto aux = LoadBin("stereo_aux.bin");
    const std::int64_t ts = 2000;
    const std::uint64_t left_seq = f.PublishFrame(f.left_pool, f.left_topic, ref, ts);
    (void)f.PublishFrame(f.right_pool, f.right_topic, aux, ts);

    auto box1 = MakeBox(20, 20, 160, 230);
    box1.frame_sequence = left_seq;
    auto box2 = MakeBox(150, 20, kViewWidth - 15, 230);
    box2.frame_sequence = left_seq;
    (void)f.detection_topic.Emplace(box1);
    (void)f.detection_topic.Emplace(box2);

    int received = 0;
    for (int i = 0; i < 2; ++i) {
        const auto msg = out_sub.WaitTakeFor(2s);
        ASSERT_TRUE(msg.has_value()) << "第 " << i + 1 << " 条未收到";
        EXPECT_EQ((*msg)->frame_sequence, left_seq);
        ++received;
    }
    EXPECT_EQ(received, 2);
    EXPECT_FALSE(out_sub.WaitTakeFor(100ms).has_value());  // 恰好两条
    f.ranger.Stop();
}

// ---------------------------------------------------------------------------
// 新序号到达 ⇒ 上批闭合（无需等 settle）
// ---------------------------------------------------------------------------
TEST(StereoRangerTest, NewSequenceClosesPreviousBatch) {
    Fixture f;
    auto out_sub = f.ranger.DistanceOutput().Subscribe(8);
    ASSERT_TRUE(f.ranger.Start());

    const auto ref = LoadBin("stereo_ref.bin");
    const auto aux = LoadBin("stereo_aux.bin");
    const std::uint64_t seq_a = f.PublishFrame(f.left_pool, f.left_topic, ref, 3000);
    (void)f.PublishFrame(f.right_pool, f.right_topic, aux, 3000);
    const std::uint64_t seq_b = f.PublishFrame(f.left_pool, f.left_topic, ref, 3100);
    (void)f.PublishFrame(f.right_pool, f.right_topic, aux, 3100);

    auto box_a = MakeBox(20, 20, 160, 230);
    box_a.frame_sequence = seq_a;
    (void)f.detection_topic.Emplace(box_a);
    auto box_b = MakeBox(20, 20, 160, 230);
    box_b.frame_sequence = seq_b;  // 新序号 ⇒ 批 a 立即闭合
    (void)f.detection_topic.Emplace(box_b);

    const auto first = out_sub.WaitTakeFor(2s);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ((*first)->frame_sequence, seq_a);
    const auto second = out_sub.WaitTakeFor(2s);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ((*second)->frame_sequence, seq_b);
    f.ranger.Stop();
    EXPECT_EQ(f.ranger.BatchCount(), 2u);
}

// ---------------------------------------------------------------------------
// 右帧缺失：按无效发布（valid=false，距离 NaN），计数并保持节拍
// ---------------------------------------------------------------------------
TEST(StereoRangerTest, MissingRightFramePublishesInvalid) {
    Fixture f;
    auto out_sub = f.ranger.DistanceOutput().Subscribe(4);
    ASSERT_TRUE(f.ranger.Start());

    // 先发一个"过新"的右帧（超出配对窗）：右缓冲最新帧已超过左帧时间 +
    // pair_window，组件据此判定该批次等不到右帧，立即降级。
    const auto aux = LoadBin("stereo_aux.bin");
    (void)f.PublishFrame(f.right_pool, f.right_topic, aux, 5000);
    std::this_thread::sleep_for(10ms);
    const auto ref = LoadBin("stereo_ref.bin");
    const std::uint64_t left_seq = f.PublishFrame(f.left_pool, f.left_topic, ref, 4000);

    auto box = MakeBox(20, 20, kViewWidth - 15, kViewHeight - 5);
    box.frame_sequence = left_seq;
    (void)f.detection_topic.Emplace(box);

    const auto msg = out_sub.WaitTakeFor(2s);
    ASSERT_TRUE(msg.has_value());
    EXPECT_FALSE((*msg)->valid);
    EXPECT_TRUE(std::isnan((*msg)->forward_distance_m));
    EXPECT_TRUE(std::isnan((*msg)->disparity_px));
    EXPECT_TRUE(std::isnan((*msg)->camera_x_m));  // 无效帧相机系坐标不可用
    EXPECT_TRUE(std::isnan((*msg)->camera_y_m));
    EXPECT_EQ((*msg)->frame_sequence, left_seq);
    EXPECT_FLOAT_EQ((*msg)->bbox_x, 20.f);  // 方位字段照常发布
    f.ranger.Stop();
    EXPECT_GE(f.ranger.FrameMissCount(), 1u);
    EXPECT_EQ(f.ranger.AvailableCount(), 0u);
}

// ---------------------------------------------------------------------------
// Stop 幂等与可重启
// ---------------------------------------------------------------------------
TEST(StereoRangerTest, StopIsIdempotentAndRestartable) {
    Fixture f;
    ASSERT_TRUE(f.ranger.Start());
    EXPECT_FALSE(f.ranger.Start());  // 重复启动返回 false，状态不变
    f.ranger.Stop();
    f.ranger.Stop();  // 幂等
    EXPECT_FALSE(f.ranger.IsRunning());
    EXPECT_TRUE(f.ranger.Start());  // 可重启（重新订阅）
    f.ranger.Stop();
}

// ---------------------------------------------------------------------------
// 未启动时不处理（发布到 Topic 但不消费）
// ---------------------------------------------------------------------------
TEST(StereoRangerTest, NotStartedDoesNotPublish) {
    Fixture f;
    auto out_sub = f.ranger.DistanceOutput().Subscribe(4);
    const auto ref = LoadBin("stereo_ref.bin");
    auto box = MakeBox(20, 20, 300, 230);
    (void)f.left_topic.Emplace(video::FrameHandle{});
    (void)f.detection_topic.Emplace(box);
    EXPECT_FALSE(out_sub.WaitTakeFor(50ms).has_value());
}

// ---------------------------------------------------------------------------
// 摘要静默抑制（故障排查手册 §19）：静默窗口不重复打印，活动恢复照常打印
// ---------------------------------------------------------------------------
TEST(StereoRangerTest, SummarySuppressedDuringSilenceAndResumesWithActivity) {
    drone::test::LogCapture capture;  // 须在组件启动前构造（见 log_capture.h）
    StereoRangerConfig cfg = TestConfig();
    cfg.summary_log_interval = 100ms;
    Fixture f(std::move(cfg));
    ASSERT_TRUE(f.ranger.Start());

    // 静默 5 个窗口（不发布任何帧/检测）：只有启动后首个窗口打印一次
    std::this_thread::sleep_for(550ms);
    EXPECT_EQ(capture.Count("双目测距摘要"), 1);

    // 恢复活动（帧 + 检测 → 批次处理）后，摘要照常出现
    const auto ref = LoadBin("stereo_ref.bin");
    const auto aux = LoadBin("stereo_aux.bin");
    const std::uint64_t left_seq =
        f.PublishFrame(f.left_pool, f.left_topic, ref, 1000);
    (void)f.PublishFrame(f.right_pool, f.right_topic, aux, 1000);
    common::DetectionResult box = MakeBox(20, 20, kViewWidth - 15, kViewHeight - 5);
    box.frame_sequence = left_seq;
    (void)f.detection_topic.Emplace(box);

    EXPECT_TRUE(WaitFor([&capture] { return capture.Count("双目测距摘要") >= 2; }))
        << "活动恢复后摘要未打印";
    f.ranger.Stop();
}

}  // namespace
}  // namespace drone::perception
