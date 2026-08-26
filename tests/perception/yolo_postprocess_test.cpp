/**
 * @file yolo_postprocess_test.cpp
 * @brief YOLO 后处理纯函数单元测试（不依赖硬件，开发机可跑）
 *
 * 覆盖：
 * - DecodeBranch：无 DFL 量化解码、阈值过滤、score_sum 快速过滤
 * - SortDescending + Nms：同类重叠抑制、异类保留
 * - PostProcess：多分支全链路 + letterbox 逆变换坐标还原
 * - PostProcessNormalizedXywh：`[1,5,N]` 阈值过滤、NMS、原图坐标还原
 */
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "perception/yolo_postprocess.h"

namespace drone::perception {
namespace {

using testing::Test;

/// 与实现一致的量化辅助（仿射量化，截断）。
std::int8_t Quantize(float value, std::int32_t zp, float scale) {
    float dst = value / scale + static_cast<float>(zp);
    dst = std::max(-128.f, std::min(127.f, dst));
    return static_cast<std::int8_t>(dst);
}

/// 构造一张全 0 的 NCHW 量化张量并设置指定位置。
struct TensorBuilder {
    int channels = 0;
    int grid_h = 0;
    int grid_w = 0;
    std::int32_t zp = 0;
    float scale = 1.f;
    std::vector<std::int8_t> data;

    TensorBuilder(int c, int h, int w, std::int32_t z = 0, float s = 1.f)
        : channels(c), grid_h(h), grid_w(w), zp(z), scale(s),
          data(static_cast<std::size_t>(c) * h * w, 0) {}

    /// 设置 (channel, i, j) 位置的量化值。
    void Set(int channel, int i, int j, float float_value) {
        const int grid_len = grid_h * grid_w;
        const std::size_t offset =
            static_cast<std::size_t>(channel) * grid_len +
            static_cast<std::size_t>(i) * grid_w + static_cast<std::size_t>(j);
        data[offset] = Quantize(float_value, zp, scale);
    }

    QuantTensor Make() const {
        QuantTensor t;
        t.data = data.data();
        t.zp = zp;
        t.scale = scale;
        t.channels = channels;
        t.grid_h = grid_h;
        t.grid_w = grid_w;
        return t;
    }
};

/// 期望候选：xywh + 置信度 + 类别。
struct ExpectedBox {
    float x1, y1, w, h, conf;
    int cls;
};

void ExpectBoxes(const std::vector<float>& boxes, const std::vector<float>& probs,
                 const std::vector<int>& class_ids,
                 const std::vector<ExpectedBox>& expected) {
    ASSERT_EQ(boxes.size() / 4, expected.size());
    ASSERT_EQ(probs.size(), expected.size());
    ASSERT_EQ(class_ids.size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_NEAR(boxes[i * 4 + 0], expected[i].x1, 1e-3f);
        EXPECT_NEAR(boxes[i * 4 + 1], expected[i].y1, 1e-3f);
        EXPECT_NEAR(boxes[i * 4 + 2], expected[i].w, 1e-3f);
        EXPECT_NEAR(boxes[i * 4 + 3], expected[i].h, 1e-3f);
        EXPECT_NEAR(probs[i], expected[i].conf, 1e-3f);
        EXPECT_EQ(class_ids[i], expected[i].cls);
    }
}

TEST(YoloPostProcessTest, DecodeBranchNoDfl) {
    // 模型 64x64，grid 1x1，stride=64；无 DFL（box 通道 4），1 类
    // box: b0=-1 → x1=(-(-1)+0.5)*64=96；b1=-1 → y1=96
    //       b2=2  → x2=(2+0.5)*64=160；b3=2  → y2=160 → w=h=64
    // score: 0.5（阈值 0.25 之上）；score scale=0.1 保证量化后不截断
    TensorBuilder box(4, 1, 1);
    box.Set(0, 0, 0, -1.f);
    box.Set(1, 0, 0, -1.f);
    box.Set(2, 0, 0, 2.f);
    box.Set(3, 0, 0, 2.f);
    TensorBuilder score(1, 1, 1, 0, 0.1f);
    score.Set(0, 0, 0, 0.5f);

    BranchOutput branch;
    branch.box = box.Make();
    branch.score = score.Make();

    std::vector<float> boxes;
    std::vector<float> probs;
    std::vector<int> class_ids;
    const int count = DecodeBranch(branch, 64, 1, 0.25f, boxes, probs, class_ids);

    EXPECT_EQ(count, 1);
    ExpectBoxes(boxes, probs, class_ids, {{96.f, 96.f, 64.f, 64.f, 0.5f, 0}});
}

TEST(YoloPostProcessTest, DecodeBranchFiltersLowScore) {
    // grid 1x2：cell0 低分（0.1 < 0.25）被过滤，cell1 高分保留
    TensorBuilder box(4, 1, 2);
    TensorBuilder score(1, 1, 2, 0, 0.1f);
    for (int j = 0; j < 2; ++j) {
        box.Set(0, 0, j, -1.f);
        box.Set(1, 0, j, -1.f);
        box.Set(2, 0, j, 2.f);
        box.Set(3, 0, j, 2.f);
    }
    score.Set(0, 0, 0, 0.1f);
    score.Set(0, 0, 1, 0.9f);

    BranchOutput branch;
    branch.box = box.Make();
    branch.score = score.Make();

    std::vector<float> boxes;
    std::vector<float> probs;
    std::vector<int> class_ids;
    const int count = DecodeBranch(branch, 64, 1, 0.25f, boxes, probs, class_ids);

    // 仅 cell1 保留：x1=(-(-1)+0.5+1)*64=160
    EXPECT_EQ(count, 1);
    ExpectBoxes(boxes, probs, class_ids, {{160.f, 96.f, 64.f, 64.f, 0.9f, 0}});
}

TEST(YoloPostProcessTest, DecodeBranchScoreSumFastFilter) {
    // score_sum 低于阈值时即使 score 高也跳过（快速过滤）
    TensorBuilder box(4, 1, 2);
    TensorBuilder score(1, 1, 2, 0, 0.1f);
    TensorBuilder score_sum(1, 1, 2, 0, 0.1f);
    for (int j = 0; j < 2; ++j) {
        box.Set(0, 0, j, -1.f);
        box.Set(1, 0, j, -1.f);
        box.Set(2, 0, j, 2.f);
        box.Set(3, 0, j, 2.f);
        score.Set(0, 0, j, 0.9f);
    }
    score_sum.Set(0, 0, 0, 0.05f);  // 低于阈值 → 跳过
    score_sum.Set(0, 0, 1, 0.9f);   // 高于阈值 → 保留

    BranchOutput branch;
    branch.box = box.Make();
    branch.score = score.Make();
    branch.score_sum = score_sum.Make();

    std::vector<float> boxes;
    std::vector<float> probs;
    std::vector<int> class_ids;
    const int count = DecodeBranch(branch, 64, 1, 0.25f, boxes, probs, class_ids);

    EXPECT_EQ(count, 1);
    ExpectBoxes(boxes, probs, class_ids, {{160.f, 96.f, 64.f, 64.f, 0.9f, 0}});
}

TEST(YoloPostProcessTest, NmsSuppressesOverlappingSameClass) {
    // 三个同类框：A(0,0,100x100) 0.9、B(10,10,100x100) 0.8（与 A 高重叠）、
    // C(200,200,100x100) 0.7（远离）
    std::vector<float> boxes = {0.f, 0.f, 100.f, 100.f,
                                10.f, 10.f, 100.f, 100.f,
                                200.f, 200.f, 100.f, 100.f};
    std::vector<float> scores = {0.9f, 0.8f, 0.7f};
    std::vector<int> class_ids = {0, 0, 0};

    std::vector<int> order;
    SortDescending(scores, order);
    Nms(3, boxes, class_ids, order, 0, 0.45f);

    // B 被抑制，A/C 保留；order 中剩余索引按置信度序
    std::vector<int> survivors;
    for (int idx : order) {
        if (idx != -1) {
            survivors.push_back(idx);
        }
    }
    ASSERT_EQ(survivors.size(), 2u);
    EXPECT_EQ(survivors[0], 0);
    EXPECT_EQ(survivors[1], 2);
}

TEST(YoloPostProcessTest, NmsKeepsDifferentClasses) {
    // 完全重叠但类别不同 → 都不抑制
    std::vector<float> boxes = {0.f, 0.f, 100.f, 100.f,
                                0.f, 0.f, 100.f, 100.f};
    std::vector<float> scores = {0.9f, 0.8f};
    std::vector<int> class_ids = {0, 1};

    std::vector<int> order;
    SortDescending(scores, order);
    Nms(2, boxes, class_ids, order, 0, 0.45f);
    Nms(2, boxes, class_ids, order, 1, 0.45f);

    std::vector<int> survivors;
    for (int idx : order) {
        if (idx != -1) {
            survivors.push_back(idx);
        }
    }
    EXPECT_EQ(survivors.size(), 2u);
}

TEST(YoloPostProcessTest, PostProcessFullChain) {
    // 全链路：单分支解码 → NMS → letterbox 逆变换。
    // 模型 64x64，grid 2x2，stride=32，cell (i=1, j=1)，box 量化 scale=0.1875：
    //   b0: q=3 → 0.5625 → x1=(-0.5625+1.5)*32=30
    //   b1: q=2 → 0.375  → y1=(-0.375+1.5)*32=36
    //   b2: q=4 → 0.75   → x2=(0.75+1.5)*32=72
    //   b3: q=4 → 0.75   → y2=72
    // letterbox: x_pad=10, y_pad=20, scale=0.5
    // 期望裁剪图坐标：x1=(30-10)/0.5=40, y1=(36-20)/0.5=32,
    //                x2=(72-10)/0.5=124, y2=(72-20)/0.5=104
    constexpr float kBoxScale = 0.1875f;
    TensorBuilder box(4, 2, 2, 0, kBoxScale);
    box.Set(0, 1, 1, 0.5625f);  // q=3
    box.Set(1, 1, 1, 0.375f);   // q=2
    box.Set(2, 1, 1, 0.75f);    // q=4
    box.Set(3, 1, 1, 0.75f);    // q=4
    TensorBuilder score(1, 2, 2, 0, 0.1f);
    score.Set(0, 1, 1, 0.5f);   // q=5，高于阈值 0.25

    BranchOutput branch;
    branch.box = box.Make();
    branch.score = score.Make();

    LetterBox letterbox;
    letterbox.x_pad = 10;
    letterbox.y_pad = 20;
    letterbox.scale = 0.5f;

    std::vector<YoloDetection> detections;
    const int count = PostProcess({branch}, 64, 0.25f, 0.45f, 1, letterbox,
                                  &detections);

    ASSERT_EQ(count, 1);
    ASSERT_EQ(detections.size(), 1u);
    EXPECT_NEAR(detections[0].x1, 40.f, 1e-3f);
    EXPECT_NEAR(detections[0].y1, 32.f, 1e-3f);
    EXPECT_NEAR(detections[0].x2, 124.f, 1e-3f);
    EXPECT_NEAR(detections[0].y2, 104.f, 1e-3f);
    EXPECT_NEAR(detections[0].confidence, 0.5f, 1e-3f);
    EXPECT_EQ(detections[0].class_id, 0);
}

TEST(YoloPostProcessTest, NormalizedXywhSingleOutputRestoresLetterbox) {
    // `[1,5,2]` 通道优先：候选0有效，候选1低于阈值。
    // 原图 1280x720 → 640x640：scale=0.5，顶部 padding=140。
    constexpr float kScale = 0.01f;
    std::vector<std::int8_t> data(10, 0);
    const int n = 2;
    data[0 * n + 0] = Quantize(0.50f, 0, kScale);  // center x
    data[1 * n + 0] = Quantize(0.50f, 0, kScale);  // center y
    data[2 * n + 0] = Quantize(0.50f, 0, kScale);  // width
    data[3 * n + 0] = Quantize(0.25f, 0, kScale);  // height
    data[4 * n + 0] = Quantize(0.90f, 0, kScale);  // confidence
    data[4 * n + 1] = Quantize(0.10f, 0, kScale);  // 被阈值过滤

    NormalizedXywhTensor tensor{data.data(), 0, kScale, 5, n};
    LetterBox letterbox;
    letterbox.scale = 0.5f;
    letterbox.y_pad = 140;

    std::vector<YoloDetection> detections;
    const int count = PostProcessNormalizedXywh(
        tensor, 640, 640, 0.25f, 0.45f, 0, letterbox, &detections);

    ASSERT_EQ(count, 1);
    ASSERT_EQ(detections.size(), 1u);
    EXPECT_NEAR(detections[0].x1, 320.f, 1e-3f);
    EXPECT_NEAR(detections[0].y1, 200.f, 1e-3f);
    EXPECT_NEAR(detections[0].x2, 960.f, 1e-3f);
    EXPECT_NEAR(detections[0].y2, 520.f, 1e-3f);
    EXPECT_NEAR(detections[0].confidence, 0.90f, 1e-3f);
    EXPECT_EQ(detections[0].class_id, 0);
}

TEST(YoloPostProcessTest, NormalizedXywhSingleOutputAppliesNms) {
    constexpr float kScale = 0.01f;
    constexpr int kCandidates = 2;
    std::vector<std::int8_t> data(5 * kCandidates, 0);
    for (int i = 0; i < kCandidates; ++i) {
        data[0 * kCandidates + i] = Quantize(0.50f + i * 0.01f, 0, kScale);
        data[1 * kCandidates + i] = Quantize(0.50f + i * 0.01f, 0, kScale);
        data[2 * kCandidates + i] = Quantize(0.40f, 0, kScale);
        data[3 * kCandidates + i] = Quantize(0.40f, 0, kScale);
    }
    data[4 * kCandidates + 0] = Quantize(0.90f, 0, kScale);
    data[4 * kCandidates + 1] = Quantize(0.80f, 0, kScale);

    NormalizedXywhTensor tensor{data.data(), 0, kScale, 5, kCandidates};
    std::vector<YoloDetection> detections;
    const int count = PostProcessNormalizedXywh(
        tensor, 640, 640, 0.25f, 0.45f, 0, LetterBox{}, &detections);

    ASSERT_EQ(count, 1);
    ASSERT_EQ(detections.size(), 1u);
    EXPECT_NEAR(detections[0].confidence, 0.90f, 1e-3f);
}

TEST(YoloPostProcessTest, NormalizedXywhRejectsInvalidTensor) {
    std::vector<YoloDetection> detections;
    EXPECT_EQ(PostProcessNormalizedXywh({}, 640, 640, 0.25f, 0.45f, 0,
                                        LetterBox{}, &detections),
              0);
    EXPECT_TRUE(detections.empty());
}

TEST(YoloPostProcessTest, PostProcessEmptyBranch) {
    std::vector<YoloDetection> detections;
    EXPECT_EQ(PostProcess({}, 64, 0.25f, 0.45f, 1, LetterBox{}, &detections), 0);
    EXPECT_TRUE(detections.empty());
}

TEST(YoloPostProcessTest, PostProcessNullDataBranchSkipped) {
    BranchOutput branch;  // 全空
    std::vector<YoloDetection> detections;
    EXPECT_EQ(PostProcess({branch}, 64, 0.25f, 0.45f, 1, LetterBox{},
                          &detections),
              0);
}

}  // namespace
}  // namespace drone::perception
