// =============================================================================
// tracklet_test.cpp —— Tracklet 与几何工具单元测试
//
// 覆盖 docs/视觉追踪技术路线.md：
//   - IoU 边界（空框 / 零交集 / 完全重合=1，§5.2）
//   - IsDead / IsCoasting 判定（§3.2 派生状态，§6.1 严格大于）
//   - Predict / Update / MarkMiss 计数语义（§7 末尾伪代码）
//   - BoxXYXY ↔ BoxCXCYWH 互转（§7）
// =============================================================================

#include <gtest/gtest.h>

#include "target_tracker/tracklet.h"

using drone::tracker::BoxCXCYWH;
using drone::tracker::BoxXYXY;
using drone::tracker::ComputeIoU;
using drone::tracker::ToCenterSize;
using drone::tracker::ToXYXY;
using drone::tracker::Tracklet;

namespace {

// ---- 框表示互转（§7） ----

TEST(BoxConversionTest, RoundTripPreservesValues) {
    const BoxXYXY box{10.0, 20.0, 50.0, 80.0};
    const BoxCXCYWH c = ToCenterSize(box);
    EXPECT_DOUBLE_EQ(c.cx, 30.0);   // (10+50)/2
    EXPECT_DOUBLE_EQ(c.cy, 50.0);   // (20+80)/2
    EXPECT_DOUBLE_EQ(c.w, 40.0);    // 50−10
    EXPECT_DOUBLE_EQ(c.h, 60.0);    // 80−20

    const BoxXYXY back = ToXYXY(c);
    EXPECT_DOUBLE_EQ(back.x1, box.x1);
    EXPECT_DOUBLE_EQ(back.y1, box.y1);
    EXPECT_DOUBLE_EQ(back.x2, box.x2);
    EXPECT_DOUBLE_EQ(back.y2, box.y2);
}

// ---- IoU 边界（§5.2） ----

TEST(IoUTest, IdenticalBoxesReturnOne) {
    const BoxXYXY a{10.0, 10.0, 30.0, 30.0};
    EXPECT_NEAR(ComputeIoU(a, a), 1.0, 1e-12);  // 完全重合 = 1
}

TEST(IoUTest, DisjointBoxesReturnZero) {
    const BoxXYXY a{0.0, 0.0, 10.0, 10.0};
    const BoxXYXY b{20.0, 20.0, 30.0, 30.0};
    EXPECT_DOUBLE_EQ(ComputeIoU(a, b), 0.0);  // 零交集
}

TEST(IoUTest, TouchingEdgesReturnZero) {
    // 仅边/角相接：交集宽高为 0 → 交集面积 ≤ 0 返回 0（§5.2）
    const BoxXYXY a{0.0, 0.0, 10.0, 10.0};
    const BoxXYXY b{10.0, 0.0, 20.0, 10.0};   // 右边缘相接
    EXPECT_DOUBLE_EQ(ComputeIoU(a, b), 0.0);
    const BoxXYXY c{10.0, 10.0, 20.0, 20.0};  // 角点相接
    EXPECT_DOUBLE_EQ(ComputeIoU(a, c), 0.0);
}

TEST(IoUTest, EmptyBoxReturnsZero) {
    const BoxXYXY normal{0.0, 0.0, 10.0, 10.0};
    const BoxXYXY zero_area{5.0, 5.0, 5.0, 5.0};    // 零面积
    const BoxXYXY inverted{10.0, 10.0, 0.0, 0.0};   // 反向（负宽高）
    EXPECT_DOUBLE_EQ(ComputeIoU(normal, zero_area), 0.0);
    EXPECT_DOUBLE_EQ(ComputeIoU(zero_area, normal), 0.0);
    EXPECT_DOUBLE_EQ(ComputeIoU(normal, inverted), 0.0);
    EXPECT_DOUBLE_EQ(ComputeIoU(zero_area, zero_area), 0.0);
}

TEST(IoUTest, PartialOverlapIsCorrect) {
    // A = [0,0,10,10]（面积 100），B = [5,5,15,15]（面积 100），交集 25
    const BoxXYXY a{0.0, 0.0, 10.0, 10.0};
    const BoxXYXY b{5.0, 5.0, 15.0, 15.0};
    EXPECT_NEAR(ComputeIoU(a, b), 25.0 / 175.0, 1e-12);
    EXPECT_NEAR(ComputeIoU(b, a), 25.0 / 175.0, 1e-12);  // 对称性
}

// ---- 构造与派生状态（§3.2 / §6.1） ----

TEST(TrackletTest, ConstructionInitialFields) {
    const BoxCXCYWH meas{100.0, 100.0, 20.0, 20.0};
    Tracklet t(7, meas, 0.9, 1, 8);

    EXPECT_EQ(t.track_id, 7);
    EXPECT_EQ(t.lost_count, 0);
    EXPECT_EQ(t.max_lost, 8);
    EXPECT_DOUBLE_EQ(t.confidence, 0.9);
    EXPECT_EQ(t.cls_id, 1);
    EXPECT_EQ(t.age, 1);   // 创建时为 1（§3.2）
    EXPECT_EQ(t.hits, 1);  // 创建时为 1（§3.2）
    EXPECT_DOUBLE_EQ(t.last_center.first, 100.0);
    EXPECT_DOUBLE_EQ(t.last_center.second, 100.0);
    ASSERT_TRUE(t.last_box.has_value());  // 创建当帧即赋 last_box（§7 步骤 5）
    EXPECT_DOUBLE_EQ(t.last_box->x1, 90.0);
    EXPECT_DOUBLE_EQ(t.last_box->y1, 90.0);
    EXPECT_DOUBLE_EQ(t.last_box->x2, 110.0);
    EXPECT_DOUBLE_EQ(t.last_box->y2, 110.0);
    EXPECT_FALSE(t.IsDead());
    EXPECT_FALSE(t.IsCoasting());
}

// IsDead 使用严格大于：lost_count == max_lost 仍存活（§6.1 注记 2）
TEST(TrackletTest, IsDeadUsesStrictlyGreater) {
    Tracklet t(0, {100.0, 100.0, 20.0, 20.0}, 0.9, 0, 8);
    for (int i = 0; i < 8; ++i) {
        t.MarkMiss();
        EXPECT_FALSE(t.IsDead()) << "lost_count=" << t.lost_count
                                 << " 未超过 max_lost 不应判死";
    }
    EXPECT_EQ(t.lost_count, 8);
    t.MarkMiss();  // lost_count = 9 > 8
    EXPECT_TRUE(t.IsDead());
}

TEST(TrackletTest, IsCoastingWhenLostPositive) {
    Tracklet t(0, {100.0, 100.0, 20.0, 20.0}, 0.9, 0, 8);
    EXPECT_FALSE(t.IsCoasting());
    t.MarkMiss();
    EXPECT_TRUE(t.IsCoasting());  // lost_count > 0 即 Coast（§3.2）
}

// ---- 生命周期方法计数语义（§7） ----

TEST(TrackletTest, PredictAdvancesAgeAndUpdatesLastState) {
    Tracklet t(0, {100.0, 100.0, 20.0, 20.0}, 0.9, 0, 8);
    const int age_before = t.age;
    t.Predict();  // 速度为零：位置不变，但 age+1、last_box 更新为预测框
    EXPECT_EQ(t.age, age_before + 1);
    ASSERT_TRUE(t.last_box.has_value());
    EXPECT_NEAR(t.last_center.first, 100.0, 1e-9);
    EXPECT_NEAR(t.last_center.second, 100.0, 1e-9);
    // hits / lost_count / confidence 不受预测影响
    EXPECT_EQ(t.hits, 1);
    EXPECT_EQ(t.lost_count, 0);
    EXPECT_DOUBLE_EQ(t.confidence, 0.9);
}

TEST(TrackletTest, UpdateResetsLostAndIncrementsHits) {
    Tracklet t(0, {100.0, 100.0, 20.0, 20.0}, 0.9, 0, 8);
    t.MarkMiss();
    t.MarkMiss();
    ASSERT_EQ(t.lost_count, 2);
    ASSERT_TRUE(t.IsCoasting());

    const int age_before = t.age;
    const BoxXYXY box{150.0, 160.0, 170.0, 180.0};
    t.Update(box, 0.75, 2);

    EXPECT_EQ(t.lost_count, 0);  // 匹配成功清零（§7）
    EXPECT_EQ(t.hits, 2);        // hits+1
    EXPECT_EQ(t.age, age_before + 1);  // age+1
    EXPECT_DOUBLE_EQ(t.confidence, 0.75);
    EXPECT_EQ(t.cls_id, 2);
    EXPECT_DOUBLE_EQ(t.last_center.first, 160.0);   // (150+170)/2
    EXPECT_DOUBLE_EQ(t.last_center.second, 170.0);  // (160+180)/2
    ASSERT_TRUE(t.last_box.has_value());
    EXPECT_DOUBLE_EQ(t.last_box->x1, box.x1);
    ASSERT_TRUE(t.last_raw_box.has_value());  // 实测框同步记录
    EXPECT_DOUBLE_EQ(t.last_raw_box->x2, box.x2);
    EXPECT_FALSE(t.IsCoasting());
}

TEST(TrackletTest, MarkMissIncrementsLostAndAge) {
    Tracklet t(0, {100.0, 100.0, 20.0, 20.0}, 0.9, 0, 8);
    const int age_before = t.age;
    t.MarkMiss();
    EXPECT_EQ(t.lost_count, 1);
    EXPECT_EQ(t.age, age_before + 1);
    // MarkMiss 不影响 hits / confidence
    EXPECT_EQ(t.hits, 1);
    EXPECT_DOUBLE_EQ(t.confidence, 0.9);
}

}  // namespace
