// =============================================================================
// target_tracker_test.cpp —— TargetTracker 单元测试
//
// 逐条覆盖 docs/视觉追踪技术路线.md §9.4 行为一致性验收用例（硬验收标准）：
//   1. 空检测帧序列：Coast 持续 max_lost_frames 帧，第 max_lost_frames+1 帧判丢
//   2. ID 严格递增不复用（含 Reset 后归零）
//   3. 锁定不被抢占；锁定目标死亡后切换到新目标
//   4. min_hits 回退：无已确认实测轨道时新轨道也可当选主目标
//   5. cost=1.0 恰好不构成匹配（严格小于）
//   6. 置信度优先：两轨道竞争同一检测，置信度高者获胜
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "target_tracker/target_tracker.h"

using drone::tracker::Detection;
using drone::tracker::FrameShape;
using drone::tracker::TargetTracker;
using drone::tracker::TrackResult;

namespace {

Detection MakeDet(double x1, double y1, double x2, double y2, double conf,
                  int cls = 0) {
    return Detection{x1, y1, x2, y2, conf, cls};
}

// ============================================================================
// 验收用例 1（§9.4-1）：空检测帧序列
// 连续输入空检测，主目标输出 is_predicted=true 共 max_lost_frames 帧，
// 第 max_lost_frames + 1 帧 tracked=false（死亡判定 lost_count > max_lost）。
// ============================================================================
TEST(TargetTrackerAcceptanceTest, EmptyFramesCoastThenLost) {
    TargetTracker tracker;  // 默认配置 max_lost_frames = 8
    ASSERT_EQ(tracker.GetConfig().max_lost_frames, 8);

    // 第 1 帧：给出一个检测，建立轨道并锁定为主目标
    TrackResult r = tracker.Update({MakeDet(100.0, 100.0, 140.0, 140.0, 0.9)});
    ASSERT_TRUE(r.tracked);
    ASSERT_TRUE(r.primary_id.has_value());
    EXPECT_FALSE(r.is_predicted);   // 当帧有实测
    EXPECT_EQ(r.lost_frames, 0);
    EXPECT_EQ(r.n_active, 1);
    ASSERT_TRUE(r.box.has_value());
    ASSERT_TRUE(r.raw.has_value()); // 当帧关联到原始检测框

    // 后续空检测帧：应持续 Coast 共 max_lost_frames 帧（is_predicted=true）
    for (int i = 1; i <= 8; ++i) {
        r = tracker.Update({});
        ASSERT_TRUE(r.tracked) << "第 " << i << " 帧 Coast 期间不应判丢";
        EXPECT_TRUE(r.is_predicted) << "第 " << i << " 帧应为纯预测输出";
        EXPECT_EQ(r.lost_frames, i);
        EXPECT_TRUE(r.primary_id.has_value());
        EXPECT_TRUE(r.box.has_value());     // Coast 期间继续输出预测框
        EXPECT_FALSE(r.raw.has_value());    // Coast 时 raw 为空（§2.3）
        EXPECT_DOUBLE_EQ(r.confidence, 0.9); // 继承最近实测置信度（§2.3）
    }

    // 第 max_lost_frames + 1 帧：轨道死亡回收 → tracked=false
    r = tracker.Update({});
    EXPECT_FALSE(r.tracked);
    EXPECT_FALSE(r.primary_id.has_value());
    EXPECT_FALSE(r.box.has_value());
    EXPECT_EQ(r.lost_frames, 0);  // 丢失时 lost_frames=0（§2.3）
    EXPECT_EQ(r.n_active, 0);
}

// ============================================================================
// 验收用例 2（§9.4-2）：ID 单调性
// 多次进出画面的目标获得严格递增、不复用的 ID；Reset 后计数器归零。
// ============================================================================
TEST(TargetTrackerAcceptanceTest, IdStrictlyIncreasingNoReuse) {
    TargetTracker::Config cfg;
    cfg.max_lost_frames = 1;  // 缩短 Coast 窗口，加速轨道死亡
    TargetTracker tracker(cfg);

    // 目标 A 出现 → id 0
    TrackResult r = tracker.Update({MakeDet(100.0, 100.0, 120.0, 120.0, 0.9)});
    ASSERT_TRUE(r.primary_id.has_value());
    const int id_a = *r.primary_id;
    EXPECT_EQ(id_a, 0);

    // 目标 A 消失至死亡（max_lost=1：第 2 帧 Coast、第 3 帧死亡回收）
    tracker.Update({});
    r = tracker.Update({});
    EXPECT_FALSE(r.tracked);
    EXPECT_EQ(tracker.ActiveCount(), 0);

    // 目标 B 出现（同一位置）：ID 不复用，严格递增 → id 1
    r = tracker.Update({MakeDet(100.0, 100.0, 120.0, 120.0, 0.9)});
    ASSERT_TRUE(r.primary_id.has_value());
    const int id_b = *r.primary_id;
    EXPECT_EQ(id_b, 1);
    EXPECT_GT(id_b, id_a);

    // 目标 C 同帧新建 → id 2（同帧多目标亦严格递增）
    r = tracker.Update({MakeDet(100.0, 100.0, 120.0, 120.0, 0.9),
                        MakeDet(400.0, 300.0, 430.0, 330.0, 0.8)});
    EXPECT_EQ(tracker.ActiveCount(), 2);
    int max_id = 0;
    for (const auto& kv : tracker.ActiveTracks()) {
        max_id = std::max(max_id, kv.first);
    }
    EXPECT_EQ(max_id, 2);

    // Reset 后计数器归零（§6.2）：新轨道重新从 id 0 开始
    tracker.Reset();
    EXPECT_EQ(tracker.ActiveCount(), 0);
    EXPECT_FALSE(tracker.LastResult().tracked);
    r = tracker.Update({MakeDet(50.0, 50.0, 70.0, 70.0, 0.9)});
    ASSERT_TRUE(r.primary_id.has_value());
    EXPECT_EQ(*r.primary_id, 0);
}

// ============================================================================
// 验收用例 3（§9.4-3）：锁定不被抢占
// 锁定目标 Coast 期间出现更高置信度新目标，primary_id 不变；
// 锁定目标死亡后切换到新目标。
// ============================================================================
TEST(TargetTrackerAcceptanceTest, LockNotStolenAndSwitchAfterDeath) {
    TargetTracker::Config cfg;
    cfg.max_lost_frames = 3;
    TargetTracker tracker(cfg);

    // 低置信度目标 A 建立并锁定
    TrackResult r = tracker.Update({MakeDet(100.0, 100.0, 140.0, 140.0, 0.4)});
    ASSERT_TRUE(r.primary_id.has_value());
    const int id_a = *r.primary_id;

    // A 消失进入 Coast；期间出现更高置信度新目标 B（远离 A，不会被误关联）
    r = tracker.Update({MakeDet(500.0, 300.0, 540.0, 340.0, 0.95)});
    ASSERT_TRUE(r.primary_id.has_value());
    EXPECT_EQ(*r.primary_id, id_a) << "Coast 期间锁定不应被高置信度目标抢占";
    EXPECT_TRUE(r.is_predicted);
    EXPECT_EQ(tracker.ActiveCount(), 2);  // A（Coast）与 B 并存

    // 继续 Coast，锁定依旧延续
    r = tracker.Update({MakeDet(500.0, 300.0, 540.0, 340.0, 0.95)});
    EXPECT_EQ(*r.primary_id, id_a);
    EXPECT_TRUE(r.is_predicted);

    // A 继续丢失直至死亡（max_lost=3：lost 达 4 帧时回收）
    for (int i = 0; i < 2; ++i) {
        r = tracker.Update({MakeDet(500.0, 300.0, 540.0, 340.0, 0.95)});
    }
    // A 已死亡删除 → 锁定解除，重新选举切换到 B（此时 B 已确认且实测）
    ASSERT_TRUE(r.tracked);
    EXPECT_NE(*r.primary_id, id_a) << "锁定目标死亡后应切换到新目标";
    EXPECT_FALSE(r.is_predicted);

    // 后续 B 持续作为主目标
    r = tracker.Update({MakeDet(500.0, 300.0, 540.0, 340.0, 0.95)});
    ASSERT_TRUE(r.primary_id.has_value());
    EXPECT_NE(*r.primary_id, id_a);
}

// ============================================================================
// 验收用例 4（§9.4-4）：min_hits 回退
// 新轨道（hits 未达 min_hits）在无其他已确认实测轨道时也可当选主目标。
// ============================================================================
TEST(TargetTrackerAcceptanceTest, MinHitsFallbackAllowsNewTrack) {
    TargetTracker::Config cfg;
    cfg.min_hits = 5;  // 抬高确认门槛，保证测试期间轨道始终未确认
    TargetTracker tracker(cfg);

    // 首帧新建轨道 hits=1 < min_hits：候选池 confirmed 为空 → 退回 measured
    TrackResult r = tracker.Update({MakeDet(200.0, 150.0, 240.0, 190.0, 0.6)});
    ASSERT_TRUE(r.tracked) << "无已确认轨道时新轨道应能当选主目标";
    ASSERT_TRUE(r.primary_id.has_value());
    EXPECT_EQ(*r.primary_id, 0);
    EXPECT_FALSE(r.is_predicted);

    // 再跟几帧（hits 增长但仍未达 min_hits=5），依旧当选
    for (int i = 0; i < 3; ++i) {
        r = tracker.Update({MakeDet(200.0, 150.0, 240.0, 190.0, 0.6)});
        ASSERT_TRUE(r.tracked);
        EXPECT_EQ(*r.primary_id, 0);
    }
}

// ============================================================================
// 验收用例 5（§9.4-5）：cost 边界
// 恰好 cost = 1.0 不构成匹配（仅严格小于才匹配）。
// 构造：w_dist=1、w_iou=0（纯距离项），中心距离恰等于 Dmax → cost=1.0；
// 同时给出略小于阈值的对照帧证明边界两侧行为正确。
// ============================================================================
TEST(TargetTrackerAcceptanceTest, CostExactlyOneDoesNotMatch) {
    TargetTracker::Config cfg;
    cfg.max_lost_frames = 1;
    cfg.max_association_dist = 200.0;
    cfg.dist_weight = 1.0;  // 纯距离项，IoU 项权重为 0 且两框零交集
    cfg.iou_weight = 0.0;
    TargetTracker tracker(cfg);

    // 建立轨道：中心 (100,100)
    TrackResult r = tracker.Update({MakeDet(90.0, 90.0, 110.0, 110.0, 0.9)});
    ASSERT_TRUE(r.primary_id.has_value());
    const int id_a = *r.primary_id;

    // 下一帧检测中心 (100,300)：距离 = 200 = Dmax → cost = 1.0，恰好不匹配
    // （预测后轨道中心速度为 0，仍位于 (100,100)；两框零交集 IoU=0 无影响）
    r = tracker.Update({MakeDet(90.0, 290.0, 110.0, 310.0, 0.9)});
    // 未匹配 → 原轨道 Coast、检测新建轨道；两个目标并存
    EXPECT_EQ(tracker.ActiveCount(), 2) << "cost=1.0 不应匹配，应新建轨道";
    // 原轨道（锁定）进入 Coast：lost_frames=1
    EXPECT_TRUE(r.tracked);
    EXPECT_TRUE(r.is_predicted);
    EXPECT_EQ(*r.primary_id, id_a);
    EXPECT_EQ(r.lost_frames, 1);

    // 对照：距离略小于阈值（199px → cost=0.995 < 1.0）应匹配成功
    TargetTracker tracker2(cfg);
    r = tracker2.Update({MakeDet(90.0, 90.0, 110.0, 110.0, 0.9)});
    const int id_b = *r.primary_id;
    r = tracker2.Update({MakeDet(90.0, 289.0, 110.0, 309.0, 0.9)});
    EXPECT_EQ(tracker2.ActiveCount(), 1) << "cost<1.0 应匹配成功，不应新建轨道";
    ASSERT_TRUE(r.primary_id.has_value());
    EXPECT_EQ(*r.primary_id, id_b);
    EXPECT_FALSE(r.is_predicted);  // 实测命中
}

// ============================================================================
// 验收用例 6（§9.4-6）：置信度优先
// 两轨道竞争同一检测时，置信度高者获胜。
// ============================================================================
TEST(TargetTrackerAcceptanceTest, HigherConfidenceTrackWinsDetection) {
    TargetTracker tracker;

    // 同帧建立两条轨道：A 置信度 0.5、B 置信度 0.9，相距 80px（不互相重叠）
    TrackResult r = tracker.Update({MakeDet(100.0, 100.0, 120.0, 120.0, 0.5),
                                    MakeDet(180.0, 100.0, 200.0, 120.0, 0.9)});
    ASSERT_EQ(tracker.ActiveCount(), 2);

    // 找出两条轨道 ID（按置信度区分）
    int id_low = -1;
    int id_high = -1;
    for (const auto& kv : tracker.ActiveTracks()) {
        if (kv.second.confidence > 0.8) {
            id_high = kv.first;
        } else {
            id_low = kv.first;
        }
    }
    ASSERT_NE(id_low, -1);
    ASSERT_NE(id_high, -1);

    // 下一帧只给一个检测，放在两轨道预测中心之间（各距 40px，对称）：
    // 距离项对两者相同，贪婪按置信度降序 → 高置信度轨道 B 优先匹配
    r = tracker.Update({MakeDet(140.0, 100.0, 160.0, 120.0, 0.7)});

    const auto tracks = tracker.ActiveTracks();
    ASSERT_EQ(tracks.size(), 2u);
    // 获胜者：命中（lost=0、hits=2、置信度更新）
    EXPECT_EQ(tracks.at(id_high).lost_count, 0) << "高置信度轨道应获胜";
    EXPECT_EQ(tracks.at(id_high).hits, 2);
    EXPECT_DOUBLE_EQ(tracks.at(id_high).confidence, 0.7);
    // 失败者：未匹配进入 Coast（lost=1、hits 不变）
    EXPECT_EQ(tracks.at(id_low).lost_count, 1) << "低置信度轨道应落选";
    EXPECT_EQ(tracks.at(id_low).hits, 1);
    EXPECT_DOUBLE_EQ(tracks.at(id_low).confidence, 0.5);
}

// ============================================================================
// NaN/Inf 防御（评审修复项）：坐标或置信度含非有限值的检测被丢弃，
// 不崩溃、不新建轨道、不污染主目标输出
// ============================================================================
TEST(TargetTrackerAcceptanceTest, NonFiniteDetectionsAreDiscarded) {
    TargetTracker tracker;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    // 第 1 帧：建立正常轨道
    TrackResult r = tracker.Update({MakeDet(100.0, 100.0, 140.0, 140.0, 0.9)});
    ASSERT_TRUE(r.tracked);

    // 第 2 帧：混入 NaN 坐标 / Inf 坐标 / NaN 置信度检测与一个正常检测：
    // 非有限检测全部被丢弃，仅正常检测参与关联，不新建轨道
    r = tracker.Update({MakeDet(nan, 100.0, 140.0, 140.0, 0.9),
                        MakeDet(100.0, -inf, 140.0, 140.0, 0.9),
                        MakeDet(100.0, 100.0, nan, 140.0, 0.9),
                        MakeDet(102.0, 102.0, 142.0, 142.0, nan),
                        MakeDet(101.0, 101.0, 141.0, 141.0, 0.9)});
    EXPECT_EQ(tracker.ActiveCount(), 1) << "非有限检测不应新建轨道";
    ASSERT_TRUE(r.tracked);
    EXPECT_FALSE(r.is_predicted);  // 正常检测命中，实测输出
    // 主目标输出未被 NaN 污染
    EXPECT_TRUE(std::isfinite(r.center.first));
    EXPECT_TRUE(std::isfinite(r.center.second));
    ASSERT_TRUE(r.box.has_value());
    EXPECT_TRUE(std::isfinite(r.box->x1) && std::isfinite(r.box->y1));
    EXPECT_TRUE(std::isfinite(r.box->x2) && std::isfinite(r.box->y2));
    EXPECT_TRUE(std::isfinite(r.confidence));
    ASSERT_TRUE(r.raw.has_value());  // 实测帧 raw 有值

    // 第 3 帧：仅含 NaN 检测 → 等价空帧，轨道 Coast 不崩溃
    r = tracker.Update({MakeDet(nan, nan, nan, nan, nan)});
    EXPECT_TRUE(r.tracked);
    EXPECT_TRUE(r.is_predicted);
    EXPECT_EQ(r.lost_frames, 1);
    EXPECT_FALSE(r.raw.has_value());  // Coast 时 raw 为空（§2.3）
    EXPECT_TRUE(std::isfinite(r.center.first));

    // 持续 NaN 检测直至轨道死亡回收，全程不崩溃
    for (int i = 0; i < 10; ++i) {
        r = tracker.Update({MakeDet(nan, 0.0, 10.0, 10.0, 0.9)});
    }
    EXPECT_FALSE(r.tracked);
    EXPECT_EQ(tracker.ActiveCount(), 0);
}

// ============================================================================
// 补充行为测试（§2.3 / §6.3 / §9.5）
// ============================================================================

// 丢失时返回值约定：tracked=false、定位字段空、lost_frames=0（§2.3）
TEST(TargetTrackerBehaviorTest, LostResultFieldsAreEmpty) {
    TargetTracker tracker;
    TrackResult r = tracker.Update({});  // 空帧序列起点：无任何轨道
    EXPECT_FALSE(r.tracked);
    EXPECT_FALSE(r.primary_id.has_value());
    EXPECT_FALSE(r.box.has_value());
    EXPECT_FALSE(r.raw.has_value());
    EXPECT_EQ(r.lost_frames, 0);
    EXPECT_EQ(r.n_active, 0);
    EXPECT_DOUBLE_EQ(r.confidence, 0.0);

    // LastResult 返回同一快照（§9.5 跨线程读取入口）
    const TrackResult cached = tracker.LastResult();
    EXPECT_FALSE(cached.tracked);
}

// Coast 兜底选举：无实测轨道时选离画面中心最近的 Coast 轨道（§6.3）。
// 注意锁定延续优先：需先让锁定轨道死亡解除锁定，才会进入重新选举分支，
// 故采用错帧创建 + 缩短 max_lost 的方式构造场景。
TEST(TargetTrackerBehaviorTest, CoastingFallbackChoosesNearestToFrameCenter) {
    TargetTracker::Config cfg;
    cfg.max_lost_frames = 2;
    TargetTracker tracker(cfg);

    // F1：角落目标（高置信度）先建立并被锁定
    tracker.Update({MakeDet(10.0, 10.0, 30.0, 30.0, 0.95)});
    // F2：画面中心附近目标建立；角落目标丢失(1)
    tracker.Update({MakeDet(300.0, 220.0, 340.0, 260.0, 0.9)});
    // F3：远处目标建立；角落丢失(2)、中心目标丢失(1)
    tracker.Update({MakeDet(600.0, 450.0, 620.0, 470.0, 0.85)});
    // F4：全部丢失：角落丢失(3) > max_lost 死亡解除锁定；
    // 其余两条 Coast 存活 → 重新选举选离画面中心最近者
    TrackResult r = tracker.Update({}, FrameShape{480, 640});
    ASSERT_TRUE(r.tracked);
    EXPECT_TRUE(r.is_predicted);
    EXPECT_NEAR(r.center.first, 320.0, 25.0);   // 靠近画面中心 (320,240) 者当选
    EXPECT_NEAR(r.center.second, 240.0, 25.0);
    EXPECT_EQ(r.n_active, 2);                    // 角落轨道已回收
}

// 锁定延续优先于置信度选举：锁定轨道实测时即使置信度较低也保持主目标（§6.3 ①）
TEST(TargetTrackerBehaviorTest, LockedPrimaryKeptWhileMeasured) {
    TargetTracker tracker;
    TrackResult r = tracker.Update({MakeDet(100.0, 100.0, 120.0, 120.0, 0.4)});
    ASSERT_TRUE(r.primary_id.has_value());
    const int locked_id = *r.primary_id;

    // 高置信度新目标出现并持续，锁定目标也持续实测 → primary_id 不变
    for (int i = 0; i < 5; ++i) {
        r = tracker.Update({MakeDet(100.0 + i, 100.0, 120.0 + i, 120.0, 0.4),
                            MakeDet(400.0, 300.0, 430.0, 330.0, 0.95)});
        ASSERT_TRUE(r.primary_id.has_value());
        EXPECT_EQ(*r.primary_id, locked_id);
        EXPECT_FALSE(r.is_predicted);
    }
}

// frame_shape 缺省按 640×480 画面中心 (320,240) 计算（§8.2 / §2.2）：
// 两条 Coast 轨道分别位于默认中心两侧，当选者应为更近的一侧
TEST(TargetTrackerBehaviorTest, DefaultFrameCenterIs320x240) {
    TargetTracker::Config cfg;
    cfg.max_lost_frames = 2;
    TargetTracker tracker(cfg);

    // F1：角落目标先建立并被锁定（高置信度）
    tracker.Update({MakeDet(0.0, 0.0, 20.0, 20.0, 0.95)});
    // F2：左侧目标（中心 (280,240)，距默认中心 40px）建立
    tracker.Update({MakeDet(270.0, 230.0, 290.0, 250.0, 0.9)});
    // F3：右侧目标（中心 (500,240)，距默认中心 180px）建立
    tracker.Update({MakeDet(490.0, 230.0, 510.0, 250.0, 0.85)});
    // F4：全部丢失：锁定轨道死亡解除锁定，两条 Coast 轨道兜底选举
    // （不传 frame_shape → 按默认画面中心 (320,240)；两候选间距足够大，
    // 速度估计漂移不影响最近者判定）
    TrackResult r = tracker.Update({});
    ASSERT_TRUE(r.tracked);
    EXPECT_TRUE(r.is_predicted);
    EXPECT_NEAR(r.center.first, 280.0, 30.0);   // 左侧更近，当选
    EXPECT_NEAR(r.center.second, 240.0, 30.0);
}

}  // namespace
