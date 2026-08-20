// =============================================================================
// target_tracker.cpp —— 多目标追踪过滤器（TargetTracker）实现
//
// 对应规格：docs/视觉追踪技术路线.md
//   §5 检测-轨迹关联（贪婪最邻近，§5.3）
//   §6 轨迹生命周期与 ID 管理（主目标锁定与选举，§6.3）
//   §7 主流程六步（UPDATE 伪代码）
//   §9.5 线程模型（单线程写 Update，仅 last_result 缓存加锁）
// =============================================================================

#include "target_tracker/target_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone {
namespace tracker {

TargetTracker::TargetTracker() : TargetTracker(Config{}) {}

TargetTracker::TargetTracker(const Config& cfg) : config_(cfg) {}

TrackResult TargetTracker::Update(const std::vector<Detection>& detections,
                                  std::optional<FrameShape> frame_shape) {
    // §7：frame_count += 1（仅统计用）
    ++frame_count_;

    // ---- 入口防御：过滤坐标或置信度含 NaN/Inf 的检测 ----
    // 防御性增强：上游检测器理论上已过滤，但库自身必须免疫——
    // 非有限值一旦进入卡尔曼状态将永久污染轨道并传导到主目标输出
    std::vector<Detection> valid_detections;
    valid_detections.reserve(detections.size());
    for (const Detection& det : detections) {
        if (std::isfinite(det.x1) && std::isfinite(det.y1) &&
            std::isfinite(det.x2) && std::isfinite(det.y2) &&
            std::isfinite(det.conf)) {
            valid_detections.push_back(det);
        }
    }

    // ---- 步骤 1：所有存活轨道先行预测（推进一帧）----
    // （§7 步骤 1：t.predict() 更新 last_center / last_box、age += 1）
    for (auto& kv : tracks_) {
        kv.second.Predict();
    }

    // ---- 步骤 2：构造检测中心表并贪婪关联（§7 步骤 2 / §5.3） ----
    AssociationResult assoc;
    if (!valid_detections.empty()) {
        assoc = AssociateDetections(valid_detections);
    } else {
        // 无检测：全部轨道视为未匹配，全部进入 Coast 计数（§7 步骤 2 否则分支）
        for (const auto& kv : tracks_) {
            assoc.unmatched_track_ids.push_back(kv.first);
        }
    }

    // ---- 步骤 3：用匹配到的检测更新轨道（§7 步骤 3） ----
    for (const auto& match : assoc.matches) {
        const int track_id = match.first;
        const Detection& det = valid_detections[static_cast<size_t>(match.second)];
        // §7 步骤 3：由检测重建原始框 raw_box = [cx−w/2, cy−h/2, cx+w/2, cy+h/2]
        // （等价于检测框本身：cx=(x1+x2)/2、w=x2−x1 的逆运算，浮点上精确还原）
        const BoxXYXY raw_box{det.x1, det.y1, det.x2, det.y2};
        tracks_.at(track_id).Update(raw_box, det.conf, det.cls_id);
    }

    // ---- 步骤 4：未匹配轨道计丢失，死亡者回收（§7 步骤 4） ----
    for (const int track_id : assoc.unmatched_track_ids) {
        auto it = tracks_.find(track_id);
        if (it == tracks_.end()) {
            continue;
        }
        it->second.MarkMiss();  // lost_count+1、age+1
        if (it->second.IsDead()) {  // lost_count > max_lost（严格大于，§6.1）
            // 锁定的唯一解除途径之一：锁定轨道死亡被回收时清锁（§6.3）
            if (locked_primary_id_.has_value() && *locked_primary_id_ == track_id) {
                locked_primary_id_.reset();
            }
            tracks_.erase(it);
        }
    }

    // ---- 步骤 5：未匹配检测 → 创建新轨道（§7 步骤 5 / §6.2） ----
    for (const int det_idx : assoc.unmatched_det_indices) {
        const Detection& det = valid_detections[static_cast<size_t>(det_idx)];
        // §7 步骤 5：以 [cx,cy,w,h] 创建；构造函数内重建 last_box，
        // 保证创建当帧即可参与输出；next_id 从 0 递增永不复用（§6.2）
        tracks_.emplace(next_id_,
                        Tracklet(next_id_, ToCenterSize({det.x1, det.y1, det.x2, det.y2}),
                                 det.conf, det.cls_id, config_.max_lost_frames));
        ++next_id_;
    }

    // ---- 步骤 6：选取/延续主目标，组装输出并缓存（§7 步骤 6 / §6.3） ----
    TrackResult result = SelectPrimary(frame_shape);
    {
        std::lock_guard<std::mutex> lock(last_result_mutex_);
        last_result_ = result;  // 缓存供跨线程读取（§9.5）
    }
    return result;
}

TrackResult TargetTracker::LastResult() const {
    // §9.5：LastResult() 返回受互斥锁保护的拷贝
    std::lock_guard<std::mutex> lock(last_result_mutex_);
    return last_result_;
}

void TargetTracker::Reset() {
    // §2.1：清空所有轨道、ID 计数器、锁定状态与缓存结果
    tracks_.clear();
    next_id_ = 0;  // §6.2：reset() 时计数器归零
    frame_count_ = 0;
    locked_primary_id_.reset();  // 锁定的唯一解除途径之二（§6.3）
    std::lock_guard<std::mutex> lock(last_result_mutex_);
    last_result_ = TrackResult{};
}

int TargetTracker::ActiveCount() const {
    return static_cast<int>(tracks_.size());
}

std::unordered_map<int, Tracklet> TargetTracker::ActiveTracks() const {
    return tracks_;  // 值拷贝只读快照（§2.1）
}

TargetTracker::AssociationResult TargetTracker::AssociateDetections(
    const std::vector<Detection>& detections) {
    AssociationResult result;

    // §5.3：按轨道 confidence 降序排序（复制 ID 到 vector 后 sort，
    // 不直接排哈希表，§9.2）
    std::vector<int> sorted_ids;
    sorted_ids.reserve(tracks_.size());
    for (const auto& kv : tracks_) {
        sorted_ids.push_back(kv.first);
    }
    std::sort(sorted_ids.begin(), sorted_ids.end(), [this](int a, int b) {
        const double raw_a = tracks_.at(a).confidence;
        const double raw_b = tracks_.at(b).confidence;
        // NaN 安全：非有限置信度按 −∞ 处理（排最后），避免 NaN 参与比较
        // 破坏 std::sort 要求的严格弱序（未定义行为）；映射后为全序，
        // 等价传递性恒成立
        const double ca = std::isfinite(raw_a)
                              ? raw_a
                              : -std::numeric_limits<double>::infinity();
        const double cb = std::isfinite(raw_b)
                              ? raw_b
                              : -std::numeric_limits<double>::infinity();
        if (ca != cb) return ca > cb;  // 置信度从高到低
        return a < b;                  // 平局按 ID 升序作次级键，保证确定性
    });

    std::vector<bool> det_assigned(detections.size(), false);

    for (const int track_id : sorted_ids) {
        const Tracklet& track = tracks_.at(track_id);
        // §5.3：每条轨道最优代价初始化为 1.0（仅 cost 严格小于 1.0 才记录，
        // 即 cost ≥ 1.0 判为不可匹配，§5.1 匹配有效性判据）
        double best_cost = 1.0;
        int best_det = -1;
        for (size_t d = 0; d < detections.size(); ++d) {
            if (det_assigned[d]) continue;  // 跳过已分配检测
            const Detection& det = detections[d];
            // §5.1：中心点欧氏距离（像素）
            const double det_cx = (det.x1 + det.x2) / 2.0;
            const double det_cy = (det.y1 + det.y2) / 2.0;
            const double dist =
                std::hypot(track.last_center.first - det_cx,
                           track.last_center.second - det_cy);
            // §5.1：归一化距离，取 max(Dmax, 1e-3) 防除零
            const double norm_dist =
                dist / std::max(config_.max_association_dist, 1e-3);
            // §5.2：预测框与检测框的 IoU
            const double iou = track.last_box.has_value()
                                   ? ComputeIoU(*track.last_box,
                                                {det.x1, det.y1, det.x2, det.y2})
                                   : 0.0;
            // §5.1：cost = w_dist·norm_dist + w_iou·(1−IoU)
            const double cost = config_.dist_weight * norm_dist +
                                config_.iou_weight * (1.0 - iou);
            if (cost < best_cost) {  // 严格小于才更新（§5.1 / §5.3）
                best_cost = cost;
                best_det = static_cast<int>(d);
            }
        }
        if (best_det >= 0) {
            result.matches.emplace_back(track_id, best_det);
            det_assigned[static_cast<size_t>(best_det)] = true;
        } else {
            result.unmatched_track_ids.push_back(track_id);
        }
    }

    // §5.3：未分配检测收集
    for (size_t d = 0; d < detections.size(); ++d) {
        if (!det_assigned[d]) {
            result.unmatched_det_indices.push_back(static_cast<int>(d));
        }
    }
    return result;
}

TrackResult TargetTracker::SelectPrimary(std::optional<FrameShape> frame_shape) {
    TrackResult result;
    result.n_active = static_cast<int>(tracks_.size());

    // §6.3：轨道表为空 → 清除锁定，返回 tracked=false
    if (tracks_.empty()) {
        locked_primary_id_.reset();
        return result;
    }

    // §6.3 ① 锁定延续：锁定轨道仍存活则直接输出（Coast 中照样输出，
    // is_predicted=true；不被其他更高置信度目标抢占）
    Tracklet* chosen = nullptr;
    bool is_predicted = false;
    if (locked_primary_id_.has_value()) {
        auto it = tracks_.find(*locked_primary_id_);
        if (it != tracks_.end()) {
            chosen = &it->second;
            is_predicted = it->second.IsCoasting();
        }
    }

    // §6.3 ② 重新选举（无锁定 / 锁定目标已死亡被回收）
    if (chosen == nullptr) {
        // measured ← 所有 lost_count == 0 的轨道；coasting ← 其余存活轨道
        std::vector<Tracklet*> measured;
        std::vector<Tracklet*> coasting;
        for (auto& kv : tracks_) {
            if (kv.second.IsCoasting()) {
                coasting.push_back(&kv.second);
            } else {
                measured.push_back(&kv.second);
            }
        }

        if (!measured.empty()) {
            // confirmed ← measured 中 hits ≥ min_hits 者；为空则退回 measured
            // （min_hits 只影响候选池优先级，不阻止输出，§6.1 注记 1）
            std::vector<Tracklet*> pool;
            for (Tracklet* t : measured) {
                if (t->hits >= config_.min_hits) {
                    pool.push_back(t);
                }
            }
            if (pool.empty()) {
                pool = measured;
            }
            // chosen ← pool 中 confidence 最大者；平局取 unordered_map 遍历序
            // 中首个最大者，不做确定性承诺（规格 §6.3 未规定平局规则）
            chosen = pool.front();
            for (Tracklet* t : pool) {
                if (t->confidence > chosen->confidence) {
                    chosen = t;
                }
            }
            is_predicted = false;
        } else if (!coasting.empty()) {
            // 无实测轨道：选 coasting 中离画面中心最近者（欧氏距离平方排序）
            // §2.2：frame_shape 缺省按 640×480 画面中心 (320, 240)
            const double center_x =
                (frame_shape.has_value() && frame_shape->width > 0)
                    ? frame_shape->width / 2.0
                    : 320.0;
            const double center_y =
                (frame_shape.has_value() && frame_shape->height > 0)
                    ? frame_shape->height / 2.0
                    : 240.0;
            double best_dist_sq = std::numeric_limits<double>::infinity();
            for (Tracklet* t : coasting) {
                const double dx = t->last_center.first - center_x;
                const double dy = t->last_center.second - center_y;
                const double dist_sq = dx * dx + dy * dy;
                if (dist_sq < best_dist_sq) {
                    best_dist_sq = dist_sq;
                    chosen = t;
                }
            }
            if (chosen == nullptr) {
                // NaN 残余的防御兜底：若所有 coasting 轨道中心均为 NaN，
                // dist_sq 恒为 NaN 无人当选，取首个避免空指针解引用；
                // 正常输入已在 Update 入口经有限性过滤，本分支理论上不可达
                chosen = coasting.front();
            }
            is_predicted = true;  // Coast 兜底：仅预测、无实测
        } else {
            // 理论上不可达（轨道非空则 measured/coasting 必居其一）；防御清锁
            locked_primary_id_.reset();
            return result;
        }
        locked_primary_id_ = chosen->track_id;  // 选举后锁定（§6.3）
    }

    // ---- 组装输出（§2.3 九字段） ----
    result.tracked = true;
    result.primary_id = chosen->track_id;
    result.center = chosen->last_center;
    result.box = chosen->last_box;
    result.confidence = chosen->confidence;  // Coast 期间继承最近实测置信度
    result.is_predicted = is_predicted;
    result.lost_frames = chosen->lost_count;
    // §2.3：raw = 关联到的原始检测框；Coast 时为空（锁定延续分支同样遵守）
    result.raw = is_predicted ? std::nullopt : chosen->last_raw_box;
    return result;
}

}  // namespace tracker
}  // namespace drone
