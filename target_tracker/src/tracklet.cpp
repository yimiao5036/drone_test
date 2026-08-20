// =============================================================================
// tracklet.cpp —— 轨道（Tracklet）与基础几何工具实现
//
// 对应规格：docs/视觉追踪技术路线.md
//   §5.2 IoU 计算
//   §7   TRACKLET_PREDICT / TRACKLET_UPDATE 伪代码
//   §3.2 Tracklet 字段语义
// =============================================================================

#include "target_tracker/tracklet.h"

#include <algorithm>

namespace drone {
namespace tracker {

BoxCXCYWH ToCenterSize(const BoxXYXY& box) {
    // §7 TRACKLET_UPDATE：cx=(x1+x2)/2；cy=(y1+y2)/2；w=x2−x1；h=y2−y1
    return BoxCXCYWH{(box.x1 + box.x2) / 2.0, (box.y1 + box.y2) / 2.0,
                     box.x2 - box.x1, box.y2 - box.y1};
}

BoxXYXY ToXYXY(const BoxCXCYWH& box) {
    // §7 TRACKLET_PREDICT：center_size → [cx−w/2, cy−h/2, cx+w/2, cy+h/2]
    return BoxXYXY{box.cx - box.w / 2.0, box.cy - box.h / 2.0,
                   box.cx + box.w / 2.0, box.cy + box.h / 2.0};
}

double ComputeIoU(const BoxXYXY& a, const BoxXYXY& b) {
    // §5.2：任一框为空（宽或高 ≤ 0）返回 0
    const double area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    const double area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    if (area_a <= 0.0 || area_b <= 0.0) {
        return 0.0;
    }

    // 交集宽高与 0 取最大值（§5.2）
    const double inter_w = std::min(a.x2, b.x2) - std::max(a.x1, b.x1);
    const double inter_h = std::min(a.y2, b.y2) - std::max(a.y1, b.y1);
    if (inter_w <= 0.0 || inter_h <= 0.0) {
        return 0.0;  // 交集面积 ≤ 0 返回 0（§5.2）
    }

    const double inter_area = inter_w * inter_h;
    const double union_area = area_a + area_b - inter_area;
    if (union_area <= 0.0) {
        return 0.0;  // 并集 ≤ 0 返回 0（§5.2）
    }
    // IoU = area(A∩B) / (area(A)+area(B)−area(A∩B))（§5.2）
    return inter_area / union_area;
}

Tracklet::Tracklet(int id, const BoxCXCYWH& measurement, double conf, int cls,
                   int max_lost_frames)
    : track_id(id),
      max_lost(max_lost_frames),
      confidence(conf),
      cls_id(cls) {
    // §4.3：用首次观测初始化滤波器（速度置零、P 取默认对角）
    kf.Initiate(measurement);
    // §7 步骤 5：创建当帧即赋 last_center / last_box（保证当帧可参与输出）
    last_center = {measurement.cx, measurement.cy};
    last_box = ToXYXY(measurement);
    // 创建即源于实测检测：同步记录原始检测框（Coast 期间保留，
    // 对外输出按 §2.3 在 Coast 时置空）
    last_raw_box = last_box;
    // age / hits / lost_count 由类内默认值保证（1 / 1 / 0，§3.2）
}

void Tracklet::Predict() {
    // §7 TRACKLET_PREDICT：卡尔曼预测推进一帧
    const auto pred = kf.Predict();
    if (pred.has_value()) {
        // 预测成功：更新 last_center / last_box，age += 1
        last_center = {pred->cx, pred->cy};
        last_box = ToXYXY(*pred);
        ++age;
    }
}

void Tracklet::Update(const BoxXYXY& box, double conf, int cls) {
    // §7 TRACKLET_UPDATE：框 → [cx,cy,w,h] → kf.update
    kf.Update(ToCenterSize(box));
    last_center = {(box.x1 + box.x2) / 2.0, (box.y1 + box.y2) / 2.0};
    last_box = box;
    last_raw_box = box;      // 实测关联到的原始检测框（Coast 期间保留不更新）
    confidence = conf;
    cls_id = cls;
    lost_count = 0;          // 匹配成功：连续丢失计数清零
    ++hits;                  // 累计匹配次数 +1
    ++age;                   // 帧龄 +1
}

void Tracklet::MarkMiss() {
    // §7 步骤 4：未匹配计丢失 lost_count+1、age+1
    ++lost_count;
    ++age;
}

}  // namespace tracker
}  // namespace drone
