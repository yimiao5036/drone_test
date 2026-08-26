#pragma once

// =============================================================================
// tracklet.h —— 轨道（Tracklet）与基础几何工具
//
// 对应规格：docs/视觉追踪技术路线.md
//   §3.2 Tracklet（轨道）字段定义与派生状态
//   §5.2 IoU 计算
//   §7   主流程伪代码末尾 TRACKLET_PREDICT / TRACKLET_UPDATE
//
// 几何量说明：BoxCXCYWH（卡尔曼观测空间 [cx,cy,w,h]）定义于
// kalman_box_filter.h，本文件通过 include 复用并同时提供 BoxXYXY
// （像素坐标 [x1,y1,x2,y2]）与两种表示的互转函数。
// 本库只依赖 C++ 标准库，无任何第三方依赖。
// =============================================================================

#include <optional>
#include <utility>

#include "kalman_box_filter.h"

namespace drone {
namespace tracker {

// ---- 基础几何量（§9.1 模块/类划分草案） ----

/// 轴对齐边界框（像素坐标，左上/右下角表示）
struct BoxXYXY {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
};

/// [x1,y1,x2,y2] → [cx,cy,w,h]（§7 TRACKLET_UPDATE 中的转换）
BoxCXCYWH ToCenterSize(const BoxXYXY& box);

/// [cx,cy,w,h] → [x1,y1,x2,y2]（§7 TRACKLET_PREDICT 中的 center_size 逆变换）
BoxXYXY ToXYXY(const BoxCXCYWH& box);

/// 标准轴对齐 IoU（§5.2）：
/// 任一框为空、交集面积 ≤ 0 或并集 ≤ 0 时返回 0。
double ComputeIoU(const BoxXYXY& a, const BoxXYXY& b);

// =============================================================================
// Tracklet —— 单条轨道：生命周期计数 + 辅助状态 + 卡尔曼滤波器（§3.2）
// =============================================================================
struct Tracklet {
    /// 构造即创建轨道（§7 步骤 5 / §6.1 新建阶段）：
    /// 用首次观测初始化滤波器，age=1、hits=1、lost_count=0，
    /// last_center / last_box 立即由观测重建（保证创建当帧即可参与输出）。
    Tracklet(int id, const BoxCXCYWH& measurement, double conf, int cls,
             int max_lost_frames);

    int track_id = 0;                 // 全局唯一轨道 ID（§3.2）
    KalmanBoxFilter kf;               // 本轨道的滤波器（§3.2）
    int lost_count = 0;               // 当前连续未匹配帧数（§3.2）
    int max_lost = 0;                 // 最大允许连续丢失帧数（构造时注入，§3.2）
    double confidence = 0.0;          // 最近一次匹配的置信度（§3.2）
    int cls_id = 0;                   // 最近一次匹配的类别 ID（§3.2）
    int age = 1;                      // 轨道已存在的总帧数（创建时为 1，§3.2）
    int hits = 1;                     // 累计成功匹配次数（创建时为 1，§3.2）
    std::pair<double, double> last_center{0.0, 0.0};  // 最近一次中心点（实测或预测）
    std::optional<BoxXYXY> last_box;  // 最近一次边界框（实测或预测，§3.2）
    std::optional<BoxXYXY> last_raw_box;  // 最近一次实测关联到的原始检测框；
                                          // Coast 期间保留不更新，仅内部参考；
                                          // 对外输出语义按 §2.3：Coast 时 raw 置空

    // ---- 派生状态（§3.2 只读属性） ----

    /// is_dead := lost_count > max_lost（严格大于，§6.1 注记 2）
    bool IsDead() const { return lost_count > max_lost; }

    /// is_coasting := lost_count > 0（处于纯预测状态，§3.2）
    bool IsCoasting() const { return lost_count > 0; }

    // ---- 生命周期方法（§7 末尾伪代码） ----

    /// TRACKLET_PREDICT：卡尔曼预测推进一帧。
    /// 预测成功时更新 last_center / last_box 且 age+1。
    void Predict();

    /// TRACKLET_UPDATE：框 [x1,y1,x2,y2] → [cx,cy,w,h] → kf.update；
    /// lost_count 清零、hits+1、age+1，更新
    /// last_center / last_box / last_raw_box / confidence / cls_id。
    void Update(const BoxXYXY& box, double conf, int cls);

    /// 未匹配计丢失：lost_count+1、age+1（§7 步骤 4）。
    void MarkMiss();
};

}  // namespace tracker
}  // namespace drone
