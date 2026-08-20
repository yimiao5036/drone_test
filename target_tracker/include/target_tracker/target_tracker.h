#pragma once

// =============================================================================
// target_tracker.h —— 多目标追踪过滤器（TargetTracker）对外接口
//
// 对应规格：docs/视觉追踪技术路线.md
//   §2 输入输出接口定义（Detection / TrackResult）
//   §3.3 TargetTracker（管理器）
//   §5 检测-轨迹关联（贪婪最邻近，§5.3）
//   §6 轨迹生命周期与 ID 管理（主目标锁定与选举，§6.3）
//   §7 主流程（每帧 Update 六步）
//
// 线程模型（§9.5）：Update() 由单线程（推理主循环）调用；库内仅对
// last_result 缓存加一把 std::mutex，LastResult() 返回受保护拷贝。
// 本库零第三方依赖（不依赖 spdlog / Eigen / OpenCV）。
// =============================================================================

#include <mutex>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "tracklet.h"

namespace drone {
namespace tracker {

/// 单个检测框（§2.2 输入 detections 元素：[x1,y1,x2,y2,conf,cls_id]）
struct Detection {
    double x1 = 0.0;      // 边界框左上角 x（像素坐标）
    double y1 = 0.0;      // 边界框左上角 y
    double x2 = 0.0;      // 边界框右下角 x
    double y2 = 0.0;      // 边界框右下角 y
    double conf = 0.0;    // 检测置信度 ∈ [0,1]
    int cls_id = 0;       // 类别编号
};

/// 画面尺寸（height, width，§2.2）：仅在"无实测轨道、只有 Coast 轨道"
/// 时用于选取离画面中心最近的轨道；缺省按 640×480 中心 (320,240)。
struct FrameShape {
    int height = 0;
    int width = 0;
};

/// 每帧追踪输出（§2.3）
struct TrackResult {
    bool tracked = false;                       // 是否存在有效主目标；false 时调用方应刹车
    std::optional<int> primary_id;              // 主目标轨道 ID；无目标时为空
    std::pair<double, double> center{0.0, 0.0}; // 平滑后的目标中心点（tracked=false 时无效）
    std::optional<BoxXYXY> box;                 // 平滑后的边界框
    double confidence = 0.0;                    // 当前置信度（Coast 期间继承最近实测值）
    bool is_predicted = false;                  // true = Coast（仅预测、无实测）
    int lost_frames = 0;                        // 主目标连续未命中帧数
    int n_active = 0;                           // 当前活跃轨道总数
    std::optional<BoxXYXY> raw;                 // 关联到的原始检测框；Coast 时为空（§2.3）
};

/// 多目标追踪过滤器（§1.1 / §3.3）
class TargetTracker {
public:
    /// 追踪器配置参数（§8.1，默认值取自规格）
    struct Config {
        int max_lost_frames = 8;          // 最大连续丢失帧数（严格大于才判死，§6.1）
        double max_association_dist = 200.0;  // 中心距离软阈值 Dmax（像素，§5.1）
        int min_hits = 3;                 // 轨道"确认"所需累计匹配次数（§6.1）
        double dist_weight = 0.5;         // 关联代价距离项权重（§5.1）
        double iou_weight = 0.5;          // 关联代价 (1−IoU) 项权重（§5.1）
    };

    /// 默认配置构造（等价于 Config{}，§8.1 默认值）
    TargetTracker();

    /// 指定配置构造
    explicit TargetTracker(const Config& cfg);

    /// 每帧调用一次（§7 六步主流程）：输入检测列表与可选画面尺寸，
    /// 输出主目标追踪结果并缓存（供 LastResult 跨线程读取）。
    TrackResult Update(const std::vector<Detection>& detections,
                       std::optional<FrameShape> frame_shape = std::nullopt);

    /// 上一帧结果快照（线程安全：返回受互斥锁保护的拷贝，§9.5）
    TrackResult LastResult() const;

    /// 清空所有轨道、ID 计数器、锁定状态与缓存结果（§2.1）；next_id 归零（§6.2）
    void Reset();

    /// 当前存活轨道数量（§2.1）
    int ActiveCount() const;

    /// 当前所有轨道的只读快照（§2.1；值拷贝）
    std::unordered_map<int, Tracklet> ActiveTracks() const;

    /// 配置只读访问
    const Config& GetConfig() const { return config_; }

private:
    /// 贪婪关联结果（§5.3）：matches 元素为 (track_id, 检测索引)
    struct AssociationResult {
        std::vector<std::pair<int, int>> matches;
        std::vector<int> unmatched_track_ids;
        std::vector<int> unmatched_det_indices;
    };

    /// 贪婪最邻近关联（§5.3）：按轨道 confidence 降序逐一匹配，
    /// cost = w_dist·dist/max(Dmax,1e-3) + w_iou·(1−IoU)，仅 cost 严格小于
    /// best_cost（初始 1.0）才匹配，即 cost ≥ 1.0 判为不可匹配（§5.1）。
    AssociationResult AssociateDetections(const std::vector<Detection>& detections);

    /// 主目标选择/锁定延续（§6.3 SELECT_PRIMARY）
    TrackResult SelectPrimary(std::optional<FrameShape> frame_shape);

    Config config_;                                   // 配置参数（§3.3）
    int next_id_ = 0;                                 // 下一个可分配 ID，从 0 递增不复用（§6.2）
    std::unordered_map<int, Tracklet> tracks_;        // 所有存活轨道（§3.3）
    long long frame_count_ = 0;                       // 已处理帧数（仅统计用，§3.3）
    std::optional<int> locked_primary_id_;            // 当前锁定的主目标 ID（§3.3）
    TrackResult last_result_;                         // 上一帧结果缓存（§3.3）
    mutable std::mutex last_result_mutex_;            // 仅保护 last_result_（§9.5）
};

}  // namespace tracker
}  // namespace drone
