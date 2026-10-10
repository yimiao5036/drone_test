#pragma once

// =============================================================================
// stereo_ranger_core.h —— 双目测距纯算法核心（v2：稀疏 Census 路线）
//
// 对应设计：docs/superpowers/specs/2026-10-10-双目测距v2-稀疏Census-design.md
// 参考实现：tools/camera_test_ok/stereo_core.py（Python 已验证）、C++移植指南.md
//
// 目序（写死，别动）：
//   基准图 = SBS 右半 = 物理左目 = 主工程"左目 Topic"（kDecodedFrameLeft）；
//   辅助图 = SBS 左半 = 物理右目。视差 d = u_ref − u_aux > 0；Z = fxb_m / d。
//   校正矩阵以 cam2（SBS 右半）为基准，由 tools/camera_calibration/export_rectify.py
//   预置（config/stereo_rectify_uvc_1280x720.json），本模块不依赖 OpenCV。
//
// 职责（每帧一次）：原始框→校正域映射 → 行带 remap → Shi-Tomasi 关键点 →
//   Census 7×7 → 粗/精搜索 + 抛物线亚像素 + 唯一性 + 左右一致性 → bbox 近端簇
//   聚合（Z/X/Y/欧氏/IQR）→ EMA 平滑。
//
// 边界：纯算法类，不碰 Topic/线程/日志；线程不安全由组件壳独占调用；
//   不做多目标跟踪、不做 NED 融合、不做饱和点剔除（仅截断标志）。
// =============================================================================

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/latency_statistics.h"
#include "common/types.h"

namespace drone::perception {

/// 预置校正矩阵（config/stereo_rectify_uvc_1280x720.json，由
/// tools/camera_calibration/export_rectify.py 生成；解析在 config 模块完成，
/// perception 只消费结构体）。基准/辅助的 K/D/R/P 语义见文件内 note；
/// 行主序 3×3 / 3×4。
struct StereoRectifyMatrices {
    int view_width = 0;   ///< 单目视图宽（= 拆分后每目宽，如 1280）
    int view_height = 0;  ///< 单目视图高（如 720）
    double k_ref[9]{};    ///< 基准相机内参（SBS 右半）
    double d_ref[5]{};    ///< 基准相机畸变 k1 k2 p1 p2 k3
    double r_ref[9]{};    ///< 基准校正旋转
    double p_ref[12]{};   ///< 基准校正投影（fx=fy=P(0,0)、主点 P(0,2)/P(1,2)）
    double k_aux[9]{};    ///< 辅助相机内参（SBS 左半）
    double d_aux[5]{};    ///< 辅助相机畸变
    double r_aux[9]{};    ///< 辅助校正旋转
    double p_aux[12]{};   ///< 辅助校正投影
    double baseline_m = 0.0;   ///< 校正后基线（米，正值）= |P_aux(0,3)/P_aux(0,0)|
    double fx_rect_px = 0.0;   ///< 校正后水平焦距 = P_ref(0,0)
    double fxb_m_px = 0.0;     ///< 深度系数 = fx_rect_px × baseline_m（m·px）
};

/// 测距算法与组件参数。字段默认值即参考实现（distance_app.py）实测配置；
/// 输入队列/帧缓冲/配对窗/摘要间隔等仅组件壳使用。
struct StereoRangerConfig {
    std::string rectify_matrices_path =
        "stereo_rectify_uvc_1280x720.json";  ///< 预置矩阵（与 config.json 同目录；config 模块加载，核心只消费结构体）

    // ---- 匹配（路线 B 稀疏 Census）----
    double disp_min_px = 8.0;    ///< 视差下限（px）；8→约 7.9m，放宽会变慢并引入远距噪声
    double disp_max_px = 160.0;  ///< 视差上限（px）；160→约 0.40m（量程近端）
    int coarse_stride = 2;       ///< 粗搜索步长；=4 在准周期纹理上会整周期跳过真实极小值
    int fine_halfwidth = 6;      ///< 精搜索半宽（step=1）
    double uniqueness_ratio = 1.05;  ///< 唯一性：峰位 ±2 之外次优/最优 ≥ 该值才接受
    double lr_tol_px = 1.0;      ///< 左右一致性容差（px）；0=关闭
    int census_win = 7;          ///< Census 窗口（3/5/7；7→48bit，uint64）
    int max_points_per_target = 80;  ///< 每目标最大关键点数
    int grid_cells = 4;          ///< 关键点网格分散（4→4×4 轮转取点）

    // ---- 帧可用性 ----
    int warmup_frames = 15;              ///< 预热帧数（曝光/增益收敛），其间不匹配
    double black_frame_mean_threshold = 2.0;  ///< 行带均值低于该值按黑帧跳过

    // ---- 聚合（bbox 近端簇）----
    int bbox_erode_px = 5;     ///< bbox 向内腐蚀（剔除前后景混合边缘）
    int min_points = 30;       ///< 近端簇有效点数下限；不足判不可用
    double near_pct = 90.0;    ///< 近端簇分位（%）
    double near_tol_px = 3.0;  ///< 近端簇容差（px）
    double smooth_alpha = 0.4; ///< EMA 系数 [0,1]；0=关闭

    // ---- 组件壳（配对/缓冲/日志）----
    std::size_t input_queue_capacity = 2;    ///< 每路订阅队列容量（丢最旧）
    std::size_t frame_buffer_depth = 4;      ///< 帧环形缓冲深度（受拆分器池容量约束，≥2）
    std::chrono::milliseconds pair_window{20};           ///< 左右帧配对时间窗
    std::chrono::milliseconds settle{50};                ///< 检测批次闭合超时
    std::chrono::milliseconds summary_log_interval{10000};  ///< 摘要日志间隔

    /// 取值校验；非法抛 std::invalid_argument 并携带字段名。
    void Validate() const;
};

/// 单目灰度视图（Y 平面，不拥有数据；stride 单位=像素）。
struct GrayImageView {
    const std::uint8_t* data = nullptr;
    int width = 0;
    int height = 0;
    int stride = 0;

    [[nodiscard]] bool Valid() const noexcept {
        return data != nullptr && width > 0 && height > 0 && stride >= width;
    }
};

/// 单帧各阶段耗时快照（毫秒，滚动窗口统计；摘要日志与开发机基准用）。
/// 跳过帧（预热/黑帧）只记录已执行到的阶段，未执行的保持空窗口。
struct StereoStageLatency {
    common::LatencySummary remap;      ///< 行带 remap（双目）+ 黑帧检查
    common::LatencySummary keypoints;  ///< 掩膜 + Shi-Tomasi + 网格取点
    common::LatencySummary census;     ///< Census 变换（双目）
    common::LatencySummary match;      ///< 粗/精搜索 + 亚像素 + 唯一性 + 左右一致性
    common::LatencySummary aggregate;  ///< 逐框近端簇聚合 + EMA
};

/// 单框测距结果（校正图＝基准图像素坐标系；米制字段不可用时为 NaN）。
struct StereoRangeResult {
    bool available = false;  ///< 近端簇点数 ≥ min_points（匹配与聚合成功）
    bool clipped = false;    ///< 中位视差顶到搜索上限（距离可能更近，仅告警）
    double z_m = 0.0;        ///< 前向距离 Z（米，EMA 平滑后）
    double euclid_m = 0.0;   ///< 直线距离（按平滑后 Z 重算）
    double x_m = 0.0;        ///< 基准相机系横向（右为正，按平滑后 Z 重算）
    double y_m = 0.0;        ///< 基准相机系纵向（下为正）
    double disparity_px = 0.0;  ///< 近端簇中位视差（px，原始值）
    double iqr_px = 0.0;        ///< 近端簇视差四分位距（置信度，越小越稳）
    int cluster_points = 0;     ///< 近端簇点数
    double center_x_rect = 0.0; ///< 腐蚀后框中心 x（校正图，供方位字段）
    double center_y_rect = 0.0;
};

/// 双目测距纯算法核心。线程不安全，由组件壳独占调用。
class StereoRangerCore {
public:
    StereoRangerCore(StereoRangerConfig config, StereoRectifyMatrices matrices);
    ~StereoRangerCore();  // 源文件默认实现（PIMPL 完整类型要求）

    StereoRangerCore(const StereoRangerCore&) = delete;
    StereoRangerCore& operator=(const StereoRangerCore&) = delete;

    /// 处理一帧：ref=基准灰度（左目 Topic 的 Y 平面）/aux=辅助灰度；
    /// boxes=左目检测框（**原始左图**像素坐标）。返回与 boxes 等长的结果数组；
    /// 帧不可用（黑帧/预热）或框越界时对相应框返回 available=false，不抛异常。
    std::vector<StereoRangeResult> Process(
        const GrayImageView& ref, const GrayImageView& aux,
        const std::vector<common::DetectionResult>& boxes);

    // ---- 测试接口（隔离 Shi-Tomasi 与 OpenCV 的实现差异）----
    /// 注入关键点（校正图像素坐标），跳过 Shi-Tomasi；传空 vector 恢复默认检测。
    void SetKeypointsOverride(std::vector<std::pair<float, float>> rect_points);

    /// 测试接口：对给定灰度图做 Census 编码（与内部实现同一函数，含内部列的
    /// 常量展开路径与边界列的通用裁剪路径）。用于逐像素校验两条路径位序一致
    /// （位序 = win×win 行主序跳过中心；邻域 >= 中心；越界按复制延拓）。
    static std::vector<std::uint64_t> CensusEncodeForTest(const std::uint8_t* data,
                                                          int width, int height,
                                                          int stride, int win);

    /// 测试接口：逐关键点结果（校正图坐标/视差/有效性）。最近一次 Process 的
    /// 快照；跳过帧或无关键点返回空。用于定位个别关键点的匹配异常。
    struct PointDebug {
        float x = 0.0f;
        float y = 0.0f;
        float disparity = 0.0f;
        float uniqueness_ratio = 0.0f;  ///< 峰位 ±2 外次优/最优
        float lr_best_back = 0.0f;      ///< 回搜得到的最优候选视差
        bool lr_ok = false;             ///< 左右一致性是否通过
        bool valid = false;             ///< 有限 && 唯一性 && LR
    };
    std::vector<PointDebug> LastPointDebugForTest() const;

    // ---- 单点映射（黄金对拍/排查）----
    /// 校正图 → 源图采样坐标（映射表双线性查询）；越界返回 false。
    bool RectToSourceRef(float u_rect, float v_rect, float* u_src,
                         float* v_src) const;
    /// 源图（原始基准图）→ 校正图（解析式：畸变逆解定点迭代 + 校正旋转/投影）；
    /// 用于把检测框从原始坐标系映射到校正坐标系。
    bool SourceToRectRef(double u_src, double v_src, double* u_rect,
                         double* v_rect) const;

    // ---- 统计（摘要日志/测试）----
    std::uint64_t ProcessedFrameCount() const;   ///< 已处理帧数（含跳过）
    std::uint64_t MatchedFrameCount() const;     ///< 实际执行匹配的帧数
    std::uint64_t AvailableCount() const;        ///< 累计可用测距框数
    std::uint64_t ClippedCount() const;          ///< 累计截断告警框数
    std::uint64_t SkippedFrameCount() const;     ///< 黑帧+预热跳过帧数
    double LastMatchMs() const;                  ///< 最近一次匹配耗时（ms）
    double LastTotalMs() const;                  ///< 最近一次 Process 耗时（ms）
    /// 各阶段耗时滚动窗口快照（每帧热路径 Add，摘要时 Snapshot 排序）。
    StereoStageLatency StageLatency() const;

    /// 清空 EMA/统计状态（组件 Stop 时调用）。
    void Reset();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace drone::perception
