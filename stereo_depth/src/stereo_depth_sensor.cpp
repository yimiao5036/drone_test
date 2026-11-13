/**
 * @file stereo_depth_sensor.cpp
 * @brief 传感器组件壳实现（同步编排）
 */
#include "stereo_depth/stereo_depth_sensor.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "stereo_depth/census_transform.h"
#include "stereo_depth/disparity_postprocess.h"

namespace stereo_depth {
namespace {

/// 异常日志节流：第 1 次与每满 100 次返回 true。
bool ShouldLogThrottled(std::uint64_t& counter) {
    ++counter;
    return counter == 1 || counter % 100 == 0;
}

/// 从全分辨率 rectified 平面裁剪 [x0,x1)×[y0,y1) 到 dst（行主序，宽 x1-x0）。
void CropGray(const std::uint8_t* src, std::uint32_t src_w, int x0, int y0,
              int cw, int ch, std::uint8_t* dst) {
    for (int r = 0; r < ch; ++r) {
        const std::uint8_t* srow =
            src + static_cast<std::size_t>(y0 + r) * src_w + static_cast<std::size_t>(x0);
        std::uint8_t* drow = dst + static_cast<std::size_t>(r) * static_cast<std::size_t>(cw);
        std::copy_n(srow, static_cast<std::size_t>(cw), drow);
    }
}

}  // namespace

StereoDepthSensor::StereoDepthSensor(StereoDepthConfig config)
    : cfg_(std::move(config)) {}

StereoDepthSensor::~StereoDepthSensor() = default;

bool StereoDepthSensor::Init() {
    if (cfg_.calibration_path.empty()) {
        spdlog::error("[stereo_depth] 未指定标定文件路径，Init 失败");
        ++error_count_;
        return false;
    }
    return Init(cfg_.calibration_path);
}

bool StereoDepthSensor::Init(const std::string& calibration_path) {
    try {
        cfg_.Validate();
        cal_ = LoadStereoCalibration(calibration_path);
    } catch (const std::exception& e) {
        spdlog::error("[stereo_depth] 配置/标定加载失败: {}", e.what());
        ++error_count_;
        return false;
    }

    if (cal_.image_width != cfg_.source_width || cal_.image_height != cfg_.source_height) {
        spdlog::error("[stereo_depth] 标定分辨率 {}x{} 与 source {}x{} 不符",
                      cal_.image_width, cal_.image_height, cfg_.source_width,
                      cfg_.source_height);
        ++error_count_;
        return false;
    }

    try {
        rectifier_.Build(cal_, cfg_.source_width, cfg_.source_height);
        SgmConfig sc;
        sc.max_disparity = cfg_.max_disparity;
        sc.paths = cfg_.sgm_paths;
        sc.p1 = cfg_.penalty_p1;
        sc.p2 = cfg_.penalty_p2;
        sc.uniqueness_ratio = cfg_.uniqueness_ratio;
        matcher_ = std::make_unique<SgmMatcher>(
            sc, std::unique_ptr<ISubpixelRefiner>(
                    CreateSubpixelRefiner(cfg_.subpixel.c_str())));
    } catch (const std::exception& e) {
        spdlog::error("[stereo_depth] 校正/匹配器构建失败: {}", e.what());
        ++error_count_;
        return false;
    }

    const std::size_t npix =
        static_cast<std::size_t>(cfg_.source_width) * cfg_.source_height;
    rect_left_.assign(npix, 0);
    rect_right_.assign(npix, 0);
    has_frame_ = false;
    ready_ = true;

    spdlog::info("[stereo_depth] 初始化完成: rect={}x{}, f={:.1f}, B={:.4f}m, "
                 "f·B={:.2f}, D={}, paths={}, census={}x{}, subpixel={}, 有效域=[{:.1f},{:.1f}]m",
                 rectifier_.RectWidth(), rectifier_.RectHeight(), rectifier_.Focal(),
                 rectifier_.Baseline(), rectifier_.FocalTimesBaseline(),
                 cfg_.max_disparity, cfg_.sgm_paths, cfg_.census_window,
                 cfg_.census_window, cfg_.subpixel, cfg_.distance_min_m,
                 cfg_.distance_max_m);
    return true;
}

bool StereoDepthSensor::Process(const StereoFramePair& pair) {
    if (!ready_) {
        if (ShouldLogThrottled(warn_counter_))
            spdlog::warn("[stereo_depth] 未初始化即 Process（累计 {} 次）", warn_counter_);
        ++error_count_;
        return false;
    }
    if (!pair.Valid() || pair.left.width != cfg_.source_width ||
        pair.left.height != cfg_.source_height) {
        if (ShouldLogThrottled(warn_counter_))
            spdlog::warn("[stereo_depth] 输入帧尺寸非法/不匹配（累计 {} 次）", warn_counter_);
        ++error_count_;
        return false;
    }
    rectifier_.RectifyLeft(pair.left, rect_left_.data());
    rectifier_.RectifyRight(pair.right, rect_right_.data());
    has_frame_ = true;
    ++processed_frames_;
    return true;
}

double StereoDepthSensor::ApplyFilter(std::int32_t track_id, double z_raw) {
    FilterState& fs = filters_[track_id];
    if (!fs.initialized) {
        fs.z = z_raw;
        fs.initialized = true;
        return z_raw;
    }
    if (std::fabs(z_raw - fs.z) > cfg_.jump_reset_m) {
        fs.z = z_raw;  // 跳变重置，不滤波
    } else {
        fs.z = cfg_.smooth_alpha * z_raw + (1.0 - cfg_.smooth_alpha) * fs.z;
    }
    return fs.z;
}

DistanceResult StereoDepthSensor::Query(const QueryRect& rect) {
    DistanceResult res;  // 默认 invalid / NaN
    ++query_count_;
    if (!ready_ || !has_frame_) return res;
    if (!rect.Valid()) return res;

    const int rectW = static_cast<int>(rectifier_.RectWidth());
    const int rectH = static_cast<int>(rectifier_.RectHeight());

    // ---- 原始左目像素系 → rectified 系（4 角取包围盒）----
    double xs[2] = {static_cast<double>(rect.x), static_cast<double>(rect.x + rect.w)};
    double ys[2] = {static_cast<double>(rect.y), static_cast<double>(rect.y + rect.h)};
    double umin = 1e18, umax = -1e18, vmin = 1e18, vmax = -1e18;
    for (double sx : xs) {
        for (double sy : ys) {
            double u, v;
            rectifier_.SourceToRectifiedLeft(sx, sy, &u, &v);
            umin = std::min(umin, u);
            umax = std::max(umax, u);
            vmin = std::min(vmin, v);
            vmax = std::max(vmax, v);
        }
    }
    int gx0 = static_cast<int>(std::floor(umin));
    int gy0 = static_cast<int>(std::floor(vmin));
    int gx1 = static_cast<int>(std::ceil(umax));
    int gy1 = static_cast<int>(std::ceil(vmax));
    gx0 = std::clamp(gx0, 0, rectW);
    gy0 = std::clamp(gy0, 0, rectH);
    gx1 = std::clamp(gx1, 0, rectW);
    gy1 = std::clamp(gy1, 0, rectH);
    if (gx1 <= gx0 || gy1 <= gy0) return res;

    // ---- ROI 裁剪范围（含 margin 上下文 + 左侧 D 搜索余量）----
    const int D = cfg_.max_disparity;
    const int M = cfg_.roi_margin_px;
    int cropX0 = std::max(0, gx0 - M - D);
    int cropY0 = std::max(0, gy0 - M);
    int cropX1 = std::min(rectW, gx1 + M);
    int cropY1 = std::min(rectH, gy1 + M);
    const int cw = cropX1 - cropX0;
    const int ch = cropY1 - cropY0;
    if (cw <= 0 || ch <= 0) return res;

    const std::size_t cn = static_cast<std::size_t>(cw) * static_cast<std::size_t>(ch);
    roi_left_.resize(cn);
    roi_right_.resize(cn);
    CropGray(rect_left_.data(), rectifier_.RectWidth(), cropX0, cropY0, cw, ch,
             roi_left_.data());
    CropGray(rect_right_.data(), rectifier_.RectWidth(), cropX0, cropY0, cw, ch,
             roi_right_.data());

    // ---- Census → SGM → 后处理 ----
    census_left_.resize(cn);
    census_right_.resize(cn);
    CensusTransform(roi_left_.data(), static_cast<std::uint32_t>(cw),
                    static_cast<std::uint32_t>(ch), static_cast<std::uint32_t>(cw),
                    cfg_.census_window, census_left_.data());
    CensusTransform(roi_right_.data(), static_cast<std::uint32_t>(cw),
                    static_cast<std::uint32_t>(ch), static_cast<std::uint32_t>(cw),
                    cfg_.census_window, census_right_.data());

    disp_.resize(cn);
    matcher_->ComputeDisparity(census_left_.data(), census_right_.data(),
                               static_cast<std::uint32_t>(cw),
                               static_cast<std::uint32_t>(ch), disp_.data());
    disp_filt_.resize(cn);
    MedianFilter3x3(disp_.data(), static_cast<std::uint32_t>(cw),
                    static_cast<std::uint32_t>(ch), disp_filt_.data());
    if (cfg_.speckle_max_size > 1) {
        SpeckleFilter(disp_filt_.data(), static_cast<std::uint32_t>(cw),
                      static_cast<std::uint32_t>(ch), 1.0, cfg_.speckle_max_size);
    }

    // ---- 区域聚合（目标子区在 ROI 内的局部坐标）----
    const int lx0 = gx0 - cropX0;
    const int ly0 = gy0 - cropY0;
    const RegionStat st = AggregateRegion(
        disp_filt_.data(), static_cast<std::uint32_t>(cw), static_cast<std::uint32_t>(ch),
        lx0, ly0, gx1 - gx0, gy1 - gy0, rectifier_.FocalTimesBaseline(),
        cfg_.distance_min_m, cfg_.distance_max_m, cfg_.query_use_median);

    res.valid_pixel_ratio_x100 =
        static_cast<std::uint32_t>(std::round(st.valid_ratio * 100.0));
    if (st.valid_count == 0 || !std::isfinite(st.disparity_px)) return res;
    if (st.valid_ratio < cfg_.min_valid_ratio) return res;
    if (st.disparity_px < cfg_.disparity_min_px) return res;
    if (!std::isfinite(st.distance_m)) return res;

    res.disparity_px = st.disparity_px;
    res.distance_m = ApplyFilter(rect.track_id, st.distance_m);
    res.confidence = static_cast<float>(std::clamp(st.valid_ratio, 0.0, 1.0));
    res.valid = true;
    ++valid_count_;
    return res;
}

DistanceResult StereoDepthSensor::QueryPoint(std::int32_t x, std::int32_t y,
                                             std::int32_t track_id) {
    QueryRect r;
    r.x = x - 2;
    r.y = y - 2;
    r.w = 5;
    r.h = 5;
    r.track_id = track_id;
    return Query(r);
}

void StereoDepthSensor::ResetFilters() { filters_.clear(); }

}  // namespace stereo_depth
