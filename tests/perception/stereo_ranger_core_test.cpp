// =============================================================================
// stereo_ranger_core_test —— 双目测距 v2（稀疏 Census）核心单测
//
// 覆盖：
//   - 校正映射表黄金采样点（对拍 OpenCV initUndistortRectifyMap）
//   - 源图→校正图映射与映射表互逆（往返一致性）
//   - 黄金场景端到端：合成标定渲染图 + 注入关键点 → 聚合距离/视差/点数/IQR
//     对拍 Python 参考实现（tests/perception/data/stereo_golden.h）
//   - 合成视差场：整数视差复原、近端簇选择、截断标志、EMA 平滑
//   - 预热/黑帧跳过、输入尺寸非法、配置与矩阵校验拒绝
// =============================================================================

#include "perception/stereo_ranger_core.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "data/stereo_golden.h"

namespace drone::perception {
namespace {

using testdata::kViewHeight;
using testdata::kViewWidth;

StereoRangerConfig TestConfig() {
    StereoRangerConfig cfg;
    cfg.disp_min_px = 4.0;
    cfg.disp_max_px = 40.0;
    cfg.coarse_stride = 2;
    cfg.fine_halfwidth = 6;
    cfg.warmup_frames = 0;
    cfg.smooth_alpha = 0.0;  // 默认关闭 EMA，EMA 用例单独开
    return cfg;
}

common::DetectionResult MakeBox(double x0, double y0, double x1, double y1,
                                std::uint32_t class_id = 0) {
    common::DetectionResult box;
    box.class_id = class_id;
    box.confidence = 0.9f;
    box.bbox_x = static_cast<float>(x0);
    box.bbox_y = static_cast<float>(y0);
    box.bbox_w = static_cast<float>(x1 - x0);
    box.bbox_h = static_cast<float>(y1 - y0);
    return box;
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
    if (input.gcount() != static_cast<std::streamsize>(data.size())) {
        throw std::runtime_error("黄金数据长度不符: " + path);
    }
    return data;
}

/// 合成确定性纹理，供视差场用例（与黄金数据无关，自包含）。
std::uint8_t TextureAt(int x, int y) {
    const int v =
        (x * 7 + y * 13 + (x / 3) * 5 + (y / 5) * 11 + (x * y) % 17) % 251;
    return static_cast<std::uint8_t>(v);
}

std::vector<std::uint8_t> MakeRefImage() {
    std::vector<std::uint8_t> img(static_cast<std::size_t>(kViewWidth) * kViewHeight);
    for (int y = 0; y < kViewHeight; ++y) {
        for (int x = 0; x < kViewWidth; ++x) {
            img[static_cast<std::size_t>(y) * kViewWidth + x] = TextureAt(x, y);
        }
    }
    return img;
}

/// 目标图：aux(x,y) = ref(x + d(x), y)（列相关视差场，便于近端簇/截断用例）。
std::vector<std::uint8_t> MakeAuxImage(const std::vector<int>& disp_for_col) {
    const std::vector<std::uint8_t> ref = MakeRefImage();
    std::vector<std::uint8_t> aux(static_cast<std::size_t>(kViewWidth) * kViewHeight);
    for (int y = 0; y < kViewHeight; ++y) {
        for (int x = 0; x < kViewWidth; ++x) {
            const int sx = std::min(kViewWidth - 1, x + disp_for_col[x]);
            aux[static_cast<std::size_t>(y) * kViewWidth + x] =
                ref[static_cast<std::size_t>(y) * kViewWidth + sx];
        }
    }
    return aux;
}

/// 单位校正矩阵（无畸变、R=I、主点居中、基线 B）：rect↔source 映射退化为
/// 恒等，合成"已校正"输入图可直接构造已知视差场（不经过真实校正几何）。
StereoRectifyMatrices IdentityMatrices(double fx = 200.0, double baseline = 0.05) {
    StereoRectifyMatrices m;
    m.view_width = kViewWidth;
    m.view_height = kViewHeight;
    const double cx = 0.5 * (kViewWidth - 1);
    const double cy = 0.5 * (kViewHeight - 1);
    m.k_ref[0] = fx; m.k_ref[4] = fx; m.k_ref[2] = cx; m.k_ref[5] = cy; m.k_ref[8] = 1.0;
    m.k_aux[0] = fx; m.k_aux[4] = fx; m.k_aux[2] = cx; m.k_aux[5] = cy; m.k_aux[8] = 1.0;
    m.r_ref[0] = m.r_ref[4] = m.r_ref[8] = 1.0;
    m.r_aux[0] = m.r_aux[4] = m.r_aux[8] = 1.0;
    m.p_ref[0] = fx; m.p_ref[5] = fx; m.p_ref[2] = cx; m.p_ref[6] = cy; m.p_ref[10] = 1.0;
    m.p_aux[0] = fx; m.p_aux[5] = fx; m.p_aux[2] = cx; m.p_aux[6] = cy; m.p_aux[10] = 1.0;
    m.p_aux[3] = -fx * baseline;
    m.baseline_m = baseline;
    m.fx_rect_px = fx;
    m.fxb_m_px = fx * baseline;
    return m;
}

GrayImageView View(const std::vector<std::uint8_t>& img) {
    return GrayImageView{img.data(), kViewWidth, kViewHeight, kViewWidth};
}

/// 6×8 网格关键点（≥min_points=30），覆盖目标区域。
std::vector<std::pair<float, float>> GridKeypoints() {
    std::vector<std::pair<float, float>> keypoints;
    for (int gy = 0; gy < 6; ++gy) {
        for (int gx = 0; gx < 8; ++gx) {
            keypoints.emplace_back(26.0f + gx * 36.0f, 34.0f + gy * 34.0f);
        }
    }
    return keypoints;
}

// ---------------------------------------------------------------------------
// 校正映射表：黄金采样点（对拍 OpenCV）
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, RectifyMapMatchesOpenCvGolden) {
    StereoRangerCore core(TestConfig(), testdata::Matrices());
    for (const auto& s : testdata::kMapSamples) {
        float u = 0.0f;
        float v = 0.0f;
        ASSERT_TRUE(core.RectToSourceRef(static_cast<float>(s.u),
                                         static_cast<float>(s.v), &u, &v))
            << "采样点越界: " << s.u << "," << s.v;
        EXPECT_NEAR(u, s.ref_x, 0.01f) << "ref_x @" << s.u << "," << s.v;
        EXPECT_NEAR(v, s.ref_y, 0.01f) << "ref_y @" << s.u << "," << s.v;
    }
}

// ---------------------------------------------------------------------------
// 映射互逆：校正→源→校正 往返一致（解析式与映射表互相印证）
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, SourceToRectRoundTrip) {
    StereoRangerCore core(TestConfig(), testdata::Matrices());
    int checked = 0;
    for (int y = 20; y < kViewHeight - 20; y += 17) {
        for (int x = 20; x < kViewWidth - 20; x += 19) {
            float su = 0.0f;
            float sv = 0.0f;
            ASSERT_TRUE(core.RectToSourceRef(static_cast<float>(x),
                                             static_cast<float>(y), &su, &sv));
            double rx = 0.0;
            double ry = 0.0;
            ASSERT_TRUE(core.SourceToRectRef(su, sv, &rx, &ry));
            EXPECT_NEAR(rx, x, 0.35) << "往返 x @" << x << "," << y;
            EXPECT_NEAR(ry, y, 0.35) << "往返 y @" << x << "," << y;
            ++checked;
        }
    }
    EXPECT_GT(checked, 100);
}

// ---------------------------------------------------------------------------
// 黄金场景端到端：与 Python 参考实现（同一合成标定 + 渲染图）对拍
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, GoldenSceneMatchesReferenceAggregate) {
    const std::vector<std::uint8_t> ref = LoadBin("stereo_ref.bin");
    const std::vector<std::uint8_t> aux = LoadBin("stereo_aux.bin");

    StereoRangerCore core(TestConfig(), testdata::Matrices());
    std::vector<std::pair<float, float>> keypoints;
    for (const auto& kp : testdata::kKeypoints) {
        keypoints.emplace_back(kp[0], kp[1]);
    }
    core.SetKeypointsOverride(keypoints);

    const auto box = MakeBox(testdata::kBboxX0, testdata::kBboxY0,
                             testdata::kBboxX1, testdata::kBboxY1);
    const auto results = core.Process(View(ref), View(aux), {box});
    ASSERT_EQ(results.size(), 1u);
    ASSERT_TRUE(results[0].available) << "黄金场景应可用（簇点数不足）";

    EXPECT_NEAR(results[0].z_m, testdata::kExpectedZ, 0.02)
        << "端到端 Z 与参考实现偏差过大";
    EXPECT_NEAR(results[0].disparity_px, testdata::kExpectedDisparity, 0.3)
        << "端到端视差与参考实现偏差过大";
    EXPECT_NEAR(results[0].iqr_px, testdata::kExpectedIqr, 0.15);
    EXPECT_NEAR(results[0].euclid_m, testdata::kExpectedEuclid, 0.05);
    EXPECT_GE(results[0].cluster_points, testdata::kExpectedClusterPoints - 3);
    EXPECT_FALSE(results[0].clipped);
    EXPECT_EQ(core.AvailableCount(), 1u);
    EXPECT_EQ(core.MatchedFrameCount(), 1u);
}

// ---------------------------------------------------------------------------
// 合成视差场：整数视差复原
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, RecoversConstantDisparity) {
    const auto ref = MakeRefImage();
    const auto aux = MakeAuxImage(std::vector<int>(kViewWidth, 12));
    StereoRangerCore core(TestConfig(), IdentityMatrices());
    core.SetKeypointsOverride(GridKeypoints());
    const auto results =
        core.Process(View(ref), View(aux),
                     {MakeBox(20, 20, kViewWidth - 15, kViewHeight - 5)});
    ASSERT_TRUE(results[0].available);
    EXPECT_NEAR(results[0].disparity_px, 12.0, 0.3);
    const double fxb = IdentityMatrices().fxb_m_px;
    EXPECT_NEAR(results[0].z_m, fxb / 12.0, 0.02 * fxb / 12.0);
    EXPECT_LT(results[0].iqr_px, 1.0);
}

// ---------------------------------------------------------------------------
// 近端簇：bbox 内前/背景两个深度峰 → 取近端
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, NearClusterPicksForeground) {
    const auto ref = MakeRefImage();
    std::vector<int> disp_for_col(kViewWidth, 8);  // 远：d=8
    for (int x = 0; x < 150; ++x) {
        disp_for_col[x] = 16;  // 近：d=16（左半）
    }
    const auto aux = MakeAuxImage(disp_for_col);
    StereoRangerCore core(TestConfig(), IdentityMatrices());
    // 近端 32 点（x<150），远端 16 点（x≥185），避开视差场边界
    std::vector<std::pair<float, float>> keypoints;
    for (int gy = 0; gy < 4; ++gy) {
        for (int gx = 0; gx < 8; ++gx) {
            keypoints.emplace_back(24.0f + gx * 15.0f, 40.0f + gy * 40.0f);
        }
    }
    for (int gy = 0; gy < 4; ++gy) {
        for (int gx = 0; gx < 4; ++gx) {
            keypoints.emplace_back(185.0f + gx * 28.0f, 40.0f + gy * 40.0f);
        }
    }
    core.SetKeypointsOverride(keypoints);
    const auto results = core.Process(View(ref), View(aux),
                                      {MakeBox(20, 20, kViewWidth - 10, 220)});
    ASSERT_TRUE(results[0].available);
    EXPECT_NEAR(results[0].disparity_px, 16.0, 0.5) << "近端簇应选前景 d=16";
    EXPECT_GE(results[0].cluster_points, 30);
}

// ---------------------------------------------------------------------------
// 截断告警：视差顶到搜索上限
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, ClippedFlagAtSearchLimit) {
    const auto ref = MakeRefImage();
    const auto aux = MakeAuxImage(std::vector<int>(kViewWidth, 40));  // = disp_max
    StereoRangerCore core(TestConfig(), IdentityMatrices());
    core.SetKeypointsOverride(GridKeypoints());
    const auto results =
        core.Process(View(ref), View(aux), {MakeBox(20, 20, 300, 230)});
    ASSERT_TRUE(results[0].available);
    EXPECT_TRUE(results[0].clipped);
    EXPECT_EQ(core.ClippedCount(), 1u);
}

// ---------------------------------------------------------------------------
// EMA 平滑：第二帧距离减半 → 平滑值介于两帧之间；Reset 后回到裸值
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, EmaSmoothingBetweenFrames) {
    StereoRangerConfig cfg = TestConfig();
    cfg.smooth_alpha = 0.4;
    const auto ref = MakeRefImage();
    const auto aux_far = MakeAuxImage(std::vector<int>(kViewWidth, 8));
    const auto aux_near = MakeAuxImage(std::vector<int>(kViewWidth, 16));
    StereoRangerCore core(cfg, IdentityMatrices());
    core.SetKeypointsOverride(GridKeypoints());
    const auto box = MakeBox(20, 20, 300, 230);

    const auto first = core.Process(View(ref), View(aux_far), {box});
    ASSERT_TRUE(first[0].available);
    const double z_far = first[0].z_m;
    const double z_near = IdentityMatrices().fxb_m_px / 16.0;
    ASSERT_GT(z_far, z_near);

    const auto second = core.Process(View(ref), View(aux_near), {box});
    ASSERT_TRUE(second[0].available);
    EXPECT_NEAR(second[0].z_m, 0.4 * z_near + 0.6 * z_far, 0.02)
        << "EMA 应介于两帧原始距离之间";

    core.Reset();  // 清空 EMA：重新起步应回到裸值
    const auto third = core.Process(View(ref), View(aux_far), {box});
    ASSERT_TRUE(third[0].available);
    EXPECT_NEAR(third[0].z_m, z_far, 0.02);
}

// ---------------------------------------------------------------------------
// 预热与黑帧：跳过匹配但仍返回等长结果（全部不可用）
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, WarmupAndBlackFrameSkipped) {
    StereoRangerConfig cfg = TestConfig();
    cfg.warmup_frames = 2;
    const auto ref = MakeRefImage();
    const auto aux = MakeAuxImage(std::vector<int>(kViewWidth, 12));
    StereoRangerCore core(cfg, IdentityMatrices());
    const auto box = MakeBox(20, 20, 300, 230);
    EXPECT_FALSE(core.Process(View(ref), View(aux), {box})[0].available);
    EXPECT_FALSE(core.Process(View(ref), View(aux), {box})[0].available);
    EXPECT_TRUE(core.Process(View(ref), View(aux), {box})[0].available);
    EXPECT_EQ(core.SkippedFrameCount(), 2u);

    // 黑帧（行带均值 < 阈值）跳过，不产生可用结果
    std::vector<std::uint8_t> black(
        static_cast<std::size_t>(kViewWidth) * kViewHeight, 0);
    EXPECT_FALSE(core.Process(View(black), View(black), {box})[0].available);
    EXPECT_EQ(core.SkippedFrameCount(), 3u);
}

// ---------------------------------------------------------------------------
// 输入尺寸非法：不崩溃、返回不可用
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, RejectsMismatchedInputSize) {
    const auto ref = MakeRefImage();
    const auto aux = MakeAuxImage(std::vector<int>(kViewWidth, 12));
    StereoRangerCore core(TestConfig(), IdentityMatrices());
    const GrayImageView bad{ref.data(), kViewWidth - 2, kViewHeight, kViewWidth - 2};
    const auto results = core.Process(bad, View(aux), {MakeBox(20, 20, 300, 230)});
    ASSERT_EQ(results.size(), 1u);
    EXPECT_FALSE(results[0].available);
    EXPECT_EQ(core.MatchedFrameCount(), 0u);
}

// ---------------------------------------------------------------------------
// 配置校验：非法取值拒绝
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, ConfigValidationRejectsInvalidValues) {
    auto expect_throw = [](StereoRangerConfig cfg) {
        EXPECT_THROW(cfg.Validate(), std::invalid_argument);
    };
    StereoRangerConfig cfg = TestConfig();
    cfg.disp_max_px = cfg.disp_min_px;  // 必须大于
    expect_throw(cfg);
    cfg = TestConfig();
    cfg.census_win = 4;
    expect_throw(cfg);
    cfg = TestConfig();
    cfg.near_pct = 0.0;
    expect_throw(cfg);
    cfg = TestConfig();
    cfg.smooth_alpha = 1.5;
    expect_throw(cfg);
    cfg = TestConfig();
    cfg.frame_buffer_depth = 1;
    expect_throw(cfg);
    cfg = TestConfig();
    cfg.coarse_stride = 0;
    expect_throw(cfg);
    EXPECT_NO_THROW(TestConfig().Validate());
}

// ---------------------------------------------------------------------------
// 矩阵自洽：view 尺寸/焦距/基线非法时构造抛异常
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, RejectsInvalidMatrices) {
    StereoRectifyMatrices m = testdata::Matrices();
    m.baseline_m = 0.0;
    EXPECT_THROW(StereoRangerCore(TestConfig(), m), std::invalid_argument);
    m = testdata::Matrices();
    m.view_width = 0;
    EXPECT_THROW(StereoRangerCore(TestConfig(), m), std::invalid_argument);
    m = testdata::Matrices();
    m.fxb_m_px = m.fxb_m_px * 1.1;  // 与 P 不自洽
    EXPECT_THROW(StereoRangerCore(TestConfig(), m), std::invalid_argument);
    EXPECT_NO_THROW(StereoRangerCore(TestConfig(), testdata::Matrices()));
}

}  // namespace
}  // namespace drone::perception
