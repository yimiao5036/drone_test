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
#include <cstdio>
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

// 黄金数据（testdata）为 320×240 合成标定；合成视差场用例用 1280×240，
// 以便构造"参考窗口跨内列/边界列编码路径"的边缘关键点（列 ≥ W−3）。
using testdata::kViewHeight;
using testdata::kViewWidth;   // 仅黄金数据用例使用
constexpr int kSynthW = 1280;
constexpr int kSynthH = 240;

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
    std::vector<std::uint8_t> img(static_cast<std::size_t>(kSynthW) * kSynthH);
    for (int y = 0; y < kSynthH; ++y) {
        for (int x = 0; x < kSynthW; ++x) {
            img[static_cast<std::size_t>(y) * kSynthW + x] = TextureAt(x, y);
        }
    }
    return img;
}

/// 目标图：aux(x,y) = ref(x + d(x), y)（列相关视差场，便于近端簇/截断用例）。
std::vector<std::uint8_t> MakeAuxImage(const std::vector<int>& disp_for_col) {
    const std::vector<std::uint8_t> ref = MakeRefImage();
    std::vector<std::uint8_t> aux(static_cast<std::size_t>(kSynthW) * kSynthH);
    for (int y = 0; y < kSynthH; ++y) {
        for (int x = 0; x < kSynthW; ++x) {
            const int sx = std::min(kSynthW - 1, x + disp_for_col[x]);
            aux[static_cast<std::size_t>(y) * kSynthW + x] =
                ref[static_cast<std::size_t>(y) * kSynthW + sx];
        }
    }
    return aux;
}

/// 单位校正矩阵（无畸变、R=I、主点居中、基线 B）：rect↔source 映射退化为
/// 恒等，合成"已校正"输入图可直接构造已知视差场（不经过真实校正几何）。
StereoRectifyMatrices IdentityMatrices(double fx = 200.0, double baseline = 0.05) {
    StereoRectifyMatrices m;
    m.view_width = kSynthW;
    m.view_height = kSynthH;
    const double cx = 0.5 * (kSynthW - 1);
    const double cy = 0.5 * (kSynthH - 1);
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

/// 合成视差场用例的视图（1280×240）。
GrayImageView ViewSynth(const std::vector<std::uint8_t>& img) {
    return GrayImageView{img.data(), kSynthW, kSynthH, kSynthW};
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
    // 黄金数据为 320×240（testdata 尺寸），用 View 而非合成用例的 ViewSynth
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
    const auto aux = MakeAuxImage(std::vector<int>(kSynthW, 12));
    StereoRangerCore core(TestConfig(), IdentityMatrices());
    core.SetKeypointsOverride(GridKeypoints());
    const auto results =
        core.Process(ViewSynth(ref), ViewSynth(aux),
                     {MakeBox(20, 20, kSynthW - 15, kSynthH - 5)});
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
    std::vector<int> disp_for_col(kSynthW, 8);  // 远：d=8
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
    const auto results = core.Process(ViewSynth(ref), ViewSynth(aux),
                                      {MakeBox(20, 20, kSynthW - 10, 220)});
    ASSERT_TRUE(results[0].available);
    EXPECT_NEAR(results[0].disparity_px, 16.0, 0.5) << "近端簇应选前景 d=16";
    EXPECT_GE(results[0].cluster_points, 30);
}

// ---------------------------------------------------------------------------
// Census 编码规格：内部列（常量展开）与边界列（通用裁剪）两条路径逐像素对拍
// 规格实现——位序 win×win 行主序跳过中心、邻域 >= 中心、越界复制延拓
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, CensusEncodingMatchesSpecOnBothPaths) {
    const int w = 16;
    const int h = 12;
    std::vector<std::uint8_t> img(static_cast<std::size_t>(w) * h);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            img[static_cast<std::size_t>(y) * w + x] = TextureAt(x, y);
        }
    }
    const auto codes =
        StereoRangerCore::CensusEncodeForTest(img.data(), w, h, w, 7);
    ASSERT_EQ(codes.size(), static_cast<std::size_t>(w) * h);
    const int r = 3;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            // 独立的规格实现（不复用被测代码）
            const int center = img[static_cast<std::size_t>(y) * w + x];
            std::uint64_t expected = 0;
            int bit = 0;
            for (int dy = -r; dy <= r; ++dy) {
                const int ny = std::clamp(y + dy, 0, h - 1);
                for (int dx = -r; dx <= r; ++dx) {
                    if (dy == 0 && dx == 0) {
                        continue;
                    }
                    const int nx = std::clamp(x + dx, 0, w - 1);
                    if (img[static_cast<std::size_t>(ny) * w + nx] >= center) {
                        expected |= (std::uint64_t{1} << bit);
                    }
                    ++bit;
                }
            }
            EXPECT_EQ(codes[static_cast<std::size_t>(y) * w + x], expected)
                << "像素 (" << x << "," << y << ")";
        }
    }
}

// ---------------------------------------------------------------------------
// 边缘关键点匹配：参考窗口跨"内列/边界列"两条编码路径（列 ≥ w−3 走通用路径），
// 目标窗口落在内列——两条路径位序若不一致，这些点会匹配失败或视差错乱
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, MatchingWorksForKeypointsStraddlingEdgeEncoding) {
    const auto ref = MakeRefImage();
    const auto aux = MakeAuxImage(std::vector<int>(kSynthW, 12));
    StereoRangerConfig cfg = TestConfig();
    cfg.collect_detailed_latency = true;  // 本用例要用逐点快照
    StereoRangerCore core(cfg, IdentityMatrices());
    // 参考窗口落在 [W-7, W-2]：跨列 W-3（= kSynthW-3）的内列/边界列编码分界；
    // 目标窗口（左移 12px）完全落在内列——两条路径位序不一致时这些点必然失配。
    std::vector<std::pair<float, float>> keypoints;
    for (int gy = 0; gy < 6; ++gy) {
        for (int gx = 0; gx < 6; ++gx) {
            keypoints.emplace_back(static_cast<float>(kSynthW - 21 + gx * 3),
                                   40.0f + gy * 30.0f);
        }
    }
    core.SetKeypointsOverride(keypoints);
    const auto results = core.Process(
        ViewSynth(ref), ViewSynth(aux),
        {MakeBox(kSynthW - 40, 25, kSynthW, 215)});
    // 逐点直查：跨越编码分界的关键点必须全部有效且视差正确
    const auto points = core.LastPointDebugForTest();
    ASSERT_EQ(points.size(), keypoints.size());
    int valid = 0;
    for (const auto& p : points) {
        if (p.valid) {
            ++valid;
            EXPECT_NEAR(p.disparity, 12.0, 0.3)
                << "边界编码关键点 @" << p.x << "," << p.y;
        }
    }
    EXPECT_EQ(valid, static_cast<int>(keypoints.size()))
        << "跨编码分界的关键点存在失配（两条编码路径位序可能不一致）";
    ASSERT_TRUE(results[0].available)
        << "cluster_points=" << results[0].cluster_points
        << " d=" << results[0].disparity_px << " iqr=" << results[0].iqr_px;
    EXPECT_NEAR(results[0].disparity_px, 12.0, 0.3);
    EXPECT_GE(results[0].cluster_points, 30);
    const double fxb = IdentityMatrices().fxb_m_px;
    EXPECT_NEAR(results[0].z_m, fxb / 12.0, 0.02 * fxb / 12.0);
}

// ---------------------------------------------------------------------------
// 诊断开关：默认关闭时不留逐点快照、阶段窗口为空（生产热路径零诊断），
// 但测距功能不受影响
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, DetailedLatencyDisabledLeavesNoDiagnostics) {
    const auto ref = MakeRefImage();
    const auto aux = MakeAuxImage(std::vector<int>(kSynthW, 12));
    StereoRangerConfig cfg = TestConfig();
    cfg.collect_detailed_latency = false;  // 显式关闭，验证门控（不依赖默认值）
    StereoRangerCore core(cfg, IdentityMatrices());
    core.SetKeypointsOverride(GridKeypoints());
    const auto results =
        core.Process(ViewSynth(ref), ViewSynth(aux),
                     {MakeBox(20, 20, kSynthW - 15, kSynthH - 5)});
    EXPECT_TRUE(results[0].available) << "诊断关闭不应影响测距功能";
    EXPECT_TRUE(core.LastPointDebugForTest().empty());
    EXPECT_EQ(core.StageLatency().remap.window_count, 0u);
    EXPECT_EQ(core.StageLatency().match.window_count, 0u);
}

// ---------------------------------------------------------------------------
// 列裁剪等价：同一场景、同一批关键点，窄框（裁剪生效）与全宽框结果必须完全一致
// （裁剪只省算力，不改变任何被访问列与裁剪语义；用窄框直接压这条路径）
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, ColumnCropDoesNotChangeResults) {
    const auto ref = MakeRefImage();
    const auto aux = MakeAuxImage(std::vector<int>(kSynthW, 12));
    // 关键点全部落在窄框腐蚀后的区域内，两次聚合计数同一批点
    std::vector<std::pair<float, float>> keypoints;
    for (int gy = 0; gy < 6; ++gy) {
        for (int gx = 0; gx < 8; ++gx) {
            keypoints.emplace_back(212.0f + gx * 11.0f, 36.0f + gy * 33.0f);
        }
    }
    const StereoRectifyMatrices matrices = IdentityMatrices();
    StereoRangerCore narrow(TestConfig(), matrices);
    StereoRangerCore wide(TestConfig(), matrices);
    narrow.SetKeypointsOverride(keypoints);
    wide.SetKeypointsOverride(keypoints);

    const auto small =
        narrow.Process(ViewSynth(ref), ViewSynth(aux), {MakeBox(200, 25, 300, 225)});
    const auto full = wide.Process(ViewSynth(ref), ViewSynth(aux),
                                   {MakeBox(0, 0, kSynthW, kSynthH)});
    ASSERT_TRUE(small[0].available);
    ASSERT_TRUE(full[0].available);
    EXPECT_DOUBLE_EQ(small[0].z_m, full[0].z_m);
    EXPECT_DOUBLE_EQ(small[0].disparity_px, full[0].disparity_px);
    EXPECT_DOUBLE_EQ(small[0].iqr_px, full[0].iqr_px);
    EXPECT_EQ(small[0].cluster_points, full[0].cluster_points);
}

// ---------------------------------------------------------------------------
// 截断告警：视差顶到搜索上限
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, ClippedFlagAtSearchLimit) {
    const auto ref = MakeRefImage();
    const auto aux = MakeAuxImage(std::vector<int>(kSynthW, 40));  // = disp_max
    StereoRangerCore core(TestConfig(), IdentityMatrices());
    core.SetKeypointsOverride(GridKeypoints());
    const auto results =
        core.Process(ViewSynth(ref), ViewSynth(aux), {MakeBox(20, 20, kSynthW - 15, 230)});
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
    const auto aux_far = MakeAuxImage(std::vector<int>(kSynthW, 8));
    const auto aux_near = MakeAuxImage(std::vector<int>(kSynthW, 16));
    StereoRangerCore core(cfg, IdentityMatrices());
    core.SetKeypointsOverride(GridKeypoints());
    const auto box = MakeBox(20, 20, kSynthW - 15, 230);

    const auto first = core.Process(ViewSynth(ref), ViewSynth(aux_far), {box});
    ASSERT_TRUE(first[0].available);
    const double z_far = first[0].z_m;
    const double z_near = IdentityMatrices().fxb_m_px / 16.0;
    ASSERT_GT(z_far, z_near);

    const auto second = core.Process(ViewSynth(ref), ViewSynth(aux_near), {box});
    ASSERT_TRUE(second[0].available);
    EXPECT_NEAR(second[0].z_m, 0.4 * z_near + 0.6 * z_far, 0.02)
        << "EMA 应介于两帧原始距离之间";

    core.Reset();  // 清空 EMA：重新起步应回到裸值
    const auto third = core.Process(ViewSynth(ref), ViewSynth(aux_far), {box});
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
    const auto aux = MakeAuxImage(std::vector<int>(kSynthW, 12));
    StereoRangerCore core(cfg, IdentityMatrices());
    const auto box = MakeBox(20, 20, kSynthW - 15, 230);
    EXPECT_FALSE(core.Process(ViewSynth(ref), ViewSynth(aux), {box})[0].available);
    EXPECT_FALSE(core.Process(ViewSynth(ref), ViewSynth(aux), {box})[0].available);
    EXPECT_TRUE(core.Process(ViewSynth(ref), ViewSynth(aux), {box})[0].available);
    EXPECT_EQ(core.SkippedFrameCount(), 2u);

    // 黑帧（行带均值 < 阈值）跳过，不产生可用结果
    std::vector<std::uint8_t> black(
        static_cast<std::size_t>(kSynthW) * kSynthH, 0);
    EXPECT_FALSE(core.Process(ViewSynth(black), ViewSynth(black), {box})[0].available);
    EXPECT_EQ(core.SkippedFrameCount(), 3u);
}

// ---------------------------------------------------------------------------
// 输入尺寸非法：不崩溃、返回不可用
// ---------------------------------------------------------------------------
TEST(StereoRangerCoreTest, RejectsMismatchedInputSize) {
    const auto ref = MakeRefImage();
    const auto aux = MakeAuxImage(std::vector<int>(kSynthW, 12));
    StereoRangerCore core(TestConfig(), IdentityMatrices());
    const GrayImageView bad{ref.data(), kSynthW - 2, kSynthH, kSynthW - 2};
    const auto results =
        core.Process(bad, ViewSynth(aux), {MakeBox(20, 20, kSynthW - 15, 230)});
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
