// =============================================================================
// stereo_ranger_bench —— 双目测距 v2 开发机性能基准（手动运行，不注册 ctest）
//
// 用途：复现板上负载（1280×720 全幅合成图 + 标准尺寸检测框），输出
// StereoRangerCore 各阶段（remap / census / 提点 / 匹配 / 聚合）的
// avg/p95 耗时，用于定位热点与验证优化效果。
//
// 说明：
//   - 合成纹理在整幅内高频且确定（无随机源），框大小可调；
//     aux 图 = ref 按列整体位移 d(x)（近距大视差，覆盖完整搜索区间）；
//   - 校正矩阵直接用生产预置文件（真实畸变/旋转映射，remap 负载真实）；
//   - 开发机（x86）绝对值约为香橙派的 1/2~1/4，看**比例**定位热点；
//     优化后上板请对照组件摘要日志的"双目测距阶段耗时"行。
//
// 用法（仓库根目录或 build 目录均可，config/ 会同步到 build/）：
//   ./build/stereo_ranger_bench                          # 默认 30 次迭代、400x380 框
//   ./build/stereo_ranger_bench 60                       # 60 次迭代
//   ./build/stereo_ranger_bench 30 440 170 840 550      # 自定义框 x0 y0 x1 y1
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "perception/stereo_ranger_core.h"

namespace {

using json = nlohmann::json;
using drone::perception::GrayImageView;
using drone::perception::StereoRangeResult;
using drone::perception::StereoRangerConfig;
using drone::perception::StereoRangerCore;
using drone::perception::StereoRectifyMatrices;
using drone::perception::StereoStageLatency;

constexpr int kWidth = 1280;
constexpr int kHeight = 720;

StereoRectifyMatrices LoadMatrices(const std::string& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("无法打开校正矩阵: " + path);
    }
    const json root = json::parse(input);
    StereoRectifyMatrices m;
    m.view_width = root.at("view_width").get<int>();
    m.view_height = root.at("view_height").get<int>();
    m.baseline_m = root.at("baseline_m").get<double>();
    m.fx_rect_px = root.at("fx_rect_px").get<double>();
    m.fxb_m_px = root.at("fxb_m_px").get<double>();
    auto matrix = [&](const char* key, double* out, int rows, int cols) {
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                out[r * cols + c] = root.at(key).at(r).at(c).get<double>();
            }
        }
    };
    auto coeffs = [&](const char* key, double* out, int n) {
        for (int i = 0; i < n; ++i) {
            out[i] = root.at(key).at(i).get<double>();
        }
    };
    matrix("K_ref", m.k_ref, 3, 3);
    coeffs("D_ref", m.d_ref, 5);
    matrix("R_ref", m.r_ref, 3, 3);
    matrix("P_ref", m.p_ref, 3, 4);
    matrix("K_aux", m.k_aux, 3, 3);
    coeffs("D_aux", m.d_aux, 5);
    matrix("R_aux", m.r_aux, 3, 3);
    matrix("P_aux", m.p_aux, 3, 4);
    return m;
}

std::uint8_t TextureAt(int x, int y) {
    const int v =
        (x * 7 + y * 13 + (x / 3) * 5 + (y / 5) * 11 + (x * y) % 17) % 251;
    return static_cast<std::uint8_t>(v);
}

std::vector<std::uint8_t> MakeRefImage() {
    std::vector<std::uint8_t> img(static_cast<std::size_t>(kWidth) * kHeight);
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            img[static_cast<std::size_t>(y) * kWidth + x] = TextureAt(x, y);
        }
    }
    return img;
}

/// aux(x) = ref(x + d(x))：近距目标（d≈70px，约 0.9m），列向渐变。
std::vector<std::uint8_t> MakeAuxImage() {
    const std::vector<std::uint8_t> ref = MakeRefImage();
    std::vector<std::uint8_t> aux(static_cast<std::size_t>(kWidth) * kHeight);
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const int shift = 60 + (x * 20) / kWidth;  // 60..79 px
            const int sx = std::min(kWidth - 1, x + shift);
            aux[static_cast<std::size_t>(y) * kWidth + x] =
                ref[static_cast<std::size_t>(y) * kWidth + sx];
        }
    }
    return aux;
}

void PrintStage(const char* name, const drone::common::LatencySummary& s) {
    std::cout << "  " << name << ": avg=" << s.average_ms
              << "ms p95=" << s.p95_ms << "ms max=" << s.maximum_ms << "ms\n";
}

}  // namespace

int main(int argc, char** argv) {
    int iterations = 30;
    double box_x0 = 440.0;
    double box_y0 = 170.0;
    double box_x1 = 840.0;
    double box_y1 = 550.0;
    if (argc >= 2) {
        iterations = std::max(1, std::atoi(argv[1]));
    }
    if (argc >= 6) {
        box_x0 = std::atof(argv[2]);
        box_y0 = std::atof(argv[3]);
        box_x1 = std::atof(argv[4]);
        box_y1 = std::atof(argv[5]);
    }

    try {
        StereoRangerConfig cfg;
        cfg.disp_min_px = 8.0;
        cfg.disp_max_px = 160.0;
        cfg.warmup_frames = 0;
        const StereoRectifyMatrices matrices =
            LoadMatrices("config/stereo_rectify_uvc_1280x720.json");
        StereoRangerCore core(cfg, matrices);

        const std::vector<std::uint8_t> ref = MakeRefImage();
        const std::vector<std::uint8_t> aux = MakeAuxImage();
        const GrayImageView ref_view{ref.data(), kWidth, kHeight, kWidth};
        const GrayImageView aux_view{aux.data(), kWidth, kHeight, kWidth};

        drone::common::DetectionResult box;
        box.bbox_x = static_cast<float>(box_x0);
        box.bbox_y = static_cast<float>(box_y0);
        box.bbox_w = static_cast<float>(box_x1 - box_x0);
        box.bbox_h = static_cast<float>(box_y1 - box_y0);

        std::cout << "双目测距基准: 视差=[" << cfg.disp_min_px << ","
                  << cfg.disp_max_px << "]px 框=" << box.bbox_w << "x" << box.bbox_h
                  << " 迭代=" << iterations << "（开发机 x86 数字，板上约为 1/2~1/4）\n";

        // 预热（JIT/缓存/TLB 稳定）+ 计时
        for (int i = 0; i < 3; ++i) {
            (void)core.Process(ref_view, aux_view, {box});
        }
        const auto t0 = std::chrono::steady_clock::now();
        std::size_t valid = 0;
        for (int i = 0; i < iterations; ++i) {
            const auto results = core.Process(ref_view, aux_view, {box});
            if (!results.empty() && results[0].available) {
                ++valid;
            }
        }
        const auto t1 = std::chrono::steady_clock::now();
        const double total_ms =
            std::chrono::duration<double, std::milli>(t1 - t0).count();

        std::cout << "总计: " << total_ms / iterations << "ms/帧 ("
                  << 1000.0 * iterations / total_ms << " fps)，有效帧 "
                  << valid << "/" << iterations << "\n阶段耗时（滚动窗口）:\n";
        const StereoStageLatency stage = core.StageLatency();
        PrintStage("remap ", stage.remap);
        PrintStage("census", stage.census);
        PrintStage("提点  ", stage.keypoints);
        PrintStage("匹配  ", stage.match);
        PrintStage("聚合  ", stage.aggregate);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "基准失败: " << error.what() << "\n";
        return 1;
    }
}
