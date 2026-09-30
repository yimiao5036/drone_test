/**
 * @file offline_pgm_demo.cpp
 * @brief 离线验证：读一对左右目 PGM(P5) 灰度图 + 标定，跑全流程并打印区域距离
 *
 * 用途：脱离主项目在开发机/板上快速验证精度。
 * 用法：
 *   stereo_depth_demo <left.pgm> <right.pgm> [calib.json] [config.json] [x y w h]
 * 缺省标定/配置用编译期内置路径；缺省查询区域取图像中心 64×64。
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

#include "stereo_depth/stereo_depth_sensor.h"

namespace {

/// 极简 PGM(P5) 读取：返回灰度平面，宽高经引用返回。失败返回空。
std::vector<unsigned char> ReadPgm(const char* path, int* w, int* h) {
    std::vector<unsigned char> data;
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::cerr << "无法打开 " << path << "\n";
        return data;
    }
    char magic[3] = {0};
    f.read(magic, 2);
    if (std::strcmp(magic, "P5") != 0) {
        std::cerr << "非 P5 PGM: " << path << "\n";
        return data;
    }
    int width = 0, height = 0, maxv = 0;
    // 跳过空白与注释
    auto skip_ws = [&]() {
        while (true) {
            int c = f.get();
            if (c == '#') {
                while (c != '\n' && c != EOF) c = f.get();
            } else if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                continue;
            } else {
                f.unget();
                break;
            }
        }
    };
    skip_ws();
    f >> width;
    skip_ws();
    f >> height;
    skip_ws();
    f >> maxv;
    f.get();  // 吃掉 maxv 后的单个空白
    if (width <= 0 || height <= 0 || maxv != 255) {
        std::cerr << "PGM 头非法（仅支持 maxval=255）\n";
        return data;
    }
    data.resize(static_cast<std::size_t>(width) * height);
    f.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
    *w = width;
    *h = height;
    return data;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "用法: " << argv[0]
                  << " <left.pgm> <right.pgm> [calib.json] [config.json] [x y w h]\n";
        return 2;
    }
    int lw = 0, lh = 0, rw = 0, rh = 0;
    std::vector<unsigned char> left = ReadPgm(argv[1], &lw, &lh);
    std::vector<unsigned char> right = ReadPgm(argv[2], &rw, &rh);
    if (left.empty() || right.empty()) return 1;
    if (lw != rw || lh != rh) {
        std::cerr << "左右目尺寸不一致\n";
        return 1;
    }

    const char* calib = (argc > 3) ? argv[3] : nullptr;
    const char* cfgp = (argc > 4) ? argv[4] : STEREO_DEPTH_DEMO_CONFIG;

    stereo_depth::StereoDepthConfig cfg;
    try {
        cfg = stereo_depth::LoadStereoDepthConfig(cfgp);
    } catch (const std::exception& e) {
        std::cerr << "配置加载失败: " << e.what() << "\n";
        return 1;
    }
    cfg.source_width = static_cast<std::uint32_t>(lw);
    cfg.source_height = static_cast<std::uint32_t>(lh);

    stereo_depth::StereoDepthSensor sensor(cfg);
    const bool ok = calib ? sensor.Init(calib) : sensor.Init();
    if (!ok) {
        std::cerr << "传感器初始化失败\n";
        return 1;
    }

    stereo_depth::GrayImage L{left.data(), static_cast<std::uint32_t>(lw),
                              static_cast<std::uint32_t>(lh),
                              static_cast<std::uint32_t>(lw), 0};
    stereo_depth::GrayImage R{right.data(), static_cast<std::uint32_t>(rw),
                              static_cast<std::uint32_t>(rh),
                              static_cast<std::uint32_t>(rw), 0};
    stereo_depth::StereoFramePair pair{L, R};
    if (!sensor.Process(pair)) {
        std::cerr << "Process 失败\n";
        return 1;
    }

    stereo_depth::QueryRect q;
    if (argc >= 9) {
        q.x = std::atoi(argv[5]);
        q.y = std::atoi(argv[6]);
        q.w = std::atoi(argv[7]);
        q.h = std::atoi(argv[8]);
    } else {
        q.w = 64;
        q.h = 64;
        q.x = (lw - q.w) / 2;
        q.y = (lh - q.h) / 2;
    }
    const stereo_depth::DistanceResult res = sensor.Query(q);
    std::printf("查询区域 [%d,%d,%d,%d]\n", q.x, q.y, q.w, q.h);
    std::printf("valid=%d distance=%.3fm disparity=%.3fpx confidence=%.2f valid_ratio=%u%%\n",
                res.valid ? 1 : 0, res.distance_m, res.disparity_px, res.confidence,
                res.valid_pixel_ratio_x100);
    return 0;
}
