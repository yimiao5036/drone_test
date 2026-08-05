#include "yolo26_detector.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

#include "common.h"
#include "rknn_model.h"

namespace {

// 检查模型文件是否存在且可读
bool file_readable(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return f.good();
}

} // namespace

// ------------------------------------------------------------
// 实现细节（PIMPL）：持有 rknn_model，封装完整推理流程。
// 本结构体只出现在 .cpp 中，对外头文件完全不暴露 RKNN 细节。
// ------------------------------------------------------------
struct YOLO26Detector::Impl {
    rknn_model model;   // RKNN 初始化 / NPU 推理 / 后处理 + NMS 全部由它完成
    int ctx_index = 0;  // 使用的 NPU 上下文索引

    explicit Impl(const std::string& model_path) : model(model_path) {}
};

YOLO26Detector::YOLO26Detector(const std::string& model_path) {
    if (!file_readable(model_path)) {
        throw std::runtime_error("YOLO26Detector: model file not found: " + model_path);
    }
    impl_ = std::make_unique<Impl>(model_path);
}

YOLO26Detector::~YOLO26Detector() = default;
YOLO26Detector::YOLO26Detector(YOLO26Detector&&) noexcept = default;
YOLO26Detector& YOLO26Detector::operator=(YOLO26Detector&&) noexcept = default;

std::vector<Detection> YOLO26Detector::detect(const cv::Mat& image) {
    std::vector<Detection> detections;

    // 只支持 8 位图像；通道数不是 3 时先统一到 3 通道 BGR
    if (image.empty() || image.depth() != CV_8U) {
        return detections;
    }

    // 将非 3 通道图像转为 3 通道 BGR（RGA 内部完成 BGR→RGB 色彩转换）
    cv::Mat bgr;
    switch (image.channels()) {
    case 1:
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
        break;
    case 4:
        cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
        break;
    case 3:
        bgr = image;  // 已是 BGR，仅复制 Mat 头，无数据拷贝
        break;
    default:
        return detections;
    }

    // 内部预处理第一步会“居中裁剪为正方形且边长为 16 的倍数”（见 run_inference 内部），
    // 这里按同样规则预先算出裁剪偏移，推理完成后用它把检测框从裁剪子图坐标系映射回原图坐标系
    const int short_side = (std::min(bgr.cols, bgr.rows) / 16) * 16;
    if (short_side <= 0) {
        return detections; // 图像过小，无法满足 16 对齐要求
    }
    const int crop_x = (bgr.cols - short_side) / 2;
    const int crop_y = (bgr.rows - short_side) / 2;

    // 完整推理流程：RGA letterbox 预处理（含 BGR→RGB） -> NPU 推理 -> 解码 + NMS
    object_detect_result_list od_results;
    if (impl_->model.run_inference(bgr, impl_->ctx_index, &od_results) < 0) {
        return detections;
    }

    detections.reserve(od_results.count);
    for (int i = 0; i < od_results.count; ++i) {
        const object_detect_result& r = od_results.results[i];

        Detection det;
        det.class_id = r.cls_id;
        det.confidence = r.prop;
        // 结果坐标位于裁剪子图坐标系，加回裁剪偏移并夹取到原图边界
        det.x1 = clamp(r.box.left + crop_x, 0, image.cols);
        det.y1 = clamp(r.box.top + crop_y, 0, image.rows);
        det.x2 = clamp(r.box.right + crop_x, 0, image.cols);
        det.y2 = clamp(r.box.bottom + crop_y, 0, image.rows);

        // 过滤掉裁剪/夹取后退化为无效框的结果
        if (det.x2 > det.x1 && det.y2 > det.y1) {
            detections.push_back(det);
        }
    }
    return detections;
}
