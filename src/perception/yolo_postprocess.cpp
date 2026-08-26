/**
 * @file yolo_postprocess.cpp
 * @brief YOLO 检测后处理纯函数实现
 *
 * 移植自 videoPart/yolo26-rknn/src/common.cpp 与 rknn_model.cpp 的 post_process，
 * 保留数值逻辑，去掉 RKNN/OpenCV 依赖，参数化类别数与分支结构以便单元测试。
 */
#include "perception/yolo_postprocess.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>

namespace drone::perception {

namespace {

/// 数值夹取（用于量化参数换算）。
inline std::int32_t ClipFloat(float value, float min, float max) {
    return static_cast<std::int32_t>(value <= min ? min : (value >= max ? max : value));
}

/// 浮点 → 仿射量化 INT8（阈值换算用）。
inline std::int8_t QuantToAffine(float f32, std::int32_t zp, float scale) {
    if (scale <= 0.f) {
        return 0;
    }
    const float dst = (f32 / scale) + static_cast<float>(zp);
    return static_cast<std::int8_t>(ClipFloat(dst, -128.f, 127.f));
}

/// 仿射量化 INT8 → 浮点。
inline float DequantToF32(std::int8_t qnt, std::int32_t zp, float scale) {
    return (static_cast<float>(qnt) - static_cast<float>(zp)) * scale;
}

/// 坐标夹取到 [min, max]。
inline int ClampToInt(float value, int min, int max) {
    return value > static_cast<float>(min)
               ? (value < static_cast<float>(max) ? static_cast<int>(value) : max)
               : min;
}

/// 两个框的 IoU。
inline float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0,
                              float xmin1, float ymin1, float xmax1, float ymax1) {
    const float w = std::fmax(0.f, std::fmin(xmax0, xmax1) - std::fmax(xmin0, xmin1) + 1.f);
    const float h = std::fmax(0.f, std::fmin(ymax0, ymax1) - std::fmax(ymin0, ymin1) + 1.f);
    const float inter = w * h;
    const float uni = (xmax0 - xmin0 + 1.f) * (ymax0 - ymin0 + 1.f) +
                      (xmax1 - xmin1 + 1.f) * (ymax1 - ymin1 + 1.f) - inter;
    return uni <= 0.f ? 0.f : (inter / uni);
}

/// DFL 解码（YOLO11 等使用 DFL 的模型需要；YOLO26 dfl_len==1 不经过此函数）。
void ComputeDfl(const float* tensor, int dfl_len, float* box) {
    for (int b = 0; b < 4; ++b) {
        float exp_sum = 0.f;
        float acc_sum = 0.f;
        for (int i = 0; i < dfl_len; ++i) {
            const float exp_val = std::exp(tensor[i + b * dfl_len]);
            exp_sum += exp_val;
            acc_sum += exp_val * static_cast<float>(i);
        }
        box[b] = exp_sum > 0.f ? acc_sum / exp_sum : 0.f;
    }
}

}  // namespace

int DecodeBranch(const BranchOutput& branch, int stride, int num_classes,
                 float threshold, std::vector<float>& boxes_xywh,
                 std::vector<float>& obj_probs, std::vector<int>& class_ids) {
    if (branch.box.data == nullptr || branch.score.data == nullptr ||
        branch.box.grid_h <= 0 || branch.box.grid_w <= 0 || stride <= 0 ||
        num_classes <= 0) {
        return 0;
    }

    const int dfl_len = branch.box.channels / 4;
    if (dfl_len <= 0) {
        return 0;
    }
    const int grid_len = branch.box.grid_h * branch.box.grid_w;

    // 阈值换算为量化形式，避免逐网格反量化（与原型一致）
    const std::int8_t score_thres_i8 =
        QuantToAffine(threshold, branch.score.zp, branch.score.scale);
    const std::int8_t score_sum_thres_i8 =
        QuantToAffine(threshold, branch.score_sum.zp, branch.score_sum.scale);

    int valid_count = 0;
    for (int i = 0; i < branch.box.grid_h; ++i) {
        for (int j = 0; j < branch.box.grid_w; ++j) {
            int offset = i * branch.box.grid_w + j;

            // 利用 score_sum 快速过滤（该分支带 score_sum 时）
            if (branch.score_sum.data != nullptr &&
                branch.score_sum.data[offset] < score_sum_thres_i8) {
                continue;
            }

            // 找最高得分类别（score 张量 NCHW：类别通道步长为 grid_len）
            std::int8_t max_score = static_cast<std::int8_t>(-branch.score.zp);
            int max_class_id = -1;
            for (int c = 0; c < num_classes; ++c) {
                if (branch.score.data[offset] > score_thres_i8 &&
                    branch.score.data[offset] > max_score) {
                    max_score = branch.score.data[offset];
                    max_class_id = c;
                }
                offset += grid_len;
            }

            if (max_score <= score_thres_i8 || max_class_id < 0) {
                continue;
            }
            offset = i * branch.box.grid_w + j;

            // box 通道 = 4 × dfl_len（YOLO26 无 DFL 时 dfl_len==1，直接反量化）
            float box[4] = {0.f, 0.f, 0.f, 0.f};
            if (dfl_len > 1) {
                std::vector<float> before_dfl(static_cast<std::size_t>(dfl_len) * 4);
                for (int k = 0; k < dfl_len * 4; ++k) {
                    before_dfl[static_cast<std::size_t>(k)] =
                        DequantToF32(branch.box.data[offset], branch.box.zp,
                                     branch.box.scale);
                    offset += grid_len;
                }
                ComputeDfl(before_dfl.data(), dfl_len, box);
            } else {
                for (int k = 0; k < 4; ++k) {
                    box[k] = DequantToF32(branch.box.data[offset], branch.box.zp,
                                          branch.box.scale);
                    offset += grid_len;
                }
            }

            // 解码为模型坐标（左上角 + 宽高）
            const float x1 = (-box[0] + static_cast<float>(j) + 0.5f) * stride;
            const float y1 = (-box[1] + static_cast<float>(i) + 0.5f) * stride;
            const float x2 = (box[2] + static_cast<float>(j) + 0.5f) * stride;
            const float y2 = (box[3] + static_cast<float>(i) + 0.5f) * stride;

            boxes_xywh.push_back(x1);
            boxes_xywh.push_back(y1);
            boxes_xywh.push_back(x2 - x1);
            boxes_xywh.push_back(y2 - y1);
            obj_probs.push_back(DequantToF32(max_score, branch.score.zp,
                                             branch.score.scale));
            class_ids.push_back(max_class_id);
            ++valid_count;
        }
    }
    return valid_count;
}

void SortDescending(std::vector<float>& scores, std::vector<int>& indices) {
    const std::size_t n = scores.size();
    if (indices.empty()) {
        indices.resize(n);
        std::iota(indices.begin(), indices.end(), 0);
    }

    std::vector<std::pair<float, int>> paired;
    paired.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        paired.emplace_back(scores[i], indices[i]);
    }
    std::sort(paired.begin(), paired.end(),
              [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
                  return a.first > b.first;
              });
    for (std::size_t i = 0; i < n; ++i) {
        scores[i] = paired[i].first;
        indices[i] = paired[i].second;
    }
}

int Nms(int valid_count, const std::vector<float>& boxes_xywh,
        const std::vector<int>& class_ids, std::vector<int>& order,
        int filter_class_id, float threshold) {
    for (int i = 0; i < valid_count; ++i) {
        const int n = order[static_cast<std::size_t>(i)];
        if (n == -1 || class_ids[static_cast<std::size_t>(n)] != filter_class_id) {
            continue;
        }
        const float xmin0 = boxes_xywh[static_cast<std::size_t>(n) * 4 + 0];
        const float ymin0 = boxes_xywh[static_cast<std::size_t>(n) * 4 + 1];
        const float xmax0 = xmin0 + boxes_xywh[static_cast<std::size_t>(n) * 4 + 2];
        const float ymax0 = ymin0 + boxes_xywh[static_cast<std::size_t>(n) * 4 + 3];

        for (int j = i + 1; j < valid_count; ++j) {
            const int m = order[static_cast<std::size_t>(j)];
            if (m == -1 || class_ids[static_cast<std::size_t>(m)] != filter_class_id) {
                continue;
            }
            const float xmin1 = boxes_xywh[static_cast<std::size_t>(m) * 4 + 0];
            const float ymin1 = boxes_xywh[static_cast<std::size_t>(m) * 4 + 1];
            const float xmax1 = xmin1 + boxes_xywh[static_cast<std::size_t>(m) * 4 + 2];
            const float ymax1 = ymin1 + boxes_xywh[static_cast<std::size_t>(m) * 4 + 3];

            if (CalculateOverlap(xmin0, ymin0, xmax0, ymax0,
                                 xmin1, ymin1, xmax1, ymax1) > threshold) {
                order[static_cast<std::size_t>(j)] = -1;
            }
        }
    }
    return 0;
}

int PostProcessNormalizedXywh(const NormalizedXywhTensor& tensor,
                              int model_width, int model_height,
                              float conf_threshold, float nms_threshold,
                              int class_id, const LetterBox& letterbox,
                              std::vector<YoloDetection>* out) {
    if (tensor.data == nullptr || tensor.channels != 5 ||
        tensor.candidate_count <= 0 || tensor.scale <= 0.f ||
        model_width <= 0 || model_height <= 0 || letterbox.scale <= 0.f) {
        return 0;
    }

    std::vector<float> boxes_xywh;
    std::vector<float> scores;
    std::vector<int> class_ids;
    boxes_xywh.reserve(kMaxPostProcessResults * 4);
    scores.reserve(kMaxPostProcessResults);
    class_ids.reserve(kMaxPostProcessResults);

    const int candidates = tensor.candidate_count;
    for (int i = 0; i < candidates; ++i) {
        const float score = DequantToF32(tensor.data[4 * candidates + i],
                                         tensor.zp, tensor.scale);
        if (score < conf_threshold) {
            continue;
        }

        const float center_x = DequantToF32(tensor.data[i], tensor.zp,
                                            tensor.scale) * model_width;
        const float center_y = DequantToF32(tensor.data[candidates + i], tensor.zp,
                                            tensor.scale) * model_height;
        const float width = DequantToF32(tensor.data[2 * candidates + i], tensor.zp,
                                         tensor.scale) * model_width;
        const float height = DequantToF32(tensor.data[3 * candidates + i], tensor.zp,
                                          tensor.scale) * model_height;
        if (width <= 0.f || height <= 0.f) {
            continue;
        }

        boxes_xywh.push_back(center_x - width * 0.5f);
        boxes_xywh.push_back(center_y - height * 0.5f);
        boxes_xywh.push_back(width);
        boxes_xywh.push_back(height);
        scores.push_back(score);
        class_ids.push_back(class_id);
    }

    const int valid_count = static_cast<int>(scores.size());
    if (valid_count == 0) {
        return 0;
    }

    std::vector<int> order;
    SortDescending(scores, order);
    Nms(valid_count, boxes_xywh, class_ids, order, class_id, nms_threshold);

    int result_count = 0;
    for (int i = 0; i < valid_count && result_count < kMaxPostProcessResults; ++i) {
        const int candidate = order[static_cast<std::size_t>(i)];
        if (candidate < 0) {
            continue;
        }

        const float model_x1 = boxes_xywh[static_cast<std::size_t>(candidate) * 4] -
                               static_cast<float>(letterbox.x_pad);
        const float model_y1 = boxes_xywh[static_cast<std::size_t>(candidate) * 4 + 1] -
                               static_cast<float>(letterbox.y_pad);
        const float model_x2 = model_x1 +
                               boxes_xywh[static_cast<std::size_t>(candidate) * 4 + 2];
        const float model_y2 = model_y1 +
                               boxes_xywh[static_cast<std::size_t>(candidate) * 4 + 3];

        if (out != nullptr) {
            YoloDetection detection;
            detection.x1 = ClampToInt(model_x1, 0, model_width) / letterbox.scale;
            detection.y1 = ClampToInt(model_y1, 0, model_height) / letterbox.scale;
            detection.x2 = ClampToInt(model_x2, 0, model_width) / letterbox.scale;
            detection.y2 = ClampToInt(model_y2, 0, model_height) / letterbox.scale;
            detection.confidence = scores[static_cast<std::size_t>(i)];
            detection.class_id = class_id;
            if (detection.x2 > detection.x1 && detection.y2 > detection.y1) {
                out->push_back(detection);
                ++result_count;
            }
        } else {
            ++result_count;
        }
    }
    return result_count;
}

int PostProcess(const std::vector<BranchOutput>& branches, int model_size,
                float conf_threshold, float nms_threshold, int num_classes,
                const LetterBox& letterbox, std::vector<YoloDetection>* out) {
    if (branches.empty() || model_size <= 0 || num_classes <= 0) {
        return 0;
    }

    std::vector<float> boxes_xywh;
    std::vector<float> obj_probs;
    std::vector<int> class_ids;
    boxes_xywh.reserve(kMaxPostProcessResults * 4);
    obj_probs.reserve(kMaxPostProcessResults);
    class_ids.reserve(kMaxPostProcessResults);

    int valid_count = 0;
    for (const auto& branch : branches) {
        if (branch.box.data == nullptr || branch.score.data == nullptr) {
            continue;
        }
        const int stride = model_size / branch.box.grid_h;
        if (stride <= 0) {
            continue;
        }
        valid_count += DecodeBranch(branch, stride, num_classes, conf_threshold,
                                    boxes_xywh, obj_probs, class_ids);
    }

    if (valid_count <= 0) {
        return 0;
    }

    // 按置信度降序排序（同时重排候选索引）
    std::vector<int> order;
    SortDescending(obj_probs, order);

    // 逐类别 NMS：同类别框重叠超过阈值则抑制低分者
    const std::set<int> class_set(class_ids.begin(), class_ids.end());
    for (int c : class_set) {
        Nms(valid_count, boxes_xywh, class_ids, order, c, nms_threshold);
    }

    // 组装结果：letterbox 逆变换（去 pad + 还原 scale），输出裁剪图坐标
    int last_count = 0;
    for (int i = 0; i < valid_count; ++i) {
        if (order[static_cast<std::size_t>(i)] == -1 ||
            last_count >= kMaxPostProcessResults) {
            continue;
        }
        const int n = order[static_cast<std::size_t>(i)];

        float x1 = boxes_xywh[static_cast<std::size_t>(n) * 4 + 0] -
                   static_cast<float>(letterbox.x_pad);
        float y1 = boxes_xywh[static_cast<std::size_t>(n) * 4 + 1] -
                   static_cast<float>(letterbox.y_pad);
        const float x2 = x1 + boxes_xywh[static_cast<std::size_t>(n) * 4 + 2];
        const float y2 = y1 + boxes_xywh[static_cast<std::size_t>(n) * 4 + 3];

        if (out != nullptr) {
            YoloDetection det;
            det.x1 = static_cast<float>(ClampToInt(x1, 0, model_size)) / letterbox.scale;
            det.y1 = static_cast<float>(ClampToInt(y1, 0, model_size)) / letterbox.scale;
            det.x2 = static_cast<float>(ClampToInt(x2, 0, model_size)) / letterbox.scale;
            det.y2 = static_cast<float>(ClampToInt(y2, 0, model_size)) / letterbox.scale;
            det.confidence = obj_probs[static_cast<std::size_t>(i)];
            det.class_id = class_ids[static_cast<std::size_t>(n)];
            out->push_back(det);
        }
        ++last_count;
    }
    return last_count;
}

}  // namespace drone::perception
