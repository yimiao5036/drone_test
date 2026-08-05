#include "common.h"

// DFL (Distribution Focal Loss) 解码
// YOLO26 移除了 DFL，当 dfl_len == 1 时不需要此函数
void compute_dfl(float* tensor, int dfl_len, float* box) {
    for (int b = 0; b < 4; b++) {
        float exp_sum = 0.0f;
        float acc_sum = 0.0f;
        for (int i = 0; i < dfl_len; i++) {
            float exp_val = exp(tensor[i + b * dfl_len]);
            exp_sum += exp_val;
            acc_sum += exp_val * i;
        }
        box[b] = acc_sum / exp_sum;
    }
}

// 处理 INT8 量化输出
// 当 dfl_len == 1 时（YOLO26），box 值直接可用，无需 DFL 解码
// 当 dfl_len > 1 时（YOLO11 等），需要 DFL 解码
int process_i8(int8_t* box_tensor, int32_t box_zp, float box_scale,
    int8_t* score_tensor, int32_t score_zp, float score_scale,
    int8_t* score_sum_tensor, int32_t score_sum_zp, float score_sum_scale,
    int grid_h, int grid_w, int stride, int dfl_len,
    std::vector<float>& boxes,
    std::vector<float>& objProbs,
    std::vector<int>& classId,
    float threshold)
{
    int validCount = 0;
    int grid_len = grid_h * grid_w;

    // 将阈值转换为量化形式
    int8_t score_thres_i8 = qnt_f32_to_affine(threshold, score_zp, score_scale);
    int8_t score_sum_thres_i8 = qnt_f32_to_affine(threshold, score_sum_zp, score_sum_scale);

    for (int i = 0; i < grid_h; i++) {
        for (int j = 0; j < grid_w; j++) {
            int offset = i * grid_w + j;
            int max_class_id = -1;

            // 利用 score_sum 快速过滤
            if ((score_sum_tensor != nullptr) && (score_sum_tensor[offset] < score_sum_thres_i8)) {
                continue;
            }

            // 找最高得分类别
            int8_t max_score = -score_zp;
            for (int c = 0; c < OBJ_CLASS_NUM; c++) {
                if ((score_tensor[offset] > score_thres_i8) && (score_tensor[offset] > max_score)) {
                    max_score = score_tensor[offset];
                    max_class_id = c;
                }
                offset += grid_len;
            }

            if (max_score > score_thres_i8) {
                offset = i * grid_w + j;

                float box[4];

                if (dfl_len > 1) {
                    // 有 DFL 的模型（YOLO11 等）：需要 DFL 解码
                    // 使用 std::vector 替代变长数组（VLA），保证标准 C++ 合规
                    std::vector<float> before_dfl(dfl_len * 4);
                    for (int k = 0; k < dfl_len * 4; k++) {
                        before_dfl[k] = deqnt_affine_to_f32(box_tensor[offset], box_zp, box_scale);
                        offset += grid_len;
                    }
                    compute_dfl(before_dfl.data(), dfl_len, box);
                } else {
                    // 无 DFL 的模型（YOLO26）：直接反量化得到 box 值
                    for (int k = 0; k < 4; k++) {
                        box[k] = deqnt_affine_to_f32(box_tensor[offset], box_zp, box_scale);
                        offset += grid_len;
                    }
                }

                // 计算边界框真实坐标
                float x1, y1, x2, y2, w, h;
                x1 = (-box[0] + j + 0.5f) * stride;
                y1 = (-box[1] + i + 0.5f) * stride;
                x2 = (box[2] + j + 0.5f) * stride;
                y2 = (box[3] + i + 0.5f) * stride;
                w = x2 - x1;
                h = y2 - y1;

                boxes.push_back(x1);
                boxes.push_back(y1);
                boxes.push_back(w);
                boxes.push_back(h);
                objProbs.push_back(deqnt_affine_to_f32(max_score, score_zp, score_scale));
                classId.push_back(max_class_id);
                validCount++;
            }
        }
    }
    return validCount;
}

// 按置信度降序排序
void quick_sort_indice_inverse(std::vector<float>& input, std::vector<int>& indices) {
    size_t n = input.size();
    if (indices.empty()) {
        indices.resize(n);
        std::iota(indices.begin(), indices.end(), 0);
    }

    std::vector<std::pair<float, int>> paired(n);
    for (size_t i = 0; i < n; ++i) {
        paired[i] = std::make_pair(input[i], indices[i]);
    }
    std::sort(paired.begin(), paired.end(),
        [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
            return a.first > b.first;
        });
    for (size_t i = 0; i < n; ++i) {
        input[i] = paired[i].first;
        indices[i] = paired[i].second;
    }
}

// 非极大值抑制
int nms(int validCount, std::vector<float>& outputLocations, const std::vector<int>& classIds,
    std::vector<int>& order, int filterId, float threshold)
{
    for (int i = 0; i < validCount; ++i) {
        int n = order[i];
        if (n == -1 || classIds[n] != filterId) {
            continue;
        }
        float xmin0 = outputLocations[n * 4 + 0];
        float ymin0 = outputLocations[n * 4 + 1];
        float xmax0 = xmin0 + outputLocations[n * 4 + 2];
        float ymax0 = ymin0 + outputLocations[n * 4 + 3];

        for (int j = i + 1; j < validCount; ++j) {
            int m = order[j];
            if (m == -1 || classIds[m] != filterId) {
                continue;
            }
            float xmin1 = outputLocations[m * 4 + 0];
            float ymin1 = outputLocations[m * 4 + 1];
            float xmax1 = xmin1 + outputLocations[m * 4 + 2];
            float ymax1 = ymin1 + outputLocations[m * 4 + 3];

            float iou = CalculateOverlap(xmin0, ymin0, xmax0, ymax0, xmin1, ymin1, xmax1, ymax1);
            if (iou > threshold) {
                order[j] = -1;
            }
        }
    }
    return 0;
}
