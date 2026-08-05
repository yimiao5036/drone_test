#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <stdlib.h>
#include <set>
#include <iostream>

inline constexpr int OBJ_NAME_MAX_SIZE = 64;
inline constexpr int OBJ_NUMB_MAX_SIZE = 128;
inline constexpr int OBJ_CLASS_NUM = 80;
inline constexpr float NMS_THRESH = 0.45f;
inline constexpr float BOX_THRESH = 0.25f;

// letterbox 填充信息
typedef struct {
    int x_pad;
    int y_pad;
    float scale;
} letterbox_t;

// 检测框坐标
typedef struct {
    int left;
    int top;
    int right;
    int bottom;
} image_rect_t;

// 单个检测结果
typedef struct {
    image_rect_t box;
    float prop;       // 置信度
    int cls_id;       // 类别ID
} object_detect_result;

// 检测结果列表
typedef struct {
    int id;
    int count;
    object_detect_result results[OBJ_NUMB_MAX_SIZE];
} object_detect_result_list;

// 量化/反量化工具函数
inline int32_t __clip(float val, float min, float max) {
    return (int32_t)(val <= min ? min : (val >= max ? max : val));
}

inline int8_t qnt_f32_to_affine(float f32, int32_t zp, float scale) {
    float dst_val = (f32 / scale) + zp;
    return (int8_t)__clip(dst_val, -128.0f, 127.0f);
}

inline float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale) {
    return ((float)qnt - (float)zp) * scale;
}

// DFL 解码（YOLO11 等使用 DFL 的模型需要）
void compute_dfl(float* tensor, int dfl_len, float* box);

// 处理 INT8 量化输出（支持有/无 DFL）
int process_i8(int8_t* box_tensor, int32_t box_zp, float box_scale,
    int8_t* score_tensor, int32_t score_zp, float score_scale,
    int8_t* score_sum_tensor, int32_t score_sum_zp, float score_sum_scale,
    int grid_h, int grid_w, int stride, int dfl_len,
    std::vector<float>& boxes,
    std::vector<float>& objProbs,
    std::vector<int>& classId,
    float threshold);

// 按置信度降序排序（同时排序索引）
void quick_sort_indice_inverse(std::vector<float>& input, std::vector<int>& indices);

// 计算两个框的 IoU
inline float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0,
    float xmin1, float ymin1, float xmax1, float ymax1) {
    float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0f);
    float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0f);
    float i = w * h;
    float u = (xmax0 - xmin0 + 1.0f) * (ymax0 - ymin0 + 1.0f) +
              (xmax1 - xmin1 + 1.0f) * (ymax1 - ymin1 + 1.0f) - i;
    return u <= 0.f ? 0.f : (i / u);
}

// 非极大值抑制
int nms(int validCount, std::vector<float>& outputLocations, const std::vector<int>& classIds,
    std::vector<int>& order, int filterId, float threshold);

// 坐标裁剪到有效范围
inline int clamp(float val, int min, int max) {
    return val > min ? (val < max ? (int)val : max) : min;
}

#endif // _COMMON_H_
