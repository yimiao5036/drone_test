/**
 * @file yolo_postprocess.h
 * @brief YOLO 检测后处理纯函数模块（量化解码 + NMS + letterbox 逆变换）
 *
 * 属于 drone/perception 模块，是 YoloDetector 的组成部分。
 *
 * 职责边界：
 * - 本模块只做"推理输出的数值后处理"：把 NPU 输出的多分支 INT8 量化张量
 *   解码为检测候选，按类别执行 NMS，再经 letterbox 逆变换还原到裁剪图坐标。
 * - 不涉及推理本身（RKNN）、图像预处理（RGA）、内存池（视频帧），
 *   因此不依赖任何硬件库，可在开发机独立编译与单元测试。
 * - 输出坐标系为"裁剪图坐标"（居中裁剪正方形子图，见 RknnDetectionBackend），
 *   原图偏移由调用方（后端）叠加。
 *
 * 与原型 videoPart/yolo26-rknn/src/common.cpp 的对应关系：
 * - process_i8               → DecodeBranch（单分支量化解码 + 阈值过滤）
 * - quick_sort_indice_inverse→ SortDescending（按置信度降序）
 * - nms / CalculateOverlap   → Nms（按类别非极大值抑制）
 * - compute_dfl              → 内部静态函数（YOLO26 无 DFL，保留兼容 YOLO11）
 * - post_process             → PostProcess（组合流程 + letterbox 逆变换）
 *
 * 设计说明：
 * - 张量用 QuantTensor 描述（数据指针 + 反量化参数 + 网格尺寸），与 RKNN
 *   属性结构解耦，便于构造任意量化张量做单元测试。
 * - 支持带/不带 score_sum 快速过滤分支（outputs_per_branch 为 3 或 2）。
 */
#pragma once

#include <cstdint>
#include <vector>

namespace drone::perception {

/// 单分支后处理输出上限（对应原型 OBJ_NUMB_MAX_SIZE）。
inline constexpr int kMaxPostProcessResults = 128;

/// 后处理输出的单个检测（裁剪图坐标系，像素）。
struct YoloDetection {
    int class_id = 0;
    float confidence = 0.f;
    float x1 = 0.f;  ///< 裁剪图坐标左上角 x
    float y1 = 0.f;  ///< 裁剪图坐标左上角 y
    float x2 = 0.f;  ///< 裁剪图坐标右下角 x
    float y2 = 0.f;  ///< 裁剪图坐标右下角 y
};

/// letterbox 填充信息：预处理时记录，后处理逆变换使用。
struct LetterBox {
    int x_pad = 0;     ///< 左侧填充像素数
    int y_pad = 0;     ///< 顶部填充像素数
    float scale = 1.f; ///< 缩放比例（源→模型输入）
};

/// 单张量化张量描述（NCHW 布局，INT8 仿射量化）。
struct QuantTensor {
    const std::int8_t* data = nullptr;  ///< 量化数据（连续 NCHW）
    std::int32_t zp = 0;                ///< 反量化零点
    float scale = 1.f;                  ///< 反量化比例（(q - zp) * scale）
    int channels = 0;                   ///< 通道数（NCHW 的 C）
    int grid_h = 0;                     ///< 特征图高
    int grid_w = 0;                     ///< 特征图宽
};

/// 单分支输出三元组：box + score（+ 可选 score_sum 快速过滤）。
struct BranchOutput {
    QuantTensor box;        ///< [1, 4×dfl_len, H, W]
    QuantTensor score;      ///< [1, num_classes, H, W]
    QuantTensor score_sum;  ///< [1, 1, H, W]；data == nullptr 表示无此张量
};

/// 解码单分支：遍历网格、量化阈值过滤，输出候选（模型坐标系 xywh，未去 pad）。
///
/// @param branch 单分支输出（box/score/score_sum）
/// @param stride 该分支下采样步长 = model_size / grid_h
/// @param num_classes 类别数（score 张量通道数）
/// @param threshold 置信度阈值（浮点，内部换算为量化阈值）
/// @param boxes_xywh 输出候选框（模型坐标，x1/y1/w/h 依次连续 4 个）
/// @param obj_probs 输出候选置信度（与 boxes_xywh、class_ids 对齐）
/// @param class_ids 输出候选类别
/// @return 本分支新增候选数量
int DecodeBranch(const BranchOutput& branch, int stride, int num_classes,
                 float threshold, std::vector<float>& boxes_xywh,
                 std::vector<float>& obj_probs, std::vector<int>& class_ids);

/// 按置信度降序排序（同时重排 indices，与 scores 一一对应）。
/// indices 为空时初始化为 0..n-1。
void SortDescending(std::vector<float>& scores, std::vector<int>& indices);

/// 按指定类别执行 NMS：与 filter_class_id 同类的候选两两算 IoU，
/// 重叠超过 threshold 的低分候选在 order 中置 -1 标记抑制。
int Nms(int valid_count, const std::vector<float>& boxes_xywh,
        const std::vector<int>& class_ids, std::vector<int>& order,
        int filter_class_id, float threshold);

/// 完整后处理：多分支解码 → 排序 → 按类 NMS → letterbox 逆变换。
///
/// @param branches 全部输出分支（YOLO26 通常 3 个：P3/P4/P5）
/// @param model_size 模型输入边长（像素，正方形）
/// @param conf_threshold 置信度阈值
/// @param nms_threshold NMS IoU 阈值
/// @param num_classes 类别数
/// @param letterbox 预处理填充信息（x_pad/y_pad/scale）
/// @param out 输出检测列表（裁剪图坐标系）；为空指针时仅返回数量
/// @return 输出检测数量（<= kMaxPostProcessResults）
int PostProcess(const std::vector<BranchOutput>& branches, int model_size,
                float conf_threshold, float nms_threshold, int num_classes,
                const LetterBox& letterbox, std::vector<YoloDetection>* out);

}  // namespace drone::perception
