#ifndef RKNN_MODEL_H
#define RKNN_MODEL_H

#include <rknn_api.h>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <iomanip>
#include "common.h"

class rknn_model {
public:
    rknn_model(const std::string& model_path);
    ~rknn_model();

    // 查询模型信息
    void query_model_info(int& ctx_index);

    // 获取输入输出属性
    const std::vector<rknn_tensor_attr>& get_input_attrs() const { return input_attrs_; }
    const std::vector<rknn_tensor_attr>& get_output_attrs() const { return output_attrs_; }
    rknn_context& get_context(int index) { return ctxs_[index]; }

    // 打印量化信息
    void print_quantization_info() const;

    // 封装推理过程（input_image 为 BGR 格式，内部不修改输入图）
    [[nodiscard]] int run_inference(const cv::Mat& input_image, int ctx_index, object_detect_result_list* od_results);

    // 获取输入内存指针（用于外部直接写入）
    rknn_tensor_mem* get_input_mem_ptr(int ctx_index, int mem_index);

private:
    rknn_context ctxs_[3];

    std::vector<rknn_tensor_attr> input_attrs_;
    std::vector<rknn_tensor_attr> input_native_attrs_;
    std::vector<rknn_tensor_attr> output_attrs_;
    std::vector<rknn_tensor_attr> output_native_attrs_;
    rknn_input_output_num io_num_;
    int model_height_;
    int model_width_;
    int model_channel_;

    std::vector<std::vector<rknn_tensor_mem*>> input_mems_;
    std::vector<std::vector<rknn_tensor_mem*>> output_mems_;
    std::vector<std::vector<int8_t>> output_buffers_;  // 预分配的后处理输出缓冲
    int num_outputs_;

    void init_model(const std::string& model_path);
    void release_model();
    void initialize_mems();
    void release_mems();

    // 后处理：多分支输出 → 解码 + NMS
    // YOLO26 无 DFL，box 通道数为 4（直接回归），无需 DFL 解码
    int post_process(void* outputs, letterbox_t* letter_box,
        float conf_threshold, float nms_threshold, object_detect_result_list* od_results);
};

#endif // RKNN_MODEL_H
