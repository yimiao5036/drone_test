#include "rknn_model.h"
#include "rga_utils.h"
#include "common.h"

rknn_model::rknn_model(const std::string& model_path) {
    init_model(model_path);
    int ctx_index = 0;
    query_model_info(ctx_index);
    initialize_mems();
}

rknn_model::~rknn_model() {
    release_mems();
    release_model();
}

static void dump_tensor_attr(rknn_tensor_attr* attr) {
    char dims[128] = { 0 };
    for (uint32_t i = 0; i < attr->n_dims; ++i) {
        int idx = strlen(dims);
        sprintf(&dims[idx], "%d%s", attr->dims[i], (i == attr->n_dims - 1) ? "" : ", ");
    }
    printf("  index=%d, name=%s, n_dims=%d, dims=[%s], n_elems=%d, size=%d, w_stride=%d, size_with_stride=%d, "
        "fmt=%s, type=%s, qnt_type=%s, zp=%d, scale=%f\n",
        attr->index, attr->name, attr->n_dims, dims, attr->n_elems, attr->size, attr->w_stride, attr->size_with_stride,
        get_format_string(attr->fmt), get_type_string(attr->type), get_qnt_type_string(attr->qnt_type), attr->zp,
        attr->scale);
}

void rknn_model::query_model_info(int& ctx_index) {
    // 查询 SDK 版本
    rknn_sdk_version sdk_version;
    int ret = rknn_query(ctxs_[ctx_index], RKNN_QUERY_SDK_VERSION, &sdk_version, sizeof(sdk_version));
    if (ret < 0) {
        std::cerr << "rknn_query RKNN_QUERY_SDK_VERSION error ret=" << ret << std::endl;
        return;
    }
    std::cout << "SDK API Version: " << sdk_version.api_version << std::endl;
    std::cout << "Driver Version: " << sdk_version.drv_version << std::endl;

    // 查询自定义字符串
    rknn_custom_string custom_string;
    ret = rknn_query(ctxs_[ctx_index], RKNN_QUERY_CUSTOM_STRING, &custom_string, sizeof(custom_string));
    if (ret < 0) {
        std::cerr << "rknn_query RKNN_QUERY_CUSTOM_STRING error ret=" << ret << std::endl;
    } else {
        std::cout << "Custom String: " << custom_string.string << std::endl;
    }

    // 查询输入输出数量
    ret = rknn_query(ctxs_[ctx_index], RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (ret < 0) {
        std::cerr << "rknn_query RKNN_QUERY_IN_OUT_NUM error ret=" << ret << std::endl;
        return;
    }
    std::cout << "Model input num: " << io_num_.n_input << ", output num: " << io_num_.n_output << std::endl;

    // 查询输入 Tensor 属性
    input_attrs_.resize(io_num_.n_input);
    for (uint32_t i = 0; i < io_num_.n_input; ++i) {
        input_attrs_[i].index = i;
        ret = rknn_query(ctxs_[ctx_index], RKNN_QUERY_INPUT_ATTR, &input_attrs_[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            std::cerr << "rknn_query RKNN_QUERY_INPUT_ATTR error ret=" << ret << std::endl;
            return;
        }
    }

    // 查询硬件最优输入属性
    input_native_attrs_.resize(io_num_.n_input);
    for (uint32_t i = 0; i < io_num_.n_input; ++i) {
        input_native_attrs_[i].index = i;
        ret = rknn_query(ctxs_[ctx_index], RKNN_QUERY_NATIVE_INPUT_ATTR, &input_native_attrs_[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            std::cerr << "rknn_query RKNN_QUERY_NATIVE_INPUT_ATTR ret=" << ret << std::endl;
            return;
        }
        std::cout << "Input Tensor " << i << " info:" << std::endl;
        dump_tensor_attr(&(input_native_attrs_[i]));
    }

    // 查询输出 Tensor 属性
    output_attrs_.resize(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
        output_attrs_[i].index = i;
        ret = rknn_query(ctxs_[ctx_index], RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            std::cerr << "rknn_query RKNN_QUERY_OUTPUT_ATTR ret=" << ret << std::endl;
            return;
        }
    }

    // 查询硬件最优输出属性
    output_native_attrs_.resize(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
        output_native_attrs_[i].index = i;
        ret = rknn_query(ctxs_[ctx_index], RKNN_QUERY_NATIVE_OUTPUT_ATTR, &output_native_attrs_[i], sizeof(rknn_tensor_attr));
        if (ret < 0) {
            std::cerr << "rknn_query RKNN_QUERY_NATIVE_OUTPUT_ATTR ret=" << ret << std::endl;
            return;
        }
        std::cout << "Output Tensor " << i << " info:" << std::endl;
        dump_tensor_attr(&(output_native_attrs_[i]));
    }

    // 解析模型输入尺寸
    if (input_attrs_[0].fmt == RKNN_TENSOR_NCHW) {
        printf("model is NCHW input fmt\n");
        model_channel_ = input_attrs_[0].dims[1];
        model_height_ = input_attrs_[0].dims[2];
        model_width_ = input_attrs_[0].dims[3];
    } else {
        printf("model is NHWC input fmt\n");
        model_height_ = input_attrs_[0].dims[1];
        model_width_ = input_attrs_[0].dims[2];
        model_channel_ = input_attrs_[0].dims[3];
    }
    printf("model input height=%d, width=%d, channel=%d\n", model_height_, model_width_, model_channel_);

    num_outputs_ = io_num_.n_output;

    // 验证输出格式：YOLO26 非 end2end 导出应为 6 个输出（3 分支 × 2）
    // 每分支: box[1,4,H,W] + score[1,80,H,W]
    int dfl_len = output_attrs_[0].dims[1] / 4;
    std::cout << "[INFO] Output mode: TRADITIONAL (non-end2end)" << std::endl;
    std::cout << "[INFO] Output tensors: " << io_num_.n_output
              << ", dfl_len=" << dfl_len
              << (dfl_len == 1 ? " (YOLO26 no-DFL)" : " (DFL enabled)") << std::endl;
}

void rknn_model::init_model(const std::string& model_path) {
    FILE* fp = fopen(model_path.c_str(), "rb");
    if (fp == nullptr) {
        std::cerr << "Failed to open model file: " << model_path << std::endl;
        return;
    }
    fseek(fp, 0, SEEK_END);
    size_t model_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    double model_size_mb = static_cast<double>(model_size) / (1024 * 1024);
    std::cout << "Model size: " << model_size_mb << " MB" << std::endl;

    unsigned char* model_data = new unsigned char[model_size];
    if (model_data == nullptr) {
        std::cerr << "Failed to allocate memory for model data." << std::endl;
        fclose(fp);
        return;
    }

    size_t items_read = fread(model_data, 1, model_size, fp);
    if (items_read != model_size) {
        std::cerr << "Failed to read the entire model file." << std::endl;
        delete[] model_data;
        fclose(fp);
        return;
    }
    fclose(fp);

    // 初始化 RKNN 上下文（去掉 COLLECT_PERF_MASK 减少采样开销）
    int ret = rknn_init(&ctxs_[0], model_data, model_size,
        RKNN_FLAG_ENABLE_SRAM,
        nullptr);
    if (ret < 0) {
        std::cerr << "rknn_init error ret=" << ret << std::endl;
        delete[] model_data;
        return;
    }

    // 复制上下文到 3 个 NPU 核心
    for (int i = 1; i < 3; ++i) {
        ret = rknn_dup_context(&ctxs_[0], &ctxs_[i]);
        if (ret < 0) {
            std::cerr << "Failed to duplicate context to ctx" << i << ". ret=" << ret << std::endl;
            delete[] model_data;
            return;
        }
    }

    // 使用全部 NPU 核心（RK3588 有 3 个核心），单帧推理时由运行时自动调度
    rknn_set_core_mask(ctxs_[0], RKNN_NPU_CORE_ALL);
    rknn_set_core_mask(ctxs_[1], RKNN_NPU_CORE_ALL);
    rknn_set_core_mask(ctxs_[2], RKNN_NPU_CORE_ALL);

    delete[] model_data;
}

void rknn_model::release_model() {
    for (int i = 0; i < 3; ++i) {
        int ret = rknn_destroy(ctxs_[i]);
        if (ret < 0) {
            std::cerr << "rknn_destroy error ret=" << ret << std::endl;
        }
    }
}

void rknn_model::print_quantization_info() const {
    std::cout << "Quantization and Dequantization Information:" << std::endl;

    std::cout << "Input Tensors:" << std::endl;
    for (const auto& attr : input_attrs_) {
        std::cout << "  Name: " << attr.name << std::endl;
        if (attr.qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC) {
            std::cout << "    Scale: " << attr.scale << ", Zero Point: " << attr.zp << std::endl;
        }
    }

    std::cout << "Output Tensors:" << std::endl;
    for (const auto& attr : output_attrs_) {
        std::cout << "  Name: " << attr.name << std::endl;
        if (attr.qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC) {
            std::cout << "    Scale: " << attr.scale << ", Zero Point: " << attr.zp << std::endl;
        }
    }
}

rknn_tensor_mem* rknn_model::get_input_mem_ptr(int ctx_index, int mem_index) {
    if (ctx_index < 0 || ctx_index >= static_cast<int>(input_mems_.size())) {
        throw std::out_of_range("Index out of range for input_mems_ vector.");
    }
    if (mem_index < 0 || mem_index >= static_cast<int>(input_mems_[ctx_index].size())) {
        throw std::out_of_range("Index out of range for input_mems_[ctx_index] vector.");
    }
    return input_mems_[ctx_index][mem_index];
}

void rknn_model::initialize_mems() {
    int ctx_size = 3;
    input_mems_.resize(ctx_size);
    output_mems_.resize(ctx_size);

    for (size_t i = 0; i < ctx_size; ++i) {
        input_mems_[i].resize(io_num_.n_input);
        output_mems_[i].resize(io_num_.n_output);

        for (size_t j = 0; j < io_num_.n_input; ++j) {
            // 设置输入为 UINT8，NPU 内部完成 normalize + quantize
            input_native_attrs_[j].type = RKNN_TENSOR_UINT8;
            input_mems_[i][j] = rknn_create_mem(ctxs_[i], input_native_attrs_[j].size_with_stride);
            if (!input_mems_[i][j]) {
                throw std::runtime_error("rknn_create_mem failed for input");
            }
            int ret = rknn_set_io_mem(ctxs_[i], input_mems_[i][j], &input_native_attrs_[j]);
            if (ret < 0) {
                printf("input_mems_ rknn_set_io_mem fail! ret=%d\n", ret);
            }
        }

        for (size_t j = 0; j < io_num_.n_output; ++j) {
            output_mems_[i][j] = rknn_create_mem(ctxs_[i], output_native_attrs_[j].size_with_stride);
            if (!output_mems_[i][j]) {
                throw std::runtime_error("rknn_create_mem failed for output");
            }
            rknn_set_io_mem(ctxs_[i], output_mems_[i][j], &output_native_attrs_[j]);
        }
    }

    // 预分配后处理输出缓冲（避免每帧 malloc/free）
    output_buffers_.resize(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
        output_buffers_[i].resize(output_native_attrs_[i].n_elems);
    }
}

void rknn_model::release_mems() {
    int ctx_size = 3;
    for (size_t i = 0; i < ctx_size; ++i) {
        for (size_t j = 0; j < io_num_.n_input; ++j) {
            if (input_mems_[i][j] != nullptr && input_mems_[i][j]->virt_addr != nullptr) {
                rknn_destroy_mem(ctxs_[i], input_mems_[i][j]);
                input_mems_[i][j] = nullptr;
            }
        }
        for (size_t j = 0; j < io_num_.n_output; ++j) {
            if (output_mems_[i][j] != nullptr && output_mems_[i][j]->virt_addr != nullptr) {
                rknn_destroy_mem(ctxs_[i], output_mems_[i][j]);
                output_mems_[i][j] = nullptr;
            }
        }
    }
}

// NC1HWC2 → NCHW 格式转换
static int NC1HWC2_i8_to_NCHW_i8(const int8_t* src, int8_t* dst, int* dims, int channel, int h, int w) {
    int batch = dims[0];
    int C1 = dims[1];
    int C2 = dims[4];
    int hw_src = dims[2] * dims[3];
    int hw_dst = h * w;

    for (int i = 0; i < batch; i++) {
        const int8_t* src_b = src + i * C1 * hw_src * C2;
        int8_t* dst_b = dst + i * channel * hw_dst;
        for (int c = 0; c < channel; ++c) {
            int plane = c / C2;
            const int8_t* src_bc = src_b + plane * hw_src * C2;
            int offset = c % C2;
            for (int cur_hw = 0; cur_hw < hw_dst; ++cur_hw) {
                dst_b[c * hw_dst + cur_hw] = src_bc[C2 * cur_hw + offset];
            }
        }
    }
    return 0;
}

int rknn_model::run_inference(const cv::Mat& input_image, int ctx_index, object_detect_result_list* od_results) {
    using namespace std::chrono;

    const auto start = high_resolution_clock::now();
    if (input_image.empty()) {
        std::cerr << "Input image is empty!" << std::endl;
        return -1;
    }

    // 计算裁剪区域（居中裁剪为正方形，边长为 16 的倍数）
    const int short_side = (std::min(input_image.cols, input_image.rows) / 16) * 16;
    if (short_side <= 0) {
        std::cerr << "Input image too small for 16-alignment crop" << std::endl;
        return -1;
    }
    const int crop_x = (input_image.cols - short_side) / 2;
    const int crop_y = (input_image.rows - short_side) / 2;

    // 创建 ROI 视图（无拷贝），非连续内存时需要拷贝
    cv::Mat cropped = input_image(cv::Rect(crop_x, crop_y, short_side, short_side));
    if (!cropped.isContinuous()) {
        cropped = cropped.clone();
    }

    // RGA letterbox（BGR→RGB 色彩转换由 RGA 硬件完成）
    letterbox_t letter_box = {};
    constexpr uint8_t kFillColor = 114;
    int result = adaptive_letterbox(cropped, model_width_,
        static_cast<uint8_t*>(input_mems_[ctx_index][0]->virt_addr),
        &letter_box, kFillColor, INTER_LINEAR);
    if (result != 0) {
        std::cerr << "Failed to letterbox the image!" << std::endl;
        return -1;
    }

    // NPU 推理
    const auto inference_start = high_resolution_clock::now();
    result = rknn_run(ctxs_[ctx_index], nullptr);
    const auto inference_end = high_resolution_clock::now();
    if (result != 0) {
        std::cerr << "rknn_run fail! ret=" << result << std::endl;
        return -1;
    }

    // 后处理（使用预分配缓冲，避免每帧 malloc/free）
    memset(od_results, 0x00, sizeof(*od_results));

    rknn_output outputs[6] = {};  // YOLO26 最多 6 个输出
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
        const int channel = output_attrs_[i].dims[1];
        const int h = output_attrs_[i].n_dims > 2 ? output_attrs_[i].dims[2] : 1;
        const int w = output_attrs_[i].n_dims > 3 ? output_attrs_[i].dims[3] : 1;

        outputs[i].size = output_native_attrs_[i].n_elems * sizeof(int8_t);
        outputs[i].buf = output_buffers_[i].data();

        if (output_native_attrs_[i].fmt == RKNN_TENSOR_NC1HWC2) {
            NC1HWC2_i8_to_NCHW_i8(
                static_cast<int8_t*>(output_mems_[ctx_index][i]->virt_addr),
                static_cast<int8_t*>(outputs[i].buf),
                reinterpret_cast<int*>(output_native_attrs_[i].dims),
                channel, h, w);
        } else {
            memcpy(outputs[i].buf, output_mems_[ctx_index][i]->virt_addr, outputs[i].size);
        }
    }

    post_process(outputs, &letter_box, BOX_THRESH, NMS_THRESH, od_results);

    const auto end = high_resolution_clock::now();
    const double total_ms = duration<double, std::milli>(end - start).count();
    const double infer_ms = duration<double, std::milli>(inference_end - inference_start).count();
    printf("[Perf] Total: %.3f ms, Inference: %.3f ms\n", total_ms, infer_ms);

    return 0;
}

// 后处理：多分支输出解码 + NMS
// YOLO26 非 end2end 导出：6 个输出 (box+score) × 3 分支
// box 通道数 = 4（无 DFL，直接回归），score 通道数 = 80
int rknn_model::post_process(void* outputs, letterbox_t* letter_box,
    float conf_threshold, float nms_threshold, object_detect_result_list* od_results)
{
    auto* _outputs = static_cast<rknn_output*>(outputs);

    std::vector<float> filterBoxes;
    std::vector<float> objProbs;
    std::vector<int> classId;
    int validCount = 0;
    int stride = 0;
    int grid_h = 0;
    int grid_w = 0;

    // YOLO26 无 DFL，dfl_len 为 1；YOLO11 有 DFL，dfl_len > 1
    int dfl_len = output_attrs_[0].dims[1] / 4;
    int output_per_branch = io_num_.n_output / 3;

    for (int i = 0; i < 3; i++) {
        void* score_sum = nullptr;
        int32_t score_sum_zp = 0;
        float score_sum_scale = 1.0f;

        // 每分支 3 个输出时，第 3 个是 score_sum
        if (output_per_branch == 3) {
            score_sum = _outputs[i * output_per_branch + 2].buf;
            score_sum_zp = output_attrs_[i * output_per_branch + 2].zp;
            score_sum_scale = output_attrs_[i * output_per_branch + 2].scale;
        }

        int box_idx = i * output_per_branch;
        int score_idx = i * output_per_branch + 1;
        grid_h = output_attrs_[box_idx].dims[2];
        grid_w = output_attrs_[box_idx].dims[3];
        stride = model_height_ / grid_h;

        validCount += process_i8(
            static_cast<int8_t*>(_outputs[box_idx].buf), output_attrs_[box_idx].zp, output_attrs_[box_idx].scale,
            static_cast<int8_t*>(_outputs[score_idx].buf), output_attrs_[score_idx].zp, output_attrs_[score_idx].scale,
            static_cast<int8_t*>(score_sum), score_sum_zp, score_sum_scale,
            grid_h, grid_w, stride, dfl_len,
            filterBoxes, objProbs, classId, conf_threshold);
    }

    if (validCount <= 0) {
        return 0;
    }

    // 排序 + NMS
    std::vector<int> indexArray;
    quick_sort_indice_inverse(objProbs, indexArray);

    std::set<int> class_set(classId.begin(), classId.end());
    for (int c : class_set) {
        nms(validCount, filterBoxes, classId, indexArray, c, nms_threshold);
    }

    // 输出最终结果
    int last_count = 0;
    for (int i = 0; i < validCount; ++i) {
        if (indexArray[i] == -1 || last_count >= OBJ_NUMB_MAX_SIZE) {
            continue;
        }
        int n = indexArray[i];

        float x1 = filterBoxes[n * 4 + 0] - letter_box->x_pad;
        float y1 = filterBoxes[n * 4 + 1] - letter_box->y_pad;
        float x2 = x1 + filterBoxes[n * 4 + 2];
        float y2 = y1 + filterBoxes[n * 4 + 3];

        od_results->results[last_count].box.left = (int)(clamp(x1, 0, model_width_) / letter_box->scale);
        od_results->results[last_count].box.top = (int)(clamp(y1, 0, model_height_) / letter_box->scale);
        od_results->results[last_count].box.right = (int)(clamp(x2, 0, model_width_) / letter_box->scale);
        od_results->results[last_count].box.bottom = (int)(clamp(y2, 0, model_height_) / letter_box->scale);
        od_results->results[last_count].prop = objProbs[i];
        od_results->results[last_count].cls_id = classId[n];

        last_count++;
    }
    od_results->count = last_count;
    return 0;
}
