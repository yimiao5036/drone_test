/**
 * @file rknn_detection_backend.cpp
 * @brief RKNN 推理后端实现（香橙派 RK3588，仅 DRONE_HAVE_RKNN 编译）
 *
 * 整合原型 videoPart/yolo26-rknn 的 rknn_model.cpp / rga_utils.cpp /
 * yolo26_detector.cpp 三部分：
 * - RGA 预处理：完整 NV12 解码帧 → 等比例缩放 → RGB letterbox 写入模型输入内存
 * - RKNN：3 核上下文（rknn_dup_context + RKNN_NPU_CORE_ALL），
 *   输入输出零拷贝内存（rknn_create_mem / rknn_set_io_mem）
 * - 后处理：NC1HWC2→NCHW 转换（预分配缓冲）→ `[1,5,N]` 归一化
 *   xywh 解码 + NMS + letterbox 逆变换，直接得到原图坐标
 *
 * 与正式工程的解耦点：
 * - 输入不再使用 cv::Mat（原型依赖 OpenCV），直接读 NV12 FrameHandle；
 *   因此本文件不依赖 OpenCV。
 * - 后处理复用 yolo_postprocess 纯函数模块，不再重复实现。
 */
#include "perception/rknn_detection_backend.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <rknn_api.h>
#include <rga/im2d.hpp>
#include <rga/rga.h>

#include "perception/yolo_postprocess.h"

#include <spdlog/spdlog.h>

namespace drone::perception {

namespace {

/// 异常日志节流：第 1 次与每满 100 次才打印，避免高频异常刷屏。
bool ShouldLogThrottled(std::uint64_t count) {
    return count == 1 || count % 100 == 0;
}

/// 读取整个模型文件到内存。
bool ReadModelFile(const std::string& path, std::vector<uint8_t>* data) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return false;
    }
    const std::streamsize size = file.tellg();
    if (size <= 0) {
        return false;
    }
    data->resize(static_cast<std::size_t>(size));
    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char*>(data->data()), size);
    return file.good() || file.eof();
}

/// 数值夹取。
inline float ClampF(float value, float min, float max) {
    return value < min ? min : (value > max ? max : value);
}

/// NC1HWC2 → NCHW 格式转换（RKNN 硬件最优输出布局 → 后处理期望布局）。
/// src/dst 布局见原型 NC1HWC2_i8_to_NCHW_i8，channel 为 NCHW 的 C。
void ConvertNc1hwc2ToNchw(const int8_t* src, int8_t* dst,
                          const rknn_tensor_attr& native_attr, int channel,
                          int h, int w) {
    const int batch = static_cast<int>(native_attr.dims[0]);
    const int C1 = static_cast<int>(native_attr.dims[1]);
    const int C2 = static_cast<int>(native_attr.dims[4]);
    const int hw_src = static_cast<int>(native_attr.dims[2]) *
                       static_cast<int>(native_attr.dims[3]);
    const int hw_dst = h * w;

    for (int i = 0; i < batch; ++i) {
        const int8_t* src_b = src + i * C1 * hw_src * C2;
        int8_t* dst_b = dst + i * channel * hw_dst;
        for (int c = 0; c < channel; ++c) {
            const int plane = c / C2;
            const int8_t* src_bc = src_b + plane * hw_src * C2;
            const int offset = c % C2;
            for (int cur_hw = 0; cur_hw < hw_dst; ++cur_hw) {
                dst_b[c * hw_dst + cur_hw] = src_bc[C2 * cur_hw + offset];
            }
        }
    }
}

/// 将 RKNN 张量属性转换为后处理 QuantTensor（数据指针由调用方指定）。
QuantTensor MakeTensor(const int8_t* data, const rknn_tensor_attr& attr) {
    QuantTensor tensor;
    tensor.data = data;
    tensor.zp = attr.zp;
    tensor.scale = attr.scale;
    tensor.channels = static_cast<int>(attr.dims[1]);
    tensor.grid_h = attr.n_dims > 2 ? static_cast<int>(attr.dims[2]) : 1;
    tensor.grid_w = attr.n_dims > 3 ? static_cast<int>(attr.dims[3]) : 1;
    return tensor;
}

}  // namespace

/// RknnDetectionBackend 实现细节（PIMPL）：RKNN 上下文、RGA 缓冲与推理流程。
struct RknnDetectionBackend::Impl {
    explicit Impl(std::string model_path, float conf_threshold, float nms_threshold)
        : model_path(std::move(model_path)),
          conf_threshold(conf_threshold),
          nms_threshold(nms_threshold) {
        if (conf_threshold < 0.f || conf_threshold > 1.f ||
            nms_threshold < 0.f || nms_threshold > 1.f) {
            throw std::invalid_argument("RKNN 后端阈值必须在 [0,1]");
        }
    }

    ~Impl() {
        ReleaseAll();
    }

    std::string model_path;
    float conf_threshold = 0.25f;
    float nms_threshold = 0.45f;
    bool loaded = false;
    std::atomic<uint64_t> error_count{0};

    // RKNN 上下文（3 个 NPU 核）
    rknn_context ctxs_[3] = {0, 0, 0};
    bool ctx_created_[3] = {false, false, false};

    // 模型张量属性
    std::vector<rknn_tensor_attr> input_attrs_;
    std::vector<rknn_tensor_attr> input_native_attrs_;
    std::vector<rknn_tensor_attr> output_attrs_;
    std::vector<rknn_tensor_attr> output_native_attrs_;
    rknn_input_output_num io_num_{};
    int model_width_ = 0;    ///< 模型输入宽（像素）
    int model_height_ = 0;   ///< 模型输入高（像素）
    int model_channel_ = 0;  ///< 模型输入通道数
    enum class OutputMode {
        kMultiBranch,
        kNormalizedXywh,
    };
    OutputMode output_mode_ = OutputMode::kMultiBranch;
    int num_classes_ = 0;    ///< 多分支模型类别数；单输出模型固定为 1
    int outputs_per_branch_ = 0;  ///< 每分支输出数（2 或 3，含 score_sum）
    int single_output_candidates_ = 0;  ///< `[1,5,N]` 的 N

    // 输入输出内存（零拷贝）
    std::vector<std::vector<rknn_tensor_mem*>> input_mems_;
    std::vector<std::vector<rknn_tensor_mem*>> output_mems_;
    std::vector<std::vector<int8_t>> output_buffers_;  ///< 后处理输出缓冲（预分配）

    // letterbox 缩放临时 RGB 缓冲（懒分配复用，避免每帧 malloc/free）
    std::vector<uint8_t> resized_rgb_buffer_;
    std::string last_preprocess_error_;  ///< 最近一次 RGA 错误，仅检测线程访问

    /// 查询模型信息并识别单输出 `[1,5,N]` 或旧版多分支结构。
    bool QueryModelInfo() {
        rknn_sdk_version sdk_version{};
        int ret = rknn_query(ctxs_[0], RKNN_QUERY_SDK_VERSION, &sdk_version,
                             sizeof(sdk_version));
        if (ret < 0) {
            SPDLOG_ERROR("RKNN 查询 SDK 版本失败: ret={}", ret);
            return false;
        }
        SPDLOG_INFO("RKNN SDK API={} 驱动={}", sdk_version.api_version,
                    sdk_version.drv_version);

        ret = rknn_query(ctxs_[0], RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
        if (ret < 0) {
            SPDLOG_ERROR("RKNN 查询输入输出数量失败: ret={}", ret);
            return false;
        }

        input_attrs_.resize(io_num_.n_input);
        for (uint32_t i = 0; i < io_num_.n_input; ++i) {
            input_attrs_[i].index = i;
            ret = rknn_query(ctxs_[0], RKNN_QUERY_INPUT_ATTR, &input_attrs_[i],
                             sizeof(rknn_tensor_attr));
            if (ret != RKNN_SUCC) {
                SPDLOG_ERROR("RKNN 查询输入属性失败: ret={}", ret);
                return false;
            }
        }
        input_native_attrs_.resize(io_num_.n_input);
        for (uint32_t i = 0; i < io_num_.n_input; ++i) {
            input_native_attrs_[i].index = i;
            ret = rknn_query(ctxs_[0], RKNN_QUERY_NATIVE_INPUT_ATTR,
                             &input_native_attrs_[i], sizeof(rknn_tensor_attr));
            if (ret != RKNN_SUCC) {
                SPDLOG_ERROR("RKNN 查询硬件最优输入属性失败: ret={}", ret);
                return false;
            }
        }

        output_attrs_.resize(io_num_.n_output);
        output_native_attrs_.resize(io_num_.n_output);
        for (uint32_t i = 0; i < io_num_.n_output; ++i) {
            output_attrs_[i].index = i;
            ret = rknn_query(ctxs_[0], RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i],
                             sizeof(rknn_tensor_attr));
            if (ret != RKNN_SUCC) {
                SPDLOG_ERROR("RKNN 查询输出属性失败: ret={}", ret);
                return false;
            }
            output_native_attrs_[i].index = i;
            ret = rknn_query(ctxs_[0], RKNN_QUERY_NATIVE_OUTPUT_ATTR,
                             &output_native_attrs_[i], sizeof(rknn_tensor_attr));
            if (ret != RKNN_SUCC) {
                SPDLOG_ERROR("RKNN 查询硬件最优输出属性失败: ret={}", ret);
                return false;
            }
        }

        // 打印全部输出张量形状（一次性，用于核对模型导出格式：
        // 非 end2end 应为 6 个 [1,4,H,W]/[1,C,H,W]；end2end 通常为 [1,N,6/7]）。
        for (uint32_t i = 0; i < io_num_.n_output; ++i) {
            std::string dims_str;
            for (uint32_t d = 0; d < output_attrs_[i].n_dims; ++d) {
                if (d > 0) {
                    dims_str += "x";
                }
                dims_str += std::to_string(output_attrs_[i].dims[d]);
            }
            SPDLOG_INFO("RKNN 输出[{}]: 形状={} 布局={}", i, dims_str,
                        static_cast<int>(output_attrs_[i].fmt));
        }

        // 解析模型输入尺寸（NCHW 或 NHWC）
        if (input_attrs_[0].fmt == RKNN_TENSOR_NCHW) {
            model_channel_ = static_cast<int>(input_attrs_[0].dims[1]);
            model_height_ = static_cast<int>(input_attrs_[0].dims[2]);
            model_width_ = static_cast<int>(input_attrs_[0].dims[3]);
        } else {
            model_height_ = static_cast<int>(input_attrs_[0].dims[1]);
            model_width_ = static_cast<int>(input_attrs_[0].dims[2]);
            model_channel_ = static_cast<int>(input_attrs_[0].dims[3]);
        }

        // 新模型：单输出 `[1,5,N]`，通道依次为归一化 xywh + confidence。
        if (io_num_.n_output == 1 && output_attrs_[0].n_dims == 3 &&
            output_attrs_[0].dims[1] == 5) {
            output_mode_ = OutputMode::kNormalizedXywh;
            num_classes_ = 1;
            single_output_candidates_ = static_cast<int>(output_attrs_[0].dims[2]);
            if (output_attrs_[0].type != RKNN_TENSOR_INT8) {
                SPDLOG_ERROR("RKNN 单输出张量类型不支持: type={}，当前仅支持 INT8 零拷贝输出",
                             static_cast<int>(output_attrs_[0].type));
                return false;
            }
            SPDLOG_INFO("RKNN 模型: 输入={}x{}x{}，单输出=[1,5,{}]，坐标=归一化xywh",
                        model_width_, model_height_, model_channel_,
                        single_output_candidates_);
            return model_width_ > 0 && model_height_ > 0 &&
                   single_output_candidates_ > 0;
        }

        // 兼容旧模型：3 分支 × (box + score [+ score_sum])。
        output_mode_ = OutputMode::kMultiBranch;
        if (io_num_.n_output >= 2) {
            num_classes_ = static_cast<int>(output_attrs_[1].dims[1]);
        }
        outputs_per_branch_ = io_num_.n_output / 3;
        const int dfl_len = static_cast<int>(output_attrs_[0].dims[1]) / 4;

        SPDLOG_INFO("RKNN 模型: 输入={}x{}x{} 输出={} 类别数={} 每分支输出={} dfl_len={}",
                    model_width_, model_height_, model_channel_, io_num_.n_output,
                    num_classes_, outputs_per_branch_, dfl_len);
        return model_width_ > 0 && model_height_ > 0 && num_classes_ > 0 &&
               (outputs_per_branch_ == 2 || outputs_per_branch_ == 3);
    }

    /// 分配输入输出内存并绑定。
    bool InitializeMems() {
        const int ctx_count = 3;
        input_mems_.assign(ctx_count, {});
        output_mems_.assign(ctx_count, {});

        for (int i = 0; i < ctx_count; ++i) {
            input_mems_[i].resize(io_num_.n_input);
            output_mems_[i].resize(io_num_.n_output);

            for (uint32_t j = 0; j < io_num_.n_input; ++j) {
                // 输入设为 UINT8，NPU 内部完成 normalize + quantize
                input_native_attrs_[j].type = RKNN_TENSOR_UINT8;
                input_mems_[i][j] =
                    rknn_create_mem(ctxs_[i], input_native_attrs_[j].size_with_stride);
                if (input_mems_[i][j] == nullptr) {
                    SPDLOG_ERROR("RKNN 分配输入内存失败: ctx={} input={}", i, j);
                    return false;
                }
                const int ret = rknn_set_io_mem(ctxs_[i], input_mems_[i][j],
                                                &input_native_attrs_[j]);
                if (ret < 0) {
                    SPDLOG_ERROR("RKNN 绑定输入内存失败: ret={}", ret);
                    return false;
                }
            }

            for (uint32_t j = 0; j < io_num_.n_output; ++j) {
                output_mems_[i][j] =
                    rknn_create_mem(ctxs_[i], output_native_attrs_[j].size_with_stride);
                if (output_mems_[i][j] == nullptr) {
                    SPDLOG_ERROR("RKNN 分配输出内存失败: ctx={} output={}", i, j);
                    return false;
                }
                const int ret = rknn_set_io_mem(ctxs_[i], output_mems_[i][j],
                                                &output_native_attrs_[j]);
                if (ret < 0) {
                    SPDLOG_ERROR("RKNN 绑定输出内存失败: ret={}", ret);
                    return false;
                }
            }
        }

        // 预分配后处理输出缓冲（避免每帧 malloc/free）
        output_buffers_.resize(io_num_.n_output);
        for (uint32_t i = 0; i < io_num_.n_output; ++i) {
            output_buffers_[i].resize(output_native_attrs_[i].n_elems);
        }
        return true;
    }

    /// 释放全部 RKNN 资源（幂等）。
    void ReleaseAll() noexcept {
        const int ctx_count = 3;
        for (int i = 0; i < ctx_count; ++i) {
            if (static_cast<std::size_t>(i) < input_mems_.size()) {
                for (auto* mem : input_mems_[i]) {
                    if (mem != nullptr) {
                        rknn_destroy_mem(ctxs_[i], mem);
                    }
                }
            }
            if (static_cast<std::size_t>(i) < output_mems_.size()) {
                for (auto* mem : output_mems_[i]) {
                    if (mem != nullptr) {
                        rknn_destroy_mem(ctxs_[i], mem);
                    }
                }
            }
            if (ctx_created_[i]) {
                rknn_destroy(ctxs_[i]);
                ctx_created_[i] = false;
            }
        }
        input_mems_.clear();
        output_mems_.clear();
        output_buffers_.clear();
        loaded = false;
    }

    /// NV12 → RGB letterbox 写入模型输入内存（RGA 硬件加速）。
    /// @return 0 成功；-1 参数非法或 RGA 失败
    int Nv12LetterboxToRgb(const uint8_t* src, int src_w, int src_h, int src_wstride,
                           int target_size, uint8_t* dst_rgb, LetterBox* letterbox,
                           uint8_t fill_color = 114) {
        last_preprocess_error_.clear();
        if (src == nullptr || dst_rgb == nullptr || target_size <= 0 ||
            letterbox == nullptr) {
            last_preprocess_error_ = "输入指针或尺寸非法";
            return -1;
        }
        const int wstride = src_wstride > 0 ? src_wstride : src_w;
        // im2d.hpp 六参数重载顺序是 width/height/format/wstride/hstride。
        // 旧代码把 format 放在最后，导致 RGA 将 height 当成 wstride，报
        // "wstride=720 < width=1280"。
        rga_buffer_t src_buf = wrapbuffer_virtualaddr(
            const_cast<uint8_t*>(src), src_w, src_h, RK_FORMAT_YCbCr_420_SP,
            wstride, src_h);

        // 源已是目标尺寸：RGA 完成 NV12→RGB 色彩转换
        if (src_w == target_size && src_h == target_size) {
            rga_buffer_t dst_buf = wrapbuffer_virtualaddr(
                dst_rgb, target_size, target_size, RK_FORMAT_RGB_888);
            const IM_STATUS status =
                imcvtcolor(src_buf, dst_buf, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888);
            if (status != IM_STATUS_SUCCESS) {
                last_preprocess_error_ = imStrError(status);
                return -1;
            }
            letterbox->scale = 1.f;
            letterbox->x_pad = 0;
            letterbox->y_pad = 0;
            return 0;
        }

        // 与通过验证的 Python 参考实现一致：round 后等比缩放，再居中填充 114。
        const float scale = std::min(static_cast<float>(target_size) / src_w,
                                     static_cast<float>(target_size) / src_h);
        const int new_width = static_cast<int>(std::lround(src_w * scale));
        const int new_height = static_cast<int>(std::lround(src_h * scale));
        const int pad_left = (target_size - new_width) / 2;
        const int pad_top = (target_size - new_height) / 2;

        // RGA 输出先写入连续的缩放 RGB 缓冲，再逐行放入带目标 stride 的
        // letterbox 画布；不能把画布内部地址包装成紧凑 new_width stride，
        // 否则第二行起会写错位置。
        resized_rgb_buffer_.resize(static_cast<std::size_t>(new_width) *
                                   new_height * 3);
        rga_buffer_t resized_buf = wrapbuffer_virtualaddr(
            resized_rgb_buffer_.data(), new_width, new_height, RK_FORMAT_RGB_888);
        const IM_STATUS status = imresize(src_buf, resized_buf, scale, scale,
                                          INTER_LINEAR);
        if (status != IM_STATUS_SUCCESS) {
            last_preprocess_error_ = imStrError(status);
            return -1;
        }

        std::memset(dst_rgb, fill_color,
                    static_cast<std::size_t>(target_size) * target_size * 3);
        const std::size_t src_row_bytes = static_cast<std::size_t>(new_width) * 3;
        const std::size_t dst_row_bytes = static_cast<std::size_t>(target_size) * 3;
        for (int row = 0; row < new_height; ++row) {
            std::memcpy(dst_rgb + (static_cast<std::size_t>(pad_top + row) *
                                  dst_row_bytes + static_cast<std::size_t>(pad_left) * 3),
                        resized_rgb_buffer_.data() + static_cast<std::size_t>(row) *
                                                         src_row_bytes,
                        src_row_bytes);
        }
        letterbox->scale = scale;
        letterbox->x_pad = pad_left;
        letterbox->y_pad = pad_top;
        last_preprocess_error_.clear();
        return 0;
    }

    /// 对一帧执行完整推理流程。
    std::vector<BackendDetection> DetectFrame(const video::FrameHandle& frame) {
        std::vector<BackendDetection> out;
        if (!loaded) {
            return out;
        }
        const auto& info = frame.Info();
        if (info.format != video::PixelFormat::kYuv420SpNv12 ||
            frame.Data() == nullptr) {
            const uint64_t errors = error_count.fetch_add(1) + 1;
            if (ShouldLogThrottled(errors)) {
                SPDLOG_WARN("RKNN 后端收到非 NV12 或空帧，跳过，累计 {}", errors);
            }
            return out;
        }
        const int width = static_cast<int>(info.width);
        const int height = static_cast<int>(info.height);
        const int hor_stride = static_cast<int>(info.hor_stride);

        if (model_width_ != model_height_) {
            const uint64_t errors = error_count.fetch_add(1) + 1;
            if (ShouldLogThrottled(errors)) {
                SPDLOG_ERROR("RKNN 后端暂只支持正方形模型输入，实际={}x{}，累计 {}",
                             model_width_, model_height_, errors);
            }
            return out;
        }

        // 对完整原图做 letterbox，与已通过验证的 Python 参考代码一致；不再先
        // 居中裁成正方形，避免丢失 16:9 画面左右区域。
        LetterBox letterbox;
        const int ret = Nv12LetterboxToRgb(
            reinterpret_cast<const uint8_t*>(frame.Data()), width, height, hor_stride,
            model_width_, static_cast<uint8_t*>(input_mems_[0][0]->virt_addr),
            &letterbox);
        if (ret != 0) {
            const uint64_t errors = error_count.fetch_add(1) + 1;
            if (ShouldLogThrottled(errors)) {
                SPDLOG_ERROR("RKNN 预处理失败: {}，累计 {}",
                             last_preprocess_error_.empty() ? "未知 RGA 错误"
                                                            : last_preprocess_error_,
                             errors);
            }
            return out;
        }

        // NPU 推理（3 核上下文，单帧由运行时调度）
        const int run_ret = rknn_run(ctxs_[0], nullptr);
        if (run_ret != RKNN_SUCC) {
            const uint64_t errors = error_count.fetch_add(1) + 1;
            if (ShouldLogThrottled(errors)) {
                SPDLOG_ERROR("RKNN 推理失败: ret={}，累计 {}", run_ret, errors);
            }
            return out;
        }

        // 输出张量搬运（NC1HWC2 → NCHW，写入预分配缓冲）
        for (uint32_t i = 0; i < io_num_.n_output; ++i) {
            const int8_t* src = static_cast<int8_t*>(output_mems_[0][i]->virt_addr);
            int8_t* dst = output_buffers_[i].data();
            if (output_native_attrs_[i].fmt == RKNN_TENSOR_NC1HWC2) {
                const int logical_h = output_attrs_[i].n_dims == 3
                                          ? 1
                                          : static_cast<int>(output_attrs_[i].dims[2]);
                const int logical_w = output_attrs_[i].n_dims == 3
                                          ? static_cast<int>(output_attrs_[i].dims[2])
                                          : static_cast<int>(output_attrs_[i].dims[3]);
                ConvertNc1hwc2ToNchw(
                    src, dst, output_native_attrs_[i],
                    static_cast<int>(output_attrs_[i].dims[1]), logical_h, logical_w);
            } else {
                std::memcpy(dst, src, output_native_attrs_[i].n_elems);
            }
        }

        std::vector<YoloDetection> detections;
        if (output_mode_ == OutputMode::kNormalizedXywh) {
            NormalizedXywhTensor tensor;
            tensor.data = output_buffers_[0].data();
            tensor.zp = output_attrs_[0].zp;
            tensor.scale = output_attrs_[0].scale;
            tensor.channels = 5;
            tensor.candidate_count = single_output_candidates_;
            (void)PostProcessNormalizedXywh(
                tensor, model_width_, model_height_, conf_threshold, nms_threshold,
                0, letterbox, &detections);
        } else {
            std::vector<BranchOutput> branches;
            branches.reserve(3);
            for (int i = 0; i < 3; ++i) {
                const int box_idx = i * outputs_per_branch_;
                const int score_idx = box_idx + 1;
                if (score_idx >= static_cast<int>(io_num_.n_output)) {
                    break;
                }
                BranchOutput branch;
                branch.box = MakeTensor(output_buffers_[box_idx].data(),
                                        output_attrs_[box_idx]);
                branch.score = MakeTensor(output_buffers_[score_idx].data(),
                                          output_attrs_[score_idx]);
                if (outputs_per_branch_ == 3 &&
                    box_idx + 2 < static_cast<int>(io_num_.n_output)) {
                    branch.score_sum = MakeTensor(output_buffers_[box_idx + 2].data(),
                                                  output_attrs_[box_idx + 2]);
                }
                branches.push_back(std::move(branch));
            }
            (void)PostProcess(branches, model_width_, conf_threshold, nms_threshold,
                              num_classes_, letterbox, &detections);
        }

        // 后处理已撤销完整原图的 letterbox，直接限制到原图范围并过滤退化框。
        out.reserve(detections.size());
        for (const auto& d : detections) {
            BackendDetection bd;
            bd.class_id = d.class_id;
            bd.confidence = d.confidence;
            bd.x1 = ClampF(d.x1, 0.f, static_cast<float>(width));
            bd.y1 = ClampF(d.y1, 0.f, static_cast<float>(height));
            bd.x2 = ClampF(d.x2, 0.f, static_cast<float>(width));
            bd.y2 = ClampF(d.y2, 0.f, static_cast<float>(height));
            if (bd.x2 > bd.x1 && bd.y2 > bd.y1) {
                out.push_back(bd);
            }
        }
        return out;
    }
};

RknnDetectionBackend::RknnDetectionBackend(std::string model_path,
                                           float conf_threshold,
                                           float nms_threshold)
    : impl_(std::make_unique<Impl>(std::move(model_path), conf_threshold,
                                   nms_threshold)) {}

RknnDetectionBackend::~RknnDetectionBackend() = default;

bool RknnDetectionBackend::Load() {
    if (impl_->loaded) {
        return true;
    }

    std::vector<uint8_t> model_data;
    if (!ReadModelFile(impl_->model_path, &model_data)) {
        SPDLOG_ERROR("RKNN 模型文件不可读: {}", impl_->model_path);
        return false;
    }

    // 初始化 RKNN 上下文（SRAM 加速）
    int ret = rknn_init(&impl_->ctxs_[0], model_data.data(), model_data.size(),
                        RKNN_FLAG_ENABLE_SRAM, nullptr);
    if (ret < 0) {
        SPDLOG_ERROR("RKNN 初始化失败: ret={}，模型={}", ret, impl_->model_path);
        return false;
    }
    impl_->ctx_created_[0] = true;

    // 复制上下文到 3 个 NPU 核心并全部启用
    for (int i = 1; i < 3; ++i) {
        ret = rknn_dup_context(&impl_->ctxs_[0], &impl_->ctxs_[i]);
        if (ret < 0) {
            SPDLOG_ERROR("RKNN 复制上下文失败: ctx={} ret={}", i, ret);
            impl_->ReleaseAll();
            return false;
        }
        impl_->ctx_created_[i] = true;
        rknn_set_core_mask(impl_->ctxs_[i], RKNN_NPU_CORE_ALL);
    }
    rknn_set_core_mask(impl_->ctxs_[0], RKNN_NPU_CORE_ALL);

    if (!impl_->QueryModelInfo()) {
        impl_->ReleaseAll();
        return false;
    }
    if (!impl_->InitializeMems()) {
        impl_->ReleaseAll();
        return false;
    }

    impl_->loaded = true;
    SPDLOG_INFO("RKNN 后端加载成功: 模型={} 输入={}x{} 输出模式={}",
                impl_->model_path, impl_->model_width_, impl_->model_height_,
                impl_->output_mode_ == Impl::OutputMode::kNormalizedXywh
                    ? "单输出归一化xywh"
                    : "多分支");
    return true;
}

void RknnDetectionBackend::Unload() noexcept {
    if (impl_ != nullptr) {
        impl_->ReleaseAll();
    }
}

bool RknnDetectionBackend::IsLoaded() const {
    return impl_->loaded;
}

std::vector<BackendDetection> RknnDetectionBackend::Detect(
    const video::FrameHandle& frame) {
    return impl_->DetectFrame(frame);
}

}  // namespace drone::perception
