#include <iostream>
#include <vector>
#include <cstring>
#include <chrono>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "rknn_model.h"
#include "rga_utils.h"

int main(int argc, char* argv[]) {
    // 模型路径（默认使用上级目录的 yolo26n_int8.rknn）
    std::string model_path = "../yolo26n_int8.rknn";
    std::string image_path = "bus.jpg";

    // 支持命令行参数
    if (argc >= 2) {
        model_path = argv[1];
    }
    if (argc >= 3) {
        image_path = argv[2];
    }

    std::cout << "=== YOLO26 RKNN Inference ===" << std::endl;
    std::cout << "Model: " << model_path << std::endl;
    std::cout << "Image: " << image_path << std::endl;

    // 初始化模型
    rknn_model model(model_path);

    int ctx_index = 0;

    // 读取图像
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        std::cerr << "Failed to read the image: " << image_path << std::endl;
        return -1;
    }
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    std::cout << "Image size: " << image.size() << std::endl;

    // 预热推理
    std::cout << "\nWarming up..." << std::endl;
    object_detect_result_list od_results;
    for (int i = 0; i < 5; ++i) {
        cv::Mat warmup_img = image.clone();
        model.run_inference(warmup_img, ctx_index, &od_results);
    }

    // 性能测试：100 次推理
    const int num_inferences = 100;
    double total_time_ms = 0.0;

    std::cout << "\nRunning " << num_inferences << " inferences..." << std::endl;
    for (int i = 0; i < num_inferences; ++i) {
        cv::Mat infer_img = image.clone();
        auto start = std::chrono::high_resolution_clock::now();
        int ret = model.run_inference(infer_img, ctx_index, &od_results);
        auto end = std::chrono::high_resolution_clock::now();
        if (ret < 0) {
            printf("rknn_run fail! ret=%d\n", ret);
            return -1;
        }
        std::chrono::duration<double, std::milli> elapsed = end - start;
        total_time_ms += elapsed.count();
    }

    double avg_time_ms = total_time_ms / num_inferences;
    double fps = num_inferences / (total_time_ms / 1000.0);

    std::cout << "\n========== Performance ==========" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Average inference time: " << avg_time_ms << " ms" << std::endl;
    std::cout << "FPS: " << fps << std::endl;
    std::cout << "=================================" << std::endl;

    // 查询 NPU 实际推理耗时
    rknn_perf_run perf_run;
    int ret = rknn_query(model.get_context(ctx_index), RKNN_QUERY_PERF_RUN, &perf_run, sizeof(perf_run));
    if (ret == RKNN_SUCC) {
        std::cout << "NPU inference time: " << std::fixed << std::setprecision(4)
            << static_cast<double>(perf_run.run_duration) / 1000.0 << " ms" << std::endl;
    }

    // 打印检测结果
    std::cout << "\nDetected " << od_results.count << " objects:" << std::endl;
    for (int i = 0; i < od_results.count; ++i) {
        object_detect_result result = od_results.results[i];
        printf("  Object %d: class=%d, conf=%.2f, box=(%d, %d, %d, %d)\n",
            i + 1, result.cls_id, result.prop,
            result.box.left, result.box.top, result.box.right, result.box.bottom);
    }

    // 绘制检测框并保存结果
    cv::Mat image_bgr = image.clone();
    cv::cvtColor(image_bgr, image_bgr, cv::COLOR_RGB2BGR);

    for (int i = 0; i < od_results.count; ++i) {
        object_detect_result result = od_results.results[i];

        // 绘制矩形框
        cv::Rect rect(result.box.left, result.box.top,
            result.box.right - result.box.left, result.box.bottom - result.box.top);
        cv::rectangle(image_bgr, rect, cv::Scalar(0, 255, 0), 2);

        // 添加文本标签
        std::ostringstream label;
        label << "ID:" << result.cls_id << " " << std::fixed << std::setprecision(2) << result.prop;
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::rectangle(image_bgr,
            cv::Point(result.box.left, result.box.top - label_size.height - 2),
            cv::Point(result.box.left + label_size.width, result.box.top + baseLine),
            cv::Scalar(0, 255, 0), -1);
        cv::putText(image_bgr, label.str(),
            cv::Point(result.box.left, result.box.top),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }

    cv::imwrite("result.jpg", image_bgr);
    std::cout << "\nResult saved to: result.jpg" << std::endl;

    return 0;
}
