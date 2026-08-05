#ifndef YOLO26_DETECTOR_H
#define YOLO26_DETECTOR_H

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

// 单个检测结果，坐标为原图坐标系（单位：像素）
struct Detection {
    int class_id;      // 类别 ID
    float confidence;  // 置信度
    int x1;            // 检测框左上角 x
    int y1;            // 检测框左上角 y
    int x2;            // 检测框右下角 x
    int y2;            // 检测框右下角 y
};

// YOLO26 目标检测器对外唯一接口类。
// 内部封装了 RKNN 初始化、RGA 预处理、NPU 推理、后处理解码与 NMS 的全部细节，
// 调用方只需构造时传入模型路径，之后反复调用 detect() 即可。
//
// 使用示例：
//   YOLO26Detector detector("yolo26n_int8.rknn");
//   cv::Mat image = cv::imread("bus.jpg");          // BGR 图像
//   std::vector<Detection> results = detector.detect(image);
class YOLO26Detector {
public:
    // 构造时加载模型并完成 RKNN 初始化。
    // 模型文件不存在时抛出 std::runtime_error。
    explicit YOLO26Detector(const std::string& model_path);
    ~YOLO26Detector();

    // RKNN 上下文为独占资源，禁止拷贝
    YOLO26Detector(const YOLO26Detector&) = delete;
    YOLO26Detector& operator=(const YOLO26Detector&) = delete;

    // 允许移动
    YOLO26Detector(YOLO26Detector&&) noexcept;
    YOLO26Detector& operator=(YOLO26Detector&&) noexcept;

    // 核心检测接口。
    // image : 输入图像，按 OpenCV 惯例传入 BGR 三通道图像（imread / 摄像头的默认格式），
    //         内部会自动转换为模型所需的 RGB；也兼容灰度图与 BGRA 四通道图。
    // 返回  : 检测结果列表，检测框坐标已映射回原图坐标系；输入非法或推理失败时返回空列表。
    // 注意  : 本方法非线程安全，多线程使用时请对同一实例加锁。
    std::vector<Detection> detect(const cv::Mat& image);

private:
    // 前向声明 + PIMPL：RKNN/RGA/后处理等所有实现细节对外完全不可见
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#endif // YOLO26_DETECTOR_H
