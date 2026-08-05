#include "rga_utils.h"

void crop_image_to_square_and_16_alignment(cv::Mat& image) {
    int origin_width = image.cols;
    int origin_height = image.rows;

    // 计算短边尺寸
    int short_side = std::min(origin_width, origin_height);

    // 确保裁剪后的尺寸能被16整除
    int crop_width = (short_side / 16) * 16;
    int crop_height = (short_side / 16) * 16;

    if (crop_width == origin_width && crop_height == origin_height) {
        return;
    }

    // 计算裁剪区域的左上角坐标（居中裁剪）
    int crop_x = (origin_width - crop_width) / 2;
    int crop_y = (origin_height - crop_height) / 2;

    image = image(cv::Range(crop_y, crop_y + crop_height), cv::Range(crop_x, crop_x + crop_width));

    if (!image.isContinuous()) {
        image = image.clone();
    }
}

void crop_image_to_16_alignment(cv::Mat& image) {
    int origin_width = image.cols;
    int origin_height = image.rows;

    int aligned_width = (origin_width / 16) * 16;
    int aligned_height = (origin_height / 16) * 16;

    if (aligned_width == origin_width && aligned_height == origin_height) {
        return;
    }

    int crop_x = (origin_width - aligned_width) / 2;
    int crop_y = (origin_height - aligned_height) / 2;

    image = image(cv::Range(crop_y, crop_y + aligned_height), cv::Range(crop_x, crop_x + aligned_width));

    if (!image.isContinuous()) {
        image = image.clone();
    }
}

void crop_to_square_align16(const cv::Mat& src, cv::Mat& dst) {
    int src_width = src.cols;
    int src_height = src.rows;
    int short_side = std::min(src_width, src_height);

    int crop_x = (src_width - short_side) / 2;
    int crop_y = (src_height - short_side) / 2;

    // 使用 RGA 硬件加速裁剪
    rga_buffer_t src_buf = wrapbuffer_virtualaddr(src.data, src_width, src_height, RK_FORMAT_RGB_888);
    rga_buffer_t dst_buf = wrapbuffer_virtualaddr(dst.data, short_side, short_side, RK_FORMAT_RGB_888);

    im_rect crop_rect = { crop_x, crop_y, short_side, short_side };

    IM_STATUS status = imcrop(src_buf, dst_buf, crop_rect);
    if (status != IM_STATUS_SUCCESS) {
        std::cerr << "RGA crop failed: " << imStrError(status) << std::endl;
    }
}

int adaptive_letterbox(const cv::Mat& src, int target_size, uint8_t* dst_virtual_addr,
    letterbox_t* letterbox, uint8_t fill_color, int interpolation)
{
    if (src.empty() || dst_virtual_addr == nullptr || target_size <= 0 || letterbox == nullptr) {
        std::cerr << "Invalid input parameters." << std::endl;
        return -1;
    }

    int src_width = src.cols;
    int src_height = src.rows;

    // 图像已经是目标尺寸，用 RGA 完成 BGR→RGB 色彩转换后写入
    if (src_width == target_size && src_height == target_size) {
        rga_buffer_t src_buf = wrapbuffer_virtualaddr(src.data, src_width, src_height, RK_FORMAT_BGR_888);
        rga_buffer_t dst_buf = wrapbuffer_virtualaddr(dst_virtual_addr, target_size, target_size, RK_FORMAT_RGB_888);
        IM_STATUS status = imcvtcolor(src_buf, dst_buf, RK_FORMAT_BGR_888, RK_FORMAT_RGB_888);
        if (status != IM_STATUS_SUCCESS) {
            std::cerr << "RGA cvtcolor failed: " << imStrError(status) << std::endl;
            return -1;
        }
        letterbox->scale = 1.0f;
        letterbox->x_pad = 0;
        letterbox->y_pad = 0;
        return 0;
    }

    // 计算缩放比例（保持宽高比）
    float scale = std::min(static_cast<float>(target_size) / src_width,
                           static_cast<float>(target_size) / src_height);

    int new_width = static_cast<int>(src_width * scale);
    int new_height = static_cast<int>(src_height * scale);

    // 缩放后恰好等于目标尺寸（源为 BGR，目标为 RGB，RGA 缩放同时完成色彩转换）
    if (new_width == target_size && new_height == target_size) {
        rga_buffer_t src_buf = wrapbuffer_virtualaddr(src.data, src_width, src_height, RK_FORMAT_BGR_888);
        rga_buffer_t dst_buf = wrapbuffer_virtualaddr(dst_virtual_addr, target_size, target_size, RK_FORMAT_RGB_888);

        IM_STATUS status = imresize(src_buf, dst_buf, scale, scale, interpolation);
        if (status != IM_STATUS_SUCCESS) {
            std::cerr << "RGA resize failed: " << imStrError(status) << std::endl;
            return -1;
        }
        letterbox->scale = scale;
        letterbox->x_pad = 0;
        letterbox->y_pad = 0;
        return 0;
    }

    // 计算居中填充偏移
    int pad_left = (target_size - new_width) / 2;
    int pad_top = (target_size - new_height) / 2;

    // 填充背景色
    memset(dst_virtual_addr, fill_color, target_size * target_size * 3);

    // 使用 RGA 缩放并写入偏移位置（源为 BGR，目标为 RGB，RGA 缩放同时完成色彩转换）
    rga_buffer_t src_buf = wrapbuffer_virtualaddr(src.data, src_width, src_height, RK_FORMAT_BGR_888);
    rga_buffer_t dst_buf = wrapbuffer_virtualaddr(
        dst_virtual_addr + (pad_top * target_size + pad_left) * 3,
        new_width, new_height, RK_FORMAT_RGB_888);

    IM_STATUS status = imresize(src_buf, dst_buf, scale, scale, interpolation);
    if (status != IM_STATUS_SUCCESS) {
        std::cerr << "RGA resize failed: " << imStrError(status) << std::endl;
        return -1;
    }

    letterbox->scale = scale;
    letterbox->x_pad = pad_left;
    letterbox->y_pad = pad_top;

    return 0;
}

int CV_adaptive_letterbox(const cv::Mat& src, int target_size, uint8_t* dst_virtual_addr,
    uint8_t fill_color, int interpolation)
{
    if (src.empty() || dst_virtual_addr == nullptr || target_size <= 0) {
        std::cerr << "Invalid input parameters." << std::endl;
        return -1;
    }

    int src_width = src.cols;
    int src_height = src.rows;

    if (src_width == target_size && src_height == target_size) {
        memcpy(dst_virtual_addr, src.data, target_size * target_size * 3);
        return 0;
    }

    float scale = std::min(static_cast<float>(target_size) / src_width,
                           static_cast<float>(target_size) / src_height);

    int new_width = static_cast<int>(src_width * scale);
    int new_height = static_cast<int>(src_height * scale);

    if (new_width == target_size && new_height == target_size) {
        cv::Mat resized_img;
        cv::resize(src, resized_img, cv::Size(target_size, target_size), 0, 0, interpolation);
        memcpy(dst_virtual_addr, resized_img.data, target_size * target_size * 3);
        return 0;
    }

    int pad_left = (target_size - new_width) / 2;
    int pad_top = (target_size - new_height) / 2;

    cv::Mat dst_img(target_size, target_size, CV_8UC3, cv::Scalar(fill_color, fill_color, fill_color));

    cv::Mat resized_img;
    cv::resize(src, resized_img, cv::Size(new_width, new_height), 0, 0, interpolation);
    resized_img.copyTo(dst_img(cv::Rect(pad_left, pad_top, new_width, new_height)));

    memcpy(dst_virtual_addr, dst_img.data, target_size * target_size * 3);

    return 0;
}
