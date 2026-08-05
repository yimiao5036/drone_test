# VideoDecoder — RK3588 MPP硬件解码 + RGA图像预处理

## 项目概述

本项目为 **香橙派5 Plus (RK3588)** 平台提供一个 C++ 封装类 `VideoDecoder`，实现：

1. **MPP 硬件解码**：使用 Rockchip MPP 库解码 H.264/H.265 裸流文件
2. **RGA 硬件加速预处理**：将解码后的 NV12 帧通过 RGA 进行缩放和色彩空间转换（转为 RGB888）
3. **为 NPU 推理提供数据**：输出连续内存的 RGB 图像，可直接送入 YOLO 等模型推理

## 目录结构

```
video_decoder/
├── VideoDecoder.hpp    # VideoDecoder 类声明
├── VideoDecoder.cpp    # 解码 + RGA 处理实现
├── main.cpp            # 测试程序入口
├── CMakeLists.txt      # CMake 构建脚本
└── README.md           # 本文件
```

## 核心设计

### 解码流程

严格复用 Rockchip 官方 `mpi_dec_test.c` 中 `dec_simple` 的经过验证的流程：

- **`pkt_done` 送取交替循环**：送包（`decode_put_packet`）和取帧（`decode_get_frame`）交替执行，队列满时 sleep 1ms 等待，避免死锁
- **`split_parse = 1`**：启用 MPP 内部 NAL 分帧，无需外部处理帧边界
- **`info_change` 处理**：当解码器报告分辨率变化时，创建外部 `MppBufferGroup`，调用 `MPP_DEC_SET_EXT_BUF_GROUP` + `MPP_DEC_SET_INFO_CHANGE_READY`
- **简化模式**：每次从文件读取数据到临时缓冲区，复用同一个 `MppPacket` 对象

### RGA 集成

使用 `librga` 的 `im2d` API：

- `wrapbuffer_virtualaddr` 带 stride 参数包装 MPP 帧的 NV12 数据（正确处理 stride 对齐）
- `imresize` 一步完成 **缩放 + 色彩空间转换**（NV12 → RGB888），RGA 硬件自动处理格式差异
- 输出缓冲区在初始化时一次性分配，避免每帧重复 malloc

### 内存管理

- `MppFrame` 用后即 `deinit`
- `MppPacket` 复用模式，仅 `init` 一次
- `MppBufferGroup` 在 `info_change` 时创建，释放时 `put`
- RGA 输出缓冲区按目标尺寸预分配
- 析构函数自动调用 `release()` 释放全部资源

## API 接口

```cpp
class VideoDecoder {
public:
    // 初始化解码器
    // file_path:     输入裸流文件路径 (.h264/.264/.h265/.265/.hevc)
    // target_width:  RGA输出目标宽度 (如640)
    // target_height: RGA输出目标高度 (如640)
    // target_fmt:    RGA输出像素格式 (默认 RK_FORMAT_RGB_888)
    bool init(const std::string& file_path,
              int target_width,
              int target_height,
              int target_fmt = RK_FORMAT_RGB_888);

    // 解码下一帧并通过RGA转换
    // out_rgb: 输出缓冲区，填充 target_width * target_height * 3 字节
    // 返回: 成功返回true；文件结束或出错返回false
    bool getNextFrame(std::vector<uint8_t>& out_rgb);

    // 释放所有资源 (析构函数自动调用)
    void release();

    // 辅助查询
    int getFrameWidth() const;   // 解码帧原始宽度
    int getFrameHeight() const;  // 解码帧原始高度
    int getFrameCount() const;   // 已解码帧计数
};
```

## 编译与运行

### 环境要求

- 硬件：RK3588 平台（香橙派5 Plus）
- 系统：Ubuntu 22.04 (aarch64)
- 依赖：
  - `librockchip_mpp.so` — MPP 编解码库
  - `librga.so` — RGA 2D 图形加速库
  - CMake >= 3.10
  - C++17 编译器

### 编译

```bash
cd video_decoder
mkdir build && cd build
cmake ..
make -j4
```

### 运行

```bash
# 默认解码前10帧，输出640x640 RGB888
./video_decoder test.h265

# 自定义参数: 20帧, 320x320
./video_decoder test.h264 20 320 320

# 用法说明
./video_decoder
# 输出: 用法: ./video_decoder <input.h264|h265> [frame_count] [target_w] [target_h]
```

### 输出

- `frame_0.rgb` ~ `frame_N.rgb`：纯 RGB888 原始数据文件
- 可用 ffmpeg 查看验证：
  ```bash
  ffmpeg -f rawvideo -pixel_format rgb24 -video_size 640x640 -i frame_0.rgb preview.png
  ```

## 与 NPU 推理集成

`getNextFrame` 返回的 `std::vector<uint8_t>` 包含连续内存的 RGB888 数据，可直接用于 RKNN 推理：

```cpp
VideoDecoder decoder;
decoder.init("stream.h265", 640, 640);

std::vector<uint8_t> rgb;
while (decoder.getNextFrame(rgb)) {
    // rgb.data() 即为 640x640x3 的连续 RGB 数据
    // 直接送入 rknn_inputs_set
    rknn_inputs_set(ctx, 1, inputs);
    rknn_run(ctx, nullptr);
    // ...
}
```

## 关键参考

- Rockchip 官方 `mpi_dec_test.c` — 解码核心循环参考
- Rockchip MPP 文档 (`doc/Rockchip_Developer_Guide_MPP_CN.md`)
- RGA API (`im2d.hpp`) — 硬件加速图像缩放/色彩转换
