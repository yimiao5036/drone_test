# YOLO26 RKNN 推理项目

基于 YOLO26 模型的 RKNN NPU 加速推理项目，部署于香橙派 5 Plus (RK3588)。

## 项目概述

本项目参考 `rknn-cpp-yolo` 项目结构，针对 YOLO26 模型进行了适配和优化。

### 与 YOLO11 的关键区别

| 特性 | YOLO11 | YOLO26 |
|------|--------|--------|
| DFL (Distribution Focal Loss) | 有，需要 softmax 解码 | **无**，直接回归 |
| Box 输出通道数 | 64 (4×16) | **4** |
| INT8 量化精度损失 | 2-3% | **<1%** |
| 后处理复杂度 | 较高 | 较低 |

### 模型输出格式

本项目使用**非 end2end** 导出的 YOLO26 模型，输出格式为：

```
Output 0: box   [1, 4, 80, 80]    ← stride 8
Output 1: score [1, 80, 80, 80]
Output 2: box   [1, 4, 40, 40]    ← stride 16
Output 3: score [1, 80, 40, 40]
Output 4: box   [1, 4, 20, 20]    ← stride 32
Output 5: score [1, 80, 20, 20]
```

- 共 **6 个输出**（3 个检测分支 × 2：box + score）
- `dfl_len = 4/4 = 1`，无需 DFL 解码
- `output_per_branch = 2`，无 score_sum

## 目录结构

```
yolo26-rknn/
├── include/
│   ├── common.h          # 公共类型、量化/反量化、NMS 声明
│   ├── rga_utils.h       # RGA 硬件加速图像预处理
│   └── rknn_model.h      # 模型封装类声明
├── src/
│   ├── common.cpp        # 后处理实现（支持有/无 DFL）
│   ├── rga_utils.cpp     # letterbox + RGA resize 实现
│   └── rknn_model.cpp    # 模型加载、推理、后处理
├── runtime/
│   └── Linux/
│       └── librknn_api/
│           └── include/  # RKNN API 头文件
├── main.cpp              # 主程序入口
├── CMakeLists.txt        # 构建配置
├── CMakePresets.json     # CMake 预设
└── bus.jpg               # 测试图片
```

## 编译运行

### 依赖

- OpenCV
- librga (RGA 硬件加速)
- librknnrt (RKNN Runtime)

### 编译

```bash
cd yolo26-rknn
mkdir build && cd build
cmake ..
make -j4
```

### 运行

```bash
# 默认参数
./yolo26_rknn

# 指定模型和图片路径
./yolo26_rknn /path/to/yolo26n_int8.rknn /path/to/image.jpg
```

## 核心处理流程

### 1. 预处理

```
输入图像 → 居中裁剪为正方形 → RGA letterbox (640×640) → 写入 NPU 输入内存
```

- 使用 RGA 硬件加速 resize
- Letterbox 填充色: RGB(114, 114, 114)
- 保持宽高比，避免目标形变

### 2. NPU 推理

```cpp
rknn_run(ctxs_[ctx_index], NULL);
```

- 支持 3 个 NPU 核心并行推理
- 启用 SRAM 优化中间 tensor 内存

### 3. 后处理

```
NC1HWC2 → NCHW 格式转换 → INT8 反量化 → 坐标解码 → 置信度过滤 → NMS → 映射回原图
```

**YOLO26 无 DFL 的坐标解码：**
```cpp
// YOLO26: dfl_len == 1, 直接反量化
for (int k = 0; k < 4; k++) {
    box[k] = deqnt_affine_to_f32(box_tensor[offset], box_zp, box_scale);
    offset += grid_len;
}

// 计算边界框真实坐标
x1 = (-box[0] + j + 0.5f) * stride;
y1 = (-box[1] + i + 0.5f) * stride;
x2 = (box[2] + j + 0.5f) * stride;
y2 = (box[3] + i + 0.5f) * stride;
```

## 性能指标

在 RK3588 上 YOLO26n INT8 模型的典型性能：

| 指标 | 典型值    |
|------|--------|
| 推理时间 | ~25 ms |
| FPS | ~35    |
| 模型大小 | ~7 MB  |
| mAP50-95 | 0.479  |

## 代码说明

### common.cpp

- `compute_dfl()`: DFL 解码（YOLO26 不使用，但保留以兼容其他模型）
- `process_i8()`: INT8 输出处理，根据 `dfl_len` 自动选择是否 DFL 解码
- `quick_sort_indice_inverse()`: 按置信度降序排序
- `nms()`: 非极大值抑制

### rknn_model.cpp

- `rknn_model::init_model()`: 加载模型文件，初始化 3 个 NPU 上下文
- `rknn_model::initialize_mems()`: 分配输入输出内存
- `rknn_model::run_inference()`: 完整推理流程
- `rknn_model::post_process()`: 多分支后处理

### rga_utils.cpp

- `crop_image_to_square_and_16_alignment()`: 裁剪对齐
- `adaptive_letterbox()`: RGA 加速的 letterbox 填充

## 注意事项

1. **模型路径**: 默认从上级目录读取 `yolo26n_int8.rknn`，可通过命令行参数指定
2. **RKNN API**: 头文件位于 `runtime/Linux/librknn_api/include/`
3. **RGA 库**: 需要系统安装 `librga`
4. **图像格式**: 输入图像需转换为 RGB 格式

## 参考4

- [rknn-cpp-yolo](../rknn-cpp-yolo/) - 原始 YOLO11 参考项目
- [Ultralytics YOLO26 RKNN 导出文档](https://docs.ultralytics.com/zh/integrations/rockchip-rknn/)
- [RKNN Model Zoo](https://github.com/rockchip-linux/rknn_model_zoo)
