# drone_test

运行在香橙派 RK3588 算力板上的反无人机控制系统。项目当前处于需求与架构设计阶段，计划使用 C++17 和 CMake 实现。

## 系统目标

系统接收自研地面站持续下发的目标无人机位置，通过 PX4 控制拦截无人机接近目标；摄像头稳定捕获目标后切换为视觉速度跟踪。拦截执行机构采用抽象接口，V1 不限定捕网或其他具体手段。

主流程如下：

```text
地面站目标位置
  -> GPS 位置引导
  -> 视觉稳定锁定
  -> 速度与方向跟踪
  -> 拦截
  -> 算力板控制返回 Home
```

## 已确认硬件与软件

| 项目 | 当前方案 |
|------|----------|
| 算力板 | Orange Pi，RK3588，Ubuntu |
| 飞控 | Pixhawk 2.4.8，PX4 1.11.3 |
| 算力板与飞控 | 串口 MAVLink |
| 地面站 | 自研，通过电台与算力板使用 MAVLink 通信 |
| 摄像头 | 机头正前方固定安装，RJ45/TCP，H.265，1080p，25 FPS |
| 视觉算法 | YOLOv26；输出类别、置信度和目标像素位置；光流由算力板根据图像计算 |
| 测距雷达 | 机头正前方单点激光雷达，更新频率可配置 |
| 导航信息 | PX4 融合 GPS、IMU、磁力计和气压计；算力板自身没有 GPS |
| Home 点 | 使用 PX4 上报的 Home 点 |
| 遥控器 | 用于测试、故障处理和人工接管 |
| 供电 | 动力电池与算力板独立电池分开供电 |

## V1 能力边界

- 地面站持续发送目标编号、经纬度、高度、速度、航向、定位精度、时间戳和有效期。
- 未稳定识别目标时，算力板根据目标位置向 PX4 发送位置引导指令。
- 视觉连续稳定后，在到达上一位置控制目标且飞行状态稳定时，切换为速度和方向控制。
- 视觉目标短暂丢失可由状态估计器预测，并结合最新地面站位置执行搜索；完整丢失策略保留为可替换策略。
- 摄像头负责识别障碍物及其画面方位，单点激光雷达提供机头前向绝对距离，光流提供图像运动信息。
- V1 遇到障碍物只保证减速、刹停和悬停，不承诺自主绕行；避障策略通过抽象接口提供，方便后续替换。
- V1 拦截结束后默认返回，不自动执行第二次任务。
- 正常返航由算力板使用 PX4 上报的 Home 点控制；算力板、程序或串口完全失效时，由 PX4 自身的失联保护执行后备 RTL。
- 遥控器人工接管优先于算力板任务控制。

## 架构原则

- 采用线程、消息队列和内存池解耦物理链路、感知和控制。
- 图像队列有界，优先处理最新帧，不假设推理永远快于采集。
- 所有跨线程数据携带时间戳、序号、有效期和来源健康状态。
- PX4 通信线程是向飞控发送 MAVLink 指令的唯一出口。
- 状态机与控制线程集中管理任务状态和控制权。
- 避障、目标丢失、返航、拦截授权、拦截后决策和执行机构均通过接口隔离。
- V1 实现保持简单，但不得把具体硬件、阈值或算法固化到状态机中。

初始逻辑执行单元包括摄像头采集与解码、YOLO/光流感知、图传发送、PX4 通信、地面站电台通信、激光雷达读取以及状态机与控制。最终操作系统线程数量允许根据解码器和推理后端调整。

## 文档

- [需求分析](docs/需求分析.md)
- [系统架构设计](docs/系统架构设计.md)
- [状态机设计](docs/状态机设计.md)
- [开发思路](docs/开发思路.md)
- [通信与数据定义](docs/通信与数据定义.md)
- [数据接口文档](docs/数据接口文档.md)
- [感知与避障设计](docs/感知与避障设计.md)
- [电源与失效保护设计](docs/电源与失效保护设计.md)
- [RK3588 RTSP + YOLO 视频链路修复与优化方案](videoPart/rtsp_yolo_stream/OPTIMIZATION_PLAN.md)
- [项目文件结构与命名规划](docs/项目文件结构.md)
- [开发进度](docs/开发进度.md)
- [Topic 发布订阅使用文档](include/common/topic.md)

### 模块实现文档（与代码同目录，随实现更新）

- [异步文件日志](include/common/logger.md)
- [视频帧句柄](include/video/video_frame.md)
- [视频帧内存池](include/video/video_frame_pool.md)
- [RTSP 接收](include/video/camera_receiver.md)
- [视频解码](include/video/video_decoder.md)
- [串口封装](include/communication/serial_port.md)

## 当前正式工程目录

```text
drone_test/
├── CMakeLists.txt
├── main.cpp
├── README.md
├── docs/                     # 全部设计文档
├── include/
│   ├── common/               # logger.h, topic.h, types.h（公共消息类型与 Topic 名称常量）
│   ├── video/                # video_frame.h, video_frame_pool.h, camera_receiver.h, video_decoder.h
│   ├── video_transmission/   # video_sender.h
│   ├── perception/           # yolo_detector.h, optical_flow_estimator.h, laser_range_finder.h,
│   │                         #   perception_fusion.h, target_estimator.h
│   ├── communication/        # serial_port.h, px4_link.h, ground_station_link.h
│   ├── control/              # flight_controller.h
│   ├── state_machine/        # mission_state_machine.h
│   ├── health/               # health_manager.h
│   └── config/
├── src/                      # 各模块 Stub 实现（业务逻辑阶段替换）
├── tests/                    # 单元测试 + tests/skeleton/ 骨架冒烟测试
├── config/
├── third_party/
└── videoPart/                # 现有视频/RKNN验证原型，后续按模块迁移
```

## 构建环境

- 运行目标：Ubuntu / ARM64 / RK3588（香橙派）
- 开发/构建环境：Windows + WSL2 中的 **Ubuntu 24.04**（工具链已配置好，本项目唯一构建环境；WSL2 内的 Ubuntu 20.04 不用于本项目）
- C++17、CMake、spdlog、Google Test、FFmpeg（dev 包：libavformat/libavcodec/libavutil/libswscale；香橙派需 ffmpeg-rockchip 版）
- 提交前在 WSL2 Ubuntu 24.04 中执行 `cmake -S . -B build && cmake --build build` 编译通过

正式工程骨架、根入口和发布订阅基础类已经创建；13 个部件接口抽象与 Stub 占位实现已完成（接口签名见 `docs/数据接口文档.md`）。已按实施顺序迁入真实实现：`video` 模块的 RTSP 接收（CameraReceiver）与解码（VideoDecoder，rkmpp 硬解/软解回退）、`perception` 模块的 YOLO 检测（YoloDetector + 后处理纯函数 + RKNN 后端条件编译，详见 `docs/开发进度.md`）。`videoPart` 中的代码仍作为硬件验证原型保留；RKNN 推理链路留待香橙派实机验证，并在 RK3588、PX4 SITL、台架和受控飞行环境中分级验证。
