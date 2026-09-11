# drone_test

运行在香橙派 RK3588 算力板上的反无人机控制系统，使用 C++17 和 CMake 实现。当前视频/YOLO、PX4遥测、地面站MAVLink 2、TIMESYNC、目标UPDATE/ACK、健康/任务状态回传均已完成实机闭环；MissionStateMachine处于安全影子阶段。单目标`TargetEstimator`第一阶段已接入正式应用，将地面站WGS-84目标和PX4 Home转换为局部NED并进行恒速度线性卡尔曼影子估计。正式配置继续保持`enable_control=false`，不向PX4发送飞行控制。

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
| 飞控 | Pixhawk 2.4.8，PX4 1.17.0 |
| 算力板与飞控 | `/dev/ttyS1` 串口 MAVLink |
| 地面站 | 自研，通过电台与算力板使用 MAVLink 通信 |
| 摄像头 | 当前单目前视摄像头准备更换为双目摄像头，计划同时承担目标画面与深度测距；接口、码流和标定方案待新硬件确认 |
| 视觉算法 | YOLOv26 RKNN；正式模型输出 `[1,5,8400]` 归一化框与置信度，类别名称由 JSON 配置 |
| 测距 | 取消当前独立单点激光雷达方案，计划使用双目摄像头深度结果；算法与有效距离待实测 |
| 导航信息 | PX4 融合 GPS、IMU、磁力计和气压计；算力板自身没有 GPS |
| Home 点 | 使用 PX4 上报的 Home 点 |
| 遥控器 | 用于测试、故障处理和人工接管 |
| 供电 | 香橙派RK3588使用5V/4A供电并与PX4共用无人机动力电池；电量统一读取PX4 MAVLink电池状态，不设独立电源管理模块 |

## V1 能力边界

- 地面站持续发送目标编号、经纬度、高度、速度、航向、定位精度、时间戳和有效期。
- 未稳定识别目标时，算力板根据目标位置向 PX4 发送位置引导指令。
- 视觉连续稳定后，在到达上一位置控制目标且飞行状态稳定时，切换为速度和方向控制。
- 视觉目标短暂丢失可由状态估计器预测，并结合最新地面站位置执行搜索；完整丢失策略保留为可替换策略。
- 双目摄像头计划同时提供目标/障碍画面方位和深度测距，光流提供图像运动信息；双目深度无效时必须降级，不能伪造距离。
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

逻辑执行单元包括双目摄像头采集与解码、YOLO/光流/深度感知、图传发送、PX4通信、地面站电台通信以及状态机与控制。不再规划独立激光雷达线程和独立电源管理线程；最终线程数量按双目SDK、解码器和推理后端调整。

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
- [部署与打包](docs/部署与打包.md)
- [开发进度](docs/开发进度.md)
- [视频链路延迟实测分析](docs/视频链路延迟实测分析.md)
- [YOLO26 RKNN逐层性能分析](docs/YOLO26_RKNN逐层性能分析.md)
- [目标追踪库实现文档](target_tracker/target_tracker.md)
- [视觉跟踪控制律实现文档](visual_track_control/视觉跟踪控制律实现文档.md)
- [Topic 发布订阅使用文档](include/common/topic.md)

### 模块实现文档（与代码同目录，随实现更新）

- [异步文件日志](include/common/logger.md)
- [视频帧句柄](include/video/video_frame.md)
- [视频帧内存池](include/video/video_frame_pool.md)
- [RTSP 接收](include/video/camera_receiver.md)
- [视频解码](include/video/video_decoder.md)
- [视频叠加](include/video/frame_compositor.md)
- [图传发送与编码后端](include/video_transmission/video_sender.md)
- [串口封装](include/communication/serial_port.md)
- [MAVLink 字节流处理器](include/communication/mavlink_handler.md)
- [PX4 通信链路](include/communication/px4_link.md)
- [地面站数传链路](include/communication/ground_station_link.md)
- [PX4 链路冒烟测试](tools/px4_link_smoke.md)
- [PX4 1.17.0 SITL 分级测试](tools/PX4_SITL分级测试.md)
- [主程序集成与数据链路实施计划](docs/主程序集成与数据链路实施计划.md)
- [多机身份与功能寻址设计](docs/多机身份与功能寻址设计.md)
- [地面站目标位置与时间同步协议](docs/地面站目标位置与时间同步协议.md)
- [地面站时间同步响应工具](tools/地面站时间同步响应工具.md)
- [通信传输抽象](include/communication/communication_transport.md)

## 当前正式工程目录

```text
drone_test/
├── CMakeLists.txt
├── main.cpp
├── README.md
├── docs/                     # 全部设计文档
├── include/
│   ├── common/               # logger.h, topic.h, types.h（公共消息类型与 Topic 名称常量）
│   ├── video/                # video_frame.h, video_frame_pool.h, camera_receiver.h, video_decoder.h,
│   │                         #   frame_compositor.h（叠加）
│   ├── video_transmission/   # video_sender.h, video_encoder.h（编码+图传协议可替换边界）
│   ├── perception/           # YOLO、WGS84→NED、单目标线性卡尔曼、目标估计，
│   │                         #   光流/双目融合接口与兼容预留
│   ├── communication/        # serial_port.h, communication_transport.h, mavlink_handler.h,
│   │                         # px4_link.h, ground_station_link.h
│   ├── control/              # flight_controller.h
│   ├── state_machine/        # mission_state_machine.h
│   ├── health/               # health_manager.h
│   └── config/
├── src/                      # 各模块实现（含真实实现与 Stub）
├── tests/                    # 单元测试 + tests/skeleton/ 骨架冒烟测试
├── config/
├── tools/                    # PX4 串口冒烟与强制 UDP 的 SITL 分级控制测试
├── models/                   # yolo26n-int8.rknn
├── third_party/
├── target_tracker/           # 独立视觉追踪库（尚未接入主应用）
├── visual_track_control/     # 独立视觉控制律库（尚未接入主应用）
└── videoPart/                # 视频/RKNN历史验证原型
```

## 构建环境

- 运行目标：Ubuntu / ARM64 / RK3588（香橙派）
- 开发/构建环境：Windows + WSL2 中的 **Ubuntu 24.04**（工具链已配置好，本项目唯一构建环境；WSL2 内的 Ubuntu 20.04 不用于本项目）
- C++17、CMake、spdlog、Google Test、FFmpeg（dev 包：libavformat/libavcodec/libavutil/libswscale；香橙派需 ffmpeg-rockchip 版）
- 提交前在 WSL2 Ubuntu 24.04 中执行 `cmake -S . -B build && cmake --build build` 编译通过

正式视频链路已在香橙派闭环：H.265 RTSP → rkmpp硬解 → RGA DMA-BUF转存 → RKNN YOLO → 红色动态框叠加 → h264_rkmpp → MediaMTX/HM30；RGA DMA 600秒长测回退0。PX4链路已完成MAVLink解析、遥测快照、命令ACK基础和NED setpoint底层能力，SITL第3A~3D已验证Offboard、起降、水平运动和失联降级，但正式应用只接入遥测。地面站链路已完成捕网-01=`1/25`多机身份、TIMESYNC、65010/65011目标UPDATE/ACK、65012健康状态和65013任务状态回传，Web/HM30实链闭环已通过。`MissionStateMachine`当前只运行影子状态；正式单目标`TargetEstimator`也已并行接入，但其输出尚不参与状态机或控制。`target_tracker`独立目录保持不动，`visual_track_control`仍未接入`DroneApplication`。主工程在WSL2 Ubuntu 24.04下188/188测试通过；下一步是在香橙派/HM30实链验证目标NED方向、数量级、时效和噪声参数。正式配置继续保持`enable_control=false`。
