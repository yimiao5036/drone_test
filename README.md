# drone_test

运行在香橙派 RK3588 算力板上的反无人机控制系统，使用 C++17 和 CMake 实现。当前视频/YOLO、双目测距、PX4 遥测、地面站 MAVLink 2、TIMESYNC、目标 UPDATE/ACK、健康/任务状态回传均已完成实机闭环。`MissionStateMachine` 已实现 S5/S6/S7 与 S9~S14 主流程链路（S5 位置引导 → S6/S7 视觉交接与伺服 → S9/S10 拦截入口 → S11~S14 返航/降落/上锁；S8 避障休眠），但生产装配仍为安全影子：`interception.enabled=false` 不自动触发发射，`enable_control=false` 不向 PX4 发送飞行控制。单目标 `TargetEstimator` 以 EKF 锚点（`GPS_GLOBAL_ORIGIN` #49）为换算基准输出局部 NED 恒速度卡尔曼影子估计，可选融合双目视觉观测；单目标视觉观测帧级链路已接入，由 `VisualTargetMonitor` 输出视觉稳定性影子状态。

## 第一次接触项目先看这里

不要从某个大型`.cpp`文件开始猜数据流。按以下顺序阅读：

1. [项目总览与数据流](docs/项目总览与数据流.md)：正式/影子/Stub边界、全部数据流、线程和当前硬件能力；
2. [模块索引](docs/模块索引.md)：每个类的代码、Topic、配置、计数、测试和实现文档；
3. [故障排查手册](docs/故障排查手册.md)：按“无画面、无检测、无图传、串口不通、无ACK”等现象定位；
4. [新增设备接入指南](docs/新增设备接入指南.md)：以后替换摄像头、接双目或增加其他设备时的固定修改步骤。

**PX4 已到位（2026-10-10：装机体 + GPS，可出 Home/EKF 锚点流）**。视频和地面站物理链路已完整验证；依赖 PX4 位置/姿态/锚点的目标 NED 实链与状态机导航已完成室内只读与室外实链验证（结论见 `docs/开发进度.md`）。**飞行控制输出仍未启用**（`enable_control=false`），控制不能标记为已验证。

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

下表是当前已确认的硬件与软件方案；PX4 已到位（装机体 + GPS），但飞行控制输出仍未启用。

| 项目 | 当前方案 |
|------|----------|
| 算力板 | Orange Pi，RK3588，Ubuntu |
| 飞控 | Pixhawk 2.4.8，PX4 1.17.0 |
| 算力板与飞控 | `/dev/ttyS1` 串口 MAVLink |
| 地面站 | 自研，通过电台与算力板使用 MAVLink 通信 |
| 摄像头 | USB 双目（SBS MJPG，UVC）已接入并完成内参/立体标定；左目用于 YOLO 检测与图传，右目用于稀疏 Census 测距（影子） |
| 视觉算法 | YOLOv26 RKNN；正式模型输出 `[1,5,8400]` 归一化框与置信度，类别名称由 JSON 配置 |
| 测距 | 双目稀疏 Census v2 已影子接入（2026-10-10 板上有效占比 95.9%、约 40ms/帧；2026-10-20 室外实测目标 <8m 可正常使用，近距 1.9m 有效占比 99.2%；0% 根因为超出 ≈7.95m 量程）；卷尺级精度验收待独立测试工程回填，深度无效时显式降级 |
| 导航信息 | PX4 融合 GPS、IMU、磁力计和气压计；算力板自身没有 GPS |
| Home 点 | 目标估计以 EKF 锚点（`GPS_GLOBAL_ORIGIN` #49）为原点；S12 返航使用 PX4 上报的 Home 点（由 Home 经纬度 + EKF 锚点换算为局部 NED） |
| 遥控器 | 用于测试、故障处理和人工接管 |
| 供电 | 香橙派RK3588使用5V/4A供电并与PX4共用无人机动力电池；电量统一读取PX4 MAVLink电池状态，不设独立电源管理模块 |

## V1 能力边界

- 地面站持续发送目标编号、经纬度、高度、速度、航向、定位精度、时间戳和有效期。
- 未稳定识别目标时，算力板根据目标位置向 PX4 发送位置引导指令。
- 视觉连续稳定后，在到达上一位置控制目标且飞行状态稳定时，切换为速度和方向控制。
- 视觉目标短暂丢失可由状态估计器预测，并结合最新地面站位置执行搜索；完整丢失策略保留为可替换策略。
- 双目摄像头提供目标/障碍画面方位和深度测距（影子），光流提供图像运动信息；双目深度无效时必须降级，不能伪造距离。
- V1 遇到障碍物只保证减速、刹停和悬停，不承诺自主绕行；避障策略通过抽象接口提供，方便后续替换。
- V1 拦截结束后默认返回，不自动执行第二次任务。
- 正常返航由算力板使用 PX4 上报的 Home 点控制（S12 位置设定值，到达后 S13 AUTO.LAND、S14 DISARM）；正常返航不使用 PX4 自动 RTL。算力板、程序或串口完全失效时，由 PX4 自身的失联保护执行后备 RTL。
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

### 维护入口

- [项目总览与数据流](docs/项目总览与数据流.md)
- [模块索引](docs/模块索引.md)
- [故障排查手册](docs/故障排查手册.md)
- [新增设备接入指南](docs/新增设备接入指南.md)

### 设计与阶段文档

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
- [视觉目标稳定性判定实现文档](include/perception/视觉目标稳定性判定.md)
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
- [视频链路延迟探针](tools/video_latency_probe.md)
- [NPU 性能分析探针使用方法](tools/NPU性能分析探针使用方法.md)

## 当前正式工程目录

```text
drone_test/
├── CMakeLists.txt
├── main.cpp
├── README.md
├── docs/                     # 全部设计文档
├── include/
│   ├── common/               # logger.h, topic.h, types.h（公共消息类型与 Topic 名称常量）
│   ├── video/                # RTSP/UVC 接收、解码、内存池、SBS 拆分、RGA 转存、叠加
│   ├── video_transmission/   # video_sender.h, video_encoder.h（编码+图传协议可替换边界）
│   ├── perception/           # YOLO/RKNN、双目测距、WGS84→NED、目标估计、视觉稳定性判定
│   ├── communication/        # serial_port.h, communication_transport.h, mavlink_handler.h,
│   │                         # px4_link.h, ground_station_link.h
│   ├── control/              # flight_controller.h、tracking_control_law.h（视觉伺服）、
│   │                         # line_of_sight/pid_controller/rate_limiter 等
│   ├── state_machine/        # mission_state_machine.h（S0~S16，含 S5~S14 意图链）
│   ├── health/               # health_manager.h
│   ├── config/               # config.h（AppConfig 解析与校验）
│   └── application/          # drone_application.h（组件装配根）
├── src/                      # 各模块实现（含真实实现与 Stub）
├── tests/                    # 单元测试 + tests/skeleton/ 骨架冒烟测试
├── config/                   # config.json、px4_sitl*.json、相机标定/立体校正矩阵
├── tools/                    # PX4 SITL/冒烟、视频延迟探针、标定、Web 地面站等
├── models/                   # RKNN 模型（生产文件名见 config.json）
├── third_party/
├── target_tracker/           # 独立视觉追踪库（不接入主应用）
├── visual_track_control/     # 独立视觉控制律库（核心律已移植主工程，原库不接入）
└── videoPart/                # 视频/RKNN历史验证原型
```

## 构建环境

- 运行目标：Ubuntu / ARM64 / RK3588（香橙派）
- 开发/构建环境：Windows + WSL2 中的 **Ubuntu 24.04**（工具链已配置好，本项目唯一构建环境；WSL2 内的 Ubuntu 20.04 不用于本项目）
- C++17、CMake、spdlog、Google Test、FFmpeg（dev 包：libavformat/libavcodec/libavutil/libswscale；香橙派需 ffmpeg-rockchip 版）
- 提交前在 WSL2 Ubuntu 24.04 中执行 `cmake -S . -B build && cmake --build build` 编译通过

正式视频链路已在香橙派闭环：H.265 RTSP → rkmpp 硬解 → RGA DMA-BUF 转存 → RKNN YOLO → 红色动态框叠加 → h264_rkmpp → MediaMTX/HM30；RGA DMA 600 秒长测回退 0。双目 UVC 链路（MJPG 硬解 + SBS 拆分 + 稀疏 Census 测距）已板上冒烟（2026-10-10 有效占比 95.9%、约 40ms/帧）；**室外实测目标 <8m 可正常使用**（2026-10-20 近距 1.9m 有效占比 99.2%，0% 根因为超出 ≈7.95m 量程）；卷尺级精度与 `fxB` 对齐仍为控制接入前置。PX4 链路已完成 MAVLink 解析、遥测快照、命令 ACK 与 NED setpoint 底层能力；G1~G3 SITL 门禁已全部实跑通过（2026-10-14），P3 前馈 A/B 与 S9~S14 返航专项通过（2026-10-19），G4 串口台架自动部分已实跑通过（2026-10-21，ARM/Offboard/接管/零速/四向设定值方向/停发/DISARM；G4-1 电机怠速已人工确认，G4-2 实际方向待判读、G4-3 捕网暂缓；G4-4 拔串口机载侧已完成，PX4 failsafe 由飞控自身负责）；正式应用仍只接入只读遥测。地面站链路已完成捕网-01=`1/25` 多机身份、TIMESYNC、65010/65011 目标 UPDATE/ACK、65012 健康状态和 65013 任务状态回传，Web/HM30 实链闭环已通过。`MissionStateMachine` 已实现 S5/S6/S7 与 S9~S14 意图链（含 `interception` 入口配置化与返航/降落/上锁），生产装配保持影子；正式单目标 `TargetEstimator` 以 EKF 锚点（#49）为基准并行接入，输出尚不参与 PX4 控制。`target_tracker` 独立目录保持不动，`visual_track_control` 核心控制律已移植主工程、原库未接入 `DroneApplication`。主工程在 WSL2 Ubuntu 24.04 下 **495/496** 测试通过（唯一失败为既有已知失败（ctest 逐用例隔离稳定复现；整套 `target_estimator_test` 36/36 通过，见 `docs/故障排查手册.md` §3）：`TargetEstimatorBaselineTest.GroundTargetConvertsWithAnchorBaseline`）。PX4 室内只读（10-10）与室外实链只读（10-15：`SELF_CHECK→READY`、目标 UPDATE→估计器出 NED→`GPS_APPROACH`、Home 漂移对照）均已通过。正式配置保持 `enable_control=false` 与 `interception.enabled=false`；打开控制前剩余 G4-2 实际方向/G4-3 捕网执行器、G5 室外首飞、双目卷尺级精度验收、相机安装外参、围栏参数冻结等前置项（见 `docs/PX4实链验证清单.md` §7.2/§7.3）。
