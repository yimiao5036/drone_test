# AGENTS.md

本项目是一个运行在香橙派算力板上的**反无人机 C++ 控制系统**。

## 项目概述

- **运行平台**：香橙派（Orange Pi），RK3588 芯片，Ubuntu 系统
- **语言与标准**：C++17
- **构建系统**：CMake
- **日志库**：spdlog
- **主要功能**：接收地面站目标位置 → YOLOv26 目标识别 → 轨迹预测 → 控制 PX4 无人机跟踪/反制
- **通信协议**：MAVLink（与 PX4 飞控和地面站通信）

**当前安全基线**：`config/config.json` 必须保持 `"enable_control": false`，正式程序不向 PX4 发送飞行控制；`interception.enabled` 默认保持 `false`（S9/S10 入口配置化后，生产不自动触发发射），返航/降落链只在 SITL 验收配置中显式启用。`MissionStateMachine`、`TargetEstimator`、`VisualTargetMonitor` 均为影子运行（跑真实算法但不产生控制输出）。**PX4 已到位**（2026-10-10：装机体 + GPS，可出 Home），依赖 PX4 的实链验证由此可开展；但安全基线不变——阶段四控制接入路径代码已全部落地（2026-10-14，Task 1~12：装配门禁 + 状态机 S5~S7 交接链 + 捕网执行器命令 + G2/G3 的 `px4_link_smoke` stage4 工具；`enable_control=true` 才绑定 `Px4Setpoint` 并注入 PX4 快照源/命令接口，false 时影子停发），正式控制输出要等 SITL/台架门禁（G2~G4，SITL 实跑为线下执行项；G1~G3 已通过、G4 串口台架自动部分 2026-10-21 已通过，G4-1 电机怠速已人工确认，G4-2 实际方向已由遥控器成功飞行确认（2026-10-28），G4-3 机制/参数/时序 2026-10-23 冻结，G4-5 地面站直控发射拆桨台架 2026-10-28 实跑通过）验证通过后，才允许在现场本地副本打开 `enable_control`。

## 开发与构建环境（强制规则）

- **运行目标**：代码最终在香橙派（RK3588，ARM64，Ubuntu）上编译、部署和运行。
- **开发机唯一构建环境**：Windows 主机上的 WSL2 中的 **Ubuntu 24.04**（工具链与依赖已配置好）。
  所有本地编译验证、单元测试和提交前检查一律在 Ubuntu 24.04 中进行。
- WSL2 中的 Ubuntu 20.04 **不用于**本项目构建，避免环境不一致导致问题。
- 在 WSL2 Ubuntu 24.04 中的标准构建流程：

  ```bash
  # 主工程
  cmake -S . -B build
  cmake --build build -j$(nproc)
  ctest --test-dir build --output-on-failure     # 基线 638/638
  python3 tools/check_docs.py                    # 文档基线/链接/模块文档检查
  ./build/drone_control                          # 生成 logs/0001_xxx.log
  ./build/drone_control --config /路径/config.json

  # 运行单个测试（二选一）
  ctest --test-dir build -R <测试名> --output-on-failure
  ./build/<test_executable> --gtest_filter='Suite.Case'

  # 切换 RKNN / RGA 后端（CMakeCache 会记住旧值，需 -U 清除）
  cmake -S . -B build -U DRONE_HAVE_RKNN -DDRONE_HAVE_RKNN=ON
  cmake -S . -B build -U DRONE_HAVE_RGA_DMA -DDRONE_HAVE_RGA_DMA=ON
  # ARM64 原生构建两者默认 ON，x86_64 开发机默认 OFF；香橙派需 ffmpeg-rockchip、librknnrt、librga

  # 独立子库（与主工程解耦，单独构建，未接入 DroneApplication）
  cmake -S target_tracker -B target_tracker/build && cmake --build target_tracker/build
  ctest --test-dir target_tracker/build --output-on-failure
  cmake -S visual_track_control -B build_vtc -DBUILD_TESTING=OFF && cmake --build build_vtc

  # Web 开发版地面站（Vue3 + Node，tools/web平台/）
  cd tools/web平台 && npm run dev
  ```

- 工具：`tools/sitl_harness.sh`（**SITL 验收装置**：PX4 SITL + PTY 桥 + GCS 心跳 + 生产程序，子命令 `stack`/`app`/`smoke`/`stop`/`status`；文档 `tools/sitl_harness.md`）、`px4_link_smoke`（PX4 串口冒烟/SITL 分级测试；G4 台架 `--bench-stage4-controller --bench-ack-props-off`，仅串口、必须拆桨）、`video_latency_probe`（视频链路延迟探针）、`tools/*.py`（地面站 TIMESYNC/目标发送测试、`assignment_sitl.py` 任务分配验收）。
- 提交代码前必须保证在 WSL2 Ubuntu 24.04 中编译通过；香橙派上执行最终部署验证。

## 架构设计

采用**线程 + 消息队列**的解耦架构（借鉴 PX4/ROS2 设计思想）：

- 每条物理链路一个独立线程，互不阻塞
- 图像数据通过内存池零拷贝传递
- 线程间通过消息队列或通知机制通信
- 各模块可独立编译、独立测试

### 主要线程

| 线程 | 职责 |
|------|------|
| 摄像头采集线程 | RTSP/UVC 采集原始画面 → 解码 → 内存池 |
| YOLO 识别线程 | 取解码帧识别目标 → 检测结果 |
| 图传发送线程 | 叠加帧 → 编码 → RTSP 推送 |
| PX4 串口通信线程 | 与 PX4 飞控 MAVLink 双向通信 |
| 电台数传线程 | 与地面站 MAVLink 双向通信 |
| 双目深度线程 | 消费左右目帧与检测框，输出带质量/时间戳的稀疏 Census 距离（影子，无控制输出） |
| 状态机与控制线程 | 消费各线程数据，状态机决策并生成控制意图；`enable_control=false` 时控制器影子停发 |

### 架构要点（装配、数据流与边界）

**装配模型**：`main.cpp` → 解析 config → `DroneApplication`（`src/application/drone_application.cpp`）只做三件事：`BuildComponents()` 创建组件、`BindTopics()` 连接 Topic、`Start()/Stop()` 按依赖序管理生命周期。**禁止把业务算法写进 main.cpp 或 DroneApplication**。

**模块间通信**：发布订阅 `Topic<T>`（`include/common/topic.h`），公共消息类型与 Topic 名称常量在 `include/common/types.h`。跨线程数据必须带时间戳、序号、有效期和来源健康状态；图像经内存池零拷贝传递，队列有界、优先最新帧。

**主数据流**（正式装配）：

```text
摄像头 RTSP/H.265 → CameraReceiver → kCameraStream → VideoDecoder → kDecodedFrame
  → YoloDetector → kDetection（每目标一条）+ kVisualTarget（每帧一条）
  → VisualTargetMonitor（影子）→ kVisualTargetStatus
  → FrameCompositor → kAnnotatedFrame → VideoSender → H.264 → MediaMTX/HM30
PX4 /dev/ttyS1 → Px4Link → kFlightState → GroundStationLink → /dev/ttyS6 → 地面站
地面站目标 UPDATE/ACK：MAVLink2 V2_EXTENSION message_type 65010/65011，
  健康 65012、任务状态 65013
```

**模块库**：`drone_video`（拉流/解码/叠加）、`drone_video_transmission`（编码+图传）、`drone_perception`（YOLO/坐标换算/卡尔曼/目标估计）、`drone_communication`（串口/MAVLink/PX4/地面站）、`drone_state_machine`、`drone_control_core`、`drone_health`、`drone_config`、`drone_application`。FFmpeg 经 PIMPL 隔离不泄漏到头文件；MAVLink 头以 SYSTEM include 传播压警告。

**状态标记**（文档与代码边界）：正式 / 影子 / Stub / 独立 / 阻塞——改动前确认目标模块当前处于哪种状态，不要把规划能力当成已实现。

**路径约定**：部署为目录式绿色软件，所有路径经 `/proc/self/exe` 相对可执行文件解析（`config/`、`models/`、`logs/`）；CMake 每次构建自动同步 `models/` 和 `config/` 到 build 目录。配置查找顺序：`--config` 参数 → 可执行文件旁 `config/config.json` → `./config/config.json`。

## 目录结构

```
drone_test/
├── CMakeLists.txt
├── main.cpp                 # 唯一正式入口：配置/日志/信号 + DroneApplication
├── README.md / AGENTS.md
├── docs/                    # 文档入口 + 当前设计/进度/排查 + archive/ 历史材料
├── include/                 # 与 src/ 镜像；每个模块同级维护 *.md 实现文档
│   ├── application/         # drone_application.h（组件装配根）
│   ├── common/              # topic.h, types.h, logger.h, latency_statistics.h
│   ├── video/               # RTSP/UVC 接收、解码、帧池、SBS 拆分、RGA 转存、叠加
│   ├── video_transmission/  # 编码 + 图传
│   ├── perception/          # YOLO/RKNN、双目测距、WGS84→NED、目标估计、视觉稳定性
│   ├── communication/       # serial_port.h, mavlink_handler.h, px4_link.h, ground_station_link.h
│   ├── control/             # flight_controller.h, tracking_control_law.h, shadow 等
│   ├── state_machine/       # mission_state_machine.h（S0~S16）
│   ├── health/              # health_manager.h
│   └── config/              # config.h（AppConfig 解析与校验）
├── src/                     # 与 include/ 对应的 .cpp 实现
├── tests/                   # Google Test，目录与 src/ 对应
├── config/                  # JSON 配置 + 相机标定/立体校正矩阵
├── tools/                   # SITL/冒烟、探针、标定、Web 地面站、证据目录
├── models/                  # RKNN 模型（生产文件名见 config.json）
├── third_party/             # spdlog / mavlink / nlohmann / rknn 头文件
├── target_tracker/          # 独立多目标追踪库（不接入主应用）
├── visual_track_control/    # 独立视觉控制律库（核心律已移植主工程）
├── stereo_depth/            # 独立双目深度子工程（不接入）
├── yolo_pt/                 # 模型训练与 pt→RKNN 导出（Python；本地工作区，不入库）
├── videoPart/               # 视频/RKNN 历史验证原型
└── deploy/ logs/            # 部署文件 / 运行时日志（logs 不入库）
```

### 目录边界

| 目录 | 性质 |
|---|---|
| `src/`、`include/`、`main.cpp` | 正式工程（镜像结构，模块一一对应） |
| `tests/` | Google Test，结构对应 `src/`；`tests/skeleton/` 为骨架冒烟 |
| `target_tracker/`、`visual_track_control/` | 独立子工程（自有 CMakeLists），未接入主应用，保持独立构建 |
| `videoPart/` | 视频/RKNN 历史验证原型，不进正式装配 |
| `yolo_pt/` | 模型训练与 pt→RKNN 导出（Python）；本地工作区，父仓库不跟踪，内部单独 git |
| `tools/` | 冒烟/探针/地面站测试工具 + `web平台/`（Vue3 开发版地面站）+ 证据目录 |
| `docs/` | 当前文档入口 + `archive/`/`superpowers/` 历史材料（中文文件名） |

## 维护文档入口（排查/改动前先读）

1. `docs/README.md` — **文档唯一入口**：文档地图、单一事实来源、更新矩阵、基线位置
2. `docs/项目总览与数据流.md` — 正式/影子/Stub 边界、全部数据流、线程模型
3. `docs/模块索引.md` — 按类名/Topic/配置/测试定位代码
4. `docs/故障排查手册.md` — 按现象定位（无画面、无检测、无 ACK 等）
5. `docs/新增设备接入指南.md` — 替换/新增硬件的固定步骤
6. `docs/开发进度.md` — 当前状态/门禁/最近变更；`docs/开发历史.md` — 历史时间线
7. 各模块头文件同级的实现文档

## 语言、命名与信息核实规则

- **统一中文**：项目沟通、代码注释、提交说明及文档正文统一使用中文；
  代码标识符（类名、函数名、变量名）、命令行、第三方库名等保留原文，
  如 `Topic`、`cmake`、`spdlog`。
- **文件命名**：`README.md`、`AGENTS.md`、`CMakeLists.txt` 等工具约定或
  配置文件保留规定名称；其他文档使用中文文件名，如 `设计文档.md`、
  `连接协议.md`。
- **不得猜测未确认的信息**：先检查源码、配置和 git 历史；仓库没有依据时，
  查阅官方文档或可靠来源并记录链接。无法联网、来源冲突或涉及产品取舍时，
  先询问维护者确认后再执行。

## 开发规范

### 代码组织
- 头文件放在 `include/` 对应子目录，源文件放在 `src/` 对应子目录
- 新增模块时，公共类型定义放在 `include/common/types.h`
- 线程安全的数据结构放在 `common/message_queue.h` 和 `common/memory_pool.h`
- 所有串口通信使用 `communication/serial_port.h` 统一封装
- MAVLink 消息处理统一使用 `communication/mavlink_handler.h`
- **头文件保护统一使用 `#pragma once`**；已有头文件（如 `topic.h`、
  `logger.h`、`video_frame.h`）沿用原 include guard，不再改动，新头文件
  一律使用 `#pragma once`
- **行尾统一为 LF**，由根 `.gitattributes` 强制（`text=auto eol=lf`）：Windows 侧
  编辑器会把文件改写成 CRLF，产生大量“忽略空白后 diff 为空”的假改动并淹没真实改动。
  禁止用编辑器批量转换行尾；`third_party/` 必须保持上游原始行尾不变。
  判断文件是否真的改动以 `git diff` 是否为空为准，不看 `git status`（见 `.gitattributes` 内排查提示）。

### 命名规范
- 类名：大驼峰，如 `SerialPort`、`YoloDetector`
- 函数名：大驼峰，如 `SendCommand()`、`GetStatus()`
- 变量名：小写下划线，如 `target_position`、`is_armed`
- 成员变量：尾部加下划线，如 `port_`、`running_`
- 常量：k 前缀 + 大驼峰，如 `kMaxRetryCount`、`kDefaultBaudRate`
- 文件名：小写下划线，如 `serial_port.h`、`yolo_detector.cpp`
- 宏：全大写下划线，如 `MAVLINK_COMM_NUM_BUFFERS`

### 错误处理
- 构造函数和初始化：抛异常，让上层决定如何处理
- 运行时错误：使用 `std::optional` 或错误码，避免异常影响实时性
- 线程内错误：通过日志记录，关键错误通过消息队列通知控制线程统一决策
- 资源申请：一律使用 RAII，杜绝裸指针管理资源

### 日志规范
- **只打关键路径日志**：模块生命周期（创建/销毁）、错误、状态变化；
  高频热路径（每帧/每条消息调用的代码）不打日志
- **异常日志必须节流**：第 1 次与每满 100 次才打印（`ShouldLogThrottled`
  模式），防止高频异常刷屏；节流消息中带累计计数便于监控趋势
- **等级分层**：INFO = 生命周期与状态变化；WARN = 异常降级（丢帧、队列满）；
  ERROR = 逻辑缺陷与参数错误
- 同一异常只在最合适的一层记录一次，调用链上不重复打印
- 日志消息携带关键上下文（计数、容量、对象标识），便于事后定位
- 日志纪律是硬约束：宁可少打，不可刷屏

### 线程安全约定
- 每个线程的数据默认是线程独占的，不共享
- 跨线程数据传递使用消息队列，队列为线程安全实现
- 内存池为多生产者单消费者模型，通过原子变量和无锁队列同步
- 需要共享的状态（如系统当前状态）使用 `std::atomic` 或读写锁保护
- 锁的粒度尽量小，避免在持有锁时做耗时操作

### 测试规范
- 各模块独立可测，依赖通过接口注入
- 单元测试使用 Google Test 框架
- 测试文件放在 `tests/` 目录，与 `src/` 结构对应
- 硬件相关的模块测试使用 mock 替代真实设备

### 实现文档规范（强制规则）
- **文档入口**：`docs/README.md` 是唯一入口，包含文档地图、单一事实来源和
  「改代码时至少更新哪些文档」矩阵；当前状态看 `docs/开发进度.md`，历史时间线看
  `docs/开发历史.md`。不要在两处重复维护同一状态。
- **每个功能模块实现完成后，必须编写对应的实现文档**，供查看、后期排查与修改参考。
- **文档写在对应功能实现的文件夹内**，与头文件同级（如 `include/video/video_decoder.md`、
  `include/common/logger.md`），文件名与功能对应、使用中文名；沿用 `include/common/topic.md` 先例。
- 文档必须包含以下内容：
  - **功能职责**：该模块做什么、不做什么（边界）。
  - **接口与数据流**：类/接口签名、输入输出 Topic、消息类型。
  - **关键实现点**：算法/协议要点、时序与线程模型、资源管理、与设计的取舍（如为何用软解回退）。
  - **日志行为**：哪些关键路径打什么等级日志、节流规则，便于事后排查。
  - **测试方式**：如何构建、运行相关测试、期望结果；硬件相关部分说明在何处验证。
  - **排查/修改要点**：常见问题现象与定位方法、修改时需要注意的关联点。
- 修改代码时必须同步更新对应实现文档；新增功能同样在实现时补文档，不得只写代码不留文档。
- 基础设施已有文档的（如 `include/common/topic.md`），实现文档中引用即可，不重复抄录。
- 提交前必须运行：

  ```bash
  python3 tools/check_docs.py
  python3 tools/check_docs.py --changed
  ```

  前者校验基线数字、当前文档相对链接和模块实现文档是否存在；后者按 git 改动列出
  应核对的文档。测试数量变化时，先改 `docs/README.md` 的「当前基线」，再同步脚本列出的
  9 处位置。
- 改动前先看 `docs/README.md` §4「改代码时至少更新哪些文档」；改动后按性质更新：
  行为/门禁变化 → `docs/开发进度.md`；里程碑 → `docs/开发历史.md` 顶部追加；
  消息/Topic 变化 → `docs/数据接口文档.md`；故障现象 → `docs/故障排查手册.md`；
  配置键 → `include/config/配置管理.md` + `docs/部署与打包.md`。
- 历史/过时文档移入 `docs/archive/`（在 `docs/archive/README.md` 登记），不要留在当前
  文档入口；当前维护文档不得指向不存在的路径。

### C++ 设计规范
- 遵守 C++17 标准，优先使用现代 C++ 特性
- 遵循 SOLID 原则，注意解耦设计
- 模块间通过接口（抽象类）交互，减少直接依赖
- 使用 RAII 管理资源，避免裸指针
- 日志输出统一使用 spdlog
- 代码编写需要添加注释，关键逻辑必须注释说明
- 编写代码前先调研是否有现成的库或框架可用，优先复用而非自行实现；仅在库过于复杂或对项目帮助不大时才自己实现。选择库时考量：license 兼容性、ARM64 交叉编译支持、依赖链复杂度、社区活跃度。若决定自研，封装为独立模块并对外暴露接口，方便后续替换为成熟库。

### Git 管理
- 使用 git 管理工程，提交前确保代码可编译
- 提交信息简洁明了，描述本次改动内容
- 功能开发在分支上进行，完成后合并到 main

### AI 协作规则
- 遇到不确定的问题时，不要猜测，必须询问用户确认后再执行
- 回答应详细且逻辑清楚，不能为了简短而省略关键信息
- 修改代码或文档时，先全局搜索相关引用，确保所有关联文件同步更新，避免遗漏
