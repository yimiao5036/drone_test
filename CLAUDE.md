# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

运行在香橙派 RK3588（ARM64, Ubuntu）上的反无人机机载 C++17 控制系统：接收自研地面站 MAVLink 目标位置 → YOLOv26 RKNN 目标识别 → 轨迹预测 → 控制 PX4 跟踪/拦截。另见 `AGENTS.md`（完整开发规范，含强制规则）。

**当前安全基线**：`config/config.json` 必须保持 `"enable_control": false`，正式程序不向 PX4 发送飞行控制；`interception.enabled` 默认保持 `false`（S9/S10 入口配置化后，生产不自动触发发射），返航/降落链只在 SITL 验收配置中显式启用。`MissionStateMachine`、`TargetEstimator`、`VisualTargetMonitor` 均为影子运行（跑真实算法但不产生控制输出）。**PX4 已到位**（2026-10-10：装机体 + GPS，可出 Home），依赖 PX4 的实链验证由此可开展；但安全基线不变——阶段四控制接入路径代码已全部落地（2026-10-14，Task 1~12：装配门禁 + 状态机 S5~S7 交接链 + 捕网执行器命令 + G2/G3 的 `px4_link_smoke` stage4 工具；`enable_control=true` 才绑定 `Px4Setpoint` 并注入 PX4 快照源/命令接口，false 时影子停发），正式控制输出要等 SITL/台架门禁（G2~G4，SITL 实跑为线下执行项；G1~G3 已通过、G4 串口台架自动部分 2026-10-21 已通过，G4-1 电机怠速已人工确认，G4-2 实际方向待判读、G4-3 捕网暂缓）验证通过后，才允许在现场本地副本打开 `enable_control`。

## 构建与测试（唯一构建环境：WSL2 Ubuntu 24.04）

所有本地编译与测试必须在 Windows 主机的 **WSL2 Ubuntu 24.04** 中进行（Ubuntu 20.04 禁止使用）。提交前必须编译通过。

```bash
# 主工程
cmake -S . -B build
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure     # 基线 584/584
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

工具：`px4_link_smoke`（PX4 串口冒烟/SITL 分级测试；G4 台架 `--bench-stage4-controller --bench-ack-props-off`，仅串口、必须拆桨）、`video_latency_probe`（视频链路延迟探针）、`tools/*.py`（地面站 TIMESYNC/目标发送测试）、`test.py`（pymavlink SITL 飞行流程模拟）。

## 架构要点

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

## 项目规范（违反会被打回）

- **语言**：沟通、注释、提交说明、文档统一中文；标识符（类/函数/变量名）、命令行、第三方库名保留英文。
- **实现文档强制**：每个功能模块必须在头文件同级目录维护中文实现文档（如 `include/video/video_decoder.md`），含功能边界、接口与数据流、关键实现、日志行为、测试方式、排查要点；改代码必须同步改文档。
- **命名**：类/函数大驼峰（`SerialPort`、`SendCommand()`），变量小写下划线，成员变量尾下划线（`running_`），常量 k 前缀（`kMaxRetryCount`），文件小写下划线。新头文件用 `#pragma once`。
- **日志纪律（硬约束）**：只打生命周期/错误/状态变化；每帧、每条消息的热路径禁止打日志；异常日志节流（第 1 次与每满 100 次）。INFO=生命周期，WARN=降级，ERROR=逻辑缺陷。
- **错误处理**：构造/初始化抛异常；运行时用 `std::optional` 或错误码；资源一律 RAII。
- **行尾**：`.gitattributes` 强制 LF；禁止批量转换行尾；`third_party/` 保持上游原样不动。WSL2 的 `/mnt/*` 上 `git status` 会误报 modified，**以 `git diff` 是否为空为准**。
- **不得猜测**：仓库无依据的信息先查官方资料或询问维护者，不要编造硬件参数、协议字段或阈值。
- 分支开发：功能在分支（如 `biao_dev`）开发后合并 main。

## 目录边界

| 目录 | 性质 |
|---|---|
| `src/`、`include/`、`main.cpp` | 正式工程（镜像结构，模块一一对应） |
| `tests/` | Google Test，结构对应 `src/`；`tests/skeleton/` 为骨架冒烟 |
| `target_tracker/`、`visual_track_control/` | 独立子工程（自有 CMakeLists），未接入主应用，保持独立构建 |
| `videoPart/` | 视频/RKNN 历史验证原型，不进正式装配 |
| `yolo_pt/` | 模型训练与 pt→RKNN 导出（Python） |
| `tools/` | 冒烟/探针/地面站测试工具 + `web平台/`（Vue3 开发版地面站） |
| `docs/` | 当前文档入口 + `archive/`/`superpowers/` 历史材料（中文文件名） |

## 维护文档入口（排查/改动前先读）

1. `docs/README.md` — **文档唯一入口**：文档地图、单一事实来源、更新矩阵、基线位置
2. `docs/项目总览与数据流.md` — 正式/影子/Stub 边界、全部数据流、线程模型
3. `docs/模块索引.md` — 按类名/Topic/配置/测试定位代码
4. `docs/故障排查手册.md` — 按现象定位（无画面、无检测、无 ACK 等）
5. `docs/新增设备接入指南.md` — 替换/新增硬件的固定步骤
6. `docs/开发进度.md` — 当前状态/门禁/最近变更；`docs/开发历史.md` — 历史时间线
7. 各模块头文件同级的实现文档

## 文档更新（提交前强制）

- 改代码前先看 `docs/README.md` §4「改代码时至少更新哪些文档」；
- 改完运行：

  ```bash
  python3 tools/check_docs.py
  python3 tools/check_docs.py --changed
  ```

- `check_docs.py` 会校验：`docs/README.md` 当前基线、10 处基线位置、当前文档相对链接、
  `docs/模块索引.md` 中的实现文档是否存在；
- 行为/门禁变化更新 `docs/开发进度.md`；里程碑在 `docs/开发历史.md` 顶部追加一条；
- 消息/Topic 变更新 `docs/数据接口文档.md`；故障现象变更新 `docs/故障排查手册.md`；
- 测试数量变化：先改 `docs/README.md`「当前基线」一行，再同步 10 处基线位置；
- 历史文档只进 `docs/archive/` 或 `docs/superpowers/`，当前维护文档不得指向不存在的路径。
