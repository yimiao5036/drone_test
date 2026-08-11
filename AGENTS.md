# AGENTS.md

本项目是一个运行在香橙派算力板上的**反无人机 C++ 控制系统**。

## 项目概述

- **运行平台**：香橙派（Orange Pi），RK3588 芯片，Ubuntu 系统
- **语言与标准**：C++17
- **构建系统**：CMake
- **日志库**：spdlog
- **主要功能**：接收地面站目标位置 → YOLOv26 目标识别 → 轨迹预测 → 控制 PX4 无人机跟踪/反制
- **通信协议**：MAVLink（与 PX4 飞控和地面站通信）

## 开发与构建环境（强制规则）

- **运行目标**：代码最终在香橙派（RK3588，ARM64，Ubuntu）上编译、部署和运行。
- **开发机唯一构建环境**：Windows 主机上的 WSL2 中的 **Ubuntu 24.04**（工具链与依赖已配置好）。
  所有本地编译验证、单元测试和提交前检查一律在 Ubuntu 24.04 中进行。
- WSL2 中的 Ubuntu 20.04 **不用于**本项目构建，避免环境不一致导致问题。
- 在 WSL2 Ubuntu 24.04 中的标准构建流程：

  ```bash
  cmake -S . -B build
  cmake --build build -j$(nproc)
  ./build/drone_control        # 生成 logs/0001_xxx.log 日志
  ```

- 提交代码前必须保证在 WSL2 Ubuntu 24.04 中编译通过；香橙派上执行最终部署验证。

## 架构设计

采用**线程 + 消息队列**的解耦架构（借鉴 PX4/ROS2 设计思想）：

- 每条物理链路一个独立线程，互不阻塞
- 图像数据通过内存池零拷贝传递
- 线程间通过消息队列或通知机制通信
- 各模块可独立编译、独立测试

### 7 个线程

| 线程 | 职责 |
|------|------|
| 摄像头采集线程 | 网口采集原始画面 → 内存池 A |
| YOLO 识别线程 | 从内存池 A 取画面识别目标 → 内存池 B |
| 图传发送线程 | 从内存池 B 取图像 → 网口发送给图传 |
| PX4 串口通信线程 | 与 PX4 飞控 MAVLink 双向通信 |
| 电台数传线程 | 与地面站 MAVLink 双向通信 |
| 激光测距雷达线程 | 串口读取距离数据 |
| 状态机与控制线程 | 消费各线程数据，轨迹预测、状态机决策、生成控制指令 |

## 目录结构

```
drone_test/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── communication/       # serial_port.h, mavlink_handler.h, radio_link.h
│   ├── perception/          # camera.h, yolo_detector.h
│   ├── control/             # trajectory_predictor.h, px4_controller.h
│   ├── state_machine/       # state_machine.h
│   ├── video_transmission/  # video_sender.h
│   └── common/              # message_queue.h, memory_pool.h, types.h
│   └── config/              # config.h（配置文件解析与参数管理）
├── src/                     # 对应的 .cpp 实现文件 + main.cpp
├── tests/                   # 单元测试，与 src/ 结构对应
├── config/                  # 配置文件（JSON/YAML）
└── third_party/
    ├── mavlink/             # MAVLink 库
    └── spdlog/              # spdlog 日志库
```

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
