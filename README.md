# drone_test

运行在香橙派 RK3588（ARM64 / Ubuntu）上的反无人机机载 C++17 控制系统：
地面站目标位置 → YOLO/RKNN 识别 → 轨迹估计 → 控制 PX4 接近/视觉跟踪/拦截 → 返航降落。

> **当前状态与门禁**：见 [`docs/开发进度.md`](docs/开发进度.md)
> **文档入口与更新规范**：见 [`docs/README.md`](docs/README.md)

## 安全基线

- **运行模式**：`config/config.json` 的 `runtime.enable_control=true`（2026-10-30 起为仓库默认，
  现场实测期）＝**控制运行**；改成 `false` 重启即 **只读运行**（只采日志、不下发控制）。
  定义见 [`docs/运行模式与模块状态定义.md`](docs/运行模式与模块状态定义.md)。
- **估计器降级处置**（2026-11-17 P3）：`EstimatorGuard` 判据命中即**锁存** → 切 `POSCTL(3/0)`、
  停发一切控制意图、冻结任务推进（不自动返航/降落/上锁），恢复只能「重置任务状态」或重启；
  `estimator_guard.enabled=false` 是唯一回滚开关。见
  [`docs/降落与估计器保护设计-2026-11-17.md`](docs/降落与估计器保护设计-2026-11-17.md) §5.6。
- **重置任务状态会重启飞控**（2026-11-17 P4b）：65020 `type=2` 重置 = 清任务状态 + 重启 PX4
  （发 `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(246)`，飞控约 20~30 s 不可用；S1 SELF_CHECK 等飞控
  回来，超时 65013 bit10）；`mission.reset_reboot_px4=false` 回滚成"只清状态回 S2"。见同一设计文档 §6.7。
- 两个发射闸门仍关：`interception.enabled=false`（生产不自动触发捕网）、
  `actuator.gcs_fire_enabled=false`（65016 直控不装配）。
- `TargetEstimator`、`VisualTargetMonitor`、`StereoRanger` 是**观测**模块（跑真实算法、
  不产生控制输出）；`VisualTrackingShadow` 的输出 Topic 无消费者、永不接控制器。
- 功能代码**不得**按 `enable_control` 分支，只按可观测能力判断（命令接口是否注入等）。

## 快速构建与测试

唯一构建环境：WSL2 Ubuntu 24.04。

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure     # 792/792
python3 tools/check_docs.py                    # 文档基线/链接检查
./build/drone_control                          # 生成 logs/0001_xxx.log
```

CMake 会把 `config/` 和 `models/` 同步到构建目录。配置查找顺序：
`--config` 参数 → 可执行文件旁 `config/config.json` → `./config/config.json`。

## 文档入口

先看 [`docs/README.md`](docs/README.md)。日常维护最常用的几份：

- [`docs/项目总览与数据流.md`](docs/项目总览与数据流.md)：模块状态、数据流、线程、Topic
- [`docs/模块索引.md`](docs/模块索引.md)：按类名/Topic/配置/测试定位代码
- [`docs/故障排查手册.md`](docs/故障排查手册.md)：按现象定位
- [`docs/新增设备接入指南.md`](docs/新增设备接入指南.md)：替换/新增硬件
- [`docs/数据接口文档.md`](docs/数据接口文档.md)：消息字段、Topic、接口契约
- [`docs/部署与打包.md`](docs/部署与打包.md)：香橙派部署与自检
- [`docs/开发进度.md`](docs/开发进度.md)：当前状态、门禁、最近变更
- [`docs/开发历史.md`](docs/开发历史.md)：历史时间线（只读）

历史设计与性能记录在 [`docs/archive/`](docs/archive/README.md) 与
[`docs/superpowers/`](docs/superpowers/)；各模块实现文档放在 `include/` 对应目录下。

## 系统目标与主流程

系统接收地面站持续下发的目标无人机位置，先按 GPS 位置引导接近；视觉稳定锁定后
切换为速度/方向跟踪；满足拦截条件后触发执行机构（V1 单发、不判命中），随后返回 Home、
降落并上锁。

```text
地面站目标位置 → GPS 位置引导 → 视觉稳定锁定
  → 速度/方向跟踪 → 拦截 → 算力板控制返回 Home → AUTO.LAND → DISARM
```

## 已确认硬件与软件

| 项目 | 当前方案 |
|---|---|
| 算力板 | Orange Pi，RK3588，Ubuntu |
| 飞控 | Pixhawk 2.4.8，PX4 1.17.0 |
| 算力板与飞控 | `/dev/ttyS1` 串口 MAVLink |
| 地面站 | 自研，HM30 电台，`/dev/ttyS6` |
| 摄像头 | USB 双目 SBS MJPG（UVC）；左目用于 YOLO/图传，右目用于稀疏 Census 测距（观测） |
| 视觉算法 | YOLOv26 RKNN，生产输入 384×640 |
| 测距 | 双目稀疏 Census v2（观测）；室外目标 <8m 可用，卷尺级精度待验收 |
| 导航 | PX4 融合 GPS/IMU/磁力计/气压计；算力板无独立 GPS |
| Home/锚点 | 目标估计以 EKF 锚点（`GPS_GLOBAL_ORIGIN` #49）为原点；返航使用 PX4 Home |
| 捕网枪 | PX4 MAIN5，Output Function=`Peripheral via Actuator Set 1`；机载 `net_channel=1` |
| 供电 | 香橙派 5V/4A 与 PX4 共用动力电池；电量统一读 PX4 电池状态 |

## V1 能力边界

- 地面站目标：编号、经纬度、高度、速度、航向、精度、时间戳、有效期；
- 未稳定识别时按目标位置做 GPS 位置引导；视觉稳定后切换速度/方向控制；
- 视觉短暂丢失可由估计器有限预测，完整丢失策略保留为可替换接口；
- 双目深度无效时必须显式降级，不能伪造距离；
- V1 遇障只减速/刹停/悬停，不承诺绕行（S8 暂缓）；
- V1 单发拦截、不判命中，结束后默认返航；不自动执行第二次任务；
- 正常返航不用 PX4 自动 RTL；算力板/程序/串口完全失效时由 PX4 自身失联保护兜底；
- 遥控器人工接管优先。

## 目录边界

| 目录 | 性质 |
|---|---|
| `src/`、`include/`、`main.cpp` | 正式工程（镜像结构，模块一一对应） |
| `tests/` | Google Test，结构与 `src/` 对应 |
| `config/`、`models/` | 生产配置/标定与 RKNN 模型 |
| `tools/` | 探针、PX4 冒烟/SITL 分级测试、**SITL 验收装置 `sitl_harness.sh`**、标定、Web 地面站 |
| `docs/` | 当前文档入口 + `archive/` + `superpowers/` 历史材料 |
| `target_tracker/`、`visual_track_control/`、`stereo_depth/` | 独立子工程，不接入主应用 |
| `videoPart/`、`yolo_pt/` | 历史验证原型 / 模型训练导出 |

## 构建环境

- 运行目标：Ubuntu / ARM64 / RK3588；
- 开发/构建：Windows + WSL2 **Ubuntu 24.04**（唯一构建环境）；
- C++17、CMake、spdlog、Google Test、FFmpeg（香橙派需 ffmpeg-rockchip、librknnrt、librga）；
- 提交前在 WSL2 中编译通过，并运行 `ctest` 与 `python3 tools/check_docs.py`。
