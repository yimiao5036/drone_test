# PX4 实链验证清单（真机，全程只读）

> 用途：PX4 已到位（2026-10-10：装机体 + GPS，可出 Home）后的实链验证，逐条打勾并记录日志。
> 定位：主任务《主程序集成与数据链路实施计划》阶段四的前置验证；§7 收录阶段四
> 控制接入门禁（G2/G3 已实跑通过；G4 串口台架入口 2026-10-20 已落地、待实跑；
> G4/G5 线下清单）与 `enable_control` 打开前置清单。
> **全程只读**：不绑定 `Px4Setpoint`、不发 ARM/模式切换/起飞/降落，`enable_control` 保持 false。
> **现场条件：室内（2026-10-10）与室外（2026-10-15）只读验证均已完成；2026-10-19
> 地面会话（0041/0043）完成方案 C 板上核对**——Home 漂移仅诊断、锚点 `收1` 恒定、
> 目标链 0041 1→1800 / 0043 1→700（后端 5Hz 持续发送）；GS 心跳/时间同步在 0041
> 两次中断、0043 窗口未复现（见 §6、§4）。
> 室内可能拿不到 GPS fix / Home，见 §4：**室内无 Home 不是通信问题**，不要按串口故障排查。

## 0. 前置检查（上电前，一次性）

| # | 项 | 命令/期望 | 结论 |
|---|---|---|---|
| 1 | 接线：香橙派 UART6 ↔ PX4 TELEM，**`uart6-m1`**（与排针一致） | 见 `docs/部署与打包.md` §9.2 | |
| 2 | 权限：当前用户在 `dialout` 组 | `groups \| grep dialout` | |
| 3 | 串口未被系统控制台占用 | `sudo lsof /dev/ttyS1`；`systemctl status serial-getty@ttyS1` | |
| 4 | 配置：`px4.serial.device=/dev/ttyS1`、115200、`firmware_version` 与实际一致 | `config/config.json` | |
| 5 | 硬件安全：**拆螺旋桨**、TTL 电平兼容、共地 | 见 `tools/px4_link_smoke.md` §4.3 | |
| 6 | PX4 上电（USB 或电池），GPS 天线接好 | PX4 指示灯正常 | |

## 1. 第 1 步：纯链路冒烟（独立于主程序，推荐先做）

```bash
./build/px4_link_smoke --duration 30
echo "退出码=$?"
```

判读（完整列表见 `tools/px4_link_smoke.md` §5）：

| 项 | 期望 |
|---|---|
| `[PASS] PX4 HEARTBEAT` | 心跳建立 |
| `[PASS] 测试结束时心跳仍连接` | 链路稳定 |
| `[PASS] 遥测请求 COMMAND_ACK` | 主动请求被接受 |
| `[PASS] AUTOPILOT_VERSION 收到` | 固件版本可读 |
| 其他遥测（ATTITUDE 等） | 缺项只记 WARN，不判基础链路失败 |

不通过 → `tools/px4_link_smoke.md` §7 排查要点 + `docs/故障排查手册.md` 第 10 节。

## 2. 第 2 步：主程序实链（只读，逐条对照启动日志与摘要）

```bash
./build/drone_control
```

| # | 项 | 期望日志/现象 | 室内可达 |
|---|---|---|---|
| 1 | 串口打开 | `串口打开成功: device=/dev/ttyS1 baud=115200` | ✅ |
| 2 | 链路启动 | `PX4 通信启动: transport=serial:/dev/ttyS1@115200` | ✅ |
| 3 | 心跳 | `PX4 心跳已连接: source=1/1 mode=… armed=…` | ✅ |
| 4 | 固件版本 | `PX4 固件版本上报: reported=1.17.0`（与配置一致） | ✅ |
| 5 | 遥测快照 | 地面站下行有真实姿态/电池；`FlightStateSnapshot` 计数增长 | ✅（GPS 项除外） |
| 6 | 健康 | 无 `健康数据源超时: px4` | ✅ |
| 7 | 状态机 | `SELF_CHECK → READY`（需 GPS fix + Home 有效） | ⛔ 需室外 |
| 8 | TargetEstimator | `UpdateCount()` 增长、出有效 NED 影子状态（需 Home） | ⛔ 需室外 |
| 9 | 安全 | 日志无 ARM/模式/setpoint；`PX4 命令 ACK` 全部 `internal=true` | ✅ 必查 |

## 3. 安全红线（每次运行都确认）

- 螺旋桨已拆；
- 程序可见的主动内容只有 HEARTBEAT 与 `REQUEST_MESSAGE`/`SET_MESSAGE_INTERVAL`
  （日志中不应出现任何 setpoint、ARM、模式切换）；
- 异常时先 Ctrl+C（逆序停机），再按 §5 处置；不要在带桨状态下反复试。

## 4. 室内 / 室外判定（关键：别把无 Home 误判成通信问题）

```text
先看这一行是否出现：
  PX4 心跳已连接: source=1/1 …
  ├─ 出现 ⇒ 串口与 MAVLink 通信正常（链路层已通）
  │        若此时状态机仍停 SELF_CHECK / TargetEstimator 一直等待 EKF 锚点：
  │        → 看 GPS fix 与锚点流 #49（室内属预期，不是通信问题）
  └─ 不出现 ⇒ 才是通信问题 → 故障排查手册第 10 节
```

| 现象 | 结论 | 动作 |
|---|---|---|
| 有心跳、无 GPS fix/Home（室内） | **预期** | 记录"室内限制"，Home 相关项留室外 |
| 有心跳、有 GPS fix、Home 不成立 | 查 GPS 质量/解锁状态/Home 设置条件 | 地面站核对 GPS 状态 |
| 无心跳 | 通信问题 | 接线/波特率/PX4 端口（`px4_link_smoke` §7） |
| 心跳断续 | 干扰/线缆/供电 | 查时序与线缆 |

**室外补齐项（2026-10-15 完成 ✅）**：`SELF_CHECK → READY`（15:01:42 两会话独立复现）、
`TargetEstimator` 出 NED 影子状态（目标 UPDATE → 估计器接受观测 → `READY→GPS_APPROACH`）、
地面站显示 Home —— 结论已回填 §6 表与 `docs/开发进度.md`。

**室外观察要点（2026-10-13 增补；2026-10-15 室外实测；2026-10-19 方案 C 核对）**：

- **Home 漂移对照（QGC vs 机载日志）**：✅ **坐实为 PX4/GPS 侧行为**。2026-10-15 室外
  实测机载日志（gps_fix 3/DGPS、卫星 21~29、eph 0.58~0.87m 良好收星）下 Home 持续变化：
  楼旁 12 分钟水平包络 ~20m×5m、高度包络 22m（230.8~253.2m 平滑往复）；换开阔高点后
  依旧（2 分钟 13m 高度包络、8×9m 水平）。**维护者现场确认 QGC 同源数据亦同步漂移**。
  结论：PX4 disarmed 期间持续以当前位置刷新 Home + GNSS/EKF 静止游走的组合行为，
  机载代码无责。**方案 B（迟滞阈值）裁决输入**：游走为米级（10~20m 包络），迟滞阈值
  只能滤厘米级微调、对此无效；可选方向为重置换基准不清滤波状态（机载侧）或 armed
  后冻结 Home 的窗口化估计（待维护者裁决）。驻留期每 2s 一次 Home 变化使目标估计器
  反复重置，速度估计无法累积——**重置换基准不清状态的方向优先级最高**。
- **高度通道收敛性**：❌ 室外稳定 fix 下未收敛（平滑大幅往复，形态偏气压计漂移 +
  GNSS 高度解噪声混合），影响目标估计 D 轴基准，与 Home 漂移一并归入上述裁决。
- **25Hz 姿态/局部位置链路余量**：✅ 实测无 `健康数据源超时: px4`、无丢包告警
  （ground_station 一次 13s 超时为切换位置时地面站设备重启，属预期）。
- **Web 地图修复回归**：⏳ 坐标链路已验证（(0,0) 修复生效，风机在真实位置）；
  发现地图初始视野硬编码上海街道级导致异地"看不到飞机点"，已修复为
  中国全境默认 + 首次收坐标自动适配（web 平台提交 `12af944`），待下次现场复核。
- **2026-10-19 地面会话（0041，方案 C 核对）**：Home 漂移持续（4 分钟水平约 16m、
  高度约 6m），锚点 15:59:05 到达后 `收1` 恒定；无 Home 触发的估计器重置，目标
  NED 在 1→1800 条观测中稳定。16:02:26 时间同步 `SYNCHRONIZED→DEGRADED`、16:02:45
  心跳超时（3000ms）各触发一次估计器 `previous_state_expired` 重置，均自动恢复
  → **GS 心跳/时间同步问题再次复现**；GS 侧心跳诊断已加入（`GS_HB_DIAG_LOG` /
  `/api/health`），待下次实链定位。本会话 YOLO 无检测，
  双目未进入测距，10-15 的 0% 有效问题未复现。
- **2026-10-19 地面会话（0043，后端持续发送复核）**：锚点 16:39:22 首次到达后
  `收1` 恒定，Home 继续漂移但仅诊断；后端持续发送名义 5Hz，目标 UPDATE 1→700
  （每 100 条约 20.6~21.5s，实测 ≈4.8Hz），**无目标过期、无估计器重置**；本窗口
  GS 心跳/时间同步正常（无超时）。双目仍未检测到目标（累计观测 133），未进入测距。

## 5. 异常处置（指向，不在此重复）

| 现象 | 去处 |
|---|---|
| 串口打不开/被占用 | `docs/故障排查手册.md` 第 10.1/10.2 节 |
| 打开但无心跳 | 同上 10.3 + `tools/px4_link_smoke.md` §7 |
| 心跳超时告警 | `docs/故障排查手册.md` 第 14 节 |
| 状态机停留自检 | `docs/故障排查手册.md` 第 13 节 + §4 判定表 |
| 目标估计等待 EKF 锚点 | `docs/故障排查手册.md` 第 12 节 |

## 6. 结果记录（回填用）

| 日期 | 环境 | 冒烟 | 串口 | 心跳 | 固件 | 遥测 | 状态机 | NED 影子 | 备注 |
|---|---|---|---|---|---|---|---|---|---|
| 2026-10-10 | 室内 | 未跑（直接跑主程序） | ✅ | ✅ | ✅ | ✅ | ⛔ 待室外 | ⛔ 待室外 | 日志 `0028_20261010_155340.log`；心跳 `source=1/1 mode=4/3 armed=false`；一条 512 请求被拒（result=2）——与 AUTOPILOT_VERSION 成功对照，应为 HOME_POSITION(242) 因无 Home 被拒，属室内预期 |
| 2026-10-15 | 室外（楼旁/开阔高点） | 未跑（直接跑主程序） | ✅ | ✅ | ✅ | ✅ | ✅ `SELF_CHECK→READY`（15:01:42，两会话独立复现） | ✅ 目标 UPDATE（ACK ACCEPTED rtt 97ms）→ 估计器接受观测出 NED（N=-49.11 E=19.85 D=150.18）→ `READY→GPS_APPROACH` | 日志 `0034_20261015_144015.log` 及 15:00 会话；GPS 3D fix→3D DGPS，卫星 21~29 颗 eph 0.58~0.87m；**Home 漂移对照达成**（QGC 同源亦游走，坐实 PX4/GNSS 侧行为，详见 §4）；双目测距有效占比仍 0%（待排查，按 11.1）；ground_station 心跳一次 13s 超时=设备重启属预期；TIMESYNC rtt 77~126ms |
| 2026-10-19 | 地面（GPS 3D/DGPS） | 未跑（直接跑主程序） | ✅ | ✅ | ✅ | ✅ | ✅ `SELF_CHECK→READY`（15:59:03） | ✅ 目标 UPDATE→估计器 NED（N=-22.70 E=-6.70 D=-36.98，1→1800 条）→ `READY→GPS_APPROACH` | 日志 `0041_20261019_155833.log`；**方案 C 板上核对通过**：Home 漂移仅诊断、锚点 `收1` 恒定、无 Home 重置；16:02:26 时间同步 DEGRADED、16:02:45 心跳超时各触发一次估计器重置后自动恢复；双目本会话无检测（0% 不可判读）；安全：无 ARM/模式/setpoint |
| 2026-10-19（0043） | 地面（GPS 3D/DGPS） | 未跑（直接跑主程序） | ✅ | ✅ | ✅ | ✅ | ✅ `SELF_CHECK→READY`（16:39:22） | ✅ 目标 UPDATE→估计器 NED（N=-32.03 E=2.67 D=-19.70，1→700 条）→ `GPS_APPROACH` | 日志 `0043_20261019_163836.log`；后端持续发送生效：名义 5Hz，实测 ≈4.8Hz（每 100 条约 20.6~21.5s），无目标过期/估计器重置；锚点 `收1` 恒定、Home 漂移仅诊断；本窗口 GS 心跳正常；双目因无检测未测；安全：无控制输出 |

## 7. 阶段四控制接入门禁（2026-10-14 建立）

本节收录阶段四控制接入路径（设计稿
`docs/superpowers/specs/2026-10-13-阶段四控制接入路径-design.md`，实现 T1~T12）
的门禁执行清单。**SITL 门禁（G1~G3）与实机分离**：不接真实 PX4，UDP 连 PX4 SITL。

### 7.1 SITL 门禁实跑（线下执行项）

| 门禁 | 命令（构建目录内） | 通过标准 | 状态 |
|---|---|---|---|
| G1 回归 | 第 0~3D 分阶段原样复跑（`--sitl-zero-velocity` … `--sitl-offboard-loss`） | 全部 PASS（退出码 0） | ✅ 2026-10-14（WSL2，七级全 PASS） |
| G2 执行层 | `./px4_link_smoke --config config/px4_sitl.json --duration 90 --sitl-stage4-controller` | 报告全 PASS（零速漂移/阶跃斜率/停发刹车/退出时延/降落上锁） | ✅ 2026-10-14（WSL2，全 PASS；实测值回填 `tools/PX4_SITL分级测试.md` §12.1） |
| G3 全链 | `./px4_link_smoke --config config/px4_sitl.json --duration 120 --sitl-stage4-mission` | 报告全 PASS（迁移序列 S5→S6→S7→回退、交接连续、备份接管、围栏钳制、模式停发 ≤200ms） | ✅ 2026-10-14（WSL2，全 PASS；环境前置条件见 `tools/px4_link_smoke.md` §2.2） |
| P3 前馈专项（非门禁） | `./px4_link_smoke --config config/px4_sitl.json --duration 90 --sitl-stage4-feedforward [--feedforward]` | 前馈到达 PX4、S5 稳态滞后 A/B、包线/回收链安全 | ✅ 2026-10-19（WSL2）：FF off 滞后均值 1.391m / p95 1.492m；FF on 0.321m / 0.400m，p95 降低 73.2%；G3 以 FF=false 同步复跑全 PASS |
| S11~S14 返航专项（非门禁） | `./px4_link_smoke --config config/px4_sitl_return.json --duration 120 --sitl-stage4-return` | S9/S10 入口→发射→S12 返航 Home→S13 AUTO.LAND→S14 DISARM；包线/回收链安全 | ✅ 2026-10-19（WSL2 全 PASS）：状态机迁移 11 次/错误 0；最大相对高度 4.99m、北向 28.93m；S12 起始射程约 11.6m；证据 `tools/return_evidence/stage4_return_20261019.csv` |

SITL 门禁运行环境（2026-10-14 建立）：本机 WSL2 Ubuntu 24.04（`~/PX4-Autopilot`
v1.17.0 + Gazebo Harmonic 8.15 + `~/drone_test` 构建）；启动命令与前置条件见
`tools/px4_link_smoke.md` §2.2 末节。仿真机（192.168.31.209，v1.18-beta 主线 +
老 CPU 达不到实时）已弃用——EKF2 卡假位置/无航向参考系算力不足所致。

判读与排查见 `tools/px4_link_smoke.md` §2.1/§2.2/§7；失败处置纪律：G2 起任一级出现
非预期位移、非预期 armed、PX4 failsafe 触发，按缺陷处理（systematic-debugging），
禁止调参掩盖。

### 7.2 `enable_control` 打开前置清单（缺一不可，设计 §6）

| # | 条件 | 证据位置 | 状态 |
|---|---|---|---|
| 1 | G0~G4 全部通过 | 本文件 §7.1/§7.3 + `docs/开发进度.md` | ⏳ |
| 2 | 双目测距卷尺级精度验收 + fxB 绝对精度对齐 | 独立测试工程结论回填 | ⏳ 室外 <8m 现场可用已验证（2026-10-20，近距有效占比 99.2%）；卷尺级/fxB 对齐仍待独立工程回填 |
| 3 | 控制律参数整定完成（非占位值：heading.gain / vertical.vz_gain / distance PID） | SITL/回放整定记录 | ✅ 2026-10-14（SITL `--sitl-stage4-tune` 五轮整定，全指标达标；终值 heading.gain=0.9、vz_gain=0.1、distance 0.5/0.05/0.1；记录与证据见 `include/control/tracking_control_law.md` §7 + `tools/tune_evidence/`；遗留：d_exp=10m 冻结后须复跑复核零过冲） |
| 4 | 相机安装外参标定（消除全 0 准直假设，`target_estimator.visual.mount_*`） | 标定参数入库 | ⏳ |
| 5 | 围栏参数按试飞场地冻结（`controller.max_radius_m/max_altitude_m`） | `config.json` 评审 | ⏳ |
| 6 | 室外实链只读验证通过（状态机越自检、NED 影子正常） | 本文件 §4/§6 回填 | ✅ 2026-10-15（SELF_CHECK→READY 两会话复现；目标 UPDATE→估计器出 NED→GPS_APPROACH；Home 漂移对照坐实 PX4/GNSS 侧行为；链路余量无超时。双目 0% 已于 2026-10-20 定位为超出 ≈7.95m 量程，<8m 近距有效占比 99.2%；卷尺级精度仍待验收。Web 地图视野修复待复核） |

> **2026-10-19 补测（0041/0043）**：方案 C 板上核对通过（Home 漂移仅诊断、锚点 `收1`
> 恒定、无 Home 重置）；0041 中 GS 心跳/时间同步两次中断复现（16:02:26 / 16:02:45），
> 均自动恢复；0043 目标流窗口（约 2.5 分钟）未再出现心跳超时（后端持续发送 5Hz 稳定）；
> GS 侧心跳诊断已加入，待更长驻留定位；双目因无检测未复现 0%。详见 §4/§6 与
> `docs/方案C锚点基准实施清单.md` §8。

回滚通道：现场改回 `enable_control=false` 重启即回纯影子；`config.json` 中的
`enable_control=true` 永不入库（打开只发生在现场部署目录本地副本）。

### 7.3 G4/G5 线下执行清单（拆桨台架 / 室外首飞）

| 序 | 项 | 环境 | 通过标准 | 状态 |
|---|---|---|---|---|
| G4-1 | ARM/Offboard 流程（真实 PX4 + 机体，卸桨） | 台架 | 模式/上锁序列与 SITL 一致，电机仅怠速级响应 | ✅ 2026-10-21（自动序列通过；现场确认电机仅怠速） |
| G4-2 | 电机输出方向（位置/速度意图小量激励） | 台架 | 方向正确、无反转 | ⏳ 四向设定值方向 ✅ 2026-10-21；电机/机体实际方向待人工判读（方法见 `tools/px4_link_smoke.md` §2.5） |
| G4-3 | 捕网 PWM 行程/方向（DO_SET_ACTUATOR 触发/复位） | 台架 | 机构到位、方向正确；核对 `actuator.*` 与 `PWM_AUX_MINn/MAXn` 映射 | ⏸ 暂缓（维护者 2026-10-21：当前重点不是捕网控制）；通道/接线/行程未冻结，本模式不自动触发 |
| G4-4 | 断链失控保护（拔串口/停 setpoint） | 台架 | 停发/上锁符合预期（`COM_OF_LOSS_T`/`COM_OBL_RC_ACT`） | ⏳ 停 setpoint 分支 ✅ 2026-10-21（COAST/IDLE、退出 Offboard 1972ms、DISARM）；拔串口分支下次测试（工具断链后停意图/关串口/刷盘并以退出码 4 退出，PX4 结果需 QGC 确认） |
| G5-1 | 室外首飞（系留/低空低速、人工兜底） | 室外 | 另行冻结清单（含场地围栏参数、遥控兜底、天气条件） | ⏳ |

**G4 串口台架入口（2026-10-20 落地）**：

```bash
# 拆桨、机体固定、遥控器可随时接管；PX4 已落地且未解锁
./build/px4_link_smoke --config config/config.json --duration 60 \
  --bench-stage4-controller --bench-ack-props-off
echo "退出码=$?"
```

- 仅允许 `serial`；`--bench-ack-props-off` 缺失时启动前拒绝；
- 预检要求 `landed=是`、`armed=否`、局部位置/姿态有效且 **RC_CHANNELS 在线**；
  RC 流缺失时超时拒绝，不能只靠命令行确认绕过；
- PX4 默认 `COM_DISARM_PRFLT=10s`（解锁后未起飞自动上锁）。台架序列已压缩到
  解锁后约 7.5s；若需更长时间人工观察方向，先在 QGC 临时调到 ≥30s，测后恢复；
- 自动流程覆盖 G4-1/2/4 的序列、ACK、四向设定值方向、停发 Offboard 退出与
  DISARM；G4-2 的电机/机体实际方向、G4-1 的电机怠速响应必须人工勾选；
- G4-3 捕网执行器**不自动触发**：`actuator.net_channel`、接线、`net_fire_pwm`/
  `net_idle_pwm` 与 `PWM_AUX_MINn/MAXn` 尚未装机确认，猜参数可能损坏机构；
- 安全：预检必须 landed+disarmed；离地硬判据为 `landed=false` 持续 1s 且本地
  上升 >0.25m；本地位置漂移只作诊断（GNSS/EKF 游走可达米级）；
- 详细步骤、报告字段与退出码见 `tools/px4_link_smoke.md` §2.5。

**2026-10-21 台架尝试记录**：

- 第一次：预检 landed/未解锁/位置姿态通过，但全程 `rc=否`、`global=否`
  （`home` 在 1s 后有效）；零速引导与 Offboard ACK accepted，ARM 请求已入队
  但 PX4 未 armed，25s 后引导超时。处置：G4 预检强制 **RC_CHANNELS 在线**，
  ARM ACK 拒绝时立即打印 result 并中止；报告新增 RC/Home/global/local 与
  最近 ACK 诊断。
- 第二次（RC/global 已正常，ARM ACK accepted、真实 armed）：控制器未接管，
  解锁约 10s 后 PX4 `COM_DISARM_PRFLT` 自动上锁。根因是台架脚本在接管前
  只注入引导 setpoint，没有发布带 bit31 请求位的 `ControlIntent`，控制器
  授权链无法转 ACTIVE。已修复：预检通过即发布零速请求意图；同时把台架序列
  压缩到解锁后约 7.5s，并在工具/文档提示 `COM_DISARM_PRFLT` 默认 10s。
- 第三次（2026-10-21，修复后）：**自动部分通过**——RC/Home/global/local
  有效，ARM ACK accepted、真实 armed、控制器接管；零速漂移 0.1157m（≤0.15），
  四向设定值方向全部正确；COAST/IDLE 触发，IDLE→PX4 退出 Offboard 1972ms
  （≤2200ms），停发后额外发送 0 条；落地状态 DISARM 确认。最大上升/水平位移
  1.02m/0.98m 为本地位置诊断（landed 始终为是，现场确认未离地）。
  **人工确认**：G4-1 电机仅怠速 ✅。**未完成**：G4-2 电机/机体实际方向判读
  （设定值方向已全部正确）、G4-3 捕网行程（维护者暂缓，当前重点不是捕网控制）、
  G4-4 拔串口分支（下次测试；工具检测到断链后应停意图、关闭串口/心跳、
  刷盘并以退出码 4 退出，不尝试 AUTO.LAND/DISARM；PX4 侧结果用 QGC/飞控日志
  确认）。默认 `COM_DISARM_PRFLT=10s` 下解锁→DISARM 约 9s，余量仅约 1s，
  后续台架建议先临时调到 ≥30s。

G5 详细清单在设计稿 §5 G5 行标注"另行冻结"，建立时同步更新本节。
