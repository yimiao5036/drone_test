# SITL 验收装置（tools/sitl_harness.sh）

> 更新：2026-10-22。适用环境：Windows 主机 WSL2 的 **Ubuntu 24.04**（唯一构建环境）。

## 1. 功能职责

**做什么**：在 WSL2 内一键起停 PX4 SITL 验收环境，把「仿真飞控 + 机载地面站串口 +
地面站心跳 + 生产程序 `drone_control`」装配起来，供两类验收使用：

- `tools/assignment_sitl.py`：多机任务分配（65014/65015/65013 v2）与自动起飞
  场景（S2→S5、取消、目标等待/丢失、低电等），见
  [`docs/PX4实链验证清单.md`](../docs/PX4实链验证清单.md) §7.4；
- `build/px4_link_smoke`：G1~G3 与 S11~S14 分级场景，见
  [`px4_link_smoke.md`](px4_link_smoke.md)、[`PX4_SITL分级测试.md`](PX4_SITL分级测试.md)。

**不做什么**：

- 只面向 **PX4 SITL（gz_x500 仿真）**，不连接、不探测任何真实飞控/串口/网口；
- 不替代判据：装置只负责起停与日志落盘，PASS/FAIL 由 `px4_link_smoke` 报告、
  `assignment_sitl.py` 的 65015/65013 回执与机载日志判定；
- 不修改仓库配置：生产 `config/config.json` 保持 `enable_control=false`；SITL 用
  独立的 `config/px4_sitl_assignment.json`（显式 `enable_control=true`）。

## 2. 组成与数据流

| 文件 | 作用 |
|---|---|
| `tools/sitl_harness.sh` | 本装置入口（子命令 `stack`/`app`/`smoke`/`stop`/`status`） |
| `tools/sitl_pty_bridge.py` | 双 PTY 桥：`/tmp/gcs_app`（机载串口）↔ `/tmp/gcs_test`（验收工具） |
| `tools/sitl_gcs.py` | PX4 GCS 链路（UDP 14550）255/190 心跳，并打印 Home/模式 |
| `config/px4_sitl_assignment.json` | SITL 验收配置：PX4 走 UDP、地面站串口=`/tmp/gcs_app`、`enable_control=true` |
| `tools/sitl_restart.sh` | 兼容入口（等价 `sitl_harness.sh stack`），保留给既有证据 README 的复现步骤 |

```text
assignment_sitl.py --connection /tmp/gcs_test
  → sitl_pty_bridge.py（PTY 双向转发）
  → /tmp/gcs_app → GroundStationLink（机载串口链路，65010/65013/65014/65015）
drone_control 的 Px4Link
  → UDP 127.0.0.1:14540（本地）/14580（PX4）→ PX4 SITL
sitl_gcs.py → UDP 14550 回发 MAV_TYPE_GCS 心跳（PX4 无 RC 时解锁前置条件）
```

## 3. 前置条件

1. WSL2 **Ubuntu 24.04**；`~/PX4-Autopilot` 已构建（存在
   `build/px4_sitl_default/bin/px4`），Gazebo Harmonic 8.15 可用；
2. `python3 -c "import pymavlink"` 可用；`tmux` 已安装；
3. 本仓库已构建：`cmake -S . -B build && cmake --build build -j$(nproc)`
   （需要 `build/drone_control` 与 `build/px4_link_smoke`）；
4. 无需真实硬件；SITL 参数由装置在启动后设置（`COM_LOW_BAT_ACT=0`、
   `BAT_LOW_THR=0.15`、`SIM_BAT_MIN_PCT=50`、`SIM_BAT_DRAIN=0`；`smoke` 另设
   `MPC_XY_VEL_MAX=2.0`）。

## 4. 命令

```bash
# 起完整环境（PX4+桥+GCS+生产程序），等待机载进入 READY 并打印 Home
bash tools/sitl_harness.sh stack

# 仅重启生产程序（PX4/桥/GCS 保持）——换配置或重跑场景时用，省去 40s 起飞等待
bash tools/sitl_harness.sh app [--config config/px4_sitl_assignment.json]

# 起 PX4+GCS 后运行 px4_link_smoke 场景（配置可换）
bash tools/sitl_harness.sh smoke --config config/px4_sitl.json --duration 120 --sitl-stage4-mission
bash tools/sitl_harness.sh smoke --config config/px4_sitl_return.json --duration 120 --sitl-stage4-return
# 65020 主动返航 / 65018 围栏（2026-10-28 起，非门禁，用 px4_sitl.json 即可）
bash tools/sitl_harness.sh smoke --config config/px4_sitl.json --duration 120 --sitl-stage4-command
bash tools/sitl_harness.sh smoke --config config/px4_sitl.json --duration 120 --sitl-stage4-fence

# 查看状态 / 停止
bash tools/sitl_harness.sh status
bash tools/sitl_harness.sh stop
```

> **场景之间必须 `stop` 并等待数秒**（2026-10-28 复现）：`smoke` 跑完会留下 SITL 进程与
> 会话状态，紧接着再起下一段会带着上一段的初值起飞，表现为悬停窗口不达标
> （相对高度 1.35m）、`[FAIL] 飞行未触发高度/漂移安全回收`，或视觉交接段 S7 启动后回退 S5
> 导致 `[FAIL] S9/S10 进入`。这些是假失败，`stop` 后干净起始复跑即为 PASS；
> 证据与判读见 `tools/sitl_evidence/g5_prep_20261028/README.md`。

环境变量（可选）：`PX4_DIR`（默认 `$HOME/PX4-Autopilot`）、`REPO`（默认脚本上级
目录）、`LOG_DIR`（默认 `/tmp/sitl_harness`）。

验收工具用法（装置就绪后另开终端）：

```bash
# 场景 7b：正常起飞 → 20s 后停止 65010 → 保持 10s → S12 返航降落
python3 tools/assignment_sitl.py --connection /tmp/gcs_test \
  --aircraft-system 1 --aircraft-component 25 \
  --assignment-id 2 --target-id 7 --interceptor-type 25 \
  --send-target --target-latitude <Home+20m 纬度> --target-longitude <Home 经度> \
  --target-altitude-m 2.0 --stop-target-after-ms 20000 --duration 75

# 场景 7a：不发 65010（从未获得目标）→ 保持 10s → S12 返航降落
python3 tools/assignment_sitl.py --connection /tmp/gcs_test \
  --aircraft-system 1 --aircraft-component 25 \
  --assignment-id 3 --target-id 7 --interceptor-type 25 --duration 45
```

## 5. 日志与判读

| 日志 | 内容 |
|---|---|
| `/tmp/sitl_harness/px4.log` | PX4 控制台（`home set`、模式、failsafe/RTL 痕迹） |
| `/tmp/sitl_harness/gcs.log` | GCS 心跳与 Home/模式打印（取目标纬度用 Home+20m） |
| `/tmp/sitl_harness/bridge.log` | 桥启动信息（PTY 名与链接） |
| `/tmp/sitl_harness/app.log` | `drone_control` 终端输出（stdout 副本） |
| `build/logs/00xx_*.log` | 机载正式日志（spdlog，**主要证据**；`stack` 结束会打印路径） |
| `/tmp/sitl_harness/smoke_<时间戳>.log` | `px4_link_smoke` 报告 |

判读要点：

- **任务分配**：65015 `ACCEPTED` → 65013 `availability` `ASSIGNED`→`ENGAGED`；
  中止时 65015 `CANCELLED_OR_ABORTED` 带 `reason`（`target_lost` 等）。
- **S5 目标保持**：机载日志 `S5 目标失效，进入原地保持 ... reason=9` →
  `S5 原地保持结束 ... held_ms=...`；保持期间不应出现
  `飞行控制器控制权迁移: ACTIVE -> COAST`（说明 setpoint 流未断）。
- **返航归属**：S12 由机载执行时日志为 `返航爬升完成（相对 Home 5.0m）` →
  `到达 Home 附近，进入降落`；若出现 `返航中 PX4 已离开 Offboard` 或 px4.log
  有 RTL/failsafe，则是 PX4 接管（当前设计下不应发生在 S5 目标保持场景）。
- **证据归档**：`*.log` 被 `.gitignore` 排除，入库需 `git add -f`；建议按
  `tools/sitl_evidence/<主题>_<日期>/` 归档并写 README（示例见
  `tools/sitl_evidence/assignment_hold_20261022/`）。

## 6. 排查要点

| 现象 | 定位 |
|---|---|
| `PX4 SITL 启动失败` | 看 `$LOG_DIR/px4.log` 尾部；确认 `~/PX4-Autopilot` 已构建、Gazebo 可用、无残留 `gz sim`（先 `stop`） |
| `drone_control 未进入 READY` | 看 `$LOG_DIR/app.log` 与机载日志：PX4 心跳/UDP 端口占用、`ground_station.serial.device` 是否 `/tmp/gcs_app`、桥是否先于 app 启动 |
| 65014 无 65015 回执 | 地址双字段是否 `1/25`、地面站来源是否 `255/190`；`enable_control=true` 且命令接口已注入 |
| PX4 拒绝解锁 | GCS 心跳是否在线（`gcs.log`）；`COM_LOW_BAT_ACT=0`；S5 引导速度受 `MPC_XY_VEL_MAX` 约束（`smoke` 已设 2.0） |
| 手工清理进程时 shell 自杀 | 不要用 `pkill -f "gz sim"`（会匹配到自己的命令行）；用括号转义 `pkill -f "[g]z sim"`（本装置已按此处理） |
| 场景间状态残留 | 用 `stack`/`smoke` 干净重启；`app` 只重启机载，PX4 侧位置/模式状态保留 |

## 7. 安全边界

- 仅用于 WSL2 内的 PX4 SITL；**禁止**把本装置或 `config/px4_sitl_assignment.json`
  用于真实机体（该配置 `enable_control=true`，会向飞控发控制指令）；
- 真实台架/实机流程见 [`docs/PX4实链验证清单.md`](../docs/PX4实链验证清单.md)
  §7.3（拆桨、RC 兜底、预检要求），与 SITL 装置不是同一条路径。
