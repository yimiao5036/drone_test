# SITL 验收装置（tools/sitl_harness.sh）

> 更新：2026-10-22。适用环境：Windows 主机 WSL2 的 **Ubuntu 24.04**（唯一构建环境）。

## 1. 功能职责

**做什么**：在 WSL2 内一键起停 PX4 SITL 验收环境，把「仿真飞控 + 机载地面站串口 +
地面站心跳 + 生产程序 `drone_control`」装配起来，供两类验收使用：

- `tools/assignment_sitl.py`：多机任务分配（65014/65015/65013 v2）与自动起飞
  场景（S2→S5、取消、目标等待/丢失、低电等），见
  [`docs/PX4实链验证清单.md`](../docs/PX4实链验证清单.md) §7.4；
- `build/px4_link_smoke`：G1~G3 与 S11~S14 分级场景，见
  [`px4_link_smoke.md`](px4_link_smoke.md)、[`PX4_SITL分级测试.md`](PX4_SITL分级测试.md)；
- Web 地面站（`tools/web平台`，跑在 Windows）：`webgcs` 子命令把 `/tmp/gcs_test`
  桥成 TCP，让 Web 端用「TCP 客户端」链路做真实界面联调（65011/65012/65013 回传、
  65020 返航/重置、65018 围栏下发），机载代码不改。

**不做什么**：

- 只面向 **PX4 SITL（gz_x500 仿真）**，不连接、不探测任何真实飞控/串口/网口；
- 不替代判据：装置只负责起停与日志落盘，PASS/FAIL 由 `px4_link_smoke` 报告、
  `assignment_sitl.py` 的 65015/65013 回执与机载日志判定；
- 不修改仓库配置：生产 `config/config.json` 保持 `enable_control=false`；SITL 用
  独立的 `config/px4_sitl_assignment.json`（显式 `enable_control=true`）。

## 2. 组成与数据流

| 文件 | 作用 |
|---|---|
| `tools/sitl_harness.sh` | 本装置入口（子命令 `stack`/`app`/`smoke`/`webgcs`/`stop`/`status`） |
| `tools/sitl_pty_bridge.py` | 双 PTY 桥：`/tmp/gcs_app`（机载串口）↔ `/tmp/gcs_test`（验收工具） |
| `tools/sitl_gcs.py` | PX4 GCS 链路（UDP 14550）255/190 心跳，并打印 Home/模式 |
| `tools/sitl_web_gcs.py` | 把 `/tmp/gcs_test` 桥到 TCP（默认 5761），供 Windows 上的 Web 地面站接入（2026-11-04） |
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
5. **想看 3D（可选）**：需要 WSLg（`/mnt/wslg` 存在、`$DISPLAY` 有值，Windows 11 +
   WSL2 默认自带；缺了先 `wsl --update`），然后
   `SITL_GUI=1 bash tools/sitl_harness.sh stack` —— Gazebo 窗口会直接出现在 Windows
   桌面。默认 `HEADLESS=1` 无窗口。注意 GUI 会拉低仿真实时率（画面卡顿、起飞更慢），
   自动化时序断言（`height_timeout=15000ms` 等）仍建议无界面跑。

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
# 65014 已接受 + 65020（验证 65015 中止回执）/ 65018 action=hold（越界保持后再由 65020 收尾）
bash tools/sitl_harness.sh smoke --config config/px4_sitl.json --duration 150 --sitl-stage4-command-assigned
bash tools/sitl_harness.sh smoke --config config/px4_sitl.json --duration 150 --sitl-stage4-fence-hold
bash tools/sitl_harness.sh smoke --config config/px4_sitl.json --duration 150 --sitl-stage4-fence-land
bash tools/sitl_harness.sh smoke --config config/px4_sitl.json --duration 150 --sitl-stage4-fence-altitude

# 查看状态 / 停止
bash tools/sitl_harness.sh status
bash tools/sitl_harness.sh stop
```

### 4.1 接 Web 地面站（`webgcs`，2026-11-04）

Web 地面站（`tools/web平台`）跑在 Windows 上，打不开 WSL 里的 PTY；`webgcs` 用
`tools/sitl_web_gcs.py` 把验收端点 `/tmp/gcs_test` 桥成 TCP，让 Web 服务端用
「TCP 客户端」链路接进来。**机载代码与 `serial.device` 都不改**。

```bash
bash tools/sitl_harness.sh stack                 # 先起环境（PX4+桥+机载）
bash tools/sitl_harness.sh webgcs                # 停掉 sitl_gcs.py，桥到 TCP 5761
bash tools/sitl_harness.sh webgcs --port 5762    # 换端口
```

然后在 Web 端「连接设备」选 **TCP 客户端**：地址 `127.0.0.1`（WSL2 本地转发不通时
用 `hostname -I` 打印的地址），端口 `5761`。连上后应看到 65011/65012/65013 回传，
可在界面上点「主动返航」「重置状态」，或在「共享区域」里保存并下发 65018。

> - **同一个端点只能有一个地面站**：`webgcs` 会先停掉 `sitl_gcs.py`（它也发 255/190
>   心跳，两个一起发会打架）。要回到脚本验收，直接 `stack` 或 `app` 重启即可。
> - Web 端地面站身份必须是 `255/190`，机载是 `1/25`；不一致时 65010~65022 会被
>   地址校验丢掉（表现为"链路 online 但看不到任务状态"）。
> - 桥的日志在 `$LOG_DIR/webgcs.log`，`stop` 会一并清理该会话与进程。

> **场景之间必须 `stop` 并等待数秒**（2026-10-28 复现）：`smoke` 跑完会留下 SITL 进程与
> 会话状态，紧接着再起下一段会带着上一段的初值起飞，表现为悬停窗口不达标
> （相对高度 1.35m）、`[FAIL] 飞行未触发高度/漂移安全回收`，或视觉交接段 S7 启动后回退 S5
> 导致 `[FAIL] S9/S10 进入`。这些是假失败，`stop` 后干净起始复跑即为 PASS；
> 证据与判读见 `tools/sitl_evidence/g5_prep_20261028/README.md`。

环境变量（可选）：`PX4_DIR`（默认 `$HOME/PX4-Autopilot`）、`REPO`（默认脚本上级
目录）、`LOG_DIR`（默认 `/tmp/sitl_harness`）、`SITL_GUI=1`（拉起 Gazebo 3D 图形界面，
WSLg 下窗口直接出现在 Windows 桌面；默认 0 无界面）。

> `stack` 启动前会把 `build/config/zone.json` 移到 `$LOG_DIR` 备份：测试用例会把
> 泸州测试区写进该缓存，与 SITL 的苏黎世 Home 冲突时会开机即判 `距边界=-8342817m`
> 并瞬间中止任务（现象是 `围栏越界中止` + 65015 `FENCE_VIOLATION`）。

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
| 手工清理进程时 shell 自杀 | 不要用 `pkill -f "gz sim"`（会匹配到自己的命令行）；用括号转义 `pkill -f "[g]z sim"`（本装置已按此处理）。**注意**：若同一条命令里既有 `harness stop` 又出现 `gz sim`/`sitl_pty_bridge`/`sitl_gcs` 字样，括号转义仍会命中，导致 shell 被 SIGTERM；杀桥请改用 `tmux kill-session -t bridge` |
| 围栏恒越界 / 65014 恒被拒 21 | `config/zone.json` 是运行时缓存：换场地或换 SITL 前删除 `config/zone.json` 与 `build/config/zone.json`（旧坐标区域与当前 Home 相距过远会恒越界；机载启动无缓存时会生成 Home 中心 100m 默认区域 1/1，地面站需声明 1/1 或 `--zone-id 0` 不声明） |
| 场景间状态残留 | 用 `stack`/`smoke` 干净重启；`app` 只重启机载，PX4 侧位置/模式状态保留 |

## 7. 安全边界

- 仅用于 WSL2 内的 PX4 SITL；**禁止**把本装置或 `config/px4_sitl_assignment.json`
  用于真实机体（该配置 `enable_control=true`，会向飞控发控制指令）；
- 真实台架/实机流程见 [`docs/PX4实链验证清单.md`](../docs/PX4实链验证清单.md)
  §7.3（拆桨、RC 兜底、预检要求），与 SITL 装置不是同一条路径。
