# 多机任务分配 / 自动起飞 SITL 专项证据（2026-10-22）

环境：WSL2 Ubuntu 24.04；PX4 1.17.0 + Gazebo 8.15（`gz_x500`，朝北出生）；
机载身份 1/25，模拟地面站 255/190；`config/px4_sitl_assignment.json`
（`enable_control=true` 仅用于 SITL）。构建基线 584/584。

## 运行方式（可复现）

```bash
# 1. 构建（主工程构建副本）
cmake -S . -B build && cmake --build build -j$(nproc)

# 2. PTY 桥（保持运行）：/tmp/gcs_app 给机载串口，/tmp/gcs_test 给验收工具
python3 tools/sitl_pty_bridge.py

# 3. 干净重启 PX4 SITL + PX4 GCS 心跳 + drone_control
bash tools/sitl_restart.sh

# 4. 发送分配/目标（连接串直接传串口设备路径，pymavlink 默认 115200）
python3 tools/assignment_sitl.py --connection /tmp/gcs_test \
  --aircraft-system 1 --aircraft-component 25 \
  --assignment-id 1 --target-id 7 --interceptor-type 25 \
  --send-target --target-latitude <Home+20m 纬度> --target-longitude <Home 经度> \
  --target-altitude-m 2.0 --duration 45
```

低电专项在重启后覆盖 PX4 参数并重启应用：`BAT_LOW_THR=0.5`、
`SIM_BAT_MIN_PCT=40`、`SIM_BAT_DRAIN=20`（仅 armed 时放电；满量程 20s）。
电池遥测失效：飞行中执行 `battery_simulator stop`。
176/400 超时与 ACK 拒绝：使用 `build/config` 下的故障注入配置
（`mode_timeout=1ms`、`arm_timeout=1ms`、`command_ack_timeout=3000ms` 等），
或 PX4 `COM_ARM_AUTH_REQ=1`。

> 注：本目录只保留了机载 `drone_control` / 工具 spdlog 原始日志（WSL `/tmp`
> 下的终端输出在会话清理时丢失）。机载日志包含收到的 65014、65015 发送结果、
> 状态迁移与命令 ACK，是本次验收的主要证据。

## 结果总表

| # | 场景 | 结果 | 证据日志 | 关键观测 |
|---:|---|---|---|---|
| 1 | 正常自动起飞 + 空中取消 | ✅ | `0025_20261022_153537.log` | S2→S3→S4→S5；65015 accepted；availability assigned→engaged；Offboard 6/0 + armed + 2m；20s 取消后 S12→S13→S14 |
| 2 | 无分配 | ✅ | `0002_20261022_150030.log` | 仅 S2 READY；无 176/400、无 ARMING |
| 3 | 地址错误 / 类型不匹配 | ✅ | `0002_20261022_150030.log` | 2/25 地址静默（不回 65015）；类型 26 → rejected unsupported_interceptor_type，S2 |
| 4a | 176 超时（mode 未确认） | ✅ | `0012_20261022_152250.log` | `mode_timeout=1ms`；176 一次、ACK 已 accepted 但未等到实际 6/0 → MODE_TIMEOUT、S2；控制器 BOOTSTRAP→IDLE |
| 4b | 400 超时（arm 未刷新） | ✅ | `0013_20261022_152332.log` | `arm_timeout=1ms`；ARM ACK accepted 后中止，补发一次 DISARM(400 p1=0)，S2；控制器 IDLE |
| 4c | PX4 GCS 丢失（外部失效） | ✅ | `0014_20261022_152416.log` | GCS 心跳断开触发 PX4 failsafe/RTL；S4 检测离开 Offboard → S13 仅监控 → S14 |
| 4d | 400 ACK 超时（arm 授权） | ✅ | `0015_20261022_152521.log` | `COM_ARM_AUTH_REQ=1`；400 无 ACK → 1s ACK 超时 → ARM_TIMEOUT、S2 |
| 4e | 400 ACK 拒绝 | ✅ | `0016_20261022_152608.log` | `COM_ARM_AUTH_TO=0.5` + ACK 超时 3s；PX4 回 `result=1` → “400 ACK 非 ACCEPTED” → ARM_TIMEOUT、S2 |
| 5 | bootstrap 超时 | ⚠️ 结构性不可达（设计兜底） | 同 4a/4b | `bootstrap_timeout` 校验要求大于 lead+mode+arm，S3 模式/解锁超时总是先到；S3 中止后控制器 BOOTSTRAP→IDLE 已观测 |
| 6a | 起飞前取消（ARM 后） | ✅ | `0024_20261022_153455.log` | S3 中取消 → 地面中止 → 补发一次 DISARM → S2 READY；65015 cancelled |
| 6b | 起飞后取消 | ✅ | `0004_20261022_150353.log` | S5 取消 → S12 爬升 5m → 水平返航 → S13 AUTO.LAND → S14；65015 cancelled |
| 7a | 目标等待超时 | ✅（修复后） | `0005`（修复前）/`0006_20261022_151409.log`（修复后） | 10s 未获得匹配目标 → 65015 aborted TARGET_WAIT_TIMEOUT → S12；修复前 S12 卡死到返航超时，修复后 S12→S13→S14 |
| 7b | 目标丢失超时 | ✅ | `0007_20261022_151530.log` | 停止 65010 → 目标超龄 + 10s 丢失计时 → TARGET_LOST → S12；PX4 已 RTL → S13 仅监控 → S14 |
| 8 | 低电主动 S12 | ✅ | `0009_20261022_151746.log` | `BAT_LOW_THR=50%` + SIM 放电；S4/S5 越过阈值 → battery_low 防抖 → S12 返航 → S13/S14 |
| 8b | 电池遥测失效 | ✅ | `0011_20261022_152110.log` | 飞行中 `battery_simulator stop` → 2s 遥测超时 → BATTERY_TELEMETRY_INVALID → S12→S14 |
| 9 | 中止后清理 | ✅ | `0012`/`0013`/`0016`/`0024` | 176/ARM/DISARM 各一次，无重试；控制器最终 IDLE；无残留 setpoint |
| 10 | G2/G3/stage4-return 回归 | ✅ | `0017`（G2）、`0019`（G3）、`0021`（修复前 return）、`0023`（修复后 return） | 三份报告全 PASS；stage4-return 因 Task 5 的 S2→S5 语义变化一度失败，工具已用测试钩子修复 |

## 本次发现与修复

1. **BAT_LOW_THR 量纲错误（已修复）**：PX4 1.17 `BAT_LOW_THR` 为
   `unit: norm`（默认 0.15），机载原样写入 `battery_low_threshold_pct`
   并与 0~100 的剩余电量比较，低电守卫实际永不触发。已改为读取时 ×100
   （SITL 验证日志 `BAT_LOW_THR=15.0%` / `50.0%`），并同步单测与文档。
2. **地面中止竞态（已修复）**：400 ACK 已 accepted 但 armed 位尚未刷新时取消，
   原逻辑只按 `armed` 决定是否 DISARM，飞机可能被留在 armed（PX4 靠
   `COM_DISARM_PRFLT` 兜底）。现按“armed 或本轮 400 ACK accepted”补一次 DISARM；
   `0024` 验证 DISARM 已入队并被 PX4 ACK。
3. **S12 落地后卡死（已修复）**：S5 无目标/目标丢失时，PX4 会先于 10s 看门狗
   退出 Offboard 并自行 RTL 落地；落地后 PX4 又恢复 Offboard 模式，原 S12
   只按模式判续命，会对已落地/已上锁的飞机持续发返航目标并卡到 60s 超时。
   现 S12 先判 `landed && !armed`，直接转 S13→S14（`0005`→`0006`）。
4. **stage4-return 回归（已修复工具侧）**：Task 5 禁止“65010 单独把 S2 推到 S5”
   后，注入了命令接口的 stage4-tune/feedforward/return 工具无法再进入 S5。
   工具在自身起飞完成、悬停稳定后以 `ForceStateForTest(kGpsApproach)`
   显式进入（`test_hooks_enabled=true`，仅工具装配）；`0023` 全 PASS。
5. **待决策（设计层）**：S5 目标看门狗（10s）晚于 PX4 的 Offboard 丢失兜底
   （`intent_timeout 300ms` + `coast_timeout 1000ms` + `COM_OF_LOSS_T 1s`），
   目标缺失/丢失时**实际由 PX4 RTL 接管**，S12 无法执行“机载位置返航”。
   若要求现场一定由机载 S12 返航，需要缩短看门狗或估计失效时保持位置保持意图；
   本次按现状验收，未改设计。
6. **176 ACK 拒绝无法在 SITL 单点注入**（PX4 对 Offboard 请求通常接受）；
   176 超时与 400 超时/拒绝已实跑，176 ACK 拒绝分支由单测覆盖。
