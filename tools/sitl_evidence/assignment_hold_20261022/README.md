# S5 目标失效原地保持 SITL 复跑证据（2026-10-22）

环境：WSL2 Ubuntu 24.04；PX4 1.17.0 + Gazebo 8.15（`gz_x500`，朝北出生）；
机载身份 1/25，模拟地面站 255/190；`config/px4_sitl_assignment.json`
（`enable_control=true` 仅用于 SITL）；构建基线 **593/593**。

## 目的

验证 2026-10-22 落地的“S5 目标失效原地保持”改造（原因码 9、65013 bit5、
累计上限 `takeoff.target_hold_max_ms` 默认 60s）：目标丢失/从未到达时，
机载持续发当前位置保持意图，PX4 全程留在 Offboard，10s 看门狗到点后由
**S12 机载位置返航**接管，不再被 PX4 失效兜底（→ Position → 无 RC 时
`NAV_RCL_ACT=2` → AUTO.RTL）抢先。

## 运行方式（可复现）

```bash
# 1. 桥 + PX4 SITL + GCS 心跳 + 生产程序（干净重启）
python3 tools/sitl_pty_bridge.py &            # /tmp/gcs_app（机载串口）、/tmp/gcs_test（工具）
cd /root/PX4-Autopilot && HEADLESS=1 PX4_GZ_MODEL_POSE=0,0,0,0,0,1.5708 make px4_sitl gz_x500
#   等 home set 后：param set COM_LOW_BAT_ACT 0 / BAT_LOW_THR 0.15
python3 tools/sitl_gcs.py &                   # UDP 14550 GCS 心跳
cd <repo>/build && ./drone_control --config <repo>/config/px4_sitl_assignment.json

# 2a. 场景 7b：正常起飞 → 20s 后停止 65010 → 保持 10s → S12 返航降落
python3 tools/assignment_sitl.py --connection /tmp/gcs_test \
  --aircraft-system 1 --aircraft-component 25 \
  --assignment-id 2 --target-id 7 --interceptor-type 25 \
  --send-target --target-latitude 47.3981504 --target-longitude 8.5461635 \
  --target-altitude-m 2.0 --stop-target-after-ms 20000 --duration 75

# 2b. 场景 7a：不发送 65010（从未获得目标）→ 保持 10s → S12 返航降落
python3 tools/assignment_sitl.py --connection /tmp/gcs_test \
  --aircraft-system 1 --aircraft-component 25 \
  --assignment-id 3 --target-id 7 --interceptor-type 25 --duration 45
```

## 结果

| 场景 | 结果 | 证据 | 关键观测 |
|---|---|---|---|
| 7b 目标丢失（获得后停发 65010） | ✅ | `0023_7b_target_lost_20261022.log`、`tool_7b_target_lost.log` | S5 引导 → 目标失效 19:38:34.029 进入保持（hold=PX4 当前 21.24,-0.69,-1.80）→ 保持 **10009ms**（期间无 COAST）→ 19:38:44.038 看门狗 `target_lost` → S12 爬升 5.0m → 水平返航 → 19:38:58.752 到达 Home → S13 AUTO.LAND → S14；65015 `CANCELLED_OR_ABORTED/TARGET_LOST`，availability TERMINAL |
| 7a 从未获得目标 | ✅ | `0024_7a_wait_timeout_20261022.log`、`tool_7a_wait_timeout.log` | S5 入口即进入保持（hold=起飞点 0.13,-0.11,-1.90）→ 保持 **10009ms**（无 COAST）→ 看门狗 `target_wait_timeout` → S12 爬升 5m → 返航 → S13 → S14；65015 `CANCELLED_OR_ABORTED/TARGET_WAIT_TIMEOUT` |
| 7b（首次运行，工具被 `head` 截断致 SIGPIPE 停发） | ✅ | `0022_7b_sigpipe_20261022.log` | 同一行为：保持 **10015ms**、无 COAST、S12 返航降落（SIGPIPE 只是停止发送的触发方式，不影响判读） |

PX4 侧：`px4_console.log` 与 `px4_rtl_failsafe_grep.txt` 中**没有 RTL/failsafe/
Offboard 丢失**记录（仅启动早期 `Preflight Fail: No connection to the GCS`，
GCS 心跳建立后消失）。

## 结论

- 改造达成设计目标：目标失效期间控制流不断（控制器保持 ACTIVE、PX4 保持
  Offboard），10s 看门狗到点后 **S12 执行机载 5m 位置返航**，未出现 PX4 RTL
  抢先接管；`S12 进入时 PX4 已离开 Offboard` 的让位分支不再被触发。
- 与改造前对比：旧证据 `tools/sitl_evidence/assignment_20261022/0007` 中
  目标丢失后 PX4 自行 RTL，S12 只做监控；本目录为改造后行为。
- 场景 7a/7b 通过标准更新见 `docs/PX4实链验证清单.md` §7.4。

## 保持改造后的门禁回归（2026-10-22）

| 场景 | 结果 | 证据 | 关键观测 |
|---|---|---|---|
| G3 全链（`--sitl-stage4-mission`，`config/px4_sitl.json`） | ✅ 全 PASS | `regression_g3_stage4_mission_20261022.log`、`regression_g3_px4_console.log`、`regression_g3_gcs.log` | 迁移 0→1→2→5→6→7→5；S5 引导 `reason=7`（保持未介入）；交接缝 201ms、窗口内无 COAST；切非 Offboard 停发 40ms；退出 Offboard 1459ms（上限 2200） |
| stage4-return（`--sitl-stage4-return`，`config/px4_sitl_return.json`） | ✅ 全 PASS | `regression_return_stage4_20261022.log`、`regression_return_px4_console.log`、`regression_return_gcs.log` | S9→S10（发射）→S12 机载返航→S13 AUTO.LAND→S14；187 触发/复位 ACK accepted，保持间隔 1609ms（reset_delay 1500ms，容差 500ms）；迁移 11 次、错误 0 |

两次回归均在 S5 目标连续新鲜的正常引导路径上运行，保持逻辑未介入，确认改造
未影响既有 S5→S6→S7 交接链与 S9~S14 返航链。
