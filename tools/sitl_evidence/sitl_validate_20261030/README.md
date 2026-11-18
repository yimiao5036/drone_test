# SITL 回归验证证据（2026-10-30）

> 目的：验证 2026-10-30 全量缺陷修复批（机载通信层/状态机与围栏、地面站协议字段）
> 在 WSL2 PX4 SITL 上的行为未回退、新行为符合预期。
> 环境：WSL2 Ubuntu 24.04，PX4-Autopilot（gz_x500，Gazebo 8.15），
> `build/drone_control` 与 `build/px4_link_smoke` 均为修复后构建（ctest 665/665）。

## 1. 执行命令与结果

| 场景 | 命令（`tools/sitl_harness.sh` 之后参数） | 结果 | 日志 |
|---|---|---|---|
| 围栏返航 | `smoke --config config/px4_sitl.json --duration 150 --sitl-stage4-fence` | **PASS**（11 项）：65018 下发、65019 VIOLATED、设定值钳 18.98m、S12→S13→S14 | `smoke_20261030_005041.log` |
| 围栏降落 | `smoke --config config/px4_sitl.json --duration 150 --sitl-stage4-fence-land` | **PASS**：动作=land 不经 S12 直达 S13/S14（`AbortForFence(kLand)` 统一路径） | `smoke_20261030_005211.log` |
| 高度带 | `smoke --config config/px4_sitl.json --duration 150 --sitl-stage4-fence-altitude` | **PASS**：仅高度越界 1.02m（下界 2m）→ 65019 + S12 | `smoke_20261030_005314.log` |
| 65020 返航 | `smoke --config config/px4_sitl.json --duration 150 --sitl-stage4-command` | **PASS**：65021 accepted、65020 单发、返航段带航向指向 Home | `smoke_20261030_005442.log` |
| 已分配+65020 | `smoke --config config/px4_sitl.json --duration 150 --sitl-stage4-command-assigned` | **PASS**：65015 `cancelled_by_gcs` 必发 | `smoke_20261030_005613.log` |
| G3 全链 | `smoke --config config/px4_sitl.json --duration 150 --sitl-stage4-mission` | **PASS**：S0→1→2→5→6→7→5；切非 Offboard→停发 20ms；围栏钳制 98.87m/100m | `smoke_20261030_005737.log` |
| 围栏保持 | `smoke --config config/px4_sitl.json --duration 150 --sitl-stage4-fence-hold` | **PASS**：越界原地保持 2500ms，未提前返航，65020 收尾 | `smoke_20261030_005842.log` |

注：`smoke` 场景按文档要求逐段 `stop` 后干净重跑；各场景退出码 0。

## 2. 任务分配链路（`stack` + `assignment_sitl.py`）

| 场景 | 结果 | 证据 |
|---|---|---|
| 65014 声明 zone=5/1（缓存 1/1） | 65015 **REJECTED reason=21**（fail-closed 生效） | `0065_20261030_010247.log`（同文件含默认区域生成） |
| 65014 声明 zone=1/1 + 65010 目标 | 65015 **ACCEPTED** → S3 ARMING → 自动起飞 2m → S5 → 目标失效保持 → S12→S13→S14 | `0065_20261030_010247.log` |
| 重启机载后 zone=6/1 | 65015 **REJECTED reason=ZONE_MISMATCH**（缓存 1/1 从 `zone.json` 装载成功） | `0066_20261030_010610.log` |
| 串口重连（A-01） | 杀 PTY 桥 → `串口等待读取失败` → 自动重连 17 次（500ms 退避）→ 桥恢复后 `串口重连成功` → 心跳/65014 恢复 | `0067_20261030_010811.log` |

## 3. 关键发现与注意

1. **旧场地围栏缓存会导致"恒越界"**：`config/zone.json` 残留广州坐标（23.11/113.11），
   与 SITL Home（47.40/8.55）相距 834 万米；`drone_control` 正常装载并持续上报越界。
   属于缓存持久化的正常工作方式，但换场地/换 SITL 前必须清缓存（见 `tools/sitl_harness.md` §6）。
2. **机载默认区域 1/1**：无缓存时自动生成 Home 中心 100m 默认区域（id/version=1/1），
   地面站若声明其他 id/version 会被 fail-closed 拒绝（reason=21）。测试地面站/验收脚本
   需声明 1/1 或 `--zone-id 0`（不声明）。
3. **`assignment_sitl.py` 补 `ZONE_MISMATCH` 标签**：此前 reason=21 显示为 `UNKNOWN(21)`。
4. 杀 PTY 桥验证重连时，禁止在同一条 shell 命令里附带 `sitl_pty_bridge`/`gz sim` 字样
   再调用 `harness stop`（pkill 模式会自匹配）；建议 `tmux kill-session -t bridge`。

## 4. 复现

```bash
cd /mnt/e/dswj/无人机/香橙派的代码/drone_test
rm -f config/zone.json build/config/zone.json     # 换场地时先清缓存
bash tools/sitl_harness.sh stop
bash tools/sitl_harness.sh smoke --config config/px4_sitl.json --duration 150 --sitl-stage4-fence
# ……逐场景（每段之间 stop + 等待数秒）
bash tools/sitl_harness.sh stop
bash tools/sitl_harness.sh stack --config config/px4_sitl_assignment.json
python3 tools/assignment_sitl.py --connection /tmp/gcs_test \
  --aircraft-system 1 --aircraft-component 25 --assignment-id 1 --target-id 7 \
  --interceptor-type 25 --send-target --zone-id 1 --zone-version 1 \
  --target-latitude <Home+20m> --target-longitude <Home> --target-altitude-m 2.0 \
  --stop-target-after-ms 20000 --duration 75
```
