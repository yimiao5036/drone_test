# 65020 主动返航 / 65018 围栏 SITL 专项证据（2026-10-28）

两条新场景由 `tools/sitl_harness.sh smoke` 驱动，**非门禁**，用于在没有真机和
场地的情况下先把「人工返航指令」与「共享区域越界」两条链路跑通。机载代码与
生产配置一致，工具只在自身进程内冒充地面站（源 id 255/190）。

## 复现命令

```bash
tools/sitl_harness.sh stop                      # 必须先停，避免上一段会话残留
tools/sitl_harness.sh smoke --config config/px4_sitl.json --duration 120 --sitl-stage4-command
tools/sitl_harness.sh stop
tools/sitl_harness.sh smoke --config config/px4_sitl.json --duration 120 --sitl-stage4-fence
```

## 文件

| 文件 | 说明 |
|---|---|
| `stage4_command_20261028_pass.log` | 65020 主动返航专项 **PASS** |
| `stage4_fence_20261028_pass.log` | 65018 围栏专项 **PASS** |

## 判读要点

### 65020（stage4-command）

| 检查项 | 实测 |
|---|---|
| 65020 下发 | `command_id=1`，S7 稳定锁定后单发 |
| 65021 回执 | `result=0 reason=0 state=12`（accepted，已在 S12） |
| 未进 S9/S10 | 人工中止不是拦截完成，`saw_intercept_executing_` 全 false |
| S12 航向 | 返航段位置设定值 `USE_YAW=1`，航向指向 Home（276 个样本，偏差 ≤30°） |
| S12→S13→S14 | 状态机接管返航/降落/上锁，工具不再补发 AUTO.LAND |

`65015 reason=cancelled_by_gcs` 在本场景**按设计不发**：场景里没有先发 65014
任务分配，机载 `accepted_assignment_` 为空，`PublishTakeoffAbortAck` 直接返回
（`mission_state_machine.cpp` 中该方法首行即 `if (!accepted_assignment_) return;`）。
带分配的中止路径由单测 `MissionCommandReturnHomeAbortsToReturnHome` 覆盖。

### 65018（stage4-fence）

| 检查项 | 实测 |
|---|---|
| 65018 下发 | `zone=21/1` 方形区域、半径 20m、动作 `kReturn`、Home 相对高度 |
| 注入时位置 | 局部 N=22.31m（区域半径 20m，机体已在区域外） |
| 65019 | `VIOLATED`，距边界 −3.88m，持续 901ms |
| 设定值钳制 | 最大设定值模长 **18.98m ≤ 20m**（控制器按多边形内缩 1m 钳制） |
| S12→S13→S14 | 越界中止 → 返航 → 降落 → 上锁，迁移 8 次、错误 0 |

## 教训

- `sitl_harness.sh smoke` 之间必须 `stop` 并等待，否则上一段 PX4/PTY 残留会让
  下一段读到脏状态（2026-10-28 已踩过两次假失败，见 `tools/sitl_harness.md`）。
- 围栏场景的 20m 半径是刻意选的：S7 稳态在目标（北向 40m）后方约 10m 处，
  机体 N≈30m，注入即越界；主目标 40m 与 S5 接近窗 20m 之外的**更小**半径才能
  在视觉锁定后立刻触发，否则要等机体再飞出去。
