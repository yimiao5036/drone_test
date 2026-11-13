# 2026-10-28 SITL 复测证据（G3 全链 / S11~S14 返航专项）

配合 `docs/PX4实链验证清单.md` §7.1 使用。本目录是**复测证据**，不改变任何门禁的既有结论等级。

## 环境

- WSL2 Ubuntu 24.04，PX4 SITL v1.17.0（`~/PX4-Autopilot`）
- 仓库提交 `4755d38` 及其后仅文档改动；本次复测**未改动任何代码或配置**
- 每段由 `tools/sitl_harness.sh smoke` 启动独立 SITL 会话

## 文件 → 场景

| 文件 | 场景 | 结果 |
|---|---|---|
| `stage4_mission_20261028_rerun2.log` | G3 全链，**干净起始**（先 `stop` 并等待） | **PASS**：`结果: stage4-mission 全链门禁通过` |
| `stage4_mission_20261028.log` | G3 全链，**上一段会话未 stop 直接接续** | 未通过：悬停窗口 + 包线误判，见下 |
| `stage4_return_20261028_pass.log` | S11~S14 返航专项，干净起始 | **PASS**：迁移 11 次/错误 0、187 间隔 1607ms |
| `stage4_return_20261028_harness.log` | 同上，harness 控制台输出 | — |
| `stage4_return_20261028_attempt1_handover_fallback.log` | 返航专项，**上一段会话未 stop 直接接续** | 未通过：S7 回退 S5、未进 S9/S10，见下 |
| `stage4_tune_20261028_160828.csv` | 返航专项逐拍 CSV（20Hz） | — |
| `gcs.log` / `px4.log` / `px4_g3.log` | 对应会话的 GCS 心跳与 PX4 控制台输出 | — |

当天更早还有两次**干净起始**的 PASS（G3 15:47、S11~S14 15:48），其日志留在 `/tmp/sitl_harness/`，
WSL 重启后被清空（本次重跑就是为了补上证据）。

## 结论：两次未通过都指向「场景之间没有 stop」

同一提交、同一配置下：

- **G3**：干净起始 PASS（迁移 `0→1→2→5→6→7→5`、`结果: stage4-mission 全链门禁通过`）；
  上一段未 stop 接续运行时 FAIL，表现为
  `[SITL][安全回收] 高度/水平位移超包线: 相对高度 1.34701 m，北向 14.362 m`、
  `[FAIL] 相对起飞 1m 稳定悬停完成`、`[FAIL] 目标过期…`、`[FAIL] 飞行未触发高度/漂移安全回收`，
  迁移 6 次。悬停窗口判据是 `z 误差 ≤0.15m / 水平漂移 ≤0.3m / vz ≤0.3m/s`（`kTuneHover*`），
  1.347m 的相对高度说明起飞后姿态/位置初值带着上一会话的余量，悬停窗口自然不满足。
- **S11~S14**：干净起始两次 PASS（迁移 11 次/错误 0、187 触发与复位 ACK accepted、间隔 1607ms，
  与 2026-10-22 的 1596ms 一致）；未 stop 接续那次 `[FAIL] S9 INTERCEPT_READY 进入`、
  `[FAIL] S10 INTERCEPT_EXECUTING 进入`、`[FAIL] 187 触发 ACK accepted`（`触发 ACK=否，复位 ACK=否`），
  逐拍日志显示 S7 启动后**回退 S5**、`射程=16.1185` 恒定：`interception.max_range_m = 8.0`，
  射程始终在窗口外，因此不发 187；随后 S5 停止注入目标/视觉，目标丢失看门狗到点进 S12 返航
  （这条链本身符合设计）。迁移 9 次。

也就是说：**执行器时序、返航链、围栏钳制都没有变化**，两次未通过都是"上一段 SITL 会话的残留状态
带进下一段"造成的假失败。干净起始的复跑即为 PASS。

## 后续收口建议

1. 每段场景前**必须** `bash tools/sitl_harness.sh stop` 并等待数秒（本次教训）。
2. 给 `--sitl-stage4-return` 增加「未进入 S9/S10 时显式标注原因（射程窗口/交接回退）」，
   当前只报 FAIL，需人读逐拍日志才能判断，容易被误读成回归。
3. 让 `smoke` 在启动前做残留检测并拒绝启动（或自动 stop），从工具侧消除这类假失败。

## 复现命令（仓库根目录，WSL2）

```bash
bash tools/sitl_harness.sh stop            # 场景之间必须执行
bash tools/sitl_harness.sh smoke --config config/px4_sitl.json \
  --duration 120 --sitl-stage4-mission
bash tools/sitl_harness.sh stop
bash tools/sitl_harness.sh smoke --config config/px4_sitl_return.json \
  --duration 120 --sitl-stage4-return
bash tools/sitl_harness.sh stop
```

注意：`/tmp/sitl_harness/` 的日志会被下一次 `smoke` 覆盖，WSL 重启还会清空 `/tmp`，
因此证据要当场拷到本目录（本次即因 `/tmp` 被清空而重跑生成）。
