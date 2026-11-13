# feedforward_report —— P3 前馈专项 A/B 报告

## 功能边界

读取 `px4_link_smoke --sitl-stage4-feedforward` 生成的逐拍 CSV，统计：

- `phase=feedforward_run` 段的样本数；
- N 轴前馈样本数（`setpoint_ff_valid_mask` bit0 置位）与前馈均值；
- 运动段最后 5s 的北向稳态滞后 `lag_north_m`（目标北向 − 自机北向）：
  正值=自机落后，负值=自机超前；报告均值与 `p95|lag|`；
- 注入地面站目标速度 `target_velocity_north_mps`。

纯 stdlib，不依赖主工程构建产物。

## 用法

```bash
# FF 关闭基线（默认）
./build/px4_link_smoke --config config/px4_sitl.json --duration 90 \
  --sitl-stage4-feedforward --tune-csv logs/ff_off.csv

# FF 打开（命令行只覆盖本进程 controller 开关，不改 JSON）
./build/px4_link_smoke --config config/px4_sitl.json --duration 90 \
  --sitl-stage4-feedforward --feedforward --tune-csv logs/ff_on.csv

python3 tools/feedforward_report.py logs/ff_off.csv logs/ff_on.csv
```

**每次运行前必须重启 PX4 SITL（干净会话）**：目标坐标为 Home 相对绝对坐标，
上一轮降落点残留会让本轮从错误位置起飞（2026-10-19 实测：FF-on 未重启时
自机从 ~41m 北起飞，目标反而在身后，滞后统计失效）。

## 指标与验收

- FF 关闭：N 轴前馈样本应为 0；FF 打开：样本数 > 0 且均值≈注入目标速度；
- 硬判据（工具报告）：S5 接管、位置设定值流建立、运动段结束水平间隙 ≤5m、
  剧本/回收链完整、无安全回收与控制器错误；
- 软判据（本报告）：A/B 的 `p95|lag|` 降低比例。2026-10-19 SITL 实测
  （目标 1.0 m/s、MPC_XY_VEL_MAX=2.0）：

| 运行 | N 轴前馈样本 | 平均前馈 | 稳态滞后均值 | p95\|lag\| | 末端间隙 |
|---|---|---|---|---|---|
| FF off | 0 | 0 | 1.391 m | 1.492 m | 1.453 m |
| FF on | 413 | 1.000 m/s | 0.321 m | 0.400 m | 0.329 m |

p95|lag| 降低约 73%，均值降低约 77%。该专项为非门禁（P3 Phase B），
不替代 G2/G3；G3 以 FF 默认关闭回归。
