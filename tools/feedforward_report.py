#!/usr/bin/env python3
"""stage4-feedforward P3 专项 A/B 报告：读取 --sitl-stage4-feedforward 的 CSV，
统计前馈样本与 S5 运动段最后 5s 的北向稳态滞后。纯 stdlib。

用法：
    python3 tools/feedforward_report.py <ff_off.csv> [ff_on.csv]

指标口径（与 tools/px4_link_smoke.md 的 P3 专项一致）：
- 只取 phase=feedforward_run 的行；
- FF 样本：setpoint_ff_valid_mask 的 bit0（N 轴）置位的逐拍行；
- 稳态滞后：运动段最后 5s 的 lag_north_m（目标北向 − 自机北向），
  正值=自机落后，负值=自机超前；报告均值与 p95|lag|；
- 目标速度：target_velocity_north_mps（注入地面站速度）。

退出码：0=读取成功；2=参数/文件错误。
"""

import csv
import math
import sys
from dataclasses import dataclass

STEADY_WINDOW_MS = 5000


@dataclass
class RunStats:
    path: str
    rows: int
    ff_samples: int
    ff_average_mps: float
    lag_mean_m: float
    lag_p95_abs_m: float
    final_lag_m: float
    target_speed_mps: float


def f(row, name):
    value = row.get(name, "")
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def load_run(path):
    with open(path, newline="") as fp:
        rows = [row for row in csv.DictReader(fp)
                if row.get("phase") == "feedforward_run"]
    if not rows:
        raise ValueError(f"{path}: 没有 phase=feedforward_run 的行")
    for row in rows:
        row["_t_ms"] = int(row["t_ms"])

    t_end = rows[-1]["_t_ms"]
    cutoff = t_end - STEADY_WINDOW_MS
    steady = [row for row in rows if row["_t_ms"] >= cutoff]
    lag = [f(row, "lag_north_m") for row in steady]
    lag = [value for value in lag if math.isfinite(value)]
    if not lag:
        raise ValueError(f"{path}: 稳态窗口内没有有效 lag_north_m")

    ff_rows = [row for row in rows
               if int(f(row, "setpoint_ff_valid_mask") or 0) & 0x01]
    ff_values = [f(row, "setpoint_ff_vx_mps") for row in ff_rows]
    ff_values = [value for value in ff_values if math.isfinite(value)]
    target_speed = f(rows[-1], "target_velocity_north_mps")
    abs_lag = sorted(abs(value) for value in lag)
    p95_index = max(0, math.ceil(0.95 * len(abs_lag)) - 1)
    return RunStats(
        path=path,
        rows=len(rows),
        ff_samples=len(ff_rows),
        ff_average_mps=(sum(ff_values) / len(ff_values)) if ff_values else 0.0,
        lag_mean_m=sum(lag) / len(lag),
        lag_p95_abs_m=abs_lag[p95_index],
        final_lag_m=f(rows[-1], "lag_north_m"),
        target_speed_mps=target_speed,
    )


def print_run(stats):
    ff_mode = "ON" if stats.ff_samples > 0 else "OFF"
    print(f"CSV: {stats.path}")
    print(f"  FF={ff_mode}  运动段样本={stats.rows}  "
          f"目标速度={stats.target_speed_mps:.2f} m/s")
    print(f"  N 轴前馈样本={stats.ff_samples}  平均前馈={stats.ff_average_mps:.3f} m/s")
    print(f"  稳态滞后（最后 {STEADY_WINDOW_MS // 1000}s，北向）："
          f"均值={stats.lag_mean_m:.3f} m  p95|lag|={stats.lag_p95_abs_m:.3f} m  "
          f"末端 lag={stats.final_lag_m:.3f} m")


def main(argv):
    if len(argv) not in (2, 3):
        print(__doc__.strip())
        return 2
    try:
        runs = [load_run(path) for path in argv[1:]]
    except (OSError, ValueError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 2
    for stats in runs:
        print_run(stats)
    if len(runs) == 2:
        off, on = runs
        baseline = off.lag_p95_abs_m
        if baseline > 0.0:
            reduction = (baseline - on.lag_p95_abs_m) / baseline * 100.0
            print(f"A/B: p95|lag| {baseline:.3f} m → {on.lag_p95_abs_m:.3f} m "
                  f"（降低 {reduction:.1f}%）")
        else:
            print("A/B: 基线 p95|lag| 为 0，无法计算降低比例")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
