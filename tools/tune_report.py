#!/usr/bin/env python3
"""stage4-tune 整定报告：读取 --sitl-stage4-tune 的 CSV，按阶段/阶跃段计算
三通道整定指标并打印 PASS/FAIL 表。指标定义与验收值见 tools/tune_report.md。

用法：python3 tools/tune_report.py <csv路径>
纯 stdlib 实现；指标一律用发布侧意图 + 注入几何真值计算（不用遥测差分）。
"""

import csv
import math
import sys
from dataclasses import dataclass, field

# ---- 验收阈值（与 tools/tune_report.md 同源，改动须同步） ----
ANGLE_SETTLE_DEG = 0.5        # 视线角收敛带
ANGLE_SETTLE_HOLD_S = 2.0     # 收敛保持时长
ANGLE_SETTLE_LIMIT_S = 4.0    # 航向收敛时间上限
VERT_SETTLE_LIMIT_S = 8.0     # 垂直收敛时间上限（P-on-视线角渐近收敛 +
                              # 姿态瞬态耦合限制增益上限，判据放宽，见 tune_report.md）
ANGLE_SIGN_CHANGE_MAX = 1     # β/α 变号次数上限
ANGLE_STEADY_DEG = 0.5        # 稳态均值上限
YAW_RATE_LIMIT_DPS = 30.0     # 航向角速度限幅
VZ_LIMIT_MPS = 2.0            # 垂直速度限幅
DIST_OVERSHOOT_MARGIN_M = 0.3  # 零过冲余量（range_true ≥ d_exp − 余量）
DIST_SETTLE_BAND_M = 0.5      # 距离收敛带
DIST_SETTLE_LIMIT_S = 8.0     # 距离收敛时间上限
DIST_STEADY_ERR_M = 0.3       # 距离稳态误差上限
VX_SIGN_CHANGE_PER_5S_MAX = 2  # 防积分极限环
INTENT_DUTY_MIN = 0.99        # S7 意图占空比下限


@dataclass
class Segment:
    phase: str
    rows: list = field(default_factory=list)

    @property
    def t0(self):
        return self.rows[0]["t_ms"] / 1000.0


def load_segments(path):
    """按 phase + 目标换位切分段。"""
    segments = []
    current = None
    prev_key = None
    with open(path, newline="") as fp:
        for row in csv.DictReader(fp):
            row["t_ms"] = int(row["t_ms"])  # 毫秒时间戳转数值（其余列按需 float()）
            key = (row["phase"], row["target_north_m"], row["target_east_m"],
                   row["target_up_m"])
            if key != prev_key:
                current = Segment(phase=row["phase"])
                segments.append(current)
                prev_key = key
            current.rows.append(row)
    return segments


def f(row, name):
    try:
        return float(row[name])
    except (ValueError, KeyError):
        return math.nan


def settle_time(rows, value_name, band, hold_s):
    """|value| 进入 band 并保持 hold_s 的最早时刻（相对段起点秒数）；未收敛返回 None。"""
    t0 = rows[0]["t_ms"]
    i = 0
    while i < len(rows):
        if abs(f(rows[i], value_name)) <= band:
            j = i
            while j < len(rows) and abs(f(rows[j], value_name)) <= band:
                j += 1
            hold_ms = (rows[j - 1]["t_ms"] - rows[i]["t_ms"]) if j > i else 0
            if hold_ms >= hold_s * 1000:
                return (rows[i]["t_ms"] - t0) / 1000.0
            i = j
        else:
            i += 1
    return None


def sign_changes(rows, value_name, band=0.05, excursion=0.5):
    """有效变号次数：过零且在另一侧越出 excursion 才计（防零点附近的估计噪声
    把"单次小超调"刷成多次变号；2026-10-14 首轮实测：±0.2° 的 EKF 航向噪声
    会在 0.05° 死区口径下产生 3~6 次伪变号）。"""
    changes = 0
    last_sign = 0
    overshoot_armed = False
    for row in rows:
        value = f(row, value_name)
        if math.isnan(value) or abs(value) < band:
            continue
        sign = 1 if value > 0 else -1
        if last_sign != 0 and sign != last_sign:
            overshoot_armed = True  # 过零，等待是否构成有效反冲
        if overshoot_armed and abs(value) >= excursion:
            changes += 1
            overshoot_armed = False
        last_sign = sign
    return changes


def steady_mean(rows, value_name, tail_s=5.0):
    """段末 tail_s 秒的 |值| 均值。"""
    t_end = rows[-1]["t_ms"]
    tail = [abs(f(r, value_name)) for r in rows
            if r["t_ms"] >= t_end - tail_s * 1000 and not math.isnan(f(r, value_name))]
    return sum(tail) / len(tail) if tail else math.nan


def max_abs(rows, value_name):
    values = [abs(f(r, value_name)) for r in rows if not math.isnan(f(r, value_name))]
    return max(values) if values else math.nan


def check(name, value_text, ok):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {value_text}")
    return ok


def report_angle_channel(label, segments, phase_name, angle_col, rate_col, rate_limit,
                         settle_limit_s=ANGLE_SETTLE_LIMIT_S, excursion=0.5):
    """航向/垂直通道报告（非零目标偏置段为激励段）。"""
    print(f"\n===== {label}（{phase_name} 段）=====")
    all_ok = True
    stimulus = [s for s in segments if s.phase == phase_name and len(s.rows) >= 100]
    if not stimulus:
        print("  [FAIL] 未找到该通道的阶跃段数据")
        return False
    for seg in stimulus:
        offset = f(seg.rows[0], "target_east_m") if phase_name == "step_lateral" \
            else f(seg.rows[0], "target_up_m") - 1.0
        tag = f"段@t={seg.t0:.0f}s 偏置={offset:+.1f}"
        settle = settle_time(seg.rows, angle_col, ANGLE_SETTLE_DEG, ANGLE_SETTLE_HOLD_S)
        all_ok &= check(f"{tag} 收敛时间 ≤{settle_limit_s}s",
                        "未收敛" if settle is None else f"{settle:.2f}s",
                        settle is not None and settle <= settle_limit_s)
        # 变号判据只对激励段评估：偏置=0 的回位/入口过渡段混入换段瞬态
        # （上段运动残余 + 姿态-几何耦合），不代表通道自身的振荡特性
        if abs(offset) >= 0.1:
            changes = sign_changes(seg.rows, angle_col, excursion=excursion)
            all_ok &= check(f"{tag} 变号次数 ≤{ANGLE_SIGN_CHANGE_MAX}", f"{changes}",
                            changes <= ANGLE_SIGN_CHANGE_MAX)
        steady = steady_mean(seg.rows, angle_col)
        all_ok &= check(f"{tag} 稳态 |角| ≤{ANGLE_STEADY_DEG}°", f"{steady:.2f}°",
                        not math.isnan(steady) and steady <= ANGLE_STEADY_DEG)
        peak = max_abs(seg.rows, rate_col)
        all_ok &= check(f"{tag} |指令| ≤{rate_limit}", f"{peak:.2f}",
                        not math.isnan(peak) and peak <= rate_limit)
    return all_ok


def report_distance_channel(segments, d_exp):
    print(f"\n===== 距离通道（step_range 段，d_exp={d_exp}m）=====")
    all_ok = True
    stimulus = [s for s in segments if s.phase == "step_range" and len(s.rows) >= 100]
    if not stimulus:
        print("  [FAIL] 未找到距离阶跃段数据")
        return False
    for seg in stimulus:
        target_north = f(seg.rows[0], "target_north_m")
        tag = f"段@t={seg.t0:.0f}s N={target_north:.0f}"
        ranges = [f(r, "range_true_m") for r in seg.rows
                  if not math.isnan(f(r, "range_true_m"))]
        min_range = min(ranges, default=math.nan)
        # 零过冲判据只对"接近段"成立（段起点射程 > d_exp：目标比我们远，
        # 接近过程不得冲过 d_exp）；目标向自机跳近（段起点已在 d_exp 内）
        # 时射程天然 < d_exp，正确行为是后退重建射程，由收敛判据覆盖。
        approach_segment = not ranges or ranges[0] > d_exp + DIST_SETTLE_BAND_M
        if approach_segment:
            all_ok &= check(
                f"{tag} 零过冲（range ≥ {d_exp - DIST_OVERSHOOT_MARGIN_M:.1f}）",
                f"min={min_range:.2f}m",
                not math.isnan(min_range) and
                min_range >= d_exp - DIST_OVERSHOOT_MARGIN_M)
        else:
            print(f"  [跳过] {tag} 目标向自机跳近（起点射程 {ranges[0]:.1f}m < "
                  f"d_exp），零过冲不适用，由收敛/稳态判据覆盖")
        err_rows = [{"t_ms": r["t_ms"], "err": abs(f(r, "distance_m") - d_exp)}
                    for r in seg.rows]
        settle = settle_time(err_rows, "err", DIST_SETTLE_BAND_M, ANGLE_SETTLE_HOLD_S)
        all_ok &= check(f"{tag} 收敛(±{DIST_SETTLE_BAND_M}m) ≤{DIST_SETTLE_LIMIT_S}s",
                        "未收敛" if settle is None else f"{settle:.2f}s",
                        settle is not None and settle <= DIST_SETTLE_LIMIT_S)
        steady = steady_mean(
            [{"t_ms": r["t_ms"], "err": abs(f(r, "distance_m") - d_exp)}
             for r in seg.rows], "err")
        all_ok &= check(f"{tag} 稳态误差 ≤{DIST_STEADY_ERR_M}m", f"{steady:.2f}m",
                        not math.isnan(steady) and steady <= DIST_STEADY_ERR_M)
        changes = sign_changes(seg.rows, "intent_vx_mps", band=0.05, excursion=0.2)
        all_ok &= check(f"{tag} vx 变号 ≤{VX_SIGN_CHANGE_PER_5S_MAX} 次", f"{changes}",
                        changes <= VX_SIGN_CHANGE_PER_5S_MAX)
        # 控制器钳制后（发布侧设定值）速度 ≤ max_speed=2.0；意图侧上限是
        # approach_velocity_limit=3.0（钳制在 FlightController，属设计行为）
        peak = max_abs(seg.rows, "setpoint_vx")
        all_ok &= check(f"{tag} |vx| ≤2.0（控制器钳制后设定值）", f"{peak:.2f}",
                        not math.isnan(peak) and peak <= 2.0 + 1e-6)
    return all_ok


def report_cross_cutting(segments):
    """横切指标：S7 驻留/阶跃段意图占空比与刹车意图计数。"""
    print("\n===== 横切（S7 驻留/阶跃段） =====")
    rows = [r for s in segments if s.phase in
            ("dwell_static", "step_lateral", "step_range", "step_vertical")
            for r in s.rows]
    if not rows:
        print("  [FAIL] 无 S7 驻留段数据")
        return False
    present = sum(1 for r in rows if r["intent_present"] == "1")
    duty = present / len(rows)
    ok = check(f"意图占空比 ≥{INTENT_DUTY_MIN:.0%}", f"{duty:.1%}", duty >= INTENT_DUTY_MIN)
    # intent_type: ControlIntentType 枚举值（types.h）：kBrakeHover=3
    brakes = sum(1 for r in rows if r["intent_present"] == "1" and r["intent_type"] == "3")
    ok &= check("驻留段零刹车意图", f"{brakes} 条", brakes == 0)
    return ok


def main():
    if len(sys.argv) != 2:
        print("用法: python3 tools/tune_report.py <stage4_tune CSV>")
        return 1
    segments = load_segments(sys.argv[1])
    if not segments:
        print("CSV 为空或无法解析")
        return 1
    d_exp = math.nan
    for seg in segments:
        for row in seg.rows:
            if not math.isnan(f(row, "d_exp_m")):
                d_exp = f(row, "d_exp_m")
                break
        if not math.isnan(d_exp):
            break
    print(f"样本: {sum(len(s.rows) for s in segments)} 行，"
          f"阶段: {' '.join(dict.fromkeys(s.phase for s in segments))}")
    ok = True
    ok &= report_angle_channel("航向通道", segments, "step_lateral", "beta_deg",
                               "intent_yaw_rate_dps", YAW_RATE_LIMIT_DPS)
    # 垂直通道的 α 噪声带由姿态估计噪声决定（SITL 实测 pitch 漂移 ±0.5°），
    # 变号判据的反冲阈值放宽到 1.0° 防噪声误计
    ok &= report_angle_channel("垂直通道", segments, "step_vertical", "alpha_deg",
                               "intent_vz_mps", VZ_LIMIT_MPS, VERT_SETTLE_LIMIT_S,
                               excursion=1.0)
    ok &= report_distance_channel(segments, d_exp)
    ok &= report_cross_cutting(segments)
    print(f"\n总体: {'全部指标达标' if ok else '存在不达标项'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
