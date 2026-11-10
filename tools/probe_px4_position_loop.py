"""PX4 位置环 + S5 引导点推进的量化对比（一次性分析工具，无第三方依赖）。

用途：回答"只给位置点 / 给目标速度前馈 / 给引导点速度前馈"三种下发的差别，
以及"固定速度上限"对远距离接近时间的代价。模型对齐
`src/control/flight_controller.cpp` 的 PublishPosition 与状态机 ApplyApproachPolicy：

    guide 每 50 ms 从当前位置向目标推进 step = min(cap, sqrt(2*2.0*dist)) * dt
    预测点 = 目标 + v_t*0.5 s，再过 200 ms 一阶低通
    PX4 ：v_sp = v_ff + MPC_XY_P*(p_sp-p)；a = MPC_XY_VEL_P*(v_sp-v)，|a| <= g*tan(45)

参数默认值来自 config/config.json 与 PX4 默认参数表；模型只用于设计取舍，
不是飞行性能预测。对应分析见 docs/接近段速度律-数学分析-2026-11-09.md。

    python tools/probe_px4_position_loop.py
"""
import math

G = 9.81
AMAX = G * math.tan(math.radians(45.0))   # 倾角 45° 的可用水平加速度 ≈ 9.81
KP = 0.95                                 # MPC_XY_P（1/s）
KV = 1.8                                  # MPC_XY_VEL_P_ACC（1/s，需现场核对）
PERIOD = 0.05                             # controller.period_ms
DT = 0.02


def run(guide=True, cap=1.5, ff='none', target_speed=0.0, start_d=40.0,
        guide_accel=2.0, lead_s=0.5, tau_s=0.2, vel_law=False, ref_accel=3.0,
        vcap=10.0, a_d=3.0, R=3.0, margin=1.3, tmax=240.0):
    """跑一次接近过程，返回 (到点时间, 最小距离, 峰值加速度, 末端滞后, 末速)。

    guide=False：位置目标直通（阶跃），即"只给点"的最坏情况；
    ff：'none' 无前馈 / 'target' 前馈目标速度 / 'guide' 前馈引导点速度；
    vel_law=True：不走位置环，直接下速度指令 v_des = v_t + u（相对系制动包络）。
    """
    px = vx = 0.0
    tx = start_d
    gx = px
    lp = tx
    t = 0.0
    max_a = 0.0
    min_d = 1e9
    t_arr = None
    last_ctrl = -1
    ff_hold = 0.0
    prev_gx = gx
    v_sp = 0.0
    ramp = 0.0
    samples = []          # (t, |gx-px|, |v|)，巡航滞后统计
    while t < tmax:
        tx += target_speed * DT
        raw = tx + target_speed * lead_s
        lp += (raw - lp) * (DT / (tau_s + DT)) if tau_s > 0 else 0.0
        if int(t / PERIOD) != last_ctrl:
            last_ctrl = int(t / PERIOD)
            if guide:
                dist = lp - gx
                step = min(cap, math.sqrt(2.0 * guide_accel * abs(dist))) * PERIOD
                gx = lp if abs(dist) <= step else gx + math.copysign(step, dist)
                if ff == 'guide':
                    ff_hold = (gx - prev_gx) / PERIOD
                prev_gx = gx
            else:
                gx = lp
            if ff == 'target':
                ff_hold = target_speed
            if vel_law:
                d = tx - px
                u = min(vcap - target_speed,
                        math.sqrt(2 * (a_d / margin) * max(0.0, d - R)))
                v_des = target_speed + max(0.0, u)
                dv = max(-ref_accel * PERIOD, min(ref_accel * PERIOD, v_des - ramp))
                ramp += dv
                v_sp = ramp
            else:
                v_sp = ff_hold + KP * (gx - px)
        a = KV * (v_sp - vx)
        a = max(-AMAX, min(AMAX, a))
        max_a = max(max_a, abs(a))
        vx += a * DT
        px += vx * DT
        t += DT
        samples.append((t, abs(gx - px), abs(vx)))
        d = tx - px
        min_d = min(min_d, d)
        if t_arr is None and d <= R and abs(vx - target_speed) <= 0.5:
            t_arr = t
            break
    # 巡航段的位置环滞后：取速度 ≥ 90% 峰值且 t ≥ 1 s 的平均值（阶跃/停点阶段不算）
    lag_ss = 0.0
    if not vel_law and samples:
        v_peak = max(s[2] for s in samples)
        cruise = [s[1] for s in samples if s[0] >= 1.0 and s[2] >= 0.9 * v_peak]
        lag_ss = sum(cruise) / len(cruise) if cruise else 0.0
    return dict(t=None if t_arr is None else round(t_arr, 2), min_d=round(min_d, 2),
                a=round(max_a, 2), lag=round(lag_ss, 2), v=round(vx, 2))


def show(name, **kw):
    r = run(**kw)
    print(f'{name:40s} t={str(r["t"]):>6s}s  min_d={r["min_d"]:6.2f}  a_max={r["a"]:5.2f}  '
          f'lag={r["lag"]:5.2f}  v={r["v"]:4.2f}')


def main():
    print('静止目标 40 m')
    show('位置直通（阶跃），只给点', guide=False, cap=99)
    show('位置引导 cap=1.5（现状 approach）', cap=1.5)
    show('位置引导 cap=6（position_guide）', cap=6.0)
    show('位置引导 cap=1.5 + 前馈=引导点速度', cap=1.5, ff='guide')
    print('\n后退目标 2 m/s 40 m')
    show('引导 cap=3（现状），前馈=目标速度', cap=3.0, ff='target', target_speed=2.0)
    show('引导 cap=3，前馈=引导点速度', cap=3.0, ff='guide', target_speed=2.0)
    print('\n后退目标 2 m/s 120 m（"固定上限后远了慢悠悠"）')
    show('引导 cap=3，前馈=目标速度', cap=3.0, ff='target', target_speed=2.0, start_d=120)
    print('\n速度律 v_des = v_t + min(cap-v_t, sqrt(2*a_d/1.3*(d-R)))，参考加速度 3')
    show('40 m，cap=10', vel_law=True, target_speed=2.0, start_d=40, vcap=10)
    show('120 m，cap=10', vel_law=True, target_speed=2.0, start_d=120, vcap=10)
    show('40 m 静止，cap=8', vel_law=True, target_speed=0.0, start_d=40, vcap=8)
    print(f'\n公式：斜坡稳态滞后 e_ss = V/MPC_XY_P = V/0.95；'
          f'加速度饱和门限 e* = a_max/(KP*KV) = {AMAX / (KP * KV):.2f} m')


if __name__ == '__main__':
    main()
