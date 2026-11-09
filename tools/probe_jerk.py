"""对比 jerk 限幅开关对接近过程的影响。

关注三件事：
1. 到点时间差多少；
2. 指令加速度峰值与 jerk 峰值（jerk 限幅应该把后者钳住）；
3. 同样的参数在不同起始距离下，实际达到的加速度是不是自然不同（用户关心的"死板"）。

用法：python tools/probe_jerk.py
"""
import json
import pathlib
import sys

from playwright.sync_api import sync_playwright

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

URI = (pathlib.Path("docs") / "示意图" / "追踪梯形曲线-交互演示-2026-11-05.html").resolve().as_uri()

RUN = """
(cfg) => {
  Object.assign(P, cfg);
  resetSims();
  P.paused = true;
  const dt = DT;
  let t = 0, lastV = 0, lastA = 0, maxJerk = 0, maxA = 0, maxACmd = 0, aAt05 = 0;
  for (let i = 0; i < 60 * 400; i++) {
    stepSim(A, dt);
    t += dt;
    const vx = A.vel.x, vy = A.vel.y;
    const ax = (vx - lastV) / dt, ay = (vy - 0) / dt;   // 仅用于 jerk 统计近似
    if (t > 0.3 && t < 0.35) aAt05 = Math.hypot(A.acc.x, A.acc.y);
    maxJerk = Math.max(maxJerk, Math.abs(Math.hypot(A.acc.x, A.acc.y) - lastA) / dt);
    maxA = Math.max(maxA, Math.hypot(A.acc.x, A.acc.y));
    maxACmd = Math.max(maxACmd, A.aCmd || 0);
    lastA = Math.hypot(A.acc.x, A.acc.y);
    lastV = vx;
    if (A.arrived) break;
  }
  return {arrived: !!A.arrived, t: +t.toFixed(2), d: +A.d.toFixed(2),
          vpeak: +(A.vpeak || 0).toFixed(2), aCmdMax: +maxACmd.toFixed(2),
          aJerk: +maxJerk.toFixed(1), aAt05: +aAt05.toFixed(2)};
}
"""

SCEN = [
    ("静止 40 m", dict(vt=0, turn="straight", heading=90, startD=40)),
    ("静止 15 m", dict(vt=0, turn="straight", heading=90, startD=15)),
    ("直线 2 m/s 40 m", dict(vt=2, turn="straight", heading=180, startD=40)),
    ("绕圈 4 m/s a=3", dict(vt=4, turn="circle", startD=40)),
]
BASE = dict(law="brake", profile="trap", policy="fast", floor=True, floorOn=True,
            accel=3.0, decel=3.0, slew=3.0, vcap=10, k=0.6, end="match",
            src="truth", lead=0.0)
JERKS = [0, 4, 8, 16]

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1440, "height": 900})
    pg.goto(URI)
    pg.wait_for_timeout(600)
    for scen, cfg in SCEN:
        print("\n=== %s（a_cap=3、V_catch=10） ===" % scen)
        for j in JERKS:
            out = pg.evaluate(RUN, {**BASE, **cfg, "jerk": j})
            label = "jerk=关闭" if j == 0 else "jerk=%-4g" % j
            print("  %-10s %s" % (label, json.dumps(out, ensure_ascii=False)))
    b.close()
