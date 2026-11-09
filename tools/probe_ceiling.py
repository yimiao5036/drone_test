"""扫「允许接近速度上限」V_catch：给到点时间 / 峰值 / jerk，回答"上限提到多少才划算"。

制动曲线律、a_a=a_d=2.0、起点 40 m、真值来源。
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
  let t = 0, lastV = 0, maxJerk = 0;
  for (let i = 0; i < 60 * 400; i++) {
    stepSim(A, dt);
    t += dt;
    const steady = t > 2 && A.d > 5;
    if (steady) maxJerk = Math.max(maxJerk, Math.abs(A.vref - lastV) / dt);
    lastV = A.vref;
    if (A.arrived) break;
  }
  return {arrived: !!A.arrived, t: +t.toFixed(1), d: +A.d.toFixed(2),
          vpeak: +(A.vpeak || 0).toFixed(2), jerk: +maxJerk.toFixed(1)};
}
"""

SCEN = [
    ("静止 40m", dict(vt=0, turn="straight", heading=90)),
    ("直线 vt=2", dict(vt=2, turn="straight", heading=180)),
    ("绕圈 4 a=2", dict(vt=4, turn="circle")),
    ("蛇形 3 a=1", dict(vt=3, turn="weave")),
]
CAPS = [2, 4, 6, 8, 10, 12]
BASE = dict(law="brake", profile="trap", policy="fast", floor=True, accel=2.0, decel=2.0,
            k=0.6, end="match", src="truth", lead=0.0, slew=0.0)

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1440, "height": 900})
    pg.goto(URI)
    pg.wait_for_timeout(600)
    for scen, cfg in SCEN:
        print("\n=== %s（制动曲线，a=2.0，起点 40 m） ===" % scen)
        for cap in CAPS:
            out = pg.evaluate(RUN, {**BASE, **cfg, "vcap": cap})
            print("V_catch=%-4g %s" % (cap, json.dumps(out, ensure_ascii=False)))
    b.close()
