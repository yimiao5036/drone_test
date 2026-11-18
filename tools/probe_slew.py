"""量化「参考限速」（速度指令变化率上限）对到点瞬间的速度台阶与到点时间的影响。"""
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
  let t = 0, lastV = 0, maxJerk = 0, arrivedV = null, prev = 0;
  for (let i = 0; i < 60 * 200; i++) {
    stepSim(A, dt);
    t += dt;
    if (t > 2) maxJerk = Math.max(maxJerk, Math.abs(A.vref - lastV) / dt);
    lastV = A.vref;
    if (A.arrived) { arrivedV = +A.vref.toFixed(2); break; }
  }
  return {t: +t.toFixed(1), d: +A.d.toFixed(2), jerk: +maxJerk.toFixed(1),
          vpeak: +(A.vpeak || 0).toFixed(2), vArrive: arrivedV};
}
"""

CASES = [
    ("制动·静止 40m", dict(law="brake", vt=0, turn="straight", heading=90)),
    ("制动·直线 2m/s", dict(law="brake", vt=2, turn="straight", heading=180)),
    ("比例·静止 40m", dict(law="prop", vt=0, turn="straight", heading=90)),
    ("比例·直线 2m/s", dict(law="prop", vt=2, turn="straight", heading=180)),
]
BASE = dict(vcap=0, accel=1.0, decel=1.0, k=0.6, end="match", src="truth", lead=0.0)

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1440, "height": 900})
    pg.goto(URI)
    pg.wait_for_timeout(600)
    for name, cfg in CASES:
        row = {}
        for slew in (0, 1.0, 2.0):
            out = pg.evaluate(RUN, {**BASE, **cfg, "slew": slew, "profile": "trap",
                                    "policy": "fast", "floor": True})
            row["slew=%s" % slew] = out
        print("\n=== %s ===" % name)
        for k, v in row.items():
            print("  %-8s %s" % (k, json.dumps(v, ensure_ascii=False)))
    b.close()
