"""对比基线（比例接近律）与梯形包络：绕圈场景的加速度/增益敏感度。"""
import json
import pathlib

from playwright.sync_api import sync_playwright

URI = (pathlib.Path("docs") / "示意图" / "追踪梯形曲线-交互演示-2026-11-05.html").resolve().as_uri()

RUN = """
(cfg) => {
  Object.assign(P, cfg);
  resetSims();
  P.paused = true;
  const dt = 1 / 60;
  let t = 0, lastPhase = '', switches = 0, minD = 1e9;
  for (let i = 0; i < 60 * 150; i++) {
    stepSim(A, dt); stepSim(B, dt);
    t += dt;
    if (A.phase !== lastPhase) { switches++; lastPhase = A.phase; }
    if (A.d < minD) minD = A.d;
    if (A.arrived) break;
  }
  return { arrived: !!A.arrived, t: +t.toFixed(1), d: +A.d.toFixed(2),
           vpeak: +(A.vpeak || 0).toFixed(2), switches, phase: A.phase };
}
"""

SCEN = [
    ("line vt=2", dict(vt=2, turn="straight", heading=180, vcap=0, accel=1, decel=1, k=0.6, end="match")),
    ("static 40m", dict(vt=0, turn="straight", heading=90, vcap=0, accel=1, decel=1, k=0.6, end="match")),
    ("weave a=1", dict(vt=3, turn="weave", vcap=0, accel=1.0, decel=1.0, k=0.6, end="match")),
    ("weave a=3", dict(vt=3, turn="weave", vcap=0, accel=3.0, decel=3.0, k=0.6, end="match")),
    ("circle a=1.0", dict(vt=4, turn="circle", vcap=0, accel=1.0, decel=1.0, k=0.6, end="match")),
    ("circle a=1.5", dict(vt=4, turn="circle", vcap=0, accel=1.5, decel=1.5, k=0.6, end="match")),
    ("circle a=2.0", dict(vt=4, turn="circle", vcap=0, accel=2.0, decel=2.0, k=0.6, end="match")),
    ("circle a=3.0", dict(vt=4, turn="circle", vcap=0, accel=3.0, decel=3.0, k=0.6, end="match")),
    ("circle k=0.30", dict(vt=4, turn="circle", vcap=0, accel=1.0, decel=1.0, k=0.30, end="match")),
    ("circle k=1.20", dict(vt=4, turn="circle", vcap=0, accel=1.0, decel=1.0, k=1.20, end="match")),
    ("line k=0.6", dict(vt=2, turn="straight", heading=180, vcap=0, accel=1, decel=1, k=0.6, end="match")),
    ("line k=1.2", dict(vt=2, turn="straight", heading=180, vcap=0, accel=1, decel=1, k=1.2, end="match")),
]

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1440, "height": 900})
    pg.goto(URI)
    pg.wait_for_timeout(600)
    for law in ("prop",):
        print("\n=== " + law + " ===")
        for name, cfg in SCEN:
            pg.evaluate("(a) => { P.law = a; P.profile = 'trap'; P.policy = 'fast'; P.floor = true; }", law)
            out = pg.evaluate(RUN, cfg)
            print("%-22s %s" % (name, json.dumps(out, ensure_ascii=False)))
    b.close()
