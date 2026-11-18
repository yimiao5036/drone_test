"""对比三条速度律：比例接近（prop）/ 梯形包络（env）/ 制动曲线（brake）。

输出到点时间、峰值速度、参考速度最大变化率（jerk 代理）与本机加速度峰值，
用来回答"制动曲线律相对现有两律有没有实质改进"。
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
  const dt = DT;                     // 必须用页面自己的步长，伪造 1/60 会改变量测节拍
  let t = 0, lastV = 0, maxJerk = 0, maxA = 0, lastVel = {x: 0, y: 0};
  for (let i = 0; i < 60 * 200; i++) {
    stepSim(A, dt);
    t += dt;
    const steady = t > 2 && A.d > 5;   // 跳过起步阶跃与到点切末速那一下
    if (steady) maxJerk = Math.max(maxJerk, Math.abs(A.vref - lastV) / dt);
    const ax = (A.vel.x - lastVel.x) / dt, ay = (A.vel.y - lastVel.y) / dt;
    if (steady) maxA = Math.max(maxA, Math.hypot(ax, ay));
    lastV = A.vref; lastVel = {x: A.vel.x, y: A.vel.y};
    if (A.arrived) break;
  }
  return {arrived: !!A.arrived, t: +t.toFixed(1), d: +A.d.toFixed(2),
          vpeak: +(A.vpeak || 0).toFixed(2), jerk: +maxJerk.toFixed(1),
          amax: +maxA.toFixed(2)};
}
"""

SCEN = [
    ("直线 vt=2", dict(vt=2, turn="straight", heading=180)),
    ("静止 40m", dict(vt=0, turn="straight", heading=90)),
    ("绕圈 4 a=2", dict(vt=4, turn="circle", accel=2.0, decel=2.0)),
    ("蛇形 3 a=1", dict(vt=3, turn="weave")),
]
BASE = dict(vcap=0, accel=1.0, decel=1.0, k=0.6, end="match", src="truth", lead=0.0)
LAWS = [("prop", "比例接近"), ("env", "梯形包络"), ("brake", "制动曲线")]

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1440, "height": 900})
    pg.goto(URI)
    pg.wait_for_timeout(600)
    for law, name in LAWS:
        print("\n=== %s（%s） ===" % (law, name))
        for scen, cfg in SCEN:
            pg.evaluate("(a) => { P.law = a.law; P.profile = 'trap'; P.policy = 'fast'; P.floor = true; }",
                        {"law": law})
            out = pg.evaluate(RUN, {**BASE, **cfg})
            print("%-14s %s" % (scen, json.dumps(out, ensure_ascii=False)))
    b.close()
