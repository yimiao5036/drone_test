"""绕圈 / 蛇形的稳态复核 + 截图。

量三件事：
1. 能不能到点（绕圈在 a=1、v_t=4、r=20 m 时应当能在 15 s 左右追上）；
2. 蛇形追不上的原因是不是「横向加速度不够」（把 a 提到 3 复测）；
3. 瞄准点相对目标的前后偏移（正值 = 在目标前方）。
"""
import pathlib
import sys

from playwright.sync_api import sync_playwright

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEMO = ROOT / "docs" / "示意图" / "追踪梯形曲线-交互演示-2026-11-05.html"
SHOT = ROOT / "tools" / "layout_evidence"

RUN = r"""
(cfg) => {
  Object.assign(P, cfg);
  P.paused = true;
  resetSims();
  let tArrive = null;
  const samples = [];
  for (let i = 0; i < 60 * 50; i++) {
    stepSim(A, 0.02); stepSim(B, 0.02);
    if (A.arrived && tArrive === null) tArrive = +A.arrivedAt.toFixed(1);
    if (i % 25 === 0 && !A.arrived) {
      const s = A, vn = Math.hypot(s.tv.x, s.tv.y) || 1e-9;
      const aim = s.est && s.est.aim ? s.est.aim : s.seen;
      samples.push({
        t: +s.t.toFixed(0),
        d: +s.dReal.toFixed(2),
        vref: +s.vref.toFixed(2),
        vd: +Math.hypot(s.vel.x, s.vel.y).toFixed(2),
        aimAhead: +(((aim.x - s.target.x) * s.tv.x + (aim.y - s.target.y) * s.tv.y) / vn).toFixed(2),
        aimErr: +Math.hypot(aim.x - s.target.x, aim.y - s.target.y).toFixed(2),
        lag: +s.lag.toFixed(2),
        tGo: +(s.tGo || 0).toFixed(2),
      });
    }
    if (A.arrived) break;
  }
  return { tArrive, samples, t: +A.t.toFixed(1), d: +A.dReal.toFixed(2) };
}
"""


def summarize(name, res):
    print(f"\n=== {name} ===")
    if res["tArrive"] is not None:
        print(f"到点：{res['tArrive']} s（末态 d={res['d']} m）")
    else:
        print(f"未到点：跑满 {res['t']} s，末态 d={res['d']} m")
    rows = res["samples"]
    tail = rows[-6:]
    print(f"{'t':>4} {'d':>7} {'vref':>6} {'vd':>6} {'瞄准超前':>9} {'瞄准误差':>9} {'lag':>5} {'t_go':>5}")
    for r in tail:
        print(
            f"{r['t']:>4} {r['d']:>7.2f} {r['vref']:>6.2f} {r['vd']:>6.2f} "
            f"{r['aimAhead']:>9.2f} {r['aimErr']:>9.2f} {r['lag']:>5.2f} {r['tGo']:>5.2f}"
        )
    if not res["tArrive"]:
        ds = [r["d"] for r in rows if r["t"] > 20]
        if ds:
            print(f"稳态视距：中位 {sorted(ds)[len(ds)//2]:.2f} m，范围 {min(ds):.2f}~{max(ds):.2f} m")


def main():
    SHOT.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=2)
        page.goto(DEMO.as_uri())
        page.wait_for_function("() => typeof A !== 'undefined'")

        cases = [
            ("绕圈 vt=4 a=1（默认）", {"vt": 4.0, "turn": "circle", "src": "kf"}, "demo-circle-1440.png"),
            ("蛇形 vt=4 a=1（横向加速度不足）", {"vt": 4.0, "turn": "weave", "src": "kf"}, "demo-weave-1440.png"),
            ("蛇形 vt=4 a=3（加速度够）", {"vt": 4.0, "turn": "weave", "src": "kf", "accel": 3.0}, "demo-weave-accel3-1440.png"),
        ]
        for name, cfg, shot in cases:
            res = page.evaluate(RUN, cfg)
            summarize(name, res)
            page.evaluate("(cfg) => { Object.assign(P, cfg); P.paused = false; resetSims(); }", cfg)
            page.wait_for_timeout(2500)
            page.screenshot(path=str(SHOT / shot))

        browser.close()
    print("\n截图：tools/layout_evidence/demo-{circle,weave,weave-accel3}-1440.png")


if __name__ == "__main__":
    main()
