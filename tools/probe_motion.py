"""探针：绕圈/蛇形下量 设计机的距离、参考点领先、预测点相对目标的前后偏离。

用 file:// 打开演示页（Windows 侧 python + playwright），通过页面里的全局 `A`/`P`/`resetSims`
直接改场景参数并逐拍采样。输出每 2 s 一行。
"""
import json
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

URL = (Path("docs") / "示意图" / "追踪梯形曲线-交互演示-2026-11-05.html").resolve().as_uri()

PROBE = r"""
(cfg) => {
  Object.assign(P, cfg);
  if (cfg.src) P.src = cfg.src;
  resetSims();
  const out = [];
  const dt = 0.02;
  for (let i = 0; i < 60 * 50; i++) {
    stepSim(A, dt);
    stepSim(B, dt);
    if (i % 25 === 0) {
      const s = A;
      const vn = Math.hypot(s.tv.x, s.tv.y) || 1e-9;
      const pred = s.est && s.est.pred ? s.est.pred : s.seen;
      // 预测点相对"目标当前点"的沿速度方向投影：正 = 在目标前方，负 = 落在目标后面
      const offAlong = ((pred.x - s.target.x) * s.tv.x + (pred.y - s.target.y) * s.tv.y) / vn;
      // 垂直于速度方向的偏差（绕圈时能看出预测点被甩到圆弧内侧/外侧）
      const offCross = ((pred.x - s.target.x) * s.tv.y - (pred.y - s.target.y) * s.tv.x) / vn;
      out.push({
        t: +s.t.toFixed(1),
        d: +s.d.toFixed(2),
        lag: +s.lag.toFixed(2),
        guideFromDrone: +Math.hypot(s.guide.x - s.drone.x, s.guide.y - s.drone.y).toFixed(2),
        predTruth: +Math.hypot(pred.x - s.target.x, pred.y - s.target.y).toFixed(2),
        offAlong: +offAlong.toFixed(2),
        offCross: +offCross.toFixed(2),
        vref: +s.vref.toFixed(2),
        env: +(Math.max(0, s.d - (P.end === 'match' ? 3 : 0.6))).toFixed(1),
        decel: P.decel, accel: P.accel, vcap: P.vcap, policy: P.policy,
        vd: +Math.hypot(s.vel.x, s.vel.y).toFixed(2),
        phase: s.phase,
        arrived: s.arrived,
        tArrive: s.arrivedAt ? +s.arrivedAt.toFixed(1) : null,
      });
    }
  }
  return out;
}
"""


def run(page, cfg):
    page.evaluate("() => { paused = true; }")
    return page.evaluate(PROBE, cfg)


def main():
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1440, "height": 900})
        page.goto(URL)
        page.wait_for_selector("#scene")
        page.wait_for_function("() => typeof A !== 'undefined'")

        for name, cfg in [
            ("circle vt=4", {"vt": 4.0, "turn": "circle", "src": "kf"}),
            ("weave vt=4", {"vt": 4.0, "turn": "weave", "src": "kf"}),
        ]:
            rows = run(page, cfg)
            print(f"\n=== {name} ===")
            print(f"{'t':>5} {'d':>7} {'lag':>7} {'guide':>7} {'predErr':>8} {'offAl':>7} {'offCr':>7} {'vref':>6} {'vd':>6} phase")
            last = None
            for r in rows:
                if int(r["t"] * 10) % 40 == 0:  # 每 4 s 打一行
                    print(
                        f"{r['t']:>5.1f} {r['d']:>7.2f} {r['lag']:>7.2f} {r['guideFromDrone']:>7.2f} "
                        f"{r['predTruth']:>8.2f} {r['offAlong']:>7.2f} {r['offCross']:>7.2f} "
                        f"{r['vref']:>6.2f} {r['vd']:>6.2f} {r['phase']} "
                        f"accel={r.get('accel')} decel={r.get('decel')} vcap={r.get('vcap')} policy={r.get('policy')}"
                    )
                last = r
            print(f"--- 末态：t={last['t']} d={last['d']} lag={last['lag']} arrived={last['arrived']} tArrive={last['tArrive']}")

        browser.close()


if __name__ == "__main__":
    sys.exit(main())
