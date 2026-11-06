"""量测四种目标速度来源下的接近性能。

来源：
  truth = 完美真值（只验法则本身）
  vmsg  = 地面站显式速度（65010 的 vn/ve/vd），位置带噪带时延，速度带 σ_v 误差
  kf    = 只有位置，速度靠卡尔曼（σ_a 可调）
  raw   = 只有位置，速度靠相邻两点差分

用法：python tools/probe_sources.py
"""
import json
import pathlib
import sys

from playwright.sync_api import sync_playwright

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = pathlib.Path(__file__).resolve().parent.parent
PAGE = (ROOT / "docs" / "示意图" / "追踪梯形曲线-交互演示-2026-11-05.html").as_uri()

RUN = """
({ src, vsig, sigmaA, vt, turn, accel, tmax, seed }) => {
  P.src = src; P.vsig = vsig; P.sigmaA = sigmaA;
  P.vt = vt; P.turn = turn; P.accel = accel; P.law = 'prop';
  P.paused = true; P.startD = 40; P.latency = 200; P.noise = 0.8; P.measHz = 5;
  resetSims();
  // 噪声序列由 s.step 索引决定；偏移起点就换一条噪声实现（同参数多种子跑中位数）。
  A.step = seed * 4096; B.step = seed * 4096;
  // 必须用页面自己的 DT（0.02 s / 50 Hz），否则量测节拍与噪声抽样序列都变了，
  // 结果和真实播放对不上（实测 1/60 会把绕圈 26.6 s 算成 34.9 s）。
  const dt = DT;
  let guard = 0;
  while ((!A.arrived || !B.arrived) && A.t < tmax && guard++ < 40000) {
    if (!A.arrived) stepSim(A, dt);
    if (!B.arrived) stepSim(B, dt);
  }
  const out = (s) => ({
    arrived: !!s.arrived,
    arrivedAt: s.arrivedAt,
    d: s.d,
    vc: s.vc,
    vmax: s.vmax,
    vref: s.vref,
    t: s.t,
  });
  return { A: out(A), B: out(B) };
}
"""

SCENARIOS = [
    ("直线 2 m/s", dict(vt=2.0, turn="straight", accel=1.0)),
    ("绕圈 4 m/s r=20 a=1.5", dict(vt=4.0, turn="circle", accel=1.5)),
    ("蛇形 3 m/s a=1.0", dict(vt=3.0, turn="weave", accel=1.0)),
]

SOURCES = [
    ("真值", dict(src="truth", vsig=0.0, sigmaA=2.0)),
    ("地面站 σ_v=0.2", dict(src="vmsg", vsig=0.2, sigmaA=2.0)),
    ("地面站 σ_v=0.5", dict(src="vmsg", vsig=0.5, sigmaA=2.0)),
    ("卡尔曼 σ_a=2", dict(src="kf", vsig=0.0, sigmaA=2.0)),
    ("卡尔曼 σ_a=8（机载）", dict(src="kf", vsig=0.0, sigmaA=8.0)),
    ("差分", dict(src="raw", vsig=0.0, sigmaA=2.0)),
]


def main():
    rows = []
    with sync_playwright() as pw:
        browser = pw.chromium.launch()
        page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=2)
        errs = []
        page.on("pageerror", lambda e: errs.append(str(e)))
        page.goto(PAGE)
        page.wait_for_function("() => typeof P !== 'undefined' && typeof stepSim === 'function'")
        page.wait_for_timeout(400)
        for name, sc in SCENARIOS:
            for sname, sp in SOURCES:
                times, dists, peaks = [], [], []
                for seed in range(1, 6):
                    args = dict(sp, **sc, tmax=150.0, seed=seed)
                    a = page.evaluate(RUN, args)["A"]
                    dists.append(a["d"])
                    peaks.append(a["vmax"])
                    if a["arrivedAt"] is not None:
                        times.append(a["arrivedAt"])
                times.sort()
                rows.append(
                    {
                        "场景": name,
                        "来源": sname,
                        "到点": f"{len(times)}/5",
                        "用时 s": None if not times else round(times[len(times) // 2], 1),
                        "末距 m": round(sorted(dists)[len(dists) // 2], 2),
                        "峰值": round(max(peaks), 2),
                    }
                )
        browser.close()
    if errs:
        print("页面错误：", errs[:3])
    cur = None
    print(f"{'来源':<22}{'到点':<6}{'用时 s':>8}{'末距 m':>9}{'峰值':>7}")
    for r in rows:
        if r["场景"] != cur:
            cur = r["场景"]
            print("")
            print("— " + cur)
            print(f"{'来源':<22}{'到点':<6}{'用时 s':>8}{'末距 m':>9}{'峰值':>7}")
        use = f"{r['用时 s']:.1f}" if r["用时 s"] is not None else "—"
        print(f"{r['来源']:<22}{r['到点']:<6}{use:>8}{r['末距 m']:>9.2f}{r['峰值']:>7.2f}")


if __name__ == "__main__":
    main()
