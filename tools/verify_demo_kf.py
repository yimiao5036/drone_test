"""验证演示页里的卡尔曼估计与前视预测（2026-11-06）。

断言（默认：σ=1.5 m、5 Hz、时延 200 ms、σ_a=2.0）：
1) 无 JS 报错，量测点按 5 Hz 累积；
2) 估计误差（滤波状态 vs 该量测时刻真值）小于量测误差的 70% —— 滤波器确实在降噪；
3) 预测误差（状态前视 200 ms 到"现在"）小于量测误差的 70% —— 前视补掉了时延；
4) 用机载当前的 σ_a=8.0 时，前视误差反而明显变大 —— 说明这个参数适合"跟位置"，
   不适合"往前推"，要做前视必须重新整定（演示页里可以直接拖 σ_a 看这条曲线）；
5) σ=0 时估计误差≈0（链路本身没有偏置）；
6) 预测点确实在估计点前方（沿目标运动方向，长度 ≈ 估计速度 × 提前量）。

用法（必须在 Windows 侧跑，WSL 里没有 playwright）：
    python tools/verify_demo_kf.py
"""

import pathlib
import sys

from playwright.sync_api import sync_playwright

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEMO = ROOT / "docs" / "示意图" / "追踪梯形曲线-交互演示-2026-11-05.html"
SHOT = ROOT / "tools" / "layout_evidence"


def set_src(page, src):
    """直接切目标位置来源（面板按钮已精简，只剩 真值 / 速度位）。"""
    page.evaluate("(src) => { P.src = src; resetSims(); }", src)


def set_sigma(page, value):
    """σ_a 滑块已从面板移除，直接改参数供卡尔曼回归用。"""
    page.evaluate("(v) => { P.sigmaA = v; resetSims(); }", value)


def set_slider(page, sid, value):
    page.evaluate(
        """([sid, v]) => {
            const el = document.getElementById(sid);
            el.value = String(v);
            el.dispatchEvent(new Event('input', {bubbles: true}));
        }""",
        [sid, value],
    )


def click_seg(page, seg_id, attr, value):
    page.evaluate(
        """([seg, attr, value]) => {
            const b = [...document.querySelectorAll('#' + seg + ' button')]
                .find(x => x.dataset[attr] === value);
            b.click();
        }""",
        [seg_id, attr, value],
    )


def advance(page, seconds):
    page.evaluate("(s) => { P.speed = 8; window.__adv = s; }", seconds)
    page.wait_for_function("(t) => A.t >= t || A.arrived", arg=seconds, timeout=90_000)


def stats(page):
    return page.evaluate(
        """() => {
            const rms = (a, keep) => {
                if (!a.length) return null;
                const arr = a.slice(-keep);
                let s = 0; for (const v of arr) s += v * v;
                return Math.sqrt(s / arr.length);
            };
            const e = A.est;
            return {
                t: A.t,
                em: rms(e.errMeas, 200), ee: rms(e.errEst, 200), ep: rms(e.errPred, 200),
                measCount: e.meas.length,
                wander: pathWander(A.trailD, 3, A.t),
                lead: (e.pred && e.kfX.init)
                    ? Math.hypot(e.pred.x - e.kfX.pos, e.pred.y - e.kfY.pos) : null,
                speedEst: Math.hypot(e.kfX.vel, e.kfY.vel),
            };
        }"""
    )


def main():
    SHOT.mkdir(parents=True, exist_ok=True)
    errors = []
    ok = True

    def check(cond, good, bad):
        nonlocal ok
        if cond:
            print("✓ " + good)
        else:
            ok = False
            print("✗ " + bad)

    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=2)
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.goto(DEMO.as_uri())
        page.wait_for_selector("#scene")
        page.wait_for_timeout(400)

        # 统一工况：目标 2 m/s 直线、梯形、时间最优、无上限、σ=1.5 m、5 Hz、时延 200 ms
        click_seg(page, "policySeg", "policy", "fast")
        set_slider(page, "vcap", 0)
        set_slider(page, "vt", 2)
        set_slider(page, "noise", 1.5)
        set_slider(page, "measHz", 5)
        set_slider(page, "latency", 200)

        # --- 1) 整定后的 σ_a = 2.0
        set_sigma(page, 2.0)
        set_src(page, "kf")
        advance(page, 10)
        tuned = stats(page)
        page.screenshot(path=str(SHOT / "demo-kf-1440.png"))

        # --- 2) 机载当前 σ_a = 8.0
        set_sigma(page, 8.0)
        advance(page, 10)
        onboard = stats(page)

        # --- 3) 只追量测（同一噪声序列）
        set_src(page, "raw")
        advance(page, 10)
        raw = stats(page)

        # --- 4) 无噪声
        set_slider(page, "noise", 0)
        set_src(page, "kf")
        advance(page, 8)
        clean = stats(page)

        # --- 5) 工具条不被裁
        kfbar = page.evaluate(
            """() => {
                const el = document.querySelector('.kfbar');
                return {sw: el.scrollWidth, cw: el.clientWidth};
            }"""
        )

        browser.close()

    print("整定后 σ_a=2.0：量测 %.2f m / 估计 %.2f m / 前视 %.2f m（采样 %d）" % (
        tuned["em"] or -1, tuned["ee"] or -1, tuned["ep"] or -1, tuned["measCount"]))
    print("机载 σ_a=8.0：估计 %.2f m / 前视 %.2f m" % (onboard["ee"] or -1, onboard["ep"] or -1))
    print("只追量测：误差 %.2f m（同一噪声序列）" % (raw["em"] or -1))
    print("无噪声：估计 %.3f m" % (clean["ee"] if clean["ee"] is not None else -1))
    print("预测点领先估计点 %.2f m（估计速度 %.2f m/s × 0.2 s）" % (
        tuned["lead"] or -1, tuned["speedEst"]))
    print("页面错误：", errors or "无")

    check(not errors, "无 JS 报错", "JS 报错：%s" % errors)
    check(tuned["measCount"] > 35, "量测点按 5 Hz 累积（%d 个）" % tuned["measCount"],
          "量测点太少：%d" % tuned["measCount"])
    check(tuned["ee"] is not None and tuned["em"] is not None and tuned["ee"] < tuned["em"] * 0.7,
          "卡尔曼把位置噪声压到量测的 %.0f%%（%.2f→%.2f m）" % (
              100 * tuned["ee"] / tuned["em"], tuned["em"], tuned["ee"]),
          "卡尔曼没有降噪：估计 %.2f m vs 量测 %.2f m" % (tuned["ee"] or -1, tuned["em"] or -1))
    check(tuned["ep"] is not None and tuned["ep"] < tuned["em"] * 0.7,
          "前视 200 ms 后误差 %.2f m，低于过期量测的 %.2f m" % (tuned["ep"], tuned["em"]),
          "前视没有补掉时延：%.2f m vs %.2f m" % (tuned["ep"] or -1, tuned["em"] or -1))
    check(onboard["ep"] is not None and tuned["ep"] is not None
          and onboard["ep"] > tuned["ep"] * 1.25,
          "机载 σ_a=8.0 的前视误差 %.2f m 明显大于整定后的 %.2f m（→ 需重新整定）" % (
              onboard["ep"], tuned["ep"]),
          "σ_a 对前视没有影响？%.2f vs %.2f" % (onboard["ep"] or -1, tuned["ep"] or -1))
    check(clean["ee"] is not None and clean["ee"] < 0.02,
          "无噪声时估计误差≈0（%.3f m）" % (clean["ee"] or -1),
          "无噪声仍有偏置：%.3f m" % (clean["ee"] if clean["ee"] is not None else -1))
    check(kfbar["sw"] <= kfbar["cw"] + 1,
          "「目标位置来源」工具条不溢出（%d/%d）" % (kfbar["sw"], kfbar["cw"]),
          "工具条被裁：%s" % kfbar)

    check(tuned["lead"] is not None and tuned["lead"] > 0.2,
          "预测点在估计点前方 %.2f m" % (tuned["lead"] or -1),
          "预测点没有前视：%.2f m" % (tuned["lead"] or -1))

    print("OK" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
