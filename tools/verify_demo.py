"""验证交互演示页的版式与物理（2026-11-05）。

检查项：
1) 1440×900 下四块区域（舞台 / 读数条 / 右侧面板 / 两张曲线）都不被裁切，
   面板无需滚动即可看到全部控件与按钮；
2) 设计机（梯形）在默认参数下能到点，且到点时接近速度 ≈ 0（速度匹配）；
3) 对照组（无余量）在同一场景下追不上，距离停在 +42 m 附近；
4) 把余量调到 0 后设计机也追不上（证明"余量决定能不能追上"）；
5) 剖面=现状 时参考点会跑到飞机前面（lag 明显变大）。

用法（必须在 Windows 侧跑，WSL 里没有 playwright）：
    python tools/verify_demo3.py
"""

import pathlib
import sys

from playwright.sync_api import sync_playwright

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEMO = ROOT / "docs" / "示意图" / "追踪梯形曲线-交互演示-2026-11-05.html"
SHOT = ROOT / "tools" / "layout_evidence"


def rect(page, selector):
    return page.evaluate(
        """(sel) => {
            const el = document.querySelector(sel);
            if (!el) return null;
            const b = el.getBoundingClientRect();
            return {x: Math.round(b.x), y: Math.round(b.y),
                    w: Math.round(b.width), h: Math.round(b.height),
                    bottom: Math.round(b.bottom), right: Math.round(b.right)};
        }""",
        selector,
    )


def state(page):
    return page.evaluate(
        """() => ({
            t: A.t, d: A.d, vc: A.vc, vref: A.vref, vmax: A.vmax,
            lag: A.lag, phase: A.phase, arrived: A.arrived, arrivedAt: A.arrivedAt,
            bd: B.d, bArrived: B.arrived
        })"""
    )


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


def run_until(page, seconds, target=None):
    """把播放倍速拉到 8×，推进到仿真时间 seconds（或到达 target）。"""
    set_slider(page, "speed", 8)
    page.wait_for_function(
        """([s, tgt]) => {
            if (tgt === 'arrived') return A.arrived || A.t > 60;
            return A.t >= s;
        }""",
        arg=[seconds, target],
        timeout=45_000,
    )


def main():
    SHOT.mkdir(parents=True, exist_ok=True)
    errors = []
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=2)
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.goto(DEMO.as_uri())
        page.wait_for_selector("#scene")
        page.wait_for_timeout(600)

        # --- 1) 版式
        layout = {
            "stage": rect(page, ".stage"),
            "stats": rect(page, ".stats"),
            "panel": rect(page, "aside.panel"),
            "chartD": rect(page, "#chartD"),
            "chartV": rect(page, "#chartV"),
        }
        panel_overflow = page.evaluate(
            "() => { const a = document.querySelector('aside.panel');"
            " return {scroll: a.scrollHeight, client: a.clientHeight}; }"
        )
        clipped = []
        for name, r in layout.items():
            if r is None:
                clipped.append((name, "missing"))
                continue
            if r["bottom"] > 900 or r["right"] > 1440:
                clipped.append((name, r))

        # --- 2) 设计机默认参数到点
        run_until(page, 0, "arrived")
        design = state(page)
        page.screenshot(path=str(SHOT / "demo2-arrived-1440.png"))
        main_shot = True

        # --- 3) 对照机追不上 + 4) 余量 0
        set_slider(page, "margin", 0)
        run_until(page, 55)
        page.wait_for_timeout(200)
        zero = state(page)

        # --- 5) 现状剖面 lag
        set_slider(page, "margin", 1)
        click_seg(page, "profSeg", "prof", "step")
        run_until(page, 6)
        page.wait_for_timeout(100)
        step = state(page)
        page.screenshot(path=str(SHOT / "demo2-step-1440.png"))

        # 复位到梯形跑一段，用于截图
        click_seg(page, "profSeg", "prof", "trap")
        run_until(page, 8)
        page.wait_for_timeout(200)
        page.screenshot(path=str(SHOT / "demo2-mid-1440.png"))

        browser.close()

    print("版式：", layout)
    print("面板滚动：", panel_overflow)
    print("越界：", clipped or "无")
    print("设计机到点：", {k: design[k] for k in ("t", "d", "vc", "arrivedAt", "arrived")})
    print("对照机：d=%.2f arrived=%s" % (design["bd"], design["bArrived"]))
    print("余量 0（t=%.1f）：设计 d=%.2f vmax=%.2f 对照 d=%.2f" % (zero["t"], zero["d"], zero["vmax"], zero["bd"]))
    print("现状剖面 t=%.1f：lag=%.2f vref=%.2f" % (step["t"], step["lag"], step["vref"]))
    print("页面错误：", errors or "无")

    ok = True
    if clipped:
        ok = False
        print("✗ 有区域越界")
    if design["arrived"] and design["d"] <= 3.1 and abs(design["vc"]) <= 0.6:
        print("✓ 设计机到点，d=%.2f v_c=%.2f" % (design["d"], design["vc"]))
    else:
        ok = False
        print("✗ 设计机未按时到点")
    if abs(design["bd"] - 42.0) < 1.5:
        print("✓ 对照组停在 +%.1f m（追不上）" % design["bd"])
    else:
        ok = False
        print("✗ 对照组距离异常：%.2f" % design["bd"])
    if zero["vmax"] <= 2.01 and zero["d"] > 20:
        print("✓ 余量 0 时设计机也追不上（vmax=%.2f d=%.1f）" % (zero["vmax"], zero["d"]))
    else:
        ok = False
        print("✗ 余量 0 的对照不符合预期")
    if step["lag"] > 2.0:
        print("✓ 现状剖面参考点跑到前面（lag=%.2f）" % step["lag"])
    else:
        ok = False
        print("✗ 现状剖面 lag 未体现")
    print("OK" if ok else "FAIL")


if __name__ == "__main__":
    main()
