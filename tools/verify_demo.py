"""验证交互演示页的版式与物理（2026-11-05，第二版）。

检查项：
1) 1440×900 下四块区域（舞台 / 读数条 / 右侧面板 / 两张曲线）都不被裁切，
   面板无需滚动即可看到全部控件与按钮；
2) 默认（无速度上限）时峰值由「距离 + 两个加速度」算出，剖面是三角形：
   实际峰值/到点用时与闭式解 theory() 一致（说明"最高速度不是拍出来的"）；
3) 把上限设成 4.0 m/s 后，峰顶被削平、匀速段出现，用时与闭式解仍然一致；
4) 对照组（无余量）追不上，距离停在 +42 m 附近；
5) 保守策略 + 余量 0 时设计机也追不上（证明"余量决定能不能追上"）；
6) 剖面=现状 时参考点会跑到飞机前面（lag 明显变大）；
7) 时间最优比保守快 25% 以上（目标静止 40 m）。

用法（必须在 Windows 侧跑，WSL 里没有 playwright）：
    python tools/verify_demo.py
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
            vpeak: A.vpeak, lag: A.lag, phase: A.phase,
            arrived: A.arrived, arrivedAt: A.arrivedAt,
            bd: B.d, bArrived: B.arrived,
            theory: THEORY ? {vp: THEORY.vp, tTotal: THEORY.tTotal,
                              tCruise: THEORY.tCruise} : null
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
        timeout=60_000,
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

        # --- 2) 默认：无上限 → 三角形，用时与闭式解一致
        run_until(page, 0, "arrived")
        tri = state(page)
        page.screenshot(path=str(SHOT / "demo2-arrived-1440.png"))

        # --- 3) 上限 4.0 → 峰顶削平、出现匀速段
        set_slider(page, "vcap", 4)
        run_until(page, 0, "arrived")
        capped = state(page)

        # --- 4) 对照机追不上
        # --- 5) 保守 + 余量 0
        click_seg(page, "policySeg", "policy", "safe")
        set_slider(page, "margin", 0)
        run_until(page, 55)
        page.wait_for_timeout(200)
        zero = state(page)

        # --- 6) 现状剖面 lag
        set_slider(page, "margin", 1)
        click_seg(page, "profSeg", "prof", "step")
        run_until(page, 6)
        page.wait_for_timeout(100)
        step = state(page)
        page.screenshot(path=str(SHOT / "demo2-step-1440.png"))

        # --- 7) 保守 vs 时间最优（目标静止 40 m）
        click_seg(page, "profSeg", "prof", "trap")
        set_slider(page, "vcap", 0)
        click_seg(page, "policySeg", "policy", "safe")
        set_slider(page, "vt", 0)
        run_until(page, 0, "arrived")
        safe_static = state(page)

        click_seg(page, "policySeg", "policy", "fast")
        run_until(page, 0, "arrived")
        fast_static = state(page)
        page.screenshot(path=str(SHOT / "demo2-fast-static-1440.png"))

        # 复位到梯形跑一段，用于截图
        click_seg(page, "policySeg", "policy", "safe")
        set_slider(page, "vt", 2)
        run_until(page, 8)
        page.wait_for_timeout(200)
        page.screenshot(path=str(SHOT / "demo2-mid-1440.png"))

        browser.close()

    print("版式：", layout)
    print("面板滚动：", panel_overflow)
    print("越界：", clipped or "无")
    print("三角形（无上限）：到点 %.2f s，峰值 %.2f m/s，理论 %.2f s / %.2f m/s，匀速段 %.2f s" % (
        tri["arrivedAt"] or -1, tri["vpeak"], tri["theory"]["tTotal"], tri["theory"]["vp"],
        tri["theory"]["tCruise"]))
    print("上限 4.0：到点 %.2f s，峰值 %.2f m/s，理论 %.2f s / %.2f m/s，匀速段 %.2f s" % (
        capped["arrivedAt"] or -1, capped["vpeak"], capped["theory"]["tTotal"], capped["theory"]["vp"],
        capped["theory"]["tCruise"]))
    print("余量 0（t=%.1f）：设计 d=%.2f vmax=%.2f 对照 d=%.2f" % (zero["t"], zero["d"], zero["vmax"], zero["bd"]))
    print("现状剖面 t=%.1f：lag=%.2f vref=%.2f" % (step["t"], step["lag"], step["vref"]))
    print("目标静止 40 m：保守 %.1f s / 时间最优 %.1f s" % (
        safe_static["arrivedAt"] or -1, fast_static["arrivedAt"] or -1))
    print("页面错误：", errors or "无")

    check(not clipped, "四块区域都不越界", "有区域越界：%s" % clipped)
    check(abs(panel_overflow["scroll"] - panel_overflow["client"]) <= 1,
          "右侧面板无需滚动", "右侧面板被裁：%s" % panel_overflow)
    check(not errors, "无 JS 报错", "JS 报错：%s" % errors)

    check(tri["arrived"], "默认参数能到点", "默认参数未到点")
    check(abs(tri["arrivedAt"] - tri["theory"]["tTotal"]) < 1.0,
          "三角形用时与闭式解一致（%.2f s vs %.2f s）" % (tri["arrivedAt"], tri["theory"]["tTotal"]),
          "三角形用时偏离闭式解（%.2f s vs %.2f s）" % (tri["arrivedAt"], tri["theory"]["tTotal"]))
    check(abs(tri["vpeak"] - tri["theory"]["vp"]) < 0.15,
          "峰值由几何算出（%.2f m/s）" % tri["vpeak"],
          "峰值与闭式解不符（%.2f vs %.2f）" % (tri["vpeak"], tri["theory"]["vp"]))
    check(tri["theory"]["tCruise"] < 0.05,
          "无上限时匀速段为 0（三角形）",
          "无上限却出现匀速段 %.2f s" % tri["theory"]["tCruise"])

    check(abs(capped["vpeak"] - 4.0) < 0.15, "上限 4.0 时峰值被削到 4.0（%.2f）" % capped["vpeak"],
          "上限未生效，峰值 %.2f" % capped["vpeak"])
    check(capped["theory"]["tCruise"] > 0.5,
          "峰顶削平后出现匀速段（%.2f s）" % capped["theory"]["tCruise"],
          "削平后仍未出现匀速段")
    check(abs(capped["arrivedAt"] - capped["theory"]["tTotal"]) < 1.0,
          "带匀速段时用时与闭式解一致（%.2f s vs %.2f s）" % (capped["arrivedAt"], capped["theory"]["tTotal"]),
          "带匀速段时用时偏离闭式解")

    check(abs(tri["bd"] - 42.0) < 1.5, "对照组停在 +%.1f m（追不上）" % tri["bd"],
          "对照组距离异常：%.2f" % tri["bd"])
    check(zero["vmax"] <= 2.01 and zero["d"] > 20,
          "余量 0 时设计机也追不上（vmax=%.2f d=%.1f）" % (zero["vmax"], zero["d"]),
          "余量 0 的对照不符合预期")
    check(safe_static["arrived"] and fast_static["arrived"]
          and fast_static["arrivedAt"] < safe_static["arrivedAt"] * 0.75,
          "时间最优比保守快 %.0f%%（%.1f s vs %.1f s）" % (
              100 * (1 - fast_static["arrivedAt"] / safe_static["arrivedAt"]),
              fast_static["arrivedAt"], safe_static["arrivedAt"]),
          "时间最优没有明显更快")
    check(step["lag"] > 2.0, "现状剖面参考点跑到前面（lag=%.2f）" % step["lag"],
          "现状剖面 lag 未体现")

    print("OK" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
