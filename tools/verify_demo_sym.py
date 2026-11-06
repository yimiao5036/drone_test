"""校验演示页「符号表」覆盖层：收起/展开两种状态的版式。

在 Windows 侧跑（playwright 只装在 Windows python 里）：

    python tools/verify_demo_sym.py

判定：按钮在顶栏、不挤占舞台；点开后覆盖层居中、20 条两列不重叠、
卡片不超出视口；Esc 与「关闭」都能收起并让舞台复原；无 console 错误。
截图落在 tools/layout_evidence/。
"""

from __future__ import annotations

import pathlib
import sys

from playwright.sync_api import sync_playwright

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = pathlib.Path(__file__).resolve().parent.parent
PAGE = ROOT / "docs" / "示意图" / "追踪梯形曲线-交互演示-2026-11-05.html"
OUT = ROOT / "tools" / "layout_evidence"
OUT.mkdir(parents=True, exist_ok=True)

MEASURE = r"""
() => {
  const q = (s) => document.querySelector(s);
  const box = (el) => { if (!el) return null; const b = el.getBoundingClientRect();
    return {x: Math.round(b.x), y: Math.round(b.y), w: Math.round(b.width), h: Math.round(b.height),
            r: Math.round(b.right), b: Math.round(b.bottom)}; };
  const modal = q('#symModal');
  const grid = q('#symModal .symgrid');
  const items = grid ? [...grid.children].map(el => el.getBoundingClientRect()) : [];
  const overlap = items.some((a, i) => items.some((b, j) => i < j &&
    !(a.right <= b.left || b.right <= a.left || a.bottom <= b.top || b.bottom <= a.top)));
  const card = q('#symModal .symcard');
  return {
    stage: box(q('.stage')), stats: box(q('.stats')), btn: box(q('#symBtn')),
    open: !!modal && !modal.hidden,
    card: box(card), bodyScroll: card ? card.querySelector('.symbody').scrollHeight : 0,
    bodyClient: card ? card.querySelector('.symbody').clientHeight : 0,
    items: items.length, overlap,
    docScroll: document.documentElement.scrollHeight, winH: window.innerHeight,
    overflowX: document.documentElement.scrollWidth > window.innerWidth + 1,
  };
}
"""


def main() -> int:
    ok = True
    errs: list[str] = []

    def check(cond: bool, good: str, bad: str) -> None:
        nonlocal ok
        ok = ok and bool(cond)
        print(("OK   " + good) if cond else ("FAIL " + bad))

    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=2)
        page.on("pageerror", lambda e: errs.append(str(e)))
        page.on("console", lambda m: errs.append(m.text) if m.type == "error" else None)
        page.goto(PAGE.as_uri())
        page.wait_for_selector("#scene")
        page.wait_for_timeout(600)
        page.evaluate("() => { P.paused = true; }")

        closed = page.evaluate(MEASURE)
        page.screenshot(path=str(OUT / "sym-closed-1440.png"))
        check(not closed["open"], "默认不展开", "默认就展开了")
        check(closed["btn"]["y"] < 52, f"按钮在顶栏（y={closed['btn']['y']}）", f"按钮不在顶栏（y={closed['btn']['y']}）")
        check(closed["stage"]["h"] >= 440, f"收起时舞台 {closed['stage']['h']} px", f"舞台只有 {closed['stage']['h']} px")
        check(closed["docScroll"] <= closed["winH"] + 1, "整页无纵向滚动", "整页出现纵向滚动")
        check(not closed["overflowX"], "无横向溢出", "横向溢出")

        page.click("#symBtn")
        page.wait_for_timeout(250)
        opened = page.evaluate(MEASURE)
        page.screenshot(path=str(OUT / "sym-open-1440.png"))
        check(opened["open"], "点按钮能打开", "点了没反应")
        check(opened["items"] == 20, f"条目 {opened['items']} 条", f"条目只有 {opened['items']} 条")
        check(not opened["overlap"], "两列符号不重叠", "符号条目重叠")
        card = opened["card"]
        check(card["y"] >= 0 and card["b"] <= opened["winH"], f"卡片在视口内（{card['y']}~{card['b']}）", "卡片超出视口")
        check(card["x"] >= 0 and card["r"] <= 1440, "卡片未超出左右边界", "卡片左右溢出")
        check(opened["bodyClient"] <= opened["bodyScroll"], "内容超高时卡片内部滚动", "内容被卡片裁掉且不能滚")
        check(opened["stage"]["h"] == closed["stage"]["h"], "打开覆盖层不影响舞台高度", "覆盖层把舞台挤小了")

        page.keyboard.press("Escape")
        page.wait_for_timeout(200)
        after_esc = page.evaluate(MEASURE)
        check(not after_esc["open"], "Esc 可关闭", "Esc 没关掉")

        page.click("#symBtn")
        page.wait_for_timeout(150)
        page.click("#symClose")
        page.wait_for_timeout(200)
        after_btn = page.evaluate(MEASURE)
        check(not after_btn["open"], "「关闭」按钮可关", "关闭按钮无效")
        check(after_btn["stage"]["h"] == closed["stage"]["h"], "关闭后舞台复原", "关闭后舞台没复原")
        # --- 起始距离滑块
        page.evaluate("() => { resetSims(); }")
        page.wait_for_timeout(120)
        base = page.evaluate("() => ({ x: A.drone.x, t: THEORY.tTotal, d: A.d })")
        check(abs(base["x"] + 40) < 0.5, f"默认起点 x={base['x']:.1f}（起始距离 40 m）", f"默认起点不对：x={base['x']}")
        page.evaluate("""() => {
            const el = document.getElementById('startd');
            el.value = '80';
            el.dispatchEvent(new Event('input', { bubbles: true }));
        }""")
        page.wait_for_timeout(150)
        far = page.evaluate("""() => ({
            x: A.drone.x, xb: B.drone.x, d: A.d, t: THEORY.tTotal,
            out: document.getElementById('startdOut').textContent.trim(),
        })""")
        page.screenshot(path=str(OUT / "sym-startd80-1440.png"))
        check(abs(far["x"] + 80) < 0.5 and abs(far["xb"] + 80) < 0.5,
              f"滑到 80 m 后两机起点都在 x=-80（{far['x']:.0f}/{far['xb']:.0f}）",
              f"起点没跟着改：{far['x']}/{far['xb']}")
        check(abs(far["d"] - 80) < 2.0, f"初始距离读数 {far['d']:.1f} m", f"距离读数不对：{far['d']:.1f}")
        check(far["out"] == "80 m", f"读数文案「{far['out']}」", f"读数文案不对：{far['out']}")
        check(far["t"] > base["t"] + 3, f"理论用时随距离变长（{base['t']:.1f} → {far['t']:.1f} s）",
              f"理论用时没变：{base['t']:.1f} → {far['t']:.1f}")
        page.evaluate("""() => {
            const el = document.getElementById('startd');
            el.value = '40';
            el.dispatchEvent(new Event('input', { bubbles: true }));
        }""")
        page.wait_for_timeout(120)
        back = page.evaluate("() => A.drone.x")
        check(abs(back + 40) < 0.5, "滑回 40 m 起点复原", f"滑回后起点 {back:.1f}")

        check(not errs, "无 pageerror / console error", f"有错误: {errs[:3]}")

        browser.close()

    print("\n全部通过" if ok else "\n存在失败项")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
