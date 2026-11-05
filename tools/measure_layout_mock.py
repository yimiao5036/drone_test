"""量测 docs/示意图/地面站布局示意-2026-11-05.html 的两屏关键元素几何。

用法：python tools/measure_layout_mock.py
输出每屏 topbar/banner/contextbar/metricbar/map/coords/video/panel/legend 的 x/y/w/h
以及各容器的 scrollWidth-clientWidth 溢出量，用于确认「不溢出、不遮挡」。
"""

import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

# Windows 控制台默认 GBK，示意图文案含 ⚠ / · 等字符，统一按 UTF-8 输出
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

HTML = Path("docs/示意图/地面站布局示意-2026-11-05.html").resolve().as_uri()

JS = """
(sid) => {
  const q = (s) => document.querySelector('#' + sid + ' ' + s);
  const box = (el) => {
    if (!el) return null;
    const b = el.getBoundingClientRect();
    return { x: Math.round(b.x), y: Math.round(b.y), w: Math.round(b.width),
             h: Math.round(b.height), r: Math.round(b.right), b: Math.round(b.bottom) };
  };
  const one = (s) => { const e = q(s); return e ? Math.round(e.scrollWidth - e.clientWidth) : null; };
  const oneh = (s) => { const e = q(s); return e ? Math.round(e.scrollHeight - e.clientHeight) : null; };
  const NL = String.fromCharCode(10);
  const txt = (el) => (el ? el.textContent.split(NL).join(' ').trim() : null);
  const out = { sid: sid };
  out.topbar = box(q('.topbar'));
  out.topbarOverflow = one('.topbar');
  out.chip = box(q('.chip.alert'));
  out.chipText = txt(q('.chip.alert'));
  out.chipCompact = !!document.querySelector('#' + sid + ' .chip.alert.compact');
  out.banner = box(q('.banner'));
  out.bannerOverflow = one('.banner');
  out.bannerBtns = [...document.querySelectorAll('#' + sid + ' .banner .mini')].map((e) => e.textContent.trim());
  out.ctx = box(q('.contextbar'));
  out.ctxOverflow = one('.contextbar');
  out.ctxPills = [...document.querySelectorAll('#' + sid + ' .contextbar .pill')].map((e) => e.textContent.trim());
  out.metricBar = box(q('.metricbar'));
  out.map = box(q('.map'));
  out.coords = box(q('.map-coords'));
  out.coordsOverflow = one('.map-coords');
  out.video = box(q('.video'));
  out.panel = box(q('.alert-panel'));
  out.legend = box(q('.legend'));
  out.legendOverflow = oneh('.legend');
  out.fleetTitle = txt(q('.fleet-title'));
  return out;
}
"""


def main() -> None:
    with sync_playwright() as pw:
        browser = pw.chromium.launch()
        page = browser.new_page(viewport={"width": 1440, "height": 900})
        page.goto(HTML)
        for sid in ("screen1", "screen2"):
            data = page.evaluate(JS, sid)
            print(data["sid"])
            for key, val in data.items():
                if key == "sid":
                    continue
                print(f"    {key} = {val}")
        browser.close()


if __name__ == "__main__":
    main()
