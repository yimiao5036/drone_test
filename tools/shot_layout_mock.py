"""把布局示意图 HTML 渲染为 JPG（1440x900 @2x，与现有图一致）。

用法：python tools/shot_layout_mock.py
输出：
  docs/示意图/地面站布局示意-主界面-2026-11-05.jpg
  docs/示意图/地面站布局示意-多机告警-2026-11-05.jpg
"""
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
HTML = ROOT / "docs" / "示意图" / "地面站布局示意-2026-11-05.html"
OUT = ROOT / "docs" / "示意图"

SHOTS = [
    ("#screen1", "地面站布局示意-主界面-2026-11-05.jpg"),
    ("#screen2", "地面站布局示意-多机告警-2026-11-05.jpg"),
]


def main() -> None:
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=2)
        page.goto(HTML.as_uri())
        page.wait_for_timeout(300)
        for selector, name in SHOTS:
            el = page.locator(selector)
            el.screenshot(path=str(OUT / name), type="jpeg", quality=92)
            print("wrote", name)
        browser.close()


if __name__ == "__main__":
    main()
