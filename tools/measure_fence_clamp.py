"""机载 bit7「引导点越出围栏」提示的量测。

前置：WSL 里 `cd tools/web平台 && npx vite preview --port 4173 --strictPort` 常驻。
运行：Windows 侧 `python tools/measure_fence_clamp.py`（playwright 只在 Windows 侧 python 里）。

验证：
1. 65013 置 bit7 时顶栏出现「目标在围栏外」chip、机队列表出现「围栏外」角标；
2. 三种宽度（1440/1280/1180）顶栏不溢出、按钮文字不被挤成两行；
3. bit7 清掉后 chip 与角标一起消失。
"""
import json
import pathlib
import sys

from playwright.sync_api import sync_playwright

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "tools" / "layout_evidence"
URL = "http://127.0.0.1:4173/"

# 预览页没有 /ws，真 WebSocket 立刻失败会触发 clearDisconnectedState() 清空注入数据，
# 用一个永不连上的桩顶掉。
WS_STUB = """window.WebSocket = class StubSocket {
  static CONNECTING = 0; static OPEN = 1; static CLOSING = 2; static CLOSED = 3
  constructor() { this.readyState = 0; this.onopen = this.onclose = this.onerror = this.onmessage = null }
  addEventListener() {} removeEventListener() {} send() {} close() {}
};"""

INJECT = r"""
(payload) => {
  const pinia = document.querySelector('#app').__vue_app__.config.globalProperties.$pinia
  const gs = pinia._s.get('ground-station')
  const key = '25:1'
  gs.setFleet({
    aircraft: {
      '25:1': {
        key, callsign: '捕网-01', systemId: 25, componentId: 25, conflict: false,
        telemetry: {
          ...gs.telemetry,
          vehicle: { systemId: 25, componentId: 25, connected: true, armed: true, flightMode: 'OFFBOARD',
            systemStatus: '任务中', landedState: '空中', lastHeartbeatAt: new Date().toISOString() },
          gps: { fixType: 3, fixLabel: '3D', latitude: 31.1234567, longitude: 121.1234567, altitudeMslM: 71.8,
            relativeAltitudeM: 62.4, satellites: 18, hdop: 0.8, headingDeg: 90, groundSpeedMps: 2.0,
            updatedAt: new Date().toISOString() },
          mission: { state: 5, stateLabel: 'S5 目标接近', controlSource: '机载', taskPhase: '接近',
            activeWarningBits: payload.bits, stateEnteredMs: Date.now(), frontDistanceM: 12,
            interceptionAuthorized: false, powerStatusBits: 0, assignmentId: 11, targetId: 7,
            availability: 1, interceptorType: 'net', interceptorTypeLabel: '捕网',
            updatedAt: new Date().toISOString() }
        },
        assignmentHistory: [], activeAssignmentId: null
      }
    },
    link: { connected: true, healthy: true, lastHeartbeatAt: new Date().toISOString() }
  })
  gs.setSelectedAircraft(key)
  return 1
}
"""

READ = r"""
() => {
  const q = sel => document.querySelector(sel)
  const rect = el => { if (!el) return null; const r = el.getBoundingClientRect(); return {
    x: Math.round(r.x), y: Math.round(r.y), w: Math.round(r.width), h: Math.round(r.height) } }
  const topbar = q('.topbar'); const actions = q('.top-actions')
  const chip = q('.clamp-chip'); const badge = q('.fleet-clamp')
  const buttons = [...document.querySelectorAll('.top-actions .el-button')]
  return {
    chip: chip ? chip.textContent.trim() : null,
    chipRect: rect(chip),
    badge: badge ? badge.textContent.trim() : null,
    badgeTitle: badge ? badge.getAttribute('title') : null,
    topbarH: topbar ? Math.round(topbar.getBoundingClientRect().height) : null,
    topActionsOverflow: actions ? Math.max(0, actions.scrollWidth - actions.clientWidth) : null,
    topbarOverflow: topbar ? Math.max(0, topbar.scrollWidth - topbar.clientWidth) : null,
    wrapped: buttons.filter(b => b.getClientRects().length > 1).map(b => b.textContent.trim()),
  }
}
"""


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    results = {}
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=2)
        page.add_init_script(WS_STUB)
        page.goto(URL, wait_until="domcontentloaded")
        page.wait_for_selector(".topbar")
        page.wait_for_timeout(1200)

        # bit7 置位：顶栏 chip + 侧栏角标应同时出现
        page.evaluate(INJECT, {"bits": 1 << 7})
        page.wait_for_timeout(600)
        results["withBit7"] = page.evaluate(READ)
        page.locator(".topbar").screenshot(path=str(OUT / "clamp-topbar.png"))
        page.locator(".sidebar").screenshot(path=str(OUT / "clamp-sidebar.png"))

        for width in (1440, 1280, 1180):
            page.set_viewport_size({"width": width, "height": 900})
            page.wait_for_timeout(400)
            results[f"width{width}"] = page.evaluate(READ)

        # 清掉 bit7：chip 与角标一起消失
        page.set_viewport_size({"width": 1440, "height": 900})
        page.evaluate(INJECT, {"bits": 0})
        page.wait_for_timeout(600)
        results["cleared"] = page.evaluate(READ)
        browser.close()

    print(json.dumps(results, ensure_ascii=False, indent=2))

    assert results["withBit7"]["chip"], results["withBit7"]
    assert results["withBit7"]["badge"] == "围栏外", results["withBit7"]
    for width in (1440, 1280, 1180):
        row = results[f"width{width}"]
        assert row["topbarH"] == 58, (width, row)
        assert row["topActionsOverflow"] == 0, (width, row)
        assert row["topbarOverflow"] == 0, (width, row)
        assert row["wrapped"] == [], (width, row)
        assert row["chip"], (width, row)
    assert results["cleared"]["chip"] is None, results["cleared"]
    assert results["cleared"]["badge"] is None, results["cleared"]
    print("OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
