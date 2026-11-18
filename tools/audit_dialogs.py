"""审查地图左下角叠放与四个抽屉的内部布局（真实页面，预览服务 4173）。

用法（需先起预览服务）：
    cd tools/web平台 && npx vite preview --port 4173 --strictPort
    python tools/audit_dialogs.py

产出：控制台几何表 + tools/layout_evidence/audit-*.png
"""

import sys

from playwright.sync_api import sync_playwright

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

URL = "http://127.0.0.1:4173/"
WIDTH, HEIGHT = 1440, 900
SHOT = "tools/layout_evidence"

WS_STUB = """
(() => {
  class StubSocket {
    constructor(url) { this.url = url; this.readyState = 0 }
    send() {} close() {} addEventListener() {} removeEventListener() {}
  }
  StubSocket.CONNECTING = 0; StubSocket.OPEN = 1; StubSocket.CLOSING = 2; StubSocket.CLOSED = 3
  window.WebSocket = StubSocket
})()
"""

INJECT = """
() => {
  const pinia = document.querySelector('#app').__vue_app__.config.globalProperties.$pinia
  const store = pinia._s.get('ground-station')
  const tpl = store.telemetry
  const mk = (key, callsign, state, label, avail) => {
    const t = JSON.parse(JSON.stringify(tpl))
    t.vehicle.connected = true
    t.vehicle.armed = true
    t.vehicle.flightMode = 'OFFBOARD'
    t.vehicle.systemStatus = 'ACTIVE'
    t.mission.state = state
    t.mission.stateLabel = label
    t.mission.availability = avail
    t.gps.fixType = 3
    t.gps.fixLabel = '3D'
    t.gps.satellites = 14
    t.gps.hdop = 0.8
    t.gps.latitude = 31.1234567
    t.gps.longitude = 121.1234567
    t.gps.altitudeMslM = 71.8
    t.gps.relativeAltitudeM = 62.4
    t.gps.headingDeg = 274
    t.gps.groundSpeedMps = 0.1
    return {
      key, callsign,
      systemId: Number(key.split(':')[0]),
      componentId: Number(key.split(':')[1]),
      connected: true, controllable: true, conflict: false,
      telemetry: t,
      assignmentHistory: [],
      activeAssignmentId: null
    }
  }
  store.setFleet({ aircraft: {
    '25:1': mk('25:1', '捕网-01', 3, 'S3 任务调度', 3),
    '25:2': mk('25:2', '捕网-02', 7, 'S7 目标跟踪', 1),
    '26:1': mk('26:1', '火箭-01', 2, 'S2 等待起飞', 1)
  }})
}

"""

GEOM = """
() => {
  const r = (s) => { const e = document.querySelector(s); if (!e) return null
    const b = e.getBoundingClientRect()
    return { x: Math.round(b.x), y: Math.round(b.y), w: Math.round(b.width), h: Math.round(b.height),
             r: Math.round(b.right), b: Math.round(b.bottom) } }
  const hit = (a, b) => !!(a && b && a.x < b.r && b.x < a.r && a.y < b.b && b.y < a.b)
  const legend = r('.map-legend'), coords = r('.map-coordinates'), status = r('.map-status-card')
  const zoom = r('.maplibregl-ctrl-bottom-right')
  return {
    legend, coords, status, zoom,
    legendHitsCoords: hit(legend, coords),
    legendHitsStatus: hit(legend, status),
    coordsHitsStatus: hit(coords, status),
    coordsHitsZoom: hit(coords, zoom),
    legendText: (document.querySelector('.map-legend')?.textContent || '').trim(),
    coordsText: (document.querySelector('.map-coordinates')?.textContent || '').trim()
  }
}
"""

DRAWER = """
(sel) => {
  const d = document.querySelector(sel)
  if (!d) return null
  const r = d.getBoundingClientRect()
  const body = d.querySelector('.el-drawer__body')
  const head = d.querySelector('.el-drawer__header')
  const scroll = body ? { h: body.clientHeight, sh: body.scrollHeight, ov: body.scrollWidth - body.clientWidth } : null
  // 找出所有水平溢出的直接子块，方便判断是不是"内容比抽屉宽"
  const wide = [...(body ? body.querySelectorAll('*') : [])]
    .filter(e => e.scrollWidth - e.clientWidth > 2 && e.clientWidth > 0)
    .slice(0, 6)
    .map(e => `${e.tagName.toLowerCase()}.${(e.className || '').toString().split(' ')[0]} +${e.scrollWidth - e.clientWidth}`)
  return {
    rect: { x: Math.round(r.x), y: Math.round(r.y), w: Math.round(r.width), h: Math.round(r.height) },
    headH: head ? Math.round(head.getBoundingClientRect().height) : null,
    scroll, wide,
    title: (head?.textContent || '').trim().slice(0, 40)
  }
}
"""


def main() -> int:
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": WIDTH, "height": HEIGHT}, device_scale_factor=2)
        page.add_init_script(WS_STUB)
        page.goto(URL, wait_until="domcontentloaded")
        page.wait_for_selector(".topbar")
        page.evaluate(INJECT)
        page.wait_for_timeout(900)

        print("map:", page.evaluate(GEOM))
        page.locator(".map-panel").screenshot(path=f"{SHOT}/audit-map-corner.png")

        drawers = [
            ("mission", "任务调度", "() => { document.querySelector('#app').__vue_app__.config.globalProperties.$pinia._s.get('ground-station').missionDrawerVisible = true }", ".mission-drawer"),
            ("netfire", "捕网发射", "() => { document.querySelector('#app').__vue_app__.config.globalProperties.$pinia._s.get('ground-station').netFireDrawerVisible = true }", ".netfire-drawer"),
            ("fence", "共享区域", "() => { document.querySelector('#app').__vue_app__.config.globalProperties.$pinia._s.get('ground-station').fenceDrawerVisible = true }", ".fence-drawer"),
        ]
        for tag, name, open_js, sel in drawers:
            page.evaluate(open_js)
            page.wait_for_timeout(700)
            print(f"drawer[{tag}]:", page.evaluate(DRAWER, sel))
            page.screenshot(path=f"{SHOT}/audit-{tag}.png")
            page.keyboard.press("Escape")
            page.wait_for_timeout(500)

        # 系统状态：按钮在顶栏，点开即可
        page.get_by_role("button", name="系统状态").click()
        page.wait_for_timeout(600)
        print("drawer[status]:", page.evaluate(DRAWER, ".el-drawer"))
        page.screenshot(path=f"{SHOT}/audit-status.png")
        browser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
