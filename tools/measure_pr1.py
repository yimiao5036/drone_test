"""量测 PR1（布局 1A）落地后的真实页面几何。

用法（需先起预览服务）：
    cd tools/web平台 && npx vite preview --port 4173 --strictPort
    python tools/measure_pr1.py

说明：页面数据来自 WebSocket/HTTP，没有飞控时列表为空，上下文条会 v-if 隐藏，
量不到真实高度。所以这里通过 #app.__vue_app__ 拿到 Pinia 实例，直接用
store.setFleet() 注入三架假飞机（模板取自 store.telemetry 的空遥测），
量完再清空，顺便验证「未选中机时上下文条不渲染」。
"""

import sys

from playwright.sync_api import sync_playwright

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

URL = "http://127.0.0.1:4173/"
WIDTHS = [1440, 1280, 1180]
HEIGHT = 900

# 预览服务没有 /ws：真实 WebSocket 立即失败会触发 onclose →
# store.clearDisconnectedState() 把注入的遥测清空（实测显示 S0 / 未上报 / 无定位）。
# 用一个永不连上的假 WebSocket 顶掉它，页面就停在「未连接但数据在」的状态。
WS_STUB = """
(() => {
  class StubSocket {
    constructor(url) {
      this.url = url
      this.readyState = 0
      this.onopen = null
      this.onclose = null
      this.onerror = null
      this.onmessage = null
    }
    send() {}
    close() {}
    addEventListener() {}
    removeEventListener() {}
  }
  StubSocket.CONNECTING = 0
  StubSocket.OPEN = 1
  StubSocket.CLOSING = 2
  StubSocket.CLOSED = 3
  window.WebSocket = StubSocket
})()
"""

INJECT = """
() => {
  const app = document.querySelector('#app').__vue_app__
  const pinia = app.config.globalProperties.$pinia
  const store = pinia._s.get('ground-station')
  const tpl = store.telemetry
  const mk = (key, callsign, state, stateLabel, availability, connected) => {
    // store.telemetry 是 Vue 响应式代理，structuredClone 会抛 DataCloneError，
    // 用 JSON 往返拿一份普通对象。
    const t = JSON.parse(JSON.stringify(tpl))
    t.vehicle.connected = connected
    t.vehicle.armed = false
    t.vehicle.flightMode = 'OFFBOARD'
    t.vehicle.systemStatus = 'STANDBY'
    t.mission.state = state
    t.mission.stateLabel = stateLabel
    t.mission.availability = availability
    t.mission.activeWarningBits = 1 << 5
    t.gps.fixLabel = '3D'
    t.gps.satellites = 14
    t.gps.hdop = 0.8
    t.gps.latitude = 31.1234567
    t.gps.longitude = 121.1234567
    t.gps.relativeAltitudeM = 62.4
    t.gps.altitudeMslM = 71.8
    t.gps.headingDeg = 274
    t.battery.remainingPercent = 78
    t.battery.voltageV = 15.9
    t.battery.currentA = 12.4
    // controllable / componentId 由服务端 fleet.snapshot 给出，注入时按同一口径补上，
    // 否则侧栏「就绪 N」恒为 0（readyCounts 要求 connected && controllable && availability===1）。
    return {
      key, callsign, conflict: false,
      componentId: Number(key.split(':')[0]),
      controllable: connected && availability === 1,
      telemetry: t, targetHistory: [], assignmentHistory: [], netFireHistory: []
    }
  }
  store.setFleet({
    aircraft: {
      '25:1': mk('25:1', '捕网-01', 5, 'S5 目标接近', 1, true),
      '25:2': mk('25:2', '捕网-02', 7, 'S7 跟踪', 1, true),
      '26:1': mk('26:1', '火箭-01', 2, 'S2 READY', 1, false)
    },
    link: { ...store.link, type: 'serial', name: 'SITL-5761', health: 'online', packets: 3491 }
  })
  store.setSelectedAircraft('25:1')
  return store.aircraftList.length
}
"""

CLEAR = """
() => {
  const app = document.querySelector('#app').__vue_app__
  const store = app.config.globalProperties.$pinia._s.get('ground-station')
  store.setFleet({ aircraft: {}, link: store.link })
  return store.aircraftList.length
}
"""

MEASURE = """
() => {
  const NL = String.fromCharCode(10)
  const box = (s) => {
    const el = document.querySelector(s)
    if (!el) return null
    const b = el.getBoundingClientRect()
    return { x: Math.round(b.x), y: Math.round(b.y), w: Math.round(b.width),
             h: Math.round(b.height), r: Math.round(b.right), b: Math.round(b.bottom) }
  }
  const ov = (s) => {
    const el = document.querySelector(s)
    return el ? Math.round(el.scrollWidth - el.clientWidth) : null
  }
  const btnLines = [...document.querySelectorAll('.top-actions .el-button span')]
    .map(el => el.getClientRects().length)
  const out = {
    vw: window.innerWidth,
    topbar: box('.topbar'),
    topbarOverflow: ov('.topbar'),
    topActions: box('.top-actions'),
    topActionsOverflow: ov('.top-actions'),
    btnTextRects: btnLines,
    taskVisible: (() => { const e = document.querySelector('.topbar-task'); return e ? getComputedStyle(e).display !== 'none' : null })(),
    menuTaskVisible: (() => { const e = document.querySelector('.menu-narrow'); return e ? getComputedStyle(e).display !== 'none' : null })(),
    contextbar: box('.contextbar'),
    contextbarOverflow: ov('.contextbar'),
    contextText: (() => { const e = document.querySelector('.contextbar'); return e ? e.textContent.split(NL).join(' ').replace(/\\s+/g, ' ').trim() : null })(),
    metricbar: box('.metricbar'),
    metricbarOverflow: ov('.metricbar'),
    mapPanel: box('.primary-grid .map-panel'),
    mapViewport: box('#mission-map'),
    statusCard: box('.map-status-card'),
    toolbar: box('.map-toolbar'),
    modeBar: box('.map-mode-bar'),
    legend: box('.map-legend'),
    legendOverflow: ov('.map-legend'),
    coords: box('.map-coordinates'),
    coordsOverflow: ov('.map-coordinates'),
    fleetBtn: (() => { const e = document.querySelector('.fleet-toggle small'); return e ? e.textContent.trim() : null })(),
    fleetLines: [...document.querySelectorAll('.fleet-item small')].map(el => Math.round(el.getBoundingClientRect().height)),
    // 截断检测：溢出隐藏会让 scrollWidth > clientWidth
    fleetClipped: [...document.querySelectorAll('.fleet-item small')].map(el => el.scrollWidth > el.clientWidth + 1),
    fleetTexts: [...document.querySelectorAll('.fleet-item small')].map(el => el.textContent.trim()),
    docScroll: { h: document.documentElement.scrollHeight, w: document.documentElement.scrollWidth }
  }
  return out
}
"""


def main() -> None:
    with sync_playwright() as pw:
        browser = pw.chromium.launch()
        page = browser.new_page(viewport={"width": WIDTHS[0], "height": HEIGHT}, device_scale_factor=1)
        # 页面会持续重连 WebSocket（预览服务没有 /ws），networkidle 永远等不到，
        # 只等 DOM 就绪，再显式等 Vue 挂载完成（#app 有子节点）。
        page.add_init_script(WS_STUB)
        page.goto(URL, wait_until="domcontentloaded")
        page.wait_for_selector(".topbar", timeout=15000)
        page.wait_for_timeout(1200)
        injected = page.evaluate(INJECT)
        print(f"注入飞机数：{injected}")
        page.wait_for_timeout(500)

        for width in WIDTHS:
            page.set_viewport_size({"width": width, "height": HEIGHT})
            page.wait_for_timeout(350)
            data = page.evaluate(MEASURE)
            print(f"\n===== {width}x{HEIGHT} =====")
            for key in (
                "topbar", "topbarOverflow", "topActions", "topActionsOverflow", "btnTextRects",
                "taskVisible", "menuTaskVisible", "contextbar", "contextbarOverflow",
                "metricbar", "metricbarOverflow", "mapPanel", "mapViewport", "statusCard",
                "toolbar", "modeBar", "legend", "legendOverflow", "coords", "coordsOverflow",
                "fleetBtn", "fleetLines", "fleetClipped", "fleetTexts", "docScroll"
            ):
                print(f"  {key}: {data[key]}")
            print(f"  contextText: {data['contextText']}")

        page.set_viewport_size({"width": 1440, "height": HEIGHT})
        page.wait_for_timeout(300)
        page.screenshot(path="tools/layout_evidence/pr1-1440.png", full_page=False)

        cleared = page.evaluate(CLEAR)
        page.wait_for_timeout(350)
        empty = page.evaluate(MEASURE)
        print(f"\n===== 清空机队（{cleared} 架）=====")
        print(f"  contextbar: {empty['contextbar']}（应为 None）")
        print(f"  metricbar:  {empty['metricbar']}")
        print(f"  mapPanel:   {empty['mapPanel']}")
        print(f"  docScroll:  {empty['docScroll']}")
        browser.close()


if __name__ == "__main__":
    main()
