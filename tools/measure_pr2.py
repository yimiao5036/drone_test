"""量测 PR2（到点未识别告警）落地后的真实页面几何与交互。

用法（需先起预览服务）：
    cd tools/web平台 && npx vite preview --port 4173 --strictPort
    python tools/measure_pr2.py

说明：预览服务没有飞控数据，这里通过 #app.__vue_app__ 拿到 Pinia，注入两架
「已到点」的假飞机（一架 S5 未识别 → 等待告警，一架 S7 已识别 → 绿色到点角标），
并把 handover 阈值临时调小（hold 200 ms、提醒 1200 ms），避免真等 20 s。
量完清空机队，验证「没有告警时 chip 与卡片都不渲染」。
"""

import sys

from playwright.sync_api import sync_playwright

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

URL = "http://127.0.0.1:4173/"
WIDTH = 1440
HEIGHT = 900

# 预览服务没有 /ws：真实 WebSocket 立即失败会触发 onclose → clearDisconnectedState()
# 把注入的遥测清空。用一个永不连上的假 WebSocket 顶掉它（PR1 量测脚本同款）。
WS_STUB = """
(() => {
  class StubSocket {
    constructor(url) {
      this.url = url; this.readyState = 0
      this.onopen = null; this.onclose = null; this.onerror = null; this.onmessage = null
    }
    send() {} close() {} addEventListener() {} removeEventListener() {}
  }
  StubSocket.CONNECTING = 0; StubSocket.OPEN = 1; StubSocket.CLOSING = 2; StubSocket.CLOSED = 3
  window.WebSocket = StubSocket
})()
"""

INJECT = """
() => {
  const app = document.querySelector('#app').__vue_app__
  const pinia = app.config.globalProperties.$pinia
  const store = pinia._s.get('ground-station')
  const handover = pinia._s.get('handover')
  // 缩短阈值，避免脚本真等 20 s；判据逻辑本身仍走 stepHandover。
  handover.thresholds = { ...handover.thresholds, arrivalHoldMs: 200, visualWaitWarnMs: 1200 }
  const tpl = store.telemetry
  const mk = (key, callsign, state, stateLabel, warnBits) => {
    const t = JSON.parse(JSON.stringify(tpl))
    t.vehicle.connected = true
    t.vehicle.armed = true
    t.vehicle.flightMode = 'OFFBOARD'
    t.vehicle.systemStatus = 'ACTIVE'
    t.mission.state = state
    t.mission.stateLabel = stateLabel
    t.mission.availability = 3
    t.mission.activeWarningBits = warnBits
    t.gps.fixLabel = '3D'
    t.gps.satellites = 14
    t.gps.hdop = 0.8
    t.gps.latitude = 31.1234567
    t.gps.longitude = 121.1234567
    t.gps.altitudeMslM = 71.8
    t.gps.relativeAltitudeM = 62.4
    t.gps.headingDeg = 274
    t.gps.groundSpeedMps = 0.1
    t.battery.remainingPercent = 78
    t.battery.voltageV = 15.9
    t.battery.currentA = 12.4
    // 目标就放在机头附近 ≈1.1 m / 高差 0.6 m，满足 3 m / 2 m 到点条件。
    const target = { targetId: 7, latitude: 31.1234667, longitude: 121.1234567, altitudeMslM: 71.2, validForMs: 60000 }
    return {
      key, callsign, conflict: false,
      componentId: Number(key.split(':')[0]),
      controllable: false,
      activeAssignmentId: 11,
      assignmentHistory: [{
        aircraftKey: key, callsign, systemId: Number(key.split(':')[0]), componentId: Number(key.split(':')[0]),
        assignmentId: 11, targetId: 7, interceptorType: 1, interceptorTypeLabel: '捕网',
        flags: 0, validForMs: 60000, status: 'accepted', cancelRequested: false, resendCount: 0,
        source: 'gcs', createdAt: new Date().toISOString(), updatedAt: new Date().toISOString(),
        target
      }],
      telemetry: t, targetHistory: [], netFireHistory: []
    }
  }
  store.setFleet({
    aircraft: {
      // S5 且没有 bit5 → 判为"没看见目标"，到点后进入等待 → 黄/红告警
      '25:1': mk('25:1', '捕网-01', 5, 'S5 目标接近', 0),
      // S7 且无失效位 → 判为"已识别"，到点即绿角标，不弹卡片
      '25:2': mk('25:2', '捕网-02', 7, 'S7 跟踪', 0)
    },
    link: { ...store.link, type: 'serial', name: 'SITL-5761', health: 'online', packets: 3491 }
  })
  store.setSelectedAircraft('25:1')
  handover.tick()
  return { aircraft: store.aircraftList.length, alerts: handover.alerts.length }
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
  const hit = (a, b) => !!a && !!b && a.x < b.r && b.x < a.r && a.y < b.b && b.y < a.b
  const ov = (s) => {
    const el = document.querySelector(s)
    return el ? Math.round(el.scrollWidth - el.clientWidth) : null
  }
  const card = box('.handover-card')
  const map = box('.primary-grid .map-panel')
  const dock = box('.map-video-dock')
  const coords = box('.map-coordinates')
  const legend = box('.map-legend')
  const zoom = box('.maplibregl-ctrl-bottom-right')
  const statusCard = box('.map-status-card')
  const pinia = document.querySelector('#app').__vue_app__.config.globalProperties.$pinia
  const handover = pinia._s.get('handover')
  const gs = pinia._s.get('ground-station')
  return {
    alerts: handover.alerts.map(a => `${a.callsign}/${a.level}/${a.waitedMs}ms/${a.text}`),
    phases: JSON.stringify(handover.phases),
    bagdes: gs.aircraftList.map(i => {
      const b = handover.badgeOf(i.key)
      return b ? `${i.callsign}·${b.level}·${b.abbr}` : `${i.callsign}·none`
    }),
    chipText: (() => { const e = document.querySelector('.handover-chip'); return e ? e.textContent.trim() : null })(),
    chipBox: box('.handover-chip'),
    topbarOverflow: ov('.topbar'),
    topActionsOverflow: ov('.top-actions'),
    card, cardInMap: hit(card, map),
    cardHitsDock: hit(card, dock),
    cardHitsCoords: hit(card, coords),
    cardHitsLegend: hit(card, legend),
    cardHitsZoom: hit(card, zoom),
    cardHitsStatus: hit(card, statusCard),
    cardGapRight: card && map ? map.r - card.r : null,
    cardGapBottom: card && map ? map.b - card.b : null,
    cardOverflow: ov('.handover-card'),
    cardRows: [...document.querySelectorAll('.hc-row')].map(el => el.textContent.split(NL).join(' ').replace(/\\\\s+/g, ' ').trim()),
    cardButtons: [...document.querySelectorAll('.hc-actions button')].map(el => ({ text: el.textContent.trim(), disabled: el.disabled })),
    fleetBadges: [...document.querySelectorAll('.fleet-handover')].map(el => `${el.textContent.trim()}/${el.className}`),
    fleetClipped: [...document.querySelectorAll('.fleet-item small')].map(el => el.scrollWidth > el.clientWidth + 1)
  }
}
"""

CLEAR = """
() => {
  const app = document.querySelector('#app').__vue_app__
  const pinia = app.config.globalProperties.$pinia
  const store = pinia._s.get('ground-station')
  const handover = pinia._s.get('handover')
  store.setFleet({ aircraft: {}, link: store.link })
  handover.tick()
  return { aircraft: store.aircraftList.length, alerts: handover.alerts.length }
}
"""

EMPTY = """
() => ({
  chip: !!document.querySelector('.handover-chip'),
  card: !!document.querySelector('.handover-card'),
  alerts: document.querySelector('#app').__vue_app__.config.globalProperties.$pinia._s.get('handover').alerts.length
})
"""


def main() -> int:
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": WIDTH, "height": HEIGHT}, device_scale_factor=2)
        page.add_init_script(WS_STUB)
        page.goto(URL, wait_until="domcontentloaded")
        page.wait_for_selector(".topbar")
        # networkidle 等不到（WebSocket 一直在重连），用固定等待让首次渲染稳定。
        page.wait_for_timeout(800)

        print("inject:", page.evaluate(INJECT))
        # 到点需要 holdMs 稳定 + 等待计时累计，留 3.5 s 让提醒升级到红。
        page.wait_for_timeout(3500)
        page.screenshot(path="tools/layout_evidence/pr2-1440.png", full_page=False)
        metrics = page.evaluate(MEASURE)
        for key, value in metrics.items():
            print(f"{key}: {value}")
        page.locator(".handover-card").screenshot(path="tools/layout_evidence/pr2-alert-card.png")

        # 收起卡片后只剩一行标题
        page.locator(".hc-head").click()
        page.wait_for_timeout(250)
        page.screenshot(path="tools/layout_evidence/pr2-1440-collapsed.png", full_page=False)
        print("collapsed:", page.evaluate("() => document.querySelector('.handover-card').getBoundingClientRect().height"))

        # 三宽复核顶栏：告警 chip 出现后最容易把「捕网-01 · ARMED · 任务中」挤成两行。
        WRAP = """
() => {
  const h = (s) => { const e = document.querySelector(s); return e ? Math.round(e.getBoundingClientRect().height) : null }
  const ov = (s) => { const e = document.querySelector(s); return e ? Math.round(e.scrollWidth - e.clientWidth) : null }
  return {
    topbarH: h('.topbar'), flightBadgeH: h('.flight-badge'), chipH: h('.handover-chip'),
    statusH: h('.status-chip'), topActionsOv: ov('.top-actions'), topbarOv: ov('.topbar'),
    chipVisible: !!document.querySelector('.handover-chip'), cardVisible: !!document.querySelector('.handover-card')
  }
}
"""
        for width in (1440, 1280, 1180):
            page.set_viewport_size({"width": width, "height": HEIGHT})
            page.wait_for_timeout(400)
            print(f"wrap@{width}:", page.evaluate(WRAP))
        page.set_viewport_size({"width": WIDTH, "height": HEIGHT})
        page.wait_for_timeout(300)

        # 上面点过「收起」，交互前先展开回来。
        page.locator(".hc-head").click()
        page.wait_for_timeout(250)
        # 交互复核：「继续等待」把等待计时归零（卡片消失、20 s 后重新提醒）；
        # 「稍后提醒 60 s」把卡片与顶栏 chip 一起静默。
        page.get_by_role("button", name="继续等待").click()
        page.wait_for_timeout(700)
        print("after-continue:", page.evaluate("() => { const h = document.querySelector('#app').__vue_app__.config.globalProperties.$pinia._s.get('handover'); return { alerts: h.alerts.length, waited: JSON.stringify(h.waited), card: !!document.querySelector('.handover-card') } }"))
        page.wait_for_timeout(1600)
        page.get_by_role("button", name="稍后提醒 60 s").click()
        page.wait_for_timeout(700)
        print("after-snooze:", page.evaluate(EMPTY))

        print("clear:", page.evaluate(CLEAR))
        page.wait_for_timeout(900)
        print("empty:", page.evaluate(EMPTY))
        browser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
