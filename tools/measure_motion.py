"""动点（移动目标）地图表现量测：分段拆线、走向箭头、预测航迹、清除轨迹。

前置：在 WSL 里 `cd tools/web平台 && npx vite preview --port 4173 --strictPort` 常驻。
运行：Windows 侧 `python tools/measure_motion.py`（playwright 只装在 Windows 侧 python 里）。

为什么需要 WS_STUB：预览服务没有 /ws，真 WebSocket 一连就失败，
store.clearDisconnectedState() 会把注入的机队数据清回 S0/无定位。
"""
import json
import pathlib
import sys

from playwright.sync_api import sync_playwright

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "tools" / "layout_evidence"
URL = "http://127.0.0.1:4173/"

WS_STUB = """window.WebSocket = class StubSocket {
  static CONNECTING = 0; static OPEN = 1; static CLOSING = 2; static CLOSED = 3
  constructor() { this.readyState = 0; this.onopen = this.onclose = this.onerror = this.onmessage = null }
  addEventListener() {} removeEventListener() {} send() {} close() {}
};"""

INJECT = r"""
(payload) => {
  const pinia = document.querySelector('#app').__vue_app__.config.globalProperties.$pinia
  const gs = pinia._s.get('ground-station')
  gs.setFleet({
    aircraft: {
      '25:1': {
        key: '25:1', callsign: '捕网-01', systemId: 25, componentId: 25,
        conflict: false,
        telemetry: {
          ...gs.telemetry,
          vehicle: { systemId: 25, componentId: 25, connected: true, armed: true, flightMode: 'OFFBOARD',
            systemStatus: '任务中', landedState: '空中', lastHeartbeatAt: new Date().toISOString() },
          gps: { fixType: 3, fixLabel: '3D', latitude: 31.1234567, longitude: 121.1234567, altitudeMslM: 71.8,
            relativeAltitudeM: 62.4, satellites: 18, hdop: 0.8, headingDeg: 90, groundSpeedMps: 2.0,
            updatedAt: new Date().toISOString() },
          mission: { state: 5, stateLabel: 'S5 目标接近', controlSource: '机载', taskPhase: '接近',
            activeWarningBits: 0, stateEnteredMs: Date.now(), frontDistanceM: 12, interceptionAuthorized: false,
            powerStatusBits: 0, assignmentId: 11, targetId: 7, availability: 1, interceptorType: 'net',
            interceptorTypeLabel: '捕网', updatedAt: new Date().toISOString() }
        },
        assignmentHistory: [], activeAssignmentId: null
      }
    },
    link: { connected: true, healthy: true, lastHeartbeatAt: new Date().toISOString() }
  })
  gs.setSelectedAircraft('25:1')
  gs.selectedCoordinate = { latitude: payload.picked.latitude, longitude: payload.picked.longitude }

  // 真实链路：直接喂 target.sample 走 appendTargetSample，验证分段判定。
  const feed = (targetId, latitude, longitude, elapsedSeconds) => gs.handleEnvelope({ type: 'target.sample', timestamp: new Date().toISOString(), data: {
    aircraftKey: '25:1', targetId, latitude, longitude, altitudeMslM: 71.2, speedMps: 2.0,
    headingDeg: 90, velocityNorthMps: 0, velocityEastMps: 2.0, elapsedSeconds } })
  // 第一段：从 picked 正东推进，每 5 s 一个采样（0.0001° ≈ 9.5 m，2 m/s × 5 s = 10 m，
  // 与真实链路一致），共 8 拍 ≈ 66 m，保证 z17 下肉眼可见一条实线。
  for (let i = 0; i < 8; i += 1) feed(7, payload.picked.latitude, payload.picked.longitude + i * 0.0001, i * 5)
  // 第二段：重新打点（elapsed 归零 + 位置跳变约 200 m），应当另起 segment=1。
  for (let i = 0; i < 5; i += 1) feed(7, payload.picked.latitude + 0.0015, payload.picked.longitude + 0.0015 + i * 0.0001, i * 5)

  gs.targetRepeats = { '25:1': {
    hz: 1, targetId: 7, aircraftKey: '25:1', assignmentId: null, startedAt: new Date().toISOString(),
    stopReason: null,
    motion: { headingDeg: 90, speedMps: 2.0, verticalSpeedMps: 0, stopMode: 'distance', durationSeconds: 0, distanceMeters: 600 },
    progress: { elapsedSeconds: 40, distanceMeters: 100, speedMps: 2.0, headingDeg: 90 }
  } }
  const segments = (gs.targetSamples['25:1'] || []).map(s => s.segment ?? 0)
  return { segments, count: segments.length }
}
"""


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=2)
        page.add_init_script(WS_STUB)
        page.goto(URL, wait_until="domcontentloaded")
        page.wait_for_selector(".topbar")
        page.wait_for_timeout(1200)
        picked = {"latitude": 31.1234567, "longitude": 121.1234567}
        info = page.evaluate(INJECT, {"picked": picked})
        page.wait_for_timeout(900)
        # 缩到目标附近才看得清箭头与预测线：直接把视野对准两段轨迹的包围盒中心。
        page.get_by_role("button", name="机队视野").click()
        page.wait_for_timeout(800)
        page.evaluate("""() => {
          const map = window.__missionMap
          const pts = []
          for (const f of map.getSource('target-tracks')._data.features) {
            for (const c of (f.geometry.type === 'LineString' ? f.geometry.coordinates : [f.geometry.coordinates])) pts.push(c)
          }
          for (const f of map.getSource('target-motion')._data.features) {
            if (f.geometry.type === 'LineString') pts.push(...f.geometry.coordinates)
            else pts.push(f.geometry.coordinates)
          }
          if (!pts.length) return
          const lons = pts.map(p => p[0]), lats = pts.map(p => p[1])
          const b = [[Math.min(...lons), Math.min(...lats)], [Math.max(...lons), Math.max(...lats)]]
          map.fitBounds(b, { padding: 120, maxZoom: 18, duration: 0 })
        }""")
        page.wait_for_timeout(1500)

        map_panel = page.locator(".map-panel").first
        map_panel.screenshot(path=str(OUT / "motion-map-1440.png"))
        page.locator("#mission-map").screenshot(path=str(OUT / "motion-canvas-1440.png"))

        actions = page.locator(".map-actions button").all_inner_texts()
        clear_button = page.get_by_role("button", name="清除轨迹")
        report = {
            "map": page.evaluate("""() => {
              const map = window.__missionMap
              if (!map) return { missing: true }
              const src = (id) => {
                const s = map.getSource(id)
                if (!s) return null
                const d = s._data
                return d && d.features ? { features: d.features.length, kinds: d.features.map(f => f.properties && (f.properties.kind || f.geometry.type)) } : { raw: true }
              }
              return {
                layers: ['target-tracks-line', 'target-tracks-head', 'target-motion-line', 'target-motion-arrow', 'target-origin-label'].map(id => [id, !!map.getLayer(id)]),
                tracks: src('target-tracks'),
                motion: src('target-motion'),
                target: src('target'),
                zoom: map.getZoom()
              }
            }"""),
            "segments": info["segments"],
            "sampleCount": info["count"],
            "actions": actions,
            "clearEnabled": clear_button.is_enabled(),
        }
        clear_button.click()
        page.wait_for_timeout(400)
        report["afterClear"] = page.evaluate("""() => {
          const pinia = document.querySelector('#app').__vue_app__.config.globalProperties.$pinia
          const gs = pinia._s.get('ground-station')
          return { count: (gs.targetSamples['25:1'] || []).length }
        }""")
        page.locator(".map-panel").first.screenshot(path=str(OUT / "motion-cleared-1440.png"))
        print(json.dumps(report, ensure_ascii=False, indent=2))
        browser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
