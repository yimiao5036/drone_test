"""目标动点「终点固定 + 跟随停止按钮」量测。

前置：WSL 里 `cd tools/web平台 && npx vite preview --port 4173 --strictPort` 常驻。
运行：Windows 侧 `python tools/measure_target_stop.py`（playwright 只在 Windows 侧 python 里）。

验证两件事：
1. 终点标记钉在「起点 + 总里程」处，不随目标推进而移动（2026-11-05 反馈 1）；
2. 目标点下方有「停止目标流 / 取消分配 #N」按钮，且位置跟着目标点走（反弹 3）。
"""
import json
import pathlib
import sys

from playwright.sync_api import sync_playwright

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "tools" / "layout_evidence"
URL = "http://127.0.0.1:4173/"

# 1x1 透明 PNG：离线环境里卫星瓦片全部失败时，地图的 isStyleLoaded() 一直是 false，
# doUpdate() 直接 return，所有 GeoJSON 源停在空集合（量测会误判成“功能没生效”）。
# 这里把瓦片请求全部用这张图回应，让样式源装载完成。
TILE_PNG = bytes.fromhex(
    "89504e470d0a1a0a0000000d49484452000000010000000108060000001f15c4"
    "890000000a49444154789c6360000002000100ffff03000006000557bfabd4"
    "0000000049454e44ae426082"
)

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
        key: '25:1', callsign: '捕网-01', systemId: 25, componentId: 25, conflict: false,
        telemetry: {
          ...gs.telemetry,
          vehicle: { systemId: 25, componentId: 25, connected: true, armed: true, flightMode: 'OFFBOARD',
            systemStatus: '任务中', landedState: '空中', lastHeartbeatAt: new Date().toISOString() },
          gps: { fixType: 3, fixLabel: '3D', latitude: 31.1234567, longitude: 121.1234567, altitudeMslM: 71.8,
            relativeAltitudeM: 62.4, satellites: 18, hdop: 0.8, headingDeg: 90, groundSpeedMps: 2.0,
            updatedAt: new Date().toISOString() },
          mission: { state: 5, stateLabel: 'S5 目标接近', controlSource: '机载', taskPhase: '接近',
            activeWarningBits: 0, stateEnteredMs: Date.now(), frontDistanceM: 12, interceptionAuthorized: false,
            powerStatusBits: 0, assignmentId: payload.assignmentId, targetId: 7, availability: 1,
            interceptorType: 'net', interceptorTypeLabel: '捕网', updatedAt: new Date().toISOString() }
        },
        assignmentHistory: [], activeAssignmentId: null
      }
    },
    link: { connected: true, healthy: true, lastHeartbeatAt: new Date().toISOString() }
  })
  gs.setSelectedAircraft('25:1')
  gs.selectedCoordinate = { latitude: payload.origin.latitude, longitude: payload.origin.longitude }
  gs.targetSamples = {}
  const feed = (latitude, longitude, elapsedSeconds) => gs.handleEnvelope({ type: 'target.sample', timestamp: new Date().toISOString(), data: {
    aircraftKey: '25:1', targetId: 7, latitude, longitude, altitudeMslM: 71.2, speedMps: 2.0,
    headingDeg: 90, velocityNorthMps: 0, velocityEastMps: 2.0, elapsedSeconds } })
  for (let i = 0; i < payload.count; i += 1) feed(payload.origin.latitude, payload.origin.longitude + i * 0.0001, i * 5)
  gs.targetRepeats = { '25:1': {
    aircraftKey: '25:1', running: true, hz: 1, sent: payload.count, targetId: 7,
    source: payload.source, assignmentId: payload.source === 'assignment' ? payload.assignmentId : undefined,
    motion: { headingDeg: 90, speedMps: 2.0, verticalSpeedMps: 0, stopMode: 'distance', durationSeconds: 0, distanceMeters: 300 },
    progress: { elapsedSeconds: payload.count * 5, distanceMeters: payload.count * 10, speedMps: 2.0, headingDeg: 90 }
  } }
  return (gs.targetSamples['25:1'] || []).length
}
"""

READ = r"""
() => {
  const map = window.__missionMap
  const data = map.getSource('target-motion').serialize().data
  const end = data.features.find(f => f.properties && f.properties.kind === 'end')
  const last = data.features.find(f => f.properties && f.properties.kind === 'arrow')
  const button = document.querySelector('.map-target-stop')
  const rect = button ? button.getBoundingClientRect() : null
  const head = last ? map.project(last.geometry.coordinates) : null
  return {
    end: end ? { lon: Number(end.geometry.coordinates[0].toFixed(9)), lat: Number(end.geometry.coordinates[1].toFixed(9)) } : null,
    endLabel: end ? end.properties.labelIcon : null,
    head: head ? { x: Math.round(head.x), y: Math.round(head.y) } : null,
    button: button ? { text: button.textContent.trim(), x: Math.round(rect.x), y: Math.round(rect.y), w: Math.round(rect.width), h: Math.round(rect.height) } : null
  }
}
"""


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    origin = {"latitude": 31.1234567, "longitude": 121.1234567}
    results = {}
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=2)
        page.add_init_script(WS_STUB)
        page.route("**/arcgisonline.com/**", lambda route: route.fulfill(
            status=200, content_type="image/png", body=TILE_PNG))
        page.goto(URL, wait_until="domcontentloaded")
        page.wait_for_selector(".topbar")
        page.wait_for_timeout(1200)

        # 离线预览里瓦片请求会失败，地图 isStyleLoaded() 长期为 false，
        # MapPanel.doUpdate() 直接 return（所有 GeoJSON 源停在空集合）。
        # 量测只关心要素几何，这里把门禁放开再触发一次 store 变化让 watcher 重画。
        page.evaluate("""() => {
          const map = window.__missionMap
          map.isStyleLoaded = () => true
          const gs = document.querySelector('#app').__vue_app__.config.globalProperties.$pinia._s.get('ground-station')
          gs.setSelectedAircraft(null)
          gs.setSelectedAircraft('25:1')
        }""")

        for count in (5, 8):
            page.evaluate(INJECT, {"origin": origin, "count": count, "source": "manual", "assignmentId": 0})
            page.evaluate("""() => {
              const gs = document.querySelector('#app').__vue_app__.config.globalProperties.$pinia._s.get('ground-station')
              gs.setSelectedAircraft(null)
              gs.setSelectedAircraft('25:1')
            }""")
            page.wait_for_timeout(600)
            results[f"count{count}"] = page.evaluate(READ)

        page.screenshot(path=str(OUT / "motion-stop-1440.png"))

        page.evaluate(INJECT, {"origin": origin, "count": 8, "source": "assignment", "assignmentId": 11})
        page.evaluate("""() => {
          const gs = document.querySelector('#app').__vue_app__.config.globalProperties.$pinia._s.get('ground-station')
          gs.setSelectedAircraft(null)
          gs.setSelectedAircraft('25:1')
        }""")
        page.wait_for_timeout(600)
        results["assignment"] = page.evaluate(READ)
        canvas_rect = page.evaluate("() => { const r = document.querySelector('#mission-map').getBoundingClientRect(); return { x: Math.round(r.x), y: Math.round(r.y) } }")

        browser.close()

    first = results["count5"]
    later = results["count8"]
    assert first["end"], "缺少终点要素"
    assert later["end"], "缺少终点要素"
    same_end = first["end"] == later["end"]
    print("终点（5 拍）:", first["end"], first["endLabel"])
    print("终点（8 拍）:", later["end"], later["endLabel"])
    print("终点固定:", same_end)
    print("轨迹头 5 拍:", first["head"], "8 拍:", later["head"])
    print("停止按钮:", later["button"])
    print("分配接管时按钮:", results["assignment"]["button"]["text"] if results["assignment"]["button"] else None)

    assert same_end, "终点随目标推进而移动了"
    assert later["button"], "没有停止按钮"
    assert later["button"]["text"] == "停止目标流", later["button"]["text"]
    assert results["assignment"]["button"]["text"] == "取消分配 #11", results["assignment"]["button"]["text"]
    # 按钮应在目标点正下方约 16 px（居中）。map.project() 给的是画布内坐标，
    # getBoundingClientRect() 是视口坐标，比较前要加上画布的视口偏移。
    canvas = canvas_rect  # noqa: F821  （下面重新取值）
    assert abs(later["button"]["y"] - (canvas["y"] + later["head"]["y"] + 16)) <= 3, later
    assert abs(later["button"]["x"] + later["button"]["w"] / 2 -
               (canvas["x"] + later["head"]["x"])) <= 3, later
    print("OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
