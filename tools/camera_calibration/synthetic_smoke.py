# -*- coding: utf-8 -*-
"""标定管线合成冒烟：虚拟双目渲染 ChArUco 板 → 跑 calibrate_stereo.py → 核对恢复的内参。"""
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

rng = np.random.default_rng(42)

# 虚拟相机：1280x720，fx=fy=900, cx=640, cy=360，右目相对左目平移 -60mm
W, H = 1280, 720
K_TRUE = np.array([[900.0, 0, 640.0], [0, 900.0, 360.0], [0, 0, 1.0]])
BASELINE = 0.06

COLS, ROWS, SQ = 11, 8, 0.022
board = cv2.aruco.CharucoBoard((COLS, ROWS), SQ, 0.016,
                               cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000))

# 渲染板图：2000 px/米，使源图与屏幕成像尺度约 1:1
# （源图太小会把 ArUco 位模式毁掉，太大则 warpPerspective 缩采样混叠）
S = 2000
board_img = board.generateImage((int(COLS * SQ * S), int(ROWS * SQ * S)), marginSize=0)

session = Path(sys.argv[1])
session.mkdir(parents=True, exist_ok=True)


# 板平面尺寸（米），位姿以板中心为基准
BOARD_W_M = COLS * SQ
BOARD_H_M = ROWS * SQ


def RenderEye(rvec, tvec, K):
    """板平面（z=0，米，位姿以板中心为基准）渲染到 1280x720。"""
    R, _ = cv2.Rodrigues(rvec)
    # 板图像素(u,v) -> 以板中心为原点的米制坐标(u/S - W/2, v/S - H/2, 0, 1)
    to_metric = np.array([[1.0 / S, 0, -BOARD_W_M / 2],
                          [0, 1.0 / S, -BOARD_H_M / 2],
                          [0, 0, 1.0]])
    M = K @ np.column_stack([R[:, 0], R[:, 1], tvec]) @ to_metric
    return cv2.warpPerspective(board_img, M, (W, H), flags=cv2.INTER_LINEAR,
                               borderValue=(255, 255, 255))


poses = []
for i in range(15):
    rx = np.radians(rng.uniform(-25, 25))
    ry = np.radians(rng.uniform(-25, 25))
    rz = np.radians(rng.uniform(-15, 15))
    z = rng.uniform(0.35, 0.55)
    x = rng.uniform(-0.04, 0.04)
    y = rng.uniform(-0.03, 0.03)
    poses.append((np.array([rx, ry, rz]), np.array([x, y, z])))

for idx, (rvec, tvec) in enumerate(poses):
    left = RenderEye(rvec, tvec, K_TRUE)
    # 右目：相机在 x 负方向 60mm → 板相对右目平移 t + [baseline, 0, 0]（旋转一致）
    t_right = tvec + np.array([BASELINE, 0.0, 0.0])
    right = RenderEye(rvec, t_right, K_TRUE)
    # 轻度模糊+噪声模拟实拍
    for name, img in (("left", left), ("right", right)):
        noise = rng.normal(0, 1.5, img.shape).astype(np.float32)
        img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        # imencode+write_bytes：Unicode 路径安全（cv2.imwrite 不支持非 ASCII 路径）
        ok, buf = cv2.imencode(".png", img)
        assert ok
        (session / f"{name}_{idx:03d}.png").write_bytes(buf.tobytes())

meta = {
    "image_width_full": 2560, "image_height": 720, "fps": 30,
    "eye_width": W, "eye_height": H, "pairs": len(poses),
    "board": {"type": "charuco", "dictionary": "DICT_5X5_1000",
              "cols": COLS, "rows": ROWS, "square_m": SQ, "marker_m": 0.016},
    "captured_at": "2026-09-22T00:00:00",
}
(session / "meta.json").write_text(json.dumps(meta), encoding="utf-8")
print(f"合成采集完成：{session}，{len(poses)} 对")

result = subprocess.run(
    [sys.executable, str(Path(__file__).parent / "calibrate_stereo.py"),
     "--session", str(session)],
    capture_output=True, text=True, encoding="utf-8", errors="replace")
print(result.stdout)
print(result.stderr, file=sys.stderr)

calib = json.loads((session / "calibration.json").read_text(encoding="utf-8"))
for eye in ("left", "right"):
    K = np.array(calib[eye]["camera_matrix"])
    print(f"[{eye}] fx 真值900 测得 {K[0,0]:.1f} | cx 真值640 测得 {K[0,2]:.1f} | "
          f"cy 真值360 测得 {K[1,2]:.1f} | RMS {calib[eye]['rms']:.3f}")
print(f"基线 真值60mm 测得 {calib['stereo']['baseline_m']*1000:.1f}mm | "
      f"立体 RMS {calib['stereo']['rms']:.3f}")
print("退出码:", result.returncode)
