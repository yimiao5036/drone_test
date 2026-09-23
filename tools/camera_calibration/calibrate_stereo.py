#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线立体标定：读 capture_stereo_charuco.py 的采集目录，产出 calibration.json。

流程：
    1. 逐对图检测左右目 ChArUco 角点（任一目不足则跳过该对）；
    2. 左/右目各自 calibrateCamera → 内参 + 畸变 + 单目 RMS；
    3. 取左右共同角点 id → stereoCalibrate(CALIB_FIX_INTRINSIC) → R/T + 立体 RMS；
    4. stereoRectify + remap 生成校正拼图（rectified_preview.png）目视查极线；
    5. 基线 |T| 与标称 60mm 比对，偏差 >10% 告警（多半是格子边长没按实测填）；
    6. 列出复投影误差最大的 3 对图文件名，供剔除后重跑。

用法：
    python calibrate_stereo.py                       # 自动取 captures/ 最新会话
    python calibrate_stereo.py --session captures/20260922_170000
    python calibrate_stereo.py --exclude left_003    # 剔除指定对（不含扩展名）
"""

import argparse
import datetime
import json
import sys
from pathlib import Path

import cv2
import numpy as np

# Windows 控制台默认 GBK，中文输出强制 UTF-8（配合 chcp 65001 或 Windows Terminal）
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

MIN_PAIRS = 10          # 少于该数拒绝标定
MIN_CORNERS = 6         # 每目最少角点
NOMINAL_BASELINE_M = 0.060   # 模组标称基线（规格书），仅作偏差告警参考
BASELINE_WARN_RATIO = 0.10


def ParseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="双目 ChArUco 离线立体标定")
    parser.add_argument("--session", default=None,
                        help="采集会话目录（默认 captures/ 下最新）")
    parser.add_argument("--exclude", nargs="*", default=[],
                        help="剔除的图对名（不含扩展名，如 left_003）")
    parser.add_argument("--min-corners", type=int, default=MIN_CORNERS)
    return parser.parse_args()


def FindSession(arg: str | None) -> Path:
    if arg:
        session = Path(arg)
        if not session.is_dir():
            raise SystemExit(f"会话目录不存在：{session}")
        return session
    captures_root = Path(__file__).parent / "captures"
    sessions = sorted(p for p in captures_root.glob("*") if p.is_dir())
    if not sessions:
        raise SystemExit(f"{captures_root} 下没有采集会话，先运行 capture_stereo_charuco.py")
    return sessions[-1]


def LoadMeta(session: Path) -> dict:
    meta_path = session / "meta.json"
    if not meta_path.is_file():
        raise SystemExit(f"缺 meta.json：{meta_path}（采集脚本异常退出？）")
    return json.loads(meta_path.read_text(encoding="utf-8"))


def BuildBoard(meta: dict) -> cv2.aruco.CharucoBoard:
    """板参数一律以采集时的 meta.json 为准，防止采集/标定板不一致。"""
    board_meta = meta["board"]
    dictionary_id = getattr(cv2.aruco, board_meta["dictionary"], None)
    if dictionary_id is None:
        raise SystemExit(f"未知 ArUco 字典：{board_meta['dictionary']}")
    return cv2.aruco.CharucoBoard(
        (board_meta["cols"], board_meta["rows"]),
        board_meta["square_m"], board_meta["marker_m"],
        cv2.aruco.getPredefinedDictionary(dictionary_id))


def ListPairs(session: Path, exclude: list) -> list:
    stems = sorted(p.stem for p in session.glob("left_*.png"))
    pairs = []
    for stem in stems:
        suffix = stem[len("left"):]
        if (session / f"right{suffix}.png").is_file():
            excluded = any(ex in stem for ex in exclude)
            pairs.append((suffix, excluded))
    return pairs


def DetectPairs(session: Path, pairs: list, detector, min_corners: int):
    """逐对检测左右目角点，返回有效样本列表 [(suffix, l_corners, l_ids, r_corners, r_ids)]。"""
    samples = []
    skipped = 0
    for suffix, excluded in pairs:
        if excluded:
            continue
        left = cv2.imread(str(session / f"left{suffix}.png"), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(session / f"right{suffix}.png"), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            skipped += 1
            continue
        l_corners, l_ids, _, _ = detector.detectBoard(left)
        r_corners, r_ids, _, _ = detector.detectBoard(right)
        if (l_ids is None or r_ids is None or
                len(l_ids) < min_corners or len(r_ids) < min_corners):
            skipped += 1
            continue
        samples.append((suffix, l_corners, l_ids, r_corners, r_ids))
    return samples, skipped


def PerImageReprojErrors(board, samples, eye: str, camera_matrix, dist_coeffs) -> list:
    """逐图复投影误差：以该图单目外参投影对象点回图像平面。"""
    errors = []
    for suffix, l_c, l_ids, r_c, r_ids in samples:
        corners, ids = (l_c, l_ids) if eye == "left" else (r_c, r_ids)
        obj_pts, img_pts = board.matchImagePoints(corners, ids)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs)
        if not ok:
            continue
        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
        err = float(np.linalg.norm(proj.reshape(-1, 2) - img_pts.reshape(-1, 2),
                                   axis=1).mean())
        errors.append((err, f"{'left' if eye == 'left' else 'right'}{suffix}.png"))
    return errors


def ComputeFov(camera_matrix, width: int, height: int) -> tuple:
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    return (float(np.degrees(2 * np.arctan(width / (2 * fx)))),
            float(np.degrees(2 * np.arctan(height / (2 * fy)))))


def main() -> int:
    args = ParseArgs()
    session = FindSession(args.session)
    meta = LoadMeta(session)
    board = BuildBoard(meta)
    detector = cv2.aruco.CharucoDetector(board)
    width, height = meta["eye_width"], meta["eye_height"]
    print(f"会话：{session}  单目分辨率：{width}x{height}")

    pairs = ListPairs(session, args.exclude)
    samples, skipped = DetectPairs(session, pairs, detector, args.min_corners)
    print(f"图对：目录 {len(pairs)}，剔除 {len(args.exclude)}，"
          f"角点不足跳过 {skipped}，有效 {len(samples)}")
    if len(samples) < MIN_PAIRS:
        raise SystemExit(f"有效图对 {len(samples)} < {MIN_PAIRS}，请补采后再标定")

    image_size = (width, height)
    results = {}
    calib_per_eye = {}
    for eye in ("left", "right"):
        obj_points, img_points = [], []
        for suffix, l_c, l_ids, r_c, r_ids in samples:
            corners, ids = (l_c, l_ids) if eye == "left" else (r_c, r_ids)
            obj_pts, img_pts = board.matchImagePoints(corners, ids)
            obj_points.append(obj_pts)
            img_points.append(img_pts)
        rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
            obj_points, img_points, image_size, None, None)
        calib_per_eye[eye] = (camera_matrix, dist_coeffs)
        errors = PerImageReprojErrors(board, samples, eye, camera_matrix, dist_coeffs)
        errors.sort(reverse=True)
        fov_h, fov_v = ComputeFov(camera_matrix, width, height)
        results[eye] = {
            "camera_matrix": camera_matrix.tolist(),
            "dist_coeffs": dist_coeffs.ravel().tolist(),
            "rms": float(rms),
        }
        print(f"[{eye}] 单目 RMS={rms:.3f}px  fx={camera_matrix[0, 0]:.1f} "
              f"fy={camera_matrix[1, 1]:.1f} cx={camera_matrix[0, 2]:.1f} "
              f"cy={camera_matrix[1, 2]:.1f}  HFOV={fov_h:.1f}° VFOV={fov_v:.1f}°")
        print(f"  复投影误差最大 3 对：" +
              ", ".join(f"{name}={err:.2f}px" for err, name in errors[:3]))
        results[eye]["fov_h_deg"] = fov_h
        results[eye]["fov_v_deg"] = fov_v

    # 立体标定：左右共同角点 id，固定内参只求 R/T
    stereo_obj, stereo_l, stereo_r = [], [], []
    chessboard_corners = board.getChessboardCorners()
    for suffix, l_c, l_ids, r_c, r_ids in samples:
        common = np.intersect1d(l_ids.ravel(), r_ids.ravel())
        if len(common) < args.min_corners:
            continue
        l_map = {int(i): p for i, p in zip(l_ids.ravel(), l_c.reshape(-1, 2))}
        r_map = {int(i): p for i, p in zip(r_ids.ravel(), r_c.reshape(-1, 2))}
        stereo_obj.append(chessboard_corners[common].astype(np.float32))
        stereo_l.append(np.array([l_map[int(i)] for i in common], dtype=np.float32))
        stereo_r.append(np.array([r_map[int(i)] for i in common], dtype=np.float32))
    print(f"立体标定样本：{len(stereo_obj)} 对（左右共同角点≥{args.min_corners}）")
    if len(stereo_obj) < MIN_PAIRS:
        raise SystemExit("左右共同角点样本不足，无法立体标定（采集时板子需同时入两目画面）")

    k_l, d_l = calib_per_eye["left"]
    k_r, d_r = calib_per_eye["right"]
    stereo_rms, _, _, _, _, rot, trans, _, _ = cv2.stereoCalibrate(
        stereo_obj, stereo_l, stereo_r, k_l, d_l, k_r, d_r, image_size,
        flags=cv2.CALIB_FIX_INTRINSIC)
    baseline = float(np.linalg.norm(trans))
    print(f"[stereo] RMS={stereo_rms:.3f}px  基线 |T|={baseline * 1000:.1f}mm")
    baseline_dev = abs(baseline - NOMINAL_BASELINE_M) / NOMINAL_BASELINE_M
    if baseline_dev > BASELINE_WARN_RATIO:
        print(f"警告：基线与标称 {NOMINAL_BASELINE_M * 1000:.0f}mm 偏差 "
              f"{baseline_dev * 100:.0f}%，优先检查 --square-mm 是否按打印实测填写")

    # 校正拼图：取首个有效对，stereoRectify + remap 后左右拼接画水平极线
    r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(
        k_l, d_l, k_r, d_r, image_size, rot, trans)
    suffix = samples[0][0]
    imgs = {}
    for eye, k, d, r, p in (("left", k_l, d_l, r1, p1),
                            ("right", k_r, d_r, r2, p2)):
        img = cv2.imread(str(session / f"{eye}{suffix}.png"))
        map_x, map_y = cv2.initUndistortRectifyMap(k, d, r, p, image_size, cv2.CV_32FC1)
        imgs[eye] = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)
    stacked = np.hstack([imgs["left"], imgs["right"]])
    for y in range(0, height, 40):
        cv2.line(stacked, (0, y), (2 * width, y), (0, 255, 0), 1)
    preview_path = session / "rectified_preview.png"
    cv2.imwrite(str(preview_path), stacked)
    print(f"校正预览（目视查同名点是否落在同一水平绿线）：{preview_path}")

    output = {
        "image_width": width,
        "image_height": height,
        "board": meta["board"],
        "left": results["left"],
        "right": results["right"],
        "stereo": {
            "R": rot.tolist(),
            "T_m": trans.ravel().tolist(),
            "baseline_m": baseline,
            "rms": float(stereo_rms),
        },
        "fov_deg": {
            "left_h": results["left"]["fov_h_deg"],
            "left_v": results["left"]["fov_v_deg"],
            "right_h": results["right"]["fov_h_deg"],
            "right_v": results["right"]["fov_v_deg"],
        },
        "opencv_version": cv2.__version__,
        "calibrated_at": datetime.datetime.now().isoformat(timespec="seconds"),
    }
    out_path = session / "calibration.json"
    out_path.write_text(json.dumps(output, ensure_ascii=False, indent=2),
                        encoding="utf-8")
    print(f"标定文件：{out_path}")

    # 验收阈值提示（判定留给人，脚本只报告）
    ok_l = results["left"]["rms"] < 1.0
    ok_r = results["right"]["rms"] < 1.0
    ok_s = stereo_rms < 1.0
    print(f"阈值检查：单目 RMS<1.0 [{'通过' if ok_l and ok_r else '未达标'}]  "
          f"立体 RMS<1.0 [{'通过' if ok_s else '未达标'}]")
    return 0 if (ok_l and ok_r and ok_s) else 1


if __name__ == "__main__":
    raise SystemExit(main())
