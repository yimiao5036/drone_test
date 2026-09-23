#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""USB 双目模组 ChArUco 交互式采集工具（Windows）。

双目模组输出 SBS 全幅（默认 2560×720@30 MJPG），左右半幅各 1280×720
对应左右目。实时预览叠加角点检测与采集提示：

    空格  采一对图（左右目同帧都检出 >= --min-corners 个角点才落盘）
    Enter 结束采集（写出 meta.json）
    Esc   放弃并退出（已采图片保留）

用法：
    python capture_stereo_charuco.py --camera 0
    python capture_stereo_charuco.py --camera 1 --square-mm 22.3   # 实测边长

采集姿势要求见 相机标定工具.md：目标 ~25 对，覆盖画面 3x3 全部分区、
远/中/近三档距离与多方向倾斜。
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

EYE_NAMES = ("left", "right")


def ParseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="USB 双目 ChArUco 标定采集")
    parser.add_argument("--camera", type=int, default=0, help="摄像头设备序号")
    parser.add_argument("--width", type=int, default=2560, help="SBS 全幅宽")
    parser.add_argument("--height", type=int, default=720, help="SBS 全幅高")
    parser.add_argument("--fps", type=int, default=30, help="帧率")
    parser.add_argument("--cols", type=int, default=11, help="标定板格子列数")
    parser.add_argument("--rows", type=int, default=8, help="标定板格子行数")
    parser.add_argument("--square-mm", type=float, default=22.0,
                        help="格子边长 mm（以打印实测为准）")
    parser.add_argument("--marker-mm", type=float, default=16.0, help="标记边长 mm")
    parser.add_argument("--dictionary", default="DICT_5X5_1000", help="ArUco 字典名")
    parser.add_argument("--min-corners", type=int, default=6,
                        help="落盘要求的每目最少角点数")
    parser.add_argument("--out", default=str(Path(__file__).parent / "captures"),
                        help="采集输出根目录")
    return parser.parse_args()


def BuildBoard(args: argparse.Namespace) -> cv2.aruco.CharucoBoard:
    dictionary_id = getattr(cv2.aruco, args.dictionary, None)
    if dictionary_id is None:
        raise SystemExit(f"未知 ArUco 字典：{args.dictionary}")
    return cv2.aruco.CharucoBoard(
        (args.cols, args.rows), args.square_mm / 1000.0, args.marker_mm / 1000.0,
        cv2.aruco.getPredefinedDictionary(dictionary_id))


def OpenCamera(args: argparse.Namespace) -> cv2.VideoCapture:
    """按采集档位打开摄像头；失败时探测可用设备号后报错退出。"""
    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
        ok, frame = cap.read()
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if ok and actual_w == args.width and actual_h == args.height:
            return cap
        print(f"摄像头 {args.camera} 档位设置失败：实际 {actual_w}x{actual_h}，"
              f"要求 {args.width}x{args.height}")
        cap.release()
    else:
        print(f"摄像头 {args.camera} 打不开")

    # 探测可用设备号，帮助用户选 --camera
    available = []
    for index in range(6):
        probe = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if probe.isOpened():
            ok, frame = probe.read()
            if ok and frame is not None:
                available.append(f"{index}({frame.shape[1]}x{frame.shape[0]})")
        probe.release()
    print(f"可用设备：{', '.join(available) if available else '未探测到任何摄像头'}")
    raise SystemExit(2)


def DetectEye(detector: cv2.aruco.CharucoDetector, gray: np.ndarray):
    """返回 (charuco角点, 角点id)；检测失败返回 (None, None)。"""
    corners, ids, _, _ = detector.detectBoard(gray)
    if ids is None or corners is None:
        return None, None
    return corners, ids


def CoverageCells(corners: np.ndarray, width: int, height: int) -> set:
    """角点落入的 3x3 分区序号集合，用于覆盖度提示。"""
    cells = set()
    for pt in corners.reshape(-1, 2):
        col = min(int(pt[0] / (width / 3)), 2)
        row = min(int(pt[1] / (height / 3)), 2)
        cells.add(row * 3 + col)
    return cells


def main() -> int:
    args = ParseArgs()
    board = BuildBoard(args)
    detector = cv2.aruco.CharucoDetector(board)
    cap = OpenCamera(args)

    session_dir = Path(args.out) / datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir.mkdir(parents=True, exist_ok=True)

    eye_w, eye_h = args.width // 2, args.height
    pair_count = 0
    covered_cells = set()  # 左目分区覆盖（左右目视场接近，以左目统计）
    print(f"采集目录：{session_dir}")
    print(f"操作：空格=采一对（每目≥{args.min_corners}角点） Enter=结束 Esc=放弃")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("读取帧失败，摄像头可能掉线，已采图片保留")
            break
        halves = {
            "left": frame[:, :eye_w],
            "right": frame[:, eye_w:],
        }
        detections = {}
        for name in EYE_NAMES:
            gray = cv2.cvtColor(halves[name], cv2.COLOR_BGR2GRAY)
            detections[name] = DetectEye(detector, gray)

        # 预览叠加：左目角点绿、右目角点蓝，顶部状态行
        preview = np.hstack([halves["left"].copy(), halves["right"].copy()])
        for index, name in enumerate(EYE_NAMES):
            corners, _ = detections[name]
            if corners is not None:
                for pt in corners.reshape(-1, 2):
                    cv2.circle(preview,
                               (int(pt[0]) + index * eye_w, int(pt[1])),
                               4, (0, 255, 0) if name == "left" else (255, 128, 0),
                               1)
        counts = {name: (0 if detections[name][0] is None
                         else len(detections[name][0])) for name in EYE_NAMES}
        status = (f"pairs={pair_count}  L={counts['left']} R={counts['right']}  "
                  f"cells={len(covered_cells)}/9")
        cv2.putText(preview, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 255), 2)
        cv2.imshow("stereo charuco capture", preview)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # Esc
            print("放弃采集")
            break
        if key in (13, 10):  # Enter
            break
        if key == ord(" "):
            if all(counts[name] >= args.min_corners for name in EYE_NAMES):
                for name in EYE_NAMES:
                    cv2.imwrite(str(session_dir / f"{name}_{pair_count:03d}.png"),
                                halves[name])
                covered_cells |= CoverageCells(detections["left"][0], eye_w, eye_h)
                pair_count += 1
                print(f"已采 {pair_count} 对（L={counts['left']} "
                      f"R={counts['right']}，分区覆盖 {len(covered_cells)}/9）")
            else:
                print(f"角点不足（L={counts['left']} R={counts['right']}，"
                      f"要求各≥{args.min_corners}），未落盘")

    cap.release()
    cv2.destroyAllWindows()

    meta = {
        "image_width_full": args.width,
        "image_height": args.height,
        "fps": args.fps,
        "eye_width": eye_w,
        "eye_height": eye_h,
        "pairs": pair_count,
        "board": {
            "type": "charuco",
            "dictionary": args.dictionary,
            "cols": args.cols,
            "rows": args.rows,
            "square_m": args.square_mm / 1000.0,
            "marker_m": args.marker_mm / 1000.0,
        },
        "captured_at": datetime.datetime.now().isoformat(timespec="seconds"),
    }
    (session_dir / "meta.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"采集结束：{pair_count} 对，meta.json 已写出")
    if 0 < pair_count < 10:
        print(f"警告：有效图对 {pair_count} < 10，标定将被拒绝，请补采")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
