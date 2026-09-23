#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 ChArUco 标定板打印图（PNG，按指定 DPI 精确物理尺寸）。

用法：
    python generate_charuco_board.py
    python generate_charuco_board.py --cols 11 --rows 8 --square-mm 24 --marker-mm 18 --dpi 300

打印要点（标定误差最大来源）：
    1. 必须按脚本输出的 DPI 无缩放打印（打印对话框选"实际大小/100%"）；
    2. 打印后用尺子实测格子边长，实测值喂给采集/标定脚本的 --square-mm；
    3. 贴平到硬质平板（KT板/亚克力），褶皱会引入不可忽略的误差。
"""

import argparse
import sys
from pathlib import Path

import cv2

# Windows 控制台默认 GBK，中文输出强制 UTF-8（配合 chcp 65001 或 Windows Terminal）
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


def SaveImageUnicode(path: Path, image) -> bool:
    """Unicode 路径安全写图（cv2.imwrite 在 Windows 不支持非 ASCII 路径，静默失败）。"""
    ok, buf = cv2.imencode(".png", image)
    if not ok:
        return False
    try:
        path.write_bytes(buf.tobytes())
    except OSError:
        return False
    return path.is_file()


def ParseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="生成 ChArUco 标定板打印图")
    parser.add_argument("--cols", type=int, default=11, help="格子列数（横向）")
    parser.add_argument("--rows", type=int, default=8, help="格子行数（纵向）")
    parser.add_argument("--square-mm", type=float, default=22.0, help="格子边长 mm")
    parser.add_argument("--marker-mm", type=float, default=16.0,
                        help="ArUco 标记边长 mm（必须小于格子）")
    parser.add_argument("--dictionary", default="DICT_5X5_1000",
                        help="ArUco 字典名（需与采集/标定一致）")
    parser.add_argument("--dpi", type=int, default=300, help="打印 DPI")
    parser.add_argument("--margin-mm", type=float, default=10.0, help="白边宽度 mm")
    parser.add_argument("--out", default="charuco_board.png", help="输出 PNG 路径")
    return parser.parse_args()


def main() -> int:
    args = ParseArgs()
    if args.cols < 3 or args.rows < 3:
        raise SystemExit("格子行列数至少为 3")
    if args.marker_mm >= args.square_mm:
        raise SystemExit("标记边长必须小于格子边长")
    dictionary_id = getattr(cv2.aruco, args.dictionary, None)
    if dictionary_id is None:
        raise SystemExit(f"未知 ArUco 字典：{args.dictionary}")

    # 按 DPI 精确换算像素尺寸：px = mm / 25.4 * dpi
    px_per_mm = args.dpi / 25.4
    board_w_px = round(args.cols * args.square_mm * px_per_mm)
    board_h_px = round(args.rows * args.square_mm * px_per_mm)
    margin_px = round(args.margin_mm * px_per_mm)
    out_w = board_w_px + 2 * margin_px
    out_h = board_h_px + 2 * margin_px

    board = cv2.aruco.CharucoBoard(
        (args.cols, args.rows), args.square_mm / 1000.0, args.marker_mm / 1000.0,
        cv2.aruco.getPredefinedDictionary(dictionary_id))
    image = board.generateImage((out_w, out_h), marginSize=margin_px)
    if not SaveImageUnicode(Path(args.out), image):
        raise SystemExit(f"标定板写入失败：{args.out}")

    print(f"标定板已生成：{args.out}")
    print(f"  格子：{args.cols}x{args.rows}，边长 {args.square_mm}mm，"
          f"标记 {args.marker_mm}mm，字典 {args.dictionary}")
    print(f"  图像：{out_w}x{out_h}px @ {args.dpi} DPI，"
          f"板面 {args.cols * args.square_mm:.0f}x{args.rows * args.square_mm:.0f}mm")
    # A4 横放可用面积需扣除四周白边
    fits_a4 = (args.cols * args.square_mm <= 297 - 2 * args.margin_mm and
               args.rows * args.square_mm <= 210 - 2 * args.margin_mm)
    print(f"  A4 横放（297x210mm，含 {args.margin_mm}mm 白边）可容纳："
          f"{'是' if fits_a4 else '否，请缩小格子或改用 A3'}")
    print("打印时选“实际大小/100%”，打印后实测格子边长并以实测值标定")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
