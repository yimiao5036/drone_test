"""从标定产物导出双目测距 v2 的预置校正矩阵（config/stereo_rectify_uvc_1280x720.json）。

用途：主工程 C++ 侧不引入 OpenCV，stereoRectify 一次性在本脚本（Windows 开发机，
Python + OpenCV）算好，矩阵随程序入库；C++ 侧只做映射表生成与 remap。

★ 目序（2026-10-09/10-10 实测定论，详见 docs/superpowers/specs/2026-10-10-双目测距v2-稀疏Census-design.md §1）：
   - 标定 JSON 的 left/right 指 SBS 半幅：left=SBS 左半（物理右目）、right=SBS 右半（物理左目）；
   - OpenCV 约定 T 是 cam1 原点在 cam2 系中的位置，本模组 T.x=+0.0601>0
     ⇒ cam1（SBS 左半）物理上是【右】目；
   - 视差必须为正 ⇒ 基准图取 SBS 右半（= 物理左目 = 主工程"左目 Topic"）。
     因此以 cam2 为基准：R'=Rᵀ、T'=−RᵀT，stereoRectify 输入顺序 (K2,D2,K1,D1)。
   - 主工程 stereo_frame_splitter 的左目 Topic 就是 SBS 右半，语义一致。

何时重跑：相机标定更新后（config/camera_calibration_uvc_1280x720.json 变化）、
         或更换相机模组/分辨率档位后，必须重跑本脚本。

用法：
    python tools/camera_calibration/export_rectify.py
    python tools/camera_calibration/export_rectify.py --calib <标定json> --out <输出json>
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

# Windows 控制台默认 GBK，中文输出前先切 UTF-8（同 distance_app.py 的做法）
sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def main() -> int:
    repo_root = Path(__file__).resolve().parents[2]
    ap = argparse.ArgumentParser(description="导出双目测距 v2 预置校正矩阵")
    ap.add_argument("--calib", default=str(repo_root / "config" /
                                          "camera_calibration_uvc_1280x720.json"),
                    help="输入标定 JSON（ChArUco 标定产物）")
    ap.add_argument("--out", default=str(repo_root / "config" /
                                         "stereo_rectify_uvc_1280x720.json"),
                    help="输出预置矩阵 JSON")
    args = ap.parse_args()

    with open(args.calib, "r", encoding="utf-8") as f:
        calib = json.load(f)

    # 标定 JSON 的 left/right = SBS 半幅（不是物理目）
    K1 = np.array(calib["left"]["camera_matrix"], np.float64)      # SBS 左半 = 物理右目
    K2 = np.array(calib["right"]["camera_matrix"], np.float64)     # SBS 右半 = 物理左目
    D1 = np.array(calib["left"]["dist_coeffs"], np.float64).reshape(-1, 1)
    D2 = np.array(calib["right"]["dist_coeffs"], np.float64).reshape(-1, 1)
    R = np.array(calib["stereo"]["R"], np.float64)
    T = np.array(calib["stereo"]["T_m"], np.float64).reshape(3, 1)
    width = int(calib.get("image_width", 1280))
    height = int(calib.get("image_height", 720))

    if T[0, 0] <= 0:
        print(f"[FAIL] T_m[0]={T[0,0]:.6f} 应为正（cam1 在 cam2 的 +x 侧）；"
              f"标定文件或目序假设不符，拒绝导出。", file=sys.stderr)
        return 1

    # 以 cam2（SBS 右半）为基准：cam2→cam1 的 R'/T'
    rb = R.T
    tb = -R.T @ T
    r_ref, r_aux, p_ref, p_aux, q_mat, _, _ = cv2.stereoRectify(
        K2, D2, K1, D1, (width, height), rb, tb,
        alpha=0, flags=cv2.CALIB_ZERO_DISPARITY)

    # 基线取校正后 P2 的口径（= |T|，与 T' 的 x 分量同量级；不要用 T_m[0]，差 0.057%）
    baseline_m = abs(float(p_aux[0, 3]) / float(p_aux[0, 0]))
    fx_rect_px = float(p_ref[0, 0])
    fxb_m_px = fx_rect_px * baseline_m

    # 来源路径尽量存仓库相对路径（跨机器可读；在仓库外则存原样）
    try:
        source_rel = Path(args.calib).resolve().relative_to(repo_root).as_posix()
    except ValueError:
        source_rel = Path(args.calib).as_posix()

    out = {
        "note": ("双目测距 v2 预置校正矩阵：基准图 = SBS 右半 = 物理左目 = 主工程左目 Topic；"
                 "辅助图 = SBS 左半 = 物理右目。视差 d = u_ref - u_aux > 0，Z = fxb_m_px / d（米）。"
                 "由 tools/camera_calibration/export_rectify.py 生成，勿手改；重标定后必须重跑。"),
        "source_calibration": source_rel,
        "calibrated_at": calib.get("calibrated_at", ""),
        "opencv_version": calib.get("opencv_version", cv2.__version__),
        "view_width": width,
        "view_height": height,
        "baseline_m": baseline_m,
        "fx_rect_px": fx_rect_px,
        "fxb_m_px": fxb_m_px,
        "K_ref": K2.tolist(),
        "D_ref": D2.reshape(-1).tolist(),
        "R_ref": r_ref.tolist(),
        "P_ref": p_ref.tolist(),
        "K_aux": K1.tolist(),
        "D_aux": D1.reshape(-1).tolist(),
        "R_aux": r_aux.tolist(),
        "P_aux": p_aux.tolist(),
        "Q": q_mat.tolist(),
    }
    # newline="\n"：Windows 下文本模式默认把 \n 转成 CRLF，仓库强制 LF（.gitattributes）
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
        f.write("\n")

    print(f"[OK] 已写出 {args.out}")
    print(f"   fx_rect = {fx_rect_px:.5f} px   主点 = ({p_ref[0,2]:.4f}, {p_ref[1,2]:.4f})")
    print(f"   基线 B  = {baseline_m*1000:.3f} mm   fxb = {fxb_m_px:.3f} m·px")
    print(f"   对照指南 §3.2：fx 1057.33492 / 主点 692.7685,529.0832 / B 0.060120 / fxb 63.57")
    return 0


if __name__ == "__main__":
    sys.exit(main())
