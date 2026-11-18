"""生成双目测距 v2 的黄金值数据（C++ 单测对拍用）。

产物（tests/perception/data/）：
  - stereo_golden.h   ：合成标定的预置矩阵、映射表采样点期望值、注入关键点、
                        期望视差/有效掩膜、bbox 聚合期望值（C++ 直接 include）
  - stereo_ref.bin    ：基准图（SBS 右半投影，raw 8bit W×H）
  - stereo_aux.bin    ：辅助图（SBS 左半投影，raw 8bit W×H）

方法：合成一套与真实模组同构的双目标定（T.x>0，SBS 左半=物理右目），
把一块 Z0=1.0m 的纹理平面按真实 K/D/R/T 投影渲染进两台相机（含畸变），
再用**已验证的 Python 参考实现** tools/camera_test_ok/stereo_core.py 做
校正+稀疏 Census 匹配，把结果固化成黄金值。C++ 侧在同样的矩阵与输入图上
跑自研实现，应与黄金值在容差内一致（见 stereo_ranger_core_test.cpp）。

依赖：本机 Python + numpy + opencv；参考实现位于 tools/camera_test_ok/
（该目录被 .gitignore 排除，脚本仅开发机重跑黄金值时使用；提交的是产物）。
何时重跑：仅在黄金值需要更新时（如参考实现语义变更）；日常构建不需要。
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "camera_test_ok"))
try:
    from stereo_core import StereoConfig, census_transform, distance_in_bbox, sparse_match
except ImportError as exc:  # pragma: no cover - 开发机脚本
    raise SystemExit(
        "找不到参考实现 tools/camera_test_ok/stereo_core.py（该目录未入库，"
        f"重跑黄金值需要它）: {exc}")

OUT_DIR = Path(__file__).resolve().parent / "data"

# ---- 合成标定（与真实模组同构：SBS 左半=物理右目，T.x>0）--------------------
W, H = 320, 240
PLANE_Z0_M = 1.0            # 纹理平面在 cam2（基准）系中的深度
K1 = np.array([[200.0, 0.0, 150.0], [0.0, 200.5, 118.0], [0.0, 0.0, 1.0]])  # SBS 左
K2 = np.array([[199.5, 0.0, 205.0], [0.0, 200.0, 122.0], [0.0, 0.0, 1.0]])  # SBS 右=基准
D1 = np.array([-0.05, 0.08, 0.0005, -0.0004, -0.02])
D2 = np.array([-0.04, 0.06, -0.0006, 0.0005, -0.015])
R_STEREO = cv2.Rodrigues(np.array([0.004, 0.009, -0.002]))[0]   # cam1→cam2 旋转
T_STEREO = np.array([[0.05], [0.0008], [-0.0012]])              # cam1 原点在 cam2 系中的位置

# ---- 匹配配置（与 C++ 单测使用的一致）--------------------------------------
CFG = dict(disp_min=4.0, disp_max=40.0, coarse_stride=2, fine_halfwidth=6,
           uniqueness_ratio=1.05, lr_tol=1.0, census_win=7,
           max_points_per_target=80, grid_cells=4)


def texture(px, py):
    """平面上的确定性纹理（物理坐标，米）：哈希噪声 + 低频结构。"""
    cell = 0.02
    cx = np.floor(px / cell).astype(np.int64)
    cy = np.floor(py / cell).astype(np.int64)
    h = (cx * 73856093) ^ (cy * 19349663)
    h = (h * 2654435761) & 0xFFFFFFFF
    noise = (h % 256).astype(np.float64)
    low = np.sin(px * 6.0) * np.cos(py * 5.0)
    v = 60.0 + 0.55 * noise + 25.0 * low
    return np.clip(v, 0, 255).astype(np.uint8)


def undistort_rays(k, d, width, height):
    """整幅像素的归一化理想坐标（畸变逆解）。"""
    us, vs = np.meshgrid(np.arange(width, dtype=np.float64),
                         np.arange(height, dtype=np.float64))
    pts = np.stack([us.ravel(), vs.ravel()], axis=1).reshape(-1, 1, 2)
    norm = cv2.undistortPoints(pts, k, d.reshape(-1, 1))
    return (norm.reshape(height, width, 2)[..., 0],
            norm.reshape(height, width, 2)[..., 1])


def render_eye(k, d, use_other):
    """渲染一台相机：像素射线与 Z0 平面求交，采样纹理。

    use_other=False：本相机即 cam2（平面定义所在系）；
    use_other=True ：本相机是 cam1，射线经 R/T 变换到 cam2 系后求交。
    """
    x, y = undistort_rays(k, d, W, H)
    ray = np.stack([x, y, np.ones_like(x)], axis=-1)      # (H,W,3)
    if not use_other:
        p = ray * PLANE_Z0_M
    else:
        ray_c2 = ray @ R_STEREO.T + T_STEREO.reshape(3)   # 方向 + 平移
        s = (PLANE_Z0_M - float(T_STEREO[2, 0])) / ray_c2[..., 2]
        p = ray_c2 * s[..., None]
    return texture(p[..., 0], p[..., 1])


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # ---- 与 export_rectify.py 相同的基准口径：cam2（SBS 右半）为基准 ----
    rb = R_STEREO.T
    tb = -R_STEREO.T @ T_STEREO
    r_ref, r_aux, p_ref, p_aux, _q, _, _ = cv2.stereoRectify(
        K2, D2.reshape(-1, 1), K1, D1.reshape(-1, 1), (W, H), rb, tb,
        alpha=0, flags=cv2.CALIB_ZERO_DISPARITY)
    baseline_m = abs(float(p_aux[0, 3]) / float(p_aux[0, 0]))
    fx_rect = float(p_ref[0, 0])
    fxb = fx_rect * baseline_m

    map_ref = cv2.initUndistortRectifyMap(K2, D2.reshape(-1, 1), r_ref, p_ref,
                                          (W, H), cv2.CV_32FC1)
    map_aux = cv2.initUndistortRectifyMap(K1, D1.reshape(-1, 1), r_aux, p_aux,
                                          (W, H), cv2.CV_32FC1)

    # ---- 渲染源图（cam2=基准；cam1=辅助）----
    img_ref = render_eye(K2, D2, use_other=False)
    img_aux = render_eye(K1, D1, use_other=True)
    img_ref.tofile(OUT_DIR / "stereo_ref.bin")
    img_aux.tofile(OUT_DIR / "stereo_aux.bin")

    # ---- 参考实现：校正 + 稀疏匹配（注入关键点）----
    rect_ref = cv2.remap(img_ref, map_ref[0], map_ref[1], cv2.INTER_LINEAR)
    rect_aux = cv2.remap(img_aux, map_aux[0], map_aux[1], cv2.INTER_LINEAR)
    cfg = StereoConfig(fx=fx_rect, fy=float(p_ref[1, 1]), cx=float(p_ref[0, 2]),
                       cy=float(p_ref[1, 2]), baseline_mm=baseline_m * 1000.0,
                       disp_min=CFG["disp_min"], disp_max=CFG["disp_max"],
                       coarse_stride=CFG["coarse_stride"],
                       fine_halfwidth=CFG["fine_halfwidth"],
                       uniqueness_ratio=CFG["uniqueness_ratio"],
                       lr_tol=CFG["lr_tol"], max_points_per_target=80,
                       grid_cells=4)

    # 关键点：8×6 网格错开取点（≥min_points=30，覆盖目标区域）
    keypoints = []
    for gy in range(6):
        for gx in range(8):
            keypoints.append([26.0 + gx * 36.0 + 2.0 * (gy % 2),
                              34.0 + gy * 34.0 + 2.0 * (gx % 3)])
    keypoints = np.array(keypoints, np.float32)

    cen_ref = census_transform(rect_ref)
    cen_aux = census_transform(rect_aux)
    match = sparse_match(rect_ref, rect_aux, cfg, pts=keypoints,
                         census=(cen_ref, cen_aux))
    disp = np.where(match.valid, match.disp, np.nan)

    # 目标 bbox（覆盖全部关键点，供聚合与 C++ 行带）
    bbox = (20, 20, 305, 235)
    info = distance_in_bbox(match, bbox, fxb, p_ref, erode=5, min_points=30,
                            near_pct=90.0, near_tol=3.0, view_w=W, view_h=H)
    if info is None:
        raise SystemExit("参考实现聚合失败：合成场景点数不足，检查渲染/参数")

    # 自查：平面真值应能被参考实现复原（合成链路自检）
    z_err = abs(info["z"] - PLANE_Z0_M) / PLANE_Z0_M
    print(f"[自检] 参考实现 z={info['z']:.4f}m 真值={PLANE_Z0_M:.4f}m 误差={z_err*100:.2f}%")
    if z_err > 0.05:
        raise SystemExit("合成场景自检失败：参考实现距离偏离真值 >5%")

    # 映射表采样点（整数坐标，C++ RectToSourceRef 用 lround 取整后查询）
    samples = []
    for (su, sv) in [(0, 0), (W - 1, 0), (0, H - 1), (W - 1, H - 1),
                     (W // 2, H // 2), (37, 91), (123, 45), (201, 177),
                     (288, 210), (75, 12), (16, 150), (300, 120)]:
        samples.append((su, sv, float(map_ref[0][sv, su]), float(map_ref[1][sv, su]),
                        float(map_aux[0][sv, su]), float(map_aux[1][sv, su])))

    valid_count = int(match.valid.sum())
    print(f"[生成] 关键点 {len(keypoints)} 个，有效 {valid_count} 个；"
          f"聚合 z={info['z']:.4f}m d={info['d']:.2f}px n={info['n']} "
          f"iqr={info['iqr']:.3f}")

    # ---- 写 C++ 头（黄金值）----
    def fmt(v):
        return "std::numeric_limits<float>::quiet_NaN()" if not np.isfinite(v) else f"{float(v):.6f}f"

    lines = []
    lines.append("// 由 tests/perception/gen_stereo_golden.py 生成，勿手改。")
    lines.append("// 合成标定 + 参考实现（tools/camera_test_ok/stereo_core.py）黄金值。")
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <limits>")
    lines.append("")
    lines.append("#include \"perception/stereo_ranger_core.h\"")
    lines.append("")
    lines.append("namespace drone::perception::testdata {")
    lines.append("")
    lines.append(f"inline constexpr int kViewWidth = {W};")
    lines.append(f"inline constexpr int kViewHeight = {H};")
    lines.append("")

    def mat(name, m):
        rows = m if m.ndim == 2 else m.reshape(1, -1)
        body = ", ".join(f"{float(v):.10f}" for v in rows.reshape(-1))
        lines.append(f"inline constexpr double {name}[] = {{{body}}};")

    mat("kKRef", K2)
    mat("kDRef", D2)
    mat("kRRef", r_ref)
    mat("kPRef", p_ref)
    mat("kKAux", K1)
    mat("kDAux", D1)
    mat("kRAux", r_aux)
    mat("kPAux", p_aux)
    lines.append("")
    lines.append("inline StereoRectifyMatrices Matrices() {")
    lines.append("    StereoRectifyMatrices m;")
    lines.append("    m.view_width = kViewWidth;")
    lines.append("    m.view_height = kViewHeight;")
    lines.append("    for (int i = 0; i < 9; ++i) { m.k_ref[i] = kKRef[i]; m.k_aux[i] = kKAux[i]; }")
    lines.append("    for (int i = 0; i < 5; ++i) { m.d_ref[i] = kDRef[i]; m.d_aux[i] = kDAux[i]; }")
    lines.append("    for (int i = 0; i < 9; ++i) { m.r_ref[i] = kRRef[i]; m.r_aux[i] = kRAux[i]; }")
    lines.append("    for (int i = 0; i < 12; ++i) { m.p_ref[i] = kPRef[i]; m.p_aux[i] = kPAux[i]; }")
    lines.append(f"    m.baseline_m = {baseline_m:.10f};")
    lines.append(f"    m.fx_rect_px = {fx_rect:.10f};")
    lines.append(f"    m.fxb_m_px = {fxb:.10f};")
    lines.append("    return m;")
    lines.append("}")
    lines.append("")
    lines.append("/// 映射表采样：rect 整数坐标 → rect→source 采样坐标（OpenCV 黄金值）")
    lines.append("struct MapSample { int u; int v; float ref_x; float ref_y; float aux_x; float aux_y; };")
    lines.append("inline constexpr MapSample kMapSamples[] = {")
    for su, sv, rx, ry, ax, ay in samples:
        lines.append(f"    {{{su}, {sv}, {rx:.6f}f, {ry:.6f}f, {ax:.6f}f, {ay:.6f}f}},")
    lines.append("};")
    lines.append("")
    lines.append("/// 注入关键点（校正图坐标）与参考实现期望视差（无效=NaN）")
    lines.append("inline constexpr float kKeypoints[][2] = {")
    for kp in keypoints:
        lines.append(f"    {{{kp[0]:.3f}f, {kp[1]:.3f}f}},")
    lines.append("};")
    lines.append("inline constexpr float kExpectedDisp[] = {")
    for d in disp:
        lines.append(f"    {fmt(d)},")
    lines.append("};")
    lines.append("")
    lines.append(f"inline constexpr int kBboxX0 = {bbox[0]};")
    lines.append(f"inline constexpr int kBboxY0 = {bbox[1]};")
    lines.append(f"inline constexpr int kBboxX1 = {bbox[2]};")
    lines.append(f"inline constexpr int kBboxY1 = {bbox[3]};")
    lines.append(f"inline constexpr double kExpectedZ = {info['z']:.6f};")
    lines.append(f"inline constexpr double kExpectedDisparity = {info['d']:.6f};")
    lines.append(f"inline constexpr int kExpectedClusterPoints = {info['n']};")
    lines.append(f"inline constexpr double kExpectedIqr = {info['iqr']:.6f};")
    lines.append(f"inline constexpr double kExpectedEuclid = {info['euclid']:.6f};")
    lines.append("")
    lines.append("}  // namespace drone::perception::testdata")
    lines.append("")

    # newline="\n"：Windows 文本模式默认写 CRLF，仓库强制 LF（.gitattributes）
    with open(OUT_DIR / "stereo_golden.h", "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[OK] 已写出 {OUT_DIR}/stereo_golden.h / stereo_ref.bin / stereo_aux.bin")
    return 0


if __name__ == "__main__":
    sys.exit(main())
