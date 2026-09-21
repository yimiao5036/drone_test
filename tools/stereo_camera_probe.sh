#!/usr/bin/env bash
# stereo_camera_probe.sh —— 双目 UVC 相机硬件事实确认探测脚本（香橙派 RK3588 板上运行）
#
# 用途：为《双目深度估计思路.md》P3/P4 待确认项做板上实证：
#   1) 枚举 UVC 采集节点（SBS 同帧单流应仅 1 个；双流会出 2 个）
#   2) 列出全部格式/分辨率/帧间隔，与规格书对拍
#   3) 逐档实测 MJPG 帧率（USB2.0 带宽验证）
#   4) 确认 buffer 时间戳来源（印证设计文档 §3.3）
#   5) 目标档位抓帧存图（SBS 判读 + 画质检查）
#
# 依赖：v4l-utils 与 usbutils（sudo apt install -y v4l-utils usbutils）；python3 或 ffmpeg 可选（用于拆分抓帧）
# 用法：bash tools/stereo_camera_probe.sh [输出根目录=.]
#       可用 DEV=/dev/videoN 环境变量指定探测节点（默认取第一个采集节点）
# 注意：运行期间确保没有其他进程占用相机节点；脚本每步失败仅记录不中断。
#
# 修订记录：
#   - 修复采集节点识别：v4l2-ctl --all 输出中 "Device Caps" 行有缩进、
#     能力名称在该行之后的缩进行上，原 awk 同行提取与 ^ 锚点匹配均失效；
#     改为 grep -A 3 'Device Caps' 抓取后续能力名。

set -u

OUT_ROOT="${1:-.}"
TS="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_ROOT}/stereo_probe_${TS}"
FRAMES_DIR="${OUT_DIR}/frames"
REPORT="${OUT_DIR}/report.txt"

log() { echo "$*" | tee -a "${REPORT}"; }
section() {
    log ""
    log "================================================================"
    log "== $1"
    log "================================================================"
}
# 执行命令并把输出写入报告；失败记录后继续
run() {
    log "\$ $*"
    "$@" >>"${REPORT}" 2>&1
    local rc=$?
    if [ "${rc}" -ne 0 ]; then
        log "!! 命令失败 rc=${rc}: $*（已记录，继续后续步骤）"
    fi
    return "${rc}"
}
# 判断档位是否存在于设备列表
have_res() {
    local r="$1" x
    for x in "${RES_LIST[@]:-}"; do
        [ "$x" = "$r" ] && return 0
    done
    return 1
}

mkdir -p "${FRAMES_DIR}" || { echo "无法创建输出目录: ${OUT_DIR}" >&2; exit 1; }

section "0. 环境与自检"
log "探测时间 : $(date '+%F %T')"
log "主机内核 : $(uname -a)"
if ! command -v v4l2-ctl >/dev/null 2>&1; then
    log "缺少 v4l2-ctl。请先执行: sudo apt update && sudo apt install -y v4l-utils"
    exit 1
fi
log "v4l2-ctl  : $(v4l2-ctl --version 2>&1 | head -n 1)"
run lsusb
run lsusb -t
# dmesg 可能受限（kernel.dmesg_restrict），尽力而为
if dmesg_out="$(sudo -n dmesg 2>/dev/null || dmesg 2>/dev/null)"; then
    log "\$ dmesg | grep -iE 'uvc|video|usb' | tail -n 40"
    echo "${dmesg_out}" | grep -iE 'uvc|video|usb' | tail -n 40 >>"${REPORT}"
else
    log "!! dmesg 不可读（需 root），跳过内核日志摘录"
fi

section "1. 设备枚举（P3 判据一：采集节点数）"
run v4l2-ctl --list-devices
NODES=()
for n in /dev/video*; do
    [ -e "$n" ] || continue
    # Device Caps 行有前导缩进，能力名（Video Capture 等）在其后的缩进行上
    caps="$(v4l2-ctl -d "$n" --all 2>/dev/null | grep -A 3 'Device Caps')"
    if echo "${caps}" | grep -q "Video Capture" && ! echo "${caps}" | grep -q "Multiplanar"; then
        NODES+=("$n")
        caps_oneline="$(echo "${caps}" | tr '\n' ' ' | tr -s ' ')"
        log "采集节点: $n  (${caps_oneline})"
    fi
done
if [ "${#NODES[@]}" -eq 0 ]; then
    log "!! 未发现任何具备 Video Capture 能力的节点，相机未识别？请检查 USB 连接后重跑"
    exit 1
fi
log "采集节点共 ${#NODES[@]} 个: ${NODES[*]}"
log "判读：1 个 => SBS 同帧单流；2 个 => 可能双流（以第 6 节抓帧判读为准）"

DEV="${DEV:-${NODES[0]}}"
log "探测节点 DEV=${DEV}"

section "2. 节点详细信息"
run v4l2-ctl -d "${DEV}" --all

section "3. 格式与档位（与规格书对拍）"
run v4l2-ctl -d "${DEV}" --list-formats-ext
run v4l2-ctl -d "${DEV}" --get-parm

# 提取 MJPG 档位（分辨率列表）
mapfile -t RES_LIST < <(
    v4l2-ctl -d "${DEV}" --list-formats-ext 2>/dev/null |
    awk '/^[[:space:]]*\[/{in_mjpg=($0 ~ /MJPG|Motion-JPEG/)} /Size: Discrete/ && in_mjpg {print $3}' |
    sort -u
)
if [ "${#RES_LIST[@]}" -eq 0 ]; then
    log "!! 未解析到 MJPG 档位，跳过后续实测与抓帧"
    section "7. 完成"
    log "报告目录: ${OUT_DIR}"
    exit 1
fi
log "MJPG 档位: ${RES_LIST[*]}"

section "4. MJPG 各档实测帧率（每档 300 帧）"
for res in "${RES_LIST[@]}"; do
    w="${res%x*}"; h="${res#*x}"
    log "---- ${w}x${h} ----"
    run v4l2-ctl -d "${DEV}" "--set-fmt-video=width=${w},height=${h},pixelformat=MJPG"
    run v4l2-ctl -d "${DEV}" --get-fmt-video
    log "\$ v4l2-ctl -d ${DEV} --stream-mmap=4 --stream-count=300"
    timeout 40 v4l2-ctl -d "${DEV}" --stream-mmap=4 --stream-count=300 >>"${REPORT}" 2>&1
    rc=$?
    [ "${rc}" -ne 0 ] && log "!! 档位 ${w}x${h} 实测失败 rc=${rc}"
done

section "5. buffer 时间戳来源（印证设计文档 §3.3）"
TS_RES="3840x1080"
have_res "${TS_RES}" || TS_RES="${RES_LIST[0]}"
w="${TS_RES%x*}"; h="${TS_RES#*x}"
run v4l2-ctl -d "${DEV}" "--set-fmt-video=width=${w},height=${h},pixelformat=MJPG"
log "\$ v4l2-ctl -d ${DEV} --stream-mmap=4 --stream-count=3 --verbose"
timeout 20 v4l2-ctl -d "${DEV}" --stream-mmap=4 --stream-count=3 --verbose >>"${REPORT}" 2>&1 \
    || log "!! 时间戳抓取失败"
log "判读：Timestamp 行应标注时钟来源（Monotonic 等），确认是否无可靠采集时刻"

section "6. 抓帧（SBS 判读与画质检查）"
GRAB_RES=("3840x1080" "3840x1520" "2560x720")
grabbed_any=0
for res in "${GRAB_RES[@]}"; do
    if ! have_res "${res}"; then
        log "跳过 ${res}：设备档位列表中不存在"
        continue
    fi
    w="${res%x*}"; h="${res#*x}"
    raw="${FRAMES_DIR}/${res}_raw.mjpg"
    run v4l2-ctl -d "${DEV}" "--set-fmt-video=width=${w},height=${h},pixelformat=MJPG"
    log "预热 45 帧丢弃（自动曝光收敛）: ${res}"
    timeout 20 v4l2-ctl -d "${DEV}" --stream-mmap=4 --stream-count=45 --stream-to=/dev/null >>"${REPORT}" 2>&1
    log "抓取 10 帧 -> ${raw}"
    timeout 20 v4l2-ctl -d "${DEV}" --stream-mmap=4 --stream-count=10 --stream-to="${raw}" >>"${REPORT}" 2>&1
    rc=$?
    if [ "${rc}" -ne 0 ]; then
        log "!! ${res} 抓帧失败 rc=${rc}"
        continue
    fi
    grabbed_any=1
    # 拆分 MJPG 拼接流为单帧 JPEG：优先 python3，其次 ffmpeg；失败保留拼接文件
    if command -v python3 >/dev/null 2>&1; then
        python3 - "${raw}" "${FRAMES_DIR}/${res}" >>"${REPORT}" 2>&1 <<'PY'
import sys
src, prefix = sys.argv[1], sys.argv[2]
data = open(src, 'rb').read()
frames, start = [], data.find(b'\xff\xd8')
while start != -1:
    end = data.find(b'\xff\xd9', start)
    if end == -1:
        break
    frames.append(data[start:end + 2])
    start = data.find(b'\xff\xd8', end + 2)
for i, f in enumerate(frames):
    with open(f"{prefix}_{i:02d}.jpg", 'wb') as fp:
        fp.write(f)
print(f"split {len(frames)} frames")
PY
        rc=$?
        if [ "${rc}" -eq 0 ]; then
            log "python3 拆分完成: ${FRAMES_DIR}/${res}_NN.jpg"
            rm -f "${raw}"
        else
            log "!! python3 拆分失败 rc=${rc}，保留拼接文件 ${raw}"
        fi
    elif command -v ffmpeg >/dev/null 2>&1; then
        log "\$ ffmpeg -i ${raw} ${FRAMES_DIR}/${res}_%02d.jpg"
        ffmpeg -hide_banner -loglevel error -y -i "${raw}" "${FRAMES_DIR}/${res}_%02d.jpg" >>"${REPORT}" 2>&1
        rc=$?
        if [ "${rc}" -eq 0 ]; then
            log "ffmpeg 拆分完成: ${FRAMES_DIR}/${res}_%02d.jpg"
            rm -f "${raw}"
        else
            log "!! ffmpeg 拆分失败 rc=${rc}，保留拼接文件 ${raw}"
        fi
    else
        log "!! 无 python3/ffmpeg，保留拼接文件 ${raw}（可在 Windows 端用 ffmpeg 拆分）"
    fi
done
[ "${grabbed_any}" -eq 0 ] && log "!! 未抓到任何帧"

section "7. 完成"
log "报告与抓帧目录: ${OUT_DIR}"
log "请将整个目录拷回 Windows（report.txt + frames/*.jpg）交给分析会话"