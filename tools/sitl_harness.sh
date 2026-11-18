#!/usr/bin/env bash
# -*- coding: utf-8 -*-
# ============================================================================
# SITL 验收装置（PX4 SITL + PTY 桥 + GCS 心跳 + 生产程序 / px4_link_smoke）
#
# 用途：在 WSL2 Ubuntu 24.04 上一键起停 SITL 验收环境，供
#       tools/assignment_sitl.py（65014/65010 任务分配）与
#       build/px4_link_smoke（G1~G3/S11~S14 场景）使用。
#
# 用法：
#   tools/sitl_harness.sh stack [--config <cfg>]   起 PX4+桥+GCS+drone_control 并等 READY
#   tools/sitl_harness.sh app   [--config <cfg>]   仅重启 drone_control（PX4/桥/GCS 保持）
#   tools/sitl_harness.sh smoke --config <cfg> <px4_link_smoke 参数...>
#                                                  起 PX4+GCS 后运行冒烟/SITL 场景
#   tools/sitl_harness.sh webgcs [--port 5761]     把 /tmp/gcs_test 桥到 TCP，交给 Web 地面站
#                                                  （PX4 侧 14550 心跳由 sitl_gcs.py 继续提供，
#                                                    没在跑会自动补启；两条链路互不重叠）
#   tools/sitl_harness.sh stop                     停止全部（tmux 会话与进程）
#   tools/sitl_harness.sh status                   打印会话/进程/最新日志路径
#
# 环境变量：
#   PX4_DIR  PX4-Autopilot 源码目录（默认 $HOME/PX4-Autopilot）
#   REPO     工程根目录（默认本脚本上级目录，一般无需设置）
#   LOG_DIR  运行日志目录（默认 /tmp/sitl_harness）
#   SITL_GUI 1=启动 Gazebo 图形界面（需 WSLg/显示器，默认 0 无界面）
#   PX4_GZ_MODEL  gz 机体名，默认 x500（与改动前完全一致）。实际执行
#                 `make px4_sitl gz_${PX4_GZ_MODEL}`，可用值 = PX4 源码
#                 ROMFS/px4fmu_common/init.d-posix/airframes/ 里 `*_gz_<名>` 的后半段
#                 （如 x500_lidar_down / x500_lidar_2d / x500_lidar_front）
#   SITL_LIDAR    1=下视激光雷达机体，简写形式，等价 `PX4_GZ_MODEL=x500_lidar_down`；
#                 两者同时给出时以 PX4_GZ_MODEL 为准。雷达机体下自动设置
#                 SYS_HAS_NUM_DIST / EKF2_RNG_CTRL / EKF2_RNG_A_HMAX（见 set_lidar_params；
#                 故意不动 EKF2_HGT_REF，原因见该函数注释）。默认 0，行为与改动前一致
#
# 安全边界：本脚本只面向 WSL2 内的 PX4 SITL（默认 gz_x500；SITL_LIDAR=1 时
#          gz_x500_lidar_down —— 都是仿真模型名，不是真机型号），
#          不连接任何真实飞控/串口；生产 `enable_control=false` 不受影响，
#          SITL 配置 config/px4_sitl_assignment.json 显式 `enable_control=true`。
# ============================================================================
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${REPO:-$(cd "$SCRIPT_DIR/.." && pwd)}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
LOG_DIR="${LOG_DIR:-/tmp/sitl_harness}"
# SITL_GUI=1 时去掉 HEADLESS=1，让 Gazebo 拉起图形界面（WSLg 下可直接看到 3D）
SITL_HEADLESS_ENV="HEADLESS=1"
if [ "${SITL_GUI:-0}" = "1" ]; then SITL_HEADLESS_ENV=""; fi

# gz 机体（2026-11-18 P5 雷达接入）：默认 x500 → 与改动前逐字节一致。
# 机体与 airframe 的对应由 PX4 在建时反查文件名完成，本脚本只负责给出目标名：
#   make 目标 `gz_<模型>`（PX4 源码 src/modules/simulation/gz_bridge/CMakeLists.txt:97~110
#   对 airframes/*_gz_* 做 `.*_gz_` 前缀剥离得到），px4 启动时再按
#   `airframes/[0-9]+_gz_<模型>` 反查 SYS_AUTOSTART（ROMFS/.../rcS:56），
#   最后 `MODEL_NAME=${PX4_SIM_MODEL#*gz_}` 交给 gz spawn（px4-rc.gzsim:108~110）。
# 说明：PX4 1.15 起 `PX4_GZ_MODEL` 环境变量已废弃、并入 `PX4_SIM_MODEL`
# （docs/en/sim_gazebo_gz/index.md:267），PX4 侧不再读它，故这里可安全地把它
# 当作本装置自己的变量名。
if [ "${SITL_LIDAR:-0}" = "1" ] && [ -z "${PX4_GZ_MODEL:-}" ]; then
    PX4_GZ_MODEL="x500_lidar_down"
fi
PX4_GZ_MODEL="${PX4_GZ_MODEL:-x500}"
# 是否雷达机体：模型名里带 lidar 才设置雷达参数（默认 x500 走原路径，不受影响）。
SITL_LIDAR_MODEL=0
case "$PX4_GZ_MODEL" in
    *lidar*) SITL_LIDAR_MODEL=1 ;;
esac

APP_BIN="$REPO/build/drone_control"
SMOKE_BIN="$REPO/build/px4_link_smoke"
BRIDGE_PY="$REPO/tools/sitl_pty_bridge.py"
GCS_PY="$REPO/tools/sitl_gcs.py"
WEB_GCS_PY="$REPO/tools/sitl_web_gcs.py"
DEFAULT_APP_CONFIG="$REPO/config/px4_sitl_assignment.json"

PX4_LOG="$LOG_DIR/px4.log"
GCS_LOG="$LOG_DIR/gcs.log"
BRIDGE_LOG="$LOG_DIR/bridge.log"
APP_LOG_TEE="$LOG_DIR/app.log"
APP_LOG_PATH_FILE="$LOG_DIR/app_log_path"

die() { echo "[harness][ERROR] $*" >&2; exit 1; }
info() { echo "[harness] $*"; }

require_file() { [ -f "$1" ] || die "缺少文件: $1"; }
require_dir() { [ -d "$1" ] || die "缺少目录: $1"; }

# 进程清理：用括号转义模式，避免 pkill 匹配到本脚本自身命令行。
kill_processes() {
    pkill -f "[p]x4_sitl_default" 2>/dev/null
    pkill -f "[g]z sim" 2>/dev/null
    pkill -f "[s]itl_pty_bridge" 2>/dev/null
    pkill -f "^python3 .*sitl_gcs\.py" 2>/dev/null
    pkill -f "^python3 .*sitl_web_gcs\.py" 2>/dev/null
}

kill_sessions() {
    tmux kill-session -t app 2>/dev/null
    tmux kill-session -t gcs 2>/dev/null
    tmux kill-session -t px4 2>/dev/null
    tmux kill-session -t bridge 2>/dev/null
    tmux kill-session -t webgcs 2>/dev/null
}

stop_all() {
    kill_sessions
    kill_processes
    sleep 2
    info "已停止 SITL 会话与进程"
}

start_bridge() {
    require_file "$BRIDGE_PY"
    tmux new-session -d -s bridge "python3 $BRIDGE_PY 2>&1 | tee $BRIDGE_LOG"
    sleep 1
    info "PTY 桥已启动（/tmp/gcs_app 给机载、/tmp/gcs_test 给验收工具）"
}

start_px4() {
    require_dir "$PX4_DIR"
    require_file "$PX4_DIR/build/px4_sitl_default/bin/px4"
    # 机体存在性预检：名字写错时给明确报错，而不是让 make 抛 "No rule to make target"。
    # 判据与 PX4 的 airframe 反查同名（rcS:56），所以这里过了就一定能起。
    ls "$PX4_DIR"/ROMFS/px4fmu_common/init.d-posix/airframes/*_gz_"$PX4_GZ_MODEL" >/dev/null 2>&1 \
        || die "PX4 里没有机体 gz_${PX4_GZ_MODEL}：$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/ 下找不到 *_gz_${PX4_GZ_MODEL}"
    tmux new-session -d -s px4 \
        "cd $PX4_DIR && $SITL_HEADLESS_ENV PX4_GZ_MODEL_POSE=0,0,0,0,0,1.5708 make px4_sitl gz_${PX4_GZ_MODEL} 2>&1 | tee $PX4_LOG"
    info "PX4 SITL 启动中（机体 gz_${PX4_GZ_MODEL}，朝北出生，等待 home set）..."
    local i
    for i in $(seq 1 150); do
        grep -q "home set" "$PX4_LOG" 2>/dev/null && break
        sleep 1
    done
    grep -q "home set" "$PX4_LOG" 2>/dev/null || { tail -25 "$PX4_LOG"; die "PX4 SITL 启动失败"; }
    info "PX4 SITL 就绪"
}

px4_param() {
    tmux send-keys -t px4 "param set $1 $2" Enter
}

set_common_params() {
    px4_param COM_LOW_BAT_ACT 0
    px4_param BAT_LOW_THR 0.15
    px4_param SIM_BAT_MIN_PCT 50
    px4_param SIM_BAT_DRAIN 0
    sleep 1
}

# 雷达相关 PX4 参数（P5 雷达接入，2026-11-18）。只在雷达机体上生效。
#
# 参数核对（2026-11-18 在本机 /root/PX4-Autopilot 源码逐条 grep 确认，
# commit d6f12ad；不是凭记忆写的。下面「文件:行号」都指该源码树）：
#   参数                 默认   取值范围(元数据)   定义位置
#   SYS_HAS_NUM_DIST     0      0~4                src/lib/systemlib/system_params.c:241
#   EKF2_RNG_CTRL        1      枚举 0/1/2         src/modules/ekf2/params_range_finder.yaml:5
#   EKF2_HGT_REF         1      枚举 0/1/2/3       src/modules/ekf2/module.yaml:87
#   EKF2_RNG_A_HMAX      5.0    1.0~10.0 m         src/modules/ekf2/params_range_finder.yaml:75
# 枚举含义（同上 yaml/module.yaml）：
#   EKF2_RNG_CTRL 0=Disable 1=Enabled(条件模式) 2=Enabled(强制/始终融合)
#   EKF2_HGT_REF  0=Baro 1=GPS 2=Range sensor 3=Vision
#
# 取值口径（每条都写了为什么，改动前先读）：
# - SYS_HAS_NUM_DIST=1：commander 的测距检查在参数 `<=0` 时**直接 return**
#   （src/modules/commander/HealthAndArmingChecks/checks/distanceSensorChecks.cpp:40~42），
#   不设它 PX4 既不会报 "Preflight Fail: Distance Sensor missing"，也不会在 ARM 时
#   因缺测距数据拒飞；设 1 让"雷达数据没到"表现为明确的解锁拒绝，这本身就是
#   雷达链路的一条验收信号。
# - EKF2_RNG_CTRL=1（条件融合），**绝不用 2**：2 是始终融合，设计稿明确"先别上 2"。
#   本版默认值已是 1，显式写出是为了钉住口径（防止机体/参数文件把人带偏）。
# - EKF2_HGT_REF **故意不设**：设计意图是"雷达不作高度主基准"，而 PX4 默认值 1 已是
#   GPS（0=Baro、1=GPS、2=Range sensor、3=Vision），本就不含雷达。显式写 0 或 1 都会
#   覆盖现场基线（基线若是 Baro 会被这行悄悄改掉），所以只在这里钉住"必须是 0/1/3，
#   **绝不能是 2**"。
# - EKF2_RNG_A_HMAX=10：设计稿写 12，但 PX4 元数据 max=10.0（params_range_finder.yaml:82~84）。
#   `param set` 不做范围校验（src/systemcmds/param/param.cpp:762~828 直接把值写进去），
#   写 12 会被存成 12 但属于元数据外的值（地面站会标红、后续 PX4 版本可能拒绝）。
#   SITL 低空验收（<5 m）下 10 与 12 无差别，故取元数据上限 10。
#
# 生效方式：`param set` → PX4 发 parameter_update → EKF2 在 Run() 里 updateParams()
# 重新读参数（src/modules/ekf2/EKF2.cpp:445~451），**无需重启 PX4**。
# EKF2_HGT_REF 元数据标了 reboot_required，但代码路径是每周期读
# `_params.ekf2_hgt_ref`（src/modules/ekf2/EKF/height_control.cpp:70），故同样即时生效。
# 相邻但**故意没设**的参数（说明用）：
#   EKF2_RNG_A_VMAX 默认 1.0 m/s —— 条件模式下平飞超过这个水平速度也不融合，
#   跑水平机动场景时雷达会"时断时续"，属预期，不要当成链路故障。
#   SIM_GZ_EN_LIDAR 默认 1（src/modules/simulation/gz_bridge/parameters.c:52），
#   gz_bridge 默认就在发布 distance_sensor，无需设置。
set_lidar_params() {
    # 非雷达机体直接跳过：默认路径（gz_x500）行为与改动前完全一致。
    [ "$SITL_LIDAR_MODEL" = "1" ] || return 0
    info "雷达机体：设置 PX4 雷达参数（SYS_HAS_NUM_DIST=1 / EKF2_RNG_CTRL=1 / EKF2_RNG_A_HMAX=10）"
    px4_param SYS_HAS_NUM_DIST 1
    px4_param EKF2_RNG_CTRL 1
    px4_param EKF2_RNG_A_HMAX 10
    sleep 1
}

start_gcs() {
    require_file "$GCS_PY"
    tmux new-session -d -s gcs "python3 $GCS_PY 2>&1 | tee $GCS_LOG"
    sleep 2
    info "GCS 心跳（255/190，UDP 14550）已启动"
}

# 等待 drone_control 进入 READY，打印日志路径与 Home。
wait_app_ready() {
    require_file "$APP_BIN"
    local marker="$LOG_DIR/app_start_marker" app_log=""
    touch "$marker"
    local i
    for i in $(seq 1 90); do
        app_log=$(find "$REPO/build/logs" -maxdepth 1 -name '*.log' -newer "$marker" 2>/dev/null | head -1)
        if [ -n "$app_log" ] && grep -q "SELF_CHECK -> READY" "$app_log" 2>/dev/null; then
            break
        fi
        sleep 1
    done
    [ -n "$app_log" ] && grep -q "SELF_CHECK -> READY" "$app_log" 2>/dev/null \
        || { tail -20 "$APP_LOG_TEE" 2>/dev/null; die "drone_control 未进入 READY"; }
    echo "$app_log" > "$APP_LOG_PATH_FILE"
    info "drone_control 就绪: $app_log"
    grep "HOME lat" "$GCS_LOG" 2>/dev/null | tail -1
}

start_app() {
    local config="$1"
    require_file "$config"
    require_file "$APP_BIN"
    tmux kill-session -t app 2>/dev/null
    sleep 1
    tmux new-session -d -s app "cd $REPO/build && ./drone_control --config $config 2>&1 | tee $APP_LOG_TEE"
    wait_app_ready
}

parse_config() {
    local cfg="$DEFAULT_APP_CONFIG"
    while [ $# -gt 0 ]; do
        case "$1" in
            --config) cfg="${2:-}"; shift 2 ;;
            *) break ;;
        esac
    done
    [ -n "$cfg" ] || die "--config 缺少参数"
    case "$cfg" in
        /*) echo "$cfg" ;;
        *) echo "$REPO/$cfg" ;;
    esac
}

cmd_stack() {
    local cfg; cfg="$(parse_config "$@")"
    mkdir -p "$LOG_DIR"
    stop_all
    # 清掉上一次测试/实机遗留的围栏缓存：测试用例会把泸州测试区写进
    # build/config/zone.json，SITL 的 Home 在苏黎世，残留多边形会让
    # 状态机开机即判"距边界=-8342817m"并立刻中止。
    if [ -f "$REPO/build/config/zone.json" ]; then
        mv "$REPO/build/config/zone.json" "$LOG_DIR/zone.json.bak-$(date +%s)"
        info "已把 build/config/zone.json 备份移出（避免与 SITL Home 冲突）"
    fi
    start_bridge
    start_px4
    set_lidar_params
    set_common_params
    # stack 是人工/SITL 验收场景：水平速度上限与 controller.max_speed_mps=10
    # 同源，避免上一轮 smoke 持久化的 MPC_XY_VEL_MAX=2 把速度律压成 2 m/s。
    px4_param MPC_XY_VEL_MAX 10.0
    start_gcs
    start_app "$cfg"
    info "环境就绪。验收工具示例："
    info "  python3 tools/assignment_sitl.py --connection /tmp/gcs_test \\"
    info "    --aircraft-system 1 --aircraft-component 25 --assignment-id 1 \\"
    info "    --target-id 7 --interceptor-type 25 --send-target \\"
    info "    --target-latitude <Home+20m> --target-longitude <Home> --target-altitude-m 2.0 \\"
    info "    --stop-target-after-ms 20000 --duration 75"
    info "日志：$LOG_DIR（app.log/px4.log/gcs.log/bridge.log；机载日志见上方路径）"
}

cmd_app() {
    local cfg; cfg="$(parse_config "$@")"
    mkdir -p "$LOG_DIR"
    start_app "$cfg"
}

cmd_smoke() {
    local cfg=""
    if [ "${1:-}" = "--config" ]; then
        cfg="$2"; shift 2
    fi
    [ -n "$cfg" ] || die "smoke 需要 --config <cfg>"
    case "$cfg" in /*) ;; *) cfg="$REPO/$cfg" ;; esac
    require_file "$cfg"
    require_file "$SMOKE_BIN"
    mkdir -p "$LOG_DIR"
    stop_all
    start_px4
    set_lidar_params
    px4_param MPC_XY_VEL_MAX 2.0
    set_common_params
    start_gcs
    local smoke_log="$LOG_DIR/smoke_$(date +%Y%m%d_%H%M%S).log"
    info "运行 px4_link_smoke --config $cfg $* （日志 $smoke_log）"
    ( cd "$REPO/build" && ./px4_link_smoke --config "$cfg" "$@" ) 2>&1 | tee "$smoke_log"
    local rc=${PIPESTATUS[0]}
    info "px4_link_smoke 退出码=$rc；PX4/GCS 仍在运行，可继续场景或执行 stop"
    return "$rc"
}

# 保证 PX4 侧存在 255/190 的 MAV_TYPE_GCS 心跳（UDP 14550）。
# 没有它，commander 会以 `Preflight Fail: No connection to the GCS` 拒绝解锁；
# 这与机载地面站链路是两条独立通道，Web 接入时同样需要保留。
# 只认真正的 python 进程：`pgrep -f "sitl_gcs"` 会命中调用方 shell 自己的命令行
# （例如诊断命令里带了该文件名），造成“明明没跑却判为在跑”的假阳性。
px4_gcs_pids() {
    pgrep -f "^python3 .*sitl_gcs\.py" 2>/dev/null
}

ensure_px4_gcs() {
    if [ -n "$(px4_gcs_pids)" ]; then
        return 0
    fi
    require_file "$GCS_PY"
    tmux kill-session -t gcs 2>/dev/null
    tmux new-session -d -s gcs "python3 $GCS_PY 2>&1 | tee $GCS_LOG"
    local i
    for i in $(seq 1 20); do
        [ -n "$(px4_gcs_pids)" ] && { info "PX4 侧 GCS 心跳已启动（sitl_gcs.py → UDP 14550）"; return 0; }
        sleep 0.5
    done
    return 1
}

cmd_webgcs() {
    local port=5761
    while [ $# -gt 0 ]; do
        case "$1" in
            --port) port="${2:-}"; shift 2 ;;
            *) die "webgcs 参数非法: $1（可用: --port <端口>）" ;;
        esac
    done
    require_file "$WEB_GCS_PY"
    [ -e /tmp/gcs_test ] || die "/tmp/gcs_test 不存在（先执行 stack）"
    # PX4 的解锁前置检查的是它自己那条 MAVLink（SITL 为 UDP 14550）上是否收到
    # MAV_TYPE_GCS 心跳；Web 地面站的心跳走 /tmp/gcs_app→/tmp/gcs_test 的机载链路，
    # 到不了 PX4。因此 webgcs 不能停 sitl_gcs.py，反而要保证它在跑：
    # 两条链路互不重叠（14550 vs PTY），同时存在不冲突。
    ensure_px4_gcs || die "PX4 侧 GCS 心跳未能启动（见 $GCS_LOG）"
    tmux kill-session -t webgcs 2>/dev/null
    tmux new-session -d -s webgcs "python3 $WEB_GCS_PY --port $port 2>&1 | tee $LOG_DIR/webgcs.log"
    sleep 1
    local host_ip
    host_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
    [ -n "$host_ip" ] || host_ip="127.0.0.1"
    info "Web 地面站桥已启动：/tmp/gcs_test <-> TCP $port"
    info "PX4 侧 GCS 心跳保持运行（sitl_gcs.py → UDP 14550），解锁前置已满足"
    info "Web 端「连接设备」选 TCP 客户端："
    info "  地址 127.0.0.1（WSL2 本地转发不通时用 $host_ip）  端口 $port"
    info "  机载身份 1/25，地面站必须是 255/190；连上后应看到 65011/65012/65013 回传"
    info "日志：$LOG_DIR/webgcs.log"
}

cmd_status() {
    echo "== tmux 会话 =="; tmux ls 2>&1 | head -10
    echo "== 进程 =="; pgrep -af "[p]x4_sitl|[g]z sim|[s]itl_pty_bridge|[s]itl_gcs|[d]rone_control" | head -10
    echo "== 日志 =="; ls -l "$LOG_DIR" 2>/dev/null | head -10
    [ -f "$APP_LOG_PATH_FILE" ] && echo "最新机载日志: $(cat "$APP_LOG_PATH_FILE")"
}

case "${1:-}" in
    stack)  shift; cmd_stack "$@" ;;
    app)    shift; cmd_app "$@" ;;
    smoke)  shift; cmd_smoke "$@" ;;
    webgcs) shift; cmd_webgcs "$@" ;;
    stop)   stop_all ;;
    status) cmd_status ;;
    ""|-h|--help|help)
        sed -n '2,38p' "$0" | sed 's/^# \{0,1\}//'
        ;;
    *) die "未知命令: $1（可用: stack/app/smoke/stop/status）" ;;
esac
