#!/bin/bash
# 干净重启 PX4 SITL + 地面站心跳 + drone_control（SITL 任务分配验收用）。
#
# 环境前置（见 docs/PX4实链验证清单.md §7.4）：
#   - WSL2 Ubuntu 24.04，PX4 在 $HOME/PX4-Autopilot（v1.17.0）；
#   - 本仓库构建副本在 $HOME/drone_test（build/drone_control 已编译）；
#   - 首次运行前启动 PTY 桥：python3 tools/sitl_pty_bridge.py（保持运行）。
#
# 用法：bash tools/sitl_restart.sh [config 相对 build 目录的路径]
# 默认：config/px4_sitl_assignment.json
set -u
CONFIG="${1:-config/px4_sitl_assignment.json}"
PROJECT_DIR="${PROJECT_DIR:-$HOME/drone_test}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

tmux kill-session -t app 2>/dev/null
tmux kill-session -t gcs 2>/dev/null
tmux kill-session -t px4 2>/dev/null
pkill -f px4_sitl_default 2>/dev/null
pkill -f "gz[ ]sim" 2>/dev/null
sleep 2

rm -f /tmp/px4_sitl.log /tmp/gcs.log /tmp/app_run.log
tmux new-session -d -s px4 "cd $PX4_DIR && HEADLESS=1 PX4_GZ_MODEL_POSE=0,0,0,0,0,1.5708 make px4_sitl gz_x500 2>&1 | tee /tmp/px4_sitl.log"
for _ in $(seq 1 120); do
    grep -q "home set" /tmp/px4_sitl.log 2>/dev/null && break
    sleep 1
done
if ! grep -q "home set" /tmp/px4_sitl.log 2>/dev/null; then
    echo "PX4_START_FAILED"
    tail -20 /tmp/px4_sitl.log
    exit 1
fi
echo "PX4_READY"

# SITL 正常场景参数（低电专项在启动后再覆盖这些值并重启应用）。
tmux send-keys -t px4 "param set COM_LOW_BAT_ACT 0" Enter
tmux send-keys -t px4 "param set BAT_LOW_THR 0.15" Enter
tmux send-keys -t px4 "param set SIM_BAT_MIN_PCT 50" Enter
tmux send-keys -t px4 "param set SIM_BAT_DRAIN 0" Enter
sleep 1

tmux new-session -d -s gcs "python3 $PROJECT_DIR/tools/sitl_gcs.py 2>&1 | tee /tmp/gcs.log"
sleep 1
touch /tmp/app_start_marker
tmux new-session -d -s app "cd $PROJECT_DIR/build && ./drone_control --config $CONFIG 2>&1 | tee /tmp/app_run.log"

APP_LOG=""
for _ in $(seq 1 60); do
    APP_LOG=$(find "$PROJECT_DIR/build/logs" -maxdepth 1 -name '*.log' -newer /tmp/app_start_marker 2>/dev/null | head -1)
    if [ -n "$APP_LOG" ] && grep -q "SELF_CHECK -> READY" "$APP_LOG" 2>/dev/null; then
        break
    fi
    sleep 1
done
if [ -z "$APP_LOG" ] || ! grep -q "SELF_CHECK -> READY" "$APP_LOG" 2>/dev/null; then
    echo "APP_NOT_READY"
    tail -20 "$APP_LOG" 2>/dev/null
    exit 2
fi
echo "APP_READY log=$APP_LOG"
