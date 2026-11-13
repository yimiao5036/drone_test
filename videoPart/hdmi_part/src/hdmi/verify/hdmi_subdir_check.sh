#!/usr/bin/env bash
# 验证「主工程 add_subdirectory 接入」这条链路（在 WSL2 Ubuntu 中执行）
#
# 用一个临时工程模拟主工程，因此**不需要改动仓库根 CMakeLists.txt**。
#
# 覆盖点：
#   1. 仓库根自动定位：configure 日志应出现 "-- HDMI 直显模块: 仓库根=..."
#   2. 不出现 "未找到 drone_video_transmission 目标" 告警
#   3. drone_hdmi_display 能挂到既有的 drone_video_transmission 目标上
#   4. PUBLIC 传播：下游只链 drone_video_transmission，仍能
#      #include "hdmi/hdmi_display_backend.h" 并调用工厂
#
# 用法: bash videoPart/hdmi_part/src/hdmi/verify/hdmi_subdir_check.sh

set -u

PROJ=/mnt/d/ProgramData/drone_test/drone_test
MOD="$PROJ/videoPart/hdmi_part/src/hdmi"
WORK="$HOME/drone_hdmi_subdir"
LOG="$WORK/check.log"

rm -rf "$WORK"
mkdir -p "$WORK/harness"

# ---- 模拟主工程：先定义既有的 drone_video_transmission，再 add_subdirectory ----
cat > "$WORK/harness/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.16)
project(hdmi_subdir_harness LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 模拟主工程里既有的图传库（真实工程中即 drone_video_transmission）
add_library(drone_video_transmission STATIC dummy.cpp)

add_subdirectory("${HDMI_MODULE_DIR}" hdmi)

# 下游消费者：只链 drone_video_transmission，验证 PUBLIC include / link 传播
add_executable(downstream_include_check downstream.cpp)
target_link_libraries(downstream_include_check PRIVATE drone_video_transmission)
EOF

cat > "$WORK/harness/dummy.cpp" <<'EOF'
int drone_video_transmission_dummy() { return 0; }
EOF

cat > "$WORK/harness/downstream.cpp" <<'EOF'
// 模拟装配处：只链 drone_video_transmission，靠 PUBLIC 传播拿到头文件与库
#include <cstdio>
#include "hdmi/hdmi_display_backend.h"

int main() {
    drone::video_transmission::HdmiDisplayConfig cfg;
    cfg.width = 1280;
    cfg.height = 720;
    cfg.refresh_hz = 30;
    auto factory = drone::video_transmission::HdmiMakeBackendFactory(cfg);
    std::printf("下游注入成功: factory valid=%d, 配置 %ux%u@%dHz\n",
                factory ? 1 : 0, cfg.width, cfg.height, cfg.refresh_hz);
    return factory ? 0 : 1;
}
EOF

# ---- configure + build ----
{
    echo "########## CONFIGURE ##########"
    cmake -S "$WORK/harness" -B "$WORK/build" \
          -DHDMI_MODULE_DIR="$MOD" -DCMAKE_BUILD_TYPE=Release
    echo "########## BUILD ##########"
    cmake --build "$WORK/build" -j"$(nproc)"
} >"$LOG" 2>&1
rc=$?

FATAL=0
echo "=== add_subdirectory 接入验证 ==="

root=$(grep -oE 'HDMI 直显模块: 仓库根=.*' "$LOG" | head -1)
if [ -n "$root" ]; then
    echo "  仓库根定位   : $root"
else
    echo "  ✗ 未见仓库根定位输出"; FATAL=1
fi

if grep -q '未找到 drone_video_transmission 目标' "$LOG"; then
    echo "  ✗ 出现「未找到 drone_video_transmission 目标」告警"; FATAL=1
else
    echo "  链接既有目标 : OK（无告警）"
fi

errs=$(grep -c 'error:' "$LOG" || true)
warns=$(grep -c 'warning:' "$LOG" || true)
printf '  编译         : %s (error=%s warning=%s)\n' \
       "$([ "$rc" -eq 0 ] && echo 通过 || echo "失败 rc=$rc")" "$errs" "$warns"
[ "$rc" -eq 0 ] || FATAL=1
[ "$warns" -eq 0 ] || FATAL=1

if [ -x "$WORK/build/downstream_include_check" ]; then
    "$WORK/build/downstream_include_check"
    drc=$?
    printf '  下游 include : %s\n' "$([ "$drc" -eq 0 ] && echo 通过 || echo "失败 rc=$drc")"
    [ "$drc" -eq 0 ] || FATAL=1
else
    echo "  ✗ 下游可执行未生成"; FATAL=1
fi

echo "  日志: $LOG"
echo "=============================================================="
if [ "$FATAL" -eq 0 ]; then
    echo " add_subdirectory 接入验证通过"
else
    echo " 存在失败项，详见上方输出与日志"
fi
exit "$FATAL"
