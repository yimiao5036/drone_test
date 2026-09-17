#!/usr/bin/env bash
# HDMI 直显后端完整验证（在 WSL2 Ubuntu 中执行）
#
# 用法:
#   bash videoPart/hdmi_part/src/hdmi/verify/hdmi_build_check.sh          # 依次验证 KMS=ON / KMS=OFF
#   bash videoPart/hdmi_part/src/hdmi/verify/hdmi_build_check.sh on       # 只验证 KMS=ON
#   bash videoPart/hdmi_part/src/hdmi/verify/hdmi_build_check.sh off      # 只验证 KMS=OFF
#
# 构建目录放在 WSL 原生文件系统（~/drone_hdmi_build），不写入 Windows 侧仓库。
# 注意：本脚本必须在文件里执行，不要用 wsl.exe 内联命令传 $变量 —— 会被外层 shell 吞掉。

set -u

PROJ=/mnt/d/ProgramData/drone_test/drone_test
SRC="$PROJ/videoPart/hdmi_part/src/hdmi"
ROOT="$HOME/drone_hdmi_build"

FATAL=0

# ---------------------------------------------------------------------------
# 单轮验证：配置 → 编译 → 单测 → 探针退出码
# ---------------------------------------------------------------------------
verify() {
    local mode="$1" kms="$2"
    local build="$ROOT/$mode"
    local log="$ROOT/build_$mode.log"

    rm -rf "$build"
    mkdir -p "$build"

    echo "=============================================================="
    echo " 验证配置: DRONE_HAVE_HDMI_KMS=$kms   (构建目录 $build)"
    echo "=============================================================="

    {
        echo "########## CONFIGURE ##########"
        cmake -S "$SRC" -B "$build" -DDRONE_HAVE_HDMI_KMS="$kms" \
              -DCMAKE_BUILD_TYPE=Release
        echo "########## BUILD ##########"
        cmake --build "$build" -j"$(nproc)"
    } >"$log" 2>&1
    local build_rc=$?

    local errors warnings
    errors=$(grep -c 'error:' "$log" || true)
    warnings=$(grep -c 'warning:' "$log" || true)
    printf '  编译         : %s  (error=%s warning=%s)\n' \
           "$([ "$build_rc" -eq 0 ] && echo 通过 || echo "失败 rc=$build_rc")" \
           "$errors" "$warnings"

    if [ "$build_rc" -ne 0 ]; then
        echo '  --- 编译错误（本模块） ---'
        grep -E 'error:' "$log" | grep 'hdmi_' | head -20
        echo "  --- 完整日志: $log ---"
        FATAL=1
        return
    fi
    if [ "$warnings" -ne 0 ]; then
        echo '  --- 编译告警（本模块） ---'
        grep -E 'warning:' "$log" | grep 'hdmi_' | head -20
    fi

    # ---- 单元测试 ----
    if [ -x "$build/hdmi_display_backend_selftest" ]; then
        if ctest --test-dir "$build" --output-on-failure >>"$log" 2>&1; then
            local passed
            passed=$(grep -E '[0-9]+% tests passed' "$log" | tail -1)
            printf '  单元测试     : 通过  (%s)\n' "${passed:-详见日志}"
        else
            printf '  单元测试     : 失败\n'
            echo '  --- ctest 汇总 ---'
            sed -n '/The following tests FAILED/,$p' "$log" | head -20
            grep -E '^\[  FAILED  \]|Failure$|SEGFAULT' "$log" | sort -u | head -20
            FATAL=1
        fi
    else
        printf '  单元测试     : 未构建（未找到 GTest？）\n'
    fi

    # ---- 探针退出码 ----
    if [ -x "$build/hdmi_probe" ]; then
        local rc_modes rc_test rc_usage rc_badopt rc_badparam
        "$build/hdmi_probe" modes        >/dev/null 2>&1; rc_modes=$?
        "$build/hdmi_probe" --help       >/dev/null 2>&1; rc_usage=$?
        "$build/hdmi_probe" test --width 0 >/dev/null 2>&1; rc_badparam=$?
        "$build/hdmi_probe" bogus        >/dev/null 2>&1; rc_badopt=$?
        if [ -e /dev/dri/card0 ]; then
            "$build/hdmi_probe" test     >/dev/null 2>&1; rc_test=$?
        else
            rc_test="N/A(无/dev/dri)"
        fi
        printf '  探针退出码   : modes=%s test=%s help=%s 非法子命令=%s 非法参数=%s\n' \
               "$rc_modes" "$rc_test" "$rc_usage" "$rc_badopt" "$rc_badparam"

        # 断言：帮助=0；非法子命令/非法参数=非 0；无设备时 modes 必须非 0
        [ "$rc_usage" -eq 0 ]      || { echo '    ✗ --help 应返回 0'; FATAL=1; }
        [ "$rc_badopt" -ne 0 ]     || { echo '    ✗ 非法子命令应返回非 0'; FATAL=1; }
        [ "$rc_badparam" -ne 0 ]   || { echo '    ✗ 非法参数应返回非 0'; FATAL=1; }
        if [ ! -e /dev/dri/card0 ]; then
            [ "$rc_modes" -ne 0 ] || { echo '    ✗ 无 DRM 设备时 modes 应返回非 0'; FATAL=1; }
        fi
    fi
    echo "  日志: $log"
    echo
}

case "${1:-all}" in
    on)  verify on ON ;;
    off) verify off OFF ;;
    all) verify on ON; verify off OFF ;;
    *)   echo "用法: $0 [on|off|all]"; exit 2 ;;
esac

echo "=============================================================="
if [ "$FATAL" -eq 0 ]; then
    echo " 全部验证通过"
else
    echo " 存在失败项，详见上方输出与日志"
fi
echo "=============================================================="
exit "$FATAL"
