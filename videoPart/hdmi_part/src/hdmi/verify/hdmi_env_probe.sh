#!/usr/bin/env bash
# HDMI 后端开发机环境探测（在 WSL2 Ubuntu 中执行）
# 用法: wsl.exe -d Ubuntu -- bash /mnt/d/.../videoPart/hdmi_part/src/hdmi/verify/hdmi_env_probe.sh

set -u

echo "=== 系统 ==="
cat /etc/os-release | grep -E '^(PRETTY_NAME|VERSION_ID)='
echo "arch: $(uname -m)"
echo "kernel: $(uname -r)"

echo
echo "=== 编译器 / 构建 ==="
printf 'gcc  : %s\n' "$(g++ --version 2>/dev/null | head -1 || echo MISSING)"
printf 'cmake: %s\n' "$(cmake --version 2>/dev/null | head -1 || echo MISSING)"
printf 'make : %s\n' "$(make --version 2>/dev/null | head -1 || echo MISSING)"

echo
echo "=== 图形依赖头文件 ==="
for h in libdrm/drm.h libdrm/drm_mode.h gbm.h EGL/egl.h EGL/eglext.h GLES3/gl3.h; do
    if [ -f "/usr/include/$h" ]; then
        printf '  %-22s OK\n' "$h"
    else
        printf '  %-22s MISSING\n' "$h"
    fi
done

echo
echo "=== pkg-config ==="
if command -v pkg-config >/dev/null 2>&1; then
    for p in libdrm gbm egl glesv2; do
        if pkg-config --exists "$p" 2>/dev/null; then
            printf '  %-8s %s\n' "$p" "$(pkg-config --modversion "$p")"
        else
            printf '  %-8s MISSING\n' "$p"
        fi
    done
else
    echo "  pkg-config 未安装"
fi

echo
echo "=== 运行库 .so（链接期需要） ==="
for lib in libdrm.so.2 libgbm.so.1 libEGL.so.1 libGLESv2.so.2; do
    found=$(ldconfig -p 2>/dev/null | grep -c "$lib")
    printf '  %-16s %s\n' "$lib" "$found"
done

echo
echo "=== DRM 设备 ==="
if [ -d /dev/dri ]; then
    ls -la /dev/dri
else
    echo "  /dev/dri 不存在（WSL2 正常，无法真机上屏）"
fi

echo
echo "=== 项目路径 ==="
PROJ=/mnt/d/ProgramData/drone_test/drone_test
if [ -d "$PROJ" ]; then
    echo "  OK $PROJ"
    echo "  hdmi 源文件:"
    ls -1 "$PROJ/videoPart/hdmi_part/src/hdmi" | sed 's/^/    /'
else
    echo "  MISSING $PROJ"
fi
