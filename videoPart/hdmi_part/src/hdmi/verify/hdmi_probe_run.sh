#!/usr/bin/env bash
# hdmi_probe 运行时行为验证（在 WSL2 Ubuntu 中执行）
#
# 使用 KMS=OFF 构建（占位显示器）：Open 会成功，从而能跑通完整帧循环，
# 验证提交/丢弃/延迟统计与参数处理。真实上屏需在香橙派执行同样命令。
#
# 用法: bash videoPart/hdmi_part/src/hdmi/verify/hdmi_probe_run.sh

set -u

BUILD="$HOME/drone_hdmi_build/off"
PROBE="$BUILD/hdmi_probe"
TMP="$HOME/drone_hdmi_build/probe_nv12"

echo "=== 构建占位显示器版本（若尚未构建） ==="
if [ ! -x "$PROBE" ]; then
    cmake -S /mnt/d/ProgramData/drone_test/drone_test/videoPart/hdmi_part/src/hdmi \
          -B "$BUILD" -DDRONE_HAVE_HDMI_KMS=OFF -DCMAKE_BUILD_TYPE=Release >/dev/null
    cmake --build "$BUILD" -j"$(nproc)" >/dev/null
fi
echo "探针: $PROBE"
echo

echo "=== 1) test 子命令：彩条上屏 30 帧（占位显示器） ==="
"$PROBE" test --frames 30
echo "退出码: $?"

echo
echo "=== 2) 非默认参数：1920x1080@25 stride=2048 BT.601 全范围 ==="
"$PROBE" test --width 1920 --height 1080 --refresh 25 --stride 2048 \
         --bt601 --full-range --frames 10
echo "退出码: $?"

echo
echo "=== 3) nv12 子命令：读取裸 NV12 文件 ==="
# 生成 1280x720、stride=1344 的 NV12 测试帧（Y 渐变 + 中性 UV）
python3 - "$TMP" <<'PY'
import sys
path = sys.argv[1]
width, height, stride = 1280, 720, 1344
with open(path, "wb") as f:
    for y in range(height):
        f.write(bytes(((x * 255) // width) & 0xFF for x in range(width)))
        f.write(bytes(stride - width))          # 行填充
    for y in range(height // 2):
        f.write(bytes([128, 128]) * (width // 2))
        f.write(bytes(stride - width))
print("已生成", path)
PY

"$PROBE" nv12 "$TMP" --stride 1344 --frames 20
echo "退出码: $?"

echo
echo "=== 4) 文件过小：应报错并返回非 0 ==="
head -c 1000 "$TMP" > "$TMP.small"
"$PROBE" nv12 "$TMP.small" --stride 1344 --frames 1
echo "退出码: $?"

echo
echo "=== 5) 文件不存在：应报错并返回非 0 ==="
"$PROBE" nv12 /nonexistent.nv12
echo "退出码: $?"

rm -f "$TMP" "$TMP.small"
