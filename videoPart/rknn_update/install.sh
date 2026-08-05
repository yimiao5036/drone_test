#!/bin/bash
# RKNN 运行时 2.3.2 更新脚本
# 用途：将 rknn_server 从 1.3.0 升级到 2.3.2，同步刷新 librknnrt.so
# 用法：解压后执行  cd rknn_update && sudo bash install.sh
set -e
cd "$(dirname "$0")"

echo "[1/4] 备份旧文件（已有备份则跳过）..."
[ -f /usr/bin/rknn_server ] && [ ! -f /usr/bin/rknn_server.bak ] && cp /usr/bin/rknn_server /usr/bin/rknn_server.bak
[ -f /usr/lib/librknnrt.so ] && [ ! -f /usr/lib/librknnrt.so.bak ] && cp /usr/lib/librknnrt.so /usr/lib/librknnrt.so.bak

echo "[2/4] 安装 rknn_server..."
cp rknn_server /usr/bin/rknn_server
cp restart_rknn.sh start_rknn.sh /usr/bin/ 2>/dev/null || true
chmod 755 /usr/bin/rknn_server

echo "[3/4] 安装 librknnrt.so..."
cp librknnrt.so /usr/lib/librknnrt.so

echo "[4/4] 安装完成。请执行 sudo reboot 重启生效。"
echo "重启后验证："
echo "  strings /usr/bin/rknn_server | grep -m1 -i version   # 应为 2.3.x"
echo "  ~/yoloPart/yolo26-rknn/build/yolo26_rknn             # 对比 [Perf] 推理时间"
