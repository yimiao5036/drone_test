#!/usr/bin/env bash
# ============================================================
# 香橙派 MediaMTX 安装脚本 —— 图传 RTSP 服务器 中转方案
# 架构：香橙派程序 推 → 本机 mediamtx → 图传设备 拉
# ============================================================
# 用法：
#   1) 把 mediamtx 二进制放到 /opt/mediamtx/mediamtx（x64/aarch64 视架构）
#   2) 运行本脚本：sudo ./install_mediamtx.sh
# ============================================================
set -e

MEDIAMTX_DIR=/opt/mediamtx
MEDIAMTX_CFG=${MEDIAMTX_DIR}/mediamtx.yml
SERVICE=mediamtx

echo "[1/4] 检查/创建目录 ${MEDIAMTX_DIR}"
sudo mkdir -p "${MEDIAMTX_DIR}"

# 复制随本脚本一同部署的配置文件（若无则以默认路径兜底）
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
if [ -f "${SCRIPT_DIR}/mediamtx.yml" ]; then
  echo "[2/4] 安装配置 ${MEDIAMTX_CFG}"
  sudo cp "${SCRIPT_DIR}/mediamtx.yml" "${MEDIAMTX_CFG}"
else
  echo "[2/4] 未找到 mediamtx.yml，跳过（请手工放置）"
fi

echo "[3/4] 校验 mediamtx 二进制"
if [ ! -f "${MEDIAMTX_DIR}/mediamtx" ]; then
  echo "  [ERROR] 未找到 ${MEDIAMTX_DIR}/mediamtx"
  echo "  请先下载解压 MediaMTX 到 /opt/mediamtx/mediamtx"
  echo "  下载: https://github.com/bluenviron/mediamtx/releases"
  echo "  (香橙派 RK3588 = linux arm64: mediamtx_<ver>_linux_arm64.tar.gz)"
  exit 1
fi
chmod +x "${MEDIAMTX_DIR}/mediamtx"

echo "[4/4] 注册 systemd 服务并自启"
sudo tee /etc/systemd/system/${SERVICE}.service > /dev/null <<'EOF'
[Unit]
Description=MediaMTX RTSP server (drone video output)
After=network.target

[Service]
ExecStart=/opt/mediamtx/mediamtx /opt/mediamtx/mediamtx.yml
Restart=always
RestartSec=2
User=orangepi

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable ${SERVICE}
sudo systemctl restart ${SERVICE}
sudo systemctl status ${SERVICE} --no-pager || true

echo ""
echo "== 安装完成 =="
echo "mediamtx 已启动，监听 8554 端口（推流/拉流）"
echo "香橙派程序 config.json 的 video.output_rtsp 应为身份模板:"
echo "  rtsp://127.0.0.1:8554/drone_{aircraft_component_id}_{aircraft_system_id}"
echo "当前生产默认捕网-01解析后为:"
echo "  rtsp://127.0.0.1:8554/drone_25_1"
echo "图传设备/地面端拉取地址:"
echo "  rtsp://<香橙派IP>:8554/drone_25_1"
echo "  可用命令查看对外 IP: ip -br addr | grep 144"
echo "本机验证拉流:  ffplay rtsp://127.0.0.1:8554/drone_25_1"
