#!/usr/bin/env python3
"""把 SITL 的验收端点 /tmp/gcs_test 桥到 TCP，让 Windows 上的 Web 地面站接入。

背景：`tools/sitl_harness.sh stack` 起的链路是
    PX4 SITL ── UDP ── sitl_pty_bridge.py ── PTY ──> /tmp/gcs_app（机载 drone_control）
                                          └─ PTY ──> /tmp/gcs_test（验收脚本）

验收脚本和机载程序都在 WSL 里，能直接打开 PTY；但 Web 地面站的服务端跑在
Windows 上，打不开 WSL 的 /tmp 路径。本脚本在 WSL 侧打开 /tmp/gcs_test，把
字节原样搬到 TCP 端口（默认 5761），Windows 侧的 Web 服务端用「TCP 客户端」
链路连过来即可，机载代码与串口配置都不用改。

用法：
    python3 tools/sitl_web_gcs.py                 # 监听 0.0.0.0:5761
    python3 tools/sitl_web_gcs.py --port 5762     # 换端口
    python3 tools/sitl_web_gcs.py --pty /tmp/xxx  # 换端点

注意（同一个端点只能有一个“地面站”）：
- 接了 Web 地面站就要停掉 `tools/sitl_gcs.py`（它同样扮演 255/190 地面站、发心跳），
  否则心跳与指令来源打架。`tools/sitl_harness.sh webgcs` 会自动处理；
- Web 端必须发 255/190 心跳，并在机载的 `ground_station` 配置范围内，
  否则 65010~65022 会被地址校验丢掉；
- 停止：Ctrl-C，或 `tools/sitl_harness.sh stop`。
"""

from __future__ import annotations

import argparse
import os
import select
import socket
import sys
import termios
import tty

# 单次转发缓冲：MAVLink2 常见帧 < 300 字节，8 KB 足够且不会明显增加延迟。
BUFFER_SIZE = 8192


def log(message: str) -> None:
    """统一带前缀输出，便于在 harness 日志里 grep。"""
    print(f"[web-gcs] {message}", flush=True)


def open_pty(path: str) -> int:
    """打开 PTY 从端并设为 raw，避免行规程改写 MAVLink 二进制流。"""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd, termios.TCSANOW)
    return fd


def serve(pty_fd: int, host: str, port: int) -> None:
    """监听 TCP 并双向转发；客户端断开后继续等下一个连接。"""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    log(f"监听 {host}:{port}，等待 Web 地面站连接（PTY 就绪）")

    while True:
        conn, addr = server.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        log(f"Web 地面站已连接: {addr[0]}:{addr[1]}")
        to_web = 0  # PTY -> TCP 字节数（机载/飞控下行）
        to_pty = 0  # TCP -> PTY 字节数（Web 上行）
        try:
            while True:
                readable, _, errored = select.select([pty_fd, conn], [], [pty_fd, conn], 1.0)
                if errored:
                    log("端点异常，结束本次转发")
                    break
                if pty_fd in readable:
                    chunk = os.read(pty_fd, BUFFER_SIZE)
                    if not chunk:
                        log("PTY 返回 EOF（桥或机载已退出）")
                        return
                    conn.sendall(chunk)
                    to_web += len(chunk)
                if conn in readable:
                    chunk = conn.recv(BUFFER_SIZE)
                    if not chunk:
                        break
                    os.write(pty_fd, chunk)
                    to_pty += len(chunk)
        except (BrokenPipeError, ConnectionResetError, OSError) as exc:
            log(f"转发中断: {exc}")
        finally:
            conn.close()
            log(f"Web 地面站断开（本次下行 {to_web} 字节 / 上行 {to_pty} 字节）")


def main() -> int:
    parser = argparse.ArgumentParser(description="SITL 验收端点 PTY → TCP 桥")
    parser.add_argument("--pty", default="/tmp/gcs_test", help="PTY 从端路径（默认 /tmp/gcs_test）")
    parser.add_argument("--host", default="0.0.0.0", help="监听地址（默认 0.0.0.0，便于 Windows 访问）")
    parser.add_argument("--port", type=int, default=5761, help="监听端口（默认 5761）")
    args = parser.parse_args()

    if not os.path.exists(args.pty):
        log(f"端点不存在: {args.pty}（先执行 tools/sitl_harness.sh stack）")
        return 2
    try:
        pty_fd = open_pty(args.pty)
    except OSError as exc:
        log(f"打开端点失败: {exc}")
        return 2
    try:
        serve(pty_fd, args.host, args.port)
    except KeyboardInterrupt:
        log("收到 Ctrl-C，退出")
    finally:
        os.close(pty_fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
