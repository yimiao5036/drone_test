#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SITL 验收用双 PTY 桥：把机载地面站串口与 pymavlink 连接对接。

创建两个独立 PTY，建立 /tmp/gcs_app（给 drone_control config 的
ground_station.serial.device）与 /tmp/gcs_test（给 assignment_sitl.py 的
serial: 连接）符号链接，并双向转发字节。

用法：python3 sitl_pty_bridge.py [--suffix _1]
退出：Ctrl-C 或 SIGTERM；清理符号链接。
"""

import argparse
import os
import pty
import select
import signal
import sys
import tty

running = True


def on_signal(_signum, _frame):
    global running
    running = False


def make_pty():
    master, slave = pty.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    return master, slave, os.ttyname(slave)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--suffix", default="", help="符号链接后缀，避免多场景冲突")
    args = parser.parse_args()
    app_link = "/tmp/gcs_app" + args.suffix
    test_link = "/tmp/gcs_test" + args.suffix

    m_app, s_app, app_name = make_pty()
    m_test, s_test, test_name = make_pty()
    for link, target in ((app_link, app_name), (test_link, test_name)):
        try:
            os.unlink(link)
        except FileNotFoundError:
            pass
        os.symlink(target, link)
    print(f"[BRIDGE] app={app_link} -> {app_name}  test={test_link} -> {test_name}",
          flush=True)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    fds = [m_app, m_test]
    peer = {m_app: m_test, m_test: m_app}
    try:
        while running:
            readable, _, _ = select.select(fds, [], [], 0.2)
            for fd in readable:
                try:
                    data = os.read(fd, 4096)
                except OSError:
                    data = b""
                if data:
                    try:
                        os.write(peer[fd], data)
                    except OSError:
                        # 对端 PTY 尚未被打开/已关闭：丢弃本帧，不中断桥接
                        pass
    finally:
        for link in (app_link, test_link):
            try:
                os.unlink(link)
            except FileNotFoundError:
                pass
        for fd in (m_app, m_test, s_app, s_test):
            try:
                os.close(fd)
            except OSError:
                pass
        print("[BRIDGE] 已退出", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
