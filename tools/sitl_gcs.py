#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SITL 验收用地面站心跳 + Home 读取。

PX4 SITL 默认 GCS 链路在 UDP 14550；本脚本以 255/190 发 MAV_TYPE_GCS
心跳（满足 commander 无 RC 解锁条件），并打印 HOME_POSITION 与本机位置，
供 assignment_sitl.py 选择目标经纬度。

用法：python3 sitl_gcs.py [--connection udpin:0.0.0.0:14550] [--duration 0]
"""

import argparse
import os
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", default="udpin:0.0.0.0:14550")
    parser.add_argument("--duration", type=float, default=0,
                        help="0=一直运行")
    args = parser.parse_args()

    master = mavutil.mavlink_connection(
        args.connection, source_system=255, source_component=190)
    print(f"[GCS] 连接 {args.connection}，来源 255/190", flush=True)

    last_heartbeat = 0.0
    last_report = 0.0
    home = None
    start = time.monotonic()
    while args.duration <= 0 or time.monotonic() - start < args.duration:
        now = time.monotonic()
        if now - last_heartbeat >= 1.0:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE)
            last_heartbeat = now
        msg = master.recv_match(blocking=True, timeout=0.1)
        if msg is None:
            continue
        if msg.get_type() == "HOME_POSITION" and home is None:
            home = (msg.latitude / 1e7, msg.longitude / 1e7,
                    msg.altitude / 1000.0)
            print(f"[GCS] HOME lat={home[0]:.7f} lon={home[1]:.7f} alt={home[2]:.3f}",
                  flush=True)
        if now - last_report >= 5.0:
            last_report = now
            hb = master.messages.get("HEARTBEAT")
            if hb is not None:
                armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                print(f"[GCS] mode={hb.custom_mode} armed={armed}", flush=True)
            if home is None:
                print("[GCS] 尚未收到 HOME_POSITION", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
