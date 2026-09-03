#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地面站TIMESYNC实机测量工具。

安全边界：
1. 以固定来源255/190周期发送MAV_TYPE_GCS心跳；
2. 使用本机单调时钟响应飞机发起的MAVLink TIMESYNC请求；
3. 主动向指定飞机发送TIMESYNC请求并统计RTT/offset/jitter。

不发送目标位置、PX4命令、模式切换、解锁或setpoint。
"""

import argparse
import signal
import statistics
import sys
import time

from pymavlink import mavutil

GROUND_SYSTEM_ID = 255
GROUND_COMPONENT_ID = 190

running = True


def on_signal(_signum, _frame):
    global running
    running = False


def parse_args():
    parser = argparse.ArgumentParser(
        description="以255/190测量TIMESYNC，不发送任何控制命令")
    parser.add_argument(
        "--connection",
        default="udpout:192.168.144.12:19856",
        help="pymavlink连接串，默认udpout:192.168.144.12:19856",
    )
    parser.add_argument(
        "--aircraft-system",
        type=int,
        default=1,
        help="目标飞机system ID，默认1",
    )
    parser.add_argument(
        "--aircraft-component",
        type=int,
        default=25,
        help="目标飞机component ID，默认25（捕网类）",
    )
    parser.add_argument(
        "--heartbeat-interval",
        type=float,
        default=1.0,
        help="GCS心跳周期（秒），默认1.0",
    )
    parser.add_argument(
        "--request-interval",
        type=float,
        default=1.0,
        help="地面站主动TIMESYNC请求周期（秒），默认1.0",
    )
    parser.add_argument(
        "--summary-interval",
        type=float,
        default=5.0,
        help="统计摘要周期（秒），默认5.0",
    )
    parser.add_argument(
        "--window-size",
        type=int,
        default=120,
        help="统计窗口样本数，默认120",
    )
    return parser.parse_args()


def send_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_timesync(master, tc1, ts1, target_system, target_component):
    try:
        master.mav.timesync_send(
            int(tc1),
            int(ts1),
            int(target_system),
            int(target_component),
        )
    except TypeError as error:
        raise RuntimeError(
            "当前pymavlink生成代码不支持TIMESYNC目标扩展字段，"
            "请升级pymavlink后重试"
        ) from error


def send_time_sync_response(master, request):
    # 当前common.xml语义：响应方时间放tc1，请求方原始ts1镜像回ts1。
    send_timesync(
        master,
        time.monotonic_ns(),
        int(request.ts1),
        request.get_srcSystem(),
        request.get_srcComponent(),
    )


def percentile(values, ratio):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * ratio))))
    return ordered[index]


def print_summary(response_count, ignored_count, timeout_count, rtt_ms, offset_ms):
    if not rtt_ms:
        print(
            f"[INFO] 飞机请求响应={response_count} 主动测量样本=0 "
            f"超时={timeout_count} 忽略={ignored_count}"
        )
        return
    offset_median = statistics.median(offset_ms)
    offset_jitter = max(abs(value - offset_median) for value in offset_ms)
    print(
        "[INFO] 飞机请求响应={} 主动样本={} 超时={} 忽略={} "
        "RTT ms[min/median/p95/max]={:.3f}/{:.3f}/{:.3f}/{:.3f} "
        "aircraft-ground offset ms[median/jitter]={:.3f}/{:.3f}".format(
            response_count,
            len(rtt_ms),
            timeout_count,
            ignored_count,
            min(rtt_ms),
            statistics.median(rtt_ms),
            percentile(rtt_ms, 0.95),
            max(rtt_ms),
            offset_median,
            offset_jitter,
        )
    )


def main():
    args = parse_args()
    if (
        args.heartbeat_interval <= 0
        or args.request_interval <= 0
        or args.summary_interval <= 0
        or args.window_size <= 0
    ):
        raise ValueError("心跳、请求、摘要周期和窗口容量必须为正数")
    if not 1 <= args.aircraft_system <= 254:
        raise ValueError("aircraft-system必须在1～254")
    if not 1 <= args.aircraft_component <= 255:
        raise ValueError("aircraft-component必须在1～255")

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    master = mavutil.mavlink_connection(
        args.connection,
        source_system=GROUND_SYSTEM_ID,
        source_component=GROUND_COMPONENT_ID,
    )
    print(f"[INFO] 连接: {args.connection}")
    print(
        "[INFO] 地面站来源255/190，测量飞机={}/{}，不发送目标或控制".format(
            args.aircraft_system, args.aircraft_component
        )
    )

    aircraft_request_response_count = 0
    ignored_count = 0
    timeout_count = 0
    pending = {}
    rtt_ms = []
    offset_ms = []
    last_heartbeat = 0.0
    last_request = 0.0
    last_summary = time.monotonic()

    while running:
        now = time.monotonic()
        if now - last_heartbeat >= args.heartbeat_interval:
            send_heartbeat(master)
            last_heartbeat = now

        if now - last_request >= args.request_interval:
            request_time_ns = time.monotonic_ns()
            send_timesync(
                master,
                0,
                request_time_ns,
                args.aircraft_system,
                args.aircraft_component,
            )
            pending[request_time_ns] = request_time_ns
            last_request = now

        message = master.recv_match(blocking=True, timeout=0.05)
        if message is not None and message.get_type() == "TIMESYNC":
            target_system = int(getattr(message, "target_system", 0))
            target_component = int(getattr(message, "target_component", 0))
            source_system = message.get_srcSystem()
            source_component = message.get_srcComponent()
            is_broadcast = target_system == 0 and target_component == 0
            is_addressed_to_ground = (
                target_system == GROUND_SYSTEM_ID
                and target_component == GROUND_COMPONENT_ID
            )

            if int(message.tc1) == 0:
                if is_broadcast or is_addressed_to_ground:
                    send_time_sync_response(master, message)
                    aircraft_request_response_count += 1
                else:
                    ignored_count += 1
            elif (
                source_system == args.aircraft_system
                and source_component == args.aircraft_component
                and is_addressed_to_ground
                and int(message.ts1) in pending
            ):
                receive_time_ns = time.monotonic_ns()
                request_time_ns = pending.pop(int(message.ts1))
                if receive_time_ns > request_time_ns:
                    round_trip_ns = receive_time_ns - request_time_ns
                    midpoint_ns = request_time_ns + round_trip_ns // 2
                    # 此处offset语义为“飞机时钟 - 地面站时钟”。
                    offset_ns = int(message.tc1) - midpoint_ns
                    rtt_ms.append(round_trip_ns / 1_000_000.0)
                    offset_ms.append(offset_ns / 1_000_000.0)
                    if len(rtt_ms) > args.window_size:
                        rtt_ms.pop(0)
                        offset_ms.pop(0)
                else:
                    ignored_count += 1
            else:
                ignored_count += 1

        expire_before_ns = time.monotonic_ns() - 2_000_000_000
        expired = [key for key in pending if key < expire_before_ns]
        for key in expired:
            pending.pop(key, None)
            timeout_count += 1

        now = time.monotonic()
        if now - last_summary >= args.summary_interval:
            print_summary(
                aircraft_request_response_count,
                ignored_count,
                timeout_count,
                rtt_ms,
                offset_ms,
            )
            last_summary = now

    print_summary(
        aircraft_request_response_count,
        ignored_count,
        timeout_count,
        rtt_ms,
        offset_ms,
    )
    print("[INFO] 停止")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:  # 工具入口只输出一次错误。
        print(f"[ERROR] {error}", file=sys.stderr)
        sys.exit(1)
