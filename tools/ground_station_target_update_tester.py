#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地面站目标位置UPDATE/ACK实机测试工具。

安全边界：
1. 固定以255/190发送MAV_TYPE_GCS心跳；
2. 响应飞机发起的标准TIMESYNC请求；
3. 主动维持TIMESYNC测量，用于构造source_time_ms；
4. 发送V2_EXTENSION承载的TRACK_TARGET_UPDATE并等待TRACK_TARGET_ACK。

不发送PX4命令、模式切换、解锁、起飞、降落或setpoint。
"""

import argparse
import os
import signal
import statistics
import struct
import sys
import time
from dataclasses import dataclass

os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil
except ModuleNotFoundError as error:
    print("[ERROR] 缺少pymavlink，请先执行: python -m pip install -U pymavlink", file=sys.stderr)
    raise SystemExit(1) from error

GROUND_SYSTEM_ID = 255
GROUND_COMPONENT_ID = 190
MAVLINK2_MAGIC = 0xFD
MAVLINK_MSG_ID_TIMESYNC = 111
MAVLINK_MSG_ID_TIMESYNC_CRC_EXTRA = 34
MAVLINK_MSG_ID_TIMESYNC_LEN = 18
MAVLINK_MSG_ID_V2_EXTENSION = 248
MAVLINK_MSG_ID_V2_EXTENSION_CRC_EXTRA = 8
MAVLINK_MSG_ID_V2_EXTENSION_LEN = 254
V2_EXTENSION_PAYLOAD_LEN = 249
TRACK_TARGET_UPDATE_USED_LEN = 55
TRACK_TARGET_UPDATE_TYPE = 65010
TRACK_TARGET_ACK_TYPE = 65011
TRACK_TARGET_PROTOCOL_VERSION = 1
TRACK_TARGET_COORDINATE_FRAME_WGS84 = 1
TRACK_TARGET_FLAGS_ALL = 0x007F

ACK_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "REJECTED_INVALID_FIELD",
    2: "REJECTED_STALE_OR_DUPLICATE",
    3: "REJECTED_UNSUPPORTED_VERSION",
    4: "REJECTED_TIME_SYNC_UNAVAILABLE",
    5: "REJECTED_NOT_READY",
    6: "REJECTED_INTERNAL_ERROR",
}

ACK_REASON_NAMES = {
    0: "OK",
    1: "SOURCE_ID_INVALID",
    2: "TARGET_ADDRESS_MISMATCH",
    3: "PROTOCOL_VERSION_UNSUPPORTED",
    4: "BOOT_ID_INVALID_OR_CHANGED",
    5: "UPDATE_SEQUENCE_STALE",
    6: "TARGET_ID_INVALID",
    7: "LATITUDE_OR_LONGITUDE_INVALID",
    8: "VALID_FOR_INVALID",
    9: "FLAGS_INVALID",
    10: "ALTITUDE_REFERENCE_INVALID",
    11: "HEADING_INVALID",
    12: "ACCURACY_INVALID",
    13: "TIME_SYNC_UNAVAILABLE",
    14: "SOURCE_TIME_IN_FUTURE",
    15: "TARGET_EXPIRED",
    16: "REMAINING_VALIDITY_TOO_SHORT",
    17: "MODULE_NOT_READY",
    100: "INTERNAL_ERROR",
}

running = True


def on_signal(_signum, _frame):
    global running
    running = False


def monotonic_ms():
    return time.monotonic_ns() // 1_000_000


def x25_crc_accumulate(crc, byte):
    tmp = byte ^ (crc & 0xFF)
    tmp ^= (tmp << 4) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def x25_crc(data, crc_extra):
    crc = 0xFFFF
    for byte in data:
        crc = x25_crc_accumulate(crc, byte)
    crc = x25_crc_accumulate(crc, crc_extra)
    return crc


def next_sequence(master):
    sequence = int(getattr(master.mav, "seq", 0)) & 0xFF
    setattr(master.mav, "seq", (sequence + 1) & 0xFF)
    return sequence


def send_mavlink2_frame(master, msg_id, payload, crc_extra):
    sequence = next_sequence(master)
    msg_id = int(msg_id)
    header_without_magic = struct.pack(
        "<BBBBBBBBB",
        len(payload),
        0,
        0,
        sequence,
        GROUND_SYSTEM_ID,
        GROUND_COMPONENT_ID,
        msg_id & 0xFF,
        (msg_id >> 8) & 0xFF,
        (msg_id >> 16) & 0xFF,
    )
    checksum = x25_crc(header_without_magic + payload, crc_extra)
    master.write(bytes([MAVLINK2_MAGIC]) + header_without_magic + payload + struct.pack("<H", checksum))


def send_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_timesync(master, tc1, ts1, target_system, target_component):
    payload = struct.pack("<qqBB", int(tc1), int(ts1), int(target_system), int(target_component))
    send_mavlink2_frame(master, MAVLINK_MSG_ID_TIMESYNC, payload, MAVLINK_MSG_ID_TIMESYNC_CRC_EXTRA)


def extract_timesync_targets(message):
    if hasattr(message, "target_system") and hasattr(message, "target_component"):
        return int(message.target_system), int(message.target_component), True
    try:
        raw = bytes(message.get_msgbuf())
    except Exception:
        raw = bytes(getattr(message, "_msgbuf", b""))
    if len(raw) >= 10 and raw[0] == MAVLINK2_MAGIC:
        payload_len = raw[1]
        payload_start = 10
        payload_end = payload_start + payload_len
        if len(raw) >= payload_end + 2 and payload_len >= MAVLINK_MSG_ID_TIMESYNC_LEN:
            payload = raw[payload_start:payload_end]
            return int(payload[16]), int(payload[17]), True
    return 0, 0, False


def respond_timesync(master, request):
    send_timesync(master, time.monotonic_ns(), int(request.ts1), request.get_srcSystem(), request.get_srcComponent())


def le_u8(data, offset):
    return data[offset]


def le_u16(data, offset):
    return struct.unpack_from("<H", data, offset)[0]


def le_u32(data, offset):
    return struct.unpack_from("<I", data, offset)[0]


@dataclass
class TimeSyncState:
    pending: dict
    offsets_ms: list
    rtt_ms: list
    responses: int = 0
    aircraft_requests: int = 0
    timeouts: int = 0
    ignored: int = 0

    def synchronized(self, minimum_samples):
        return len(self.offsets_ms) >= minimum_samples

    def ground_offset_ms(self):
        if not self.offsets_ms:
            return 0.0
        # 工具测得语义：飞机时钟 - 地面站时钟；构造目标source_time要用相反数。
        aircraft_minus_ground = statistics.median(self.offsets_ms)
        return -aircraft_minus_ground


def debug_message(args, message, prefix="RX"):
    if not getattr(args, "debug_messages", False) or message is None:
        return
    text = f"[DEBUG] {prefix} type={message.get_type()} source={message.get_srcSystem()}/{message.get_srcComponent()}"
    if message.get_type() == "V2_EXTENSION":
        try:
            if hasattr(message, "message_type"):
                text += (
                    f" message_type={int(message.message_type)}"
                    f" target={int(message.target_system)}/{int(message.target_component)}"
                )
            else:
                raw = bytes(message.get_msgbuf())
                payload_len = raw[1]
                ext = raw[10:10 + payload_len]
                if len(ext) >= 5:
                    text += f" message_type={le_u16(ext, 0)} target={ext[3]}/{ext[4]}"
        except Exception as error:
            text += f" v2_debug_error={error}"
    elif message.get_type() == "TIMESYNC":
        target_system, target_component, has_targets = extract_timesync_targets(message)
        text += f" tc1={int(message.tc1)} ts1={int(message.ts1)} target={target_system}/{target_component} has_target={has_targets}"
    print(text)


def process_message(master, message, aircraft_system, aircraft_component, sync_state, args=None):
    if message is None:
        return None
    if args is not None:
        debug_message(args, message)
    if message.get_type() == "TIMESYNC":
        target_system, target_component, has_targets = extract_timesync_targets(message)
        source_system = message.get_srcSystem()
        source_component = message.get_srcComponent()
        addressed_to_ground = target_system == GROUND_SYSTEM_ID and target_component == GROUND_COMPONENT_ID
        broadcast = target_system == 0 and target_component == 0
        if int(message.tc1) == 0:
            if broadcast or addressed_to_ground:
                respond_timesync(master, message)
                sync_state.aircraft_requests += 1
            else:
                sync_state.ignored += 1
        elif (
            source_system == aircraft_system
            and source_component == aircraft_component
            and (addressed_to_ground or not has_targets)
            and int(message.ts1) in sync_state.pending
        ):
            receive_ns = time.monotonic_ns()
            request_ns = sync_state.pending.pop(int(message.ts1))
            if receive_ns > request_ns:
                rtt_ns = receive_ns - request_ns
                midpoint_ns = request_ns + rtt_ns // 2
                offset_ns = int(message.tc1) - midpoint_ns
                sync_state.rtt_ms.append(rtt_ns / 1_000_000.0)
                sync_state.offsets_ms.append(offset_ns / 1_000_000.0)
                if len(sync_state.rtt_ms) > 120:
                    sync_state.rtt_ms.pop(0)
                    sync_state.offsets_ms.pop(0)
                sync_state.responses += 1
            else:
                sync_state.ignored += 1
        else:
            sync_state.ignored += 1
    elif message.get_type() == "V2_EXTENSION":
        return message
    return None


def send_timesync_request(master, aircraft_system, aircraft_component, sync_state):
    request_ns = time.monotonic_ns()
    send_timesync(master, 0, request_ns, aircraft_system, aircraft_component)
    sync_state.pending[request_ns] = request_ns


def prune_timesync_pending(sync_state, request_timeout_s):
    expire_before_ns = time.monotonic_ns() - int(request_timeout_s * 1_000_000_000)
    expired = [key for key in sync_state.pending if key < expire_before_ns]
    for key in expired:
        sync_state.pending.pop(key, None)
        sync_state.timeouts += 1


def build_track_target_payload(args, boot_id, seq, source_time_ms, mode):
    target_system = args.aircraft_system
    target_component = args.aircraft_component
    lat = int(round(args.latitude * 10_000_000))
    lon = int(round(args.longitude * 10_000_000))
    valid_for_ms = args.valid_for_ms
    protocol_version = TRACK_TARGET_PROTOCOL_VERSION
    flags = TRACK_TARGET_FLAGS_ALL

    if mode == "wrong-address":
        target_system = 2 if args.aircraft_system != 2 else 3
    elif mode == "expired":
        source_time_ms -= args.valid_for_ms + 500
    elif mode == "invalid-lat":
        lat = 910000000
    elif mode == "bad-version":
        protocol_version = 99
    elif mode == "bad-flags":
        flags = 0x8000

    payload = bytearray(TRACK_TARGET_UPDATE_USED_LEN)
    struct.pack_into(
        "<QIIIIiii hhh HHHH BBBBB".replace(" ", ""),
        payload,
        0,
        int(source_time_ms),
        int(boot_id),
        int(seq),
        int(args.target_id),
        int(valid_for_ms),
        int(lat),
        int(lon),
        int(args.altitude_mm),
        int(round(args.velocity_north_mps * 100)),
        int(round(args.velocity_east_mps * 100)),
        int(round(args.velocity_down_mps * 100)),
        int(round(args.heading_deg * 100)),
        int(round(args.horizontal_accuracy_m * 100)),
        int(round(args.vertical_accuracy_m * 100)),
        int(flags),
        int(target_system),
        int(target_component),
        TRACK_TARGET_COORDINATE_FRAME_WGS84,
        int(protocol_version),
        int(args.alt_reference),
    )
    return bytes(payload), target_system, target_component


def send_track_target_update(master, args, boot_id, seq, source_time_ms, mode):
    payload_used, target_system, target_component = build_track_target_payload(
        args, boot_id, seq, source_time_ms, mode
    )
    # MAVLink2允许对尾部0字节做空值截断。目标UPDATE固定字段当前只使用55字节，
    # 不发送V2_EXTENSION的满长249字节payload，可避免低速串口/数传桥处理约266字节长帧时丢包。
    extension_payload = struct.pack(
        "<HBBB",
        TRACK_TARGET_UPDATE_TYPE,
        0,
        target_system,
        target_component,
    ) + payload_used
    send_mavlink2_frame(
        master,
        MAVLINK_MSG_ID_V2_EXTENSION,
        extension_payload,
        MAVLINK_MSG_ID_V2_EXTENSION_CRC_EXTRA,
    )
    return len(extension_payload)


def parse_ack(message):
    if message.get_type() != "V2_EXTENSION":
        return None
    source_system = message.get_srcSystem()
    source_component = message.get_srcComponent()
    if source_system == GROUND_SYSTEM_ID and source_component == GROUND_COMPONENT_ID:
        return None

    if hasattr(message, "message_type"):
        message_type = int(message.message_type)
        target_system = int(message.target_system)
        target_component = int(message.target_component)
        payload = bytes(message.payload)
    else:
        raw = bytes(message.get_msgbuf())
        payload_len = raw[1]
        payload_start = 10
        ext = raw[payload_start:payload_start + payload_len]
        if len(ext) < 5:
            return None
        message_type = le_u16(ext, 0)
        target_system = ext[3]
        target_component = ext[4]
        payload = ext[5:]

    if message_type != TRACK_TARGET_ACK_TYPE:
        return None
    if target_system != GROUND_SYSTEM_ID or target_component != GROUND_COMPONENT_ID:
        return None
    if len(payload) < 26:
        return None
    return {
        "source": f"{source_system}/{source_component}",
        "boot_id": le_u32(payload, 0),
        "update_seq": le_u32(payload, 4),
        "target_id": le_u32(payload, 8),
        "measured_age_ms": le_u32(payload, 12),
        "timesync_rtt_ms": le_u32(payload, 16),
        "reason": le_u16(payload, 20),
        "result": le_u8(payload, 22),
        "target_system": le_u8(payload, 23),
        "target_component": le_u8(payload, 24),
        "protocol_version": le_u8(payload, 25),
    }


def wait_for_sync(master, args, sync_state):
    last_heartbeat = 0.0
    last_request = 0.0
    deadline = time.monotonic() + args.sync_wait_timeout
    while running and time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_heartbeat >= args.heartbeat_interval:
            send_heartbeat(master)
            last_heartbeat = now
        if now - last_request >= args.request_interval:
            send_timesync_request(master, args.aircraft_system, args.aircraft_component, sync_state)
            last_request = now
        message = master.recv_match(blocking=True, timeout=0.05)
        process_message(master, message, args.aircraft_system, args.aircraft_component, sync_state, args)
        prune_timesync_pending(sync_state, args.request_timeout)
        if sync_state.synchronized(args.minimum_sync_samples):
            return True
    return False


def wait_for_ack(master, args, sync_state):
    last_heartbeat = 0.0
    last_request = 0.0
    deadline = time.monotonic() + args.ack_timeout
    while running and time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_heartbeat >= args.heartbeat_interval:
            send_heartbeat(master)
            last_heartbeat = now
        if now - last_request >= args.request_interval:
            send_timesync_request(master, args.aircraft_system, args.aircraft_component, sync_state)
            last_request = now
        message = master.recv_match(blocking=True, timeout=0.05)
        maybe_v2 = process_message(master, message, args.aircraft_system, args.aircraft_component, sync_state, args)
        if maybe_v2 is not None:
            ack = parse_ack(maybe_v2)
            if ack is not None:
                return ack
        prune_timesync_pending(sync_state, args.request_timeout)
    return None


def print_ack(mode, ack):
    if ack is None:
        print(f"[INFO] 测试={mode} ACK=无")
        return
    result_name = ACK_RESULT_NAMES.get(ack["result"], f"UNKNOWN({ack['result']})")
    reason_name = ACK_REASON_NAMES.get(ack["reason"], f"UNKNOWN({ack['reason']})")
    print(
        "[INFO] 测试={} ACK source={} result={} reason={} boot={} seq={} "
        "target_id={} measured_age_ms={} timesync_rtt_ms={} echo_target={}/{} proto={}".format(
            mode,
            ack["source"],
            result_name,
            reason_name,
            ack["boot_id"],
            ack["update_seq"],
            ack["target_id"],
            ack["measured_age_ms"],
            ack["timesync_rtt_ms"],
            ack["target_system"],
            ack["target_component"],
            ack["protocol_version"],
        )
    )


def parse_args():
    parser = argparse.ArgumentParser(description="发送TRACK_TARGET_UPDATE并接收ACK，不发送控制命令")
    parser.add_argument("--connection", default="udpout:192.168.144.12:19856")
    parser.add_argument("--aircraft-system", type=int, default=1)
    parser.add_argument("--aircraft-component", type=int, default=25)
    parser.add_argument("--heartbeat-interval", type=float, default=1.0)
    parser.add_argument("--request-interval", type=float, default=1.0)
    parser.add_argument("--request-timeout", type=float, default=3.0)
    parser.add_argument("--minimum-sync-samples", type=int, default=5)
    parser.add_argument("--sync-wait-timeout", type=float, default=10.0)
    parser.add_argument("--ack-timeout", type=float, default=10.0)
    parser.add_argument("--boot-id", type=int, default=None)
    parser.add_argument("--start-seq", type=int, default=1)
    parser.add_argument("--mode", choices=["accepted", "duplicate", "wrong-address", "expired", "invalid-lat", "bad-version", "bad-flags", "all"], default="accepted")
    parser.add_argument("--latitude", type=float, default=23.123)
    parser.add_argument("--longitude", type=float, default=113.123)
    parser.add_argument("--altitude-mm", type=int, default=120000)
    parser.add_argument("--alt-reference", type=int, default=1)
    parser.add_argument("--velocity-north-mps", type=float, default=1.2)
    parser.add_argument("--velocity-east-mps", type=float, default=-0.5)
    parser.add_argument("--velocity-down-mps", type=float, default=0.0)
    parser.add_argument("--heading-deg", type=float, default=90.0)
    parser.add_argument("--horizontal-accuracy-m", type=float, default=1.5)
    parser.add_argument("--vertical-accuracy-m", type=float, default=2.5)
    parser.add_argument("--valid-for-ms", type=int, default=1000)
    parser.add_argument("--target-id", type=int, default=7)
    parser.add_argument("--debug-messages", action="store_true", help="打印收到的MAVLink消息类型、来源和V2_EXTENSION类型")
    return parser.parse_args()


def run_one_test(master, args, sync_state, boot_id, seq, mode, label=None):
    # 协议要求source_time_ms使用地面站本机单调时钟，不能加offset。
    # 飞机端会使用TIMESYNC估计的“地面站-飞机”offset自行映射到飞机时钟。
    source_time_ms = int(monotonic_ms())
    payload_len = send_track_target_update(master, args, boot_id, seq, source_time_ms, mode)
    if args.debug_messages:
        print(
            f"[DEBUG] TX TRACK_TARGET_UPDATE mode={mode} boot={boot_id} seq={seq} "
            f"source_time_ms={source_time_ms} ground_offset_ms={sync_state.ground_offset_ms():.3f} "
            f"target={args.aircraft_system}/{args.aircraft_component} mavlink_payload_len={payload_len}"
        )
    ack = wait_for_ack(master, args, sync_state)
    print_ack(label or mode, ack)
    return ack


def main():
    args = parse_args()
    if not 1 <= args.aircraft_system <= 254:
        raise ValueError("aircraft-system必须在1~254")
    if not 1 <= args.aircraft_component <= 255:
        raise ValueError("aircraft-component必须在1~255")
    if args.valid_for_ms <= 0 or args.request_timeout <= 0 or args.ack_timeout <= 0:
        raise ValueError("有效期和超时必须为正数")

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    master = mavutil.mavlink_connection(
        args.connection,
        source_system=GROUND_SYSTEM_ID,
        source_component=GROUND_COMPONENT_ID,
    )
    boot_id = args.boot_id if args.boot_id is not None else int(time.time()) & 0xFFFFFFFF
    sync_state = TimeSyncState(pending={}, offsets_ms=[], rtt_ms=[])

    print(f"[INFO] 连接: {args.connection}")
    print(
        "[INFO] 地面站来源255/190，目标飞机={}/{}，UPDATE类型={} ACK类型={}，不发送控制".format(
            args.aircraft_system,
            args.aircraft_component,
            TRACK_TARGET_UPDATE_TYPE,
            TRACK_TARGET_ACK_TYPE,
        )
    )
    if not wait_for_sync(master, args, sync_state):
        raise RuntimeError("等待TIMESYNC稳定超时，未发送目标")
    print(
        "[INFO] TIMESYNC就绪: 样本={} offset_ms(ground-aircraft)={:.3f} rtt_ms_median={:.3f} aircraft_requests={} timeout={} ignored={}".format(
            len(sync_state.offsets_ms),
            sync_state.ground_offset_ms(),
            statistics.median(sync_state.rtt_ms) if sync_state.rtt_ms else 0.0,
            sync_state.aircraft_requests,
            sync_state.timeouts,
            sync_state.ignored,
        )
    )

    if args.mode == "all":
        tests = ["accepted", "duplicate", "wrong-address", "expired", "invalid-lat", "bad-version", "bad-flags"]
    else:
        tests = [args.mode]

    seq = args.start_seq
    accepted_seq = None
    for mode in tests:
        if mode == "duplicate":
            if accepted_seq is None:
                run_one_test(master, args, sync_state, boot_id, seq, "accepted", "duplicate-prepare")
                accepted_seq = seq
                time.sleep(0.2)
            run_one_test(master, args, sync_state, boot_id, accepted_seq, "accepted", "duplicate")
        else:
            ack = run_one_test(master, args, sync_state, boot_id, seq, mode)
            if mode == "accepted":
                accepted_seq = seq
            seq += 1
        time.sleep(0.2)

    print("[INFO] 完成")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        sys.exit(1)
