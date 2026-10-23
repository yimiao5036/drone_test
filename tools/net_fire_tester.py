#!/usr/bin/env python3
"""65016 捕网直控发射指令测试工具（G4-4）。

用法示例（SITL/台架；带桨绝对禁止）：
  # 正常发射：落地且未解锁时受理，观察 65017 ACCEPTED → COMPLETED
  python3 tools/net_fire_tester.py --connection udpin:0.0.0.0:14550 \
      --aircraft-system 1 --aircraft-component 25 --command-id 1

  # 负向测试：不置确认位（预期 REJECTED/CONFIRMATION_MISSING）
  python3 tools/net_fire_tester.py ... --no-confirm

  # 重复帧幂等测试：同一 command_id 连发 3 次（预期仅首次受理）
  python3 tools/net_fire_tester.py ... --repeat 3 --repeat-interval 0.5

协议（V2_EXTENSION 短帧，payload 55 字节，地址 50/51 双字段必须匹配目标飞机）：
  65016 NET_FIRE：0=version(u8) 1=command_id(u32 LE) 5=flags(u8) 50=sys 51=comp 54=marker(1)
  65017 NET_FIRE_ACK：0=version 1=command_id 5=result 6=reason 7=actuator_state
                     50=飞机 sys 51=飞机 comp 54=marker(1)

退出码：0=受理（含 COMPLETED）；1=被拒绝；2=超时无回执；3=参数错误。
"""

import argparse
import os
import signal
import struct
import sys
import time

os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil
except ModuleNotFoundError as error:
    print("[ERROR] 缺少pymavlink，请先执行: python -m pip install -U pymavlink", file=sys.stderr)
    raise SystemExit(1) from error

GROUND_SYSTEM_ID = 255
GROUND_COMPONENT_ID = 190

MAVLINK2_MAGIC = 0xFD
MAVLINK_MSG_ID_V2_EXTENSION = 248
MAVLINK_MSG_ID_V2_EXTENSION_CRC_EXTRA = 8

NET_FIRE_TYPE = 65016
NET_FIRE_ACK_TYPE = 65017

PROTOCOL_VERSION = 1
SHORT_FRAME_USED_LEN = 55
FRAME_LENGTH_MARKER = 1
FLAG_CONFIRMED = 0x01

RESULT_NAMES = {0: "ACCEPTED", 1: "COMPLETED", 2: "REJECTED"}

REASON_NAMES = {
    0: "NONE",
    1: "INVALID_MESSAGE",
    2: "UNSUPPORTED_VERSION",
    3: "CONFIRMATION_MISSING",
    4: "DUPLICATE_COMMAND",
    5: "COMMAND_STALE",
    6: "DISABLED",
    7: "FLIGHT_STATE_STALE",
    8: "ARMED",
    9: "NOT_LANDED",
    10: "ACTUATOR_UNAVAILABLE",
    11: "ACTUATOR_FAULTED",
    12: "ACTUATOR_BUSY",
    13: "COOLDOWN",
    14: "NO_REMAINING_SHOTS",
    15: "SEND_FAILED",
}

ACTUATOR_STATE_NAMES = {0: "READY", 1: "BUSY", 2: "FAULTED"}

_stop = False


def on_signal(_signum, _frame):
    global _stop
    _stop = True


# ---------------------------------------------------------------------------
# MAVLink2 编码（与机载 ground_station_link 的短帧编码保持一致）
# ---------------------------------------------------------------------------
def x25_crc_accumulate(crc, byte):
    tmp = byte ^ (crc & 0xFF)
    tmp ^= (tmp << 4) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def x25_crc(data, crc_extra):
    crc = 0xFFFF
    for byte in data:
        crc = x25_crc_accumulate(crc, byte)
    return x25_crc_accumulate(crc, crc_extra)


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
    master.write(
        bytes([MAVLINK2_MAGIC]) + header_without_magic + payload + struct.pack("<H", checksum)
    )


def send_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_v2_extension(master, message_type, target_system, target_component, payload_used):
    extension_payload = struct.pack(
        "<HBBB", int(message_type), 0, int(target_system), int(target_component)
    ) + payload_used
    send_mavlink2_frame(
        master,
        MAVLINK_MSG_ID_V2_EXTENSION,
        extension_payload,
        MAVLINK_MSG_ID_V2_EXTENSION_CRC_EXTRA,
    )
    return len(extension_payload)


# ---------------------------------------------------------------------------
# 65016 NET_FIRE
# ---------------------------------------------------------------------------
def build_net_fire_payload(args, flags):
    payload = bytearray(SHORT_FRAME_USED_LEN)
    # 0=version(u8)，1=command_id(u32 LE)，5=flags(u8)
    struct.pack_into("<BI", payload, 0, int(args.version), int(args.command_id))
    payload[5] = int(flags)
    # 50/51 为 payload 内地址双字段；54 为短帧长度标记。
    payload[50] = int(args.aircraft_system) & 0xFF
    payload[51] = int(args.aircraft_component) & 0xFF
    payload[54] = FRAME_LENGTH_MARKER
    return bytes(payload)


def send_net_fire(master, args, flags):
    payload = build_net_fire_payload(args, flags)
    payload_len = send_v2_extension(
        master,
        NET_FIRE_TYPE,
        args.aircraft_system,
        args.aircraft_component,
        payload,
    )
    print(
        "[INFO] TX 65016 command_id={} version={} flags=0x{:02X} address={}/{} "
        "mavlink_payload_len={}".format(
            args.command_id,
            args.version,
            flags,
            args.aircraft_system,
            args.aircraft_component,
            payload_len,
        )
    )


# ---------------------------------------------------------------------------
# 下行解析：65017
# ---------------------------------------------------------------------------
def extract_v2(message):
    """把 V2_EXTENSION 解析为 {type, target_system, target_component, payload}。"""
    if message is None or message.get_type() != "V2_EXTENSION":
        return None
    if hasattr(message, "message_type"):
        return {
            "type": int(message.message_type),
            "target_system": int(message.target_system),
            "target_component": int(message.target_component),
            "payload": bytes(message.payload),
        }
    try:
        raw = bytes(message.get_msgbuf())
    except Exception:
        raw = bytes(getattr(message, "_msgbuf", b""))
    if len(raw) < 10 or raw[0] != MAVLINK2_MAGIC:
        return None
    payload_len = raw[1]
    ext = raw[10:10 + payload_len]
    if len(ext) < 5:
        return None
    return {
        "type": struct.unpack_from("<H", ext, 0)[0],
        "target_system": ext[3],
        "target_component": ext[4],
        "payload": ext[5:],
    }


def parse_net_fire_ack(v2):
    if v2 is None or v2["type"] != NET_FIRE_ACK_TYPE:
        return None
    payload = v2["payload"]
    if len(payload) < 8:
        return None
    return {
        "protocol_version": payload[0],
        "command_id": struct.unpack_from("<I", payload, 1)[0],
        "result": payload[5],
        "reason": payload[6],
        "actuator_state": payload[7] if len(payload) > 7 else 0,
    }


def print_net_fire_ack(ack):
    result = RESULT_NAMES.get(ack["result"], f"UNKNOWN({ack['result']})")
    reason = REASON_NAMES.get(ack["reason"], f"UNKNOWN({ack['reason']})")
    actuator = ACTUATOR_STATE_NAMES.get(
        ack["actuator_state"], f"UNKNOWN({ack['actuator_state']})"
    )
    print(
        "[ACK] 65017 command_id={} result={} reason={} actuator={}".format(
            ack["command_id"], result, reason, actuator
        )
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="以地面站255/190身份发送65016捕网直控发射并观察65017回执；"
        "机载只在落地且未解锁时受理（拆桨测试）"
    )
    parser.add_argument("--connection", default="udpout:192.168.144.12:19856",
                        help="pymavlink连接串；SITL常用 udpin:0.0.0.0:14550")
    parser.add_argument("--aircraft-system", type=int, default=1, help="目标飞机 system_id")
    parser.add_argument("--aircraft-component", type=int, default=25,
                        help="目标飞机 component_id（25=捕网，26=火箭）")
    parser.add_argument("--command-id", type=int, default=None,
                        help="指令编号；缺省用当前毫秒时间戳，重复测试请显式指定")
    parser.add_argument("--version", type=int, default=PROTOCOL_VERSION,
                        help="协议版本（负向测试可置 0）")
    parser.add_argument("--no-confirm", action="store_true", help="不置flags.bit0（负向测试）")
    parser.add_argument("--repeat", type=int, default=1, help="同一command_id重发次数（幂等测试）")
    parser.add_argument("--repeat-interval", type=float, default=0.5, help="重发间隔秒")
    parser.add_argument("--timeout", type=float, default=8.0, help="等待65017总时长（秒）")
    parser.add_argument("--heartbeat-interval", type=float, default=1.0)
    parser.add_argument("--connect-timeout", type=float, default=5.0,
                        help="等待飞机心跳的时长；0=不等待")
    parser.add_argument("--dry-run", action="store_true", help="只打印payload十六进制，不发送")
    parser.add_argument("--debug-messages", action="store_true", help="打印V2_EXTENSION类型")
    args = parser.parse_args()
    if args.command_id is None:
        args.command_id = int(time.time() * 1000) & 0x7FFFFFFF
    return args


def main():
    args = parse_args()
    if not 1 <= args.aircraft_system <= 254:
        raise ValueError("aircraft-system 必须在 1~254")
    if not 1 <= args.aircraft_component <= 255:
        raise ValueError("aircraft-component 必须在 1~255")
    if args.command_id <= 0:
        raise ValueError("command-id 必须为正数")
    if not 0 <= args.version <= 255:
        raise ValueError("version 必须在 0~255")
    if args.repeat <= 0 or args.timeout <= 0 or args.connect_timeout < 0:
        raise ValueError("repeat/timeout 必须为正，connect-timeout 不能为负")

    flags = 0 if args.no_confirm else FLAG_CONFIRMED

    if args.dry_run:
        print(
            "[DRY] payload={} 说明: 0=version 1-4=command_id 5=flags 50/51=地址 54=marker".format(
                build_net_fire_payload(args, flags).hex()
            )
        )
        return 0

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    master = mavutil.mavlink_connection(
        args.connection,
        source_system=GROUND_SYSTEM_ID,
        source_component=GROUND_COMPONENT_ID,
    )
    print(f"[INFO] 连接: {args.connection}")

    # 等待目标飞机心跳，确认链路地址正确（不等待时直接发）。
    deadline = time.monotonic() + args.connect_timeout
    while args.connect_timeout > 0 and time.monotonic() < deadline:
        send_heartbeat(master)
        message = master.recv_match(blocking=True, timeout=0.2)
        if message is None:
            continue
        if message.get_type() == "HEARTBEAT":
            print(
                "[INFO] 收到心跳: {}/{}".format(message.get_srcSystem(), message.get_srcComponent())
            )
            break
    else:
        if args.connect_timeout > 0:
            print("[WARN] 未收到心跳，仍继续发送（检查连接串与地址）")

    # 发送 65016（同一 command_id 重发用于幂等验证）。
    for index in range(args.repeat):
        if _stop:
            break
        send_net_fire(master, args, flags)
        if index + 1 < args.repeat:
            end = time.monotonic() + args.repeat_interval
            while time.monotonic() < end:
                master.recv_match(blocking=True, timeout=0.05)
                time.sleep(0.02)

    accepted = False
    completed = False
    rejected_reason = None
    timeout_at = time.monotonic() + args.timeout
    next_heartbeat = 0.0
    while not _stop and time.monotonic() < timeout_at:
        now = time.monotonic()
        if now >= next_heartbeat:
            send_heartbeat(master)
            next_heartbeat = now + args.heartbeat_interval
        message = master.recv_match(blocking=True, timeout=0.1)
        if message is None:
            continue
        v2 = extract_v2(message)
        if v2 is None:
            continue
        if args.debug_messages:
            print(
                "[DEBUG] RX V2_EXTENSION type={} source={}/{}".format(
                    v2["type"], message.get_srcSystem(), message.get_srcComponent()
                )
            )
        ack = parse_net_fire_ack(v2)
        if ack is None or ack["command_id"] != args.command_id:
            continue
        print_net_fire_ack(ack)
        if ack["result"] == 0:
            accepted = True
        elif ack["result"] == 2:
            rejected_reason = ack["reason"]
            break
        elif ack["result"] == 1:
            completed = True
            break

    if completed:
        print("[PASS] 发射周期完成（ACCEPTED → COMPLETED）")
        return 0
    if accepted:
        print("[PASS] 已受理（未见COMPLETED回执，检查执行器复位时序/回执链路）")
        return 0
    if rejected_reason is not None:
        print(
            "[FAIL] 被拒绝: {}".format(
                REASON_NAMES.get(rejected_reason, f"UNKNOWN({rejected_reason})")
            )
        )
        return 1
    print("[FAIL] 等待65017回执超时（未装配直控发射 / gcs_fire_enabled=false / 链路不通）")
    return 2


if __name__ == "__main__":
    signal.signal(signal.SIGINT, on_signal)
    raise SystemExit(main())
