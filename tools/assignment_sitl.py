#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多机任务分配（65014）/ 自动起飞 SITL 与台架验收工具。

安全边界：
1. 固定以地面站身份 255/190 发送 MAV_TYPE_GCS 心跳并响应飞机 TIMESYNC；
2. 只发送 V2_EXTENSION 65014 MISSION_ASSIGN，可选发送 65010 目标位置；
3. 订阅并打印 65015 MISSION_ASSIGN_ACK、65013 MISSION_STATUS（v2）；
4. 不直接发送 PX4 模式切换、解锁、起飞、降落或 setpoint。

用法示例（SITL）：
  # SITL 以 --out udp:127.0.0.1:14550 推送时，用 udpin 监听并回发
  python3 tools/assignment_sitl.py --connection udpin:0.0.0.0:14550 \
      --aircraft-system 1 --aircraft-component 25 \
      --assignment-id 1 --target-id 7 --interceptor-type 25 \
      --send-target --duration 30

  # 取消当前分配（flags.bit1）
  python3 tools/assignment_sitl.py --assignment-id 1 --cancel

  # 起飞前取消（800ms 后切取消）/ 起飞后取消（如 15000ms）
  python3 tools/assignment_sitl.py ... --cancel-after-ms 800

  # 目标丢失：先持续发送 65010，15s 后停止，等待目标超龄/丢失看门狗
  python3 tools/assignment_sitl.py ... --send-target --stop-target-after-ms 15000

  # 台架/实机：经 HM30/串口转发时沿用地面站链路连接串
  python3 tools/assignment_sitl.py --connection udpout:192.168.144.12:19856 \
      --aircraft-system 1 --aircraft-component 25 --assignment-id 1 --target-id 7
  # 直连串口设备时 pymavlink 直接传设备路径（默认 115200）：
  python3 tools/assignment_sitl.py --connection /dev/ttyUSB0

前置条件（自动分配才会被接受）：
- 目标飞机 enable_control=true 且已通过 G2~G4 门禁（生产配置保持 false）；
- 地面站链路在线（心跳）、TIMESYNC 同步；
- PX4 低电参数可读且 COM_LOW_BAT_ACT=0（状态机预检）。
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
MAVLINK_MSG_ID_TIMESYNC = 111
MAVLINK_MSG_ID_TIMESYNC_CRC_EXTRA = 34
MAVLINK_MSG_ID_V2_EXTENSION = 248
MAVLINK_MSG_ID_V2_EXTENSION_CRC_EXTRA = 8

MISSION_ASSIGN_TYPE = 65014
MISSION_ASSIGN_ACK_TYPE = 65015
MISSION_STATUS_TYPE = 65013
TRACK_TARGET_UPDATE_TYPE = 65010
TRACK_TARGET_ACK_TYPE = 65011

PROTOCOL_VERSION = 1
SHORT_FRAME_USED_LEN = 55
FRAME_LENGTH_MARKER = 1
FLAG_TAKEOFF_AUTHORIZED = 0x01
FLAG_CANCEL = 0x02

MISSION_ASSIGN_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "REJECTED",
    2: "CANCELLED_OR_ABORTED",
}

MISSION_ASSIGN_REASON_NAMES = {
    0: "NONE",
    1: "INVALID_MESSAGE",
    2: "UNSUPPORTED_VERSION",
    3: "UNSUPPORTED_INTERCEPTOR_TYPE",
    4: "NOT_READY",
    5: "ALREADY_ASSIGNED",
    6: "PREFLIGHT_FAILED",
    7: "CONTROL_DISABLED",
    8: "MODE_TIMEOUT",
    9: "ARM_TIMEOUT",
    10: "BOOTSTRAP_TIMEOUT",
    11: "TAKEOFF_TIMEOUT",
    12: "CANCELLED_BY_GCS",
    13: "TARGET_WAIT_TIMEOUT",
    14: "TARGET_LOST",
    15: "BATTERY_LOW",
    16: "BATTERY_TELEMETRY_INVALID",
    17: "BATTERY_CONFIG_UNKNOWN",
    18: "PX4_DISCONNECTED",
    19: "MISSION_FAULT",
    20: "INTERNAL_ERROR",
    21: "ZONE_MISMATCH",
}

AVAILABILITY_NAMES = {
    0: "UNAVAILABLE",
    1: "READY",
    2: "ASSIGNED",
    3: "ENGAGED",
    4: "RETURNING",
    5: "TERMINAL",
}

running = True


def on_signal(_signum, _frame):
    global running
    running = False


def monotonic_ms():
    return time.monotonic_ns() // 1_000_000


# ---------------------------------------------------------------------------
# MAVLink2 编码（V2_EXTENSION 短帧；与机载 ground_station_link 保持一致）
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
        if len(raw) >= payload_end + 2 and payload_len >= 18:
            payload = raw[payload_start:payload_end]
            return int(payload[16]), int(payload[17]), True
    return 0, 0, False


def respond_timesync(master, request):
    """回应飞机发起的 TIMESYNC（tc1=0）：ts1 回显、tc1 填本机单调时钟纳秒。"""
    send_timesync(
        master,
        time.monotonic_ns(),
        int(request.ts1),
        request.get_srcSystem(),
        request.get_srcComponent(),
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
# 65014 MISSION_ASSIGN
# ---------------------------------------------------------------------------
def build_assignment_payload(args, flags):
    payload = bytearray(SHORT_FRAME_USED_LEN)
    struct.pack_into(
        "<BIIBBI",
        payload,
        0,
        PROTOCOL_VERSION,
        int(args.assignment_id),
        int(args.target_id),
        int(args.interceptor_type),
        int(flags),
        int(args.valid_for_ms),
    )
    # offset 15/17：围栏区域号与版本（2026-10-29 增，落在 v1 保留区；0=未声明）。
    # 机载用它与本地 zone.json 缓存比对，不一致回 65015 reason=21(zone_mismatch)。
    zone_id = getattr(args, "zone_id", 0)
    zone_version = getattr(args, "zone_version", 0)
    if not (0 <= zone_id <= 0xFFFF) or not (0 <= zone_version <= 0xFFFF):
        raise ValueError("zone-id/zone-version 必须在 0~65535")
    struct.pack_into("<HH", payload, 15, zone_id, zone_version)
    # offset 19~49 reserved=0；50/51 为 payload 内地址双字段；54 为短帧标记。
    payload[50] = int(args.aircraft_system) & 0xFF
    payload[51] = int(args.aircraft_component) & 0xFF
    payload[54] = FRAME_LENGTH_MARKER
    return bytes(payload)


def send_assignment(master, args, flags):
    payload = build_assignment_payload(args, flags)
    payload_len = send_v2_extension(
        master,
        MISSION_ASSIGN_TYPE,
        args.aircraft_system,
        args.aircraft_component,
        payload,
    )
    label = "CANCEL" if flags & FLAG_CANCEL else "ASSIGN"
    print(
        "[INFO] TX 65014 {} assignment_id={} target_id={} interceptor_type={} flags=0x{:02X} "
        "valid_for_ms={} zone={}/{} address={}/{} mavlink_payload_len={}".format(
            label,
            args.assignment_id,
            args.target_id,
            args.interceptor_type,
            flags,
            args.valid_for_ms,
            args.zone_id,
            args.zone_version,
            args.aircraft_system,
            args.aircraft_component,
            payload_len,
        )
    )
    return payload_len


# ---------------------------------------------------------------------------
# 65010 TRACK_TARGET_UPDATE（可选，复用现有目标测试工具的短帧编码）
# ---------------------------------------------------------------------------
def build_target_payload(args, boot_id, seq):
    lat = int(round(args.target_latitude * 10_000_000))
    lon = int(round(args.target_longitude * 10_000_000))
    payload = bytearray(SHORT_FRAME_USED_LEN)
    struct.pack_into(
        "<QIIIIiiihhhHHHHBBBBB",
        payload,
        0,
        int(monotonic_ms()),
        int(boot_id),
        int(seq),
        int(args.target_id),
        int(args.target_valid_for_ms),
        lat,
        lon,
        int(round(args.target_altitude_m * 1000)),
        int(round(args.target_velocity_north_mps * 100)),
        int(round(args.target_velocity_east_mps * 100)),
        int(round(args.target_velocity_down_mps * 100)),
        int(round(args.target_heading_deg * 100)),
        int(round(args.target_horizontal_accuracy_m * 100)),
        int(round(args.target_vertical_accuracy_m * 100)),
        0x007F,
        int(args.aircraft_system),
        int(args.aircraft_component),
        1,  # WGS84
        PROTOCOL_VERSION,
        1,  # 高度参考：相对起飞点
    )
    return bytes(payload)


def send_target(master, args, boot_id, seq):
    payload = build_target_payload(args, boot_id, seq)
    payload_len = send_v2_extension(
        master,
        TRACK_TARGET_UPDATE_TYPE,
        args.aircraft_system,
        args.aircraft_component,
        payload,
    )
    if args.debug_messages:
        print(
            "[DEBUG] TX 65010 target_id={} seq={} pos={:.7f}/{:.7f} mavlink_payload_len={}".format(
                args.target_id, seq, args.target_latitude, args.target_longitude, payload_len
            )
        )
    return payload_len


# ---------------------------------------------------------------------------
# 下行解析：65015 / 65013 / 65011
# ---------------------------------------------------------------------------
def extract_v2(message):
    """把 V2_EXTENSION 解析为 {type, target, payload}；失败返回 None。"""
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


def parse_assignment_ack(v2, args):
    if v2 is None or v2["type"] != MISSION_ASSIGN_ACK_TYPE:
        return None
    payload = v2["payload"]
    if len(payload) < 28:
        return None
    if v2["target_system"] != GROUND_SYSTEM_ID or v2["target_component"] != GROUND_COMPONENT_ID:
        return None
    return {
        "protocol_version": payload[0],
        "assignment_id": struct.unpack_from("<I", payload, 1)[0],
        "target_id": struct.unpack_from("<I", payload, 5)[0],
        "result": payload[9],
        "reason": payload[10],
        "mission_state": payload[11],
        "availability": payload[12],
        "aircraft_system": payload[26],
        "aircraft_component": payload[27],
    }


def parse_mission_status(v2):
    if v2 is None or v2["type"] != MISSION_STATUS_TYPE:
        return None
    payload = v2["payload"]
    if len(payload) < 50:
        return None
    if v2["target_system"] != GROUND_SYSTEM_ID or v2["target_component"] != GROUND_COMPONENT_ID:
        return None
    return {
        "protocol_version": payload[0],
        "sequence": struct.unpack_from("<I", payload, 1)[0],
        "state": payload[5],
        "control_source": payload[6],
        "task_phase": payload[7],
        "assignment_id": struct.unpack_from("<I", payload, 40)[0],
        "target_id": struct.unpack_from("<I", payload, 44)[0],
        "availability": payload[48],
        "interceptor_type": payload[49],
        "aircraft_system": payload[26],
        "aircraft_component": payload[27],
    }


def parse_target_ack(v2):
    if v2 is None or v2["type"] != TRACK_TARGET_ACK_TYPE:
        return None
    payload = v2["payload"]
    if len(payload) < 26:
        return None
    if v2["target_system"] != GROUND_SYSTEM_ID or v2["target_component"] != GROUND_COMPONENT_ID:
        return None
    return {
        "result": payload[22],
        "reason": struct.unpack_from("<H", payload, 20)[0],
        "target_id": struct.unpack_from("<I", payload, 8)[0],
    }


def print_assignment_ack(ack):
    result = MISSION_ASSIGN_RESULT_NAMES.get(ack["result"], f"UNKNOWN({ack['result']})")
    reason = MISSION_ASSIGN_REASON_NAMES.get(ack["reason"], f"UNKNOWN({ack['reason']})")
    availability = AVAILABILITY_NAMES.get(ack["availability"], f"UNKNOWN({ack['availability']})")
    print(
        "[ACK ] 65015 assignment_id={} target_id={} result={} reason={} "
        "mission_state=S{} availability={} aircraft={}/{}".format(
            ack["assignment_id"],
            ack["target_id"],
            result,
            reason,
            ack["mission_state"],
            availability,
            ack["aircraft_system"],
            ack["aircraft_component"],
        )
    )


def print_mission_status(status):
    availability = AVAILABILITY_NAMES.get(status["availability"], f"UNKNOWN({status['availability']})")
    print(
        "[STAT] 65013 v{} seq={} state=S{} control_source={} task_phase={} "
        "assignment_id={} target_id={} availability={} interceptor_type={} aircraft={}/{}".format(
            status["protocol_version"],
            status["sequence"],
            status["state"],
            status["control_source"],
            status["task_phase"],
            status["assignment_id"],
            status["target_id"],
            availability,
            status["interceptor_type"],
            status["aircraft_system"],
            status["aircraft_component"],
        )
    )


def status_signature(status):
    return (
        status["state"],
        status["assignment_id"],
        status["target_id"],
        status["availability"],
        status["control_source"],
        status["task_phase"],
    )


# ---------------------------------------------------------------------------
# 主循环
# ---------------------------------------------------------------------------
def handle_message(master, message, args, state):
    if message is None:
        return
    if message.get_type() == "TIMESYNC":
        target_system, target_component, has_targets = extract_timesync_targets(message)
        addressed_to_ground = (
            target_system == GROUND_SYSTEM_ID and target_component == GROUND_COMPONENT_ID
        )
        broadcast = target_system == 0 and target_component == 0
        if int(message.tc1) == 0 and (broadcast or addressed_to_ground):
            respond_timesync(master, message)
            state["timesync_responses"] += 1
            if args.debug_messages and state["timesync_responses"] in (1, 5, 10):
                print(f"[DEBUG] TIMESYNC 已响应 {state['timesync_responses']} 次")
        return

    v2 = extract_v2(message)
    if v2 is None:
        return
    if args.debug_messages:
        source = f"{message.get_srcSystem()}/{message.get_srcComponent()}"
        print(f"[DEBUG] RX V2_EXTENSION type={v2['type']} source={source}")

    ack = parse_assignment_ack(v2, args)
    if ack is not None:
        print_assignment_ack(ack)
        return

    target_ack = parse_target_ack(v2)
    if target_ack is not None:
        print(
            "[TACK] 65011 target_id={} result={} reason={}".format(
                target_ack["target_id"], target_ack["result"], target_ack["reason"]
            )
        )
        return

    status = parse_mission_status(v2)
    if status is not None:
        signature = status_signature(status)
        if signature != state["last_status"]:
            print_mission_status(status)
            state["last_status"] = signature
        return


def parse_args():
    parser = argparse.ArgumentParser(
        description="以地面站255/190身份发送65014任务分配并观察65015/65013，不直接发送控制命令"
    )
    parser.add_argument("--connection", default="udpout:192.168.144.12:19856",
                        help="pymavlink连接串；SITL常用 udpin:0.0.0.0:14550")
    parser.add_argument("--aircraft-system", type=int, default=1, help="目标飞机 system_id")
    parser.add_argument("--aircraft-component", type=int, default=25,
                        help="目标飞机 component_id（25=捕网，26=火箭）")
    parser.add_argument("--assignment-id", type=int, default=1, help="地面站分配编号")
    parser.add_argument("--target-id", type=int, default=7, help="指定目标编号（0非法）")
    parser.add_argument("--interceptor-type", type=int, default=25,
                        help="拦截类型，必须等于飞机component_id")
    parser.add_argument("--cancel", action="store_true", help="发送取消（flags.bit1）而非授权起飞")
    parser.add_argument("--cancel-after-ms", type=int, default=0,
                        help="运行N毫秒后把65014切为取消（起飞前/后取消场景）；0=不切换")
    parser.add_argument("--no-grant", action="store_true", help="不置flags.bit0（负向测试）")
    parser.add_argument("--valid-for-ms", type=int, default=5000, help="分配有效期")
    parser.add_argument("--zone-id", type=int, default=0,
                        help="65014 offset15 zone_id（0=未声明；非 0 时与机载缓存比对，见 docs/数据接口文档.md）")
    parser.add_argument("--zone-version", type=int, default=0,
                        help="65014 offset17 zone_version（0=未声明；与机载缓存不一致 → 65015 reason=21）")
    parser.add_argument("--duration", type=float, default=15.0, help="运行时长（秒）")
    parser.add_argument("--assignment-interval", type=float, default=2.0,
                        help="65014重发间隔；同一assignment_id重发是幂等的")
    parser.add_argument("--heartbeat-interval", type=float, default=1.0)
    parser.add_argument("--send-target", action="store_true", help="同时发送匹配target_id的65010目标")
    parser.add_argument("--stop-target-after-ms", type=int, default=0,
                        help="运行N毫秒后停止发送65010（目标丢失场景）；0=不停")
    parser.add_argument("--target-interval", type=float, default=0.5)
    parser.add_argument("--target-valid-for-ms", type=int, default=5000)
    parser.add_argument("--target-latitude", type=float, default=23.123)
    parser.add_argument("--target-longitude", type=float, default=113.123)
    parser.add_argument("--target-altitude-m", type=float, default=120.0)
    parser.add_argument("--target-velocity-north-mps", type=float, default=1.0)
    parser.add_argument("--target-velocity-east-mps", type=float, default=-0.5)
    parser.add_argument("--target-velocity-down-mps", type=float, default=0.0)
    parser.add_argument("--target-heading-deg", type=float, default=90.0)
    parser.add_argument("--target-horizontal-accuracy-m", type=float, default=1.5)
    parser.add_argument("--target-vertical-accuracy-m", type=float, default=2.5)
    parser.add_argument("--debug-messages", action="store_true",
                        help="打印TIMESYNC响应和V2_EXTENSION类型")
    return parser.parse_args()


def main():
    args = parse_args()
    if not 1 <= args.aircraft_system <= 254:
        raise ValueError("aircraft-system 必须在 1~254")
    if not 1 <= args.aircraft_component <= 255:
        raise ValueError("aircraft-component 必须在 1~255")
    if args.assignment_id <= 0 or args.target_id <= 0:
        raise ValueError("assignment-id 和 target-id 必须为正数")
    if args.valid_for_ms <= 0 or args.target_valid_for_ms <= 0:
        raise ValueError("valid-for-ms 必须为正数")
    if args.duration <= 0 or args.assignment_interval <= 0:
        raise ValueError("duration 和 assignment-interval 必须为正数")
    if args.send_target and args.target_interval <= 0:
        raise ValueError("target-interval 必须为正数")
    if args.cancel and args.no_grant:
        raise ValueError("cancel 与 no-grant 不能同时使用")
    if args.cancel_after_ms < 0 or args.stop_target_after_ms < 0:
        raise ValueError("cancel-after-ms 和 stop-target-after-ms 不能为负数")

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    flags = FLAG_CANCEL if args.cancel else (0 if args.no_grant else FLAG_TAKEOFF_AUTHORIZED)
    master = mavutil.mavlink_connection(
        args.connection,
        source_system=GROUND_SYSTEM_ID,
        source_component=GROUND_COMPONENT_ID,
    )
    state = {
        "timesync_responses": 0,
        "last_status": None,
    }
    boot_id = int(time.time()) & 0xFFFFFFFF
    target_seq = 1

    print(f"[INFO] 连接: {args.connection}")
    print(
        "[INFO] 地面站来源 255/190，目标飞机={}/{}，65014 类型={}，ACK={}，状态={}，不直接发送控制".format(
            args.aircraft_system,
            args.aircraft_component,
            MISSION_ASSIGN_TYPE,
            MISSION_ASSIGN_ACK_TYPE,
            MISSION_STATUS_TYPE,
        )
    )
    if args.send_target:
        print(
            "[INFO] 同时发送 65010：target_id={} pos={:.7f}/{:.7f} alt={:.1f}m".format(
                args.target_id,
                args.target_latitude,
                args.target_longitude,
                args.target_altitude_m,
            )
        )

    deadline = time.monotonic() + args.duration
    run_start = time.monotonic()
    last_heartbeat = 0.0
    last_assignment = 0.0
    last_target = 0.0
    assignment_sent = 0
    cancel_active = bool(args.cancel)
    target_active = bool(args.send_target)
    while running and time.monotonic() < deadline:
        now = time.monotonic()
        elapsed_ms = (now - run_start) * 1000.0
        if (not cancel_active and args.cancel_after_ms > 0 and
                elapsed_ms >= args.cancel_after_ms):
            cancel_active = True
            print("[INFO] 已到取消时刻，65014切换为取消（flags.bit1）", flush=True)
        if (target_active and args.stop_target_after_ms > 0 and
                elapsed_ms >= args.stop_target_after_ms):
            target_active = False
            print("[INFO] 已到停止目标时刻，停止发送65010（目标将超龄）", flush=True)
        current_flags = (FLAG_CANCEL if cancel_active else
                         (0 if args.no_grant else FLAG_TAKEOFF_AUTHORIZED))
        if now - last_heartbeat >= args.heartbeat_interval:
            send_heartbeat(master)
            last_heartbeat = now
        if now - last_assignment >= args.assignment_interval:
            send_assignment(master, args, current_flags)
            assignment_sent += 1
            last_assignment = now
        if target_active and now - last_target >= args.target_interval:
            send_target(master, args, boot_id, target_seq)
            target_seq += 1
            last_target = now
        message = master.recv_match(blocking=True, timeout=0.05)
        handle_message(master, message, args, state)

    print(
        "[INFO] 完成：65014 发送 {} 次，TIMESYNC 响应 {} 次".format(
            assignment_sent, state["timesync_responses"]
        )
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        sys.exit(1)
