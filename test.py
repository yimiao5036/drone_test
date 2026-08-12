#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机载电脑（算力板）飞行控制模拟脚本 —— PX4 SITL 仿真
使用 pymavlink 连接 SITL (udp:127.0.0.1:14540)

飞行流程（对应《状态机设计.md》S3~S14）：
  1. 解锁 (ARMING)
  2. 起飞到指定高度 (TAKEOFF)
  3. 到达高度后先创建流：启动 OFFBOARD 设定值发送线程（原地悬停）
  4. 进入机载模式 (OFFBOARD)
  5. 逐任务点接近（任务点以经纬度记录）：
     - 距离远（> 切换阈值）：位置控制
     - 距离近（<= 切换阈值）：速度方向控制（速度矢量指向目标）
     - 到达目标点后悬停 30s
  6. 飞回起飞点（算力板手动控制，不使用 RTL 返航模式）
  7. 到达起飞点后降落 (LAND) 并上锁
"""

import time
import math
import threading
from pymavlink import mavutil

# ==================== 任务点数据结构 ====================
class TaskPoint:
    """任务点数据结构：以经纬度(WGS84)记录目标位置，高度为相对起飞点的相对高度"""
    def __init__(self, name, lat, lon, rel_alt):
        self.name = name        # 任务点名称（仅用于日志显示）
        self.lat = lat          # 纬度（度）
        self.lon = lon          # 经度（度）
        self.rel_alt = rel_alt  # 相对高度（米，正向上）

    def __str__(self):
        return f"{self.name}(lat={self.lat:.6f}, lon={self.lon:.6f}, alt={self.rel_alt}m)"


# ==================== 配置区 ====================
CONNECTION_STRING = 'udp:127.0.0.1:14540'   # SITL 连接地址
TAKEOFF_ALT = 5.0                            # 起飞/巡航高度（米，相对高度，正向上）
REACH_THRESHOLD = 1.0                        # 到达判定距离阈值（米，3D）
POS_VEL_SWITCH_DIST = 5.0                    # 位置控制 -> 速度方向控制的切换距离（米）
POS_CTRL_STEP = 4.0                          # 位置控制的引导步长（米）：每帧下发"当前位置指向目标
                                             # 方向前方 POS_CTRL_STEP 米"的中间点，而非直接下发最终
                                             # 目标点。目标移动时引导点自动跟随；避障时只需在
                                             # 引导点生成处替换为绕行点（应小于 POS_VEL_SWITCH_DIST）
APPROACH_SPEED = 1.5                         # 速度方向控制时的接近速率（m/s）
HOVER_TIME = 30.0                            # 到达目标点后的悬停时间（秒）
SETPOINT_RATE = 20                           # OFFBOARD setpoint 发送频率（Hz）
LAND_STABLE_TIME = 3.0                       # 回起飞点后、降落前的稳定悬停时间（秒）

# 任务点列表（记录经纬度）
# 示例为 SITL 默认起飞点 (47.397742, 8.545594) 附近约 20m 处的 3 个点：
#   1 度纬度 ≈ 111320m，1 度经度 ≈ 111320 * cos(纬度) m
# 如需修改，请替换为实际目标点的经纬度
TASK_POINTS = [
    TaskPoint("任务点1-正北20m", 47.397922, 8.545594, TAKEOFF_ALT),
    TaskPoint("任务点2-正东20m", 47.397742, 8.545860, TAKEOFF_ALT),
    TaskPoint("任务点3-东北20m", 47.397922, 8.545860, TAKEOFF_ALT),
]
# ================================================


class VehicleState:
    """飞机状态存储（由后台接收线程实时更新）"""
    def __init__(self):
        self.connected = False
        self.armed = False
        self.mode = "UNKNOWN"
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw = 0.0
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.relative_alt = 0
        self.battery_voltage = 0.0
        self.battery_remaining = 0

    def update_from_msg(self, msg):
        """从 MAVLink 消息更新状态"""
        msg_type = msg.get_type()
        if msg_type == 'HEARTBEAT':
            self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.mode = mavutil.mode_string_v10(msg)
        elif msg_type == 'LOCAL_POSITION_NED':
            # NED 坐标：x=北, y=东, z=向下（负值代表向上），单位：米
            self.x = msg.x
            self.y = msg.y
            self.z = msg.z
            self.vx = msg.vx
            self.vy = msg.vy
            self.vz = msg.vz
        elif msg_type == 'GLOBAL_POSITION_INT':
            # 经纬度单位为 1e-7 度（int32）
            self.lat = msg.lat
            self.lon = msg.lon
            self.alt = msg.alt
            self.relative_alt = msg.relative_alt
        elif msg_type == 'ATTITUDE':
            self.yaw = msg.yaw
        elif msg_type == 'SYS_STATUS':
            self.battery_voltage = msg.voltage_battery / 1000.0
            self.battery_remaining = msg.battery_remaining

    def __str__(self):
        return (f"[State] Mode={self.mode} Armed={self.armed} "
                f"Pos=({self.x:.2f}, {self.y:.2f}, {self.z:.2f}) "
                f"Bat={self.battery_voltage:.2f}V")


def connect_vehicle(conn_str):
    """连接飞机并等待心跳"""
    print(f"[*] 正在连接: {conn_str}")
    master = mavutil.mavlink_connection(conn_str)
    master.wait_heartbeat()
    print(f"[+] 已连接，系统ID: {master.target_system}, 组件ID: {master.target_component}")
    return master


def heartbeat_thread(master):
    """后台心跳线程，保持与飞控的连接"""
    while True:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        time.sleep(1)


def request_data_stream(master):
    """请求飞控发送遥测数据流（位置、姿态、电池等）"""
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,  # 10Hz
        1
    )
    print("[+] 已请求数据流 10Hz")


def set_mode(master, mode_name):
    """
    切换飞行模式（PX4 1.11）
    注意：pymavlink 的 mode_mapping() 对 PX4 返回三元组 (base_mode, main_mode, sub_mode)，
    必须用 MAV_CMD_DO_SET_MODE 命令分别传递这三个字段；
    直接发 SET_MODE 消息在 PX4 1.11 上不生效
    """
    mode_map = master.mode_mapping()
    if mode_map is None or mode_name not in mode_map:
        print(f"[!] 未知模式: {mode_name}")
        return False
    base_mode, main_mode, sub_mode = mode_map[mode_name]
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        base_mode, main_mode, sub_mode,
        0, 0, 0, 0
    )
    print(f"[*] 正在切换模式 -> {mode_name} ...")
    return True


def arm_disarm(master, arm=True):
    """解锁/上锁"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1 if arm else 0,
        0, 0, 0, 0, 0, 0
    )
    action = "解锁" if arm else "上锁"
    print(f"[*] 正在执行: {action}")


def takeoff(master, rel_alt, home_alt):
    """
    起飞到指定相对高度（rel_alt 正向上，单位米）
    PX4 1.11 对 MAV_CMD_NAV_TAKEOFF 的参数要求（踩坑总结）：
      - param1~param3  : 未使用，传 NaN
      - param4 (yaw)   : 未指定时传 NaN（传 0 会被当作 0° 航向，可接受但不推荐）
      - param5/param6  : 目标经纬度，未指定时必须传 NaN！
                        传 0.0 会被 PX4 当作有效坐标 (0°N, 0°E)，起飞目标点跑到几内亚湾，
                        飞机会一直不起飞（QGC 传 NaN 所以正常）
      - param7 (alt)   : 目标绝对海拔（AMSL），必须 = 起飞点海拔 + 相对高度；
                        传相对高度会被钳位到 MIS_TAKEOFF_ALT 最低起飞高度
    """
    nan = float('nan')
    print(f"[*] 起飞到相对高度 {rel_alt:.1f}m (AMSL {home_alt + rel_alt:.1f}m) ...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        nan, nan, nan,          # param1-3: 未指定
        nan, nan, nan,          # param4-6: yaw/经纬度未指定（必须传 NaN）
        home_alt + rel_alt      # param7: 目标绝对海拔（AMSL）
    )


def latlon_to_ned_offset(lat, lon, lat0, lon0):
    """
    经纬度差 -> 局部 NED 偏移（x=北, y=东）
    等距圆柱投影近似，几公里内精度足够，用于把经纬度任务点换算为本地坐标
    """
    EARTH_RADIUS = 6371000.0        # 地球平均半径（米）
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = EARTH_RADIUS * dlat                                       # 北向偏移（米）
    y = EARTH_RADIUS * dlon * math.cos(math.radians(lat0))        # 东向偏移（米）
    return x, y


def send_position_target_ned(master, x, y, z, yaw=0.0):
    """
    发送位置目标（位置控制）：只使用 x/y/z/yaw，忽略速度与加速度
    注意：所有字段必须传有限值！PX4 1.11 会检查全部 10 个字段是否有限，
    任何 NaN（即使被 type_mask 声明忽略，如 yaw）都会导致 offboard 控制信号
    被判为无效，从而无法进入 OFFBOARD 模式
    """
    # 类型掩码：bit 置 1 表示忽略该项；此处仅控制位置 + yaw
    # 0b0000111111111000 -> 使用 x,y,z,yaw
    type_mask = 0b0000111111111000
    master.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (0=立即)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,                    # 位置
        0, 0, 0,                    # 速度（忽略，但传有限值 0）
        0, 0, 0,                    # 加速度（忽略，但传有限值 0）
        yaw, 0                      # yaw, yaw_rate（均须有限）
    )


def send_velocity_target_ned(master, vx, vy, vz, yaw=0.0):
    """
    发送速度目标（速度方向控制）：只使用 vx/vy/vz + yaw，忽略位置与加速度
    yaw 用于让机头时刻正对飞行方向
    注意：同 send_position_target_ned，所有字段必须传有限值（yaw 不能传 NaN）
    """
    # 类型掩码：0b0000111111000111 -> 使用 vx,vy,vz,yaw（bit 3/4/5 和 bit 10 置 0）
    type_mask = 0b0000111111000111
    master.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (0=立即)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0,                    # 位置（忽略，但传有限值 0）
        vx, vy, vz,                 # 速度
        0, 0, 0,                    # 加速度（忽略，但传有限值 0）
        yaw, 0                      # yaw, yaw_rate（均须有限）
    )


def distance_3d(x1, y1, z1, x2, y2, z2):
    """计算两点三维距离"""
    return math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)


def direction_velocity(state, tx, ty, tz):
    """
    速度方向控制：速度矢量 = 指向目标的单位方向 × 接近速率
    垂直分量随高度误差自动比例修正（高度差越大 vz 越大），保证高度平稳
    """
    dx, dy, dz = tx - state.x, ty - state.y, tz - state.z
    d = math.sqrt(dx*dx + dy*dy + dz*dz)
    if d < 1e-6:                    # 已在目标点，防止除零
        return 0.0, 0.0, 0.0
    return (APPROACH_SPEED * dx / d,
            APPROACH_SPEED * dy / d,
            APPROACH_SPEED * dz / d)


def guidance_point(state, tx, ty, tz):
    """
    动态位置引导点：当前位置沿"指向目标"方向前进 POS_CTRL_STEP 米的位置
    位置控制不直接下发最终目标点，而是每帧用最新位置+目标重新生成引导点：
      - 目标点移动时，引导点自动跟随新目标
      - 需要避障时，只需在此处把引导点替换为绕行点（改返回值即可，不动其他逻辑）
    """
    dx, dy, dz = tx - state.x, ty - state.y, tz - state.z
    d = math.sqrt(dx*dx + dy*dy + dz*dz)
    if d < 1e-6:                    # 已在目标点，防止除零
        return tx, ty, tz
    step = min(POS_CTRL_STEP, d)    # 距目标不足一步时直接落到目标上
    return (state.x + POS_CTRL_STEP * dx / d,
            state.y + POS_CTRL_STEP * dy / d,
            state.z + POS_CTRL_STEP * dz / d)


def heading_to_target(state, tx, ty):
    """机头航向（NED 偏航角）：指向目标方向。x=北, y=东, atan2(东, 北)"""
    return math.atan2(ty - state.y, tx - state.x)


def wait_for_mode(master, state, target_mode, timeout=10):
    """等待模式切换完成"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if state.mode == target_mode:
            print(f"[+] 模式已切换为: {target_mode}")
            return True
        time.sleep(0.2)
    print(f"[!] 模式切换超时，当前模式: {state.mode}")
    return False


def wait_armed(master, state, armed=True, timeout=10):
    """等待解锁/上锁状态"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if state.armed == armed:
            status = "已解锁" if armed else "已上锁"
            print(f"[+] {status}")
            return True
        time.sleep(0.2)
    print(f"[!] 等待{'解锁' if armed else '上锁'}超时")
    return False


def wait_altitude(master, state, rel_alt, timeout=30):
    """等待飞机到达指定相对高度（容忍 0.5m 误差）"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        # state.z 为 NED（负值向上），与相对高度相差一个负号
        if abs(state.z - (-rel_alt)) < 0.5:
            print(f"[+] 已到达相对高度 {rel_alt:.1f}m")
            return True
        time.sleep(0.2)
    print(f"[!] 等待高度超时，当前 NED z = {state.z:.2f}m")
    return False


def offboard_setpoint_thread(master, setpoint_ref):
    """
    OFFBOARD 设定值后台发送线程 —— 即"创建流"
    进入机载模式前必须先启动本线程并持续发送（>2Hz），否则 PX4 会退出 OFFBOARD
    setpoint_ref: dict，'mode' 决定控制方式：
        'position' -> 位置控制（取 x/y/z，yaw 取机头航向）
        'velocity' -> 速度方向控制（取 vx/vy/vz，yaw 取机头航向）
    """
    rate = 1.0 / SETPOINT_RATE
    while True:
        sp = setpoint_ref
        if sp.get('mode') == 'velocity':
            send_velocity_target_ned(master, sp['vx'], sp['vy'], sp['vz'], sp.get('yaw', 0.0))
        else:
            send_position_target_ned(master, sp['x'], sp['y'], sp['z'], sp.get('yaw', 0.0))
        time.sleep(rate)


def approach_point(master, state, setpoint_ref, tx, ty, tz, label):
    """
    控制飞机接近目标点，直到进入到达阈值
    控制策略（对应状态机 GPS_APPROACH -> VISUAL_TRACKING）：
      距离 >  POS_VEL_SWITCH_DIST : 位置控制（动态中间引导点）
      距离 <= POS_VEL_SWITCH_DIST : 速度方向控制
      距离 <  REACH_THRESHOLD    : 判定到达，转为位置保持
    说明：
      - 位置控制不直接下发最终目标点，而是每帧生成"前方 POS_CTRL_STEP 米"的
        引导点（guidance_point），目标移动时自动跟随、避障时可直接替换引导点
      - 机头航向 yaw 每帧指向目标方向，保证飞行时机头正对飞行方向
    """
    while True:
        dist = distance_3d(state.x, state.y, state.z, tx, ty, tz)

        if dist > POS_VEL_SWITCH_DIST:
            # 远距离：位置控制，下发动态中间引导点（而非最终目标点）
            gx, gy, gz = guidance_point(state, tx, ty, tz)
            setpoint_ref['mode'] = 'position'
            setpoint_ref['x'], setpoint_ref['y'], setpoint_ref['z'] = gx, gy, gz
            setpoint_ref['yaw'] = heading_to_target(state, tx, ty)
        elif dist > REACH_THRESHOLD:
            # 近距离：速度方向控制，速度矢量指向目标
            vx, vy, vz = direction_velocity(state, tx, ty, tz)
            setpoint_ref['mode'] = 'velocity'
            setpoint_ref['vx'], setpoint_ref['vy'], setpoint_ref['vz'] = vx, vy, vz
            setpoint_ref['yaw'] = heading_to_target(state, tx, ty)
        else:
            # 到达：保持位置悬停（后续由调用方执行悬停计时）
            setpoint_ref['mode'] = 'position'
            setpoint_ref['x'], setpoint_ref['y'], setpoint_ref['z'] = tx, ty, tz
            setpoint_ref['yaw'] = heading_to_target(state, tx, ty)
            print(f"[+] 已到达 {label}")
            return True

        control = "位置控制" if dist > POS_VEL_SWITCH_DIST else "速度方向控制"
        print(f"    {label} 距离: {dist:.2f}m | {control}")
        time.sleep(0.2)


def hover_at(state, setpoint_ref, hx, hy, hz, seconds, label):
    """在指定点位置保持悬停 seconds 秒（持续发送位置设定值）"""
    setpoint_ref['mode'] = 'position'
    setpoint_ref['x'], setpoint_ref['y'], setpoint_ref['z'] = hx, hy, hz
    t0 = time.time()
    while time.time() - t0 < seconds:
        remain = seconds - (time.time() - t0)
        print(f"    {label}悬停中 位置: ({state.x:.2f}, {state.y:.2f}, {state.z:.2f}) "
              f"剩余: {remain:.0f}s")
        time.sleep(1)


def main():
    # ---- 1. 连接 SITL ----
    master = connect_vehicle(CONNECTION_STRING)
    state = VehicleState()

    # ---- 2. 启动心跳线程 ----
    hb_thread = threading.Thread(target=heartbeat_thread, args=(master,), daemon=True)
    hb_thread.start()

    # ---- 3. 请求遥测数据流 ----
    request_data_stream(master)

    # ---- 4. 启动状态接收线程 ----
    def receive_thread():
        while True:
            msg = master.recv_match(blocking=True, timeout=1)
            if msg:
                state.update_from_msg(msg)

    rx_thread = threading.Thread(target=receive_thread, daemon=True)
    rx_thread.start()

    # 等待状态初始化
    print("[*] 等待状态初始化...")
    time.sleep(2)
    print(f"[+] 当前状态: {state}")

    # 记录起飞点：NED 原点 + 经纬度 + 海拔（作为任务点坐标换算基准和返航目标）
    home_x, home_y, home_z = state.x, state.y, state.z
    home_lat, home_lon = state.lat / 1e7, state.lon / 1e7   # GLOBAL_POSITION_INT 单位为 1e-7 度
    home_alt = state.alt / 1000.0                            # AMSL 海拔（mm -> m）
    print(f"[+] 起飞点: NED({home_x:.2f}, {home_y:.2f}, {home_z:.2f}) "
          f"经纬度({home_lat:.6f}, {home_lon:.6f}) 海拔({home_alt:.1f}m)")

    # ---- 5. 解锁 (ARMING) ----
    arm_disarm(master, arm=True)
    if not wait_armed(master, state, armed=True):
        print("[!] 解锁失败，退出")
        return

    # ---- 6. 起飞 (TAKEOFF) ----
    takeoff(master, rel_alt=TAKEOFF_ALT, home_alt=home_alt)
    if not wait_altitude(master, state, TAKEOFF_ALT):
        print("[!] 起飞未到达指定高度，退出")
        return
    print(f"[+] 起飞完成 当前: ({state.x:.2f}, {state.y:.2f}, {state.z:.2f})")

    # ---- 7. 到达高度后先创建流：启动 OFFBOARD 设定值发送线程（原地悬停） ----
    current_setpoint = {
        'mode': 'position',
        'x': home_x, 'y': home_y, 'z': -TAKEOFF_ALT,  # 悬停于起飞点上方巡航高度
        'yaw': 0.0,
    }
    sp_thread = threading.Thread(
        target=offboard_setpoint_thread, args=(master, current_setpoint), daemon=True)
    sp_thread.start()
    time.sleep(1.0)   # 先持续发送 1s 设定值，满足 PX4 进入 OFFBOARD 的前置条件

    # ---- 8. 进入机载模式 (OFFBOARD) ----
    set_mode(master, 'OFFBOARD')
    if not wait_for_mode(master, state, 'OFFBOARD'):
        print("[!] 进入 OFFBOARD 失败，退出")
        return
    print("[+] 已成功进入机载模式 (OFFBOARD)")

    # ---- 9. 逐任务点接近：远=位置控制 / 近=速度方向控制 ----
    for idx, task in enumerate(TASK_POINTS, 1):
        # 任务点经纬度 -> 局部 NED 坐标（以起飞点经纬度为原点）
        tx, ty = latlon_to_ned_offset(task.lat, task.lon, home_lat, home_lon)
        tz = -task.rel_alt   # 相对高度 -> NED z
        print(f"\n=== 任务点 {idx}/{len(TASK_POINTS)}: {task} -> "
              f"NED({tx:.2f}, {ty:.2f}, {tz:.2f}) ===")

        # 9.1 接近目标点（位置/速度方向双模式控制）
        approach_point(master, state, current_setpoint, tx, ty, tz, f"任务点{idx}")

        # 9.2 到达目标点后悬停 HOVER_TIME 秒
        print(f"[*] 任务点{idx}悬停 {HOVER_TIME}s ...")
        hover_at(state, current_setpoint, tx, ty, tz, HOVER_TIME, f"任务点{idx}")

    # ---- 10. 飞回起飞点（算力板手动控制，不使用 RTL 返航模式） ----
    print(f"\n=== 所有任务点完成，飞回起飞点 NED({home_x:.2f}, {home_y:.2f}, {-TAKEOFF_ALT:.2f}) ===")
    approach_point(master, state, current_setpoint, home_x, home_y, -TAKEOFF_ALT, "起飞点")

    # ---- 11. 降落前稳定悬停 ----
    print(f"[*] 降落前稳定悬停 {LAND_STABLE_TIME}s ...")
    hover_at(state, current_setpoint, home_x, home_y, -TAKEOFF_ALT, LAND_STABLE_TIME, "起飞点")

    # ---- 12. 降落 (LAND) ----
    print("[*] 开始降落...")
    set_mode(master, 'LAND')
    if not wait_for_mode(master, state, 'LAND'):
        print("[!] 进入 LAND 失败，退出")
        return

    # 等待落地：NED z 回到地面高度附近，或 PX4 落地后自动上锁（任一项即完成）
    t0 = time.time()
    while time.time() - t0 < 30:
        if state.z > home_z - 0.3 or not state.armed:
            break
        print(f"    高度: {state.z:.2f}m，等待降落...")
        time.sleep(0.5)
    print("[+] 降落完成")

    # ---- 13. 上锁 (DISARMED) ----
    arm_disarm(master, arm=False)
    wait_armed(master, state, armed=False)

    print("\n[✓] 任务全部完成！")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n[!] 用户中断")
