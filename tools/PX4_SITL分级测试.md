# PX4 1.17.0 SITL 分级测试

## 1. 目标与安全边界

在真实 Pixhawk 拆桨测试前，通过 PX4 SITL 验证 `Px4Link` 的 UDP 通信和
`SET_POSITION_TARGET_LOCAL_NED` 编码。

当前只开放第 1 阶段：

```text
UDP 连接 → 心跳/遥测 → 持续发送零 NED 速度 → 不切 Offboard → 不解锁
```

程序强制要求 `transport=udp` 才接受 `--sitl-zero-velocity`，因此该参数无法误用于香橙派
`/dev/ttyS1` 真实串口。

## 2. 配置

使用 `config/px4_sitl.json`：

```json
"transport": "udp",
"udp": {
    "bind_address": "127.0.0.1",
    "bind_port": 14540,
    "remote_address": "127.0.0.1",
    "remote_port": 14580
}
```

常规硬件配置 `config/config.json` 保持 `transport="serial"`，两者禁止混用。

## 3. 启动 PX4 SITL

在 PX4-Autopilot v1.17.0 仓库中启动适合当前机型的仿真目标。多旋翼初测可使用官方 x500：

```bash
make px4_sitl gz_x500
```

等待 PX4 shell 和仿真器稳定。若本机没有该仿真环境，先按 PX4 1.17 官方文档安装依赖，
不要改用真实飞控代替此阶段。

在 PX4 shell 中检查：

```text
mavlink status
```

应存在向 UDP 14540 发送的 Onboard/API 链路。若端口不是 14540/14580，以实际 status 为准修改
`config/px4_sitl.json`，不得猜测。

## 4. 第 0 阶段：只读 UDP 链路

另开终端，在本工程执行：

```bash
cmake -S . -B build
cmake --build build --target px4_link_smoke -j$(nproc)
./build/px4_link_smoke \
  --config config/px4_sitl.json \
  --duration 10
```

期望：

- HEARTBEAT PASS；
- 固件版本/遥测可接收；
- ACK 无超时；
- 退出码 0。

### 4.1 第 0 阶段实测结果（2026-08-27）

```text
时长：10 秒
总接收：2665
ACK：10，超时 0
链路错误：0
版本/落地/全局位置/局部位置/姿态/GPS/电池/Home：PASS
RC：未收到（SITL 未注入 RC）
```

UDP 14540/14580、PX4 1.17.0 版本和完整仿真遥测均验证成功，第 0 阶段通过。

## 5. 第 1 阶段：零速度流，不切模式

确认第 0 阶段通过后执行：

```bash
./build/px4_link_smoke \
  --config config/px4_sitl.json \
  --duration 10 \
  --sitl-zero-velocity
```

程序行为：

- 以 10Hz 发布 `Px4Setpoint` 零 NED 速度；
- `Px4Link` 以配置的 20Hz 发送 `SET_POSITION_TARGET_LOCAL_NED`；
- frame=`MAV_FRAME_LOCAL_NED`；
- 位置/加速度/yaw/yaw rate 全部 ignore，仅 vx/vy/vz 生效；
- 不发送模式命令，不解锁；
- 结束前发布 `valid=false` 并停止设定值流。

最终应出现：

```text
[PASS] SITL 零速度设定值已发送
设定值发送数: 约 180~200（连接建立时间会造成少量差异）
结果: 基础 PX4 链路通过
```

### 5.1 第 1 阶段应用侧实测结果（2026-08-27）

```text
时长：10 秒
SET_POSITION_TARGET_LOCAL_NED 发送：172 条
模式：4/3 AUTO/LOITER，全程未变化
armed：false
接收目标消息：2600
ACK：10，超时 0
链路错误：0
```

应用侧已通过；还需完成下面的 PX4 内部 topic 确认，才算第 1 阶段完整通过。

## 6. PX4 侧确认

测试运行期间在 PX4 shell 中检查：

```text
mavlink status
listener trajectory_setpoint 1
```

预期 `trajectory_setpoint` 速度为零或接近零。若当前 PX4 版本的内部 topic 名称不同，以
PX4 shell `listener`/`uorb top` 实际输出为准并记录，不凭记忆修改代码。

飞行模式应保持原模式，不能因为第 1 阶段自动进入 Offboard。

## 7. 验收与后续阶段

第 1 阶段验收条件：

- UDP 心跳稳定；
- `SetpointSendCount > 0`；
- PX4 侧确认收到零速度 target；
- 未切换 Offboard；
- 未解锁；
- 停止后设定值不再增长；
- 无链路错误。

第 1 阶段 PX4 侧已确认新鲜 `offboard_control_mode`：`velocity=true`，其余控制层级均 false。
`trajectory_setpoint` 被当前 AUTO/LOITER 发布器覆盖，读取到的是任务轨迹；这不影响 MAVLink
输入类型确认，数值将在进入 Offboard 后由该模式消费时验证。

## 7. 第 2 阶段：切换 Offboard，但保持 disarmed

第 1 阶段通过后执行：

```bash
./build/px4_link_smoke \
  --config config/px4_sitl.json \
  --duration 15 \
  --sitl-offboard-disarmed
```

程序先持续发送零 NED 速度至少 2 秒，再通过 `MAV_CMD_DO_SET_MODE` 请求 PX4 main mode 6
（OFFBOARD）。安全门禁：仅 UDP 可用，程序不发送 ARM 命令，并持续检查 `armed=false`。

验收：

- 模式从 `4/3` 变为 `6/0`；
- 最终报告 `PX4 已进入 Offboard=PASS`；
- 全程 `armed=否`；
- `trajectory_setpoint.velocity=[0,0,0]`；
- ACK 无超时、链路无错误。

### 7.1 第 2 阶段实测结果（2026-08-27）

PX4 内部确认：

```text
offboard_control_mode: velocity=true，其余控制层级=false
trajectory_setpoint:
  position=[nan,nan,nan]
  velocity=[0,0,0]
  acceleration=[nan,nan,nan]
  yaw/yawspeed=nan
```

50 秒应用结果：

```text
模式：4/3 → 6/0 OFFBOARD
MAV_CMD_DO_SET_MODE ACK：command=176 result=0
设定值：876 条
armed：全程 false
总接收：12871
ACK：11，超时 0
链路错误：0
```

第 2 阶段完整通过。

## 8. 第 3A 阶段：SITL 解锁，始终零速度

第 2 阶段通过后执行：

```bash
./build/px4_link_smoke \
  --config config/px4_sitl.json \
  --duration 20 \
  --sitl-arm-zero-velocity
```

程序顺序：

1. 零 NED 速度预发送至少 2 秒；
2. 请求并确认 OFFBOARD 6/0；
3. 仅在 connected、landed、local position、attitude 均有效时发送 ARM；
4. 以 heartbeat `armed=true` 确认解锁，不以 ACK 代替状态；
5. 全程持续零速度，不发送位置/非零速度/起飞；
6. 记录解锁期间三维最大位移；
7. 测试结束前发送 DISARM，继续零速度并等待 `armed=false`；
8. 确认上锁后停止设定值和链路。

安全门禁：该参数只允许 `transport=udp`，最短时长 10 秒，不能与其他 SITL 阶段参数共用。
串口配置会在启动前被拒绝。

验收报告必须全部 PASS：

```text
Offboard 模式请求已入队
PX4 已进入 Offboard
ARM 请求已入队
PX4 已确认 armed
DISARM 请求已入队
PX4 已确认 disarmed
```

如果 ARM 被拒绝，检查最近 `command=400` ACK 和 PX4 preflight 日志，不得使用 force arm 绕过检查。

第 3B 阶段才允许在 SITL 使用最大 0.5m/s、2 秒的小幅水平速度。任何阶段未通过都不得进入
真实 Pixhawk 控制测试。

## 8. 常见问题

| 现象 | 排查 |
|---|---|
| UDP bind 14540 失败 | QGC/MAVSDK/旧进程占用，执行 `ss -lunp | grep 14540` |
| 一直 connected=否 | 检查 SITL `mavlink status` 的远端端口 |
| 有心跳但发送数为 0 | 是否遗漏 SITL 阶段参数，或配置误用 serial |
| Offboard 请求被拒绝 | 设定值预发送不足、local position 无效、模式命令 ACK 非 accepted |
| PX4 没有 target | remote_port 错误、type mask/消息未到，抓取 `mavlink status` |
| 程序拒绝启动 | 安全保护检测到 `--sitl-zero-velocity` 配合了非 UDP 配置 |
