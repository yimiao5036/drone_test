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

通过后再实现第 2 阶段：持续预发送设定值至少 1 秒后请求 Offboard，但仍保持 disarmed；
第 3 阶段才在 SITL 解锁并验证位置/速度效果与 Offboard 丢失保护。任何阶段未通过都不得进入
真实 Pixhawk 控制测试。

## 8. 常见问题

| 现象 | 排查 |
|---|---|
| UDP bind 14540 失败 | QGC/MAVSDK/旧进程占用，执行 `ss -lunp | grep 14540` |
| 一直 connected=否 | 检查 SITL `mavlink status` 的远端端口 |
| 有心跳但发送数为 0 | 是否遗漏 `--sitl-zero-velocity`，或配置误用 serial |
| PX4 没有 target | remote_port 错误、type mask/消息未到，抓取 `mavlink status` |
| 程序拒绝启动 | 安全保护检测到 `--sitl-zero-velocity` 配合了非 UDP 配置 |
