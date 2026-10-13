# PX4 实链验证清单（真机，全程只读）

> 用途：PX4 已到位（2026-10-10：装机体 + GPS，可出 Home）后的实链验证，逐条打勾并记录日志。
> 定位：主任务《主程序集成与数据链路实施计划》阶段四的前置验证。
> **全程只读**：不绑定 `Px4Setpoint`、不发 ARM/模式切换/起飞/降落，`enable_control` 保持 false。
> **现场条件：室内（2026-10-10）**——室内可能拿不到 GPS fix / Home，见 §4：
> **室内无 Home 不是通信问题**，不要按串口故障排查。

## 0. 前置检查（上电前，一次性）

| # | 项 | 命令/期望 | 结论 |
|---|---|---|---|
| 1 | 接线：香橙派 UART6 ↔ PX4 TELEM，**`uart6-m1`**（与排针一致） | 见 `docs/部署与打包.md` §9.2 | |
| 2 | 权限：当前用户在 `dialout` 组 | `groups \| grep dialout` | |
| 3 | 串口未被系统控制台占用 | `sudo lsof /dev/ttyS1`；`systemctl status serial-getty@ttyS1` | |
| 4 | 配置：`px4.serial.device=/dev/ttyS1`、115200、`firmware_version` 与实际一致 | `config/config.json` | |
| 5 | 硬件安全：**拆螺旋桨**、TTL 电平兼容、共地 | 见 `tools/px4_link_smoke.md` §4.3 | |
| 6 | PX4 上电（USB 或电池），GPS 天线接好 | PX4 指示灯正常 | |

## 1. 第 1 步：纯链路冒烟（独立于主程序，推荐先做）

```bash
./build/px4_link_smoke --duration 30
echo "退出码=$?"
```

判读（完整列表见 `tools/px4_link_smoke.md` §5）：

| 项 | 期望 |
|---|---|
| `[PASS] PX4 HEARTBEAT` | 心跳建立 |
| `[PASS] 测试结束时心跳仍连接` | 链路稳定 |
| `[PASS] 遥测请求 COMMAND_ACK` | 主动请求被接受 |
| `[PASS] AUTOPILOT_VERSION 收到` | 固件版本可读 |
| 其他遥测（ATTITUDE 等） | 缺项只记 WARN，不判基础链路失败 |

不通过 → `tools/px4_link_smoke.md` §7 排查要点 + `docs/故障排查手册.md` 第 10 节。

## 2. 第 2 步：主程序实链（只读，逐条对照启动日志与摘要）

```bash
./build/drone_control
```

| # | 项 | 期望日志/现象 | 室内可达 |
|---|---|---|---|
| 1 | 串口打开 | `串口打开成功: device=/dev/ttyS1 baud=115200` | ✅ |
| 2 | 链路启动 | `PX4 通信启动: transport=serial:/dev/ttyS1@115200` | ✅ |
| 3 | 心跳 | `PX4 心跳已连接: source=1/1 mode=… armed=…` | ✅ |
| 4 | 固件版本 | `PX4 固件版本上报: reported=1.17.0`（与配置一致） | ✅ |
| 5 | 遥测快照 | 地面站下行有真实姿态/电池；`FlightStateSnapshot` 计数增长 | ✅（GPS 项除外） |
| 6 | 健康 | 无 `健康数据源超时: px4` | ✅ |
| 7 | 状态机 | `SELF_CHECK → READY`（需 GPS fix + Home 有效） | ⛔ 需室外 |
| 8 | TargetEstimator | `UpdateCount()` 增长、出有效 NED 影子状态（需 Home） | ⛔ 需室外 |
| 9 | 安全 | 日志无 ARM/模式/setpoint；`PX4 命令 ACK` 全部 `internal=true` | ✅ 必查 |

## 3. 安全红线（每次运行都确认）

- 螺旋桨已拆；
- 程序可见的主动内容只有 HEARTBEAT 与 `REQUEST_MESSAGE`/`SET_MESSAGE_INTERVAL`
  （日志中不应出现任何 setpoint、ARM、模式切换）；
- 异常时先 Ctrl+C（逆序停机），再按 §5 处置；不要在带桨状态下反复试。

## 4. 室内 / 室外判定（关键：别把无 Home 误判成通信问题）

```text
先看这一行是否出现：
  PX4 心跳已连接: source=1/1 …
  ├─ 出现 ⇒ 串口与 MAVLink 通信正常（链路层已通）
  │        若此时状态机仍停 SELF_CHECK / TargetEstimator 一直等待 Home：
  │        → 看 GPS fix 与 Home（室内属预期，不是通信问题）
  └─ 不出现 ⇒ 才是通信问题 → 故障排查手册第 10 节
```

| 现象 | 结论 | 动作 |
|---|---|---|
| 有心跳、无 GPS fix/Home（室内） | **预期** | 记录"室内限制"，Home 相关项留室外 |
| 有心跳、有 GPS fix、Home 不成立 | 查 GPS 质量/解锁状态/Home 设置条件 | 地面站核对 GPS 状态 |
| 无心跳 | 通信问题 | 接线/波特率/PX4 端口（`px4_link_smoke` §7） |
| 心跳断续 | 干扰/线缆/供电 | 查时序与线缆 |

**室外补齐项**（拿到 GPS fix 后）：`SELF_CHECK → READY`、`TargetEstimator` 出 NED 影子状态、
地面站显示 Home —— 成功后把结论回填 `docs/开发进度.md` 与主任务文档检查点。

**室外观察要点（2026-10-13 增补，厚云边缘收星实测后待对照）**：

- **Home 漂移对照（QGC vs 机载日志）**：QGC 与机载日志的 Home/全局位置同源，若 QGC 也看到
  位置游走，坐实为 PX4/GPS 侧行为；据此决定 Home 变化迟滞阈值（方案 B）是否实施。
  看机载日志 WARN 附带的 `gps_fix/卫星/eph/epv`（下行 GPS_RAW_INT 已透传真实值，
  Web/QGC 可直接看卫星数与精度）。
- **高度通道收敛性**：室内实测 Home 高度单调爬升 ≈0.12m/s（22 分钟 +8.7m 量级），
  室外稳定 fix 下观察是否收敛；不收敛则查气压计温漂，影响目标估计 D 轴基准。
- **25Hz 姿态/局部位置链路余量**：频率提升后确认无丢包/无 telem 超时告警
  （`px4_link_smoke` 或主程序日志的接收计数与 ACK 计数）。
- **Web 地图修复回归**：飞机标记应稳定显示在真实位置（不再漂到 (0,0)），
  刷新约 5Hz；轨迹线不应出现指向 Null Island 的长线。

## 5. 异常处置（指向，不在此重复）

| 现象 | 去处 |
|---|---|
| 串口打不开/被占用 | `docs/故障排查手册.md` 第 10.1/10.2 节 |
| 打开但无心跳 | 同上 10.3 + `tools/px4_link_smoke.md` §7 |
| 心跳超时告警 | `docs/故障排查手册.md` 第 14 节 |
| 状态机停留自检 | `docs/故障排查手册.md` 第 13 节 + §4 判定表 |
| 目标估计等待 Home | `docs/故障排查手册.md` 第 12 节 |

## 6. 结果记录（回填用）

| 日期 | 环境 | 冒烟 | 串口 | 心跳 | 固件 | 遥测 | 状态机 | NED 影子 | 备注 |
|---|---|---|---|---|---|---|---|---|---|
| 2026-10-10 | 室内 | 未跑（直接跑主程序） | ✅ | ✅ | ✅ | ✅ | ⛔ 待室外 | ⛔ 待室外 | 日志 `0028_20261010_155340.log`；心跳 `source=1/1 mode=4/3 armed=false`；一条 512 请求被拒（result=2）——与 AUTOPILOT_VERSION 成功对照，应为 HOME_POSITION(242) 因无 Home 被拒，属室内预期 |
