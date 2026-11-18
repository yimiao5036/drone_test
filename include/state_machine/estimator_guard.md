# 估计器健康判据（estimator_guard）

> 对应实现：`include/state_machine/estimator_guard.h`、`src/state_machine/estimator_guard.cpp`
> 使用方：`MissionStateMachine`（P3 起已接线：判据命中 → 切 POSCTL 悬停 + 停发意图 + 冻结任务）
> 设计文档：`docs/降落与估计器保护设计-2026-11-17.md` §3.3/§5
> **接线状态（2026-11-17，P3）**：**已接线**——`MissionStateMachine::Evaluate()` 每决策拍
> （100 ms）调用 `Update()`，命中即锁存降级并由状态机处置（切 `POSCTL(3/0)` 悬停、
> 停发一切控制意图、冻结任务推进，设计 §5/§5.6）。判据模块本身仍只回答"能不能信"：
> 不切模式、不发命令、不打日志。

## 1. 功能职责

只回答一个问题：**这一帧 PX4 快照里的位置/高度估计还能不能信。**

| 做 | 不做 |
|---|---|
| 从一帧 `FlightStateSnapshot` 里判跳变、比值、健康位、气压交叉校验 | 切模式、发 `COMMAND_LONG`、写日志、访问 Topic |
| 维护滞回计时（持续型判据要满足 `*_hold_ms`） | 决定处置动作（POSCTL/返航/降落由状态机定） |
| 首次命中后**锁存**，`Reset()` 才清除 | 自动恢复：好转也不解除，避免抖动下反复切模式 |
| 给出首个命中的原因与量值（日志/告警用） | 判断"该不该起飞"（那是起飞前门禁的事） |

线程安全：无内部同步，只能在状态机决策线程内调用。所有接口不抛异常。

**与处置的分工（2026-11-17 P3 起已接线）**：`MissionStateMachine::Evaluate()` 每决策拍
（100 ms）喂一帧快照；判据命中后由状态机**锁存**并发 `DO_SET_MODE(POSCTL 3/0)`、停发
一切控制意图、冻结任务推进（不返航/不降落/不上锁/不重试切模式）。本模块不决定处置、
不持有命令接口，也不感知 `enable_control`——只读运行时由状态机按"命令接口是否注入"
决定只告警。

## 2. 接口与数据流

```cpp
EstimatorGuard guard;                 // 阈值由构造注入（estimator_guard 节，13 键）
guard.Update(snapshot, now_ms, armed);// 状态机每决策拍（100 ms）调一次
guard.result();                       // { degraded, issue, since_ms, magnitude }
guard.Thresholds();                   // 生效阈值（只读；critical_sensor_mask 已归一）
guard.Reset();                        // 重置任务状态 / 程序重启时清锁存
```

输入只有快照与 `now_ms`、`armed`；不额外持有时间戳表（`LOCAL_POSITION_NED` 的
到达时刻由 `px4_link` 写进快照的 `local_position_time_ms`）。

**阈值来源（2026-11-17 P3 起外置）**：`config/config.json` 的 `estimator_guard` 节
13 键——`enabled` + `z_jump_m`、`z_jump_speed_margin_m`、`xy_jump_m`、
`xy_jump_speed_margin_m`、`ratio_limit`、`ratio_hold_ms`、`baro_divergence_m`、
`baro_hold_ms`、`sensor_hold_ms`、`local_position_stale_ms`、`critical_sensor_mask`、
`bad_estimator_flag_mask`；`enabled=false` 是唯一回滚开关（判据完全不跑）。
`DroneApplication::BindTopics` 整节注入 `MissionStateMachineConfig::estimator_guard`，
`config::EstimatorGuardConfig` → `EstimatorGuardThresholds` 的换算**唯一**发生在
`src/state_machine/mission_state_machine.cpp` 的 `ToEstimatorThresholds()`——本模块
不依赖 config，保持纯函数、只依赖 `common/types.h`（这样 13 条判据单测可以直接
构造阈值，不必拉起配置解析）。

`Thresholds()` 是只读访问器：构造函数会把 `critical_sensor_mask==0` 归一成模块默认
掩码 **`0x200007`**（陀螺 `1<<0` | 加速度计 `1<<1` | 磁罗盘 `1<<2` | AHRS `1<<21`，
不含 GPS），所以**启动日志与现场排查以 `Thresholds()` 为准**，不要回读 `config.json`
猜实际生效值。

## 3. 判据与阈值

| 判据 | 触发条件 | 形态 |
|---|---|---|
| `kLocalPositionStale` | `local_position_time_ms` 距今 > 500 ms，或该字段从未有效 | 瞬时 |
| `kVerticalJump` | 单拍 `|Δz| > 0.5 m` **且** `|Δz| > |vz|·Δt + 0.1 m` | 瞬时（一阵命中） |
| `kHorizontalJump` | 单拍 `|Δxy| > 1.0 m` **且** `|Δxy| > 地速·Δt + 0.3 m` | 瞬时 |
| `kStatustext` | `STATUSTEXT` 严重级 ≤ WARNING，且文本含 `ekf`/`estimator` | 瞬时 |
| `kEstimatorFlag` | `estimator_flags` 命中 `bad_estimator_flag_mask` | 瞬时（**默认关闭**） |
| `kInnovationRatio` | `pos_vert_ratio`/`pos_horiz_ratio`/`vel_ratio` 任一 > 1.0 | 持续 1 s |
| `kSensorUnhealthy` | `sensors_present & ~sensors_health` 命中关键掩码 | 持续 1 s |
| `kBaroDivergence` | `|气压 AGL − EKF 相对高度| > 1.5 m` | 持续 2 s |

关键设计点：

- **只在 `armed` 时判**。地面上的 EKF 重置无害，而且上电阶段的对齐抖动会把判据
  打成常亮；`armed=false` 时只刷新"上一拍位置"基线。
- **单拍跳变要扣掉速度位移**。0.8 m/s 慢降时每 40 ms 只移动 0.032 m，不能用
  "绝对位移 > 门限"硬判——否则正常下降就会误报。
- **只看"新的一帧"**。跳变比较以 `local_position_time_ms` 前进为触发条件，
  否则发布频率（10 Hz）高于遥测频率（25 Hz 时反而更密）会重复计算。
- **气压交叉校验用两个不同物理路径**：`ALTITUDE.altitude_amsl` 只受气压计/QNH
  影响，`base_z − local_z` 来自 EKF；EKF 被整体重置时后者平移、前者不动，差值即
  证据。这是现场 9 m 重置唯一的"事后也看得出来"的判据。
- **`ESTIMATOR_STATUS.flags` 默认不判**：不同 MAVLink 方言位号不同（本项目
  `third_party` 里是"置位=该路可用"的旧语义，部分固件用另一套位定义）。宁可
  少判，也不能用错极性误报；位掩码由 config 打开前必须先实机核对。

### 为什么关键传感器掩码里**没有** `LASER_POSITION`（2026-11-18 P5）

设计稿 P5 曾计划"给 `EstimatorGuard` 的关键传感器掩码加 `MAV_SYS_STATUS_SENSOR_LASER_POSITION`
（bit8=0x100）"。**2026-11-18 核对 PX4 源码后否决**，两条理由都写在这里，避免以后有人
"补上"：

1. **那个位在本版 PX4 永远不置位**。`src/modules/mavlink/streams/SYS_STATUS.hpp` 的
   `fillOutComponent()` 只填 battery / motors_escs / parachute / accel / gyro / mag /
   gps / rc / attitude_estimation / local_position_estimate / AHRS / logging /
   pressure 这些组件，**没有 `distance_sensor`**。把它加进 `critical_sensor_mask`
   是死代码：`present & ~health` 里永远不含这一位，判据永远不会命中。
   （同一原因，起飞前门禁也不用这个位，改判 `DISTANCE_SENSOR` 是否在报数——
   `common::RadarAlive()`；PX4 侧"是否要求装测距"由参数 `SYS_HAS_NUM_DIST` 管。）
2. **飞行中雷达掉数据不该判"估计器降级"**。这是**设计内的安全方向**，不是故障：
   `radar.enabled=true` 时高度出口是"雷达 AGL → 气压 AGL → EKF 差分"逐级退化，
   雷达过期/超量程就退到气压（气压不被 EKF 坐标重置平移），再退到 EKF 差分——
   判据要回答的问题是"估计还能不能信"，而"雷达没数据"并不让 EKF 变得更不可信。
   把降级锁存打上去反而会**冻结任务、停发控制、不降落**，比继续用气压降落危险得多。
   同理 65013 的 `bit13 雷达不可用`只在**地面待命 + `radar.preflight_required=true`**
   期间置位，飞行中雷达掉数据不置位。

**那"雷达坏了"由谁兜？** 三层：①高度出口逐级退化（气压/EKF 仍在）；
②`EstimatorGuard` 已有的**气压 AGL vs EKF 相对高度**交叉校验（`kBaroDivergence`）
仍能抓到"EKF 高度被重置而气压平滑"这类真问题；③S13a 保护 1 的对照源随在用高度源切换，
用气压时照样对照 EKF 差分。P5 的落地确认门禁（`landing.landed_max_agl_m`）则负责
"PX4 说落地了但高度见证说还悬着"这一条。

## 4. 日志行为

模块本身**不打日志**（纯判据，便于单测）。日志全部在状态机侧（2026-11-17 P3 起）：
命中一条 WARN，冻结后每 10 s 一条存活摘要，逐条文案见
[`mission_state_machine.md`](mission_state_machine.md)「日志行为」。命中 WARN 形如：

```text
估计器降级: 切 POSCTL 悬停，请飞手切 STABILIZED 接管: 判据=EKF 高度单拍跳变 数值=9.00 判定耗时=..ms 模式命令=已入队（POSCTL 3/0） state=..
```

锁存后不再重复打印命中 WARN，也不会随快照周期刷屏；切模式命令**入队成功后永不再发**
（只在 `SendCommand` 返回 false 时按 `return_home.command_retry` 重试，最多 3 次）。

## 5. 测试方式

```bash
cmake --build build -j$(nproc)
./build/estimator_guard_test
# 或 ctest --test-dir build -R estimator_guard --output-on-failure
```

`tests/state_machine/estimator_guard_test.cpp` 覆盖 13 条：

- 未上锁时 9 m 跳变不判（地面重置无害）；
- 9 m 高度跳变**一帧**命中，量级≈9.0；
- 0.8 m/s 正常下降 100 帧不误报；
- 水平跳变命中，而 10 m/s 真实高速位移不误报；
- 局部位置超时、气压不一致持续 2 s、传感器健康位持续 1 s、比值持续 1 s；
- WARN 文本触发、INFO 文本不触发；
- **锁存不自动恢复**，`Reset()` 后可重新判定；
- flags 掩码默认关闭、打开后按位判定。

## 6. 排查/修改要点

| 现象 | 先查 |
|---|---|
| 从不触发 | `armed` 是否一直为 false；`local_position_time_valid` 是否写入（`HandleLocalPosition`）；`baro_takeoff_ref_valid` 是否在解锁时抓到了气压基准 |
| 频繁误报 | `ratio_hold_ms`/`baro_hold_ms`/`sensor_hold_ms` 是否太短；气压基准是否在起飞瞬间就偏（起飞前气压曲线是否平稳）；`critical_sensor_mask` 是否包含了 PX4 正常波动会清位的传感器 |
| 跳变判据漏报 | `z_jump_m` 是否被调大；`local_position_time_ms` 是否没随消息更新（会导致一直比同一帧） |
| 降级后不恢复 | 这是设计：只能 `Reset()`（状态机在「重置任务状态」65020 `type=2` 或程序重启时调用）。若冻结在非 S13 的任务态，落地后状态机不会自行走到 S14，65020 会被拒，需重启机载程序 |

修改判据时**必须**同步：`docs/降落与估计器保护设计-2026-11-17.md` §3.3 判据表、
`include/common/types.h` 的字段注释、`docs/数据接口文档.md` 的快照字段说明。
