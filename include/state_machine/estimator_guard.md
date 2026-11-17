# 估计器健康判据（estimator_guard）

> 对应实现：`include/state_machine/estimator_guard.h`、`src/state_machine/estimator_guard.cpp`
> 使用方：`MissionStateMachine`（P3 接入：判据置位 → 切 POSCTL 悬停 + 告警）
> 设计文档：`docs/降落与估计器保护设计-2026-11-17.md` §3.3
> **接线状态（2026-11-17）**：P1 已完成判据与单测；**状态机尚未接线**，
> 也就是说当前这是"看得见但不动作"的观测层，P3 才接处置。

## 1. 功能职责

只回答一个问题：**这一帧 PX4 快照里的位置/高度估计还能不能信。**

| 做 | 不做 |
|---|---|
| 从一帧 `FlightStateSnapshot` 里判跳变、比值、健康位、气压交叉校验 | 切模式、发 `COMMAND_LONG`、写日志、访问 Topic |
| 维护滞回计时（持续型判据要满足 `*_hold_ms`） | 决定处置动作（POSCTL/返航/降落由状态机定） |
| 首次命中后**锁存**，`Reset()` 才清除 | 自动恢复：好转也不解除，避免抖动下反复切模式 |
| 给出首个命中的原因与量值（日志/告警用） | 判断"该不该起飞"（那是起飞前门禁的事） |

线程安全：无内部同步，只能在状态机决策线程内调用。所有接口不抛异常。

## 2. 接口与数据流

```cpp
EstimatorGuard guard;                 // 阈值默认见头文件；P2 起由 config.landing 外置
guard.Update(snapshot, now_ms, armed);// 每收到一帧 FlightStateSnapshot 调一次
guard.result();                       // { degraded, issue, since_ms, magnitude }
guard.Reset();                        // 重置任务状态 / 程序重启时清锁存
```

输入只有快照与 `now_ms`、`armed`；不额外持有时间戳表（`LOCAL_POSITION_NED` 的
到达时刻由 `px4_link` 写进快照的 `local_position_time_ms`）。

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

## 4. 日志行为

模块本身**不打日志**（纯判据，便于单测）。由调用方在首次命中时打一条 WARN：

```text
估计器判据命中: issue=VerticalJump magnitude=9.00m since_ms=... （切换 POSCTL 悬停）
```

锁存后不再重复打印；状态机每到"判据降级"就切一次模式并记一条事件日志，
不会随快照周期刷屏。

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
| 降级后不恢复 | 这是设计：只能 `Reset()`（清任务状态/程序重启）。要恢复请走 P4 |

修改判据时**必须**同步：`docs/降落与估计器保护设计-2026-11-17.md` §3.3 判据表、
`include/common/types.h` 的字段注释、`docs/数据接口文档.md` 的快照字段说明。
