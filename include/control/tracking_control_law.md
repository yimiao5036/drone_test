# 视觉跟踪控制律（tracking_control_law + visual_tracking_shadow）

> 对应实现：`include/control/`、`src/control/`（tracking_config / tracking_types /
> line_of_sight / pid_controller / rate_limiter / staleness_checker /
> frame_transform / tracking_control_law / visual_tracking_shadow）
> 规格：`docs/物理追踪思路.md`；移植设计：`docs/superpowers/specs/2026-09-23-视觉跟踪控制律移植-design.md`
> 来源：自 `visual_track_control/` 独立子库移植（2026-09-23，原库冻结存档、
> 保持独立构建，不再演进）；网关（mavlink_command_gateway）未移植，通信由 Px4Link 承担

## 1. 功能职责

**TrackingControlLaw**（纯算法，单线程调用）：每个控制周期消费
`ControlSnapshot`（视觉追踪样本 + 目标距离 + 自机姿态），输出限幅后的
`ControlOutput`（NED 速度 + 航向指令）：

- ①超龄判定：姿态缺失/过期或视觉超龄 → 降级保持（§7.1/§9.2）；
- ②判丢（!tracked）→ 零速刹车并清积分/限幅器（§6.4）；
- ③Coast（is_predicted）→ 清距离积分防饱和（§6.4）；
- ④像素坐标 → 视线角：**内参针孔式 `atan((px−c)/f)`**（水平 β / 垂直 α）；
- ⑤水平航向：速率式 β→yaw_rate_dps / 位置式 β+当前航向→yaw_deg（§4.2）；
- ⑥垂直统一速度支路：α→vz（§5.2，小误差自然映射小速度）；
- ⑦距离通道：Δd→PID→vx；无距离按配置降级（§6.2/§6.3）；
- ⑧机体系→NED 随航向旋转输出（§2.3）。

**VisualTrackingShadow**（影子适配层组件）：20Hz（配置）固定节拍组装快照
驱动控制律，把输出映射为 `ControlIntent` 发布 `kControlIntent`。**纯影子：
kControlIntent 当前无消费者，不产生真实控制输出。**

**第二调用点（阶段四 Task 9）**：`MissionStateMachine` 的 S7 视觉伺服协作者
直接持有 `TrackingControlLaw` 实例（配置来自 `visual_tracking` 节同源注入），
在 10Hz 决策周期内组装快照并调用 `Update`（控制律按实际时钟差推算 dt，
配置频率仅作首拍兜底），输出映射为状态机 `ControlIntent`（`control_source=1`
带 bit31，接 `FlightController`——正式控制链）。影子与状态机的意图输出是两个
独立 Topic，互不干扰；影子保留供观察（见 `visual_tracking_shadow.h` 并存说明）。

不做什么：不接 Px4Link；不做多目标选举；不做 NED 空间转换（属
TargetEstimator 融合阶段）。距离通道仅在注入 `StereoTargetDistance` 且该帧
有效时启用；未注入/无效时 distance_valid 恒 false，走配置的
`no_distance_action` 支路。生产装配当前尚未调用 `SetEstimationInputs`
（视觉/测距输入未接线），S7 保持影子；SITL 验收工具会注入三路输入。

## 2. 接口与数据流

```text
Topic<VisualTargetStatus> ─┐
                           ├─► VisualTrackingShadow ──► Topic<ControlIntent>(kControlIntent)
Topic<FlightStateSnapshot>─┘   （固定节拍 control.frequency_hz）
```

- 状态映射：`VisualTargetStatus.state==kLocked` → tracked=true，其余 false；
  is_predicted 恒 false（主链路无预测）。
- 虚拟姿态：姿态缺失/超龄（PX4 未连接或未上电；2026-10-10 起现场 PX4 已到位，
  正常应走真实姿态）时，`shadow.virtual_attitude_when_absent`（默认 true）以零姿态
  继续计算仅供观察（首次进入 INFO 标注）；false 保持真实语义（每拍降级保持）。
- ControlIntent 映射：kVelocityHeading→target_x/y/z=NED 速度 + 速率式
  `yaw_rate_dps`（types.h 新增字段）/位置式 `yaw_deg`；kBrakeHover→kBrakeHover；
  kDegradedHold→kNone。reason_code 影子段（避开骨架 0-7）：
  **100=正常跟踪 101=Coast 102=判丢刹车 103=姿态超龄 104=视觉超龄
  105=持续无距离退出**。

## 3. 关键实现点

- **内参式角度换算（移植核心修复）**：原库以 FOV+画面中心为基准；2026-09-23
  标定实测主点偏中心（左目 cx=620.7、cy=532.1 @1280×720，cy 偏差约 9.7°），
  改为 `PixelToAngle(f_px, c_px)` 内参式，偏移自主点量起。camera 组默认值
  即左目标定实测；**内参只对 1280×720 档有效，换档必须重新标定**。
  畸变不参与（中心区域 <0.5°；由测距 v1.0 立体校正承担）。
- **拆 target_tracker 依赖**：原库 ControlSnapshot.track 内嵌 TrackResult，
  移植后换自有精简 `VisualTrackSample`（tracked/is_predicted/center_x/y）。
- **配置**：`config.json` 的 `visual_tracking` 段（control/camera/heading/
  vertical/distance/accel_limit/shadow 七组，全部可选、缺席用默认值），
  解析在 config.cpp（含枚举串映射），跨字段校验在
  `TrackingConfig::Validate()`（主点必须在画面内、PID 非负、滤波系数∈[0,1) 等）。
  开关 `runtime.enable_visual_tracking`（依赖 enable_visual_monitor + enable_video）。
- **节拍线程**：固定周期 sleep_until，严重滞后（>500ms）重置节拍基准防追帧
  空转；输入订阅容量 1 + kDropOldest，每拍排空取最新。
- **装配**：DroneApplication 在 visual_monitor 之后创建/启动/停止；
  姿态输入可选（px4_link 存在才接）。

## 4. 日志行为

| 等级 | 场景 |
|------|------|
| INFO | 影子启动/停止、虚拟姿态启用（一次）、控制律状态切换（刹车进出/降级切换）、节流摘要（默认 10s：模式/速度/yaw_rate/coast/虚拟姿态） |
| ERROR | 影子启动失败（视觉输入未接线） |

热路径（每拍/每消息）零日志。

## 5. 测试方式

`tests/control/`（CMake 统一注册，链接 drone_control_core）：

- 移植用例：pid/rate_limiter/staleness/frame_transform/tracking_control_law
  （25 例，超龄/刹车/Coast/航向两式/垂直/距离降级/限幅/NED 旋转）；
- 新增：line_of_sight 内参式（含标定实测量值：cy=532 处 α=0、画面中心
  360 → α≈−9.68°）；tracking_config Validate 边界；visual_tracking_shadow
  （未接线启动失败、虚拟姿态两态、仅 kLocked 输出、kLost→刹车、
  超龄→kNone+降级码、reason_code/控制源字段）。

```bash
ctest --test-dir build -R 'ControlLaw|PixelToAngle|VisualTrackingShadow|TrackingConfig' --output-on-failure
```

## 6. 排查/修改要点

| 现象 | 排查方向 |
|------|----------|
| 摘要日志行为（2026-10-12 起） | 无目标/无输入时窗口内容与上次完全相同 ⇒ 不打印；离散状态（模式/有效/coast/虚拟姿态/距离有效性）变化立即打印一次，首拍必打印（`docs/故障排查手册.md` 第 19 节） |
| 摘要一直是"降级保持" | 看 reason：姿态超龄（PX4 未连接/未上电，且虚拟姿态被关）/视觉超龄（monitor 没发布或视频链路没跑）；虚拟姿态开关在 `visual_tracking.shadow.virtual_attitude_when_absent` |
| 垂直角方向反了/量值不对 | 确认 camera 组 cy_px 是标定值 532.07 而非 360；主点假设错误的典型症状就是垂直方向固定偏约 9.7° |
| 换了分辨率档位后角度错 | 内参只对 1280×720 有效，换档重新标定并更新 camera 组 |
| 影子 kControlIntent 没人消费 | 影子主题无消费者（2026-10-13 阶段四 Task 9 起正式路径 = 状态机 S7 `MissionStateMachine::IntentOutput` → `FlightController`）；影子保留供观察，靠摘要日志核对行为 |
| 增益/PID 整定 | 已整定（2026-10-14，见下节）；调整时只改 config.json，不动代码，改后必须用 `--sitl-stage4-tune` 复跑验证 |

## 7. 参数整定记录（2026-10-14，SITL 整定完成）

**环境与工具**：WSL2 Ubuntu 24.04 + PX4 v1.17.0 SITL（环境前置条件见
`tools/px4_link_smoke.md` §2.2）；整定平台 `--sitl-stage4-tune`（S7 长驻留 +
分通道阶跃换位），逐拍 CSV + `tools/tune_report.py` 指标判定（指标定义与验收值见
`tools/tune_report.md`）。整定顺序：heading(P) → vertical(P) → distance(PID)。

**终值**（已回填 `config/config.json`，`config/px4_sitl.json` 同值）：

| 参数 | 占位值 | 终值 | 第 5 轮实测（达标） |
|---|---|---|---|
| heading.gain | 1.0 | **0.9** | ±2/±4m 阶跃：收敛 2.0~3.8s（上限 4s）、变号 ≤1、稳态 ≤0.12°、指令 ≤21.4dps |
| vertical.vz_gain | 0.1 | **0.1**（保持） | +1/+3m 阶跃：收敛 3.7~6.0s（上限 8s）、变号 0、稳态 ≤0.17°、vz ≤1.5m/s |
| distance kp/ki/kd | 0.5/0.05/0.1 | **保持** | ±2.5/±5m 阶跃：零过冲、收敛 1.5~5.8s（上限 8s）、稳态 ≤0.13m、vx 无振荡、设定值 ≤2.0 |

**证据**：`tools/tune_evidence/stage4_tune_20261014_r5.csv`（6049 行逐拍数据），
复跑命令：

```bash
./build/px4_link_smoke --config config/px4_sitl.json --duration 600 \
  --sitl-stage4-tune --tune-script full --tune-csv /tmp/tune.csv
python3 tools/tune_report.py /tmp/tune.csv    # 期望"全部指标达标"
```

**整定过程关键发现**（供后续复整定参考）：

1. **姿态-几何耦合**：偏航机动/追赶加速的俯仰瞬态（SITL 实测峰值 4.7°）会经相机
   外参几何直接注入 α/β（"相机固连机体"的固有耦合，非缺陷）。vz_gain=0.2 时该
   耦合可在 1m 悬停下压 0.7m 触发安全回收——**垂直增益上限由该耦合约束，
   vz_gain=0.1 是实测安全侧**；S5 位置引导是 3D 的（会把机体拉向目标高度），
   不能靠抬高起飞高度获得垂直余量。
2. **heading.gain**：0.8 在一段 ±4m 回摆收敛 4.24s（判定限 4s）边际超标；
   0.9 全段 3.8s 内收敛且稳态/变号/指令纪律均达标。
3. **距离 PID 占位值即可用**：0.5/0.05/0.1 在 d=10m 射程零过冲、无积分极限环；
   微分支路（kd=0.1 + 低通 0.2）配合 slew 2m/s² 提供阻尼。
4. **遗留**：`d_exp_m=10` 仍为占位期望射程；该值冻结后必须复跑本整定场景
   复核零过冲硬判据（PID 增益基于该射程的闭环特性整定）。

