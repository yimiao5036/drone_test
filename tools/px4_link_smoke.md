# PX4 串口冒烟与 SITL 分级控制测试（px4_link_smoke）

> 对应实现：`tools/px4_link_smoke.cpp`
> 使用模块：`SerialPort`、`MavlinkHandler`、`Px4Link`、`FlightStateSnapshot`、
> `FlightController`（stage4/stage4-mission）、`MissionStateMachine` +
> `TargetEstimator`（stage4-mission）

## 1. 功能职责

该程序用于分级验证真实 Pixhawk/PX4 串口基础链路和 PX4 SITL UDP 控制链路；stage 0~3D
不依赖视频、YOLO、图传、状态机和控制器，stage4（`--sitl-stage4-controller`）装配真实
`FlightController` 验证执行层全链（G2 门禁），stage4-mission
（`--sitl-stage4-mission`）装配状态机全链验证 S5→S6→S7→回退→停发（G3 门禁，
另补验 G2 表列的围栏钳制与切模式停发），两者均不依赖视频/相机链路。

测试内容：

- 严格加载 JSON 中的机载电脑身份、PX4 目标身份、固件版本和串口参数；
- 打开 `/dev/ttyS1`，启动真实 `Px4Link`；
- 发送标准机载电脑 HEARTBEAT；
- 接收 PX4 HEARTBEAT 和已配置输出的遥测；
- 每秒打印一次状态摘要，连接/armed/模式变化时立即打印；
- 到期或 Ctrl+C 后停止并输出 PASS/WARN/FAIL 验收报告；
- 根据基础链路结果返回进程退出码；
- stage4：装配 `ControlIntent → FlightController → Px4Setpoint → Px4Link` 全链，
  按脚本化意图序列验收零速漂移、速度阶跃、停发刹车与 Offboard 退出时延；
- stage4-mission：装配 `MissionStateMachine → FlightController → Px4Link` 全链并注入
  地面站目标/模拟视觉观测/模拟测距，验收迁移序列（S5→S6→S7→回退→停发）、交接处
  输出连续与卡尔曼备份接管，另含围栏钳制与切非 Offboard 停发时延补验。

安全边界：不带 SITL 阶段参数时，程序不绑定 `Px4Setpoint`，不会发送解锁、模式切换、起飞、
降落或 Offboard 控制；主动内容仅为 HEARTBEAT 和遥测请求。stage4-mission 会在 SITL 中
真实起飞（相对 1m）、水平引导至注入目标点并在该点降落上锁（全程 armed 包线内），
不得在真实飞机上运行。所有控制阶段参数强制要求
`transport=udp` 且互斥，串口配置会在启动前拒绝。`--sitl-takeoff-land` 固定相对起飞 1m，
不可通过命令行扩大，确认 landed 前禁止 DISARM。stage4 的意图/边界同样固定，不可经命令行放大。

## 2. 构建与运行

香橙派仓库目录中执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target px4_link_smoke -j6
./build/px4_link_smoke --config config/config.json --duration 30
echo "退出码=$?"
```

构建目标会把 `config/` 复制到 `build/config/`，因此也可直接执行：

```bash
./build/px4_link_smoke --duration 30
```

参数：

| 参数 | 说明 |
|---|---|
| `--config <路径>` | 显式指定 JSON；缺省按“可执行文件旁 config → 当前目录 config”查找 |
| `--duration <秒>` | 测试时长，范围 1~3600，默认 30 秒；解锁零速度最短 10 秒，起飞降落最短 20 秒 |
| `--sitl-zero-velocity` | 仅 UDP，零 NED 速度，不切模式、不解锁 |
| `--sitl-offboard-disarmed` | 仅 UDP，零速度后进入 Offboard，全程不解锁 |
| `--sitl-arm-zero-velocity` | 仅 UDP，ARM 后零速度保持 5 秒，再主动 DISARM |
| `--sitl-takeoff-land` | 仅 UDP，相对起飞 1m、稳定悬停 3 秒、AUTO.LAND、确认 landed 后主动 DISARM |
| `--sitl-horizontal-motion` | 仅 UDP，起飞 1m 后北向 0.5m/s×2 秒、零速度制动、返回起点、降落上锁；最短 30 秒 |
| `--sitl-offboard-loss` | 仅 UDP，起飞 1m 后停止 setpoint、验证 PX4 退出 Offboard，再 AUTO.LAND 回收；最短 25 秒 |
| `--sitl-stage4-controller` | 仅 UDP，装配真实 `FlightController` + 意图脚本（G2 执行层门禁）；最短 60 秒 |
| `--sitl-stage4-mission` | 仅 UDP，装配状态机全链 + 注入模拟数据（G3 全链门禁，含 G2 补验两项）；最短 60 秒 |
| `--selftest` | 离线自检工具纯逻辑（G3 意图分类等），不读配置、不触 SITL/硬件；已纳入 `ctest`（`px4_link_smoke_selftest`） |
| `--help` | 显示帮助 |

### 2.1 stage4 执行层门禁脚本（`--sitl-stage4-controller`）

装配与数据流：工具创建 `Topic<ControlIntent>`、真实 `FlightController`（配置节
`controller`，字段与默认值同源 `config/config.h`）并接线
`ControlIntent → FlightController → Px4Setpoint → Px4Link`。工具扮演“状态机 + 地面站”
角色发布脚本化意图（每 100ms 一条，带 `reason_code` bit31 控制权请求位、
`valid_until_ms = now+500ms`），控制器授权链要求 `connected + armed + Offboard(6/0)`。

**进入 Offboard 的引导流**：PX4 要求进入 Offboard 前已存在 setpoint 流（stage3A 纪律），
而控制器授权链要求 armed + Offboard 才发布——先有鸡还是先有蛋。工具因此在接管前直接向
控制器输出主题注入零速度引导流（控制器 IDLE 期间不发布，无冲突），满足“≥2 秒且 ≥20 条”
后请求 Offboard；ARM 确认 armed、控制器授权链全通转 ACTIVE 后停止注入，由控制器供流
（停注后 `SetpointSendCount` 继续增长即证明接管成功）。该引导顺序说明进入 Offboard 的
生产接管次序仍待后续任务（状态机侧的 ARM/模式命令）落地。

脚本序列与验收断言（报告逐条 PASS/FAIL，实测值一并打印）：

| 段 | 内容 | 验收断言 |
|---|---|---|
| 0 | 引导零速度流 → Offboard(6/0) → ARM | Offboard/ARM 的 ACK accepted + 真实模式/armed；控制器接管后链路发送持续增长 |
| 1 | 位置引导意图（`control_source=0`）起飞至相对 1m 并稳定悬停 3 秒 | 高度误差 ≤0.15m、水平漂移 ≤0.30m、垂速 ≤0.30m/s；该段位置意图零拒绝（含 Home 基准） |
| 2 | 零速度意图保持 3 秒（`control_source=1`） | 最大水平漂移 < 0.10m |
| 3 | 北向 2m/s 阶跃意图保持 2.5 秒 | **设定值**爬升斜率 ≤ max_accel_mps2×1.2（发布侧样本、300ms 时间窗差分，2026-10-14 起由遥测差分改为发布侧——遥测成簇/抖动曾测出 4.63 m/s² 伪值，ulog 地面真值：设定值 2.12、实机 1.77）；稳态速度 = min(2, max_speed_mps) ±0.25m/s（遥测侧） |
| 4 | 停止发布意图（超龄注入） | 控制器转 COAST 零速刹车；速度过零（≤0.10m/s）时间 ≤ 2000ms（PX4 速度环指数收敛物理下限 ≈1.4s + 遥测量化余量；早前以 coast_timeout=1000ms 为上限物理不可达，2026-10-14 ulog 复核修正） |
| 5 | 持续停发 | 控制器转 IDLE 停发（`valid=false`），PX4 在 2.2s 内退出 Offboard（COM_OF_LOSS_T 1.0s + HEARTBEAT 1Hz 相位 ≤1s + 余量；早前 1.3s 上限按此测法随机失败）；停发后额外发送 ≤2 条 |
| 6 | AUTO.LAND → landed → 主动 DISARM | ACK accepted + 真实 landed/armed 状态；未落地禁止 DISARM |

硬包线（armed 期间越界即安全回收 AUTO.LAND）：相对高度 ≤1.30m；阶跃前水平漂移 ≤0.50m；
阶跃/停发段北向 ≤6.0m、东西 ≤1.0m、高度 ≥0.50m；连接/位置/姿态失效同样触发回收。
非预期的 Offboard 退出（非步骤 5）计入失败。

退出码沿用第 5 节；无 SITL 时在 10 秒连接超时后中止脚本并报告 FAIL（退出码 3），不得误报 PASS。

### 2.2 stage4-mission 全链门禁脚本（`--sitl-stage4-mission`，G3）

**运行环境前置条件（2026-10-14 实跑建立，缺一即失败）**：

1. **机体朝北出生**：gz 世界为 ENU，模型默认 yaw=0 朝东；必须
   `HEADLESS=1 PX4_GZ_MODEL_POSE=0,0,0,0,0,1.5708 make px4_sitl gz_x500`
   启动（gz yaw=+90° ⇒ PX4 航向 0° 朝北，映射经 gz set_pose 活体实验标定）。
   否则目标不在相机前向，视觉注入自检（`cam_z ≤ 0.1` 拒绝）会中止脚本。
2. **干净会话**：注入目标为 Home 相对绝对坐标（主目标北向 40m），车辆必须位于
   Home 原点附近——上一次运行会在降落点残留，复用会话会“出生在目标身后”直接
   中止。每次跑 G3 前重启 SITL（tmux 会话与 gz 进程都清掉）。
3. **PX4 巡航限速 2m/s**：S5 位置引导透传目标点，接近速度由 PX4
   `MPC_XY_VEL_MAX`（默认 12m/s）决定，全速 6 秒即冲过 40m 目标导致几何退化；
   开跑前 `param set MPC_XY_VEL_MAX 2.0`（SITL 参数持久化，重启不失，须复核）。
   该行为同时是生产事实：远端引导段接近速度上限由 PX4 参数而非
   `controller.max_speed_mps` 决定，现场围栏外接近速度规划需注意。
4. **GCS 心跳在场**：无 GCS 心跳时 commander 拒绝解锁（Preflight Fail: No
   connection to the GCS）；无 QGC 时可用 pymavlink 在 14550 端口 1Hz 回发
   MAV_TYPE_GCS 心跳（注意必须先 `recv` 学习对端地址，否则发不出去）。

装配与数据流：工具装配 **正式 `MissionStateMachine`**（唯一意图生产者）+
`FlightController` + `Px4Link`，并扮演**地面站/视觉监视器/双目测距**三路生产者注入
模拟数据（生产装配同构，仅上游换为注入）：

```text
注入地面站目标(10Hz, WGS-84) → TargetEstimator(真实卡尔曼 + 视觉融合) → TargetState ─┐
注入 VisualTargetStatus(20Hz, 像素) ──────────────────────────────────────────────┼→ MissionStateMachine
注入 StereoTargetDistance(20Hz, 相机系坐标) ─────────────────────────────────────┘   （S5 位置引导 /
Px4Link 真实遥测 → FlightStateSnapshot（导航门禁/Home/伺服姿态/估计器输入）            交接判据/S7 伺服/
                                                                                     热备份核对）
  → ControlIntent → FlightController（门禁/限幅/无扰衔接）→ Px4Setpoint → Px4Link → PX4
```

配置与几何注入约定（2026-10-13 落地）：

- 配置经 `LoadAppConfig` **同源加载**（`controller`/`handover`/`visual_tracking`/
  `target_estimator`/`actuator` 整节；缺席节用默认值），`fxb_m_px` 从双目校正矩阵
  同源读取；启动打印状态机阈值与注入几何；
- **装配期注入几何自检**（开跑前 fail-fast）：相机系反变换
  （`NedToCameraFrame` ↔ `perception::VisualCameraToLocalNed`）与地理反解
  （`OffsetToGeodetic` ↔ `perception::GeodeticToLocalNed`）必须互逆（阈值
  1e-6m / 0.01m），否则拒绝开跑——注入观测与卡尔曼航迹必须落在同一点，否则
  创新门限会误拒交接；
- 注入目标/观测的几何：探针目标北向 150m（> `controller.max_radius_m`）、主目标
  北向 40m、目标高度 +1m（与悬停同高）；视觉像素与测距由"目标 NED − 自身 NED"
  经真实姿态/安装外参反变换生成（俯仰/偏航变化下仍一致）；
- 注入目标高度基准 = Home（`alt_reference=1`，WGS-84 反解与 `GeodeticToLocalNed`
  同式，发布前正向回验 ≤0.05m）；目标有效期 2s、10Hz 刷新；
- 注入测距**不模拟** `StereoRanger` 最小视差有效域（z_max≈12.7m）：G3 验状态机
  判据与迁移序列，测距有效域/精度归实拍与独立测试工程；
- 短期/持续 LOST 期间**测距保持有效**（模拟"测距仍跟踪、跟踪器判丢"），使 S7
  退出由 LOST 迟滞路径唯一决定（测距失效路径由单测覆盖）；
- 交接缝判据按**跨缝测量**而非 S6 驻留期取样：`lock_frames` 门限 + 20Hz 注入使 S6
  只驻留 1 决策拍，按 S6 标签取间隔样本恒为 0（行为正确的系统会被误判 FAIL）——
  改为"末条 S5 位置意图 → 首条 S7 速度意图 < `intent_timeout` 且窗口内未跌入
  COAST"，与设计 §2.5"交接点不得出现意图超龄→刹车→再起步"等价；S6 冻结续发的
  深层语义（逐拍滚动续期、超时封顶）由单测覆盖，工具侧只记录"是否观察到 S6 标签
  的冻结续发意图"与"伺服接管前（S5 引导 + 交接缝）相邻意图最大间隔"作诊断打印——
  交接缝判据 FAIL 时可据此区分"意图链真断档"（最大间隔 ≥ 有效期，含冻结续发缺口）
  与"仅 S5→S7 过渡耗时偏长"（间隔仍小于有效期）；
- 交接缝所引用的 `intent_timeout` 与状态机意图有效期**同源**（`controller.intent_timeout`
  经 `LoadAppConfig` 注入，报告打印实测值），`valid_for_ms` 一致的 500ms 常量仅用于
  工具自身引导意图（工具是接管前的意图生产者）；
- **意图分类纪律（2026-10-14 终审 Critical 防复发）**：工具按 reason 低位段 +
  意图类型分类状态机意图（`ClassifyMissionIntent` 纯函数）——位置引导 7（S5/S6 与
  S7 过渡）、视觉伺服 100、判丢刹车 102；**只排除工具自身引导意图（低字节 0）**。
  不得写成"只认低字节 7"（那会把 S7 伺服/刹车意图整体丢弃，`servo_intent_seen_`
  永假、交接判据永不计算、脚本在交接段超时中止）。该分类由 `--selftest` 离线断言
  （8 条用例，含 servo/brake 通过过滤与工具引导被排除），并已纳入 `ctest`。

脚本序列与验收断言（报告逐条 PASS/FAIL，实测值一并打印）：

| 段 | 内容 | 验收断言 |
|---|---|---|
| 0 | 引导零速度流 → Offboard(6/0) → ARM → 工具引导意图接管 | 同 stage4 引导段；控制器接管后链路发送持续增长 |
| 1 | **[G2 补验]** 命令 AUTO.LOITER（切非 Offboard） | 首个非 Offboard 快照 → 控制器转 IDLE 实测 ≤200ms；停发后设定值额外发送 ≤2 条 |
| 2 | 引导流重入 Offboard 并重新接管 | Offboard ACK accepted + 真实 6/0 + 控制器回 ACTIVE |
| 3 | 工具位置引导意图起飞至相对 1m、稳定悬停 | 高度误差 ≤0.15m、水平漂移 ≤0.30m、垂速 ≤0.30m/s |
| 4 | **[G2 补验]** 注入围栏外目标（150m） | `ClampedCount` 增长 ≥1 且控制器输出位置设定值水平模长 ≤ `max_radius_m`+0.5m |
| 5 | 换注入近距目标（40m）→ 状态机接管意图流 | 出现 `reason` 低位=7 的位置引导意图（工具停止注入）；状态=S5 |
| 6 | 注入 SEARCHING→LOCKED 帧 + 测距 | 迁移序列 S5→S6→S7；**交接缝无意图超龄**（末条 S5 位置意图 → 首条 S7 速度意图间隔 < `intent_timeout`，且交接窗口内控制器未跌入 COAST）；出现 `control_source=1` 速度意图 |
| 7 | 短期 LOST（10 帧 < `exit_lost_frames`） | 状态保持 S7；出现零速刹车意图（reason 低位 102） |
| 8 | 持续 LOST（达 `exit_lost_frames`）→ 回退 | 状态回 S5；位置引导意图恢复（卡尔曼备份接管）；控制权全程不中断（ACTIVE） |
| 9 | 交接/回退全程输出连续性 | 速度式设定值隐含加速度 ≤ `max_accel_mps2`×1.5（含位置式→速度式段首"从 0 起步"核验） |
| 10 | 停止注入目标（过期） | 告警位 bit1 置位 + 意图停发（> `intent_timeout` 无新意图）→ COAST 零速刹车 → IDLE → PX4 ≤2.2s 退出 Offboard（COM_OF_LOSS_T 1.0s + HEARTBEAT 1Hz 相位 ≤1s + 余量，2026-10-14 由 1.3s 修正）；停发后额外发送 ≤2 条 |
| 11 | AUTO.LAND → landed → 主动 DISARM | 同 stage4 收尾（未落地禁止空中 DISARM） |

硬包线（armed 期间越界即安全回收 AUTO.LAND）：相对高度 0.50~1.30m（下限只对升空后
生效）、北向 ≤ 主目标北向+6m、东西 ≤2m、水平半径 ≤ `max_radius_m`+0.5m；连接/位置/
姿态失效同样触发回收；非预期退出 Offboard（非步骤 1/10）计失败。

退出码沿用第 5 节；无 SITL 时在 10 秒连接超时后中止脚本并报告 FAIL（退出码 3），
不得误报 PASS（无数据判定项不得空过：连续性要求速度样本非空、控制器异常计数要求
已 armed）。

**SITL 实跑已于 2026-10-14 在本机 WSL2 完成并通过**（环境搭建与门禁结果见
`docs/PX4实链验证清单.md` §7.1）；负对照（无 SITL → 退出码 3、21 条 FAIL、
0 条 PASS）与注入几何自检（相机系互逆 0m、地理反解 1.7e-10m）在任务内完成。

### 2.3 stage4-tune 控制律整定场景（`--sitl-stage4-tune`）

装配与 G3 同构（`MissionStateMachine` + `FlightController` + `Px4Link` + 注入构件
`Stage4InjectionRig`），剧本面向**控制律参数整定**（`visual_tracking.heading.gain` /
`vertical.vz_gain` / `distance.kp/ki/kd`）：

```text
起飞悬停（相对 1m，与门禁同） → 注入主目标（北向 40m、up=1m）→ S5 位置引导接近至
≤20m（--tune-approach-until-m；注意 S5 为 3D 引导，接近段会把机体拉向目标高度）
→ SEARCHING 500ms → LOCKED → S7 长驻留（--tune-static-dwell-s，默认 25s）
→ 横向阶跃 E: 0→+2→0→+4→0→−2→0→−4→0（航向通道）
→ 距离阶跃 N: 40→42.5→40→45→40→37.5→40→35→40（距离 PID 通道；幅值上限 5m——
  机体以 d_exp=10m 尾随，远跳追赶后目标回跳时几何余量最小，更大幅值会跌破
  换位前向余量校验）
→ 垂直阶跃 up: 1→2→1→4→1（垂直通道）
→ 停视觉注入 → 断流回退 S5 → 停目标 → 过期停发链 → AUTO.LAND → DISARM
```

- 阶跃段每段驻留 `--tune-dwell-s`（默认 15s）；换位经 `SetTargetNed` 同步移动
  地面站目标与视觉/测距注入点（卡尔曼热备份不发散），换位合法性校验：围栏内
  （≤max_radius−10m）且换位后预算 cam_z ≥2m；
- 逐拍（20Hz）落 CSV：注入像素/视线角/距离真值、意图与设定值（发布侧）、自机
  遥测 NED/yaw、phase/目标列天然分段；指标分析用 `tools/tune_report.py`
  （指标定义与验收值见 `tools/tune_report.md`）；
- `--tune-script static|heading|distance|vertical|full`（默认 full）按通道裁剪
  剧本；硬包线（armed 越界即 AUTO.LAND 回收）：相对高度 0.3~6.0m（垂直阶跃会爬升
  至 ~4m，故高于 G3 的 1.3m）、水平半径 ≤90m；环境前置条件与 G3 完全相同
  （§2.2 末节：朝北出生/干净会话/MPC_XY_VEL_MAX=2/GCS 心跳）；
- **姿态-几何耦合**：偏航机动/追赶的俯仰瞬态会经相机外参直接注入视线角（相机
  固连机体的固有耦合）。垂直通道的增益上限由该耦合约束——2026-10-14 实测
  vz_gain=0.2 时可在 1m 悬停下压 0.7m 触发安全回收，**0.1 为实测安全侧**；
  整定垂直增益时留意此现象，勿归因于垂直通道振荡；
- **配置前置**：`config/px4_sitl.json` 必须带完整 `visual_tracking` 节——缺席时
  落到头文件默认值（distance PID=0/0/0，距离通道不工作），整定结果无效；
- 退出码沿用第 5 节：剧本走通（含回退与过期停发链）且 CSV 可写、无中断/安全回收
  → 0，否则 3。

## 3. 当前配置

机载身份（`onboard_system_id`/`onboard_component_id`）自 2026-09-02 多机区分配置改造起
与正式主程序一致，位于 **`px4` 节**（历史顶层 `mavlink` 节已废弃，照抄旧样例会被工具以
JSON 异常拒绝）。下面是与 `config/px4_sitl.json` 一致的**完整可运行样例**（可直接照抄为
SITL 配置文件）：

```json
{
    "log": {
        "dir": "logs/",
        "level": "info"
    },
    "px4": {
        "transport": "udp",
        "onboard_system_id": 1,
        "onboard_component_id": 191,
        "firmware_version": "1.17.0",
        "target_system_id": 1,
        "target_component_id": 1,
        "mavlink_version": 2,
        "heartbeat_send_interval_ms": 1000,
        "heartbeat_timeout_ms": 3000,
        "telemetry_timeout_ms": 2000,
        "state_publish_interval_ms": 100,
        "command_ack_timeout_ms": 1000,
        "setpoint_send_interval_ms": 50,
        "setpoint_timeout_ms": 500,
        "setpoint_queue_capacity": 4,
        "command_queue_capacity": 16,
        "one_shot_message_requests": [148, 242],
        "message_interval_requests": [
            {"message_id": 1, "interval_us": 1000000},
            {"message_id": 24, "interval_us": 200000},
            {"message_id": 30, "interval_us": 100000},
            {"message_id": 32, "interval_us": 100000},
            {"message_id": 33, "interval_us": 200000},
            {"message_id": 65, "interval_us": 200000},
            {"message_id": 147, "interval_us": 1000000},
            {"message_id": 245, "interval_us": 500000}
        ],
        "serial": {
            "device": "/dev/ttyS1",
            "baud_rate": 115200,
            "data_bits": 8,
            "stop_bits": 1,
            "parity": "N",
            "read_timeout_ms": 20,
            "write_timeout_ms": 100
        },
        "udp": {
            "bind_address": "127.0.0.1",
            "bind_port": 14540,
            "remote_address": "127.0.0.1",
            "remote_port": 14580,
            "read_timeout_ms": 20,
            "write_timeout_ms": 100
        },
        "reconnect_interval_ms": 1000
    }
}
```

上板串口（`config/config.json`）的 `px4` 节结构完全相同，区别只有：`transport` 为
`"serial"`（用 `serial` 子节，`udp` 子节仅为完整性保留）。工具只读 `log`、`px4` 节，
以及 stage4 / stage4-mission 用到的控制相关配置节（`controller`/`handover`/
`visual_tracking`/`target_estimator`/`actuator` 等）——**这些节在 `config/config.json`
与 `config/px4_sitl.json` 中均可缺席**（缺席时用 `config/config.h` 默认值，stage4 缺省
阈值即默认值），其余节忽略。stage4-mission 需要 `config/` 目录同级的双目校正矩阵
（读 `fxb_m_px`，路径取 `stereo_ranger.rectify_matrices_path`）。

波特率必须与 PX4 对应 MAVLink 实例一致。若实机不是 115200，只修改 JSON 后重新运行，不改代码。

## 4. 上板前检查

### 4.1 设备与权限

```bash
ls -l /dev/ttyS1
groups
```

当前用户应有串口读写权限，通常属于 `dialout` 组。若不属于：

```bash
sudo usermod -aG dialout "$USER"
```

随后注销并重新登录。不要长期使用 sudo 运行测试，否则日志目录可能产生 root 所有文件。

### 4.2 检查串口是否被系统控制台占用

```bash
sudo lsof /dev/ttyS1
systemctl status serial-getty@ttyS1.service
```

只有确认 `serial-getty@ttyS1` 正在占用该 PX4 专用串口时，才执行：

```bash
sudo systemctl disable --now serial-getty@ttyS1.service
```

### 4.3 硬件安全

- 首次联调拆除螺旋桨；
- 确认香橙派 UART 与 Pixhawk TELEM 接口均为兼容 TTL 电平；
- TX/RX 交叉连接并共地；
- 不把 RS-232 电平直接接到 TTL UART；
- 遥控器保持可用，PX4 不允许自动解锁。

## 5. 输出与退出码

运行中摘要示例：

```text
[3s] connected=是 armed=否 mode=3/0 landed=是 gps=3D+
     global=是 local=是 attitude=是 battery=是 home=是 rc=是
```

最终报告：

```text
[PASS] PX4 HEARTBEAT
[PASS] 测试结束时心跳仍连接
[PASS] 遥测请求 COMMAND_ACK
[PASS] AUTOPILOT_VERSION 收到 1 条
[PASS] ATTITUDE
...
结果: 基础 PX4 串口链路通过
```

退出码：

| 退出码 | 含义 |
|---|---|
| 0 | PX4 心跳建立、结束时仍连接且链路错误为 0 |
| 1 | 参数、配置文件或 JSON 解析失败 |
| 2 | 串口打开/线程启动失败 |
| 3 | 未收到 PX4 心跳、结束时断开或链路出现错误 |

缺少某一遥测消息只记 WARN，不直接判定基础串口失败。它通常表示 PX4 对应 MAVLink 实例没有
配置该消息流，后续需要设置流频率或由 `Px4Link` 主动请求。

## 6. 日志行为

- 控制台：每秒状态摘要、状态变化、最终验收报告；
- 文件日志：沿用 `config.log` 配置和工程异步日志；
- 不逐条打印高频姿态/位置消息；
- `Px4Link` 仅记录生命周期、连接变化、版本首次上报和节流错误；命令拒绝/失败使用 WARN；
- SITL 飞行阶段仅打印状态变化、阶段切换、安全回收和最终验收，不逐设定值打印；
- stage4 期间控制器的控制权迁移（IDLE/ACTIVE/COAST 及原因串）由 `FlightController`
  自身 WARN 日志记录，工具侧只打印阶段切换与实测值；
- stage4-mission 每秒打印一行任务摘要（状态机状态/告警位/注入目标/视觉注入状态/
  卡尔曼估计/控制权），阶段切换与验收证据打印实测值；状态机/估计器/控制器的业务
  日志按各自模块纪律记录，工具不逐条转发。

## 7. 排查要点

| 现象 | 排查方向 |
|---|---|
| 无法打开 `/dev/ttyS1` | 设备节点、dialout 权限、serial-getty/lsof 占用 |
| 串口打开但 HEARTBEAT FAIL | TX/RX、共地、波特率、PX4 MAVLink 端口、target ID |
| HEARTBEAT 反复断开 | 接线/电平、波特率、心跳频率、系统负载 |
| 只有 HEARTBEAT，其他全 WARN | PX4 MAVLink stream 配置或尚未主动请求消息 |
| 版本 WARN | PX4 未主动发送 AUTOPILOT_VERSION；后续命令阶段主动请求 |
| 收到消息但数值无效 | GPS/估计器/电池/RC 本身未就绪，检查 valid 标志而非默认零值 |
| 退出码 3 且错误计数增加 | 查看 `logs/` 中串口/UDP read/write 节流错误 |
| 起飞阶段拒绝启动 | 必须使用 `config/px4_sitl.json` 且 duration≥20 |
| ARM ACK denied | 检查 PX4 preflight 和 QGC/GCS 连接，禁止 force arm |
| 高度/漂移安全回收 | 查看最大相对高度、水平漂移和 local position；程序会优先请求 AUTO.LAND |
| AUTO.LAND 后未 landed | 检查 mode 4/6、land detector 和位置估计；未 landed 时程序不会空中 DISARM |
| 水平运动阶段安全回收 | 北向位移超过 1.50m、东西偏差超过 0.30m，或相对高度离开 0.65~1.30m |
| 水平运动位移验收失败 | 制动完成时北向位移必须在 0.70~1.30m；速度命令结束瞬间位移仅用于观察加速响应 |
| 停止 setpoint 后仍为 6/0 | 检查 `COM_OF_LOSS_T` 和实际 setpoint 发送计数；本工具最长等待 10 秒 |
| Offboard loss 保护模式与预期不同 | 对照运行前记录的 `COM_OBL_RC_ACT` 和 RC 是否存在，不在工具内猜测或改参数 |
| stage4 引导阶段超时 | 引导流是否 ≥2 秒且 ≥20 条、Offboard ACK 是否 accepted、PX4 是否真实 6/0；查日志中控制器迁移原因串 |
| stage4“控制器未接管设定值流” | 控制器授权链未全通：查 `controller.reason_code` bit31 是否置位、快照是否 armed+6/0（工具每拍带请求位） |
| stage4 起飞阶段超时 | 位置引导意图被拒（`home_valid=false` → 无 Home 基准）或 `local_position` 无效；报告打印控制器 rejected 计数 |
| stage4 刹车/退出时延超标 | 刹车过零上限 2000ms（指数收敛物理下限 ≈1.4s + 量化余量）、退出上限 2200ms（COM_OF_LOSS_T 1.0s + HEARTBEAT 1Hz 相位）；超标先看 PX4 `COM_OF_LOSS_T` 是否被改大，再对照报告打印的两段实测值 |
| stage4 无 SITL 时 | 10 秒连接超时后中止并 FAIL（退出码 3）；若退出码 1，先看参数互斥/时长/transport 门禁提示 |
| stage4-mission 退出码 1 且报"注入几何自检失败" | 反变换/正变换不互逆（代码级缺陷）：查 `NedToCameraFrame`/`OffsetToGeodetic` 与 `visual_ned_converter.cpp`/`geodetic_converter.cpp` 是否同步改动 |
| stage4-mission 卡在"交接段超时" | 按状态机排查要点四项逐项查（`include/state_machine/mission_state_machine.md`"停在 S5 不进 S6"）；工具侧另看：注入目标是否新鲜（Home 有效）、测距注入是否在跑、姿态/位置遥测是否有效、估计器诊断是否注入（本工具已注入真实估计器） |
| stage4-mission 卡在"S5 位置引导段超时" | 状态机未出位置意图：查估计是否有效（地面站目标是否被注入、Home 是否有效）、`TargetState.valid`、`reason` 低位=7 的意图是否出现（工具据此停止注入引导意图） |
| stage4-mission 卡在"围栏探针段超时" | 探针目标 150m 是否 > `controller.max_radius_m`（默认 100m；若配置把围栏放宽到 ≥150m 则无钳制可验）；控制器拒发位置意图（`home_valid=false`）时同样无钳制 |
| stage4-mission 卡在"切非 Offboard 停发补验" | AUTO.LOITER 是否被 ACK accepted、真实模式是否离开 6/0（快照 `flight_mode/flight_sub_mode`）、控制器是否仍 ACTIVE（授权链快照门禁） |
| stage4-mission 模式停发时延超标（>200ms） | 测量起点是**工具首个非 Offboard 快照**，终点是控制器转 IDLE；超标多为控制周期/快照发布周期设置过大（`controller.period_ms`、`px4.state_publish_interval_ms`），非状态机问题 |
| stage4-mission"目标过期链"超时 | 过期判据依赖注入目标有效期（工具 2s）；看状态机告警位 bit1、控制器 COAST（意图超龄）与 IDLE（`coast_timeout`）、PX4 `COM_OF_LOSS_T` 是否与配置自洽 |

测试后请保存完整控制台输出和对应日志文件，供后续调整 PX4 1.17.0 消息流与串口参数。

## 8. 首次香橙派实测结果（2026-08-27）

```text
设备：/dev/ttyS1，115200，MAVLink 2
目标：system/component = 1/1
时长：30 秒
接收目标消息：763
ACK：首次版本尚未实现请求
链路错误：0
退出码：0
结果：基础 PX4 串口链路通过
```

- HEARTBEAT 全程稳定，实测模式 `4/3` 对应 PX4 1.17.0 `AUTO/LOITER`，飞控未解锁；
- 已收到 `EXTENDED_SYS_STATE`、`LOCAL_POSITION_NED`、`ATTITUDE`、`GPS_RAW_INT`；
- GPS 消息存在但没有 3D fix，因此 `GLOBAL_POSITION_INT`、Home 不可用符合当前台架条件；
- `AUTOPILOT_VERSION`、电池、Home、RC 未收到或未形成有效状态，需要主动请求或检查硬件；
- landed/local 偶尔显示未知，是消息间隔超过 `telemetry_timeout_ms=2000`，心跳并未断开。

下一轮 smoke 已具备 `MAV_CMD_REQUEST_MESSAGE`、`MAV_CMD_SET_MESSAGE_INTERVAL`、ACK
匹配/超时和每类 message ID 计数，可区分“消息未配置”“消息已收到但字段无效”和“解析缺陷”。

## 9. 第二次香橙派实测结果（2026-08-27）

```text
时长：30 秒
ACK 匹配：10
ACK 超时：0
sequence 1：REQUEST_MESSAGE(AUTOPILOT_VERSION)，result=0 ACCEPTED
sequence 2：REQUEST_MESSAGE(HOME_POSITION)，result=2 DENIED（当前无 GPS/Home）
sequence 3~10：SET_MESSAGE_INTERVAL，全部 result=0 ACCEPTED
AUTOPILOT_VERSION：1 条，版本 1.17.0，capabilities=59647
EXTENDED_SYS_STATE：17 条
LOCAL_POSITION_NED：85 条
ATTITUDE：86 条
GPS_RAW_INT：42 条，无 3D fix
SYS_STATUS/BATTERY_STATUS：18 条，但电池字段无效
GLOBAL_POSITION_INT / HOME_POSITION / RC_CHANNELS：未收到
链路错误：0
退出码：0
```

结论：PX4 1.17.0 接受版本请求和全部 8 条频率请求；Home 请求被明确拒绝，符合当前没有
GPS 3D fix、尚未建立 Home 的状态。可靠命令/ACK 链路实机通过；
落地和局部位置不再因 2 秒超时反复失效。无 GPS 3D fix 时全局位置/Home 缺失符合预期。
实机测试时未插动力电池、未开启遥控器，因此电池消息字段无效和 RC 未收到属于预期状态；
后续接入硬件后再验证。PX4 `mavlink status` 确认对应 instance #0 为 PX4 内部
`/dev/ttyS0 @115200`（对端香橙派 `/dev/ttyS1`），mode=Normal、tx rate max=1200 B/s、
rate multiplier=0.283，解释了实际消息频率约为请求值的三分之一。

## 10. 第三次香橙派实测结果：提高 MAVLink 带宽（2026-08-27）

将 PX4 参数 `MAV_0_RATE` 从 1200 调整为 0，使 PX4 按 115200 波特率自动使用约
`baudrate/20 = 5760 B/s` 上限。30 秒结果：

```text
EXTENDED_SYS_STATE：59 条（约 2 Hz）
LOCAL_POSITION_NED：294 条（约 10 Hz）
ATTITUDE：304 条（约 10 Hz）
GPS_RAW_INT：150 条（5 Hz）
SYS_STATUS/BATTERY_STATUS：60 条（两类各约 1 Hz）
总接收目标消息：1674
ACK 匹配：10
ACK 超时：0
链路错误：0
```

请求频率已全部达到预期，且 30 秒内状态有效性稳定。无全局位置/Home、无有效电池和无 RC
分别与无 GPS 3D fix、未插动力电池、未开启遥控器一致。当前 115200 串口带宽满足基础遥测，
PX4 接收与安全命令阶段验收完成。
