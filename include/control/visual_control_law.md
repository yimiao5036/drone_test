# 视觉控制段抽象（visual_control_law）

> 对应实现：`include/control/visual_control_law.h`、`src/control/visual_control_law.cpp`
> 使用方：`MissionStateMachine`（S7 视觉伺服，经接口每拍 `Update`）
> 相关：`include/control/tracking_control_law.h`（当前唯一实现）

## 1. 功能边界

把 S7 视觉伺服的**控制律**抽成接口，按“拦截剖面”在装配期选择实现：

| 剖面 | 当前状态 | 说明 |
|---|---|---|
| `net_capture` | 已实现 | 捕网：保持距离 + 瞄准，复用 `TrackingControlLaw` |
| `jammer` | 已实现（共用） | 干扰：与捕网同构，参数按本机配置（保持距离/伴飞） |
| `impact` | **预留** | 撞击：全速穿越、不保持距离；工厂 `fail-fast` |

**不做什么**：不包 S6/S7 进入/退出编排、不包 S9/S10 拦截动作、不做寻址。
这些仍由 `MissionStateMachine` 与 `IInterceptionDevice` 负责。

## 2. 接口与数据流

```cpp
enum class VisualProfile : std::uint8_t { kNetCapture, kJammer, kImpact };

class IVisualControlLaw {
    virtual ControlOutput Update(const ControlSnapshot&, std::int64_t now_ms) = 0;
    virtual void Reset() = 0;
    virtual VisualProfile Profile() const = 0;
};

std::unique_ptr<IVisualControlLaw> CreateVisualControlLaw(
    VisualProfile profile, const TrackingConfig& config);
```

数据流（与 `TrackingControlLaw` 一致）：

```text
VisualTargetStatus / StereoTargetDistance / FlightStateSnapshot
  → ControlSnapshot
  → IVisualControlLaw::Update
  → ControlOutput（局部 NED 速度 + 航向/角速度 + 模式/降级原因）
  → MissionStateMachine::BuildVisualIntent → ControlIntent
```

## 3. 关键实现点

- **装配期选择**：`DroneApplication` 按本机功能类别映射剖面
  （25=捕网、27=干扰、26=撞击），控制回路不查询飞机身份；身份只用于寻址。
- **fail-fast**：`impact` 剖面工厂直接抛 `std::invalid_argument`，不允许撞击机
  静默套用捕网“保持距离”律；未知枚举值同样抛异常。
- **共用实现**：`TrackingControlLaw` 实现本接口，构造期接收剖面仅作日志/诊断
  标注，不改变数学；捕网与干扰的差异全部来自各自 `visual_tracking` 配置。
- **单线程**：与状态机决策线程同线程调用，实现不得内部起线程或做堆分配。

## 4. 日志行为

- 状态机创建日志打印 `servo_profile={}`（剖面名）与伺服频率；
- 工厂本身不打日志；非法剖面由装配层异常上报（启动失败）。

## 5. 测试方式

```bash
cmake --build build -j$(nproc) --target visual_control_law_test
./build/visual_control_law_test
ctest --test-dir build -R VisualControlLaw --output-on-failure
```

覆盖：捕网/干扰共用实现且剖面正确；撞击剖面 fail-fast；剖面名映射。

## 6. 排查/修改要点

- 新增剖面：在 `VisualProfile` 加枚举、工厂加分支、装配层加映射，
  并补 `visual_control_law_test`；
- 撞击实现落地时：新增独立控制律类（不修改 `TrackingControlLaw` 数学），
  工厂返回新实现；S6 交接判据与 S7 退出判据需同步按剖面区分
  （当前交接/退出仍按捕网语义，见 `docs/追踪算法-接近段速度律实现说明-2026-11-11.md`）。
