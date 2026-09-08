# 健康管理部件实现文档

## 一、功能职责与边界

`IHealthManager` 是系统健康管理与故障汇总接口，负责把摄像头、PX4、地面站、电源、解码器、NPU/YOLO、激光雷达和图传等模块的运行状态，统一转换为可供状态机和地面站消费的 `HealthStatus` 快照。

它解决的问题是：**系统当前是否仍具备执行某一阶段任务的条件，以及故障或数据超时发生在哪里**。它不是单个硬件驱动，也不是控制器，不直接修复故障或发送飞行命令。

### 1.1 负责的功能

- 注册需要监控的数据源及其最大允许数据年龄 `max_age_ms`；
- 接收各模块上报的最近数据到达时间；
- 按单调时钟判断数据是否新鲜，识别链路断开、设备异常和数据超时；
- 汇总链路健康位、设备健康位、数据新鲜度位、错误位和算力板负载；
- 周期性发布 `common::HealthStatus`；
- 统计数据超时事件和内部错误次数，供状态机和诊断工具观察；
- 为状态机提供任务相关的安全门禁输入，例如“PX4遥测是否新鲜”“视频/NPU是否可用”。

### 1.2 不负责的功能

- 不打开或读取串口、摄像头、雷达等硬件；硬件通信由对应模块负责；
- 不替代 `Px4Link`、`GroundStationLink` 或 `VideoSender` 的连接状态维护；这些模块提供原始状态，HealthManager 负责统一汇总；
- 不执行重连、重启服务、切换飞行模式、ARM/DISARM、返航或降落；
- 不根据单次异常直接决定任务状态，状态转移由 `MissionStateMachine` 结合任务阶段和健康快照完成；
- 不直接发送 `Px4Setpoint` 或任何飞行控制命令；
- 不把健康位伪装成详细诊断数据。详细错误上下文仍应由产生错误的模块记录并按需提供。

## 二、接口与数据流

### 2.1 接口

```cpp
class IHealthManager {
public:
    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual bool IsRunning() const = 0;

    virtual bool RegisterSource(const std::string& name,
                                uint64_t max_age_ms) = 0;
    virtual void ReportData(const std::string& name,
                            uint64_t receive_time_ms) = 0;

    virtual common::Topic<common::HealthStatus>& Output() = 0;
    virtual uint64_t TimeoutEventCount() const = 0;
    virtual uint64_t ErrorCount() const = 0;
};
```

### 2.2 输入与输出

```text
CameraReceiver / VideoDecoder / YoloDetector / VideoSender
Px4Link / GroundStationLink / LaserRangeFinder / 电源监测
        │
        │ RegisterSource + ReportData + 模块错误/状态
        ▼
   IHealthManager
        │ 周期性汇总
        ▼
Topic<common::HealthStatus>
        ├── MissionStateMachine：任务安全门禁和降级判断
        └── GroundStationLink：后续健康状态回传
```

`HealthStatus` 当前包含：

| 字段 | 含义 |
|------|------|
| `link_health_bits` | 链路健康位：bit0 摄像头、bit1 PX4、bit2 地面站电台、bit3 激光雷达、bit4 图传 |
| `device_health_bits` | 设备健康位：bit0 解码器、bit1 NPU/YOLO、bit2 电源A、bit3 电源B |
| `data_freshness_bits` | 数据新鲜度位，与链路位顺序一致；`0` 表示新鲜，`1` 表示超时 |
| `error_bits` | 模块错误汇总位，具体位定义需随正式实现冻结，不得在不同模块重复使用同一含义 |
| `cpu_load_pct` | 算力板负载百分比；无有效采样时不得伪造为健康值 |

当前实现冻结位语义：健康位中 `1` 表示对应数据源已注册且数据新鲜，`0` 表示未注册或不可用；`data_freshness_bits` 中 `1` 表示对应链路数据超时，`0` 表示未超时。未注册的源不会被当作故障。`error_bits` 和 `cpu_load_pct` 暂无上报接口，当前保持默认值，后续扩展时必须同步更新公共类型和地面站协议。

### 2.3 Topic 与生命周期

- 输出 Topic 类型：`common::Topic<common::HealthStatus>`；
- Topic 名称：`common::topics::kHealthStatus`；
- 所有数据源必须在 `Start()` 前完成注册和绑定；
- 停止时先停止产生数据的模块，再停止 HealthManager 消费/汇总线程；
- `Start()`、`Stop()` 必须幂等；
- HealthManager 不应因为单个数据源失效而阻塞其他数据源或阻止系统安全停机。

## 三、关键实现点

### 3.1 新鲜度判断

- 所有超时判断使用算力板单调时钟，不使用会回拨的墙上时间；
- `ReportData()` 只记录最近一次有效到达时间，不能把发送时间误当作接收时间；
- `now - receive_time_ms > max_age_ms` 时置对应新鲜度超时；
- 时间戳为零、时间回拨或明显非法时按“不可信/超时”处理，并累计错误；
- 超时事件应按“进入超时状态”计数，不能每个周期重复累计同一个持续超时事件。

### 3.2 健康汇总与任务决策解耦

HealthManager 只发布事实快照，例如 PX4 数据已超时、YOLO 最近没有有效结果或地面站已断联；它不直接把这些事实转换为 `RETURN_HOME`、`LANDING` 等任务状态。状态机需要结合当前任务阶段决定严重程度：例如 GPS 接近阶段的视频故障可能只是降级，而视觉跟踪阶段的视频故障可能必须退出视觉控制。

### 3.3 数据源命名

正式实现应集中定义数据源名称和位映射，避免各调用方散落字符串。建议至少覆盖：

```text
camera / px4 / ground_station / laser_range / video
video_decoder / yolo / power_a / power_b
```

具体名称、最大年龄和错误位必须与配置、模块实现和地面站协议同步，不能只修改其中一处。

### 3.4 当前实现状态

当前仓库已经实现 `HealthManager` 真实类，保留 `HealthManagerStub` 供骨架冒烟测试：

- 支持 9 个标准数据源名称和链路/设备位映射；
- `RegisterSource()` 只允许在启动前注册，拒绝未知名称、重复注册和非法超时；
- `ReportData()` 保存最近接收时间，收到恢复数据时立即请求发布快照；
- 独立监控线程默认每 100ms 检查一次新鲜度；
- 健康位、新鲜度位、状态序号和超时事件计数已经实现；
- 状态进入超时和恢复时记录一次状态变化日志；
- `error_bits` 和 `cpu_load_pct` 尚未接入真实来源，当前保持默认值；
- `DroneApplication` 已创建真实 HealthManager、接入状态机并绑定地面站 HealthStatus 输入 Topic；应用级监控线程把视频、PX4、地面站、解码器、YOLO和图传计数变化转换为 `ReportData()`；地面站健康状态MAVLink下行协议尚未编码；
- `error_bits` 和 `cpu_load_pct` 尚未接入真实来源，激光雷达、电源等模块尚未装配。

`HealthManagerStub` 仍固定返回未实现，用于验证旧接口和骨架生命周期，不应与真实类混用。

## 四、日志行为

遵循项目日志纪律：

- INFO：HealthManager 创建、启动、停止、销毁；
- INFO/WARN：数据源首次进入超时或从超时恢复时记录一次状态变化；
- WARN：单个数据源异常降级、注册参数不合法；
- ERROR：重复注册、非法时间戳、内部线程或 Topic 错误；
- 高频 `ReportData()` 和每周期健康汇总不打印日志；
- 持续异常按第 1 次和每满 100 次节流，日志中带累计次数和数据源名称。

## 五、测试方式

当前测试：

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

`tests/health/health_manager_test.cpp` 当前覆盖：

- 合法/非法、未知、重复和运行中数据源注册；
- 数据新鲜、无数据超时、超时恢复；
- 链路位、设备位和新鲜度位映射；
- 未注册数据和未来时间戳拒绝；
- 启停、重复启停和重新启动。

后续还需补充：

- 多数据源独立超时的长期运行测试；
- Topic 队列满时不阻塞数据源上报；
- `error_bits` 和 CPU 负载来源接入；
- 状态机消费健康快照后的安全降级行为。

硬件验证应在香橙派上进行：拔断 PX4、地面站、摄像头或图传链路，分别确认对应健康位和新鲜度位变化，且其他独立模块仍可运行；恢复链路后确认状态恢复，不能发送任何未授权飞行控制命令。

## 六、排查与修改要点

- 看到“健康超时”时，先确认对应模块是否调用 `ReportData()`，再检查接收时间是否使用单调时钟；
- 如果所有数据源同时超时，优先检查 HealthManager 线程、Topic 绑定或系统单调时钟，而不是逐个怀疑硬件；
- 如果只有 `data_freshness_bits` 变化，说明链路可能仍在线，但业务数据没有按期到达；不能简单等同于串口断开；
- 修改位定义时必须同步更新 `include/common/types.h`、`docs/数据接口文档.md`、本实现文档、状态机规则和地面站协议；
- 修改 `DroneApplication` 健康接线时，必须先完成 Topic 绑定，再启动状态机、地面站消费者和 HealthManager；
- 在 `enable_control=false` 阶段，健康状态只能用于观察和影子决策，不能触发任何 PX4 控制命令。
