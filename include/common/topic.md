# Topic\<Message\> 发布订阅使用文档

> 位置：`include/common/topic.h`
> 命名空间：`drone::common`
> 适用范围：各模块线程间解耦通信的基础设施

## 1. 概述

`Topic<Message>` 是类型安全的有界发布订阅主题，用于跨线程传递消息。它借鉴了
PX4/ROS2 的发布订阅模型，但使用纯 C++17 标准库实现，不引入第三方依赖。

```text
发布线程（任意多个）                        消费线程（每个订阅者独占一个）
┌──────────────┐  Publish/Emplace   ┌──────────────────────────────┐
│ 摄像头采集线程 │ ─────────────────▶ │ Topic<DecodedFrame>          │
│ YOLO 识别线程  │                    │   ├─ 订阅者A（图传）独立有界队列 │
│ 状态机控制线程 │                    │   └─ 订阅者B（感知）独立有界队列 │
└──────────────┘                    └──────────────────────────────┘
                                          每个订阅者：TryTake / WaitTake / WaitTakeFor
```

典型用途：

- 摄像头解码线程发布视频帧句柄 → YOLO 识别线程订阅消费。
- YOLO 识别线程发布检测结果 → 图传线程、状态机线程分别订阅。
- PX4 通信线程发布遥测快照 → 状态机控制线程订阅。

## 2. 核心特性

| 特性 | 说明 |
|------|------|
| 编译期类型安全 | 消息类型由模板参数决定，禁止 `void*` 和无类型字节指针 |
| 零拷贝多播 | 消息以 `std::shared_ptr<const Message>` 分发，多订阅者共享同一对象，不复制消息 |
| 独立有界队列 | 每个订阅者拥有自己的队列，慢订阅者不阻塞发布者和其他订阅者 |
| 两种溢出策略 | 队列满时丢弃最旧（默认）或拒绝最新，见第 5 节 |
| RAII 生命周期 | 订阅句柄析构自动退订并清空队列，杜绝泄漏和悬挂 |
| 确定性停机 | `Close()` 唤醒所有等待线程，停机顺序可控 |

## 3. 快速开始

```cpp
#include "common/topic.h"

#include <chrono>
#include <memory>

using namespace drone::common;

// 1. 定义消息类型（建议放 include/common/types.h）
struct DroneState {
    std::uint64_t sequence = 0;
    double latitude = 0.0;
    double longitude = 0.0;
};

int main() {
    // 2. 创建主题
    Topic<DroneState> drone_state_topic;

    // 3. 订阅：队列容量 1，满时丢弃最旧（默认策略）
    auto controller = drone_state_topic.Subscribe(1);

    // 4. 发布：Emplace 就地构造，一次创建
    drone_state_topic.Emplace(DroneState{1, 30.0, 120.0});

    // 5. 消费：非阻塞读取
    auto message = controller.TryTake();
    if (message) {
        const DroneState& state = **message;  // MessagePtr -> * -> Message
        // ... 使用 state
    }
    return 0;
}
```

## 4. API 说明

### 4.1 主题侧（Topic）

| 方法 | 说明 |
|------|------|
| `Subscribe(capacity, policy)` | 创建订阅者。`capacity` 必须 > 0，否则抛 `std::invalid_argument`。返回可移动的 `Subscription` |
| `Publish(MessagePtr)` | 发布已由智能指针管理的消息；空指针或主题已关闭返回 `accepted=false` |
| `Emplace(args...)` | 用 `std::make_shared` 就地构造消息并发布，等价于 `Publish(make_shared(args...))` |
| `Close()` | 关闭主题，唤醒所有等待中的订阅者；已入队消息仍可取完。析构自动调用 |
| `IsClosed()` | 主题是否已关闭 |
| `SubscriberCount()` | 当前有效订阅者数量（自动清理失效条目） |

发布结果：

```cpp
Topic<int> topic;
auto sub = topic.Subscribe(2);
auto result = topic.Emplace(1);
// result.accepted          : 是否被接收
// result.subscriber_count  : 有效订阅者数
// result.delivered_count   : 实际投递数
// result.dropped_count     : 因队列满丢弃数
```

### 4.2 订阅者侧（Subscription）

| 方法 | 说明 | 适用场景 |
|------|------|----------|
| `TryTake()` | 非阻塞取一条消息，队列空返回 `std::nullopt` | 固定周期轮询（控制线程） |
| `WaitTake()` | 阻塞直到有消息或主题关闭 | 独占消费线程（PX4 串口、图传） |
| `WaitTakeFor(timeout)` | 限时等待，超时返回 `std::nullopt` | 带超时的消费循环（看门狗） |
| `IsOpen()` | 订阅是否仍有效 | 停机检测 |
| `PendingCount()` | 队列中待消费消息数 | 监控积压 |
| `DroppedCount()` | 因队列满累计丢弃数 | 监控慢消费者 |
| `Reset()` | 主动退订，幂等 | 提前结束订阅 |

注意：

- `Subscription` **不可复制**（避免多消费者竞争同一队列），可以**移动**给消费线程。
- 取到的是 `std::optional<MessagePtr>`，访问消息本体需要两次解引用：
  `**message`（或 `*message->get()`）。消息按 `const Message&` 使用，不可修改。

## 5. 溢出策略选择

```cpp
// 丢弃最旧（默认）：适合“只关心最新”的数据
auto video_sub = topic.Subscribe(2, Topic<Frame>::OverflowPolicy::kDropOldest);

// 拒绝最新：适合不能覆盖旧消息的事件/命令
auto cmd_sub = topic.Subscribe(8, Topic<Command>::OverflowPolicy::kRejectNewest);
```

| 策略 | 行为 | 推荐场景 |
|------|------|----------|
| `kDropOldest` | 队列满时丢弃队首最旧消息，写入新消息 | 视频帧、无人机最新状态、检测结果 |
| `kRejectNewest` | 队列满时丢弃新消息，保留旧消息 | 事件通知、控制命令、告警 |

无论哪种策略，**发布者都不会被阻塞**；丢弃通过统计计数暴露，便于监控。

## 6. 线程安全与使用约定

- 发布者：`Publish` / `Emplace` 可在任意线程并发调用。
- 订阅者：**每个 `Subscription` 只能被一个消费者线程使用**。若确有多个线程需要
  同一份数据，应为每个线程各自 `Subscribe`。
- 主题生命周期：主题对象需比所有订阅者更长寿。订阅句柄持 `weak_ptr`，即使主题
  已析构，句柄上的读取操作也安全（返回空）。
- 消息不可变：跨线程消息一律视为 `const`，确需修改时创建新的输出消息。

## 7. 性能与内存

- 多订阅者共享同一 `MessagePtr`，不复制消息对象；发布 N 个订阅者的开销近似 O(N)
  次智能指针拷贝 + 入队。
- 发布者持锁时间仅覆盖“入队”，不执行任何消费者回调，慢消费者不会反压发布者。
- 视频帧场景：内存池返回的 RAII 帧句柄可直接作为消息发布，多个订阅者共享句柄，
  底层缓冲在最后一个消费者释放时自动归还内存池（见 `docs/项目文件结构.md` §5）。

```cpp
// 视频帧句柄作为消息的示意（视频模块实现后可用）
struct FrameHandle {
    // 内部持有内存池的槽位，析构时自动归还
    std::shared_ptr<void> buffer;
    std::uint64_t sequence = 0;
    std::int64_t timestamp_ms = 0;
    int width = 0;
    int height = 0;
};

Topic<FrameHandle> decoded_frame_topic;
auto detector = decoded_frame_topic.Subscribe(2);      // 感知消费
auto sender = decoded_frame_topic.Subscribe(2);        // 图传消费

// 发布时只移动句柄，不拷贝像素数据
decoded_frame_topic.Emplace(FrameHandle{/* 内存池槽位 */});
```

## 8. 停机流程

推荐逆序停机：先停生产者，再 `Close()` 主题，最后 join 消费者线程。

```cpp
// 1. 停止生产者（例如置位原子退出标志）
producer_running.store(false);

// 2. 关闭主题，唤醒所有 WaitTake 中的消费者
decoded_frame_topic.Close();

// 3. 消费者线程在 WaitTake 返回 std::nullopt 后自然退出
consumer_thread.join();
```

- `Close()` 后已入队消息仍可被取完，消费者能处理完手头数据再退出。
- 主题析构自动 `Close()`，但显式关闭可保证停机顺序可控。

## 9. 单元测试

测试文件：`tests/common/topic_test.cpp`，覆盖：

- 同一条消息实例分发到所有订阅者（零拷贝验证）。
- 丢弃最旧 / 拒绝最新两种溢出策略。
- 关闭主题唤醒等待者并拒绝新消息。
- RAII 退订（订阅者析构后 `SubscriberCount` 归零）。
- 容量为 0 的非法订阅抛出 `std::invalid_argument`。

构建并运行：

```bash
cmake -B build
cmake --build build
ctest --test-dir build
```

## 10. 常见问题

| 问题 | 原因与处理 |
|------|-----------|
| 取到的消息和发布内容对不上 | 队列满丢弃了中间消息；对“最新状态”类数据属正常行为，可增大容量或检查 `DroppedCount()` |
| 消费者线程无法退出 | 队列为空且主题未关闭，`WaitTake` 一直阻塞；停机时务必先 `Close()` |
| 想要多个线程消费同一数据 | 每个线程各自 `Subscribe`，不要共享同一个 `Subscription` |
| `Subscribe(0)` 抛异常 | 容量必须大于 0；容量 0 说明设计错误（永远收不到消息） |
