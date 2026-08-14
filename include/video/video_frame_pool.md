# 视频帧内存池（video_frame_pool）

> 对应实现：`include/video/video_frame_pool.h`、`src/video/video_frame_pool.cpp`
> 配套：帧句柄 `include/video/video_frame.h` + 实现文档 `video_frame.md`

## 1. 功能职责

固定容量视频帧内存池，管理帧缓冲的分配与回收：

- 构造时一次性分配 `容量 × 槽位大小` 连续内存（默认 64 字节对齐，DMA/RGA 友好）。
- `Acquire()` 单生产者取帧；`Recycle()` 多消费者归还（实现 `FrameRecycler`）。
- 池耗尽直接丢帧（返回空句柄），**不阻塞、不抛异常**。

不做什么：不负责帧内容填充（由采集/解码线程写入），不负责 Topic 分发。

## 2. 接口与数据流

```cpp
class VideoFramePool final : public FrameRecycler,
                             public std::enable_shared_from_this<VideoFramePool> {
public:
    static constexpr std::size_t kDefaultAlignment = 64;

    VideoFramePool(std::size_t capacity, const VideoFrameInfo& frame_template,
                   std::size_t alignment = kDefaultAlignment);  // 非法参数抛异常

    static std::size_t ComputeBufferSize(const VideoFrameInfo& info) noexcept;

    [[nodiscard]] FrameHandle Acquire() noexcept;  // 池空返回空句柄，计 DroppedCount
    void Recycle(std::uint32_t slot_index) noexcept override;

    std::size_t Capacity() const;   // 槽位数
    std::size_t SlotSize() const;   // 每槽位字节数
    std::size_t IdleCount() const;  // 空闲槽位
    std::size_t InFlightCount() const;  // 在途槽位
    std::uint64_t AcquiredCount() const;
    std::uint64_t RecycledCount() const;
    std::uint64_t DroppedCount() const;          // 池满丢帧
    std::uint64_t DuplicateRecycleCount() const; // 非法归还
};
```

数据流：`Acquire() → FrameHandle（持有池 shared_ptr）→ 发布 → 消费者 → 最后引用析构 → Recycle()`。

## 3. 关键实现点

- **容量规划**：池容量 ≥ 各订阅队列容量之和 + 在途处理帧数；否则解码快于消费时必然丢帧。
- **`buf_size` 自动推算**：未显式给出时按格式算（NV12 = `hor_stride×ver_stride×3/2`，RGB888 = ×3）。
- **生命周期闭环**：池用 `shared_ptr` 创建（`enable_shared_from_this`），在途 `FrameHandle` 持有池引用，
  用户侧释放后池存活至全部缓冲归还，杜绝悬垂。
- **防御**：越界/重复归还被忽略并计入 `DuplicateRecycleCount`（节流 ERROR 日志）。
- **线程安全**：互斥锁保护空闲栈，临界区 O(1)；`Acquire` 单生产者约定，`Recycle` 任意消费者线程可并发。

## 4. 日志行为

| 等级 | 场景 |
|------|------|
| INFO | 池创建（容量/槽位/总内存/格式）、池销毁（累计获取/归还/丢帧/非法归还） |
| WARN（节流） | 池满丢帧（第 1 次 + 每满 100 次，带累计计数） |
| ERROR（节流） | 非法归还（越界/重复）、构造参数错误 |

## 5. 测试方式

`tests/video/video_frame_pool_test.cpp`（9 个用例）：

```bash
cmake --build build && cd build && ctest -R VideoFramePoolTest
```

覆盖：缓冲大小计算、构造校验、Acquire 元数据、池满丢帧不阻塞、归还复用、重复/越界归还忽略、
对齐与非重叠、在途保活、并发归还线程安全。

## 6. 排查/修改要点

| 现象 | 排查方向 |
|------|----------|
| `DroppedCount` 持续增长 | 池容量 < 订阅队列容量之和 + 在途；或消费者处理过慢 |
| `DuplicateRecycleCount` 增长 | 逻辑缺陷：同一槽位归还两次（检查句柄拷贝/移动是否违规） |
| 想改对齐 | `alignment` 必须是 2 的幂；DMA 场景保持 64 |
| 解码器用池 | 参考 `VideoDecoderConfig.pool_capacity`（见 `video_decoder.md`） |
