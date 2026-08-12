# 视频帧句柄（video_frame）

> 对应实现：`include/video/video_frame.h`
> 配套：内存池 `include/video/video_frame_pool.h` + 实现文档 `video_frame_pool.md`

## 1. 功能职责

定义视频帧的**数据格式与 RAII 缓冲句柄**，让视频帧以"句柄"形式在 `Topic` 中零拷贝传递：

- 采集线程从内存池 `Acquire()` 得到可写句柄 → 写像素数据 → 发布（移动句柄）。
- 消费线程收到只读句柄，多个订阅者共享同一份缓冲；**最后一个引用释放时自动归还内存池**。

不做什么：不分配像素内存（由内存池统一分配），不涉及解码/采集逻辑。

## 2. 接口与数据流

```cpp
// 像素格式
enum class PixelFormat { kUnknown, kYuv420SpNv12, kRgb888 };
constexpr std::size_t BytesPerPixel(PixelFormat format) noexcept;

// 帧元数据（发布后不可修改）
struct VideoFrameInfo {
    std::uint32_t width, height;       // 有效像素尺寸
    std::uint32_t hor_stride, ver_stride;  // 水平 stride（像素）、垂直 stride（行）
    std::size_t buf_size;              // 缓冲实际占用字节（含对齐填充）
    PixelFormat format;
    std::uint64_t sequence;            // 递增帧序号
    std::int64_t timestamp_ms;         // 单调时钟时间戳（超龄过滤依据）
    std::int64_t source_timestamp_ms;  // 源端时间戳（可选，如 RTP）
};

// 归还接口（内存池实现），FrameBuffer 析构时调用
class FrameRecycler { virtual void Recycle(std::uint32_t slot_index) noexcept = 0; };

// 帧缓冲本体：持有数据指针、元数据与归还回调（shared_ptr 共享）
class FrameBuffer;

// 帧句柄：禁拷贝、可移动；发布即移动，源句柄变空
class FrameHandle {
    bool Valid() const;                    // 是否持有有效缓冲
    const VideoFrameInfo& Info() const;    // 元数据（空句柄返回空元数据）
    std::byte* Data();                     // 像素缓冲（可写）
    const std::byte* Data() const;         // 像素缓冲（只读）
    std::size_t Capacity() const;
    void Reset() noexcept;                 // 立即归还（幂等）
};
```

数据流：`VideoFramePool.Acquire() → FrameHandle(可写) → Publish(移动) → Topic<FrameHandle> → 消费者共享 → 最后引用释放 → Recycle()`。

## 3. 关键实现点

- **禁拷贝可移动**：同一帧的写权限只属于一个句柄，从机制上杜绝一帧被两个生产者写入。
- **发布即移动**：`Emplace(std::move(handle))` 后源句柄变空。
- **stride 单位约定**（对齐 MPP/RGA）：`hor_stride` 为像素、`ver_stride` 为行；
  `LineSizeBytes() = hor_stride × BytesPerPixel(format)`。
- 时间戳统一单调时钟毫秒，用于帧率统计与超龄过滤；源端时间戳（RTP）单独存放，不参与超龄判断。

## 4. 日志行为

纯数据类型，无日志。

## 5. 测试方式

`tests/video/video_frame_test.cpp`（7 个用例）：

```bash
cmake --build build && cd build && ctest -R VideoFrameTest
```

覆盖：BytesPerPixel、LineSizeBytes、空句柄安全读、Info/Data 暴露、可移动不可拷贝、
最后引用释放自动归还、Reset 幂等、无 recycler 时跳过归还。

## 6. 排查/修改要点

| 现象 | 排查方向 |
|------|----------|
| 句柄空（`Valid()==false`） | 池耗尽（Acquire 失败）或已移动（发布后源句柄） |
| 帧数据对不上 | 检查 stride 是否按像素/行正确设置（`hor_stride ≥ width`） |
| 想加新像素格式 | 枚举加值 + `BytesPerPixel` 分支 + `VideoFramePool::ComputeBufferSize` 分支 |
| 时间戳异常 | `timestamp_ms` 由池在 Acquire 时打点；`source_timestamp_ms` 由生产者设置 |
