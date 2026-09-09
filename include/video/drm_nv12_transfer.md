# DRM NV12 DMA-BUF转存

> 对应实现：`include/video/drm_nv12_transfer.h`、`src/video/drm_nv12_transfer.cpp`

## 功能职责

把FFmpeg/rkmpp输出的单对象、线性、连续NV12 DMA-BUF通过RGA硬件复制到现有CPU可访问NV12目标缓冲，供`VideoFramePool`、YOLO、叠加和H.264编码链路继续使用。该模块用于绕过实测会从约2ms恶化到10～15ms的`av_hwframe_transfer_data`。

不负责：压缩码流解码、DMA-BUF跨线程生命周期、完整端到端零拷贝、AFBC/非线性modifier、多对象或非NV12布局、双目同步与深度计算。

## 接口与数据流

```cpp
struct DrmNv12Layout {
    int fd;
    std::size_t object_size;
    std::uint64_t format_modifier;
    std::uint32_t drm_format;
    std::uint32_t width;
    std::uint32_t height;
    int y_object_index;
    std::ptrdiff_t y_offset;
    std::ptrdiff_t y_pitch;
    int uv_object_index;
    std::ptrdiff_t uv_offset;
    std::ptrdiff_t uv_pitch;
};

class IDrmNv12Transfer {
    virtual bool IsAvailable() const = 0;
    virtual DrmNv12TransferStatus CopyToNv12(
        const DrmNv12Layout&, std::byte* destination,
        std::uint32_t destination_stride, std::string* error) = 0;
};
```

数据流：

```text
AVDRMFrameDescriptor
→ DrmNv12Layout校验
→ RGA wrapbuffer_fd
→ RGA imcopy
→ wrapbuffer_virtualaddr(VideoFramePool)
→ FrameHandle发布
```

## 关键实现点

- 当前适配依据Orange Pi实测布局：1280×720、一个DRM对象、一个图层、`DRM_FORMAT_NV12`、modifier=0、Y/UV同fd、pitch均1280、UV offset=921600。
- `ValidateLinearDrmNv12Layout()`严格校验线性modifier、NV12 fourcc、对象索引、offset、pitch、偶数尺寸和对象容量；不匹配时禁止按紧凑NV12猜测读取。
- RGA源使用`wrapbuffer_fd(fd, width, height, RK_FORMAT_YCbCr_420_SP, pitch, height)`；目标使用`wrapbuffer_virtualaddr`包装内存池槽位；`imcopy`同步返回前保持源`AVFrame`有效。
- `video_latency_probe`优先启用DMA路径；正式`drone_control`中的`prefer_rga_dma_transfer`默认false，上板验收通过前不改变正式路径。
- RGA不可用、布局不支持或复制失败时自动回退`av_hwframe_transfer_data`，不因试验路径中断视频。
- 该方案消除FFmpeg硬件帧下载和后续CPU逐行复制，但目标仍是CPU可访问内存池，不等同于解码到NPU/编码器的完整零拷贝。

## 日志行为

- 第一次RGA DMA成功：INFO，记录fd、分辨率、源pitch和目标stride。
- RGA不可用/布局不支持/复制失败：WARN，第1次及每100次节流，并记录回退原因和累计次数。
- DRM对象和各平面布局由`VideoDecoder`每个会话只打印一次INFO。
- 高频成功帧不打印日志；探针通过`D4R`固定窗口统计观察耗时。

## 测试方式

开发机：

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
ctest --test-dir build -R DrmNv12TransferTest --output-on-failure
```

`tests/video/drm_nv12_transfer_test.cpp`覆盖实测布局通过、非线性modifier、错误UV offset和对象容量不足。开发机无librga，不执行真实DMA复制。

香橙派：

```bash
./build/video_latency_probe --duration 600 --interval 10
```

预期：`RGA_DMA成功`持续增长，`RGA_DMA回退=0`，D4/D5无样本，D4R稳定，视频颜色/尺寸/检测框正常。

## 排查/修改要点

- RGA首次即失败：检查CMake是否显示`RGA DMA-BUF转存: ON`、librga版本、DRM fourcc/modifier和`wrapbuffer_fd`参数。
- 画面偏色：重点核对NV12/NV21格式、UV offset、pitch和目标stride。
- 图像上下错位：核对source hstride；当前实测UV offset恰好等于`pitch*height`。
- 回退计数增长：查看WARN中的状态和原因；不应为了强行使用DMA放宽布局校验。
- fd生命周期：RGA同步调用返回前必须持有源`AVFrame`；若未来异步传递DMA-BUF，必须用RAII持有`av_frame_ref`。
- 后续全链路零拷贝需要新的DMA帧句柄，并分别适配RGA预处理、叠加和MPP编码，不在本模块范围内。
