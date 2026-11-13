# DRM NV12/NV16 DMA-BUF转存

> 对应实现：`include/video/drm_nv12_transfer.h`、`src/video/drm_nv12_transfer.cpp`

## 功能职责

把FFmpeg/rkmpp输出的单对象、线性、连续半规划YUV DMA-BUF（NV12 或 NV16）通过RGA复制/转换到现有CPU可访问**NV12**目标缓冲，供`VideoFramePool`、YOLO、叠加和H.264编码链路继续使用。该模块用于绕过实测会从约2ms恶化到10～15ms的`av_hwframe_transfer_data`；NV16 分支服务于 mjpeg_rkmpp 硬解（输出为 NV16/YCbCr422SP）。

不负责：压缩码流解码、DMA-BUF跨线程生命周期、完整端到端零拷贝、AFBC/非线性modifier、多对象或NV12/NV16以外布局、双目同步与深度计算。

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

/// 纯函数，无 RGA 依赖，开发机可直接测
enum class DrmSpFormat { kUnsupported, kNv12, kNv16 };
DrmSpFormat ClassifyDrmSpFormat(std::uint32_t drm_format_fourcc);
bool ValidateLinearDrmNv12Layout(const DrmNv12Layout&, std::string* error);
bool ValidateLinearDrmNv16Layout(const DrmNv12Layout&, std::string* error);
```

数据流：

```text
AVDRMFrameDescriptor
→ ClassifyDrmSpFormat(图层 fourcc)
   ├─ NV12 → ValidateLinearDrmNv12Layout → RGA imcopy（同格式纯拷贝）
   ├─ NV16 → ValidateLinearDrmNv16Layout → RGA imcvtcolor（422SP→420SP 转换）
   └─ 其他 → kUnsupportedLayout（回退 av_hwframe_transfer_data）
→ wrapbuffer_virtualaddr(VideoFramePool NV12 槽位)
→ FrameHandle发布
```

## 关键实现点

- 当前适配依据Orange Pi实测布局：1280×720、一个DRM对象、一个图层、`DRM_FORMAT_NV12`、modifier=0、Y/UV同fd、pitch均1280、UV offset=921600。mjpeg_rkmpp 输出为 NV16，布局规则相同、仅 UV 面更高。
- **NV12 与 NV16 校验规则对照**（公共规则由 `ValidateLinearDrmSpLayoutCommon` 统一实现，防逻辑漂移）：

  | 规则 | NV12 | NV16 |
  |------|------|------|
  | fourcc | `NV12` | `NV16` |
  | UV 面行数 | height/2 | height（垂直满采样） |
  | 对象容量下限 | pitch×height×3/2 | pitch×height×2 |
  | 其余（modifier=0、单对象、y_offset=0、uv_offset==pitch×height、pitch≥width、偶数尺寸） | 相同 | 相同 |

- 分派在 `CopyToNv12` 内部按 `source.drm_format` 完成，接口保持单方法、目标恒 NV12；NV12 源用 `wrapbuffer_fd(..., RK_FORMAT_YCbCr_420_SP, ...)` + `imcopy`，NV16 源用 `RK_FORMAT_YCbCr_422_SP` + `imcvtcolor(422SP→420SP)` 直写 NV12 池槽位；`imcopy`/`imcvtcolor` 同步返回前保持源`AVFrame`有效。
- **禁用语义**：布局不支持（含格式不支持）→ 调用方置 `rga_dma_disabled=true` 永久禁用（MPP 会话内布局恒定，重试无意义）；运行时 `imcopy`/`imcvtcolor` 失败 → 只计一次回退，不永久禁用。
- `VideoDecoderConfig.prefer_rga_dma_transfer`代码缺省为false；600秒长测和画面验收通过后，当前生产`config/config.json`已显式设为true。RGA不可用或失败时仍自动回退，不影响启动。
- RGA不可用、布局不支持或复制/转换失败时自动回退`av_hwframe_transfer_data`，不因试验路径中断视频。
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

`tests/video/drm_nv12_transfer_test.cpp`覆盖实测NV12布局通过、非线性modifier、错误UV offset、对象容量不足，以及 NV16 泛化用例：fourcc 分类、NV16 合法布局、NV12 容量规则对 NV16 误判被拒、NV16 坏 uv_offset、NV16 非线性 modifier。开发机无librga，不执行真实DMA复制。

香橙派：

```bash
./build/video_latency_probe --duration 600 --interval 10
```

600秒长测通过：累计输入14995个访问单元，RGA DMA成功14945帧、回退0，输入与输出固定相差50帧且运行中不再增长；D4/D5全程无样本。D4R各10秒窗口平均约1.76～2.52ms，P95约2.89～4.87ms，无随时间恶化趋势；入口到本地RTSP平均约5.21～5.44ms，P95始终9ms。慢帧事件文件只有首次启用INFO，没有WARN。性能和稳定性已通过，正式启用前仅剩画面颜色/错位/检测框人工确认。

## 排查/修改要点

- RGA首次即失败：检查CMake是否显示`RGA DMA-BUF转存: ON`、librga版本、DRM fourcc/modifier和`wrapbuffer_fd`参数。
- 画面偏色：重点核对NV12/NV21格式、UV offset、pitch和目标stride。
- 图像上下错位：核对source hstride；当前实测UV offset恰好等于`pitch*height`。
- 回退计数增长：查看WARN中的状态和原因；不应为了强行使用DMA放宽布局校验。
- fd生命周期：RGA同步调用返回前必须持有源`AVFrame`；若未来异步传递DMA-BUF，必须用RAII持有`av_frame_ref`。
- 后续全链路零拷贝需要新的DMA帧句柄，并分别适配RGA预处理、叠加和MPP编码，不在本模块范围内。
