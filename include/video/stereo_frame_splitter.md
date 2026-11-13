# 双目 SBS 帧拆分（stereo_frame_splitter）

> 对应实现：`include/video/stereo_frame_splitter.h`、`src/video/stereo_frame_splitter.cpp`
> 上游：解码 `video_decoder.md`（全幅 SBS NV12）；下游：YOLO/叠加消费左目
> 设计：`docs/superpowers/specs/2026-09-21-usb双目接入-design.md`（B 层范围）

## 1. 功能职责

订阅解码器全幅 SBS 帧（如 2560×720 NV12，左半幅一目、右半幅另一目），裁剪为左右
两个单目 NV12 帧（width/2 × height），分别发布到左目/右目输出主题：

- 左目输出接现有 YOLO/叠加链路出图（与 RTSP 链路解码帧同为 1280×720 NV12，下游无差异）；
- 右目输出本轮仅统计（无订阅者，句柄入 Topic 后自动归还内存池）；
- 左右各一个自有 `VideoFramePool`（容量 8、stride 64 对齐），独立容错：一目池满只
  计该目丢帧，另一目照常发布。

不做什么：不做双推理、不做视差测距（受 P2 标定制约）、不注册健康源（`error_bits`
新增位涉及地面站协议语义，留待双目定型后评审）、右目不回传/落盘。

## 2. 接口与数据流

```cpp
struct StereoFrameSplitterConfig {
    std::uint32_t width = 2560;          // 全幅 SBS 宽（必须为正的偶数）
    std::uint32_t height = 720;
    std::size_t pool_capacity = 8;       // 左右各自输出池容量
    std::size_t input_queue_capacity = 1;  // 1=只处理最新帧
    std::uint32_t stride_alignment = 64;
    bool prefer_rga = true;              // RGA 裁剪优先；失败/无 RGA 回退 memcpy
};

class StereoFrameSplitter final {
    // Start/Stop/IsRunning；SetInput(Topic<FrameHandle>&)；
    // LeftOutput()/RightOutput() → Topic<FrameHandle>&；
    // LeftFrameCount/RightFrameCount/LeftDroppedCount/RightDroppedCount/
    // ErrorCount/RgaFallbackCount
};
```

数据流：`Topic<FrameHandle>(全幅2560×720) ─► 裁剪 ─► Topic<FrameHandle>(左目1280×720) ─► YOLO/叠加`
`                                              └──► Topic<FrameHandle>(右目，仅统计)`

## 3. 关键实现点

- **输入校验**：句柄无效/格式非 NV12/宽高与配置不符 → 丢弃并 `ErrorCount++`
  （ERROR 节流 1+100）；构造校验全幅宽为正的偶数（否则抛 `std::invalid_argument`）。
- **RGA 裁剪**（`DRONE_HAVE_RGA_DMA` 条件编译）：`wrapbuffer_virtualaddr` 包源全幅与
  目标半幅（`RK_FORMAT_YCbCr_420_SP`，注意六参数顺序 width/height/format/wstride/
  hstride），`imcrop(src, dst, {x,0,w/2,h})` 一次调用完成一目；失败计
  `RgaFallbackCount`（WARN 节流）并回退 memcpy。
- **memcpy 回退**（x86 正常路径）：Y 面 h 行 + UV 半面 h/2 行逐行拷贝，行宽 = 单目宽
  字节；源/目的 stride 可能不同（全幅 stride vs 单目 stride），必须逐行；UV 面起始
  偏移 = stride × ver_stride。
- **帧元数据语义**（与解码器一致）：`pipeline_ingress_time_ms` 透传源帧（端内总延迟
  基准）；`timestamp_ms` 为拆分完成时刻（`SetTiming` 写入）；**序号由左右各自内存池
  分配**（FrameHandle 不支持序号透传），左右帧率一致性以 `LeftFrameCount`/
  `RightFrameCount` 与 ingress 时间戳核对。
- **线程模型**：独立消费线程，输入订阅容量 1 + `kDropOldest`（只处理最新帧）；
  `WaitTakeFor(100ms)` + Stop 经 `input_sub.Reset()` 唤醒（对齐 compositor 停机模式）；
  Start/Stop 幂等，Stop 后重启按已存主题指针重新订阅。
- **左右目对应关系（2026-09-22 已实证并交换）**：SBS **左半幅 = 物理右目**（摄像头朝前
  方位定义左右；地面站实测主用目画面为右目视角）。`ProcessOne` 两个 x_offset 已交换：
  **左目 Topic 取右半幅（x_offset=width/2）= 物理左目，右目 Topic 取左半幅（x_offset=0）
  = 物理右目**，Topic 语义与物理方位一致。实证方法：遮挡单侧镜头看地面站画面变化。

## 4. 日志行为

| 等级 | 场景 |
|------|------|
| INFO | 创建（全幅/池容量/RGA 开关）、启动、停止、销毁、输出池创建（容量/单目尺寸/stride） |
| WARN（节流） | 左右池满丢帧、RGA 裁剪失败回退 memcpy |
| ERROR（节流） | 输入帧非法（句柄/格式/尺寸不符） |

逐帧热路径零日志。

## 5. 测试方式

`tests/video/stereo_frame_splitter_test.cpp`（x86 memcpy 路径全覆盖，不依赖硬件）：

```bash
cmake --build build && cd build && ctest -R StereoFrameSplitter
```

- `SplitsLeftAndRightEyesByMemcpy` / `SplitsWithPreferRgaEnabled`：64×64 假帧左右半幅
  不同图案 → 断言左右输出各 32×64 NV12、与对应**物理目**半幅逐字节一致（已实证左半幅
  =物理右目：左目输出=右半幅内容）、ingress 透传、timestamp>0、首帧序号各为 0
  （独立池）、计数正确；
- `CountsInvalidInputWithoutPublishing`：空句柄 → ErrorCount 增、不发布；
- `DropsWhenOutputPoolExhausted`：池容量 2 且输出不消费 → 左右各发布 2 丢 4；
- `RejectsInvalidConfig`：奇数宽/0 高/0 池/0 输入队列 → 构造抛出。

RGA 路径正确性与左右帧率长期一致性由香橙派板上验收覆盖。

## 6. 排查/修改要点

| 现象 | 排查方向 |
|------|----------|
| `输入帧非法` 计数增长 | 上游解码输出尺寸 ≠ 配置全幅：确认 `video.uvc_width/height` 与实际档位一致（配置解析已透传给拆分器）；或输入源误接 RTSP 解码帧（stereo_split 仅在 camera_source=uvc 时允许） |
| 左/右 Dropped 增长 | 对应目输出池容量 < 订阅队列 + 在途；右目无消费者时计数为预期行为外的信号需复核装配 |
| `RgaFallbackCount` 增长 | librga 调用失败（日志含 `imStrError` 原因）：查 stride 对齐与 DRM/RGA 驱动；已自动回退 memcpy 不影响画面 |
| 左右画面相反 | 物理左右目未经实证属已知悬置；交换 `ProcessOne` 两个 x_offset（测距阶段实证后定论） |
| 修改全幅尺寸 | 只改 `video.uvc_width/height`（配置解析透传拆分器）；注意改为奇数宽会在配置阶段被拒绝 |
