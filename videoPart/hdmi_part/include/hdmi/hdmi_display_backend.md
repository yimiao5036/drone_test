# HDMI 直显后端（HdmiDisplayBackend）实现文档

> 对应实现：`include/video_transmission/hdmi/hdmi_display_backend.h`
>（实现 `src/video_transmission/hdmi/hdmi_display_backend.cpp`、
>`hdmi_display_stub.cpp`、`hdmi_drm_display.cpp`、`hdmi_probe.cpp`、`CMakeLists.txt`）

## 功能职责

把 `FrameCompositor` 产出的标注帧（NV12 `FrameHandle`）**不编码**直接送到香橙派
HDMI 口输出，用于驱动 HDMI-in 数字图传发射机或本地监视屏。

- 本目录是 `video_encoder.h` 定义的**可替换边界 `IVideoEncoderBackend` 的第二个实现**。
  默认的 FFmpeg 编码后端（RTSP 推流）保持不变，两者通过
  `VideoSenderConfig::backend_factory` 在装配层二选一。
- **不编码**：标注帧本身已是 NV12，GPU 只做 YUV→RGB 与缩放到 GBM 扫描缓冲，
  省掉 H.264 编解码与 RTSP 链路。
- **不做**：图像叠加/裁剪（`FrameCompositor` 职责）、目标识别、RTSP 推流、
  音频、多屏/多 connector 同时输出。

**为什么不改动既有文件**：`video_encoder.h/.cpp` 与 `VideoSender` 主流程零改动。
`VideoSender` 早已预留 `backend_factory` 注入点，本实现只需在装配处替换注入即可。

## 接口与数据流

```
kAnnotatedFrame (Topic<FrameHandle>, NV12)
   ─► VideoSender（不改动）──► HdmiDisplayBackend ──► IHdmiDisplay::Submit
        ──► GLES3：NV12 双纹理 → YUV→RGB shader → GBM 缓冲
        ──► drmModeAddFB2 → drmModePageFlip ──► VOP2 ──► HDMI 输出
```

### 对外类型

| 名称 | 说明 |
|---|---|
| `HdmiColorSpace` | `kBt601` / `kBt709`，YUV→RGB 矩阵选择 |
| `HdmiSubmitResult` | `kSubmitted` 已提交 / `kBusy` 上次翻转未完成丢弃 / `kFailed` 设备错误 |
| `HdmiDisplayConfig` | 输出配置，见下表 |
| `IHdmiDisplay` | 显示设备抽象，隔离 libdrm/libgbm/EGL |
| `HdmiDisplayBackend` | 实现 `IVideoEncoderBackend`，供注入 |
| `HdmiMakeBackendFactory(cfg)` | 返回可直接赋给 `backend_factory` 的 lambda |
| `HdmiConfigFromEncoderConfig(encode)` | 从既有 `EncoderBackendConfig` 桥接出 HDMI 配置 |
| `CreateHdmiDisplay()` | 编译期分派：有 DRM 依赖用真实实现，否则用占位实现 |
| `CreateHdmiDrmDisplay()` / `CreateHdmiDisplayStub()` | 显式创建（测试/自检用） |
| `DescribeHdmiDrmResources(...)` | 只读枚举 DRM 资源报告，供 `hdmi_probe modes` |

### 语义映射（接口名沿用编码后端，避免改动冻结接口）

| `IVideoEncoderBackend` 方法 | HDMI 后端语义 | 实测落点 |
|---|---|---|
| `EncodeFrame` | 提交一帧 NV12 上屏 | 翻转繁忙/失败返回 `false`，`VideoSender` 计入 `DroppedFrameCount` |
| `SentFrameCount` | 累计成功上屏帧数 | — |
| `ErrorCount` | 输入非法、尺寸不匹配、设备错误累计 | 翻转繁忙**不计**错误（预期降级） |
| `FramePrepareLatency` | NV12 纹理上传 + 绘制 + swap | 探针第 11 项（H264输入帧准备）位置 |
| `PacketWriteLatency` | 翻转事件收割 + `drmModePageFlip` 提交 | 探针第 13 项（RTSP单包写入）位置 |

> `video_latency_probe` 无需改动即可读取上述指标；编号 11/13 的名称在 HDMI 模式下
> 语义变为"纹理准备/翻转提交"，**名称未改是刻意的**（探针属既有文件，本次不动）。

### 配置项

| 字段 | 默认 | 说明 |
|---|---|---|
| `width` / `height` | 1280 / 720 | 输出模式分辨率 |
| `refresh_hz` | 30 | 输出刷新率；源 25fps，30Hz 足够保持时序 |
| `strict_mode` | false | true 时只接受精确匹配或合成模式；false 允许回退到同分辨率的其他刷新率 |
| `drm_device` | `/dev/dri/card0` | DRM 设备节点 |
| `connector_name` | 空 | 空 = 自动选第一个已连接连接器；可填 `HDMI-A-1` |
| `color_space` | `kBt709` | 源为 720p H.264，普遍 BT.709 |
| `full_range` | false | false = 有限范围 16-235（与 H.264 源一致） |
| `flip_timeout_ms` | 100 | 收尾等待翻转的上限；运行期超时判定设备卡死 |
| `blank_on_stop` | true | 停止时点黑场保持 CRTC；false 则恢复进入前的 CRTC 状态 |
| `log_every_n_frames` | 0 | >0 时每 N 帧打一次 INFO，仅上板调试用 |

## 关键实现点

### 1. 时序优先：Open 即建立 HDMI 信号

`Open()` 的最后一步是 `ModesetBlack()`——渲染黑场并 `drmModeSetCrtc`。
**Start 返回时 HDMI 信号已锁定**，HDMI-in 图传发射机不会因"等第一帧"而反复重锁。
`Close()` 默认（`blank_on_stop=true`）再点一帧黑场并保持 CRTC 使能，
让发射机停在稳定黑场而不是冻结画面。

这也意味着：更改分辨率/刷新率后**必须重启进程**，运行中不做动态 modeset。

### 2. 三级输出模式选择（固定 720p30 的落地方式）

`PickOutputMode()` 按以下顺序选择，并在日志中标注实际采用的级别：

1. **EDID 精确匹配**：同宽高且刷新率在 ±2Hz 内 → 直接用，INFO。
2. **EDID 同分辨率回退**：同宽高但刷新率不同（`strict_mode=false` 时）→ 采用，
   打 **WARN**。理由：图传"能锁定"比"恰好 30Hz"更重要。
3. **合成时序**：EDID 完全没有该分辨率时，按 CEA-861 消隐参数缩放像素时钟
   （720p60 的 74.25MHz ÷ 2 = 37.125MHz → 30Hz，htotal 1650、vtotal 750 不变），
   打 **WARN** 提示图传可能不锁定。

第 3 级目前只覆盖 1280x720。其他分辨率无 EDID 模式时 `Open` 直接失败，
并在日志中给出连接器支持的模式数量，提示用 `hdmi_probe modes` 查看完整列表。

### 3. NV12 上屏与 stride 处理

- 双纹理：Y 用 `GL_R8`，UV 交错平面用 `GL_RG8`（半分辨率）。
- **`GL_UNPACK_ROW_LENGTH`**：`FrameHandle` 的 `hor_stride` 可能大于有效宽
  （`video_frame.h` 明确 stride ≥ width，行有对齐填充），必须让驱动按 stride 跨行
  取样，否则图像每行错位累积成斜纹。GLES3 的 `GL_UNPACK_ROW_LENGTH` 以**源格式
  纹素**计：R8 时填 `hor_stride`；RG8 每纹素 2 字节，填 `hor_stride / 2`。
- UV 平面偏移 `hor_stride * height`，与 `video_encoder.cpp` 的拷贝布局完全一致。
- **硬性要求 GLES3**：GLES2 的 `GL_UNPACK_ROW_LENGTH` 在部分驱动上不生效会花屏，
  因此不提供 ES2 回退，而是明确失败并打印原因。RK3588 的 Mali-G610 原生支持 3.2。

### 4. 绝不阻塞的页面翻转

- DRM fd 设为 `O_NONBLOCK`；翻转事件用 `poll(fd, POLLIN, 0)` + `drmHandleEvent` 收割。
- `Submit()` 先收割事件，**仍挂起则直接返回 `kBusy` 丢帧**，绝不等待垂直同步。
  这与 `VideoSender`"拥塞只丢图传帧、不反压上游"的约定一致。
- 卡死检测：翻转挂起超过 `flip_timeout_ms` 判定设备异常，**复位翻转状态**并返回
  `kFailed`，避免一次异常把整条输出链永久卡死。
- 唯一会阻塞的是收尾路径 `WaitForPendingFlip()`，只在 `Close()` 调用。

### 5. framebuffer 一帧滞后回收

`drmModeAddFB2` 在每次提交时新建，`drmModeRmFB` **在下一个翻转确认后**才删除
"上一个"已上屏的 fb。任何时刻至多 2~3 个 fb 存活。
不能在翻转确认前删除当前 fb（内核仍在扫描），也不能在提交前删除旧 fb（会黑屏）。

`Close()` **刻意不回收当前 framebuffer**：让黑场/最后一帧保持到进程退出，
避免图传发射机在收尾瞬间失锁。fd 关闭时内核统一释放，无泄漏。

### 6. DRM master 与权限

`open(/dev/dri/cardN)` 在无其他 master 时即持有 master，`drmSetMaster()` 失败不视为
致命（真正的判据是 `drmModeSetCrtc` 是否被拒）。**需要 root 或 CAP_SYS_ADMIN**，
且不能有桌面环境/其他程序持有 master，否则 modeset 报 `EACCES`，
`DescribeDrmError()` 会补上可操作的中文提示。

### 7. 隔离设计：为什么拆 `IHdmiDisplay`

`libdrm/libgbm/EGL` 只出现在 `hdmi_drm_display.cpp` 一个文件里。
`hdmi_display_backend.cpp` 与 `hdmi_display_stub.cpp` 不引用任何图形库，
因此**可在无 `/dev/dri` 的开发机上编译并单测**，也让 DRM 实现可被 Mock 替换。
这与项目既有 `video_decoder.cpp` / `video_decoder_stub.cpp` 的约定一致。

## 日志行为

- **INFO**：后端创建/销毁/启停；DRM 设备、连接器、CRTC、模式、EGL 版本、
  GL 版本/渲染器；EDID 精确匹配的最终模式。
- **WARN**：DRM master 显式获取失败；模式回退到 EDID 其他刷新率；使用合成时序；
  收尾等待翻转超时；点黑场失败。
- **ERROR**：打开设备/GBM/EGL/GLES/着色器失败；无可用连接器/CRTC/模式；
  `drmModeSetCrtc` / `drmModeAddFB2` / `drmModePageFlip` / `eglSwapBuffers` 失败；
  翻转卡死超时；输入帧非法或尺寸不匹配（**节流：第 1 次 + 每满 100 次**）。
- **逐帧热路径不打日志**，除非 `log_every_n_frames > 0`（仅调试）。
- 每次 ERROR 都携带可操作提示（权限、EDID、带宽、驱动、GLES3 要求）。

## 构建

### 依赖

```bash
sudo apt install libdrm-dev libgbm-dev libegl1-mesa-dev libgles2-mesa-dev
```

香橙派若使用厂商 EGL/GLES 实现（libmali），需确认 `libEGL.pc` / `glesv2.pc`
在 pkg-config 搜索路径内；否则用 `-DDRONE_HAVE_HDMI_KMS=OFF` 先跑通其余部分。

### 独立编译验证（推荐先做这一步）

```bash
cd /mnt/d/ProgramData/drone_test/drone_test
cmake -S src/video_transmission/hdmi_part -B build_hdmi
cmake --build build_hdmi -j$(nproc)
```

ARM64 默认 `DRONE_HAVE_HDMI_KMS=ON`；x86_64 开发机默认 OFF，
此时编译 `hdmi_display_stub.cpp` 路径，**这一步只证明能编译，不证明能上屏**。

### 主工程接入（唯一需要改的三处，均为追加，不改既有逻辑）

1. **根 `CMakeLists.txt` 末尾**追加一行（必须在 `drone_video_transmission` 定义之后）：

   ```cmake
   add_subdirectory(src/video_transmission/hdmi)
   ```

   本子目录会新建独立静态库 `drone_hdmi_display` 并挂到 `drone_video_transmission` 上。
   刻意不把源文件塞进既有目标，因此既有源文件的编译选项完全不变。

2. **装配处**（`src/application/drone_application.cpp` 中创建 `VideoSender` 之前）
   按开关选择后端：

   ```cpp
   if (config_.video_sender_use_hdmi) {          // 需在 config 中新增该开关
       config_.video_sender.backend_factory =
           video_transmission::HdmiMakeBackendFactory(
               video_transmission::HdmiConfigFromEncoderConfig(
                   config_.video_sender.encode));
   }
   ```

   若希望 HDMI 参数可配，改为从 `AppConfig` 构造 `HdmiDisplayConfig` 传入。
   不改 `VideoSender`、不改 `video_encoder.h/.cpp`。

3. **配置文件**（`config/*.json` 的 `video` 段）新增开关，例如：

   ```json
   "video": {
       "output_backend": "hdmi_part",
       "hdmi_part": { "width": 1280, "height": 720, "refresh_hz": 30,
                 "connector_name": "", "color_space": "bt709", "full_range": false }
   }
   ```

   选 `"rtsp"`（或字段缺省）时保持原有行为完全不变。

## 测试方式

### 上板分三步验证（推荐按顺序，避免在整条链路里瞎猜）

```bash
# 步骤 1：只读枚举。不获取 master、不改变显示，可在正式程序运行前后安全执行。
#         重点看"-> 目标 1280x720@30Hz 将采用"这一行，确认 EDID 是否真给 720p30。
sudo ./build_hdmi/hdmi_probe modes

# 步骤 2：彩条上屏。验证 DRM/GBM/EGL/GL 全链路与页面翻转。
#         画面应出现 8 条彩条，且有一根白竖线在横向移动（说明在持续推进而非冻结）。
sudo ./build_hdmi/hdmi_probe test --frames 180

# 步骤 3：真实帧上屏。先抓一帧裸 NV12，再用同一 stride 显示。
sudo ./build_hdmi/hdmi_probe nv12 frame.nv12 --width 1280 --height 720 --stride 1280
```

### 单元与集成测试

- 注入 Mock 可完全绕开 DRM：`HdmiDisplayBackend(config, std::move(mock))`，
  参照 `tests/video_transmission/video_sender_test.cpp` 的 `MockBackend` 写法。
  可验证：输入帧非法/尺寸不匹配丢帧、`kBusy` 不计错误、`kFailed` 计错误、
  计数与延迟接口。
- `CreateHdmiDisplayStub()` 可在开发机跑通 `VideoSender` 全链路
  （`backend_factory` 返回 `HdmiDisplayBackend` 且注入 stub），
  验证线程、订阅队列、`DroppedFrameCount` 统计。

> **本次交付未附带新测试文件**（约束为不改动既有目录）。若需要，可在
> `tests/video_transmission/` 下新增 `hdmi_display_backend_test.cpp`，
> 参考上表断言点；它只依赖 `drone_hdmi_display`，不需要 DRM 设备。

### 无法在开发机验证的部分

WSL2 Ubuntu 24.04 无 `/dev/dri`，**HDMI 上屏只能在香橙派实机验证**。
开发机能保证的只有：编译通过、参数校验与丢帧逻辑正确、无设备时 `Start()` 返回
`false` 且日志给出原因。

## 排查 / 修改要点

| 现象 | 定位方法 |
|---|---|
| `Start()` 返回 false，日志"获取 DRM master 失败"/`EACCES` | 未用 root，或桌面/其他程序持有 master。用 `sudo`，或确认无桌面环境 |
| 日志"找不到可用连接器" | 图传发射机未上电/未输出 EDID。`hdmi_probe modes` 看 `[未连接]`；换线或确认发射机供电 |
| 日志"无可用输出模式" | EDID 无该分辨率且非 720p。`hdmi_probe modes` 看完整模式列表，改配置到 EDID 支持的档位 |
| 日志"回退到 EDID 模式 ...（非 30Hz）" | 发射机未报 720p30。若必须 30Hz，把 `strict_mode=true` 逼合成时序，并实测能否锁定 |
| 图传发射机不锁定 HDMI 信号 | 优先检查 EDID 是否含该模式（`hdmi_probe modes`）；合成时序不被接受时改用 EDID 支持的档位 |
| 画面斜纹/错位 | `hor_stride` 与 `width` 不一致但驱动未按 stride 取样。确认 `Submit` 传入的 stride 是真实行 stride；确认 GLES3 上下文（日志有 GL 版本） |
| 画面颜色发灰/偏色/过曝 | 色彩矩阵或范围不匹配。试 `--bt601`、`--full-range` 组合定位，再改配置默认值 |
| 画面上下颠倒 | `kVertexShader` 的 `v_uv` Y 翻转表达式 |
| 画面卡住不动但无报错 | 看 `hdmi_probe test` 的白竖线是否移动；不动则看日志有无"翻转超时" |
| 日志"翻转超时 ... 复位翻转状态" | 翻转事件丢失或驱动异常。确认没有第二个程序抢 master；提高 `flip_timeout_ms` 观察 |
| `eglChooseConfig` 失败/无 GLES3 配置 | GPU 驱动未提供 GLES3 或 EGL 未走 GBM 平台。检查日志 EGL/GL 版本，确认 libmali/Mesa 与 DRM 驱动匹配 |
| `eglInitialize` 失败 | libEGL 与板端驱动不匹配（常见于厂商 blob 与 Mesa 混装）。用 `eglinfo`/`es2_info` 交叉确认 |
| `drmModeAddFB2` 失败 | GBM 缓冲格式与驱动不匹配。确认 `gbm_bo_get_format` 是 `XRGB8888` |
| 运行中想换分辨率 | 不支持动态 modeset。改配置后重启进程 |

### 修改时的关联点

- 改 `HdmiDisplayConfig` 字段 → 同步本文件配置表、`hdmi_probe.cpp` 的选项解析与帮助文本。
- 改模式选择策略 → 同步 `PickOutputMode()`、`DescribeHdmiDrmResources()`
  （它复用同一函数，保证自检结论与运行期一致）与本文件"三级模式选择"。
- 改色彩矩阵/范围 → 同步 `kColorMatrixBt601/709`、`SetupGl()` 里的 offset/scale、
  `hdmi_probe.cpp` 的 `RgbToNv12Yuv()`（两者必须互补，否则彩条颜色会偏）。
- 改日志 → 遵守 AGENTS.md 日志纪律：热路径不打日志，异常节流。

## 已知限制与后续优化

1. **NV12 图层直通未实现**：RK3588 VOP2 原生支持 NV12 平面，理论上可零转换上屏。
   但当前 `FrameHandle` 是 CPU 内存池而非 DMA-BUF，需先转 GEM bo 并处理
   modifier/对齐，排查成本高。当前走 GPU 转换路径。若后续 `VideoFramePool`
   支持 DMA-BUF 导出，可新增一个 `IHdmiDisplay` 实现做直通。
2. **仅支持 1280x720 合成时序**：其他分辨率在 EDID 无匹配模式时直接失败。
   需要时扩展 `SynthesizeMode()` 的时序表。
3. **不做缩放**：输入尺寸必须与输出模式一致，否则丢帧并计错误。
   若源分辨率与输出模式不同，应在 `FrameCompositor` 侧统一，不在本模块隐性拉伸。
4. **未提供 `interrupt_callback` 类的中断机制**：所有 DRM 调用均已非阻塞，
   无阻塞写包问题（与 FFmpeg 后端不同）。
5. **未附带单元测试文件**：受"不改动既有目录"约束，见上文"单元与集成测试"。
