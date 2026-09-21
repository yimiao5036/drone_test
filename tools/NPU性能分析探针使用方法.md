# NPU 性能分析探针使用方法

> 适用对象：在香橙派（RK3588）上分析 YOLO/RKNN 推理耗时、定位"单次推理慢"原因的维护者。
> 本文只描述 NPU 相关的探针用法与判读；视频链路其他阶段（解码、叠加、编码）的完整指标定义见
> [视频链路延迟探针文档](video_latency_probe.md)。
> 历史实测结论见 [YOLO26 RKNN逐层性能分析](../docs/YOLO26_RKNN逐层性能分析.md) 与
> [YOLO26在RK3588上的性能来源核查](../docs/YOLO26在RK3588上的性能来源核查.md)。

## 1. 功能职责

`video_latency_probe` 内置四级 NPU 计时与一次逐层性能报告，用于回答两个问题：

1. 单次推理的时间到底花在哪里（RGA 预处理、NPU 执行、输出搬运、后处理）；
2. `rknn_run` 墙钟时间里有多少是 NPU 模型内部执行，有多少是提交/调度开销。

**边界**：探针测量的是时间，不是 OS 级利用率。代码中没有读取
`/sys/kernel/debug/rknpu/load` 等驱动利用率接口的功能；板端工具显示的每核百分比
需要按本文第 5 节的方法与探针计时对照判读，不能直接等同推理速度。

## 2. 使用前提

- 香橙派 ARM64 原生构建（`DRONE_HAVE_RKNN=ON`，默认开），需已安装 RKNN runtime
  与 librga；
- **先停止正式 `drone_control`**，探针独占摄像头、NPU 和 RTSP 推流地址，两者不可同时运行；
- 建议运行时长 ≥120 秒：前 10～20 秒为解码器/NPU/编码器预热，不应用于结论；
- NPU 细分（Y 系列）使用最近最多 256 个样本的滑动窗口，约等于 25 FPS 下 10 秒，
  不能把结束窗口当作全程平均。

## 3. 使用方法

### 3.1 基线：YOLO 阶段细分（默认输出，无需额外参数）

```bash
cmake -S . -B build && cmake --build build -j$(nproc)
./build/video_latency_probe --duration 120 --interval 10
```

每次报告包含 YOLO 后端细分：

| 编号 | 含义 |
|---|---|
| Y1 | 预处理总耗时（含 Y1R/Y1C 及缓冲准备） |
| Y1R | RGA 缩放+颜色转换 |
| Y1C | CPU letterbox 复制 |
| Y2W | `rknn_run` 墙钟 |
| Y3 | 输出布局转换（NC1HWC2→NCHW） |
| Y4 | 阈值过滤+NMS |

### 3.2 NPU 内部执行时间：`--rknn-perf-run`

```bash
./build/video_latency_probe --duration 120 --interval 10 \
    --yolo-queue 1 --npu-core all --rknn-perf-run
```

启用后新增三项（对应配置字段 `yolo.collect_npu_internal_perf`，正式程序保持 `false`）：

| 编号 | 含义 |
|---|---|
| Y2N | `RKNN_QUERY_PERF_RUN` 报告的模型内部执行时间 |
| Y2O | Y2W − Y2N，即提交/调度/runtime 开销 |
| Y2Q | 查询调用本身的开销 |

板端 runtime 不支持当前零拷贝模式下查询时，只打印一次 WARN 并停止后续查询，
不影响推理。

### 3.3 逐层 OP 报告：`--rknn-perf-detail`

```bash
./build/video_latency_probe --duration 20 --interval 10 \
    --yolo-queue 1 --npu-core all --rknn-perf-detail
```

以 `RKNN_FLAG_COLLECT_PERF_MASK` 初始化，第 100 次成功推理后在日志中打印一次
`RKNN_QUERY_PERF_DETAIL` 原始报告（每个 OP 的耗时、次数、核心分配 WorkLoad）。
**官方明确说明逐层采集会降低帧率**：报告中的绝对时间不能当生产基线，
只看各 OP 占比和落在哪个核心。对应配置字段 `yolo.collect_npu_perf_detail`，
正式程序保持 `false`。

### 3.4 NPU 核心掩码 A/B：`--npu-core`

```bash
./build/video_latency_probe --duration 120 --interval 10 --yolo-queue 1 --npu-core all
./build/video_latency_probe --duration 120 --interval 10 --yolo-queue 1 --npu-core core012
./build/video_latency_probe --duration 120 --interval 10 --yolo-queue 1 --npu-core core0
```

支持 `auto/core0/core01/core012/all`（对应配置字段 `yolo.npu_core_mode`，生产为
`all`）。每次运行仍是单上下文同步 `rknn_run`；`core012` 表示该次模型运行使用组合
核心，不是三个请求并行。

### 3.5 候选模型对比：`--yolo-model`

```bash
./build/video_latency_probe --duration 120 --interval 10 \
    --yolo-model ./models/candidates/model.rknn --yolo-queue 1 --npu-core all
```

相对路径按探针启动时的当前目录解析；模型不存在时启动前直接拒绝，不修改生产 JSON。

## 4. 判读指南

### 4.1 "每核利用率只有 30~50%，但单次推理很慢"不矛盾

OS 工具显示的 NPU 利用率是**采样窗口内忙碌时间占比**，不是单次推理速度。
低利用率与长延迟同时成立，机制如下：

1. 当前模型约 83% 的耗时串行在 Core0：`ConvExSwish` 占 63.03%、
   两个 `exSDPAttention` 占 9.04%、`Split` 占 5.21%，逐层报告均为
   `WorkLoad=100%/0%/0%`；真正三核分摊的 OP 合计仅约 16.9%。
   单次推理内部大部分时刻只有一个核心工作，其余核心空闲。
2. 板端实测（2026-09，`all` 模式、25 FPS 连续负载）：Core0 约 51%，
   Core1/Core2 通常低于 10%。三核平均后每核 30~50% 与"单次约 24ms"
   完全自洽。
3. YOLO 输入队列容量为 1，推理慢于采集时多余帧被丢弃，帧间隙 NPU 还有空闲，
   利用率不会到 100%。
4. 按 Amdahl 定律，83% 串行占比下组合三核的理论加速上限约 1.13 倍；
   实测 `all/core012/core0` 三种掩码差异不足 1~4%。**提高利用率和降低单次
   延迟是两件事**：前者靠喂更多帧（吞吐），后者只能靠减少串行 OP（改模型
   结构），堆核心无效。

### 4.2 "单次推理时间不断变长"的定位顺序

用 `--rknn-perf-run` 跑 120 秒以上，比较首尾几次报告：

| 现象 | 结论方向 | 下一步 |
|---|---|---|
| Y2N（内部）随时间涨 | NPU 执行本身变慢 | 查温度与 DDR 频率/带宽争抢（每帧约 41MB DDR 读写）；NPU 频率已验证不是主因（`rknpu_ondemand` 可保持 1GHz，锁 `performance` 无收益） |
| Y2N 平稳、Y2O 变大 | 变慢在提交/调度侧 | 查 CPU 线程争抢（解码、RGA、编码线程抢核） |
| 05（YOLO 总耗时）涨、Y2W 不涨 | 瓶颈不在 NPU | 看 Y1 预处理（RGA）与 Y3/Y4 CPU 后处理；历史实测 Y3+Y4 合计不足 0.3ms，优先怀疑 Y1 |
| 04（YOLO 输入队列）涨、05 不变 | 生产快于消费，排队变长 | 队列积压问题，不是单次推理变慢 |

想看哪个 OP 吃时间、落在哪个核，再补一次 `--rknn-perf-detail`（注意第 3.3 节
的降帧率警告）。

### 4.3 已有基线结论（不要重复消耗时间验证）

- 正常模式 Y2W 与 Y2N 各百分位仅差约 0.01ms，查询开销接近 0：约 24ms 几乎
  全部属于模型内部 NPU 执行，不是 CPU 调度或 runtime 阻塞；
- `rknn_run` 平均约 23~25ms（640×640 INT8 模型、25 FPS 并行负载）；
- governor 固定 1GHz 无收益，生产保持 `rknpu_ondemand`；
- 生产保持 `npu_core_mode=all`。

## 5. 板端系统级观察（代码外手段）

以下内容代码不做采集，需要在香橙派上手动观察，与探针报告同时间段记录：

- NPU 频率与 governor：板端支持 300MHz~1GHz，
  `rknpu_ondemand/userspace/powersave/performance/simple_ondemand`；
- NPU 温度与 SoC 整体温度（判断是否热降频）；
- DDR 频率与带宽争抢（解码、RGA 转存、编码与 NPU 共享内存带宽）；
- 驱动每核负载读数（仅作忙碌占比参考，判读方法见 4.1）。

## 6. 注意事项

- 探针强制 `prefer_rga_dma_transfer=true`、`slow_frame_threshold_ms=10ms`，
  并跳过前 100 帧预热；
- `--rknn-perf-run` / `--rknn-perf-detail` 只用于探针；正式程序两个配置字段
  必须保持 `false`，不在热路径增加诊断调用；
- 启动阶段约 50 个 H.265 访问单元无法解码并伴随 `invalid pps id 0`，
  属于进程在 GOP 中途接入码流、等待随机访问帧的已知现象，不影响稳态统计；
- 退出阶段 `mpp_mem_pool_put invalid mem pool ptr` 告警为独立 MPP 释放问题，
  与 NPU 无关。

## 7. 相关文档

- [视频链路延迟探针文档](video_latency_probe.md)：探针全部指标定义与通用用法；
- [YOLO26 RKNN逐层性能分析](../docs/YOLO26_RKNN逐层性能分析.md)：OP 耗时排名、
  核心分配与 Amdahl 上限推导；
- [YOLO26在RK3588上的性能来源核查](../docs/YOLO26在RK3588上的性能来源核查.md)：
  core mask 与 governor A/B 实测数据；
- [视频链路延迟实测分析](../docs/视频链路延迟实测分析.md)：端到端延迟预算；
- [模型导出与转换说明](../yolo_pt/模型导出与转换说明.md)：候选模型来源。
