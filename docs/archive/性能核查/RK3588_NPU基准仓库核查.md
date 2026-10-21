# RK3588 NPU基准仓库核查

> 核查日期：2026-09-09  
> 第三方仓库：https://github.com/hannahrepo/rk3588-npu  
> 核查提交：`8031e192ef47d8589365243049e268137148312d`  
> 官方仓库：https://github.com/airockchip/rknn-toolkit2  
> 核查提交：`59a913d172e7f5ff03c9076e2ec7b1b1288ffd08`  
> 官方文档：`04_Rockchip_RKNPU_API_Reference_RKNNRT_V2.3.2_CN.pdf`、`rknpu2/examples/rknn_benchmark`、`rknpu2/examples/rknn_zero_copy`

## 1. 官方API结论

### 1.1 组合core mask不是无条件三倍加速

官方RKNNRT 2.3.2文档说明，`RKNN_NPU_CORE_0_1`和`RKNN_NPU_CORE_0_1_2`模式下，当前能较好加速的OP主要是：

- Conv；
- DepthwiseConvolution；
- Add、Concat；
- Relu、Clip、Relu6、ThresholdedRelu、PRelu、LeakyRelu。

其余类型OP会fallback到单核Core0；官方特别举例Pool类、ConvTranspose等尚未支持组合核心。YOLO网络虽然卷积占主体，但仍包含池化、上采样/拼接及检测头相关算子，因此组合core mask不能理解为把完整网络平均切成三份。

本项目实测`all/core012/core0`完整处理平均分别27.18/26.51/27.68ms，且Core0约51%、Core1/2通常低于10%，与官方限制一致。

### 1.2 多线程应使用独立context

官方文档明确说明：`rknn_dup_context`用于生成指向同一模型的新context，可在多线程执行相同模型时复用权重。合理的帧级并行结构应是每个工作线程持有独立context和独立I/O内存，而不是多个线程同时调用同一个context。

### 1.3 `RKNN_FLAG_PRIOR_HIGH`不是额外加速开关

官方头文件中：

```cpp
#define RKNN_FLAG_PRIOR_HIGH 0x00000000
```

高优先级就是缺省值。第三方代码把它描述为“High Priority optimization”，但传入该标志与传入0等价。本项目使用`RKNN_FLAG_ENABLE_SRAM`，优先级位仍为缺省高优先级，不需要再叠加`RKNN_FLAG_PRIOR_HIGH`。

### 1.4 `pass_through=1`不等于零拷贝

官方定义：

- 零拷贝由`rknn_create_mem/rknn_set_io_mem`等接口建立；
- `pass_through=0`时，runtime按`type/fmt`把用户数据转换为模型输入；
- `pass_through=1`时，buffer直接送入模型输入节点，不再做布局、类型、归一化或量化转换。

因此只有当用户buffer已经严格匹配模型原生输入布局、类型、量化和stride时才能使用`pass_through=1`。把普通RGB `uint8`图像设置为pass-through可能得到错误结果，不能把跳过必要转换后的吞吐作为有效模型性能。

### 1.5 官方性能查询可直接在`rknn_run`后使用

官方文档说明：

```cpp
rknn_run(ctx, nullptr);
rknn_perf_run perf_run{};
rknn_query(ctx, RKNN_QUERY_PERF_RUN, &perf_run, sizeof(perf_run));
```

`run_duration`是不包含设置输入/输出的模型推理时间，单位微秒，不要求`RKNN_FLAG_COLLECT_PERF_MASK`。逐层`RKNN_QUERY_PERF_DETAIL`才需要该标志，且官方明确说明它会降低帧率。下一步可在探针中可选采集PERF_RUN，与`rknn_run`墙钟时间对比。

## 2. 官方`rknn_benchmark`行为

官方基准工具：

- 只有一个context；
- 输入只在循环前设置一次；
- warmup 5次；
- 循环中只测同步`rknn_run`墙钟时间；
- 输出在全部循环结束后获取；
- core mask通过0/1/2/4/3/7指定；
- README要求基准前锁定CPU、DDR、NPU最高频率。

该工具测的是重复静态输入下的纯模型运行时间，不是预处理、输出获取、NMS、视频解码和图传的端到端延迟。它适合回答“当前RKNN模型本身在隔离环境要多少毫秒”，不能直接代表正式链路FPS。

## 3. `hannahrepo/rk3588-npu`核查

### 3.1 README的181+ FPS没有可复现证据

README声称C++程序通过零拷贝、多核亲和和高优先级达到181+ FPS，但仓库没有提供：

- 对应模型文件和SHA-256；
- 原始运行日志；
- 每个context/核心的独立统计；
- 输出正确性校验；
- RKNN runtime/驱动版本；
- 温度、频率和功耗记录；
- P50/P95/P99。

仓库示例配置是`yolov5s_relu.rknn`，不是本项目YOLO26n单类模型。即使181 FPS真实，它也是三个核心上的聚合吞吐，不能解释为单帧5.5ms。

### 3.2 8线程共享3个context

`main.cc`创建3个context，却创建8个线程并按`i % 3`复用context，因此每个context会被2～3个线程并发调用`rknn_run`。代码没有context锁，也没有为每个线程创建/复制独立context。该用法不符合官方推荐的“多线程使用独立context”结构，不能作为可靠生产实现。

### 3.3 存在C++数据竞争

全局停止标志：

```cpp
bool stop_test = false;
```

由主线程写、工作线程无同步读取，构成C++数据竞争，行为未定义；应使用`std::atomic<bool>`或受控同步。

### 3.4 “Average Core Latency”公式无有效物理含义

代码使用：

```cpp
(1000 * NUM_THREADS) / (total_fps * NUM_CORES)
```

线程数不应这样除以核心数。三个核心完全饱和且负载均匀时，单核平均服务时间近似：

```text
3000 / total_fps ms
```

系统完成间隔是：

```text
1000 / total_fps ms
```

而8个闭环请求的平均响应时间按Little定律近似：

```text
8000 / total_fps ms
```

第三方公式不对应上述任一指标，会把聚合吞吐错误包装为“单核延迟”。

### 3.5 零拷贝实现不完整且可能破坏输入语义

代码只绑定输入内存，没有绑定输出内存，也不读取或验证输出。输入内存使用逻辑属性的`size`而不是官方示例使用的`size_with_stride`，并把普通OpenCV RGB `uint8`数据设置为`pass_through=1`。若模型原生输入不是完全相同的uint8/NHWC/stride/量化布局，推理虽然可以运行，结果可能无效。

它只在计时前复制一次静态图片，循环中反复推理同一输入，因此所谓“bypass CPU copy”不能代表真实视频每帧必须更新输入buffer的成本。

### 3.6 高优先级描述有误

`RKNN_FLAG_PRIOR_HIGH`数值为0，与默认初始化等价，不是该程序相对本项目的独有优化。

### 3.7 错误处理不足

代码没有可靠检查：

- `rknn_set_core_mask`返回值；
- 多个`rknn_query`返回值；
- `rknn_create_mem`空指针；
- `rknn_set_io_mem`返回值；
- 图像是否成功读取及尺寸/stride是否匹配；
- 推理输出是否正确。

仓库当前没有LICENSE文件，GitHub API也返回license为空，不能直接复制其实现进入本项目。

## 4. 锁频脚本核查

官方基准README确实建议锁定CPU、DDR和NPU最高频率，以减少benchmark波动。但第三方`fix_freq_rk3588.sh`不适合直接用于生产：

- 硬编码CPU、GPU、NPU、DDR路径和频率；
- 不检查频率是否在当前内核支持列表；
- 不保存和恢复原governor；
- 不设置`trap`，异常退出后系统继续锁频；
- 无条件禁用8个CPU的深度idle；
- GPU与当前RKNN链路无关，锁GPU最高频只增加功耗和温度；
- 同时修改多个变量，无法判断收益来自CPU、DDR还是NPU。

本项目已经证明NPU `performance`固定1GHz相对`rknpu_ondemand`无有效收益。板端当前没有暴露第三方脚本假定的`/sys/class/devfreq/dmc/{governor,cur_freq,...}`文件，不能直接按该路径锁DDR；需先枚举实际devfreq节点，若内核没有DMC devfreq接口则无法从该sysfs路径做DDR A/B。CPU实际最高频也与第三方硬编码不一致：policy0=1.8GHz、policy4=2.256GHz、policy6=2.304GHz，而脚本给policy4/6写2.352GHz。因此若继续基准，应先枚举真实节点和支持频率，再逐变量测试CPU或DDR，不能执行整份脚本。

## 5. 对本项目可复用的内容

可采用的思想：

1. 使用独立context分别绑定Core0/Core1/Core2做帧级并行吞吐A/B；
2. benchmark时控制CPU/DDR/NPU频率，但逐变量测试并自动恢复；
3. 使用`rknn_create_mem/rknn_set_io_mem`减少输入输出复制；
4. 使用官方`RKNN_QUERY_PERF_RUN`区分NPU内部执行时间和`rknn_run`墙钟时间；
5. 使用官方`rknn_benchmark`做隔离模型基线。

不可直接采用：

1. 8线程共享3个context；
2. `pass_through=1`但不验证原生输入布局和量化；
3. 第三方“Average Core Latency”公式；
4. 不校验输出的纯吞吐结论；
5. 无恢复机制的全系统锁频脚本；
6. README的181+ FPS作为YOLO26n性能依据。

## 6. 后续验证顺序

1. `--rknn-perf-run`实测Y2W墙钟与Y2N RKNN内部时间在平均/P50/P95/P99上均只差约0.01ms，已证明23～26ms几乎全部是模型内部NPU执行，不是CPU调度、runtime阻塞或I/O设置等待；当前Core0约50%首先反映的是约24ms服务时间乘以约25 FPS输入形成的约60%占空比，而不是NPU最大吞吐只有25 FPS；
2. 探针已增加`--rknn-perf-detail`，使用官方`RKNN_FLAG_COLLECT_PERF_MASK`并在第100次推理后打印一次逐层报告；用它定位最慢OP和fallback核心，报告模式不能与正常吞吐直接比较；
3. 使用官方风格单context静态输入基准，分别测试core mask 1/2/4/7，warmup后至少1000次；
4. 当前内核未暴露DMC devfreq节点，跳过基于该sysfs路径的DDR A/B；
5. 获取本项目ONNX、RKNN转换脚本、校准集和转换日志，核查模型图；
6. 若目标是降低正式链路队列帧龄，新增双context（Core0/Core1）探针，每个context独立线程和I/O内存，并按frame sequence丢弃乱序旧结果；
7. 不在飞行生产配置中锁死CPU/GPU/DDR/NPU最高频率，除非完成温度、功耗和长时稳定性验收。
