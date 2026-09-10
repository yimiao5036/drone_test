# YOLO26 RKNN逐层性能分析

> 实测日期：2026-09-09  
> 平台：Orange Pi 5 Plus / RK3588  
> RKNN Runtime：2.3.2（`429f97ae6b`）  
> 驱动：0.9.8  
> 模型：`yolo26n-drone-best.rknn`，640×640，INT8，单类别输出`[1,5,8400]`

## 1. 结论摘要

`RKNN_QUERY_PERF_RUN`证明正常模式下`rknn_run`墙钟与RKNN内部执行时间仅差约0.01ms，因此约24ms主要属于模型内部执行。逐层报告进一步确认：

- `ConvExSwish`共87次，占63.03%，全部显示`WorkLoad=100%/0%/0%`；
- 两个`exSDPAttention`占9.04%，同样全部运行在Core0；
- `Split`占5.21%，主要运行在Core0；
- `Reshape/Resize/MaxPool/Mul/Sub/Sigmoid`合计约5.7%，大部分运行在Core0；
- 真正分摊到三核的主要是普通`Conv`、`ConvAdd`、`Add`和`Concat`，总占比仅约16.9%。

按Amdahl定律，即使三核分摊部分达到理想3倍，约83%的Core0串行占比仍把理论总加速上限限制在约1.13倍；实测`core012`相对`all/core0`仅有几个百分点差异是合理结果，不可能达到3倍。

## 2. 模型与内存属性

```text
权重             2,846,720 B（约2.72 MiB）
中间张量         4,505,600 B（约4.30 MiB）
RKNN DMA         9,056,256 B（约8.64 MiB）
SRAM             0 / 0 B
每帧读写         41,937.41 KB（约40.95 MiB）
输入             NHWC INT8，640×640×3，stride=640，pass_through=0
输出             INT8 [1,5,8400]
```

`InputOperator`仅16us、`OutputOperator`仅19us，证明逻辑输入转换和输出边界不是主要瓶颈。`RKNN_FLAG_ENABLE_SRAM`下仍报告`SRAM=0/0B`，表示当前runtime/内核没有为该context报告可用RKNN系统SRAM；当前执行主要依赖DMA/DDR。每帧约41MB读写会产生一定访存压力，但逐层排名显示更主要的问题是大量小算子和Core0串行执行。

## 3. OP耗时排名

逐层采集模式会降低帧率，本次总OP时间27.804ms高于正常约24ms，因此绝对值不能作为生产基线，但比例和核心分配可用于定位。

| OP | 次数 | 时间 | 占比 | 核心特征 |
|---|---:|---:|---:|---|
| ConvExSwish | 87 | 17.524ms | 63.03% | 100/0/0，Core0 |
| exSDPAttention | 2 | 2.513ms | 9.04% | 100/0/0，Core0 |
| Concat | 23 | 2.432ms | 8.75% | 约三核均分 |
| Split | 12 | 1.448ms | 5.21% | 100/0/0，Core0 |
| Add | 18 | 1.294ms | 4.65% | 约三核均分 |
| Conv | 11 | 0.700ms | 2.52% | 约三核均分 |
| Reshape | 13 | 0.638ms | 2.29% | 多数Core0/无计算 |
| Resize | 2 | 0.303ms | 1.09% | Core0 |
| ConvAdd | 4 | 0.261ms | 0.94% | 约三核均分 |
| MaxPool | 3 | 0.241ms | 0.87% | Core0 |
| 其它 | - | 0.450ms | 1.61% | 主要Core0 |

## 4. 为什么组合三核无明显收益

官方RKNNRT 2.3.2只明确支持部分Conv/Add/Concat/激活OP在组合核心模式获得较好加速。当前编译图把卷积和SiLU/Swish融合为`ConvExSwish`，但逐层报告显示该融合OP没有拆分，87次调用全部落在Core0。两个注意力算子也全在Core0。

近似串行占比：

```text
ConvExSwish       63.03%
exSDPAttention     9.04%
Split              5.21%
Reshape/Resize/
Pool/Mul/Sub等     约5.7%
合计              约83%
```

理论上限：

```text
Speedup <= 1 / (0.83 + 0.17 / 3) ≈ 1.13
```

这只是忽略同步和访存开销的理想上限；实际几个百分点符合预期。

## 5. 模型结构中的主要优化候选

### 5.1 SiLU/Swish激活

`ConvExSwish`是最大瓶颈。第三方高吞吐示例特意使用`yolov5s_relu.rknn`，ReLU通常比SiLU/Swish更适合当前NPU组合核心。但不能在生产模型中直接把Swish替换为ReLU：必须重新训练或至少充分微调，并验证远距离小无人机召回率、误检率和置信度标定。

建议候选：

1. 保留当前SiLU模型作为精度基线；
2. 训练/微调同结构ReLU或ReLU6版本；
3. 使用相同校准集转INT8；
4. 对比逐层报告、Y2和真实小目标数据集精度。

也可做“拆开Conv与Swish、保持数学语义”的纯性能实验，观察普通Conv能否三核分摊；但拆分会新增Sigmoid/Mul和中间tensor写回，未必更快，必须实测。

### 5.2 注意力模块

两个`exSDPAttention`合计2.513ms，占9.04%，全在Core0。移除或替换轻量注意力理论上只能节省约2ms级，无法单独把24ms降到15ms，而且可能影响小目标特征表达，必须通过训练和精度验证决定。

### 5.3 模型内框解码

末端`Reshape/Split/Sub/Add/Mul/Concat/Sigmoid`在NPU内完成8400候选框解码，粗略合计约1.3ms。可测试导出原始多分支检测头，在CPU侧复用现有量化后处理。潜在收益约1ms级，不足以解决全部差距，但风险低于修改主干结构；需要比较输出精度、CPU后处理耗时和总延迟。

### 5.4 小算子与特征图搬运

`Concat/Split/Add/Reshape/Resize`合计超过22%，说明模型存在较多拓扑和内存搬运成本。ONNX简化、固定shape、减少冗余Split/Concat/Reshape可能带来收益，但需要原始ONNX和转换日志确认哪些节点可消除。

## 6. 不值得继续优化的方向

- CPU governor/线程亲和性不能显著降低Y2：Y2W-Y2N仅约0.01ms；
- NPU固定1GHz无收益：`rknpu_ondemand`已保持最高频；
- `core012`不会让当前图达到3倍：约83%耗时仍在Core0；
- 输入输出边界不是瓶颈：Input/Output Operator合计35us；
- CPU NMS不是瓶颈：正常模式约0.1～0.2ms；
- 单类别只减少检测头末端，不改变主干和颈部大部分计算。

## 7. PT与实际转换链路核查

已取得`yolo_pt/best.pt`和原转换脚本。checkpoint SHA-256为`e2b982...7ffb`，元数据记录Ultralytics 8.4.131、`yolo26n.yaml`、单类、`reg_max=1`、原生`end2end=True`、640输入、300 epochs、batch 128；记录指标precision=0.94338、recall=0.90882、mAP50=0.95413、mAP50-95=0.64603。

数据集绝对数量不算大，但存在明显质量风险：19752张train只对应约3831个去除Roboflow哈希后的源basename，train-valid/train-test分别有1136/620个basename重叠；抽样250个重叠basename时有45个出现dHash距离不大于5的跨集合近重复。全部label文件均非空，缺少真实负样本。当前高precision/mAP可能受增强副本、相邻帧或源图跨集合泄漏影响；后续模型结构A/B前必须按原始视频/序列重新划分和去重，否则精度比较不可信。

原脚本通过Ultralytics直接`format="rknn"`导出。8.4.131的RKNN专用路径会强制关闭end2end、对框坐标归一化并生成`[1,5,8400]`，随后使用RKNN Toolkit2默认W8A8/normal/channel/optimization level 3转换。当前模型并非缺少`optimization_level=3`；手工普通ONNX路径若没有RKNN专用归一化，会有INT8分类分数精度风险。

网上第一/第三份代码使用的`merge_ptq_and_qat`、`config(custom_hybrid='hybrid.cfg')`、`build(pre_compile=True)`和`eval_perf_accuracy()`均不属于RKNN Toolkit2 2.3.2官方调用方式。第二份基本转换流程可运行，但必须使用RKNN专用、关闭end2end且归一化坐标的ONNX，不能直接拿普通YOLO ONNX替代。

已在`yolo_pt/`提供冻结版本的基线脚本和受控候选脚本，详见`yolo_pt/模型导出与转换说明.md`。

## 8. 后续实施顺序

1. 保存当前模型、性能和精度为不可变基线；
2. 先构建opset19 baseline，确认新流程可复现当前输出和性能；
3. 仅开启`enable_flash_attention=True`构建候选，评估两个attention的约9%占比是否下降；
4. 单独构建opset11 baseline，判断ONNX opset是否改变RKNN图融合；
5. 再做“原始检测头输出、CPU解码”候选，评估约1ms级低风险收益；
6. 再做ReLU/ReLU6训练候选，检查`ConvExSwish`是否变为可三核分摊的普通Conv+激活；
7. 单独评估轻量化/移除attention，不能与激活修改混在一次实验；
8. 所有模型候选都使用相同视频和标注集比较mAP、远距离召回率、误检率、Y2平均/P95/P99；
9. 若只需要降低约12ms队列等待，而非单帧模型时间，另做双context绑定Core0/Core1的帧级并行探针，并丢弃乱序旧结果。

## 9. 当前决策

正式配置继续保持：

```text
npu_core_mode=all
NPU governor=rknpu_ondemand
collect_npu_internal_perf=false
collect_npu_perf_detail=false
input_queue_capacity=1
```

不直接替换激活、删除attention或降低640输入尺寸；这些都需要重新训练/转换和真实小无人机精度验证。
