# 延迟统计器实现文档

## 功能职责

`LatencyStatistics`提供固定容量滑动窗口统计，记录毫秒延迟并输出平均值、最小值、P50、P95、P99和最大值。它只做进程内统计，不负责打点来源、日志或跨设备时钟同步。

## 接口与数据流

```cpp
LatencyStatistics stats(2048);
stats.Add(latency_ms);
LatencySummary summary = stats.Snapshot();
stats.Reset();
```

热路径`Add()`在构造完成后不分配内存；`Snapshot()`才复制当前窗口并排序。

## 关键实现点

- 固定环形窗口，默认2048个样本；
- 非有限值和负值静默忽略；
- `total_count`为累计有效样本数，`window_count`为当前统计窗口样本数；
- 百分位采用相邻排序样本线性插值；
- 互斥锁保护单写多读，快照排序不持有内部锁。

## 日志行为

本模块不打印日志，避免热路径刷屏。参数非法时构造函数抛`std::invalid_argument`。

## 测试方式

```bash
./build/latency_statistics_test
```

覆盖空窗口、平均值/百分位、滑动覆盖、非法样本忽略和Reset。

## 排查/修改要点

窗口容量影响统计稳定性和快照排序开销；不要在逐帧路径调用`Snapshot()`。延迟定义必须由各生产模块文档说明，不能把队列等待、算法处理和跨网络显示延迟混为一项。
