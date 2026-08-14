# 异步文件日志（logger）

> 对应实现：`include/common/logger.h`、`src/common/logger.cpp`
> 配置入口：`main.cpp`（读取 `config/config.json`）

## 1. 功能职责

封装 spdlog 异步文件日志，作为全工程统一的日志出口：

- 日志写入 `logs/` 目录文件，每次运行自动递增编号、**不覆盖历史**。
- 提供全局默认 logger，业务代码直接用 `SPDLOG_INFO` / `SPDLOG_WARN` / `SPDLOG_ERROR` 宏即可，无需传递 logger 对象。
- 日志等级可由 `config/config.json` 配置，缺失时用默认值兜底。

不做什么：不负责业务日志内容，不打印到控制台（如需控制台输出需另行配置 sink）。

## 2. 接口与数据流

```cpp
// include/common/logger.h
std::shared_ptr<spdlog::logger> InitializeAsyncLogger(
    const std::string& log_dir,                      // 日志目录，如 "logs/"
    spdlog::level::level_enum level = spdlog::level::debug);
```

- 幂等：重复调用返回同一个实例。
- 初始化后 `spdlog::set_default_logger` 设为全局默认，业务代码直接 `SPDLOG_*` 宏。
- 配置加载在 `main.cpp` 的 `LoadLogConfig()`：从 `config/config.json` 读 `log.dir` / `log.level`；
  文件缺失或解析失败时用 `logs/` + `info` 兜底并输出告警。

## 3. 关键实现点

- **异步线程池**：队列 8192 条、1 个工作线程、每 3 秒冲刷磁盘（`src/common/logger.cpp` 常量）。
- **文件命名规则**：`编号_日期时间.log`（如 `0009_20260812_155342.log`），编号自动递增；
  系统时间不可信（早于 2020-01-01）时时间部分改用 12 位随机 hex，避免文件名冲突。
- **等级参数**：由 main 从 JSON 配置读取传入，`spdlog::level::from_str` 解析。

## 4. 日志行为约定（全局日志纪律）

| 等级 | 用途 | 示例 |
|------|------|------|
| INFO | 生命周期与状态变化 | 模块创建/销毁、启动/停止、连接成功、内存池创建 |
| WARN | 异常降级（可恢复） | 丢帧、断流重连、池满、软解回退 |
| ERROR | 逻辑缺陷与参数错误 | 配置非法、解码器打开失败、非法归还 |

- 异常日志必须节流：第 1 次 + 每满 100 次才打印（`ShouldLogThrottled`），消息带累计计数。
- 高频热路径（每帧/每条消息）不打日志。
- 同一异常只在最合适的一层记录一次。

## 5. 测试方式

- 无专门单测（日志为基础设施）。运行 `./build/drone_control` 后检查 `logs/` 下最新文件：
  ```
  异步文件日志已初始化: logs/000N_...log (等级=info)
  已加载日志配置: 目录=logs/ 等级=info
  系统启动
  ```
- 测试程序（`build/*_test`）不初始化文件 logger，日志直接打到终端（stderr），便于调试。

## 6. 排查/修改要点

| 现象 | 排查方向 |
|------|----------|
| 看不到日志 | 确认 `config/config.json` 存在且 `log.dir` 正确；等级是否被调成 warning 以上 |
| 日志文件一直只有一个编号 | 系统时间可信时按日期命名，同一秒内多次运行可能覆盖——编号递增逻辑检查 |
| 想临时看控制台 | 在 `InitializeAsyncLogger` 后追加 `spdlog::stdout_color_mt("console")` sink（仅调试用） |
| 修改刷新频率 | 调整 `kFlushIntervalSeconds`（3 秒） |
