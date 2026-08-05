#include "common/logger.h"
#include "common/topic.h"

int main() {
    // 初始化全局异步文件日志（生成新日志文件，不覆盖历史）
    drone::common::InitializeAsyncLogger();

    // 根入口只负责组合和启动各模块，具体业务逻辑由独立模块实现。
    SPDLOG_INFO("系统启动");
    return 0;
}
