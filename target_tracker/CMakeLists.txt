cmake_minimum_required(VERSION 3.16)

# =============================================================================
# target_tracker —— 视觉追踪库（独立动态库工程）
#
# 轻量级多目标追踪过滤器：恒定速度卡尔曼滤波 + 贪婪关联 + 主目标锁定。
# 规格见 docs/视觉追踪技术路线.md；实现文档见 target_tracker.md。
# 零第三方依赖（不引入 spdlog / Eigen / OpenCV），仅链接 C++ 标准库。
# =============================================================================

project(target_tracker VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- 动态库目标 ----
add_library(target_tracker SHARED
    src/kalman_box_filter.cpp
    src/tracklet.cpp
    src/target_tracker.cpp)
target_include_directories(target_tracker PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(target_tracker PUBLIC cxx_std_17)

# 版本与符号：VERSION 1.0.0 / SOVERSION 1；默认符号可见性（不加
# visibility=hidden 与导出宏），全部公共类直接对外导出。
set_target_properties(target_tracker PROPERTIES
    VERSION 1.0.0
    SOVERSION 1)

# 编译告警：与主工程一致（非 MSVC：-Wall -Wextra -Wpedantic）
if(MSVC)
    target_compile_options(target_tracker PRIVATE /W4 /permissive-)
else()
    target_compile_options(target_tracker PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- 单元测试（GTest；缺失时仅提示不失败，与主工程策略一致） ----
include(CTest)
if(BUILD_TESTING)
    find_package(GTest QUIET)
    if(GTest_FOUND)
        include(GoogleTest)

        # 测试可执行文件直接链接共享库 target_tracker；
        # BUILD_RPATH 指向库输出目录，保证 ctest 无需 LD_LIBRARY_PATH 即可运行
        set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR};$ORIGIN")

        # 卡尔曼滤波器测试：初始化状态、静止收敛、匀速速度收敛与预测超前、未初始化行为
        add_executable(kalman_box_filter_test tests/kalman_box_filter_test.cpp)
        target_link_libraries(kalman_box_filter_test PRIVATE
            target_tracker GTest::gtest_main)
        if(MSVC)
            target_compile_options(kalman_box_filter_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(kalman_box_filter_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()
        gtest_discover_tests(kalman_box_filter_test)

        # 轨道与几何工具测试：IoU 边界、IsDead/IsCoasting、Predict/Update/MarkMiss 计数
        add_executable(tracklet_test tests/tracklet_test.cpp)
        target_link_libraries(tracklet_test PRIVATE
            target_tracker GTest::gtest_main)
        if(MSVC)
            target_compile_options(tracklet_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(tracklet_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()
        gtest_discover_tests(tracklet_test)

        # 追踪器测试：逐条覆盖文档 §9.4 六条行为一致性验收用例
        add_executable(target_tracker_test tests/target_tracker_test.cpp)
        target_link_libraries(target_tracker_test PRIVATE
            target_tracker GTest::gtest_main)
        if(MSVC)
            target_compile_options(target_tracker_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(target_tracker_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()
        gtest_discover_tests(target_tracker_test)
    else()
        message(STATUS "GoogleTest not found; target_tracker tests will not be built")
    endif()
endif()
