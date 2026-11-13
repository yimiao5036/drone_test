cmake_minimum_required(VERSION 3.16)

# =============================================================================
# visual_track_control —— 视觉跟踪控制律静态库（独立子工程）
#
# 实现 docs/物理追踪思路.md 的水平航向控制（§4）、垂直/高度双支路控制（§5）、
# 距离 PID（§6）、全量限幅（§8）、Coast 清积分与判丢刹车（§6.4）、
# 超龄降级（§7.1、§9.2）。
#
# 构建约束：
#   - 与主工程完全解耦：根 CMakeLists.txt 零改动，可独立构建；
#   - 仅依赖 third_party：nlohmann（单头文件）、spdlog（源码）、
#     c_library_v2-master（MAVLink 纯头文件）；
#   - target_tracker 为 SHARED 库且零源码修改，原样 add_subdirectory 复用，
#     TrackResult 出现在本库公共头（vtc_types.h），故以 PUBLIC 链接；
#   - MAVLink include 目录设为 SYSTEM PRIVATE：压告警噪音且不泄漏给下游，
#     全工程仅 src/mavlink_command_gateway.cpp 一个 TU 包含 mavlink.h
#     （MAVLINK_HELPER 默认空宏，多 TU 包含会多重定义）。
# =============================================================================

project(visual_track_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# target_tracker 是 SHARED 库：BUILD_RPATH 指向构建目录，示例无需
# LD_LIBRARY_PATH 即可直接运行（部署时需随带 libtarget_tracker.so.1）。
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR};${CMAKE_BINARY_DIR}/target_tracker;$ORIGIN")

# ---- 依赖：target_tracker（零改动复用；其测试块由 -DBUILD_TESTING=OFF 压制） ----
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../target_tracker
                 ${CMAKE_BINARY_DIR}/target_tracker)

# ---- 依赖：spdlog（守卫引入，避免挂载进根构建时重复定义；关闭示例/测试/动态库/安装） ----
if(NOT TARGET spdlog::spdlog)
    set(SPDLOG_BUILD_EXAMPLE    OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_EXAMPLE_HO OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_TESTS      OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_TESTS_HO   OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_SHARED     OFF CACHE BOOL "" FORCE)
    set(SPDLOG_INSTALL          OFF CACHE BOOL "" FORCE)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/spdlog
                     ${CMAKE_BINARY_DIR}/spdlog)
endif()

# ---- 静态库本体 ----
add_library(visual_track_control STATIC
    src/vtc_config.cpp
    src/pid_controller.cpp
    src/line_of_sight.cpp
    src/tracking_control_law.cpp
    src/mavlink_command_gateway.cpp)

target_include_directories(visual_track_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# nlohmann JSON：单头文件，仅配置加载器（启动期）使用，不进入热路径
target_include_directories(visual_track_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/nlohmann)

# MAVLink 纯头文件库：SYSTEM 压告警、PRIVATE 不泄漏（公共头无任何 mavlink_* 类型）
target_include_directories(visual_track_control SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/c_library_v2-master)

target_link_libraries(visual_track_control
    PUBLIC  target_tracker
    PRIVATE spdlog::spdlog)
target_compile_features(visual_track_control PUBLIC cxx_std_17)

# 编译告警：与主工程一致
if(MSVC)
    target_compile_options(visual_track_control PRIVATE /W4 /permissive-)
else()
    target_compile_options(visual_track_control PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- 示例可执行文件（合成检测 → 追踪 → 控制律 → MAVLink 字节流打印） ----
add_executable(track_control_demo examples/track_control_demo.cpp)
target_link_libraries(track_control_demo PRIVATE visual_track_control spdlog::spdlog)
target_compile_definitions(track_control_demo PRIVATE
    VTC_DEMO_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/track_control.json")
if(MSVC)
    target_compile_options(track_control_demo PRIVATE /W4 /permissive-)
else()
    target_compile_options(track_control_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- 单元测试（Google Test） ----
# 独立选项：构建本子工程时需 -DBUILD_TESTING=OFF 压制 target_tracker 的测试，
# 故本库测试开关不复用 BUILD_TESTING，二者互不干扰。
# 开发机未安装 GTest 时自动跳过（不影响库与示例构建）。
option(VTC_BUILD_TESTS "构建 visual_track_control 单元测试" ON)
if(VTC_BUILD_TESTS)
    find_package(GTest QUIET)
    if(GTest_FOUND)
        enable_testing()
        include(GoogleTest)
        add_executable(vtc_unit_tests
            tests/rate_limiter_test.cpp
            tests/staleness_checker_test.cpp
            tests/frame_transform_test.cpp
            tests/line_of_sight_test.cpp
            tests/pid_controller_test.cpp
            tests/tracking_control_law_test.cpp
            tests/vtc_config_test.cpp
            tests/mavlink_command_gateway_test.cpp)
        target_link_libraries(vtc_unit_tests PRIVATE
            visual_track_control GTest::gtest GTest::gtest_main)
        target_compile_definitions(vtc_unit_tests PRIVATE
            VTC_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/track_control.json"
            VTC_TEST_TMP_DIR="${CMAKE_CURRENT_BINARY_DIR}")
        if(MSVC)
            target_compile_options(vtc_unit_tests PRIVATE /W4 /permissive-)
        else()
            target_compile_options(vtc_unit_tests PRIVATE -Wall -Wextra -Wpedantic)
        endif()
        gtest_discover_tests(vtc_unit_tests)
    else()
        message(STATUS "未找到 GTest：跳过 vtc_unit_tests 单元测试")
    endif()
endif()
