cmake_minimum_required(VERSION 3.16)

# =============================================================================
# stereo_depth —— 传统双目测距传感器（独立子工程，纯 CPU / ROI 全分辨率稠密匹配）
#
# 对应设计：docs/superpowers/specs/2026-09-29-传统双目测距传感器-design.md
#
# 构建约束：
#   - 与主工程完全解耦：根 CMakeLists.txt 零改动，可独立构建；
#   - 不依赖主项目任何头文件（Topic/VideoFrame/types.h 均不引入）；
#   - 仅依赖 third_party：nlohmann（单头文件，配置加载）、spdlog（源码，日志）；
#   - 核心算法（校正/remap/Census/SGM/亚像素/后处理）全自研，不引入 OpenCV；
#   - <arm_neon.h> 为编译器内建，可选加速，非 NEON 平台走标量回退。
# =============================================================================

project(stereo_depth LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ---- 依赖：spdlog（守卫引入，避免挂载进根构建时重复定义；关闭示例/测试/动态库/安装）----
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
add_library(stereo_depth STATIC
    src/stereo_depth_config.cpp
    src/stereo_rectifier.cpp
    src/census_transform.cpp
    src/sgm_matcher.cpp
    src/subpixel_refiner.cpp
    src/disparity_postprocess.cpp
    src/stereo_depth_sensor.cpp)

target_include_directories(stereo_depth PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# nlohmann JSON：单头文件，仅配置/标定加载器（启动期）使用，不进入热路径
target_include_directories(stereo_depth PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/nlohmann)

target_link_libraries(stereo_depth PRIVATE spdlog::spdlog)
target_compile_features(stereo_depth PUBLIC cxx_std_17)

# ARM64 上启用 NEON；x86 开发机自动忽略（标量回退）。
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    target_compile_options(stereo_depth PRIVATE -DSTEREO_DEPTH_USE_NEON=1)
endif()

# 线程：Stage-1/Stage-2 流水线
find_package(Threads REQUIRED)
target_link_libraries(stereo_depth PRIVATE Threads::Threads)

# 编译告警：与主工程一致
if(MSVC)
    target_compile_options(stereo_depth PRIVATE /W4 /permissive-)
else()
    target_compile_options(stereo_depth PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- 离线示例（读左右 PGM/裸灰度 + 标定，跑全流程，脱离主项目验证精度）----
add_executable(stereo_depth_demo examples/offline_pgm_demo.cpp)
target_link_libraries(stereo_depth_demo PRIVATE stereo_depth spdlog::spdlog)
target_compile_definitions(stereo_depth_demo PRIVATE
    STEREO_DEPTH_DEMO_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/stereo_depth.json")
if(MSVC)
    target_compile_options(stereo_depth_demo PRIVATE /W4 /permissive-)
else()
    target_compile_options(stereo_depth_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- 单元测试（Google Test；开发机未安装时自动跳过，不影响库与示例构建）----
option(STEREO_DEPTH_BUILD_TESTS "构建 stereo_depth 单元测试" ON)
if(STEREO_DEPTH_BUILD_TESTS)
    find_package(GTest QUIET)
    if(GTest_FOUND)
        enable_testing()
        include(GoogleTest)
        add_executable(stereo_depth_unit_tests
            tests/stereo_depth_config_test.cpp
            tests/stereo_rectifier_test.cpp
            tests/census_transform_test.cpp
            tests/sgm_matcher_test.cpp
            tests/subpixel_refiner_test.cpp
            tests/disparity_postprocess_test.cpp
            tests/stereo_depth_sensor_test.cpp)
        target_link_libraries(stereo_depth_unit_tests PRIVATE
            stereo_depth GTest::gtest GTest::gtest_main)
        target_compile_definitions(stereo_depth_unit_tests PRIVATE
            STEREO_DEPTH_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/stereo_depth.json"
            STEREO_DEPTH_TEST_CALIB="${CMAKE_CURRENT_SOURCE_DIR}/../config/camera_calibration_uvc_1280x720.json")
        if(MSVC)
            target_compile_options(stereo_depth_unit_tests PRIVATE /W4 /permissive-)
        else()
            target_compile_options(stereo_depth_unit_tests PRIVATE -Wall -Wextra -Wpedantic)
        endif()
        gtest_discover_tests(stereo_depth_unit_tests)
    else()
        message(STATUS "未找到 GTest：跳过 stereo_depth_unit_tests 单元测试")
    endif()
endif()
