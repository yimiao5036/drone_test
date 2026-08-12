cmake_minimum_required(VERSION 3.16)

project(drone_control_system VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(drone_common INTERFACE)
target_include_directories(drone_common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann>)
target_compile_features(drone_common INTERFACE cxx_std_17)

# ---- 第三方库：spdlog（异步日志） ----
set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/spdlog)

# FFmpeg（RTSP 拉流与 H.264/H.265 解码；香橙派需安装 ffmpeg-rockchip 开发包）
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavformat libavcodec libavutil libswscale)

# ---- video 模块（帧句柄、内存池、摄像头接收、解码） ----
add_library(drone_video STATIC
    src/video/video_frame_pool.cpp
    src/video/camera_receiver.cpp
    src/video/camera_receiver_stub.cpp
    src/video/video_decoder.cpp
    src/video/video_decoder_stub.cpp)
target_include_directories(drone_video PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_video PUBLIC cxx_std_17)
# 关键路径日志使用 spdlog；FFmpeg 依赖仅实现内部使用（PIMPL 不泄漏到头文件）
target_link_libraries(drone_video PRIVATE spdlog::spdlog PkgConfig::FFMPEG)

# ---- video_transmission 模块（图传发送接口） ----
add_library(drone_video_transmission STATIC
    src/video_transmission/video_sender.cpp)
target_include_directories(drone_video_transmission PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_video_transmission PUBLIC cxx_std_17)
target_link_libraries(drone_video_transmission PRIVATE spdlog::spdlog)

# ---- perception 模块（YOLO、光流、激光雷达、融合、目标估计接口） ----
add_library(drone_perception STATIC
    src/perception/yolo_detector.cpp
    src/perception/optical_flow_estimator.cpp
    src/perception/laser_range_finder.cpp
    src/perception/perception_fusion.cpp
    src/perception/target_estimator.cpp)
target_include_directories(drone_perception PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_perception PUBLIC cxx_std_17)
target_link_libraries(drone_perception PRIVATE spdlog::spdlog)

# ---- communication 模块（串口封装、PX4/地面站链路接口） ----
add_library(drone_communication STATIC
    src/communication/serial_port.cpp
    src/communication/px4_link.cpp
    src/communication/ground_station_link.cpp)
target_include_directories(drone_communication PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_communication PUBLIC cxx_std_17)
target_link_libraries(drone_communication PRIVATE spdlog::spdlog)

# ---- state_machine 模块（任务状态机接口） ----
add_library(drone_state_machine STATIC
    src/state_machine/mission_state_machine.cpp)
target_include_directories(drone_state_machine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_state_machine PUBLIC cxx_std_17)
target_link_libraries(drone_state_machine PRIVATE spdlog::spdlog)

# ---- control 模块（飞行控制接口；库名加 _core 避免与可执行 drone_control 冲突） ----
add_library(drone_control_core STATIC
    src/control/flight_controller.cpp)
target_include_directories(drone_control_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_control_core PUBLIC cxx_std_17)
target_link_libraries(drone_control_core PRIVATE spdlog::spdlog)

# ---- health 模块（健康管理接口） ----
add_library(drone_health STATIC
    src/health/health_manager.cpp)
target_include_directories(drone_health PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_health PUBLIC cxx_std_17)
target_link_libraries(drone_health PRIVATE spdlog::spdlog)

add_executable(drone_control
    main.cpp
    src/common/logger.cpp)
target_link_libraries(drone_control PRIVATE drone_common drone_video spdlog::spdlog)

if(MSVC)
    target_compile_options(drone_control PRIVATE /W4 /permissive-)
else()
    target_compile_options(drone_control PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
    find_package(GTest QUIET)
    if(GTest_FOUND)
        add_executable(topic_test tests/common/topic_test.cpp)
        target_link_libraries(topic_test PRIVATE drone_common GTest::gtest_main)
        if(MSVC)
            target_compile_options(topic_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(topic_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(topic_test)

        add_executable(video_frame_test tests/video/video_frame_test.cpp)
        target_link_libraries(video_frame_test PRIVATE drone_common GTest::gtest_main)
        if(MSVC)
            target_compile_options(video_frame_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(video_frame_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(video_frame_test)

        add_executable(video_frame_pool_test tests/video/video_frame_pool_test.cpp)
        target_link_libraries(video_frame_pool_test PRIVATE drone_common drone_video GTest::gtest_main)
        if(MSVC)
            target_compile_options(video_frame_pool_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(video_frame_pool_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(video_frame_pool_test)

        # 解码器软解测试：用 libx264 编码测试码流，验证 VideoDecoder 输出 NV12 帧
        add_executable(video_decoder_test tests/video/video_decoder_test.cpp)
        target_link_libraries(video_decoder_test PRIVATE
            drone_common drone_video GTest::gtest_main PkgConfig::FFMPEG)
        if(MSVC)
            target_compile_options(video_decoder_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(video_decoder_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(video_decoder_test)

        # 骨架冒烟测试：验证各部件接口抽象与占位实现可构造、可启停
        add_executable(stub_smoke_test tests/skeleton/stub_smoke_test.cpp)
        target_link_libraries(stub_smoke_test PRIVATE
            drone_common
            drone_video
            drone_video_transmission
            drone_perception
            drone_communication
            drone_state_machine
            drone_control_core
            drone_health
            spdlog::spdlog
            GTest::gtest_main)
        if(MSVC)
            target_compile_options(stub_smoke_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(stub_smoke_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(stub_smoke_test)
    else()
        message(STATUS "GoogleTest not found; topic tests will not be built")
    endif()
endif()
