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

# ---- video 模块（帧句柄、内存池、摄像头接收、解码、叠加） ----
add_library(drone_video STATIC
    src/video/video_frame_pool.cpp
    src/video/camera_receiver.cpp
    src/video/camera_receiver_stub.cpp
    src/video/video_decoder.cpp
    src/video/video_decoder_stub.cpp
    src/video/frame_compositor.cpp)
target_include_directories(drone_video PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_video PUBLIC cxx_std_17)
# 关键路径日志使用 spdlog；FFmpeg 依赖仅实现内部使用（PIMPL 不泄漏到头文件）
target_link_libraries(drone_video PRIVATE spdlog::spdlog PkgConfig::FFMPEG)

# ---- video_transmission 模块（图传发送接口 + 编码后端） ----
add_library(drone_video_transmission STATIC
    src/video_transmission/video_sender.cpp
    src/video_transmission/video_encoder.cpp)
target_include_directories(drone_video_transmission PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_video_transmission PUBLIC cxx_std_17)
# 编码后端内部使用 FFmpeg（PIMPL 不泄漏到头文件），故链接到 drone_perception 同级
# 此处链接 FFMPEG 到 transmission（编码器内部用），头文件不依赖 FFmpeg。
target_link_libraries(drone_video_transmission PRIVATE spdlog::spdlog)
# 编码后端内部使用 FFmpeg（PIMPL 不泄漏到头文件）。静态库符号需在最终链接传播，
# 故 FFMPEG 设为 PUBLIC，使下游可执行文件（测试/主程序）能解析编码器符号。
target_link_libraries(drone_video_transmission PUBLIC PkgConfig::FFMPEG)

# ---- perception 模块（YOLO、光流、激光雷达、融合、目标估计接口） ----
# RKNN 推理后端（香橙派专用）：ARM64 原生构建默认开启，x86_64 开发机构建默认关闭。
# 已有 build/CMakeCache.txt 会保留旧值；需重新选择时使用
# cmake -S . -B build -U DRONE_HAVE_RKNN，或显式传入 -DDRONE_HAVE_RKNN=ON/OFF。
set(DRONE_HAVE_RKNN_DEFAULT OFF)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(DRONE_HAVE_RKNN_DEFAULT ON)
endif()
option(DRONE_HAVE_RKNN "启用 RKNN 推理后端（需香橙派 RKNN runtime 与 RGA）"
       ${DRONE_HAVE_RKNN_DEFAULT})
message(STATUS "RKNN 推理后端: ${DRONE_HAVE_RKNN}（目标架构=${CMAKE_SYSTEM_PROCESSOR}）")
set(DRONE_RKNN_API_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rknn/include" CACHE PATH "RKNN API 头文件目录（含 rknn_api.h）")
set(DRONE_RGA_DIR "/usr/include/rga" CACHE PATH "RGA 头文件目录（含 rga.h / im2d.hpp）")

set(PERCEPTION_SOURCES
    src/perception/yolo_detector.cpp
    src/perception/yolo_detector_stub.cpp
    src/perception/yolo_postprocess.cpp
    src/perception/optical_flow_estimator.cpp
    src/perception/laser_range_finder.cpp
    src/perception/perception_fusion.cpp
    src/perception/target_estimator.cpp)
if(DRONE_HAVE_RKNN)
    find_library(DRONE_RKNN_LIBRARY NAMES rknnrt)
    if(NOT DRONE_RKNN_LIBRARY)
        message(FATAL_ERROR "DRONE_HAVE_RKNN=ON 但未找到 librknnrt.so（香橙派需安装 RKNN runtime）")
    endif()
    find_library(DRONE_RGA_LIBRARY NAMES rga)
    if(NOT DRONE_RGA_LIBRARY)
        message(FATAL_ERROR "DRONE_HAVE_RKNN=ON 但未找到 librga.so（香橙派需安装 librga）")
    endif()
    list(APPEND PERCEPTION_SOURCES src/perception/rknn_detection_backend.cpp)
endif()

add_library(drone_perception STATIC ${PERCEPTION_SOURCES})
target_include_directories(drone_perception PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_perception PUBLIC cxx_std_17)
target_link_libraries(drone_perception PRIVATE spdlog::spdlog)
if(DRONE_HAVE_RKNN)
    target_compile_definitions(drone_perception PRIVATE DRONE_HAVE_RKNN=1)
    target_include_directories(drone_perception PRIVATE
        ${DRONE_RKNN_API_DIR} ${DRONE_RGA_DIR})
    target_link_libraries(drone_perception PRIVATE
        ${DRONE_RKNN_LIBRARY} ${DRONE_RGA_LIBRARY})
endif()

# ---- communication 模块（串口封装、PX4/地面站链路接口） ----
add_library(drone_communication STATIC
    src/communication/serial_port.cpp
    src/communication/communication_transport.cpp
    src/communication/mavlink_handler.cpp
    src/communication/px4_link.cpp
    src/communication/ground_station_link.cpp)
target_include_directories(drone_communication PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
# MAVLink 为生成的第三方 C 头文件，按 SYSTEM 传播以屏蔽其 packed/匿名结构体警告；
# 项目自身代码仍保持 -Wall -Wextra -Wpedantic。
target_include_directories(drone_communication SYSTEM PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/c_library_v2-master>)
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

# ---- config 模块（正式主程序强类型配置） ----
add_library(drone_config STATIC
    src/config/config.cpp)
target_include_directories(drone_config PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_config PUBLIC cxx_std_17)
target_link_libraries(drone_config PUBLIC
    drone_common
    drone_communication
    drone_video
    drone_perception
    drone_video_transmission)

# ---- application 模块（正式主程序组件所有权、Topic 接线与生命周期） ----
add_library(drone_application STATIC
    src/application/drone_application.cpp)
target_include_directories(drone_application PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_application PUBLIC cxx_std_17)
target_link_libraries(drone_application PUBLIC
    drone_config
    drone_communication
    drone_video
    drone_perception
    drone_video_transmission
    spdlog::spdlog)

add_executable(drone_control
    main.cpp
    src/common/logger.cpp)

# 每次构建都同步模型和配置到可执行文件旁，匹配 /proc/self/exe 路径解析约定。
# 不能只使用 drone_control 的 POST_BUILD：当只修改 JSON/模型而可执行文件无需重链时，
# POST_BUILD 不会触发，容易继续读取 build/config 下的旧配置。
add_custom_target(drone_runtime_assets
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/models"
            "${CMAKE_BINARY_DIR}/models"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/config"
            "${CMAKE_BINARY_DIR}/config"
    COMMENT "同步 models/ 和 config/ 到构建目录"
    VERBATIM)
add_dependencies(drone_control drone_runtime_assets)
# 视频链路装配：拉流/解码(video)、识别(perception)、叠加(video)、编码推流(video_transmission)。
# drone_video 的 FFmpeg 依赖为 PRIVATE，需在此补链 FFmpeg 以解析解码/接收符号。
target_link_libraries(drone_control PRIVATE
    drone_common
    drone_application
    spdlog::spdlog
    PkgConfig::FFMPEG)

if(MSVC)
    target_compile_options(drone_control PRIVATE /W4 /permissive-)
else()
    target_compile_options(drone_control PRIVATE -Wall -Wextra -Wpedantic)
endif()

# PX4 串口冒烟与强制UDP的SITL分级测试：独立于正式主程序。
add_executable(px4_link_smoke
    tools/px4_link_smoke.cpp
    src/common/logger.cpp)
target_link_libraries(px4_link_smoke PRIVATE
    drone_common
    drone_communication
    spdlog::spdlog)
if(MSVC)
    target_compile_options(px4_link_smoke PRIVATE /W4 /permissive-)
else()
    target_compile_options(px4_link_smoke PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_custom_command(TARGET px4_link_smoke POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/config"
            "${CMAKE_BINARY_DIR}/config"
    COMMENT "复制 config/ 到 PX4 冒烟测试构建目录"
    VERBATIM)

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

        # 串口测试：使用 Linux 伪终端验证 poll 超时、双向收发与重启，不依赖真实硬件
        add_executable(serial_port_test tests/communication/serial_port_test.cpp)
        target_link_libraries(serial_port_test PRIVATE
            drone_communication spdlog::spdlog GTest::gtest_main)
        if(MSVC)
            target_compile_options(serial_port_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(serial_port_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(serial_port_test)

        # 通信传输测试：UDP 双向数据报、动态学习对端和配置校验（供 PX4 SITL）
        add_executable(communication_transport_test
            tests/communication/communication_transport_test.cpp)
        target_link_libraries(communication_transport_test PRIVATE
            drone_communication GTest::gtest_main)
        if(MSVC)
            target_compile_options(communication_transport_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(communication_transport_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(communication_transport_test)

        # MAVLink 字节流测试：验证 v1/v2、半包、粘包、坏 CRC 恢复和实例隔离
        add_executable(mavlink_handler_test tests/communication/mavlink_handler_test.cpp)
        target_link_libraries(mavlink_handler_test PRIVATE
            drone_communication GTest::gtest_main)
        if(MSVC)
            target_compile_options(mavlink_handler_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(mavlink_handler_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(mavlink_handler_test)

        # PX4 链路基础测试：PTY 串口、本机/PX4 心跳、连接超时、快照发布与重启
        add_executable(px4_link_test tests/communication/px4_link_test.cpp)
        target_link_libraries(px4_link_test PRIVATE
            drone_communication GTest::gtest_main)
        if(MSVC)
            target_compile_options(px4_link_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(px4_link_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(px4_link_test)

        # 地面站链路测试：PTY串口、MAVLink 2遥测编码、GCS心跳与重启
        add_executable(ground_station_link_test
            tests/communication/ground_station_link_test.cpp)
        target_link_libraries(ground_station_link_test PRIVATE
            drone_communication GTest::gtest_main)
        if(MSVC)
            target_compile_options(ground_station_link_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(ground_station_link_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(ground_station_link_test)

        # 正式主程序配置测试：生产配置解析和控制安全门禁
        add_executable(config_test tests/config/config_test.cpp)
        target_link_libraries(config_test PRIVATE drone_config GTest::gtest_main)
        target_compile_definitions(config_test PRIVATE
            TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
        if(MSVC)
            target_compile_options(config_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(config_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(config_test)

        # YOLO 后处理纯函数测试：量化解码、NMS、letterbox 逆变换（开发机可跑）
        add_executable(yolo_postprocess_test tests/perception/yolo_postprocess_test.cpp)
        target_link_libraries(yolo_postprocess_test PRIVATE
            drone_common drone_perception GTest::gtest_main)
        if(MSVC)
            target_compile_options(yolo_postprocess_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(yolo_postprocess_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(yolo_postprocess_test)

        # YOLO 检测器测试：注入 Mock 后端验证线程/发布/统计（不依赖硬件）
        add_executable(yolo_detector_test tests/perception/yolo_detector_test.cpp)
        target_link_libraries(yolo_detector_test PRIVATE
            drone_common drone_video drone_perception GTest::gtest_main)
        if(MSVC)
            target_compile_options(yolo_detector_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(yolo_detector_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(yolo_detector_test)

        # 叠加器测试：验证 NV12 帧 + 检测 → 标注帧（不依赖硬件）
        add_executable(frame_compositor_test tests/video/frame_compositor_test.cpp)
        target_link_libraries(frame_compositor_test PRIVATE
            drone_common drone_video GTest::gtest_main)
        if(MSVC)
            target_compile_options(frame_compositor_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(frame_compositor_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(frame_compositor_test)

        # 图传发送器测试：注入 Mock 编码后端验证线程/计数（不依赖真实 RTSP）
        add_executable(video_sender_test tests/video_transmission/video_sender_test.cpp)
        target_link_libraries(video_sender_test PRIVATE
            drone_common drone_video drone_video_transmission GTest::gtest_main)
        if(MSVC)
            target_compile_options(video_sender_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(video_sender_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(video_sender_test)

        # 编码后端开发机集成验证：软件编码 NV12→本地 MPEG-TS 文件
        add_executable(video_encoder_integration_test
            tests/video_transmission/video_encoder_integration_test.cpp)
        target_link_libraries(video_encoder_integration_test PRIVATE
            drone_common drone_video drone_video_transmission GTest::gtest_main)
        if(MSVC)
            target_compile_options(video_encoder_integration_test PRIVATE /W4 /permissive-)
        else()
            target_compile_options(video_encoder_integration_test PRIVATE -Wall -Wextra -Wpedantic)
        endif()

        include(GoogleTest)
        gtest_discover_tests(video_encoder_integration_test)
    else()
        message(STATUS "GoogleTest not found; topic tests will not be built")
    endif()
endif()
