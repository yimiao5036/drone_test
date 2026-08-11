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

# ---- video 模块（视频帧句柄与内存池） ----
add_library(drone_video STATIC
    src/video/video_frame_pool.cpp)
target_include_directories(drone_video PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_video PUBLIC cxx_std_17)
# 关键路径日志使用 spdlog（视频池内部打印，不向使用者暴露）
target_link_libraries(drone_video PRIVATE spdlog::spdlog)

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
    else()
        message(STATUS "GoogleTest not found; topic tests will not be built")
    endif()
endif()
