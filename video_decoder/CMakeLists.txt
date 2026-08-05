cmake_minimum_required(VERSION 3.10)
project(video_decoder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# 查找 MPP 库 (librockchip_mpp.so)
# ============================================================================
find_library(MPP_LIB rockchip_mpp
    PATHS
        /usr/lib
        /usr/lib/aarch64-linux-gnu
        /usr/local/lib
)

if(NOT MPP_LIB)
    message(FATAL_ERROR "未找到 librockchip_mpp.so, 请确认已安装MPP库")
endif()
message(STATUS "MPP库: ${MPP_LIB}")

# 查找 MPP 头文件 (rk_mpi.h)
find_path(MPP_INCLUDE_DIR rk_mpi.h
    PATHS
        /usr/include/rockchip
        /usr/local/include/rockchip
        /usr/include
        /usr/local/include
)

if(NOT MPP_INCLUDE_DIR)
    message(FATAL_ERROR "未找到MPP头文件 (rk_mpi.h)")
endif()
message(STATUS "MPP头文件: ${MPP_INCLUDE_DIR}")

# ============================================================================
# 查找 RGA 库 (librga.so)
# ============================================================================
find_library(RGA_LIB rga
    PATHS
        /usr/lib
        /usr/lib/aarch64-linux-gnu
        /usr/local/lib
)

if(NOT RGA_LIB)
    message(FATAL_ERROR "未找到 librga.so, 请确认已安装RGA库")
endif()
message(STATUS "RGA库: ${RGA_LIB}")

# 查找 RGA 头文件 (im2d.hpp)
find_path(RGA_INCLUDE_DIR im2d.hpp
    PATHS
        /usr/include/rga
        /usr/local/include/rga
        /usr/include
        /usr/local/include
)

if(NOT RGA_INCLUDE_DIR)
    message(FATAL_ERROR "未找到RGA头文件 (im2d.hpp)")
endif()
message(STATUS "RGA头文件: ${RGA_INCLUDE_DIR}")

# ============================================================================
# 构建目标
# ============================================================================
add_executable(video_decoder
    main.cpp
    VideoDecoder.cpp
)

# 头文件路径: MPP的父目录 (使 #include <rockchip/rk_mpi.h> 生效)
# 以及 RGA 的父目录 (使 #include <rga/im2d.hpp> 生效)
get_filename_component(MPP_PARENT_DIR ${MPP_INCLUDE_DIR} DIRECTORY)
get_filename_component(RGA_PARENT_DIR ${RGA_INCLUDE_DIR} DIRECTORY)

target_include_directories(video_decoder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${MPP_PARENT_DIR}
    ${RGA_PARENT_DIR}
    ${MPP_INCLUDE_DIR}
    ${RGA_INCLUDE_DIR}
)

# 链接库
target_link_libraries(video_decoder
    ${MPP_LIB}
    ${RGA_LIB}
    pthread
)

# 安装规则 (可选)
install(TARGETS video_decoder RUNTIME DESTINATION bin)
