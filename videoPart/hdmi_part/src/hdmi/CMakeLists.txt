# HDMI 直显后端（DRM/KMS + GBM + EGL/GLES）构建片段
#
# 本文件是新增子目录的一部分，**不修改任何既有文件**。两种接入方式：
#
# 1) 主工程接入（推荐）
#    在仓库根 CMakeLists.txt **末尾**（即 drone_video_transmission 定义之后）增加一行：
#        add_subdirectory(src/video_transmission/hdmi)
#    本文件会新建独立静态库 drone_hdmi_display，并把它挂到 drone_video_transmission 上。
#    刻意不把源文件追加进既有目标：这样既有源文件的编译选项完全不受影响。
#
# 2) 独立编译验证
#        cmake -S src/video_transmission/hdmi -B build_hdmi
#        cmake --build build_hdmi -j$(nproc)
#    只构建本模块与 hdmi_probe 自检工具，用于快速确认本子目录可编译。
#
# 依赖安装（香橙派 / Ubuntu 24.04）：
#     sudo apt install libdrm-dev libgbm-dev libegl1-mesa-dev libgles2-mesa-dev

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # 独立验证模式：自建最小工程；被 add_subdirectory 引入时跳过
    cmake_minimum_required(VERSION 3.16)
    project(drone_hdmi_display_standalone LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# 项目根目录：本目录为 <根>/src/video_transmission/hdmi
get_filename_component(DRONE_PROJECT_ROOT
                       "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

# ---- DRM/KMS 依赖开关（沿用项目 DRONE_HAVE_* 命名与默认值风格） ----
# ARM64（香橙派）默认开启；x86_64 开发机默认关闭，此时自动回退占位显示器，
# 仍可编译并跑通 VideoSender 全链路，只是不产生真实 HDMI 输出。
set(DRONE_HAVE_HDMI_KMS_DEFAULT OFF)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(DRONE_HAVE_HDMI_KMS_DEFAULT ON)
endif()
option(DRONE_HAVE_HDMI_KMS
       "启用 HDMI 直显的 DRM/KMS+GBM+EGL 后端（需 libdrm/libgbm/libEGL/libGLESv2）"
       ${DRONE_HAVE_HDMI_KMS_DEFAULT})

find_package(PkgConfig QUIET)
if(DRONE_HAVE_HDMI_KMS)
    if(NOT PkgConfig_FOUND)
        message(FATAL_ERROR
            "DRONE_HAVE_HDMI_KMS=ON 需要 pkg-config 来定位图形库")
    endif()
    pkg_check_modules(HDMI_KMS QUIET IMPORTED_TARGET libdrm gbm egl glesv2)
    if(NOT HDMI_KMS_FOUND)
        message(FATAL_ERROR
            "DRONE_HAVE_HDMI_KMS=ON 但未找到图形依赖。安装命令：\n"
            "  sudo apt install libdrm-dev libgbm-dev libegl1-mesa-dev libgles2-mesa-dev\n"
            "若使用厂商 EGL/GLES 实现（如香橙派 libmali），请确认 libEGL.pc / glesv2.pc "
            "在 pkg-config 搜索路径内；仅需关闭 DRM 后端时可加 -DDRONE_HAVE_HDMI_KMS=OFF。")
    endif()
endif()
message(STATUS "HDMI 直显 DRM/KMS 后端: ${DRONE_HAVE_HDMI_KMS}（目标架构=${CMAKE_SYSTEM_PROCESSOR}）")

set(HDMI_DISPLAY_SOURCES
    hdmi_display_backend.cpp
    hdmi_display_stub.cpp
    hdmi_drm_display.cpp)

# ---- 独立编译验证模式：自建 spdlog 依赖 ----
if(NOT TARGET spdlog::spdlog)
    find_package(spdlog QUIET)
endif()
if(NOT TARGET spdlog::spdlog)
    set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_SHARED OFF CACHE BOOL "" FORCE)
    set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)
    add_subdirectory(${DRONE_PROJECT_ROOT}/third_party/spdlog
                     ${CMAKE_BINARY_DIR}/spdlog)
endif()

# ---- 独立静态库：HDMI 显示后端 ----
# 单独成库而非并入既有目标，避免改变既有源文件的编译选项。
if(NOT TARGET drone_hdmi_display)
    add_library(drone_hdmi_display STATIC ${HDMI_DISPLAY_SOURCES})
    target_include_directories(drone_hdmi_display PUBLIC
        $<BUILD_INTERFACE:${DRONE_PROJECT_ROOT}/include>)
    target_compile_features(drone_hdmi_display PUBLIC cxx_std_17)
    # EGL 原生句柄按 GBM 语义使用（无 X11/Wayland 平台），关闭 EGL 头文件的 X11 类型
    target_compile_definitions(drone_hdmi_display PRIVATE
        EGL_NO_X11=1 MESA_EGL_NO_X11_HEADERS=1)
    target_link_libraries(drone_hdmi_display PRIVATE spdlog::spdlog)
    if(DRONE_HAVE_HDMI_KMS)
        target_compile_definitions(drone_hdmi_display PRIVATE DRONE_HAVE_HDMI_KMS=1)
        # 静态库符号需在最终链接传播，故设为 PUBLIC
        target_link_libraries(drone_hdmi_display PUBLIC PkgConfig::HDMI_KMS)
    endif()
    if(MSVC)
        target_compile_options(drone_hdmi_display PRIVATE /W4 /permissive-)
    else()
        target_compile_options(drone_hdmi_display PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()

# ---- 主工程接入：挂到既有图传库上 ----
# 既有文件零改动，只是多链一个库；VideoSender 通过 backend_factory 注入即可使用。
if(TARGET drone_video_transmission)
    target_link_libraries(drone_video_transmission PUBLIC drone_hdmi_display)
elseif(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # 被主工程引入但目标还不存在：说明 add_subdirectory 放错了位置，
    # 此时会静默编译但不会被链接进图传模块，故显式告警。
    message(WARNING
        "未找到 drone_video_transmission 目标："
        "add_subdirectory(src/video_transmission/hdmi) 必须写在该目标定义之后"
        "（建议放在根 CMakeLists.txt 末尾），否则 HDMI 后端不会被链接进图传模块。")
endif()

# ---- 带外自检工具 ----
option(DRONE_BUILD_HDMI_PROBE "构建 HDMI 直显带外自检工具 hdmi_probe" ON)
if(DRONE_BUILD_HDMI_PROBE)
    add_executable(hdmi_probe hdmi_probe.cpp)
    target_compile_features(hdmi_probe PRIVATE cxx_std_17)
    target_link_libraries(hdmi_probe PRIVATE drone_hdmi_display spdlog::spdlog)
    if(MSVC)
        target_compile_options(hdmi_probe PRIVATE /W4 /permissive-)
    else()
        target_compile_options(hdmi_probe PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()
