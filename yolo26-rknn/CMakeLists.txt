cmake_minimum_required(VERSION 3.8)

project("yolo26_rknn")

# 依赖查找
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_library(RGA_LIBRARY NAMES rga)

# 查找 RKNN Runtime 库
find_library(RKNN_API_LIBRARY NAMES rknnrt PATHS /usr/lib)
if(NOT RKNN_API_LIBRARY)
    message(FATAL_ERROR "Could not find librknnrt.so")
endif()

# O3 优化
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# ============================================================
# yolo26_detector 封装库
# 默认编译为静态库 (.a)，加 -DYOLO26_BUILD_SHARED=ON 编译为动态库 (.so)
# 对外只需要 include/yolo26_detector.h 这一个头文件
# ============================================================
option(YOLO26_BUILD_SHARED "Build yolo26_detector as shared library (.so) instead of static (.a)" OFF)

set(YOLO26_DETECTOR_SOURCES
    src/yolo26_detector.cpp
    src/rknn_model.cpp
    src/rga_utils.cpp
    src/common.cpp
)

if(YOLO26_BUILD_SHARED)
    add_library(yolo26_detector SHARED ${YOLO26_DETECTOR_SOURCES})
else()
    add_library(yolo26_detector STATIC ${YOLO26_DETECTOR_SOURCES})
endif()

set_property(TARGET yolo26_detector PROPERTY CXX_STANDARD 17)
# 静态库也开启 PIC，方便后续被链接进动态库
set_property(TARGET yolo26_detector PROPERTY POSITION_INDEPENDENT_CODE ON)

# 对外暴露的头文件路径（唯一对外头文件为 include/yolo26_detector.h）
target_include_directories(yolo26_detector PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# 内部实现依赖的头文件（RKNN API / RGA），不对外暴露
target_include_directories(yolo26_detector PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/Linux/librknn_api/include
    /usr/include/rga
)

target_link_libraries(yolo26_detector
    PUBLIC  ${OpenCV_LIBS}
    PRIVATE ${RKNN_API_LIBRARY} ${RGA_LIBRARY}
)

# ============================================================
# 原有演示可执行程序（main.cpp 保持不变，改为链接封装库）
# ============================================================
add_executable(yolo26_rknn main.cpp)

set_property(TARGET yolo26_rknn PROPERTY CXX_STANDARD 17)

target_include_directories(yolo26_rknn PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/Linux/librknn_api/include
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/rga
)

# main.cpp 直接调用了 rknn_query 等 RKNN API。
# 封装库为动态库时其 PRIVATE 依赖不会传递给使用方，因此这里需要显式链接 librknnrt
target_link_libraries(yolo26_rknn yolo26_detector ${RKNN_API_LIBRARY})
