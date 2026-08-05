cmake_minimum_required(VERSION 3.10)

project(rtsp_yolo_stream)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV ${OpenCV_VERSION}: ${OpenCV_INCLUDE_DIRS}")

# ------------------------------------------------------------
# yolo26-rknn 封装库（libyolo26_detector.so）
# 需要先在 ../yolo26-rknn 目录下执行：
#   cmake -B build -DYOLO26_BUILD_SHARED=ON && cmake --build build
# ------------------------------------------------------------
set(YOLO26_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../yolo26-rknn" CACHE PATH "yolo26-rknn 工程目录")

find_library(YOLO26_DETECTOR_LIBRARY
    NAMES yolo26_detector
    PATHS ${YOLO26_DIR}/build ${YOLO26_DIR}/lib
    NO_DEFAULT_PATH)
find_library(YOLO26_DETECTOR_LIBRARY NAMES yolo26_detector)
if(NOT YOLO26_DETECTOR_LIBRARY)
    message(FATAL_ERROR "未找到 libyolo26_detector，请先编译 ${YOLO26_DIR}（-DYOLO26_BUILD_SHARED=ON）")
endif()
message(STATUS "yolo26_detector: ${YOLO26_DETECTOR_LIBRARY}")

# 硬编码要求 FFmpeg 5.0+（rkmppenc）或 nyanmisaka/ffmpeg-rockchip，
# 通常安装到 /usr/local；香橙派镜像自带的 FFmpeg 4.4 只有解码器、没有编码器
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(rtsp_yolo_stream
    src/main.cpp
    src/rtsp_decoder.cpp
    src/rtsp_encoder.cpp)

target_include_directories(rtsp_yolo_stream PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${YOLO26_DIR}/include
    ${OpenCV_INCLUDE_DIRS})

# 把库所在目录写入 RPATH，运行时可直接找到 libyolo26_detector.so
set_target_properties(rtsp_yolo_stream PROPERTIES
    BUILD_RPATH "${YOLO26_DIR}/build;/usr/local/lib")

target_link_libraries(rtsp_yolo_stream
    ${YOLO26_DETECTOR_LIBRARY}
    -lavformat -lavcodec -lavutil -lswscale
    ${OpenCV_LIBS}
    pthread)
