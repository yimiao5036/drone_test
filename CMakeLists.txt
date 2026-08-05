cmake_minimum_required(VERSION 3.16)

project(drone_control_system VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(drone_common INTERFACE)
target_include_directories(drone_common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(drone_common INTERFACE cxx_std_17)

add_executable(drone_control main.cpp)
target_link_libraries(drone_control PRIVATE drone_common)

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
    else()
        message(STATUS "GoogleTest not found; topic tests will not be built")
    endif()
endif()
