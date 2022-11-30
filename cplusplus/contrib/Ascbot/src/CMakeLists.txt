# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(objectdetection_camera)

# Compile options
add_compile_options(-std=c++11)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../../../out")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

add_definitions(-DENABLE_DVPP_INTERFACE)
set(INC_PATH $ENV{DDK_PATH})

if (NOT DEFINED ENV{DDK_PATH})
    set(INC_PATH "/usr/local/Ascend")
    message(STATUS "set default INC_PATH: ${INC_PATH}")
else ()
    message(STATUS "env INC_PATH: ${INC_PATH}")
endif()

set(LIB_PATH $ENV{NPU_HOST_LIB})

if (NOT DEFINED ENV{NPU_HOST_LIB})
    set(LIB_PATH "/usr/local/Ascend/runtime/lib64/stub/")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else ()
    message(STATUS "env LIB_PATH: ${LIB_PATH}")
endif()

# Header path
include_directories(
    ../inc/
    $ENV{HOME}/ascend_ddk/include/third_party/
    ${INC_PATH}/runtime/include/
    $ENV{HOME}/ascend_ddk/include/
    $ENV{HOME}/Ascend/
    $ENV{HOME}/Ascend/
)

if(target STREQUAL "Simulator_Function")
    add_compile_options(-DFUNC_SIM)
endif()

# add host lib path
link_directories(
    ${LIB_PATH}
    $ENV{HOME}/ascend_ddk/lib/
    $ENV{HOME}/Ascend/driver/
    $ENV{HOME}/Ascend/
    ${INC_PATH}/compiler/lib64/
)

add_executable(main
        engine_handle.cpp
        engine_post_dange.cpp
        i2c.cpp
        wheel.cpp
        utils.cpp
        camera.cpp
        model_process.cpp
        object_detect.cpp
        dvpp_process.cpp
        dvpp_resize.cpp
        oled.cpp
        engine_post_follow.cpp
        engine_post_road.cpp
        tcp_server.cpp
        uart_print.cpp
        main.cpp)

if(target STREQUAL "Simulator_Function")
    target_link_libraries(main funcsim)
else()
    target_link_libraries(main ascendcl acl_dvpp stdc++  c_sec slog media_mini pthread   )
endif()

install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
