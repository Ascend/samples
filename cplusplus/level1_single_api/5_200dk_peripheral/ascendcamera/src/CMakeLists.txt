# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(sample_ascendcamera)

# Compile options
add_compile_options(-std=c++11)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../../../out")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

add_definitions(-DENABLE_DVPP_INTERFACE)

if (NOT DEFINED ENV{INSTALL_DIR})
    MESSAGE(FATAL_ERROR "Not Defined INSTALL_DIR")
endif()

if (NOT DEFINED ENV{THIRDPART_PATH})
    MESSAGE(FATAL_ERROR "Not Defined THIRDPART_PATH")
endif()

# Header path
include_directories(
    ../inc/
    $ENV{THIRDPART_PATH}/include/
    $ENV{INSTALL_DIR}/runtime/include/
    $ENV{INSTALL_DIR}/driver
)

if(target STREQUAL "Simulator_Function")
    add_compile_options(-DFUNC_SIM)
endif()

# add host lib path
link_directories(
    $ENV{INSTALL_DIR}/runtime/lib64/stub
    $ENV{THIRDPART_PATH}/lib/
    $ENV{INSTALL_DIR}/driver/
)

add_executable(main 
        ascend_camera_parameter.cpp
        camera.cpp
        dvpp_jpege.cpp
        dvpp_process.cpp
        dvpp_resize.cpp
        main_process.cpp
        output_info_process.cpp
        parameter_utils.cpp
        utils.cpp
        main.cpp)

if(target STREQUAL "Simulator_Function")
    target_link_libraries(main funcsim)
else()
    target_link_libraries(main ascendcl ascend_hal slog acl_dvpp stdc++  c_sec media_mini pthread protobuf presenteragent)
endif()

install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
