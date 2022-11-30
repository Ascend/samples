# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(TextRecongnize)

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

if (NOT DEFINED ENV{CPU_ARCH})
    MESSAGE(FATAL_ERROR "Not Defined CPU_ARCH")
endif()

if($ENV{CPU_ARCH} MATCHES "aarch64")
    message("arm architecture detected")
    if(EXISTS "$ENV{INSTALL_DIR}/driver/libmedia_mini.so")
        message("target aarch64 200dk")
        list(APPEND COMMON_DEPEND_LIB media_mini ascend_hal c_sec mmpa slog)
    else()
        message("target aarch64 300")
    endif()
else()
    message("x86 architecture detected")
    message("target x86 300")
endif()

# Header path
include_directories(
    ../inc/
    $ENV{INSTALL_DIR}/runtime/include/
    $ENV{THIRDPART_PATH}/include/
)

if(target STREQUAL "Simulator_Function")
    add_compile_options(-DFUNC_SIM)
endif()

# add host lib path
link_directories(
    $ENV{THIRDPART_PATH}/lib/
    $ENV{INSTALL_DIR}/driver/
    $ENV{INSTALL_DIR}/runtime/lib64/stub
)

add_executable(main
        TextRecongnize.cpp
        clipper.cpp
        main.cpp)

list(APPEND COMMON_DEPEND_LIB acllite avcodec avformat avdevice avutil swresample avfilter swscale)

if(target STREQUAL "Simulator_Function")
    target_link_libraries(main funcsim)
else()
    target_link_libraries(main ascendcl acllite acl_dvpp stdc++ ${COMMON_DEPEND_LIB} opencv_core opencv_imgproc opencv_imgcodecs dl rt jsoncpp pthread protobuf presenteragent avdevice avfilter)
endif()

install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
