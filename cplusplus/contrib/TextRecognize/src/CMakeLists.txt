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

if (${CMAKE_CXX_COMPILER} MATCHES "aarch")
    set(ARCH "arm")
    list(APPEND COMMON_DEPEND_LIB media_mini)
    add_definitions(-DENABLE_BOARD_CAMARE)
#    set(INC_PATH "$ENV{HOME}/Ascend/ascend-toolkit/20.1.rc1/arm64-linux")
else ()
    set(ARCH "x86")
#    set(INC_PATH "$ENV{HOME}/Ascend/ascend-toolkit/20.1.rc1/x86_64-linux")
endif()

set(INC_PATH $ENV{DDK_PATH})
set(LIB_PATH $ENV{NPU_HOST_LIB})

if (NOT DEFINED ENV{NPU_HOST_LIB})
    set(LIB_PATH "/usr/local/Ascend/acllib/lib64/stub/")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else ()
    message(STATUS "env LIB_PATH: ${LIB_PATH}")
endif()

# Header path
include_directories(
    ../inc/
    $ENV{HOME}/ascend_ddk/${ARCH}/include/opencv4/
    ${INC_PATH}/acllib/include/
    $ENV{HOME}/ascend_ddk/${ARCH}/include/
    $ENV{HOME}/ascend_ddk/${ARCH}/include/ascenddk
    $ENV{HOME}/Ascend/
    $ENV{HOME}/Ascend/ascenddk/
)

if(target STREQUAL "Simulator_Function")
    add_compile_options(-DFUNC_SIM)
endif()

# add host lib path
link_directories(
    ${LIB_PATH}
    $ENV{HOME}/ascend_ddk/${ARCH}/lib/
    $ENV{HOME}/Ascend/driver/
    $ENV{HOME}/Ascend/
    ${INC_PATH}/atc/lib64/
)

add_executable(main
        TextRecongnize.cpp
        clipper.cpp
        main.cpp)

list(APPEND COMMON_DEPEND_LIB atlasutil avcodec avformat avdevice avutil swresample avfilter swscale)

if(target STREQUAL "Simulator_Function")
    target_link_libraries(main funcsim)
else()
    target_link_libraries(main ascendcl acl_dvpp stdc++ ${COMMON_DEPEND_LIB} opencv_core opencv_imgproc opencv_imgcodecs dl rt jsoncpp c_sec pthread protobuf presenteragent)
endif()

install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
