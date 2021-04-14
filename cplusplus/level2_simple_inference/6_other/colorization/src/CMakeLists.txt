# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(sample-colorization)

# Compile options
add_compile_options(-std=c++11)

add_definitions(-DENABLE_DVPP_INTERFACE)

# Specify target generation path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../../../out")

set(OPENCV_PATH "/usr/local")

set(INC_PATH $ENV{DDK_PATH})
if (NOT DEFINED ENV{DDK_PATH})
    set(INC_PATH "/usr/local/Ascend")
    message(STATUS "set default INC_PATH: ${INC_PATH}")
else ()
    message(STATUS "env INC_PATH: ${INC_PATH}")
endif()

set(LIB_PATH $ENV{NPU_HOST_LIB})
if (NOT DEFINED ENV{NPU_HOST_LIB})
    set(LIB_PATH "/usr/local/Ascend/acllib/lib64/stub/")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else ()
    message(STATUS "env LIB_PATH: ${LIB_PATH}")
endif()

list(APPEND COMMON_DEPEND_LIB atlasutil avcodec avformat avdevice avutil swresample avfilter swscale)
if (${CMAKE_CXX_COMPILER} MATCHES "aarch")
    set(ARCH "arm")
    #list(APPEND COMMON_DEPEND_LIB media_mini)
else ()
    set(ARCH "x86")
endif()

# Header path
include_directories(
    ../inc/
    ${INC_PATH}/acllib/include/
    $ENV{HOME}/ascend_ddk/${ARCH}/include
    $ENV{HOME}/ascend_ddk/${ARCH}/include/opencv4
    ${OPENCV_PATH}/include/opencv4
)

# add host lib path
link_directories(
    ${LIB_PATH}
    $ENV{HOME}/ascend_ddk/${ARCH}/lib/
    ${OPENCV_PATH}/lib
    ${INC_PATH}/atc/lib64
)

add_executable(main
        colorize_process.cpp
        main.cpp)

target_link_libraries(main
        ascendcl acl_dvpp stdc++ ${COMMON_DEPEND_LIB} opencv_highgui opencv_core opencv_imgproc opencv_imgcodecs opencv_calib3d opencv_features2d opencv_videoio)

install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
