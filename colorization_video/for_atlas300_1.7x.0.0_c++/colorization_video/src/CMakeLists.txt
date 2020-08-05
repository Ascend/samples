# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(sample-colorization-video-new)

# Compile options
add_compile_options(-std=c++11)

add_definitions(-DENABLE_DVPP_INTERFACE)

# Specify target generation path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../../../out")

set(OPENCV_PATH "/usr/local")

set(INC_PATH $ENV{DDK_PATH})
if (NOT DEFINED ENV{DDK_PATH})
    set(INC_PATH "/home/c72/Ascend/ascend-toolkit/20.0.0.B002/arm64-linux_gcc7.3.0")
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

# Header path
include_directories(
    ../inc/
    $ENV{HOME}/ascend_ddk/x86/include/
    $ENV{HOME}/ascend_ddk/x86/include/opencv4
    ${INC_PATH}/acllib/include/
)


# add host lib path
link_directories(
    /usr/local/lib
    ${LIB_PATH}
    $ENV{HOME}/ascend_ddk/x86/lib/
    $ENV{HOME}/Ascend/
    ${INC_PATH}/atc/lib64
)

add_executable(main
        utils.cpp
        model_process.cpp
        colorize_process.cpp
        main.cpp)

target_link_libraries(main
        ascendcl acl_dvpp stdc++ opencv_highgui opencv_core opencv_imgproc opencv_imgcodecs opencv_calib3d opencv_features2d opencv_videoio protobuf presenteragent)

install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
