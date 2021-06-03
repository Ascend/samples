# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(classification)

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
    set(LIB_PATH "/usr/local/Ascend/acllib/lib64/stub/")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else ()
    message(STATUS "env LIB_PATH: ${LIB_PATH}")
endif()

add_definitions(-DENABLE_DVPP_INTERFACE)
list(APPEND COMMON_DEPEND_LIB atlasutil avcodec avformat avdevice avutil swresample avfilter swscale)   

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch")
    message("arm64 architecture detected")
    list(APPEND COMMON_DEPEND_LIB media_mini ascend_hal slog c_sec mmpa devmmap)
    add_definitions(-DENABLE_BOARD_CAMARE)
    set(ARCH "arm")
else()
    if (${CMAKE_CXX_COMPILER} MATCHES "aarch")
        list(APPEND COMMON_DEPEND_LIB media_mini ascend_hal slog c_sec mmpa devmmap)
        add_definitions(-DENABLE_BOARD_CAMARE)
        set(ARCH "arm")
    else ()
        set(ARCH "x86")
    endif()
endif()


# Header path
include_directories(
	$ENV{HOME}/ascend_ddk/${ARCH}/include/
	$ENV{HOME}/ascend_ddk/${ARCH}/include/opencv4
    ${INC_PATH}/acllib/include/
    ../inc/
)

if(target STREQUAL "Simulator_Function")
    add_compile_options(-DFUNC_SIM)
endif()

# add host lib path
link_directories(
    ${LIB_PATH}
    $ENV{HOME}/ascend_ddk/${ARCH}/lib/
    ${INC_PATH}/atc/lib64
    $ENV{HOME}/Ascend/driver

)

add_executable(main
        object_detect.cpp
        main.cpp)

if(target STREQUAL "Simulator_Function")
    target_link_libraries(main funcsim)
else()
    target_link_libraries(main ascendcl ${COMMON_DEPEND_LIB} acl_dvpp stdc++ opencv_core opencv_imgproc opencv_imgcodecs dl rt)
endif()

install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
