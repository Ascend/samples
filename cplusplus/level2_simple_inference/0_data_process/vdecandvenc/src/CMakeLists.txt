# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(vdecandvenc)

# Compile options
add_compile_options(-std=c++11)

add_definitions(-DENABLE_DVPP_INTERFACE)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../../../out")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

if (NOT DEFINED ENV{INSTALL_DIR})
    message(FATAL_ERROR "Not Defined INSTALL_DIR")
endif()

if (NOT DEFINED ENV{THIRDPART_PATH})
    message(FATAL_ERROR "Not Defined THIRDPART_PATH")
endif()

if (NOT DEFINED ENV{CPU_ARCH})
    message(FATAL_ERROR "Not Defined CPU_ARCH")
endif()


list(APPEND COMMON_DEPEND_LIB avcodec avformat avdevice avutil swresample avfilter swscale)
file(GLOB_RECURSE COMMON_SRC_ALL "common/src/*.cpp")

if($ENV{CPU_ARCH} MATCHES "aarch64")
    message("arm architecture detected")
    if(EXISTS "$ENV{INSTALL_DIR}/driver/libmedia_mini.so")
        message("target aarch64 200dk")
	set(COMMON_SRC "${COMMON_SRC_ALL}")
        list(APPEND COMMON_DEPEND_LIB media_mini ascend_hal c_sec mmpa slog)
        add_definitions(-DENABLE_BOARD_CAMARE)
    else()
        message("target x86 300")
    endif()
else()
    message("x86 architecture detected")
    message("target x86 300")
    string(REGEX REPLACE "common/src/camera.cpp" "" COMMON_SRC "${COMMON_SRC_ALL}")
endif()

# Header path
include_directories(
    ../inc/
    $ENV{THIRDPART_PATH}/include/
    $ENV{INSTALL_DIR}/runtime/include/
    $ENV{INSTALL_DIR}/driver
)

# add host lib path
link_directories(
    $ENV{THIRDPART_PATH}/lib/
    $ENV{INSTALL_DIR}/runtime/lib64/stub
    $ENV{INSTALL_DIR}/driver/
)

add_executable(main
	${COMMON_SRC}
        utils.cpp
        video_encoder.cpp
        main.cpp)


target_link_libraries(main ascendcl acl_dvpp stdc++ acllite pthread ${COMMON_DEPEND_LIB} dl rt)


install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
