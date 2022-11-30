# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(YOLOV4_coco_detection_picture)

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
    message(FATAL_ERROR "Not Defined THIRDPART_PATH")
endif()

if (NOT DEFINED ENV{CPU_ARCH})
    message(FATAL_ERROR "Not Defined CPU_ARCH")
endif()

if($ENV{CPU_ARCH} MATCHES "aarch64")
    message("arm architecture detected")
    if(EXISTS "$ENV{INSTALL_DIR}/driver/libmedia_mini.so")
        message("target aarch64 200dk")
    else()
        message("target x86 300")
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

# add host lib path
link_directories(
    $ENV{INSTALL_DIR}/runtime/lib64/stub
    $ENV{THIRDPART_PATH}/lib/
)

add_executable(main
        utils.cpp
        model_process.cpp
        dvpp_process.cpp
        post_process.cpp
        main.cpp)

target_link_libraries(main ascendcl acl_dvpp opencv_core opencv_imgproc opencv_imgcodecs stdc++)

install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
