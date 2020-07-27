# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(cameradetect)

# Compile options
add_compile_options(-std=c++11)

add_definitions(-DENABLE_DVPP_INTERFACE)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../../../out")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

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

# Header path
include_directories(
    ./
    ./common/
    ${INC_PATH}/acllib/include/
    $ENV{HOME}/ascend_ddk/arm/include/
    $ENV{HOME}/Ascend/ascenddk/arm/include/
    $ENV{HOME}/ascend_ddk/arm/include/ascenddk/
    $ENV{HOME}/Ascend/
)

# add host lib path
link_directories(
    ${LIB_PATH}
    $ENV{HOME}/Ascend/driver
    ${INC_PATH}/atc/lib64/
    ${INC_PATH}/acllib/lib64
    $ENV{HOME}/ascend_ddk/arm/lib/
)

add_executable(main
        #video_dataset.cpp
        camera.cpp
        face_detection_inference.cpp
        face_detection_postprocess.cpp
        dvpp_jpege.cpp
        dvpp_process.cpp
        dvpp_resize.cpp
        model_process.cpp
        common/atlas_thread_mgr.cpp
        common/atlas_app.cpp
        common/atlas_utils.cpp
        common/atlas_utils_common.cpp
        main.cpp)

target_link_libraries(main ascendcl acl_dvpp runtime c_sec slog media_mini pthread stdc++  protobuf presenteragent)


install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})