
# Copyright (c) Huawei Technologies Co., Ltd. 2021. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(vpc_demo)

# Compile options
add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

set(INC_PATH $ENV{DDK_PATH})
if (NOT DEFINED ENV{DDK_PATH})
    set(INC_PATH "/usr/local/Ascend")
    message(STATUS "set default INC_PATH: ${INC_PATH}")
else ()
    message(STATUS "env INC_PATH: ${INC_PATH}")
endif()

set(LIB_PATH $ENV{NPU_HOST_LIB})
# Dynamic libraries in the stub directory can only be used for compilation
if (NOT DEFINED ENV{NPU_HOST_LIB})
    set(LIB_PATH "/usr/local/Ascend/runtime/lib64/stub/")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else ()
    message(STATUS "env LIB_PATH: ${LIB_PATH}")
endif()

link_directories(
    ${LIB_PATH}
)
include_directories(
    ${INC_PATH}/runtime/include/acl/
    ${INC_PATH}/runtime/include/acl/dvpp/
    ./common
)

add_executable (vpc_demo
    ./sample_vpc.cpp
    ./common/sample_comm.cpp
    ./common/sample_comm_vpc_crop.cpp
    ./common/sample_comm_vpc_resize.cpp
    ./common/sample_comm_vpc_cropResize.cpp
    ./common/sample_comm_vpc_cropResizePaste.cpp
    ./common/sample_comm_vpc_cropResizeMakeBorder.cpp
    ./common/sample_comm_vpc_convertColor.cpp
    ./common/sample_comm_vpc_copyMakeBorder.cpp
    ./common/sample_comm_vpc_pyrDown.cpp
    ./common/sample_comm_vpc_calcHist.cpp
    ./common/sample_comm_vpc_equalizeHist.cpp
    ./common/sample_comm_vpc_batchCropResizePaste.cpp
    ./common/sample_comm_vpc_batchCropResizeMakeBorder.cpp
)

target_link_libraries(vpc_demo
    acl_dvpp_mpi
    ascendcl
    pthread
    stdc++
)