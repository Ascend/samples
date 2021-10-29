
# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(venc_demo)

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
)

ADD_DEFINITIONS(-DHMEV_PLATFORM_SDK -DIS_USER_SPACE -DVTOP_OS_TYPE=2 -DVTOP_LIBC_TYPE=1 -DVTOP_LITTLE_BYTE_ORDER -DVTOP_OS_SUPPORT_POSIX -DVTOP_OS_TYPE_ANDROID -DOS_ANDROID -DTSP_UPG_SYS_TYPE_BIG -DWSEC_COMPILE_CAC_OPENSSL -DWSEC_LINUX -DWSEC_ERR_CODE_BASE=0 -DWSEC_COMPILE_SDP)

add_executable (venc_demo
    ./src/sample_encoder.cpp
    ./src/sample_encoder_manage.cpp
    ./src/sample_debug.cpp
    ./src/sample_looper.cpp
)

target_link_libraries(venc_demo
    acl_dvpp_mpi
    ascendcl
    pthread
    stdc++
)
