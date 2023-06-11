# Copyright (c) Huawei Technologies Co., Ltd. 2020. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(acl_op)

# Compile options
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall -s")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CAMKE_SKIP_RPATH TRUE)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,relro,-z,now,-z,noexecstack -pie")
add_compile_options(
    -fstack-protector-all
    -D_FORTIFY_SOURCE=2 -O2
    )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../../../run/out")
message(${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

set(INC_PATH $ENV{DDK_PATH})

if (NOT DEFINED ENV{DDK_PATH})
    set(INC_PATH "/usr/local/Ascend")
    message(STATUS "set default INC_PATH: ${INC_PATH}")
else ()
    message(STATUS "env INC_PATH: ${INC_PATH}")
endif()

if (EXISTS ${INC_PATH}/runtime)
    set(ACL_NAME "runtime")
else ()
    set(ACL_NAME "acllib")
endif()

set(LIB_PATH $ENV{NPU_HOST_LIB})

if (NOT DEFINED ENV{NPU_HOST_LIB})
    set(LIB_PATH "/usr/local/Ascend/${ACL_NAME}/lib64/stub/")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else ()
    message(STATUS "env LIB_PATH: ${LIB_PATH}")
endif()

if ("${LIB_PATH}" MATCHES "acllib")
    set(ACL_NAME "acllib")
endif()
message(STATUS "set acllib package name: ${ACL_NAME}")

# Header path
include_directories(
    ${INC_PATH}/${ACL_NAME}/include/
    $ENV{THIRDPART_PATH}/include/
    $ENV{INSTALL_DIR}/runtime/include/
    ../inc
)

# add host lib path
link_directories(
    $ENV{THIRDPART_PATH}/lib/
    $ENV{INSTALL_DIR}/runtime/lib64/stub
    ${LIB_PATH}
)

add_executable(main
        op_runner.cpp
        main.cpp
        common.cpp
        op_execute.cpp
        op_test.cpp
        op_test_desc.cpp
        testcase.cpp)

target_link_libraries(main
        ascendcl
        stdc++)

install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# custom command process om conversion
message(STATUS "SOC_VERSION=${SOC_VERSION}")
set(ASCEND_HOME $ENV{ADK_PATH})
