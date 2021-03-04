# Copyright (c) Huawei Technologies Co., Ltd. 2020. All rights reserved.
cmake_minimum_required(VERSION 3.5.0)
project(inference)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INSTALL_PREFIX "../")
set(CMAKE_OUTPUT_DIR "./")

if (NOT DEFINED ASCEND_HOME)
    if (NOT "x$ENV{ASCEND_HOME}" STREQUAL "x")
        set(ASCEND_HOME $ENV{ASCEND_HOME})
    else ()
        set(ASCEND_HOME /usr/local/Ascend/)
    endif ()
endif ()
message(STATUS "ASCEND_HOME=${ASCEND_HOME}")

# Header path
include_directories(
    ${ASCEND_HOME}/acllib/include/ 
    ../../models/jsoncpp/dist 
    ./data_process
                    )

add_subdirectory(data_process)

link_directories(${ASCEND_HOME}/acllib/lib64 ./data_process)
add_executable(inference main.cpp acl_engine.cpp)
target_link_libraries(inference ascendcl dataprocess)
