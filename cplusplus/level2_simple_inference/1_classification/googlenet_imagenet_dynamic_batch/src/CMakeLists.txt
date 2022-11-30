# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(DynamicBatch)

# Compile options
add_compile_options(-std=c++11)

# Specify target generation path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../../../out")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

if (NOT DEFINED ENV{INSTALL_DIR})
    message(FATAL_ERROR "Not Defined INSTALL_DIR")
endif()

# Header path
include_directories(
    $ENV{INSTALL_DIR}/runtime/include/
    ../inc/
)

# add host lib path
link_directories(
    $ENV{INSTALL_DIR}/runtime/lib64/stub
)

SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath-link,$ENV{INSTALL_DIR}/runtime/lib64/stub")

add_executable(main 
        main.cpp)

target_link_libraries(main ascendcl stdc++)

install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
