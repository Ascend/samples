# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(dynamicBatch)

# Compile options
add_compile_options(-std=c++11)

# Specify target generation path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../../../out")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

# Header path
include_directories(
$ENV{DDK_PATH}/acllib/include/
    ../inc/
)

if(target STREQUAL "Simulator_Function")
    add_compile_options(-DFUNC_SIM)
endif()

# add host lib path
link_directories($ENV{NPU_HOST_LIB})

SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath-link,$ENV{NPU_HOST_LIB}")

add_executable(main main.cpp)

if(target STREQUAL "Simulator_Function")
    target_link_libraries(main funcsim)
else()
    target_link_libraries(main ascendcl stdc++)
endif()

install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
