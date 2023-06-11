# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(sampleYOLOV7MultiInput)

# Compile options
add_compile_options(-std=c++11)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../../../out")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

set(INC_PATH $ENV{DDK_PATH})
if (NOT DEFINED ENV{DDK_PATH})
    set(INC_PATH "/usr/local/Ascend/ascend-toolkit/latest")
    message(STATUS "set default INC_PATH: ${INC_PATH}")
else()
    message(STATUS "set INC_PATH: ${INC_PATH}")
endif ()

set(LIB_PATH $ENV{NPU_HOST_LIB})
if (NOT DEFINED ENV{NPU_HOST_LIB})
    set(LIB_PATH "/usr/local/Ascend/ascend-toolkit/latest/runtime/lib64/stub")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else()
    message(STATUS "set LIB_PATH: ${LIB_PATH}")
endif ()


add_definitions(-DENABLE_DVPP_INTERFACE)
list(APPEND COMMON_DEPEND_LIB avcodec avformat avutil swresample swscale)
aux_source_directory(${PROJECT_SOURCE_DIR}/../common/src aclLite)
list(REMOVE_ITEM aclLite ${PROJECT_SOURCE_DIR}/../common/src/CameraCapture.cpp)

# Header path
include_directories(
        /usr/include/
        ${INC_PATH}/runtime/include/
        ../inc/
        ../common/include/
)

if(target STREQUAL "Simulator_Function")
    add_compile_options(-DFUNC_SIM)
endif()

# add host lib path
link_directories(
        ${INC_PATH}/runtime/lib64/stub
        ${INC_PATH}/thirdpart/lib
)

add_executable(main
        dataInput/dataInput.cpp
        detectPreprocess/detectPreprocess.cpp
        detectInference/detectInference.cpp
        detectPostprocess/detectPostprocess.cpp
        dataOutput/dataOutput.cpp
	    pushrtsp/pictortsp.cpp
        pushrtsp/pushrtspthread.cpp
        main.cpp)

target_sources(main 
    PUBLIC
        ${aclLite})

if(target STREQUAL "Simulator_Function")
    target_link_libraries(main funcsim)
else()
    target_link_libraries(main ascendcl acl_dvpp stdc++ pthread ${COMMON_DEPEND_LIB} jsoncpp opencv_highgui opencv_core opencv_imgproc opencv_imgcodecs opencv_calib3d opencv_features2d opencv_videoio dl rt X11)
endif()

install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
