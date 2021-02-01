# Copyright (c) Huawei Technologies Co., Ltd. 2020. All rights reserved.
cmake_minimum_required(VERSION 3.5.1)
project(SceneTextRecognition)

set(PROJECT_SRC_ROOT ${CMAKE_CURRENT_LIST_DIR}/)
# Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SRC_ROOT}/dist)
# Skip build rpath
set(CMAKE_SKIP_BUILD_RPATH True)
# Compile options
add_compile_options(-std=c++11 -fPIE -fstack-protector-all -Werror -Wall)
# Add definitions ENABLE_DVPP_INTERFACE and ASCEND_MODULE_USE_ACL to use dvpp acl api
add_definitions(-DENABLE_DVPP_INTERFACE -DASCEND_MODULE_USE_ACL)

# Check environment variable
if(NOT DEFINED ENV{ASCEND_HOME})
    message(FATAL_ERROR "Please define environment variable:ASCEND_HOME")
endif()

# Find ascendbase
set(ASCEND_BASE_DIR ${PROJECT_SRC_ROOT}/AscendBase/src/Base)
get_filename_component(ASCEND_BASE_ABS_DIR ${ASCEND_BASE_DIR} ABSOLUTE)
file(GLOB_RECURSE ASCEND_BASE_SRC_FILES ${ASCEND_BASE_ABS_DIR}/*.cpp)
# exclude the DvppCommonDevice.cpp which need to compile in the open-form
list(REMOVE_ITEM ASCEND_BASE_SRC_FILES ${ASCEND_BASE_ABS_DIR}/DvppCommonDevice/DvppCommonDevice.cpp)
list(REMOVE_ITEM ASCEND_BASE_SRC_FILES ${ASCEND_BASE_ABS_DIR}/Hdc/Hdc.cpp)

# Find src
set(COMMON_FOLDER ${PROJECT_SRC_ROOT}/Common)
set(MODULE_FOLDER ${PROJECT_SRC_ROOT}/Modules)
get_filename_component(COMMON_ABS_DIR ${COMMON_FOLDER} ABSOLUTE)
get_filename_component(MODULE_ABS_DIR ${MODULE_FOLDER} ABSOLUTE)
set (SRC_ABS_DIR ${COMMON_ABS_DIR} ${MODULE_ABS_DIR})
file(GLOB SRC_FILES
     ${MODULE_ABS_DIR}/ImageReader/*.cpp
     ${MODULE_ABS_DIR}/ImagePreprocess/*.cpp
     ${MODULE_ABS_DIR}/TextDetection/*.cpp
     ${MODULE_ABS_DIR}/DetectPost/*.cpp
     ${MODULE_ABS_DIR}/WarpPerspective/*.cpp
     ${MODULE_ABS_DIR}/TextRecognition/*.cpp
     ${MODULE_ABS_DIR}/ResultProcess/*.cpp
     ${PROJECT_SRC_ROOT}/main.cpp)

# set the source file for target
set(SOURCE_FILE ${ASCEND_BASE_SRC_FILES} ${SRC_FILES})

# set the share library directory
set(ACL_LIB_DIR $ENV{ASCEND_HOME}/$ENV{ASCEND_VERSION}/$ENV{ARCH_PATTERN}/lib64/stub)
set(OpenCVFolder /usr/local/opencv)

# set all share library directory as SHARED_LIB_DIR
set(SHARED_LIB_DIR
    ${ACL_LIB_DIR}
    ${OpenCVFolder}/lib
    ${OpenCVFolder}/lib64)

# set the header file directory
# set the header file directory of acllib as ACL_INC_DIR
set(ACL_INC_DIR $ENV{ASCEND_HOME}/$ENV{ASCEND_VERSION}/$ENV{ARCH_PATTERN}/include)
# set the APP header file as APP_INC_DIR
set(APP_INC_DIR ${SRC_ABS_DIR} ${PROJECT_SRC_ROOT})
# set all header file directory as HEADER_FILE_DIR
set(HEADER_FILE_DIR
    ${ACL_INC_DIR}
    ${ASCEND_BASE_DIR}
    ${ASCEND_BASE_DIR}/Framework
    ${ASCEND_BASE_DIR}/Framework/ModuleManager
    ${APP_INC_DIR}
    ${OpenCVFolder}/include/opencv4)

# Set the target executable file
link_directories(${SHARED_LIB_DIR})
add_executable(ocr ${SOURCE_FILE})
target_include_directories(ocr PUBLIC ${HEADER_FILE_DIR})
target_link_libraries(ocr ascendcl acl_dvpp pthread opencv_world -Wl,-z,relro,-z,now,-z,noexecstack -pie -s)