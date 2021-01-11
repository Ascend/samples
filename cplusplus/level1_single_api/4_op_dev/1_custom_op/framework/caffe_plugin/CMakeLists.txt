# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

#message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
#execute_process(COMMAND sh ${CMAKE_SOURCE_DIR}/build.sh)

aux_source_directory(. SRCS)
aux_source_directory(./proto/caffe PROTO_SRCS)
list(APPEND SRCS ${PROTO_SRCS})

message(STATUS "SRCS = ${SRCS}")

if("x${SRCS}" STREQUAL "x")
    add_custom_target(${CAFFE_PLUGIN_TARGET}
            COMMAND mkdir -p ${CAFFE_PLUGIN_TARGET_OUT_DIR}
            COMMAND echo "no source to make lib${CAFFE_PLUGIN_TARGET}.so")
    return(0)
endif()

set(LIBRARY_OUTPUT_PATH ${CAFFE_PLUGIN_TARGET_OUT_DIR})

include_directories(./proto/caffe)
add_library(${CAFFE_PLUGIN_TARGET} SHARED ${SRCS})

target_compile_definitions(${CAFFE_PLUGIN_TARGET} PRIVATE
    google=ascend_private
)

target_link_libraries(${CAFFE_PLUGIN_TARGET} ${ASCEND_INC}/../lib64/libgraph.so)