# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.
aux_source_directory(. SRCS)
message(STATUS "SRCS = ${SRCS}")

if("x${SRCS}" STREQUAL "x")
	add_custom_target(${ONNX_PLUGIN_TARGET}
		COMMAND mkdir -p ${ONNX_PLUGIN_TARGET_OUT_DIR}
		COMMAND echo "no source to make lib${ONNX_PLUGIN_TARGET}.so")
    return(0)
endif()

set(LIBRARY_OUTPUT_PATH ${ONNX_PLUGIN_TARGET_OUT_DIR})

add_library(${ONNX_PLUGIN_TARGET} SHARED ${SRCS})

target_compile_definitions(${ONNX_PLUGIN_TARGET} PRIVATE
    google=ascend_private
)

target_link_libraries(${ONNX_PLUGIN_TARGET} ${ASCEND_INC}/../lib64/libgraph.so)

target_include_directories(${ONNX_PLUGIN_TARGET} PRIVATE
    ${ASCEND_INC}/../../opp/built-in/op_proto/inc/
)
