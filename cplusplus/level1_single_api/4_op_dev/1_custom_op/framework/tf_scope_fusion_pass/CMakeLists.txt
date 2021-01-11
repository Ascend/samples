# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

aux_source_directory(. SRCS)

if("x${SRCS}" STREQUAL "x")
    add_custom_target(${TF_SCOPE_FUSION_PASS_TARGET}
            COMMAND mkdir -p ${TF_SCOPE_FUSION_PASS_TARGET_OUT_DIR}
            COMMAND echo "no source to make lib${TF_SCOPE_FUSION_PASS_TARGET}.so")
    return(0)
endif()

set(LIBRARY_OUTPUT_PATH ${TF_SCOPE_FUSION_PASS_TARGET_OUT_DIR})

add_library(${TF_SCOPE_FUSION_PASS_TARGET} SHARED ${SRCS})