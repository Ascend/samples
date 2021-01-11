
# set compile option -std=c++11
set(CMAKE_CXX_STANDARD 11)

# set compile option -fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(TOP_DIR ${CMAKE_SOURCE_DIR}/../..)

if (NOT DEFINED ASCEND_TENSOR_COMPLIER_INCLUDE)
    if (NOT "x$ENV{ASCEND_TENSOR_COMPLIER_INCLUDE}" STREQUAL "x")
        set(ASCEND_TENSOR_COMPLIER_INCLUDE $ENV{ASCEND_TENSOR_COMPLIER_INCLUDE})
    else ()
        set(ASCEND_TENSOR_COMPLIER_INCLUDE /usr/local/Ascend/atc/include)
    endif ()
endif ()
message(STATUS "ASCEND_TENSOR_COMPLIER_INCLUDE=${ASCEND_TENSOR_COMPLIER_INCLUDE}")

set(ASCEND_INC ${ASCEND_TENSOR_COMPLIER_INCLUDE})

if (UNIX)
    if (NOT DEFINED SYSTEM_INFO)
        if (NOT "x$ENV{SYSTEM_INFO}" STREQUAL "x")
            set(SYSTEM_INFO $ENV{SYSTEM_INFO})
        else ()
            execute_process(COMMAND grep -i ^id= /etc/os-release
                    OUTPUT_VARIABLE SYSTEM_NAME_INFO)
            string(REGEX REPLACE "\n|id=|ID=|\"" "" SYSTEM_NAME ${SYSTEM_NAME_INFO})
            message(STATUS "SYSTEM_NAME=${SYSTEM_NAME}")
            set(SYSTEM_INFO ${SYSTEM_NAME}_${CMAKE_SYSTEM_PROCESSOR})
        endif ()
    endif ()

    message(STATUS "SYSTEM_INFO=${SYSTEM_INFO}")
else ()
    message(FATAL_ERROR "${CMAKE_SYSTEM_NAME} not support.")
endif ()

set(RUN_TARGET "custom_opp_${SYSTEM_INFO}.run")
message( STATUS "RUN_TARGET=${RUN_TARGET}")

set(PROJECT_DIR custom)

set(OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/makepkg)
message(STATUS "OUT_DIR=${OUT_DIR}")

set(TF_PLUGIN_TARGET "cust_tf_parsers")
set(TF_PLUGIN_TARGET_OUT_DIR ${OUT_DIR}/packages/framework/${PROJECT_DIR}/tensorflow/)

set(TF_SCOPE_FUSION_PASS_TARGET "cust_tf_scope_fusion")
set(TF_SCOPE_FUSION_PASS_TARGET_OUT_DIR ${OUT_DIR}/packages/framework/${PROJECT_DIR}/tensorflow/)

set(CAFFE_PARSER_TARGET "_caffe_parser")
set(CAFFE_PLUGIN_TARGET "cust_caffe_parsers")
set(CAFFE_PLUGIN_TARGET_OUT_DIR ${OUT_DIR}/packages/framework/${PROJECT_DIR}/caffe/)

set(OP_PROTO_TARGET "cust_op_proto")
set(OP_PROTO_TARGET_OUT_DIR ${OUT_DIR}/packages/op_proto/${PROJECT_DIR}/)

set(AIC_FUSION_PASS_TARGET "cust_aic_fusion_pass")
set(AIC_FUSION_PASS_TARGET_OUT_DIR ${OUT_DIR}/packages/fusion_pass/${PROJECT_DIR}/ai_core)

set(AIV_FUSION_PASS_TARGET "cust_aiv_fusion_pass")
set(AIV_FUSION_PASS_TARGET_OUT_DIR ${OUT_DIR}/packages/fusion_pass/${PROJECT_DIR}/vector_core)

set(AIC_OP_INFO_CFG_OUT_DIR ${OUT_DIR}/packages/op_impl/${PROJECT_DIR}/ai_core/tbe/config)
set(AIV_OP_INFO_CFG_OUT_DIR ${OUT_DIR}/packages/op_impl/${PROJECT_DIR}/vector_core/tbe/config/)

set(AICPU_CONFIG_JSON_TARGET "aicpu_config_json")
set(AICPU_OP_INFO_CFG_OUT_DIR ${OUT_DIR}/packages/op_impl/${PROJECT_DIR}/cpu/config)
set(AICPU_OP_IMPL_OUT_DIR ${OUT_DIR}/packages/op_impl/${PROJECT_DIR}/cpu/aicpu_kernel/custom_impl/)

set(INI_2_JSON_PY "${CMAKE_SOURCE_DIR}/cmake/util/parse_ini_to_json.py")
set(AICPU_INI_2_JSON_PY "${CMAKE_SOURCE_DIR}/cmake/util/aicpu_parser_ini.py")

set(AICPU_KERNEL_TARGET $ENV{AICPU_KERNEL_TARGET})
if("x${AICPU_KERNEL_TARGET}" STREQUAL "x")
    set(AICPU_KERNEL_TARGET "cust_aicpu_kernels")
endif()

include_directories(${ASCEND_INC})
