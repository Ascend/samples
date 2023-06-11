# Copyright (c) Huawei Technologies Co., Ltd. 2020. All rights reserved.

cmake_minimum_required(VERSION 3.5.1)
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
message(STATUS "CMAKE_CURRENT_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}")
message(STATUS "CMAKE_CURRENT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}")

project(opp)

if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/cmake/config.cmake)
    include(${CMAKE_CURRENT_BINARY_DIR}/cmake/config.cmake)
else()
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake)
        include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake)
    endif()
endif()

execute_process(COMMAND rm -rf ./makepkg)

message(STATUS "OP_COMMON_INC=${OP_COMMON_INC}")
# add sub compile directory
if(EXISTS "${CMAKE_SOURCE_DIR}/cpukernel")
    if (NOT "x$ENV{COMPILE_KERNEL_TARGET}" STREQUAL "xtbe")
        message(STATUS "compile kernel target is $ENV{COMPILE_KERNEL_TARGET}")
        add_subdirectory(cpukernel)
    endif()
endif()
if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/framework")
    add_subdirectory(framework)
endif()
if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/op_proto")
    add_subdirectory(op_proto)
endif()
if(EXISTS "${CMAKE_SOURCE_DIR}/tbe")
    if (NOT "x$ENV{COMPILE_KERNEL_TARGET}" STREQUAL "xcpu")
        message(STATUS "compile kernel target is $ENV{COMPILE_KERNEL_TARGET}")
        add_subdirectory(tbe)
    endif()
endif()

if (NOT "x$ENV{ADD_TESTCASE}" STREQUAL "x")
  # add testcases subdirectory
  if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/testcases")
    add_subdirectory(testcases)
  endif()
endif()

message(STATUS "operation system is ${CMAKE_HOST_SYSTEM_NAME}")

set(ALL_MODULES ${OP_PROTO_TARGET})
if(EXISTS "${CMAKE_SOURCE_DIR}/cpukernel/CMakeLists.txt")
    if (NOT "x$ENV{COMPILE_KERNEL_TARGET}" STREQUAL "xtbe")
        message(STATUS "kernel target and config json: $ENV{COMPILE_KERNEL_TARGET}")
        set(ALL_MODULES ${ALL_MODULES} ${AICPU_KERNEL_TARGET})
        set(ALL_MODULES ${ALL_MODULES} ${AICPU_CONFIG_JSON_TARGET})
    endif()
endif()
message(STATUS "AICPU_CONFIG_INFO_FLAG=${AICPU_CONFIG_INFO_FLAG}")

if(EXISTS "${CMAKE_SOURCE_DIR}/tbe/CMakeLists.txt")
    if (NOT "x$ENV{COMPILE_KERNEL_TARGET}" STREQUAL "xcpu")
        message(STATUS "kernel target and config json: $ENV{COMPILE_KERNEL_TARGET}")
        set(ALL_MODULES ${ALL_MODULES} config_json)
    endif()
endif()

if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/framework/caffe_plugin")
    set(ALL_MODULES ${ALL_MODULES} ${CAFFE_PLUGIN_TARGET})
endif()
if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/framework/tf_plugin")
    set(ALL_MODULES ${ALL_MODULES} ${TF_PLUGIN_TARGET})
endif()
if (IS_DIRECTORY "${CMAKE_SOURCE_DIR}/framework/tf_scope_fusion_pass")
    set(ALL_MODULES ${ALL_MODULES} ${TF_SCOPE_FUSION_PASS_TARGET})
endif ()
if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/framework/onnx_plugin")
   set(ALL_MODULES ${ALL_MODULES} ${ONNX_PLUGIN_TARGET})
endif()

message(STATUS "ALL_MODULES=${ALL_MODULES}")
add_custom_target(${RUN_TARGET} ALL DEPENDS ${ALL_MODULES})

if (NOT "x$ENV{COMPILE_KERNEL_TARGET}" STREQUAL "xcpu")
    add_custom_command(TARGET ${RUN_TARGET}
        PRE_BUILD
        COMMAND mkdir -p ./makepkg/packages/${PROJECT_DIR}/op_impl/ai_core/tbe/${OPP_CUSTOM_VENDOR}_impl
        COMMAND mkdir -p ./makepkg/packages/${PROJECT_DIR}/op_impl/vector_core/tbe/${OPP_CUSTOM_VENDOR}_impl
        )
endif()

if (NOT "x$ENV{COMPILE_KERNEL_TARGET}" STREQUAL "xtbe")
    add_custom_command(TARGET ${RUN_TARGET}
        PRE_BUILD
        COMMAND mkdir -p ./makepkg/packages/${PROJECT_DIR}/op_impl/cpu/aicpu_kernel/impl
        )
endif()

add_custom_command(TARGET ${RUN_TARGET}
        PRE_BUILD
        COMMAND mkdir -p ./makepkg/packages/${PROJECT_DIR}/fusion_rules/
        COMMAND bash ${CMAKE_SOURCE_DIR}/cmake/util/gen_impl_and_mrege_json.sh ${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR}
        COMMAND cp ${CMAKE_SOURCE_DIR}/scripts/install.sh ./makepkg/
        COMMAND cp ${CMAKE_SOURCE_DIR}/scripts/upgrade.sh ./makepkg/
        COMMAND cp ${CMAKE_SOURCE_DIR}/scripts/uninstall.sh ./makepkg/
        COMMAND cp ${CMAKE_SOURCE_DIR}/scripts/help.sh ./makepkg/
        COMMAND cp ${CMAKE_SOURCE_DIR}/scripts/set_env.bash ./makepkg/
        )

if(EXISTS ${CMAKE_SOURCE_DIR}/custom.proto)
    add_custom_command(TARGET ${RUN_TARGET}
        PRE_BUILD
        COMMAND cp ${CMAKE_SOURCE_DIR}/custom.proto ./makepkg/packages/${PROJECT_DIR}
        )
endif()

add_custom_command(TARGET ${RUN_TARGET}
        PRE_BUILD
        COMMAND chmod u+x ./makepkg/*.sh
        COMMAND chmod u+x ./makepkg/*.bash
        COMMAND ${CMAKE_SOURCE_DIR}/cmake/util/makeself/makeself.sh --gzip --complevel 4 --nomd5 --sha256 ./makepkg ${RUN_TARGET} "version:1.0" ./install.sh
        )

