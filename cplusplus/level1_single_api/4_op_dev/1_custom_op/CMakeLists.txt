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
    add_subdirectory(cpukernel)
endif()
if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/framework")
    add_subdirectory(framework)
endif()
if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/op_proto")
    add_subdirectory(op_proto)
endif()
if(EXISTS "${CMAKE_SOURCE_DIR}/tbe")
    add_subdirectory(tbe)
endif()

message(STATUS "operation system is ${CMAKE_HOST_SYSTEM_NAME}")

set(ALL_MODULES ${OP_PROTO_TARGET})
if(EXISTS "${CMAKE_SOURCE_DIR}/cpukernel/CMakeLists.txt")
    set(ALL_MODULES ${ALL_MODULES} ${AICPU_KERNEL_TARGET})
    set(ALL_MODULES ${ALL_MODULES} ${AICPU_CONFIG_JSON_TARGET})
endif()
message(STATUS "AICPU_CONFIG_INFO_FLAG=${AICPU_CONFIG_INFO_FLAG}")

if(EXISTS "${CMAKE_SOURCE_DIR}/tbe/CMakeLists.txt")
    set(ALL_MODULES ${ALL_MODULES} config_json)
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

message(STATUS "ALL_MODULES=${ALL_MODULES}")
add_custom_target(${RUN_TARGET} ALL DEPENDS ${ALL_MODULES})

add_custom_command(TARGET ${RUN_TARGET}
        PRE_BUILD
        COMMAND mkdir -p ./makepkg/packages/fusion_rules/${PROJECT_DIR}/
        COMMAND mkdir -p ./makepkg/packages/op_impl/${PROJECT_DIR}/ai_core/tbe/custom_impl
        COMMAND mkdir -p ./makepkg/packages/op_impl/${PROJECT_DIR}/vector_core/tbe/custom_impl
        COMMAND mkdir -p ./makepkg/packages/op_impl/${PROJECT_DIR}/cpu/aicpu_kernel/custom_impl
        COMMAND bash ${CMAKE_SOURCE_DIR}/cmake/util/gen_impl_and_mrege_json.sh ${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR}
        COMMAND cp ${CMAKE_SOURCE_DIR}/scripts/install.sh ./makepkg/
        COMMAND cp ${CMAKE_SOURCE_DIR}/scripts/upgrade.sh ./makepkg/
        COMMAND cp ${CMAKE_SOURCE_DIR}/scripts/uninstall.sh ./makepkg/
        COMMAND cp ${CMAKE_SOURCE_DIR}/scripts/help.sh ./makepkg/
        )

if(EXISTS ${CMAKE_SOURCE_DIR}/custom.proto)
    add_custom_command(TARGET ${RUN_TARGET}
        PRE_BUILD
        COMMAND cp ${CMAKE_SOURCE_DIR}/custom.proto ./makepkg/packages
        )
endif()

add_custom_command(TARGET ${RUN_TARGET}
        PRE_BUILD
        COMMAND chmod u+x ./makepkg/*.sh
        COMMAND ${CMAKE_SOURCE_DIR}/cmake/util/makeself/makeself.sh --gzip --complevel 4 --nomd5 --sha256 ./makepkg ${RUN_TARGET} "version:1.0" ./install.sh
        )

