# CMake minimum version number requirements
cmake_minimum_required(VERSION 3.5)
project(kernel)
set(TOOLCHAIN_DIR $ENV{TOOLCHAIN_DIR})

#Specify cross compiler
if (LHISI)
        add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)
        set(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}-g++)
        set(CMAKE_C_COMPILER   ${TOOLCHAIN_DIR}-gcc)
else()
    if (NOT "x${TOOLCHAIN_DIR}" STREQUAL "x")
        if (NOT IS_DIRECTORY ${TOOLCHAIN_DIR})
            message(FATAL_ERROR "specify cross compile toolchain directory(${TOOLCHAIN_DIR}) is not exist")
        endif()
    endif()
    message(STATUS "TOOLCHAIN_DIR=${TOOLCHAIN_DIR}")
    if (MINRC)
        add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
        set(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}/bin/aarch64-linux-gnu-gcc)
        set(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}/bin/aarch64-linux-gnu-g++)
    else()
        add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)
        set(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}/bin/aarch64-target-linux-gnu-g++)
        set(CMAKE_C_COMPILER   ${TOOLCHAIN_DIR}/bin/aarch64-target-linux-gnu-gcc)
    endif()
endif()
# =========================config json compile============================
macro(GEN_OPS_INFO_FILE op_type output_dir output_file_name_prefix)

    SET(INI_PATH ${CMAKE_CURRENT_SOURCE_DIR}/op_info_cfg)
    message(STATUS "ini file directory: ${CMAKE_CURRENT_SOURCE_DIR}/op_info_cfg")
    if(NOT EXISTS ${INI_PATH})
        set(INI_PATH ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
    execute_process(COMMAND ls -1 ${INI_PATH}
            OUTPUT_VARIABLE SUB_DIRS)
    string(REPLACE "\n" ";" SUB_DIRS ${SUB_DIRS})
    foreach(SUB_DIR ${SUB_DIRS})
        if (IS_DIRECTORY ${INI_PATH}/${SUB_DIR})
            execute_process(COMMAND find ${INI_PATH}/${SUB_DIR} -name "*.ini"
                    OUTPUT_VARIABLE INI_FILES)
            if (NOT "x${INI_FILES}" STREQUAL "x")
                string(REPLACE "\n" "\t" INI_FILES ${INI_FILES})
            endif()
            #string(REPLACE "\n" "\t" INI_FILES ${INI_FILES})
            set(output_file_name ${output_file_name_prefix}_aicpu_kernel.json)
            if (NOT "x${INI_FILES}" STREQUAL "x")
                add_custom_command(OUTPUT ${output_file_name}
                        COMMAND echo "python3.7.5 ${AICPU_INI_2_JSON_PY} ${INI_FILES} ${output_dir}/${output_file_name}"
                        COMMAND mkdir -p ${output_dir}
                        COMMAND python3.7.5 ${AICPU_INI_2_JSON_PY} ${INI_FILES} ${output_dir}/${output_file_name}
                        COMMENT "generate configure json file")
                set(OUTPUT_FILES ${OUTPUT_FILES} ${output_file_name})
            else()
                message(STATUS "ini file not exists.")
                execute_process(COMMAND mkdir -p ${output_dir})
            endif ()
        endif()
    endforeach()
endmacro(GEN_OPS_INFO_FILE)

if (NOT IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/op_info_cfg/aicpu_kernel)
    add_custom_target(${AICPU_CONFIG_JSON_TARGET}
            COMMAND echo "can not find directory ${CMAKE_CURRENT_SOURCE_DIR}/op_info_cfg/aicpu_kernel")
    return(0)
endif()

set(OUTPUT_FILES "")


GEN_OPS_INFO_FILE("" ${AICPU_OP_INFO_CFG_OUT_DIR} cust)

if ("x${OUTPUT_FILES}" STREQUAL "x")
    add_custom_target(${AICPU_CONFIG_JSON_TARGET}
            COMMAND echo "can not find ini file in ${CMAKE_CURRENT_SOURCE_DIR}/aicpu.")
    return(0)
endif()

add_custom_target(${AICPU_CONFIG_JSON_TARGET} ALL DEPENDS ${OUTPUT_FILES}
    COMMAND mkdir -p ${TF_PLUGIN_TARGET_OUT_DIR}
    COMMAND bash ${CMAKE_SOURCE_DIR}/cmake/util/merge_aicpu_info_json.sh ${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR})

# =========================kernel compile============================
macro(SUBDIRLIST current_dir)
    file(GLOB children RELATIVE ${current_dir} ${current_dir}/*)
    set(SUB_DIR_LIST "")
    foreach(child ${children})
        if(IS_DIRECTORY ${current_dir}/${child})
            LIST(APPEND SUB_DIR_LIST ${child})
        endif()
    endforeach()
endmacro(SUBDIRLIST)

# set compile option -std=c++11
set(CMAKE_CXX_STANDARD 11)

# set compile option -fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

aux_source_directory(./impl/ KERNELS_SRCS)

if("x${KERNELS_SRCS}" STREQUAL "x")
    add_custom_target(${AICPU_KERNEL_TARGET}
        COMMAND echo "no source to make lib${AICPU_KERNEL_TARGET}.so")
    return(0)
endif()

set(ASCEND_OPP_PATH $ENV{ASCEND_OPP_PATH})
if("x${ASCEND_OPP_PATH}" STREQUAL "x")
        message(FATAL_ERROR "ENV ASCEND_OPP_PATH is not set")
endif()

set(ASCEND_AICPU_PATH $ENV{ASCEND_AICPU_PATH})
if("x${ASCEND_AICPU_PATH}" STREQUAL "x")
        message(FATAL_ERROR "ENV ASCEND_AICPU_PATH is not set")
endif()

include_directories(${ASCEND_OPP_PATH}/op_impl/built-in/aicpu/aicpu_kernel/inc)

# travers subdirectory
SUBDIRLIST(${CMAKE_CURRENT_SOURCE_DIR}/impl/third_party)
include_directories(./impl/third_party/${SUB_DIR_LIST})

set(LIBRARY_OUTPUT_PATH ${AICPU_OP_IMPL_OUT_DIR})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
add_library(${AICPU_KERNEL_TARGET} SHARED ${KERNELS_SRCS})

set(AICPU_SOC_VERSION $ENV{AICPU_SOC_VERSION})
message(STATUS "AICPU_SOC_VERSION=${AICPU_SOC_VERSION}")

if(EXISTS "${ASCEND_AICPU_PATH}/opp/op_impl/built-in/aicpu/aicpu_kernel/lib/${AICPU_SOC_VERSION}/libcpu_kernels_context.a")
    target_link_libraries(${AICPU_KERNEL_TARGET} PRIVATE
        -Wl,--whole-archive
        ${ASCEND_AICPU_PATH}/opp/op_impl/built-in/aicpu/aicpu_kernel/lib/${AICPU_SOC_VERSION}/libcpu_kernels_context.a
        -Wl,--no-whole-archive
    )
else()
    if(EXISTS "${ASCEND_AICPU_PATH}/opp/op_impl/built-in/aicpu/aicpu_kernel/lib/libcpu_kernels_context.a")
        target_link_libraries(${AICPU_KERNEL_TARGET} PRIVATE
            -Wl,--whole-archive
            ${ASCEND_AICPU_PATH}/opp/op_impl/built-in/aicpu/aicpu_kernel/lib/libcpu_kernels_context.a
            -Wl,--no-whole-archive
        )
    elseif(EXISTS "${ASCEND_OPP_PATH}/op_impl/built-in/aicpu/aicpu_kernel/lib/device/libcpu_kernels_context.so")
        target_link_libraries(${AICPU_KERNEL_TARGET} PRIVATE
            -Wl,--whole-archive
            ${ASCEND_OPP_PATH}/op_impl/built-in/aicpu/aicpu_kernel/lib/device/libcpu_kernels_context.so
            -Wl,--no-whole-archive
        )
    endif()
endif()

