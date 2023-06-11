# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.


macro(GEN_OPS_INFO_FILE op_type output_dir output_file_name_prefix)

    SET(INI_PATH ${CMAKE_CURRENT_SOURCE_DIR}/op_info_cfg/${op_type})
    if(NOT EXISTS ${INI_PATH})
        set(INI_PATH ${CMAKE_CURRENT_SOURCE_DIR}/op_info_cfg.${op_type})
    endif()
    execute_process(COMMAND ls -1 ${INI_PATH}
            OUTPUT_VARIABLE SUB_DIRS)
    string(REPLACE "\n" ";" SUB_DIRS ${SUB_DIRS})
    foreach(SUB_DIR ${SUB_DIRS})
        if (IS_DIRECTORY ${INI_PATH}/${SUB_DIR})
            execute_process(COMMAND find ${INI_PATH}/${SUB_DIR} -name "*.ini"
                    OUTPUT_VARIABLE INI_FILES)
            string(REPLACE "\n" "\t" INI_FILES ${INI_FILES})
            set(output_file_name ${output_file_name_prefix}-${SUB_DIR}-ops-info.json)
            if (NOT "x${INI_FILES}" STREQUAL "x")
                add_custom_command(OUTPUT ${output_file_name}
                        COMMAND echo "python ${INI_2_JSON_PY} ${INI_FILES} ${output_dir}/${output_file_name}"
                        COMMAND mkdir -p ${output_dir}/${SUB_DIR}
                        COMMAND python ${INI_2_JSON_PY} ${INI_FILES} ${output_dir}/${SUB_DIR}/${output_file_name}
                        COMMENT "generate configure json file")
                set(OUTPUT_FILES ${OUTPUT_FILES} ${output_file_name})
            else()
                message(STATUS "ini file not exists.")
                execute_process(COMMAND mkdir -p ${output_dir})
            endif ()
        endif()
    endforeach()
endmacro(GEN_OPS_INFO_FILE)

set(OUTPUT_FILES "")

GEN_OPS_INFO_FILE(ai_core ${AIC_OP_INFO_CFG_OUT_DIR} aic)

message(STATUS "OUTPUT_FILES=${OUTPUT_FILES}")
if ("x${OUTPUT_FILES}" STREQUAL "x")
    add_custom_target(config_json
            COMMAND echo "tbe_aic_ops_info.ini and tbe_aiv_ops_info.ini not exist.")
    return(0)
endif ()

add_custom_target(config_json ALL DEPENDS ${OUTPUT_FILES}
        COMMAND mkdir -p ${TF_PLUGIN_TARGET_OUT_DIR}
	COMMAND bash ${CMAKE_SOURCE_DIR}/cmake/util/gen_ops_filter.sh ${INI_PATH} ${TF_PLUGIN_TARGET_OUT_DIR})

if (NOT "x$ENV{ADD_TESTCASE}" STREQUAL "x")
  # add testcases subdirectory
  if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/testcases/bbit")
    add_subdirectory(testcases/bbit)
  endif()
  if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/testcases/st")
    add_subdirectory(testcases/st)
  endif()
  if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/testcases/acl_op/src")
    add_subdirectory(testcases/acl_op/src)
  endif()
  file(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/testcases/st_cases/out/*")
  foreach(file ${files})
    message(STATUS ${file})
    if(IS_DIRECTORY ${file})
        add_subdirectory(${file})
    endif()
  endforeach()
endif()
