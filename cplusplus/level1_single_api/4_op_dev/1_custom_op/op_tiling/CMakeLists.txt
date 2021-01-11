# Copyright 2020 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================

# the list of source files
set(_optiling_files
  ../fusion_pass/common/fp16_t.cc
  assign.cc
)

set(_op_tiling_include
  ${OPS_INCLUDE}
  ${OPS_DIR}/built-in
  ${FWKACLLIB_INCLUDE}
  ${METADEF_DIR}
  ${METADEF_INCLUDE}
  ${METADEF_INCLUDE}/external
  ${METADEF_INCLUDE}/external/graph
  ${GRAPHENGINE_INCLUDE}
  ${GRAPHENGINE_INCLUDE}/external
  ${Protobuf_INCLUDE}
)

set(_op_tiling_link_libs
  -Wl,--no-as-needed
    graph
    register
    slog
    error_manager
    ascend_protobuf
  -Wl,--as-needed
    c_sec
    json
)

if(ENABLE_TEST STREQUAL "")
  add_library(optiling SHARED ${_optiling_files})
  target_include_directories(optiling PRIVATE
    ${_op_tiling_include}
  )
  target_compile_options(optiling PRIVATE
    -O2
    -Dgoogle=ascend_private
    # TODO: Fix warnings
    # -Werror
  )
  target_link_libraries(optiling PRIVATE
    $<BUILD_INTERFACE:intf_pub>
    ${_op_tiling_link_libs}
  )
  # to solve the compile warnings in fp16_t.cc
  target_compile_options(optiling PRIVATE
    "-fno-strict-aliasing"
  )

  if(BUILD_OPEN_PROJECT)
    set(OP_TILING_PATH "${INSTALL_PATH}/opp/op_impl/built-in/ai_core/tbe/op_tiling")
    cann_install(
      TARGET      optiling
      FILES       $<TARGET_FILE:optiling>
      DESTINATION "${OP_TILING_PATH}"
    )
  else()
    install(
      TARGETS optiling OPTIONAL
      EXPORT  optiling-targets
      LIBRARY DESTINATION ${INSTALL_LIBRARY_DIR}
    )
  endif()
else()
  add_library(optiling_llt STATIC ${_optiling_files})
  target_include_directories(optiling_llt PUBLIC
    ${_op_tiling_include}
  )
  target_compile_options(optiling_llt PUBLIC
    -O0
    -Dgoogle=ascend_private
  )
  target_link_libraries(optiling_llt
    PRIVATE
      $<BUILD_INTERFACE:intf_llt_pub>
    PUBLIC
      ${_op_tiling_link_libs}
  )
endif()
