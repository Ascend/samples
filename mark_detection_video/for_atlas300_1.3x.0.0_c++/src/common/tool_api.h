/**
 * ============================================================================
 *
 * Copyright (C) 2019, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */

#ifndef COMMON_TOOL_API_H_
#define COMMON_TOOL_API_H_

#include <memory>
#include "hiaiengine/data_type.h"
#include "hiaiengine/data_type_reg.h"
#include "hiaiengine/status.h"

// define result log, print to console
#define ERROR_LOG(fmt, args...) \
	do { \
		fprintf(stderr, "[ERROR][%s %d] " fmt "\n", __FILE__, __LINE__, ##args); \
		HIAI_ENGINE_LOG(HIAI_GRAPH_INVALID_VALUE, \
						"[%s:%d] " fmt "\n", __FILE__, __LINE__, ##args); \
	}while(0)

#define INFO_LOG(fmt, args...) \
	do {\
		fprintf(stdout, "[INFO][%s %d] " fmt "\n", __FILE__, __LINE__, ##args);\
		HIAI_ENGINE_LOG("[%s:%d] " fmt "\n", __FILE__, __LINE__, ##args);\
	}while(0)

// make_shared no throw function
template<class Type>
std::shared_ptr<Type> MakeSharedNoThrow() {
  try {
    return std::make_shared<Type>();
  } catch (...) {
    return nullptr;
  }
}

#define MAKE_SHARED_NO_THROW(memory, memory_type) \
    memory = MakeSharedNoThrow<memory_type>();

#endif /* COMMON_TOOL_API_H_ */
