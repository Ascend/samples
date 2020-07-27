/*
 * tool_api.h
 *
 *  Created on: Mar 14, 2019
 *      Author: ascend
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
