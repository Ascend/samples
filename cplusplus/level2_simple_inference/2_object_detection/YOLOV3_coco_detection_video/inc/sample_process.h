/**
* Copyright 2020 Huawei Technologies Co., Ltd
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* File sample_process.h
* Description: handle acl resource
*/
#pragma once
#include "utils.h"
#include "acl/acl.h"
#include <memory>

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

/**
* SampleProcess
*/
class SampleProcess {
public:
    /**
    * @brief Constructor
    */
    SampleProcess();

    /**
    * @brief Destructor
    */
    ~SampleProcess();

    /**
    * @brief init reousce
    * @return result
    */
    Result InitResource();
    
    /**
    * @brief encode sample process
    * @param [in] input_path: input video path
    * @return result
    */
    Result VideoProcess(std::string input_path);

private:
    void DestroyResource();

    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
};

