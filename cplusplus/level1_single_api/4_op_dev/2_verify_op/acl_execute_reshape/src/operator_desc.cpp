/**
* Copyright 2020 Huawei Technologies Co., Ltd
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#include "common.h"
#include "operator_desc.h"

using namespace std;

OperatorDesc::OperatorDesc(std::string opType) : opType(std::move(opType))
{
    opAttr = aclopCreateAttr();
}

OperatorDesc::~OperatorDesc()
{
    for (auto *desc : inputDesc) {
        aclDestroyTensorDesc(desc);
    }

    for (auto *desc : outputDesc) {
        aclDestroyTensorDesc(desc);
    }

    aclopDestroyAttr(opAttr);
}

OperatorDesc &OperatorDesc::AddInputTensorDesc(aclDataType dataType,
                                               int numDims,
                                               const int64_t *dims,
                                               aclFormat format)
{
    if (numDims > 0 && dims == nullptr) {
        ERROR_LOG("dims is nullptr while numDims > 0");
        return *this;
    }
    inputDesc.push_back(aclCreateTensorDesc(dataType, numDims, dims, format));
    return *this;
}

OperatorDesc &OperatorDesc::AddOutputTensorDesc(aclDataType dataType,
                                                int numDims,
                                                const int64_t *dims,
                                                aclFormat format)
{
    if (numDims > 0 && dims == nullptr) {
        ERROR_LOG("dims is nullptr while numDims > 0");
        return *this;
    }

    outputDesc.push_back(aclCreateTensorDesc(dataType, numDims, dims, format));
    return *this;
}
