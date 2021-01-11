/**
* @file operator_desc.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
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
	if (numDims < 0) {
		ERROR_LOG("numDims < 0, numDims is %d", numDims);
        return *this;
	}
    if (numDims > 0 && dims == nullptr) {
        ERROR_LOG("dims is nullptr while numDims > 0, numDims is %d", numDims);
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
	if (numDims < 0) {
		ERROR_LOG("numDims < 0, numDims is %d", numDims);
        return *this;
	}
    if (numDims > 0 && dims == nullptr) {
        ERROR_LOG("dims is nullptr while numDims > 0, numDims is %d", numDims);
        return *this;
    }

    outputDesc.push_back(aclCreateTensorDesc(dataType, numDims, dims, format));
    return *this;
}
