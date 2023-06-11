/**
* @file op_test_desc.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "common.h"
#include "op_test_desc.h"

using namespace std;

OpTestDesc::OpTestDesc(std::string opType) : opType(std::move(opType))
{
    opAttr = aclopCreateAttr();
}

OpTestDesc::~OpTestDesc()
{
    for (auto *desc : inputDesc) {
        aclDestroyTensorDesc(desc);
    }

    for (auto *desc : outputDesc) {
        aclDestroyTensorDesc(desc);
    }

    aclopDestroyAttr(opAttr);
}

OpTestDesc &OpTestDesc::AddInputTensorDesc(aclDataType dataType,
    int numDims, const int64_t *dims, aclFormat format)
{
    if (numDims > 0 && dims == nullptr) {
        ERROR_LOG("Dims is nullptr while numDims > 0.");
        return *this;
    }
    if (format == ACL_FORMAT_UNDEFINED) {
        inputDesc.push_back(aclCreateTensorDesc(ACL_DT_UNDEFINED, 0, nullptr, ACL_FORMAT_UNDEFINED));
        return *this;
    }
    inputDesc.push_back(aclCreateTensorDesc(dataType, numDims, dims, format));
    return *this;
}

OpTestDesc &OpTestDesc::AddOutputTensorDesc(aclDataType dataType,
    int numDims, const int64_t *dims, aclFormat format)
{
    if (numDims > 0 && dims == nullptr) {
        ERROR_LOG("Dims is nullptr while numDims > 0.");
        return *this;
    }

    outputDesc.push_back(aclCreateTensorDesc(dataType, numDims, dims, format));
    return *this;
}

bool OpTestDesc::AddTensorAttr(const OpTestAttr &attr)
{
    if (opAttr == nullptr) {
        return false;
    }
    switch (attr.type) {
        case OP_BOOL:
            aclopSetAttrBool(opAttr, attr.name.c_str(), attr.boolAttr);
            break;
        case OP_INT:
            aclopSetAttrInt(opAttr, attr.name.c_str(), attr.intAttr);
            break;
        case OP_FLOAT:
            aclopSetAttrFloat(opAttr, attr.name.c_str(), attr.floatAttr);
            break;
        case OP_STRING:
            aclopSetAttrString(opAttr, attr.name.c_str(), attr.stringAttr.c_str());
            break;
        case OP_LIST_BOOL:
            aclopSetAttrListBool(opAttr, attr.name.c_str(), attr.listBoolAttr.size(),
                attr.listBoolAttr.data());
            break;
        case OP_LIST_INT:
            aclopSetAttrListInt(opAttr, attr.name.c_str(), attr.listIntAttr.size(),
                attr.listIntAttr.data());
            break;
        case OP_LIST_FLOAT:
            aclopSetAttrListFloat(opAttr, attr.name.c_str(), attr.listFloatAttr.size(),
                attr.listFloatAttr.data());
            break;
        case OP_LIST_STRING:
            aclopSetAttrListString(opAttr, attr.name.c_str(), attr.listStringAttr.size(),
                const_cast<const char **>(attr.listStringAttr.data()));
            break;
        case OP_LIST_INT_PTR:
            aclopSetAttrListListInt(opAttr, attr.name.c_str(), attr.listIntPtrAttr.size(), const_cast<const int *>(attr.listIntNumValues.data()),
                attr.listIntPtrAttr.data());
            break;
        case OP_DTYPE:
            aclopSetAttrDataType(opAttr, attr.name.c_str(), attr.dtypeAttr);
            break;
        default:
            break;
    }

    return true;
}
