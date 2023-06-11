/**
* @file op_test_desc.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#ifndef OPERATOR_DESC_H
#define OPERATOR_DESC_H

#include <string>
#include <vector>
#include <cstdint>
#include "acl/acl.h"
#include "acl/acl_op.h"
enum AttrType {
    OP_BOOL,
    OP_INT,
    OP_FLOAT,
    OP_STRING,
    OP_LIST_BOOL,
    OP_LIST_INT,
    OP_LIST_FLOAT,
    OP_LIST_STRING,
    OP_LIST_INT_PTR,
    OP_DTYPE
};

struct OpTestAttr {
    enum AttrType type;
    std::string name;
    bool boolAttr;
    int32_t intAttr;
    float floatAttr;
    std::string stringAttr;
    std::vector<uint8_t> listBoolAttr;
    std::vector<int64_t> listIntAttr;
    std::vector<float> listFloatAttr;
    std::vector<const char*> listStringAttr;
    std::vector<int64_t*> listIntPtrAttr;
    std::vector<int32_t> listIntNumValues;
    aclDataType dtypeAttr;
};

/**
 * op test case description
 */
struct OpTestDesc {
    /**
     * Constructor
     * @param [in] opType: op type
     */
    explicit OpTestDesc(std::string opType);

    /**
     * Destructor
     */
    ~OpTestDesc();

    /**
     * Add an input tensor description
     * @param [in] dataType: data type
     * @param [in] numDims: number of dims
     * @param [in] dims: dims
     * @param [in] format: format
     * @return OpTestDesc
     */
    OpTestDesc &AddInputTensorDesc(aclDataType dataType, int32_t numDims, const int64_t *dims, aclFormat format);

    /**
     * Add an output tensor description
     * @param [in] dataType: data type
     * @param [in] numDims: number of dims
     * @param [in] dims: dims
     * @param [in] format: format
     * @return OpTestDesc
     */
    OpTestDesc &AddOutputTensorDesc(aclDataType dataType, int32_t numDims, const int64_t *dims, aclFormat format);

    bool AddTensorAttr(const OpTestAttr &attr);
    std::string opType;
    std::vector<std::vector<int64_t>> inputShape;
    std::vector<aclDataType> inputDataType;
    std::vector<aclFormat> inputFormat;
    std::vector<std::string> inputFilePath;
    std::vector<aclTensorDesc *> inputDesc;
    std::vector<bool> inputConst;
    std::vector<std::vector<int64_t>> outputShape;
    std::vector<aclDataType> outputDataType;
    std::vector<aclFormat> outputFormat;
    std::vector<std::string> outputFilePath;
    std::vector<aclTensorDesc *> outputDesc;
    std::vector<OpTestAttr> opAttrVec;
    aclopAttr *opAttr;
};

#endif // OPERATOR_DESC_H
