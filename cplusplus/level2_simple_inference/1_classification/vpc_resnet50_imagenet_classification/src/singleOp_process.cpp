/**
* @file singleOp_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "singleOp_process.h"
#include <iostream>
#include <vector>
#include "acl/acl.h"
#include "acl/ops/acl_cblas.h"
#include "utils.h"
using namespace std;

SingleOpProcess::SingleOpProcess(aclrtStream &stream) : stream_(stream), tensorSizeCast_(0),
    tensorSizeArgMaxD_(0), devBufferCast_(nullptr), devBufferArgMaxD_(nullptr), inputShape_(0)
{
    inputBuffer_[0] = nullptr;
    outputBufferCast_[0] = nullptr;
    outputBufferArgMaxD_[0] = nullptr;
}

Result SingleOpProcess::Init(const int64_t inputShape)
{
    inputShape_ = inputShape;
    aclError ret = aclopSetModelDir("op_models");
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("set op model dir failed, errorCode is %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("init sigle op success");

    return SUCCESS;
}

Result SingleOpProcess::InitInput(const aclmdlDataset *dataSet)
{
    if (dataSet == nullptr) {
        ERROR_LOG("dataSet is null");
        return FAILED;
    }

    inputBuffer_[0] = aclmdlGetDatasetBuffer(dataSet, 0);
    INFO_LOG("init sigle op input success");
    return SUCCESS;
}

Result SingleOpProcess::RunSigleOpCast()
{
    const int inputNumCast = 1;
    const int outputNumCast = 1;

    aclDataType inputDataTypeCast = ACL_FLOAT;
    aclDataType outputDataTypeCast = ACL_FLOAT16;
    int64_t castShape = inputShape_;
    std::vector<int64_t> inputShapeCast{ castShape };
    std::vector<int64_t> outputShapeCast{ castShape };

    // creat cast input description
    const aclTensorDesc *inputDescCast[inputNumCast];
    inputDescCast[0] = aclCreateTensorDesc(inputDataTypeCast, inputShapeCast.size(),
        inputShapeCast.data(), ACL_FORMAT_ND);
    if (inputDescCast[0] == nullptr) {
        ERROR_LOG("create cast input desc failed");
        return FAILED;
    }

    // creat cast output description
    const aclTensorDesc *outputDescCast[outputNumCast];
    outputDescCast[0] = aclCreateTensorDesc(outputDataTypeCast, outputShapeCast.size(),
        outputShapeCast.data(), ACL_FORMAT_ND);
    if (outputDescCast[0] == nullptr) {
        ERROR_LOG("create cast output desc failed");
        aclDestroyTensorDesc(inputDescCast[0]);
        return FAILED;
    }

    // prepare cast output buffer
    tensorSizeCast_ = aclGetTensorDescSize(outputDescCast[0]);
    aclError ret = aclrtMalloc(&devBufferCast_, tensorSizeCast_, ACL_MEM_MALLOC_NORMAL_ONLY);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("malloc cast output data buffer failed, errorCode is %d", static_cast<int32_t>(ret));
        aclDestroyTensorDesc(inputDescCast[0]);
        aclDestroyTensorDesc(outputDescCast[0]);
        return FAILED;
    }
    outputBufferCast_[0] = aclCreateDataBuffer(devBufferCast_, tensorSizeCast_);
    if (outputBufferCast_[0] == nullptr) {
        ERROR_LOG("create cast output buffer failed");
        aclDestroyTensorDesc(inputDescCast[0]);
        aclDestroyTensorDesc(outputDescCast[0]);
        return FAILED;
    }

    ret = aclopCast(inputDescCast[0], inputBuffer_[0], outputDescCast[0], outputBufferCast_[0], 0, stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("execute singleOp Cast failed, errorCode is %d", static_cast<int32_t>(ret));
        aclDestroyTensorDesc(inputDescCast[0]);
        aclDestroyTensorDesc(outputDescCast[0]);
        return FAILED;
    }

    ret = aclrtSynchronizeStream(stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("singleOp Cast synchronize stream failed, errorCode is %d", static_cast<int32_t>(ret));
        aclDestroyTensorDesc(inputDescCast[0]);
        aclDestroyTensorDesc(outputDescCast[0]);
        return FAILED;
    }

    aclDestroyTensorDesc(inputDescCast[0]);
    aclDestroyTensorDesc(outputDescCast[0]);
    INFO_LOG("execute sigleOp Cast success");

    return SUCCESS;
}

Result SingleOpProcess::RunSigleOpArgMaxD()
{
    std::string opTypeArgMaxD = "ArgMaxD";
    const int inputNumArgMaxD = 1;
    const int outputNumArgMaxD = 1;
    aclDataType inputDataTypeArgMaxD = ACL_FLOAT16;
    aclDataType outputDataTypeArgMaxD = ACL_INT32;
    int inputArgMaxDShape = inputShape_;
    int outputArgMaxDShape = 1;
    std::vector<int64_t> inputShapeArgMaxD{ inputArgMaxDShape };
    std::vector<int64_t> outputShapeArgMaxD{ outputArgMaxDShape };

    // set ArgMaxD attr
    aclopAttr *opAttr = aclopCreateAttr();
    if (opAttr == nullptr) {
        ERROR_LOG("singleOp create attr failed");
        return FAILED;
    }
    aclError ret = aclopSetAttrInt(opAttr, "dimension", 0);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("singleOp set attr failed");
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    // creat ArgMaxD input description
    aclTensorDesc *inputDescArgMaxD[inputNumArgMaxD];
    inputDescArgMaxD[0] = aclCreateTensorDesc(inputDataTypeArgMaxD, inputShapeArgMaxD.size(),
        inputShapeArgMaxD.data(), ACL_FORMAT_ND);
    if (inputDescArgMaxD[0] == nullptr) {
        ERROR_LOG("create argMaxD input desc failed");
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    // creat ArgMaxD output description
    aclTensorDesc *outputDescArgMaxD[outputNumArgMaxD];
    outputDescArgMaxD[0] = aclCreateTensorDesc(outputDataTypeArgMaxD, outputShapeArgMaxD.size(),
        outputShapeArgMaxD.data(), ACL_FORMAT_ND);
    if (outputDescArgMaxD[0] == nullptr) {
        ERROR_LOG("create argMaxD output desc failed");
        aclDestroyTensorDesc(inputDescArgMaxD[0]);
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    // prepare argmaxD output buffer
    tensorSizeArgMaxD_ = aclGetTensorDescSize(outputDescArgMaxD[0]);
    ret = aclrtMalloc(&devBufferArgMaxD_, tensorSizeArgMaxD_, ACL_MEM_MALLOC_NORMAL_ONLY);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("malloc argMaxD output data buffer failed, errorCode is %d", static_cast<int32_t>(ret));
        aclDestroyTensorDesc(inputDescArgMaxD[0]);
        aclDestroyTensorDesc(outputDescArgMaxD[0]);
        aclopDestroyAttr(opAttr);
        return FAILED;
    }
    outputBufferArgMaxD_[0] = aclCreateDataBuffer(devBufferArgMaxD_, tensorSizeArgMaxD_);
    if (outputBufferArgMaxD_[0] == nullptr) {
        ERROR_LOG("create argMaxD output buffer failed");
        aclDestroyTensorDesc(inputDescArgMaxD[0]);
        aclDestroyTensorDesc(outputDescArgMaxD[0]);
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    aclDataBuffer *inputBufferArgMaxD[inputNumArgMaxD];
    inputBufferArgMaxD[0] = outputBufferCast_[0];
    ret = aclopExecute(opTypeArgMaxD.c_str(), inputNumArgMaxD, inputDescArgMaxD, inputBufferArgMaxD,
        outputNumArgMaxD, outputDescArgMaxD, outputBufferArgMaxD_, opAttr, stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("execute singleOp argMaxD failed, errorCode is %d", static_cast<int32_t>(ret));
        aclDestroyTensorDesc(inputDescArgMaxD[0]);
        aclDestroyTensorDesc(outputDescArgMaxD[0]);
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    ret = aclrtSynchronizeStream(stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("singleOp argMaxD synchronize stream failed, errorCode is %d", static_cast<int32_t>(ret));
        aclDestroyTensorDesc(inputDescArgMaxD[0]);
        aclDestroyTensorDesc(outputDescArgMaxD[0]);
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    aclDestroyTensorDesc(inputDescArgMaxD[0]);
    aclDestroyTensorDesc(outputDescArgMaxD[0]);
    aclopDestroyAttr(opAttr);
    INFO_LOG("execute ArgMaxD success");

    return SUCCESS;
}

Result SingleOpProcess::Process()
{
    Result ret = RunSigleOpCast();
    if (ret != SUCCESS) {
        ERROR_LOG("run singleOp cast failed");
        return FAILED;
    }

    ret = RunSigleOpArgMaxD();
    if (ret != SUCCESS) {
        ERROR_LOG("run singleOp ArgMaxD failed");
        return FAILED;
    }

    INFO_LOG("sigleOp process success");
    return SUCCESS;
}

void SingleOpProcess::PrintResult()
{
    if (RunStatus::GetDeviceStatus()) {
        int32_t *index = static_cast<int32_t *>(devBufferArgMaxD_);
        INFO_LOG("---> index of classification result is %d", *index);
        return;
    }
    void* hostBuffer = nullptr;
    aclError ret = aclrtMallocHost(&hostBuffer, tensorSizeArgMaxD_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to print result, malloc host failed");
        return;
    }

    ret = aclrtMemcpy(hostBuffer, tensorSizeArgMaxD_, devBufferArgMaxD_,
        tensorSizeArgMaxD_, ACL_MEMCPY_DEVICE_TO_HOST);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to print result, memcpy device to host failed, errorCode is %d", static_cast<int32_t>(ret));
        aclrtFreeHost(hostBuffer);
        return;
    }

    int32_t* index = static_cast<int32_t*>(hostBuffer);
    INFO_LOG("---> index of classification result is %d", *index);

    aclrtFreeHost(hostBuffer);
}

void SingleOpProcess::destroyResource()
{
    if (devBufferCast_ != nullptr) {
        (void)aclrtFree(devBufferCast_);
        devBufferCast_ = nullptr;
    }

    if (devBufferArgMaxD_ != nullptr) {
        (void)aclrtFree(devBufferArgMaxD_);
        devBufferArgMaxD_ = nullptr;
    }

    if (outputBufferCast_[0] != nullptr) {
        (void)aclDestroyDataBuffer(outputBufferCast_[0]);
        outputBufferCast_[0] = nullptr;
    }

    if (outputBufferArgMaxD_[0] != nullptr) {
        (void)aclDestroyDataBuffer(outputBufferArgMaxD_[0]);
        outputBufferArgMaxD_[0] = nullptr;
    }
}
