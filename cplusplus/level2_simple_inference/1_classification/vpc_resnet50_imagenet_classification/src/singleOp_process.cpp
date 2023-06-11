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
    tensorSizeArgMaxV2_(0), devBufferCast_(nullptr), devBufferArgMaxV2_(nullptr), inputShape_(0)
{
    inputBuffer_[0] = nullptr;
    outputBufferCast_[0] = nullptr;
    outputBufferArgMaxV2_[0] = nullptr;
}

Result SingleOpProcess::Init(const int64_t inputShape)
{
    inputShape_ = inputShape;
    aclError ret = aclopSetModelDir("op_models");
    if (ret != ACL_SUCCESS) {
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
    if (ret != ACL_SUCCESS) {
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
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("execute singleOp Cast failed, errorCode is %d", static_cast<int32_t>(ret));
        aclDestroyTensorDesc(inputDescCast[0]);
        aclDestroyTensorDesc(outputDescCast[0]);
        return FAILED;
    }

    ret = aclrtSynchronizeStream(stream_);
    if (ret != ACL_SUCCESS) {
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

Result SingleOpProcess::RunSigleOpArgMaxV2()
{
    std::string opTypeArgMaxV2 = "ArgMaxV2";
    const int inputNumArgMaxV2 = 2;
    const int outputNumArgMaxV2 = 1;
    aclDataType inputDataTypeArgMaxV2 = ACL_FLOAT16;
    aclDataType outputDataTypeArgMaxV2 = ACL_INT32;
    int inputArgMaxV2Shape = inputShape_;
    int outputArgMaxV2Shape = 1;
    std::vector<int64_t> inputShapeArgMaxV2{ inputArgMaxV2Shape };
    std::vector<int64_t> inputShapeArgMaxV2Second{ 1 };
    std::vector<int64_t> outputShapeArgMaxV2{ outputArgMaxV2Shape };

    // set ArgMaxV2 attr
    aclopAttr *opAttr = aclopCreateAttr();
    if (opAttr == nullptr) {
        ERROR_LOG("singleOp create attr failed");
        return FAILED;
    }

    // creat ArgMaxV2 input description
    aclTensorDesc *inputDescArgMaxV2[inputNumArgMaxV2];
    inputDescArgMaxV2[0] = aclCreateTensorDesc(inputDataTypeArgMaxV2, inputShapeArgMaxV2.size(),
        inputShapeArgMaxV2.data(), ACL_FORMAT_ND);
    if (inputDescArgMaxV2[0] == nullptr) {
        ERROR_LOG("create ArgMaxV2 input desc failed");
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    inputDescArgMaxV2[1] = aclCreateTensorDesc(ACL_INT32, inputShapeArgMaxV2Second.size(),
        inputShapeArgMaxV2Second.data(), ACL_FORMAT_ND);
    if (inputDescArgMaxV2[1] == nullptr) {
        ERROR_LOG("create ArgMaxV2 second input desc failed");
        aclDestroyTensorDesc(inputDescArgMaxV2[0]);
        aclopDestroyAttr(opAttr);
        return FAILED;
    }
    int32_t dimension = 0;
    aclError ret = aclSetTensorConst(inputDescArgMaxV2[1], &dimension, sizeof(dimension));
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("set second tensor const failed");
        aclDestroyTensorDesc(inputDescArgMaxV2[0]);
        aclDestroyTensorDesc(inputDescArgMaxV2[1]);
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    // creat ArgMaxV2 output description
    aclTensorDesc *outputDescArgMaxV2[outputNumArgMaxV2];
    outputDescArgMaxV2[0] = aclCreateTensorDesc(outputDataTypeArgMaxV2, outputShapeArgMaxV2.size(),
        outputShapeArgMaxV2.data(), ACL_FORMAT_ND);
    if (outputDescArgMaxV2[0] == nullptr) {
        ERROR_LOG("create ArgMaxV2 output desc failed");
        aclDestroyTensorDesc(inputDescArgMaxV2[0]);
        aclDestroyTensorDesc(inputDescArgMaxV2[1]);
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    // prepare ArgMaxV2 output buffer
    tensorSizeArgMaxV2_ = aclGetTensorDescSize(outputDescArgMaxV2[0]);
    ret = aclrtMalloc(&devBufferArgMaxV2_, tensorSizeArgMaxV2_, ACL_MEM_MALLOC_NORMAL_ONLY);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc ArgMaxV2 output data buffer failed, errorCode is %d", static_cast<int32_t>(ret));
        aclDestroyTensorDesc(inputDescArgMaxV2[0]);
        aclDestroyTensorDesc(inputDescArgMaxV2[1]);
        aclDestroyTensorDesc(outputDescArgMaxV2[0]);
        aclopDestroyAttr(opAttr);
        return FAILED;
    }
    outputBufferArgMaxV2_[0] = aclCreateDataBuffer(devBufferArgMaxV2_, tensorSizeArgMaxV2_);
    if (outputBufferArgMaxV2_[0] == nullptr) {
        ERROR_LOG("create ArgMaxV2 output buffer failed");
        aclDestroyTensorDesc(inputDescArgMaxV2[0]);
        aclDestroyTensorDesc(inputDescArgMaxV2[1]);
        aclDestroyTensorDesc(outputDescArgMaxV2[0]);
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    aclDataBuffer *inputBufferArgMaxV2[inputNumArgMaxV2];
    inputBufferArgMaxV2[0] = outputBufferCast_[0];
    inputBufferArgMaxV2[1] = aclCreateDataBuffer(nullptr, 0);
    ret = aclopExecuteV2(opTypeArgMaxV2.c_str(), inputNumArgMaxV2, inputDescArgMaxV2, inputBufferArgMaxV2,
        outputNumArgMaxV2, outputDescArgMaxV2, outputBufferArgMaxV2_, opAttr, stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("execute singleOp ArgMaxV2 failed, errorCode is %d", static_cast<int32_t>(ret));
        aclDestroyTensorDesc(inputDescArgMaxV2[0]);
        aclDestroyTensorDesc(inputDescArgMaxV2[1]);
        aclDestroyTensorDesc(outputDescArgMaxV2[0]);
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    ret = aclrtSynchronizeStream(stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("singleOp ArgMaxV2 synchronize stream failed, errorCode is %d", static_cast<int32_t>(ret));
        aclDestroyTensorDesc(inputDescArgMaxV2[0]);
        aclDestroyTensorDesc(inputDescArgMaxV2[1]);
        aclDestroyTensorDesc(outputDescArgMaxV2[0]);
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    aclDestroyTensorDesc(inputDescArgMaxV2[0]);
    aclDestroyTensorDesc(inputDescArgMaxV2[1]);
    aclDestroyTensorDesc(outputDescArgMaxV2[0]);
    aclopDestroyAttr(opAttr);
    INFO_LOG("execute ArgMaxV2 success");

    return SUCCESS;
}

Result SingleOpProcess::Process()
{
    Result ret = RunSigleOpCast();
    if (ret != SUCCESS) {
        ERROR_LOG("run singleOp cast failed");
        return FAILED;
    }

    ret = RunSigleOpArgMaxV2();
    if (ret != SUCCESS) {
        ERROR_LOG("run singleOp ArgMaxV2 failed");
        return FAILED;
    }

    INFO_LOG("sigleOp process success");
    return SUCCESS;
}

void SingleOpProcess::PrintResult()
{
    if (RunStatus::GetDeviceStatus()) {
        int32_t *index = static_cast<int32_t *>(devBufferArgMaxV2_);
        INFO_LOG("---> index of classification result is %d", *index);
        return;
    }
    void* hostBuffer = nullptr;
    aclError ret = aclrtMallocHost(&hostBuffer, tensorSizeArgMaxV2_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("fail to print result, malloc host failed");
        return;
    }

    ret = aclrtMemcpy(hostBuffer, tensorSizeArgMaxV2_, devBufferArgMaxV2_,
        tensorSizeArgMaxV2_, ACL_MEMCPY_DEVICE_TO_HOST);
    if (ret != ACL_SUCCESS) {
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

    if (devBufferArgMaxV2_ != nullptr) {
        (void)aclrtFree(devBufferArgMaxV2_);
        devBufferArgMaxV2_ = nullptr;
    }

    if (outputBufferCast_[0] != nullptr) {
        (void)aclDestroyDataBuffer(outputBufferCast_[0]);
        outputBufferCast_[0] = nullptr;
    }

    if (outputBufferArgMaxV2_[0] != nullptr) {
        (void)aclDestroyDataBuffer(outputBufferArgMaxV2_[0]);
        outputBufferArgMaxV2_[0] = nullptr;
    }
}
