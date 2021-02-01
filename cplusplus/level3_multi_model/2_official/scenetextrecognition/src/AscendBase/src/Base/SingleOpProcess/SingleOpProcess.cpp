/*
 * Copyright (c) 2020.Huawei Technologies Co., Ltd. All rights reserved.
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

#include "SingleOpProcess.h"
#include "acl/acl.h"

SingleOpProcess::SingleOpProcess()
    : inputTensorNum_(0), outputTensorNum_(0), stream_(nullptr), attr_(nullptr), withHandle_(false), opHandle_(nullptr)
{}

SingleOpProcess::SingleOpProcess(const aclrtStream &stream)
    : inputTensorNum_(0), outputTensorNum_(0), attr_(nullptr), withHandle_(false), opHandle_(nullptr)
{
    stream_ = stream;
}

SingleOpProcess::~SingleOpProcess()
{
    // only need destroy handle, other resource managed by smart pointer
    if (withHandle_ == true) {
        aclopDestroyHandle(opHandle_);
        opHandle_ = nullptr;
    }
    stream_ = nullptr;
}

APP_ERROR SingleOpProcess::SetInputTensor(std::vector<Tensor> tensors)
{
    inputTensorDesc_.clear();
    for (int i = 0; i < inputTensorNum_; i++) {
        // create input tensor description
        std::shared_ptr<aclTensorDesc> tensorDesc(aclCreateTensorDesc(tensors[i].dataType, tensors[i].numDim,
                                                                      tensors[i].dims.data(), tensors[i].format),
                                                  aclDestroyTensorDesc);
        if (tensorDesc == nullptr) {
            LogError << "Failed to create acl input tensor description.";
            return APP_ERR_COMM_INVALID_POINTER;
        }
        inputTensorDesc_.push_back(tensorDesc);
    }
    return APP_ERR_OK;
}

APP_ERROR SingleOpProcess::SetInputDataBuffer(std::vector<std::shared_ptr<void>> inputDataBuf,
    std::vector<size_t> inputBufSize)
{
    inputDataBuf_.clear();
    inputData_ = inputDataBuf;
    for (int i = 0; i < inputTensorNum_; i++) {
        // create input data buffer
        std::shared_ptr<aclDataBuffer> dataBuffer(aclCreateDataBuffer(inputDataBuf[i].get(),
                                                                      inputBufSize[i]),
                                                  aclDestroyDataBuffer);
        if (dataBuffer == nullptr) {
            LogError << "Failed to create acl input data buffer.";
            return APP_ERR_COMM_INVALID_POINTER;
        }
        inputDataBuf_.push_back(dataBuffer);
    }
    return APP_ERR_OK;
}

APP_ERROR SingleOpProcess::SetOutputTensor(std::vector<Tensor> tensors)
{
    outputTensorDesc_.clear();
    outputDataBuf_.clear();
    outputData_.clear();
    outputDataSize_.clear();
    APP_ERROR ret = APP_ERR_OK;
    for (int i = 0; i < outputTensorNum_; i++) {
        // create output tensor describe
        std::shared_ptr<aclTensorDesc> tensorDesc(aclCreateTensorDesc(tensors[i].dataType, tensors[i].numDim,
                                                                      tensors[i].dims.data(), tensors[i].format),
                                                  aclDestroyTensorDesc);
        if (tensorDesc == nullptr) {
            LogError << "Failed to create acl output tensor description.";
            return APP_ERR_COMM_INVALID_POINTER;
        }
        outputTensorDesc_.push_back(tensorDesc);
        // create output data buffer
        void *outDevBuf = nullptr;
        size_t size = aclGetTensorDescSize(tensorDesc.get());
        ret = aclrtMalloc(&outDevBuf, size, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != APP_ERR_OK) {
            LogError << "Failed to aclrtMalloc out result.";
            return ret;
        }
        std::shared_ptr<aclDataBuffer> dataBuf(aclCreateDataBuffer(outDevBuf, size), aclDestroyDataBuffer);
        if (dataBuf == nullptr) {
            LogError << "Failed to create acl output data buffer.";
            return APP_ERR_COMM_INVALID_POINTER;
        }
        outputDataBuf_.push_back(dataBuf);
        std::shared_ptr<void> data(outDevBuf, aclrtFree);
        outputData_.push_back(data);
        outputDataSize_.push_back(size);
    }
    return APP_ERR_OK;
}

void SingleOpProcess::SetInputTensorNum(int num)
{
    inputTensorNum_ = num;
}
int SingleOpProcess::GetInputTensorNum() const
{
    return inputTensorNum_;
}

void SingleOpProcess::SetOutputTensorNum(int num)
{
    outputTensorNum_ = num;
}
int SingleOpProcess::GetOutputTensorNum() const
{
    return outputTensorNum_;
}

std::vector<std::shared_ptr<void>> SingleOpProcess::GetOutputData()
{
    return outputData_;
}

std::vector<size_t> SingleOpProcess::GetOutputDataSize()
{
    return outputDataSize_;
}

std::vector<std::shared_ptr<aclTensorDesc>> SingleOpProcess::GetInputTensorDesc()
{
    return inputTensorDesc_;
}

std::vector<std::shared_ptr<aclTensorDesc>> SingleOpProcess::GetOutputTensorDesc()
{
    return outputTensorDesc_;
}

APP_ERROR SingleOpProcess::SetOpAttr(std::shared_ptr<aclopAttr> attr, OpAttr attrDesc)
{
    attr_ = attr;
    // set operator specifec attribute by attrDesc.type
    switch (attrDesc.type) {
        case BOOL:
            aclopSetAttrBool(attr.get(), attrDesc.name.c_str(), attrDesc.numBool);
            break;
        case INT:
            aclopSetAttrInt(attr.get(), attrDesc.name.c_str(), attrDesc.numInt);
            break;
        case FLOAT:
            aclopSetAttrFloat(attr.get(), attrDesc.name.c_str(), attrDesc.numFloat);
            break;
        case STRING:
            aclopSetAttrString(attr.get(), attrDesc.name.c_str(), attrDesc.numString.c_str());
            break;

        case LIST_BOOL:
            aclopSetAttrListBool(attr.get(), attrDesc.name.c_str(), attrDesc.num, attrDesc.valuesBool.data());
            break;
        case LIST_INT:
            aclopSetAttrListInt(attr.get(), attrDesc.name.c_str(), attrDesc.num, attrDesc.valuesInt.data());
            break;
        case LIST_FLOAT:
            aclopSetAttrListFloat(attr.get(), attrDesc.name.c_str(), attrDesc.num, attrDesc.valuesFloat.data());
            break;
        case LIST_STRING: {
            const char **valuesString = new const char *[attrDesc.num];
            for (int i = 0; i < attrDesc.num; i++) {
                valuesString[i] = attrDesc.valuesString[i].c_str();
            }
            aclopSetAttrListString(attr.get(), attrDesc.name.c_str(), attrDesc.num, valuesString);
            delete[] valuesString;
            break;
        }
        case LIST_LIST_INT: {
            const int64_t **valuesListList = new const int64_t *[attrDesc.num];
            for (int i = 0; i < attrDesc.num; i++) {
                valuesListList[i] = attrDesc.valuesListList[i].data();
            }
            aclopSetAttrListListInt(attr.get(), attrDesc.name.c_str(),
                attrDesc.num, attrDesc.numLists.data(), valuesListList);
            delete[] valuesListList;
            break;
        }
        default:
            break;
    }
    return APP_ERR_OK;
}

std::shared_ptr<aclopAttr> SingleOpProcess::GetOpAttr()
{
    return attr_;
}

void SingleOpProcess::SetStream(aclrtStream stream)
{
    stream_ = stream;
}

const aclrtStream& SingleOpProcess::GetStream() const
{
    return stream_;
}

APP_ERROR SingleOpProcess::RunSingleOp(const bool &withHandle)
{
    withHandle_ = withHandle;
    // convert inputTensorDesc_ to aclTensorDesc ** type for execute singleOp or create handle
    std::vector<aclTensorDesc *> inTensorDesc;
    for (auto tensor : inputTensorDesc_) {
        inTensorDesc.push_back(tensor.get());
    }
    // convert inputDataBuf_ to aclDataBuffer ** type for execute singleOp
    std::vector<aclDataBuffer *> inData;
    for (auto data : inputDataBuf_) {
        inData.push_back(data.get());
    }
    // convert outputTensorDesc_ to aclTensorDesc ** type for execute singleOp or create handle
    std::vector<aclTensorDesc *> outTensorDesc;
    for (auto tensor : outputTensorDesc_) {
        outTensorDesc.push_back(tensor.get());
    }
    // convert outputDataBuf_ to aclDataBuffer ** type for execute singleOp
    std::vector<aclDataBuffer *> outData;
    for (auto data : outputDataBuf_) {
        outData.push_back(data.get());
    }
    APP_ERROR ret = APP_ERR_OK;
    if (withHandle_ == true) {
        // create handle
        ret = aclopCreateHandle(typeName_.c_str(), inputTensorNum_, inTensorDesc.data(),
            outputTensorNum_, outTensorDesc.data(), attr_.get(), &opHandle_);
        if (ret != APP_ERR_OK) {
            LogError << typeName_ << "Failed to create handle, ret = " << ret;
            return ret;
        }
        // execute single operator with handle
        ret = aclopExecWithHandle(opHandle_, inputTensorNum_,
            inData.data(), outputTensorNum_, outData.data(), stream_);
        if (ret != APP_ERR_OK) {
            LogError << typeName_ << "Failed to execute single operator with handle, ret = " << ret;
            return ret;
        }
    } else {
        // execute single operator without handle
        ret = aclopExecute(typeName_.c_str(), inputTensorNum_, inTensorDesc.data(), inData.data(),
            outputTensorNum_, outTensorDesc.data(), outData.data(), attr_.get(), stream_);
        if (ret != APP_ERR_OK) {
            LogError << "Failed to execute singleOp: " << typeName_ << " ret = " << ret;
            return ret;
        }
    }
    return APP_ERR_OK;
}
