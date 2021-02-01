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

#ifndef SINGLE_OP_PROCESS
#define SINGLE_OP_PROCESS

#include <vector>
#include <string>
#include "acl/acl.h"
#include "CommonDataType/CommonDataType.h"
#include "Log/Log.h"
#include "ErrorCode/ErrorCode.h"

/**
* SingleOpProcess
*/
class SingleOpProcess {
public:
    // constructor
    SingleOpProcess();
    // constructor with stream
    explicit SingleOpProcess(const aclrtStream &stream);
    // deconstructor
    ~SingleOpProcess();

    // Set input
    void SetInputTensorNum(int num);
    int GetInputTensorNum() const;

    // set input tensor description
    APP_ERROR SetInputTensor(std::vector<Tensor> tensors);
    // get input tensor description
    std::vector<std::shared_ptr<aclTensorDesc>> GetInputTensorDesc();

    // set input tensor databuffer & data
    APP_ERROR SetInputDataBuffer(std::vector<std::shared_ptr<void>> inputDataBuf,
        std::vector<size_t> inputBufSize);
    
    void SetOutputTensorNum(int num);
    int GetOutputTensorNum() const;

    APP_ERROR SetOutputTensor(std::vector<Tensor> tensors);

    // get output tensor description
    std::vector<std::shared_ptr<aclTensorDesc>> GetOutputTensorDesc();
   
    // get output data
    std::vector<std::shared_ptr<void>> GetOutputData();
    // get output data size
    std::vector<size_t> GetOutputDataSize();

    // set operator attribute
    APP_ERROR SetOpAttr(std::shared_ptr<aclopAttr> attr, OpAttr attrDesc);
    // get operator attribute
    std::shared_ptr<aclopAttr> GetOpAttr();

    void SetStream(aclrtStream stream);
    const aclrtStream& GetStream() const;
    
    APP_ERROR RunSingleOp(const bool &withHandle);    

private:
    std::string typeName_;  // operator type name

    int inputTensorNum_;  // number of input tensor
    std::vector<std::shared_ptr<aclTensorDesc>> inputTensorDesc_;  // describle of input tensor
    std::vector<std::shared_ptr<aclDataBuffer>> inputDataBuf_;    // input tensor's dataBuf
    // aclDataBuffer is created by follow void*, it must exist until the use of aclDataBuffer is complete
    std::vector<std::shared_ptr<void>> inputData_;    

    int outputTensorNum_;  // number of output tensor
    std::vector<std::shared_ptr<aclTensorDesc>> outputTensorDesc_;  // describle of output tensor
    std::vector<std::shared_ptr<aclDataBuffer>> outputDataBuf_;  // output tensor's dataBuf
    // aclDataBuffer is created by follow void*, it must exist until the use of aclDataBuffer is complete
    std::vector<std::shared_ptr<void>> outputData_;
    std::vector<size_t> outputDataSize_;

    aclrtStream stream_;  // stream of operator execute

    std::shared_ptr<aclopAttr> attr_;  // atrribution of operator
    
    bool withHandle_;   // whether execute singleOp with handle
    aclopHandle *opHandle_;   // execute singleOp with this handle
};

#endif
