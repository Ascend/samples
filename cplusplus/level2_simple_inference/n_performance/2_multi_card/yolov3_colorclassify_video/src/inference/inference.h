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

#include <iostream>
#include <mutex>
#include <unistd.h>
#include "acl/acl.h"
#include "AclLiteModel.h"
#include "AclLiteImageProc.h"
#include "AclLiteThread.h"
#include "params.h"

using namespace std;

/**
* ClassifyProcess
*/
class InferenceThread : public AclLiteThread {
public:
    InferenceThread(aclrtRunMode& runMode);
    ~InferenceThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, shared_ptr<void> data);
private:
    AclLiteError InitModelInput();
    AclLiteError DetectMsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg);
    AclLiteError DetectModelExecute(shared_ptr<CarDetectDataMsg> carDetectDataMsg);
    AclLiteError ClassifyModelExecute(shared_ptr<CarDetectDataMsg> carDetectDataMsg); //wait
    AclLiteError ClassifyMsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg);
    int CopyOneBatchImages(uint8_t* buffer, uint32_t bufferSize, 
                           vector<CarInfo> &carImgs, int batchIdx);
    int CopyImageData(uint8_t *buffer, uint32_t bufferSize, ImageData& image);
    void DestroyResource();
private:   
    AclLiteModel detectModel_;
    AclLiteModel classifyModel_;
    aclrtRunMode runMode_;
    int32_t batchSize_;
    uint32_t classifyInputSize_;
    uint8_t* classifyInputBuf_;
    uint32_t imageInfoSize_;
    void*    imageInfoBuf_;         //释放资源
};

