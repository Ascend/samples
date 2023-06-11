/**
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
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

* File sample_process.cpp
* Description: handle acl resource
*/
#include <iostream>
#include <sys/timeb.h>
#include "Params.h"
#include "detectPreprocess.h"
#include "AclLiteApp.h"

using namespace std;

namespace {
const uint32_t kSleepTime = 500;
}

DetectPreprocessThread::DetectPreprocessThread(uint32_t modelWidth, uint32_t modelHeight,
    uint32_t batch)
    :modelWidth_(modelWidth), modelHeight_(modelHeight), isReleased(false), batch_(batch)
{
}

DetectPreprocessThread::~DetectPreprocessThread()
{
    if(!isReleased) {
        dvpp_.DestroyResource();
    }
    isReleased = true;
}

AclLiteError DetectPreprocessThread::Init()
{
    AclLiteError aclRet = dvpp_.Init("DVPP_CHNMODE_VPC");
    if (aclRet) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", aclRet);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::Process(int msgId, shared_ptr<void> data)
{
    switch (msgId) {
        case MSG_PREPROC_DETECTDATA:
            MsgProcess(static_pointer_cast<DetectDataMsg>(data));
            MsgSend(static_pointer_cast<DetectDataMsg>(data));
            break;
        default:
            ACLLITE_LOG_INFO("Detect Preprocess thread ignore msg %d", msgId);
            break;
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::MsgProcess(shared_ptr<DetectDataMsg> detectDataMsg)
{
    AclLiteError ret;
    
    uint32_t  modelInputSize = YUV420SP_SIZE(modelWidth_, modelHeight_) * batch_;
    void* buf = nullptr;
    ret = aclrtMalloc(&buf, modelInputSize, ACL_MEM_MALLOC_HUGE_FIRST);
    if ((buf == nullptr) || (ret != ACL_ERROR_NONE)) {
        ACLLITE_LOG_ERROR("Malloc classify inference input buffer failed, "
                          "error %d", ret);
        return ACLLITE_ERROR;
    }
    uint8_t* batchBuffer = (uint8_t *)buf;
    int32_t setValue = 0;
    aclrtMemset(batchBuffer, modelInputSize, setValue, modelInputSize);
    
    size_t pos = 0;
    for (int i = 0; i < detectDataMsg->decodedImg.size(); i++) {
        ImageData resizedImg;
        ret = dvpp_.Resize(resizedImg,
            detectDataMsg->decodedImg[i], modelWidth_, modelHeight_);
        if (ret == ACLLITE_ERROR) {
            ACLLITE_LOG_ERROR("Resize image failed");
            return ACLLITE_ERROR;
        }
        uint32_t dataSize = YUV420SP_SIZE(modelWidth_, modelHeight_);
        ret = aclrtMemcpy(batchBuffer + pos, dataSize,
        resizedImg.data.get(), resizedImg.size, ACL_MEMCPY_DEVICE_TO_DEVICE);
        pos = pos + dataSize;
    }

    detectDataMsg->modelInputImg.data = SHARED_PTR_DEV_BUF(batchBuffer);
    detectDataMsg->modelInputImg.size = modelInputSize;
    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::MsgSend(shared_ptr<DetectDataMsg> detectDataMsg)
{
    while (1) {
        AclLiteError ret = SendMessage(detectDataMsg->detectInferThreadId, MSG_DO_DETECT_INFER, detectDataMsg);
        if (ret == ACLLITE_ERROR_ENQUEUE) {
            usleep(kSleepTime);
            continue;
        } else if(ret == ACLLITE_OK) {
            break;
        } else {
            ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
            return ret;
        }
    }
    return ACLLITE_OK;
}
