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

* File sample_process.cpp
* Description: handle acl resource
*/
#include <iostream>
#include "acl/acl.h"
#include "params.h"
#include "detectPostprocess.h"
#include "AclLiteUtils.h"
#include "AclLiteApp.h"

using namespace std;

namespace {
const uint32_t kBBoxDataBufId = 0;
const uint32_t kBoxNumDataBufId = 1;
enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };   
uint32_t kModelWidth = 416;
uint32_t kModelHeight = 416;
}

DetectPostprocessThread::DetectPostprocessThread() {
}

DetectPostprocessThread::~DetectPostprocessThread() {
}

AclLiteError DetectPostprocessThread::Init() {
    return ACLLITE_OK;
}

AclLiteError DetectPostprocessThread::Process(int msgId, shared_ptr<void> data) {
    struct timespec time5 = {0, 0};
    struct timespec time6 = {0, 0};
    AclLiteError ret = ACLLITE_OK;
    switch(msgId) {
        case MSG_DETECT_INFER_OUTPUT:
            clock_gettime(CLOCK_REALTIME, &time5);
            InferOutputProcess(static_pointer_cast<CarDetectDataMsg>(data));
            MsgSend(static_pointer_cast<CarDetectDataMsg>(data));
            clock_gettime(CLOCK_REALTIME, &time6);
            //cout << "postprocess time is: " << (time6.tv_sec - time5.tv_sec)*1000 + (time6.tv_nsec - time5.tv_nsec)/1000000 << "ms" << endl;
            break;
        default:
            ACLLITE_LOG_INFO("Detect PostprocessThread thread ignore msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError DetectPostprocessThread::MsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
    while(1) 
    {
        AclLiteError ret = SendMessage(carDetectDataMsg->classifyPreThreadId, MSG_DETECT_POSTPROC_DATA, carDetectDataMsg);
        if(ret == ACLLITE_ERROR_ENQUEUE)
        {
            usleep(500);
            continue;
        }
        else if(ret == ACLLITE_OK)
        {
            break;
        }
        else
        {
            ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
            return ret;
        }
    }

    return ACLLITE_OK;
}

AclLiteError DetectPostprocessThread::InferOutputProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
    if (carDetectDataMsg->isLastFrame) 
        return ACLLITE_OK;
    
    float* detectData = (float *)carDetectDataMsg->detectInferData[kBBoxDataBufId].data.get();
    if(detectData == nullptr){
        ACLLITE_LOG_ERROR("detect inferoutput is null\n");
        return ACLLITE_ERROR;
    }
    uint32_t* boxNum = (uint32_t *)carDetectDataMsg->detectInferData[kBoxNumDataBufId].data.get();
    uint32_t totalBox = boxNum[0];

    float widthScale = (float)(carDetectDataMsg->imageFrame.width) / kModelWidth;
    float heightScale = (float)(carDetectDataMsg->imageFrame.height) / kModelHeight;

    for (uint32_t i = 0; i < totalBox; i++) {
        uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
        if (score < 90) {
            continue;
        }
        uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
        if (objIndex == 2){
            CarInfo carInfo;
            carInfo.rectangle.lt.x = detectData[totalBox * TOPLEFTX + i] * widthScale;
            carInfo.rectangle.lt.y = detectData[totalBox * TOPLEFTY + i] * heightScale;
            carInfo.rectangle.rb.x = detectData[totalBox * BOTTOMRIGHTX + i] * widthScale;
            carInfo.rectangle.rb.y = detectData[totalBox * BOTTOMRIGHTY + i] * heightScale;
            carInfo.detect_result = "car" + std::to_string(score) + "\%";
            carDetectDataMsg->carInfo.emplace_back(carInfo);
        }
    }
    return ACLLITE_OK;
}
