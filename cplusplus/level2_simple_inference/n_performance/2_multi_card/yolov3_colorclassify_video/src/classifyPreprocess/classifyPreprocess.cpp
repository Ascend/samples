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
#include <sys/timeb.h>
#include "params.h"
#include "classifyPreprocess.h"
#include "AclLiteApp.h"

using namespace std;

struct timespec time7 = {0, 0};
struct timespec time8 = {0, 0};

namespace {
uint32_t kModelWidth = 224;
uint32_t kModelHeight = 224;
}

ClassifyPreprocessThread::ClassifyPreprocessThread(aclrtRunMode& runMode) : 
runMode_(runMode),
modelWidth_(kModelWidth),
modelHeight_(kModelHeight){
}

ClassifyPreprocessThread::~ClassifyPreprocessThread() {
    dvpp_.DestroyResource();
}

AclLiteError ClassifyPreprocessThread::Init() {

    AclLiteError aclRet = dvpp_.Init();
    if (aclRet) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", aclRet);
        return ACLLITE_ERROR;
    }
    return ACLLITE_OK;
}

AclLiteError ClassifyPreprocessThread::Process(int msgId, 
                             shared_ptr<void> data) {
    AclLiteError ret = ACLLITE_OK;
    switch(msgId) {
        case MSG_DETECT_POSTPROC_DATA:
            clock_gettime(CLOCK_REALTIME, &time7);
            ret = MsgProcess(static_pointer_cast<CarDetectDataMsg>(data));
            ret = MsgSend(static_pointer_cast<CarDetectDataMsg>(data));
            clock_gettime(CLOCK_REALTIME, &time8);
            //cout << "preprocess time is: " << (time8.tv_sec - time7.tv_sec)*1000 + (time8.tv_nsec - time7.tv_nsec)/1000000 << "ms" << endl;
            break;
        default:
            ACLLITE_LOG_INFO("Classify Preprocess thread ignore msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError ClassifyPreprocessThread::Crop(vector<CarInfo> &carImgs, ImageData &orgImg) {
    static int cnt = 0;
    AclLiteError ret = ACLLITE_OK;
    for (int i = 0; i < carImgs.size(); i++) {
        ret = dvpp_.Crop(carImgs[i].cropedImgs, orgImg,
                         carImgs[i].rectangle.lt.x, carImgs[i].rectangle.lt.y,
                         carImgs[i].rectangle.rb.x, carImgs[i].rectangle.rb.y);                                    
        if (ret) {
            ACLLITE_LOG_ERROR("Crop image failed, error: %d, image width %d, "
                            "height %d, size %d, crop area (%d, %d) (%d, %d)", 
                            ret, carImgs[i].cropedImgs.width, carImgs[i].cropedImgs.height,                            
                            carImgs[i].cropedImgs.size, carImgs[i].rectangle.lt.x, 
                            carImgs[i].rectangle.lt.y, carImgs[i].rectangle.rb.x, 
                            carImgs[i].rectangle.rb.y);
            return ACLLITE_ERROR;
        }
    }
    return ret;
}

AclLiteError ClassifyPreprocessThread::Resize(vector<CarInfo> &carImgs) {
    AclLiteError ret = ACLLITE_OK;
    for (size_t i = 0; i < carImgs.size(); i++) {
        AclLiteError ret = dvpp_.Resize(carImgs[i].resizedImgs, carImgs[i].cropedImgs, 
                                      kModelWidth, kModelHeight);
        if (ret) {
            ACLLITE_LOG_ERROR("ColorRecognition Resize image failed");
            return ACLLITE_ERROR;
        }
    }
    return ret;
}

AclLiteError ClassifyPreprocessThread::MsgProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
    if(carDetectDataMsg->isLastFrame == 1)
        return ACLLITE_OK;
    
    carDetectDataMsg->flag = 0;
    //No car detected
    if (carDetectDataMsg->carInfo.size() == 0) {
        carDetectDataMsg->flag = 1;
        return ACLLITE_OK;
    }

    ImageData imageDevice;
    AclLiteError ret = CopyImageToDevice(imageDevice, carDetectDataMsg->imageFrame, runMode_, MEMORY_DVPP);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }

    //crop
    ret = Crop(carDetectDataMsg->carInfo, imageDevice);
    if (ret) {
        ACLLITE_LOG_ERROR("Crop all the data failed, all the data failed");
        return ACLLITE_ERROR;
    }
    //resize
    ret = Resize(carDetectDataMsg->carInfo);
    if (ret) {
        ACLLITE_LOG_ERROR("Resize all the data failed, all the data failed");
        return ACLLITE_ERROR;              
    }
    return ACLLITE_OK;
}

AclLiteError ClassifyPreprocessThread::MsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {

    while(1) 
    {
        AclLiteError ret = SendMessage(carDetectDataMsg->inferThreadId, MSG_CLASSIFY_PREPROC_DATA, carDetectDataMsg);
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
