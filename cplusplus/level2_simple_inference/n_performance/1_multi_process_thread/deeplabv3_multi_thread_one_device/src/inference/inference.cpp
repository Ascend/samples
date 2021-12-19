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
#include "acl/acl.h"
#include "acllite/AclLiteApp.h"
#include "AclLiteModel.h"
#include "inference.h"
#include "object_detection.h"

using namespace std;
struct timespec time3 = {0, 0};
struct timespec time4 = {0, 0};

InferenceThread::InferenceThread(const string& modelPath,
                     uint32_t modelWidth, uint32_t modelHeight) : 
model_(modelPath), modelWidth_(modelWidth), modelHeight_(modelHeight){
}

InferenceThread::~InferenceThread() {
    model_.DestroyResource();
}

AclLiteError InferenceThread::Init() {
    AclLiteError ret = model_.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Model init failed, error:%d", ret);
        return ret;
    }

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError InferenceThread::ModelExecute(shared_ptr<PreprocDataMsg> preprocDataMsg,
                                       shared_ptr<InferOutputMsg> &inferOutputMsg) {
    inferOutputMsg->isLastFrame = preprocDataMsg->isLastFrame;
    inferOutputMsg->frameNum = preprocDataMsg->frameNum;
    inferOutputMsg->frame = preprocDataMsg->frame;
    inferOutputMsg->imageFileName = preprocDataMsg->imageFileName;

    AclLiteError ret = model_.CreateInput(preprocDataMsg->resizedMat.data.get(),
                                    preprocDataMsg->resizedMat.size);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }

    ret = model_.Execute(inferOutputMsg->inferData);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed, error: %d", ret);
        return ACLLITE_ERROR;
    }
    model_.DestroyInput();

    return ACLLITE_OK;
}

AclLiteError InferenceThread::MsgSend(shared_ptr<PreprocDataMsg> preprocDataMsg,
                                       shared_ptr<InferOutputMsg> &inferOutputMsg) {
    while(1)
    {
        AclLiteError ret = SendMessage(preprocDataMsg->postprocThreadId,MSG_INFER_OUTPUT, inferOutputMsg);
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

AclLiteError InferenceThread::Process(int msgId, shared_ptr<void> data) {
    shared_ptr<InferOutputMsg> inferOutputMsg = make_shared<InferOutputMsg>();
    shared_ptr<PreprocDataMsg> msgtmp = make_shared<PreprocDataMsg>();
    AclLiteError ret;
    switch(msgId) {
        case MSG_PREPROC_DATA:
            clock_gettime(CLOCK_REALTIME, &time3);
            ModelExecute(static_pointer_cast<PreprocDataMsg>(data), inferOutputMsg);
            MsgSend(static_pointer_cast<PreprocDataMsg>(data), inferOutputMsg);
            clock_gettime(CLOCK_REALTIME, &time4);
            cout << "inference time is: " << (time4.tv_sec - time3.tv_sec)*1000 + (time4.tv_nsec - time3.tv_nsec)/1000000 << "ms" << endl;
            break;
        case MSG_PREPROC_END:
            msgtmp = static_pointer_cast<PreprocDataMsg>(data);
            ret = SendMessage(msgtmp->postprocThreadId, MSG_ENCODE_FINISH, nullptr);
            if (ret != ACLLITE_OK) {
                ACLLITE_LOG_ERROR("Send frame end message failed, error %d", ret);
                return ret;
            } 
            ACLLITE_LOG_INFO("Send frame end message success, frame total num is %d", msgtmp->frameNum);
            break;
        default:
            ACLLITE_LOG_INFO("Inference thread ignore msg %d", msgId);
            break;
    }

    return ACLLITE_OK;
}

