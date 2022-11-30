/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
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
*/

#include <iostream>
#include <sys/timeb.h>
#include "acl/acl.h"
#include "acllite/AclLiteApp.h"
#include "AclLiteModel.h"
#include "inference.h"
#include "object_detection.h"

using namespace std;
const int g_timep = 500;
InferenceThread::InferenceThread(const string& modelPath,
                                 uint32_t modelWidth, uint32_t modelHeight, 
                                 aclrtContext& context)
    : stream_(nullptr), context_(context), model_(modelPath),
      modelWidth_(modelWidth), modelHeight_(modelHeight)
{
}

InferenceThread::~InferenceThread()
{
    aclError ret = aclrtSetCurrentContext(context_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("InferenceThread destructor set context failed, error: %d", ret);
    }
    model_.DestroyResource();
}

AclLiteError InferenceThread::Init()
{
    AclLiteError ret = aclrtCreateStream(&stream_);
    if (ret != ACL_ERROR_NONE) {
        ACLLITE_LOG_ERROR("Create acl stream failed, error %d", ret);
        return ACLLITE_ERROR_CREATE_STREAM;
    }

    ret = model_.Init();
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
                                           shared_ptr<InferOutputMsg> &inferOutputMsg)
{
    inferOutputMsg->isLastFrame = preprocDataMsg->isLastFrame;
    inferOutputMsg->frameNum = preprocDataMsg->frameNum;
    inferOutputMsg->imageFileName = preprocDataMsg->imageFileName;
    ImageData imageDevice;
    AclLiteError ret = CopyImageToDevice(imageDevice, preprocDataMsg->resizedMat, runMode_, MEMORY_DEVICE);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }
    ret = model_.CreateInput(imageDevice.data.get(),
                               imageDevice.size);
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
                                      shared_ptr<InferOutputMsg> &inferOutputMsg)
{
    while (1) {
        AclLiteError ret = SendMessage(preprocDataMsg->postprocThreadId, MSG_INFER_OUTPUT, inferOutputMsg);
        if (ret == ACLLITE_ERROR_ENQUEUE) {
            usleep(g_timep);
            continue;
        } else if (ret == ACLLITE_OK) {
            break;
        } else {
            ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
            return ret;
        }
    }
    return ACLLITE_OK;
}

AclLiteError InferenceThread::MsgSendEnd(shared_ptr<PreprocDataMsg> preprocDataMsg)
{
    while (1) {
        AclLiteError ret = SendMessage(preprocDataMsg->postprocThreadId, MSG_ENCODE_FINISH, nullptr);
        if (ret == ACLLITE_ERROR_ENQUEUE) {
            usleep(g_timep);
            continue;
        } else if (ret == ACLLITE_OK) {
            break;
        } else {
            ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
            return ret;
        }
    }

    return ACLLITE_OK;
}

AclLiteError InferenceThread::Process(int msgId, shared_ptr<void> data)
{
    struct timespec time3 = {0, 0};
    struct timespec time4 = {0, 0};
    shared_ptr<InferOutputMsg> inferOutputMsg = make_shared<InferOutputMsg>();
    AclLiteError ret;
    switch (msgId) {
        case MSG_PREPROC_DATA:
            clock_gettime(CLOCK_REALTIME, &time3);
            ModelExecute(static_pointer_cast<PreprocDataMsg>(data), inferOutputMsg);
            MsgSend(static_pointer_cast<PreprocDataMsg>(data), inferOutputMsg);
            clock_gettime(CLOCK_REALTIME, &time4);
            cout << "inference time is: " << (time4.tv_sec - time3.tv_sec)*1000 + (time4.tv_nsec - time3.tv_nsec)/1000000 << "ms" << endl;
            break;
        case MSG_PREPROC_END:
            MsgSendEnd(static_pointer_cast<PreprocDataMsg>(data));
            break;
        default:
            ACLLITE_LOG_INFO("Inference thread ignore msg %d", msgId);
            break;
    }

    return ACLLITE_OK;
}