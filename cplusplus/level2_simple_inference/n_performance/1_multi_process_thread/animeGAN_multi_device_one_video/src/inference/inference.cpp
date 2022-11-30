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

InferenceThread::InferenceThread(const string& modelPath,
                                 uint32_t modelWidth, uint32_t modelHeight,
                                 aclrtContext& context)
    : g_stream_(nullptr), g_context_(context), g_model_(modelPath), 
      g_modelWidth_(modelWidth), g_modelHeight_(modelHeight)
{
}

InferenceThread::~InferenceThread()
{
    aclError ret = aclrtSetCurrentContext(g_context_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("InferenceThread destructor set context failed, error: %d", ret);
    }
    g_model_.DestroyResource();
}

AclLiteError InferenceThread::Init()
{
    AclLiteError ret = aclrtCreateStream(&g_stream_);
    if (ret != ACL_ERROR_NONE) {
        ACLLITE_LOG_ERROR("Create acl stream failed, error %d", ret);
        return ACLLITE_ERROR_CREATE_STREAM;
    }

    ret = g_model_.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Model init failed, error:%d", ret);
        return ret;
    }

    ret = aclrtGetRunMode(&g_runMode_);
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
    inferOutputMsg->channelId = preprocDataMsg->channelId;
    inferOutputMsg->frameNum = preprocDataMsg->frameNum;
    inferOutputMsg->videoProcessThreadId = preprocDataMsg->videoProcessThreadId;
    if (preprocDataMsg->isLastFrame == 1) {
        cout << "it is lastframe in Inference" <<endl;
        return ACLLITE_OK;
    }
    inferOutputMsg->frame = preprocDataMsg->frame;
    inferOutputMsg->resizedMat = preprocDataMsg->resizedMat;

    ImageData imageDevice;
    AclLiteError ret = CopyImageToDevice(imageDevice, preprocDataMsg->resizedMat, g_runMode_, MEMORY_DEVICE);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }
    ret = g_model_.CreateInput(imageDevice.data.get(),
                               imageDevice.size);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }

    ret = g_model_.Execute(inferOutputMsg->inferData);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed, error: %d", ret);
        return ACLLITE_ERROR;
    }
    g_model_.DestroyInput();

    return ACLLITE_OK;
}

AclLiteError InferenceThread::MsgSend(shared_ptr<PreprocDataMsg> preprocDataMsg,
                                      shared_ptr<InferOutputMsg> &inferOutputMsg)
{
    while (1) {
        AclLiteError ret = SendMessage(preprocDataMsg->postprocThreadId, MSG_INFER_OUTPUT, inferOutputMsg);
        int timep = 500;
        if (ret == ACLLITE_ERROR_ENQUEUE) {
            usleep(timep);
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
    switch (msgId) {
        case MSG_PREPROC_DATA:
            clock_gettime(CLOCK_REALTIME, &time3);
            ModelExecute(static_pointer_cast<PreprocDataMsg>(data), inferOutputMsg);
            MsgSend(static_pointer_cast<PreprocDataMsg>(data), inferOutputMsg);
            clock_gettime(CLOCK_REALTIME, &time4);
            break;
        default:
            ACLLITE_LOG_INFO("Inference thread ignore msg %d", msgId);
            break;
    }

    return ACLLITE_OK;
}

