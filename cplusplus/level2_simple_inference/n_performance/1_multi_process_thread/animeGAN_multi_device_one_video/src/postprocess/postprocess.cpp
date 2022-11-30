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
#include "acl/acl.h"
#include "object_detection.h"
#include "postprocess.h"
#include "AclLiteUtils.h"
#include "acllite/AclLiteApp.h"

using namespace std;

PostprocessThread::PostprocessThread(uint32_t outputWidth, uint32_t outputHeight)
    : g_outputWidth_(outputWidth), g_outputHeight_(outputHeight)
{
}

PostprocessThread::~PostprocessThread()
{
}

AclLiteError PostprocessThread::Init()
{
    return ACLLITE_OK;
}

AclLiteError PostprocessThread::Process(int msgId, shared_ptr<void> data)
{
    struct timespec time5 = {0, 0};
    struct timespec time6 = {0, 0};
    AclLiteError ret = ACLLITE_OK;
    shared_ptr<PostOutputMsg> postOutputMsg = make_shared<PostOutputMsg>();
    switch (msgId) {
        case MSG_INFER_OUTPUT:
            clock_gettime(CLOCK_REALTIME, &time5);
            InferOutputProcess(static_pointer_cast<InferOutputMsg>(data), postOutputMsg);
            MsgSend(static_pointer_cast<InferOutputMsg>(data), postOutputMsg);
            clock_gettime(CLOCK_REALTIME, &time6);
            break;
        default:
            ACLLITE_LOG_INFO("PostprocessThread thread ignore msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError PostprocessThread::MsgSend(shared_ptr<InferOutputMsg> inferMsg,
                                        shared_ptr<PostOutputMsg> &postOutputMsg)
{
    int sendFlag = MSG_VIDEO_ENCODE;
    int timep = 500;
    if (inferMsg->isLastFrame == 1) {
        sendFlag = MSG_ENCODE_FINISH;
    }
    while (1) {
        AclLiteError ret = SendMessage(inferMsg->videoProcessThreadId, sendFlag, postOutputMsg);
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

AclLiteError PostprocessThread::InferOutputProcess(shared_ptr<InferOutputMsg> inferMsg,
                                                   shared_ptr<PostOutputMsg> &postOutputMsg)
{
    postOutputMsg->isLastFrame = inferMsg->isLastFrame;
    postOutputMsg->channelId = inferMsg->channelId;
    postOutputMsg->frameNum = inferMsg->frameNum;
    if (postOutputMsg->isLastFrame)
        return ACLLITE_OK;

    void* data = inferMsg->inferData[0].data.get();
    if (data == nullptr) {
        ACLLITE_LOG_ERROR("inferoutput is null\n");
        return ACLLITE_ERROR;
    }
    uint32_t dataSize = inferMsg->inferData[0].size;
    uint32_t size = static_cast<uint32_t>(dataSize) / sizeof(float);

    cv::Mat mat_result(inferMsg->resizedMat.height, inferMsg->resizedMat.width,
                       CV_32FC3, const_cast<float*>((float*)data));

    mat_result = (mat_result + 1) * 127.5;
    cv::Mat resultImage;
    cv::resize(mat_result, resultImage, cv::Size(inferMsg->frame.cols, inferMsg->frame.rows));
    cv::cvtColor(resultImage, resultImage, CV_RGB2BGR);

    cv::Mat outputImage;
    resultImage.convertTo(outputImage, CV_8U, 255.0/255);
    postOutputMsg->resultImage = outputImage;

    return ACLLITE_OK;
}