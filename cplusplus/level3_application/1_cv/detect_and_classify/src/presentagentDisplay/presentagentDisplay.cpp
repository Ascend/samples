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
#include "acl/acl.h"
#include "AclLiteApp.h"
#include "CarParams.h"
#include "presentagentDisplay.h"
using namespace std;
using namespace ascend::presenter; 
PresentAgentDisplayThread::PresentAgentDisplayThread(Channel* presenterChannel)
    :presenterChannel_(presenterChannel)
{
}

PresentAgentDisplayThread::~PresentAgentDisplayThread()
{
    delete presenterChannel_;
}

AclLiteError PresentAgentDisplayThread::VerifyPresentAgentChannel()
{
    if (presenterChannel_ == nullptr) {
        ACLLITE_LOG_ERROR("Present agent channel is nullptr");
        return ACLLITE_ERROR;
    }
    return ACLLITE_OK;
}

AclLiteError PresentAgentDisplayThread::Init()
{
    AclLiteError ret = VerifyPresentAgentChannel();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Present agent channel is invalid");
        return ret;
    }
    ACLLITE_LOG_INFO("Present agent Display init success");
    return ACLLITE_OK;
}

AclLiteError PresentAgentDisplayThread::DisplayMsgPackage(ImageFrame& packageMsg,
    shared_ptr<CarDetectDataMsg> carDetectDataMsg)
{
    // Jpeg images must serialize the Proto message before they can be sent
    vector<uint8_t> encodeImg;
    vector<int> param = vector<int>(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 95; // default(95) 0-100
    cv::imencode(".jpg", carDetectDataMsg->frame, encodeImg, param);
    // set presentagent identifier
    vector<DetectionResult> detection_results;
    DetectionResult one_result;
    one_result.result_text.append("device-");
    one_result.result_text.append(to_string(carDetectDataMsg->deviceId));
    detection_results.emplace_back(one_result);

    packageMsg.format = ImageFormat::kJpeg;
    packageMsg.width = carDetectDataMsg->frame.cols;
    packageMsg.height = carDetectDataMsg->frame.rows;
    packageMsg.size = encodeImg.size();
    packageMsg.data = reinterpret_cast<uint8_t*>(encodeImg.data());
    packageMsg.detection_results = detection_results;

    return ACLLITE_OK;
}

AclLiteError PresentAgentDisplayThread::DisplayMsgProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg)
{
    if (carDetectDataMsg->isLastFrame == 1) {
        SendMessage(carDetectDataMsg->presentAgentDisplayThreadId, MSG_ENCODE_FINISH, nullptr);
        return ACLLITE_OK;
    }

    ImageFrame frame;
    AclLiteError aclret = DisplayMsgPackage(frame, carDetectDataMsg);
    if (aclret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("display msg package failed");
        return ACLLITE_ERROR;
    }

    PresenterErrorCode ret = PresentImage(presenterChannel_, frame);
    if (ret != PresenterErrorCode::kNone) {
        ACLLITE_LOG_ERROR("Send JPEG image to presenter failed, error %d", (int)ret);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError PresentAgentDisplayThread::Process(int msgId, shared_ptr<void> data)
{
    switch (msgId) {
        case MSG_PRESENT_AGENT_DISPLAY:
            DisplayMsgProcess(static_pointer_cast<CarDetectDataMsg>(data));
            break;
        case MSG_ENCODE_FINISH:
            SendMessage(g_MainThreadId, MSG_APP_EXIT, nullptr);
            break;
        default:
            ACLLITE_LOG_INFO("Present agent display thread ignore msg %d", msgId);
            break;
    }

    return ACLLITE_OK;
}

