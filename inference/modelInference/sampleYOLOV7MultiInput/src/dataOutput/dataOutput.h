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

* File sample_process.h
* Description: handle acl resource
*/
#ifndef DATAOUTPUTTHREAD_H
#define DATAOUTPUTTHREAD_H
#pragma once

#include <iostream>
#include <mutex>
#include <queue>
#include <unistd.h>
#include "acl/acl.h"
#include "Params.h"
#include "AclLiteError.h"
#include "AclLiteUtils.h"
#include "AclLiteThread.h"
#include "AclLiteApp.h"

class DataOutputThread : public AclLiteThread {
public:
    DataOutputThread(aclrtRunMode& runMode,
        std::string outputDataType, std::string outputPath,
        int postThreadNum);
    ~DataOutputThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, std::shared_ptr<void> data);

private:
    AclLiteError SetOutputVideo();
    AclLiteError ShutDownProcess();
    AclLiteError RecordQueue(std::shared_ptr<DetectDataMsg> detectDataMsg);
    AclLiteError DataProcess();
    AclLiteError ProcessOutput(std::shared_ptr<DetectDataMsg> detectDataMsg);

    AclLiteError SaveResultVideo(std::shared_ptr<DetectDataMsg> &detectDataMsg);
    AclLiteError SaveResultPic(std::shared_ptr<DetectDataMsg> &detectDataMsg);
    AclLiteError PrintResult(std::shared_ptr<DetectDataMsg> &detectDataMsg);
    AclLiteError SendCVImshow(std::shared_ptr<DetectDataMsg> &detectDataMsg);
    AclLiteError DisplayMsgSend(std::shared_ptr<DetectDataMsg> detectDataMsg);
    AclLiteError SendImageToRtsp(std::shared_ptr<DetectDataMsg> &detectDataMsg);

private:
    aclrtRunMode runMode_;
    cv::VideoWriter outputVideo_;
    std::string outputDataType_;
    std::string outputPath_;
    int shutdown_;
    int postNum_;
    std::queue<std::shared_ptr<DetectDataMsg>> postQueue_[4];
    uint32_t frameCnt_;
    int64_t lastDecodeTime_;
    int64_t lastRecordTime_;
};

#endif
