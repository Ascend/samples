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

#ifndef YOLOV3_COCO_DETECTION_VENC_INC_DVPP_PROCESS_H
#define YOLOV3_COCO_DETECTION_VENC_INC_DVPP_PROCESS_H

#pragma once
#include <cstdint>

#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "utils.h"

/**
 * DvppProcess
 */
class DvppProcess {
public:
    DvppProcess();

    ~DvppProcess();

Result InitResource(aclrtStream& stream, int imgWidth, int imgHeight);
    Result Venc(cv::Mat& srcImage);
    void DestroyResource();

    protected:
    aclrtStream g_stream_;
    aclvencChannelDesc *g_vencChannelDesc_;
    acldvppPicDesc *g_vpcInputDesc_;
    aclvencFrameConfig *g_vencFrameConfig_;
    acldvppStreamDesc *g_outputStreamDesc;
    void *g_codeInputBufferDev_;
    acldvppPixelFormat g_format_;
    int32_t g_enType_;
    aclrtRunMode g_runMode;
    uint32_t g_inputBufferSize;
    pthread_t g_threadId_;
};

#endif