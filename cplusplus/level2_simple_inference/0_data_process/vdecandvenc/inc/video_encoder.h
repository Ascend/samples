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
#ifndef VIDEO_DECODER_H
#define VIDEO_DECODER_H
#pragma once

#include <iostream>
#include <mutex>
#include <unistd.h>

#include "acl/acl.h"
#include "utils.h"
#include "acllite/AclLiteModel.h"
#include "acllite/AclLiteImageProc.h"

class VideoEncoder {
public:
    VideoEncoder();
    ~VideoEncoder();
    AclLiteError Init(uint32_t width, uint32_t height);
    AclLiteError VideoEncode(ImageData& image);

private:
    AclLiteError InitAclResource();
    AclLiteError InitVencResource(uint32_t width, uint32_t height);
    Result SetFrameConfig(uint8_t eos, uint8_t forceIFrame);
    Result CreatePicDesc();
    AclLiteError DestroyVencResource();
    void DestroyResource();
private:
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    aclrtRunMode runMode_;

    aclvencChannelDesc *vencChannelDesc_;
    aclvencFrameConfig *vencFrameConfig_;
    acldvppPicDesc *inputPicputDesc_;
    acldvppStreamDesc *outputStreamDesc_;
    void *inBufferDev_;
    uint32_t inBufferSize_;
    pthread_t threadId_;

    acldvppPixelFormat format_;
    acldvppStreamFormat enType_;

    char *outFolder_;
    bool isInited_;
    uint32_t width_;
    uint32_t height_;
};

#endif