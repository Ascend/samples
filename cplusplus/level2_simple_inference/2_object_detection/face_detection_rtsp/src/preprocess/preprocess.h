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

* File sample_process.h
* Description: handle acl resource
*/
#pragma once

#include <iostream>
#include <mutex>
#include <unistd.h>

#include "acl/acl.h"
#include "atlas_thread.h"
#include "atlas_videocapture.h"
#include "dvpp_process.h"
#include "venc_process.h"


class Preprocess : public AtlasThread {
public:
    Preprocess(string& streamName, uint32_t modelWidth, 
               uint32_t modelHeight, uint32_t channelId, bool display);
    ~Preprocess();

    AtlasError Init();
    AtlasError Process(int msgId, shared_ptr<void> msgData);
    
private:
    AtlasError AppStartMsgProcess();
    AtlasError ReadFrameMsgProcess();
    AtlasError OpenVideoCapture();
    AtlasError GetThreadInstanceId();
    void ProcessImage(ImageData image);
    AtlasError ConvertImage(ImageData& destImage, 
                            ImageData& srcImage);

private:
    string streamName_;
    AtlasVideoCapture* cap_;
    aclrtStream stream_;
    DvppProcess dvpp_;

    bool display_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    uint32_t channelId_;    
    
    int selfThreadId_;
    int nextThreadId_;
    int postprocThreadId_;
    int frameCnt_;
    
};

