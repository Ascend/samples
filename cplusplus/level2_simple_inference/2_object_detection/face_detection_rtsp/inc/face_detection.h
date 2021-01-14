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
#include <memory>
#include <vector>
#include <unistd.h>
#include <sys/timeb.h>

#include "atlas_type.h"

using namespace std;

namespace {
const int MSG_APP_START = 1;
const int MSG_READ_FRAME = 2;
const int MSG_PREPROC_DATA = 3;
const int MSG_INFER_OUTPUT = 4;
const int MSG_DECODE_FINISH = 5;
const int MSG_APP_EXIT = 10;

const string kInferName = "inference_0";

const string kPostprocName = "postprocess_1";
const string kPostprocName2 = "postprocess_2";
}

struct PreprocDataMsg {
    bool display;
    int postprocThreadId;
    uint32_t frameWidth;
    uint32_t frameHeight;    
    uint32_t  channelId;    
    ImageData resizedImage;
    ImageData jpgImage;
};

struct InferOutputMsg {
    bool  display;
    uint32_t frameWidth;
    uint32_t frameHeight;    
    uint32_t  channelId;  
    ImageData jpgImage;
    vector<InferenceOutput> inferData;
};