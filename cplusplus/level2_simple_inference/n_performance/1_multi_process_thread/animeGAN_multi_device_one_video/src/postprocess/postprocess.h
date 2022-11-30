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

#ifndef ANIMEGAN_MULTI_DEVICE_ONE_VIDEO_POSTPROCESS_POSTPROCESS_H
#define ANIMEGAN_MULTI_DEVICE_ONE_VIDEO_POSTPROCESS_POSTPROCESS_H

#pragma once

#include <iostream>
#include <mutex>
#include <unistd.h>
#include "acl/acl.h"
#include "AclLiteError.h"
#include "acllite/AclLiteImageProc.h"
#include "acllite/AclLiteThread.h"

using namespace std;

class PostprocessThread : public AclLiteThread {
public:
    PostprocessThread(uint32_t outputWidth, uint32_t outputHeight);
    ~PostprocessThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, shared_ptr<void> data);

private:
    AclLiteError InferOutputProcess(shared_ptr<InferOutputMsg> inferMsg,
                                    shared_ptr<PostOutputMsg> &postOutputMsg);
    AclLiteError MsgSend(shared_ptr<InferOutputMsg> inferMsg,
                        shared_ptr<PostOutputMsg> &postOutputMsg);

private:
    uint32_t g_outputWidth_;
    uint32_t g_outputHeight_;
};

#endif