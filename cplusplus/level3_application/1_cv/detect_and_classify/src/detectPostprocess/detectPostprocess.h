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
#ifndef DETECTPOSTPROCESSTHREAD_H
#define DETECTPOSTPROCESSTHREAD_H
#pragma once

#include <iostream>
#include <mutex>
#include <unistd.h>
#include "acl/acl.h"
#include "AclLiteError.h"
#include "AclLiteImageProc.h"
#include "AclLiteThread.h"
#include "CarParams.h"

class DetectPostprocessThread : public AclLiteThread {
public:
    DetectPostprocessThread();
    ~DetectPostprocessThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, std::shared_ptr<void> data);

private:
    AclLiteError InferOutputProcess(std::shared_ptr<CarDetectDataMsg> carDetectDataMsg);
    AclLiteError MsgSend(std::shared_ptr<CarDetectDataMsg> carDetectDataMsg);

private:
};

#endif