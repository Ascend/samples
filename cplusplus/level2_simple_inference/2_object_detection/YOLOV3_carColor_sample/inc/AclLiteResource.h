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

#ifndef YOLOV3_CARCOLOR_SAMPLE_INC_ACLLITERESOURCE_H
#define YOLOV3_CARCOLOR_SAMPLE_INC_ACLLITERESOURCE_H

#pragma once

#include <unistd.h>
#include <string>
#include "acl/acl.h"
#include "AclLiteError.h"

class AclLiteResource {
public:
    AclLiteResource();
    ~AclLiteResource();

    AclLiteError Init();
    void Release();

private:
    bool g_isReleased_;
    bool g_useDefaultCtx_;
    int32_t g_deviceId_;
    aclrtRunMode g_runMode_;
    aclrtContext g_context_;
    std::string g_aclConfig_;
};

#endif