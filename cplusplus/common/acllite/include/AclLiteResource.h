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

* File AclLiteResource.h
* Description: handle AclLiteResource operations
*/
#ifndef ACLLITE_RESOURCE_H
#define ACLLITE_RESOURCE_H
#pragma once

#include <unistd.h>
#include <string>
#include "acl/acl.h"
#include "AclLiteError.h"

class AclLiteResource {
public:
    AclLiteResource();
    /**
    * @brief Create an AclLiteResource object, and specify the device,
    * config configuration file and thread to use context
    * @param [in]: devId: device id
    * @param [in]: aclConfigPath: config file path
    * @param [in]: UseDefaultCtx: Whether to use the current thread context
    * @return None
    */
    AclLiteResource(int32_t devId, const std::string& aclConfigPath,
                    bool useDefaultCtx = true);
    ~AclLiteResource();
    AclLiteError Init();
    void Release();
    aclrtRunMode GetRunMode()
    {
        return runMode_;
    }
    aclrtContext GetContext()
    {
        return context_;
    }
    aclrtContext GetContextByDevice(int32_t devId);

private:
    bool isReleased_;
    bool useDefaultCtx_;
    int32_t deviceId_;
    aclrtRunMode runMode_;
    aclrtContext context_;
    std::string aclConfig_;
};

#endif