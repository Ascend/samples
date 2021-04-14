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

* File utils.cpp
* Description: handle file operations
*/
#include "atlas_thread.h"

AtlasThread::AtlasThread()
:context_(nullptr), runMode_(ACL_HOST), 
instanceId_(INVALID_INSTANCE_ID), instanceName_(""), baseConfiged_(false) {
}

AtlasError AtlasThread::BaseConfig(int instanceId,
                                   const string& threadName,
                                   aclrtContext context,
                                   aclrtRunMode runMode) {
    if (baseConfiged_) {
        return ATLAS_ERROR_INITED_ALREADY; 
    }

    instanceId_ = instanceId;
    instanceName_.assign(threadName.c_str());
    context_ = context;
    runMode_ = runMode;

    baseConfiged_ = true;

    return ATLAS_OK;
}

