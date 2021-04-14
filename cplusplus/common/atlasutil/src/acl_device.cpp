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
#include "atlas_utils.h"
#include "acl_device.h"

using namespace std;

AclDevice::AclDevice():
deviceId_(0), 
aclConfig_(""), 
runMode_(ACL_HOST), 
context_(nullptr),
useDefaultCtx_(true),
isReleased_(false) {   
}

AclDevice::AclDevice(int32_t devId, 
                     const string& aclConfigPath, 
                     bool useDefaultCtx):
deviceId_(devId), 
aclConfig_(aclConfigPath), 
runMode_(ACL_HOST),
context_(nullptr), 
useDefaultCtx_(useDefaultCtx),
isReleased_(false) {  
}   

AclDevice::~AclDevice() {
    Release();
}

AtlasError AclDevice::Init() {
    // ACL init
    aclError ret = aclInit(aclConfig_.c_str());
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Acl init failed");
        return ATLAS_ERROR;
    }
    ATLAS_LOG_INFO("Acl init ok");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Acl open device %d failed", deviceId_);
        return ATLAS_ERROR;
    }
    ATLAS_LOG_INFO("Open device %d ok", deviceId_);

    if (useDefaultCtx_) {
        ret = aclrtGetCurrentContext(&context_);
        if ((ret != ACL_ERROR_NONE) || (context_ == nullptr)) {
            ATLAS_LOG_ERROR("Get current acl context error:%d", ret);
            return ATLAS_ERROR_GET_ACL_CONTEXT;
        }
    } else {
        ret = aclrtCreateContext(&context_, deviceId_);
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("Create acl context failed, error:%d", ret);
            return ATLAS_ERROR_CREATE_ACL_CONTEXT;
        }
    }

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl get run mode failed");
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}

void AclDevice::Release() {
    if (isReleased_) return;

    aclError ret;
    if ((useDefaultCtx_ == false) && (context_ != nullptr)) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("destroy context failed");
        }
        context_ = nullptr;
    }

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("reset device failed");
    }
    ATLAS_LOG_INFO("Reset device %d ok", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("finalize acl failed");
    }
    ATLAS_LOG_INFO("Finalize acl ok");

    isReleased_ = true;
}
