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

AclDevice::AclDevice():deviceId(0) {
    aclConfig.assign("");
    runMode = ACL_HOST;
    isReleased_ = false;    
}

AclDevice::AclDevice(int32_t devId, const string& aclConfigPath):
deviceId(devId), aclConfig(aclConfigPath), 
runMode(ACL_HOST), isReleased_(false) {  
}   

AclDevice::~AclDevice() {
    Release();
}

AtlasError AclDevice::Init() {
    // ACL init
    aclError ret = aclInit(aclConfig.c_str());
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Acl init failed");
        return ATLAS_ERROR;
    }
    ATLAS_LOG_INFO("Acl init ok");

    // open device
    ret = aclrtSetDevice(deviceId);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Acl open device %d failed", deviceId);
        return ATLAS_ERROR;
    }
    ATLAS_LOG_INFO("Open device %d ok", deviceId);

    ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl get run mode failed");
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}

void AclDevice::Release() {
    if (isReleased_) return;

    aclError ret = aclrtResetDevice(deviceId);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("reset device failed");
    }
    ATLAS_LOG_INFO("Reset device %d ok", deviceId);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("finalize acl failed");
    }
    ATLAS_LOG_INFO("Finalize acl ok");

    isReleased_ = true;
}
