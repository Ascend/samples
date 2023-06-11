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

* File utils.cpp
* Description: handle file operations
*/
#include "AclLiteUtils.h"
#include "AclLiteResource.h"

using namespace std;

AclLiteResource::AclLiteResource():isReleased_(false), useDefaultCtx_(true), deviceId_(0),
    runMode_(ACL_HOST), context_(nullptr), aclConfig_("")
{
}

AclLiteResource::AclLiteResource(int32_t devId, const string& aclConfigPath,
    bool useDefaultCtx):isReleased_(false),
    useDefaultCtx_(useDefaultCtx), deviceId_(devId),
    runMode_(ACL_HOST), context_(nullptr), aclConfig_(aclConfigPath)
{
}

AclLiteResource::~AclLiteResource()
{
    Release();
}

AclLiteError AclLiteResource::Init()
{
    // ACL init
    aclError ret = aclInit(aclConfig_.c_str());
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Acl init failed, errorcode is: %d", ret);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Acl init ok");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Acl open device %d failed, errorCode is : %d", deviceId_, ret);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Open device %d ok", deviceId_);

    if (useDefaultCtx_) {
        ACLLITE_LOG_INFO("Use default context currently");
        ret = aclrtGetCurrentContext(&context_);
        if ((ret != ACL_SUCCESS) || (context_ == nullptr)) {
            ACLLITE_LOG_ERROR("Get current acl context failed, errorCode is : %d", ret);
            return ACLLITE_ERROR_GET_ACL_CONTEXT;
        }
    } else {
        ret = aclrtCreateContext(&context_, deviceId_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Create acl context failed, errorCode is : %d", ret);
            return ACLLITE_ERROR_CREATE_ACL_CONTEXT;
        }
    }

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_WARNING("acl get run mode failed, errorCode is : %d", ret);
    }

    return ACLLITE_OK;
}

aclrtContext AclLiteResource::GetContextByDevice(int32_t devId)
{
    aclrtContext context = nullptr;
    aclError ret = aclrtSetDevice(devId);
    if (ret != ACL_ERROR_NONE) {
        ACLLITE_LOG_ERROR("Acl open device %d failed", devId);
        return nullptr;
    }
    ret = aclrtCreateContext(&context, devId);
    if (ret != ACL_ERROR_NONE) {
        ACLLITE_LOG_ERROR("Create acl context failed, error:%d", ret);
        return nullptr;
    }
    return context;
}

void AclLiteResource::Release()
{
    if (isReleased_) {
        return;
    }

    aclError ret;
    if ((!useDefaultCtx_) && (context_ != nullptr)) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("destroy context failed, errorCode is : %d", ret);
        }
        context_ = nullptr;
    }
    ACLLITE_LOG_INFO("destroy context ok");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("reset device failed, errorCode is : %d", ret);
    }
    ACLLITE_LOG_INFO("Reset device %d ok", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("finalize acl failed, errorCode is : %d", ret);
    }
    ACLLITE_LOG_INFO("Finalize acl ok");

    isReleased_ = true;
}
