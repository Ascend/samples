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

#include "AclLiteUtils.h"
#include "AclLiteResource.h"

using namespace std;

AclLiteResource::AclLiteResource()
    : g_isReleased_(false),
      g_useDefaultCtx_(true),
      g_deviceId_(0),
      g_runMode_(ACL_HOST),
      g_context_(nullptr),
      g_aclConfig_("")
{   
}

AclLiteResource::~AclLiteResource()
{
    Release();
}

AclLiteError AclLiteResource::Init()
{
    // ACL init
    aclError ret = aclInit(g_aclConfig_.c_str());
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Acl init failed, errorcode is: %d", ret);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Acl init ok");

    // open device
    ret = aclrtSetDevice(g_deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Acl open device %d failed, errorCode is : %d", g_deviceId_, ret);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Open device %d ok", g_deviceId_);

    if (g_useDefaultCtx_) {
        ACLLITE_LOG_INFO("Use default context currently");
        ret = aclrtGetCurrentContext(&g_context_);
        if ((ret != ACL_SUCCESS) || (g_context_ == nullptr)) {
            ACLLITE_LOG_ERROR("Get current acl context failed, errorCode is : %d", ret);
            return ACLLITE_ERROR_GET_ACL_CONTEXT;
        }
    } else {
        ret = aclrtCreateContext(&g_context_, g_deviceId_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Create acl context failed, errorCode is : %d", ret);
            return ACLLITE_ERROR_CREATE_ACL_CONTEXT;
        }
    }

    ret = aclrtGetRunMode(&g_runMode_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_WARNING("acl get run mode failed, errorCode is : %d", ret);
    }

    return ACLLITE_OK;
}

void AclLiteResource::Release()
{
    if (g_isReleased_) {
        return;
    }
    
    aclError ret;
    if ((!g_useDefaultCtx_) && (g_context_ != nullptr)) {
        ret = aclrtDestroyContext(g_context_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("destroy context failed, errorCode is : %d", ret);
        }
        g_context_ = nullptr;
    }
    ACLLITE_LOG_INFO("destroy context ok");
    
    ret = aclrtResetDevice(g_deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("reset device failed, errorCode is : %d", ret);
    }
    ACLLITE_LOG_INFO("Reset device %d ok", g_deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("finalize acl failed, errorCode is : %d", ret);
    }
    ACLLITE_LOG_INFO("Finalize acl ok");

    g_isReleased_ = true;
}
