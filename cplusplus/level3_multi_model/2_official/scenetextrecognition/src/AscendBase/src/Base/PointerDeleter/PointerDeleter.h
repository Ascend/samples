/*
 * Copyright(C) 2020. Huawei Technologies Co.,Ltd. All rights reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef POINTER_DELETER_H
#define POINTER_DELETER_H

#include "acl/acl.h"
#include "ErrorCode/ErrorCode.h"

#ifndef ENABLE_DVPP_INTERFACE
#define ENABLE_DVPP_INTERFACE
#include "acl/ops/acl_dvpp.h"
#endif

template<typename T, typename F> void SetContextAndFree(T p, aclrtContext &context, APP_ERROR (*freeFunPtr)(F))
{
    APP_ERROR ret = aclrtSetCurrentContext(context);
    if (ret != APP_ERR_OK) {
        LogError << "Fail to set context for deleter.";
        return;
    }

    ret = freeFunPtr(p);
    if (ret != APP_ERR_OK) {
        LogError << "Fail to free memory. addr " << (void *)p;
        return;
    }
}

template<typename T, typename F> void AscendDeleter(T p, aclrtContext dstContext, APP_ERROR (*freeFunPtr)(F))
{
    aclrtContext curContext;
    int32_t deviceId;
    APP_ERROR ret = aclrtGetCurrentContext(&curContext);
    if (ret != APP_ERR_OK) {
        LogDebug << "can not get context for deleter.";
        ret = aclrtGetDevice(&deviceId);
        if (ret != APP_ERR_OK) {
            LogDebug << "current thread not bound to device.";
            SetContextAndFree(p, dstContext, freeFunPtr);
        } else {
            SetContextAndFree(p, dstContext, freeFunPtr);
            ret = aclrtSetDevice(deviceId);
            if (ret != APP_ERR_OK) {
                LogError << "Fail to set device for deleter.";
                return;
            }
        }
    } else {
        LogDebug << "current thread has context for deleter.";
        SetContextAndFree(p, dstContext, freeFunPtr);
        ret = aclrtSetCurrentContext(curContext);
        if (ret != APP_ERR_OK) {
            LogError << "Fail to set context for deleter.";
            return;
        }
    }
}

#endif