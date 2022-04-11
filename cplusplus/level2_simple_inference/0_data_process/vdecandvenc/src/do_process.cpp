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

* File sample_process.cpp
* Description: handle acl resource
*/
#include <iostream>

#include "acl/acl.h"
#include "acllite/AclLiteModel.h"
#include "do_process.h"
using namespace std;

DoProcess::DoProcess()
:deviceId_(0), context_(nullptr), stream_(nullptr),  isInited_(false){}

DoProcess::~DoProcess() {
    DestroyResource();
}

AclLiteError DoProcess::InitResource() {
    /* 1. ACL初始化 */
    char aclConfigPath[32] = {'\0'};
    aclInit(aclConfigPath);

    /* 2. 运行管理资源申请,包括Device、Context、Stream */
    aclError ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Acl open device %d failed", deviceId_);
        return FAILED;
    }
    ACLLITE_LOG_INFO("Open device %d success", deviceId_);

    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl create context failed");
        return FAILED;
    }
    ACLLITE_LOG_INFO("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl create stream failed\n");
        return ACLLITE_ERROR_CREATE_STREAM;
    }
    ACLLITE_LOG_INFO("create stream success");

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl get run mode failed\n");
        return ACLLITE_ERROR_GET_RUM_MODE;
    }

    return ACLLITE_OK;
}

AclLiteError DoProcess::Init() {
    if (isInited_) {
        ACLLITE_LOG_INFO("instance is initied already!\n");
        return ACLLITE_OK;
    }
    AclLiteError ret = InitResource();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init acl resource failed, error: %d", ret);
        return ret;
    }

    ret = dvpp_.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init dvpp failed\n");
        return ret;
    }

    isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError DoProcess::Process(ImageData& image) {
    AclLiteError ret = encoder_.DoVencProcess(image);
    if(ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("video encode failed\n");
        return ACLLITE_ERROR;
    }
    return ACLLITE_OK;
}
AclLiteError DoProcess::Set(uint32_t width, uint32_t height) {
    AclLiteError ret = encoder_.InitResource(width, height, context_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init video_encoder failed\n");
        return ret;
    }
    return ACLLITE_OK;
}

void DoProcess::DestroyResource()
{
    dvpp_.DestroyResource();
    encoder_.DestroyResource();

    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("destroy stream failed");
        }
        stream_ = nullptr;
    }

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("destroy context failed");
        }
        context_ = nullptr;
    }
    ACLLITE_LOG_INFO("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("reset device failed\n");
    }
    ACLLITE_LOG_INFO("end to reset device is %d\n", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("finalize acl failed\n");
    }
    ACLLITE_LOG_INFO("end to finalize acl");

}

