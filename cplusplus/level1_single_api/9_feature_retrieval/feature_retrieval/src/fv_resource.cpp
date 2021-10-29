/**
* @file fv_resource.cpp
*
* Copyright (C) 2021. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "fv_resource.h"
#include "common.h"

FVResource::~FVResource()
{
    Finalize();
}

FVResource &FVResource::GetInstance()
{
    static FVResource inst;
    return inst;
}

bool FVResource::Initialize(size_t fsNum, int32_t deviceId)
{
    // acl initialize
    aclError ret = aclInit(nullptr);
    if (ret != ACL_SUCCESS) {
        INFO_LOG("aclInit failed, ret = %d.", ret);
        return false;
    }

    // open device
    ret = aclrtSetDevice(deviceId);
    if (ret != ACL_SUCCESS) {
        INFO_LOG("aclrtSetDevice failed, ret = %d.", ret);
        (void)aclFinalize();
        return false;
    }
    deviceId_ = deviceId;

    // create context
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_SUCCESS) {
        INFO_LOG("aclrtSetDevice failed, ret = %d.", ret);
        return false;
    }

    // get run mode, check host or device
    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        INFO_LOG("aclrtGetRunMode failed, ret = %d.", ret);
        return false;
    }

    // create fv initialize parameters
    fvInitPara_ = aclfvCreateInitPara(fsNum);
    if (fvInitPara_ == nullptr) {
        INFO_LOG("aclfvCreateInitPara failed, fsNum = %zu.", fsNum);
        return false;
    }

    // fv initialize
    ret = aclfvInit(fvInitPara_);
    if (ret != ACL_SUCCESS) {
        INFO_LOG("aclfvInit failed, ret = %d.", ret);
        (void)aclfvDestroyInitPara(fvInitPara_);
        fvInitPara_ = nullptr;
        return false;
    }

    return true;
}

void FVResource::Finalize()
{
    // destroy fv init parameter and release fv
    if (fvInitPara_ != nullptr) {
        (void)aclfvDestroyInitPara(fvInitPara_);
        fvInitPara_ = nullptr;
        (void)aclfvRelease();
    }

    // destroy context
    if (context_ != nullptr) {
        (void)aclrtDestroyContext(context_);
        context_ = nullptr;
    }

    // reset device and acl finalize
    if (deviceId_ != -1) {
        (void)aclrtResetDevice(deviceId_);
        deviceId_ = -1;
        (void)aclFinalize();
    }
}

bool FVResource::IsDevice() const
{
    return runMode_ == ACL_DEVICE;
}