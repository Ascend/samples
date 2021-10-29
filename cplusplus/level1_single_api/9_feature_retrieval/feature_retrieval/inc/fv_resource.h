/**
* @file fv_resource.h
*
* Copyright (C) 2021. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#ifndef FV_RESOURCE_H
#define FV_RESOURCE_H
#include <iostream>
#include "acl/acl.h"
#include "acl/ops/acl_fv.h"

class FVResource {
public:
    FVResource() = default;
    ~FVResource();
    static FVResource &GetInstance();
    bool Initialize(size_t fsNum, int32_t deviceId);
    void Finalize();
    bool IsDevice() const;

private:
    int32_t deviceId_ = -1;
    aclrtContext context_ = nullptr;
    aclrtRunMode runMode_ = ACL_HOST;
    aclfvInitPara *fvInitPara_ = nullptr;
};
#endif //FV_RESOURCE_H
