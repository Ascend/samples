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

* File dvpp_process.cpp
* Description: handle dvpp process
*/

#include <iostream>
#include "acl/acl.h"
#include "dvpp_resize.h"
#include "dvpp_jpegd.h"
#include "dvpp_jpege.h"
#include "dvpp_process.h"

using namespace std;

DvppProcess::DvppProcess()
    : isInitOk_(false), dvppChannelDesc_(nullptr) {
    isGlobalContext_ = false;
}

DvppProcess::~DvppProcess()
{
    DestroyResource();
}

void DvppProcess::DestroyResource()
{
    aclError aclRet;
    if (dvppChannelDesc_ != nullptr) {
        aclRet = acldvppDestroyChannel(dvppChannelDesc_);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("acldvppDestroyChannel failed, aclRet = %d\n", aclRet);
        }

        (void)acldvppDestroyChannelDesc(dvppChannelDesc_);
        dvppChannelDesc_ = nullptr;
    }

    INFO_LOG("end to destroy context");
}

Result DvppProcess::InitResource(aclrtStream& stream)
{
    aclError aclRet;
    dvppChannelDesc_ = acldvppCreateChannelDesc();
    if (dvppChannelDesc_ == nullptr) {
        ERROR_LOG("acldvppCreateChannelDesc failed\n");
        return FAILED;
    }

    aclRet = acldvppCreateChannel(dvppChannelDesc_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppCreateChannel failed, aclRet = %d\n", aclRet);
        return FAILED;
    }
    stream_ = stream;
    isInitOk_ = true;
    INFO_LOG("dvpp init resource ok\n");
    return SUCCESS;
}

Result DvppProcess::Resize(ImageData& dest, ImageData& src,
                        uint32_t width, uint32_t height) {
    DvppResize resizeOp(stream_, dvppChannelDesc_, width, height);
    return resizeOp.Process(dest, src);
}

Result DvppProcess::CvtYuv420spToJpeg(ImageData& dest, ImageData& src) {
    DvppJpegE jpegE(stream_, dvppChannelDesc_);
    return jpegE.Process(dest,src);
}
