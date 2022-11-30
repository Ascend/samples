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

#include <iostream>
#include "acl/acl.h"
#include "dvpp_resize.h"
#include "dvpp_jpegd.h"
#include "dvpp_process.h"

using namespace std;

DvppProcess::DvppProcess():g_dvppChannelDesc_(nullptr)
{
}

DvppProcess::~DvppProcess()
{
    DestroyResource();
}

void DvppProcess::DestroyResource()
{
    aclError aclRet;
    if (g_dvppChannelDesc_ != nullptr) {
        aclRet = acldvppDestroyChannel(g_dvppChannelDesc_);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("acldvppDestroyChannel failed, aclRet = %d", aclRet);
        }

        (void)acldvppDestroyChannelDesc(g_dvppChannelDesc_);
        g_dvppChannelDesc_ = nullptr;
    }
}

Result DvppProcess::InitResource(aclrtStream& stream)
{
    aclError aclRet;

    g_dvppChannelDesc_ = acldvppCreateChannelDesc();
    if (g_dvppChannelDesc_ == nullptr) {
        ERROR_LOG("acldvppCreateChannelDesc failed");
        return FAILED;
    }

    aclRet = acldvppCreateChannel(g_dvppChannelDesc_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acldvppCreateChannel failed, aclRet = %d", aclRet);
        return FAILED;
    }
    g_stream_ = stream;
    INFO_LOG("dvpp init resource ok");
    return SUCCESS;
}

Result DvppProcess::Resize(ImageData& dest, ImageData& src,
                           uint32_t width, uint32_t height)
{
    DvppResize resizeOp(g_stream_, g_dvppChannelDesc_, width, height);
    return resizeOp.Process(dest, src);
}

Result DvppProcess::CvtJpegToYuv420sp(ImageData& dest, ImageData& src)
{
    DvppJpegD jpegD(g_stream_, g_dvppChannelDesc_);
    return jpegD.Process(dest, src);
}
