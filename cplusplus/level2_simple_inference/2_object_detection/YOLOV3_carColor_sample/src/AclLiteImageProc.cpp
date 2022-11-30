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
#include "ResizeHelper.h"
#include "JpegDHelper.h"
#include "AclLiteImageProc.h"
#include "CropAndPasteHelper.h"

using namespace std;

AclLiteImageProc::AclLiteImageProc()
    : g_isReleased_(false),
      g_stream_(nullptr),
      g_dvppChannelDesc_(nullptr),
      g_isInitOk_(false)
{
}

AclLiteImageProc::~AclLiteImageProc()
{
    DestroyResource();
}

void AclLiteImageProc::DestroyResource()
{
    if (g_isReleased_) {
        return;
    }

    aclError aclRet;

    if (g_dvppChannelDesc_ != nullptr) {
        aclRet = acldvppDestroyChannel(g_dvppChannelDesc_);
        if (aclRet != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Destroy dvpp channel error: %d", aclRet);
        }

        (void)acldvppDestroyChannelDesc(g_dvppChannelDesc_);
        g_dvppChannelDesc_ = nullptr;
    }

    if (g_stream_ != nullptr) {
        aclRet = aclrtDestroyStream(g_stream_);
        if (aclRet != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Vdec destroy stream failed, error %d", aclRet);
        }
        g_stream_ = nullptr;
    }

    g_isReleased_ = true;
}

AclLiteError AclLiteImageProc::Init()
{
    aclError aclRet = aclrtCreateStream(&g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Create venc stream failed, error %d", aclRet);
        return ACLLITE_ERROR_CREATE_STREAM;
    }

    g_dvppChannelDesc_ = acldvppCreateChannelDesc();
    if (g_dvppChannelDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Create dvpp channel desc failed");
        return ACLLITE_ERROR_CREATE_DVPP_CHANNEL_DESC;
    }

    aclRet = acldvppCreateChannel(g_dvppChannelDesc_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acldvppCreateChannel failed, aclRet = %d", aclRet);
        return ACLLITE_ERRROR_CREATE_DVPP_CHANNEL;
    }

    g_isInitOk_ = true;
    ACLLITE_LOG_INFO("dvpp init resource ok");
    
    return ACLLITE_OK;
}

AclLiteError AclLiteImageProc::Resize(ImageData& dest, ImageData& src,
                                 uint32_t width, uint32_t height)
{
    ResizeHelper resizeOp(g_stream_, g_dvppChannelDesc_, width, height);
    return resizeOp.Process(dest, src);
}

AclLiteError AclLiteImageProc::JpegD(ImageData& dest, ImageData& src)
{
    JpegDHelper jpegD(g_stream_, g_dvppChannelDesc_);
    return jpegD.Process(dest, src);
}

AclLiteError AclLiteImageProc::Crop(ImageData& dest, ImageData& src,
                                    uint32_t ltHorz, uint32_t ltVert,
                                    uint32_t rbHorz, uint32_t rbVert)
{
    CropAndPasteHelper crop(g_stream_, g_dvppChannelDesc_,
                            ltHorz, ltVert, rbHorz, rbVert);
    return crop.Process(dest, src);
}

AclLiteError AclLiteImageProc::CropPaste(ImageData& dest, ImageData& src,
                                         uint32_t width, uint32_t height,
                                         uint32_t ltHorz, uint32_t ltVert,
                                         uint32_t rbHorz, uint32_t rbVert)
{
    CropAndPasteHelper crop(g_stream_, g_dvppChannelDesc_,
                            width, height, ltHorz,
                            ltVert, rbHorz, rbVert);
    return crop.ProcessCropPaste(dest, src);
}