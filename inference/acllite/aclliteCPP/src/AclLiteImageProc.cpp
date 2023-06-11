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

* File AclLiteImageProc.cpp
* Description: handle dvpp process
*/
#include <iostream>
#include "acl/acl.h"
#include "ResizeHelper.h"
#include "JpegDHelper.h"
#include "JpegEHelper.h"
#include "PngDHelper.h"
#include "AclLiteImageProc.h"
#include "CropAndPasteHelper.h"

using namespace std;

AclLiteImageProc::AclLiteImageProc():isReleased_(false), stream_(nullptr),
    dvppChannelDesc_(nullptr), isInitOk_(false)
{
}

AclLiteImageProc::~AclLiteImageProc()
{
    DestroyResource();
}

void AclLiteImageProc::DestroyResource()
{
    if (isReleased_) {
        return;
    }

    aclError aclRet;

    if (dvppChannelDesc_ != nullptr) {
        aclRet = acldvppDestroyChannel(dvppChannelDesc_);
        if (aclRet != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Destroy dvpp channel error: %d", aclRet);
        }

        (void)acldvppDestroyChannelDesc(dvppChannelDesc_);
        dvppChannelDesc_ = nullptr;
    }

    if (stream_ != nullptr) {
        aclRet = aclrtDestroyStream(stream_);
        if (aclRet != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Vdec destroy stream failed, error %d", aclRet);
        }
        stream_ = nullptr;
    }

    isReleased_ = true;
}

AclLiteError AclLiteImageProc::Init()
{
    aclError aclRet = aclrtCreateStream(&stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Create venc stream failed, error %d", aclRet);
        return ACLLITE_ERROR_CREATE_STREAM;
    }

    dvppChannelDesc_ = acldvppCreateChannelDesc();
    if (dvppChannelDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Create dvpp channel desc failed");
        return ACLLITE_ERROR_CREATE_DVPP_CHANNEL_DESC;
    }

    aclRet = acldvppCreateChannel(dvppChannelDesc_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acldvppCreateChannel failed, aclRet = %d", aclRet);
        return ACLLITE_ERRROR_CREATE_DVPP_CHANNEL;
    }

    isInitOk_ = true;
    ACLLITE_LOG_INFO("dvpp init resource ok");

    return ACLLITE_OK;
}

AclLiteError AclLiteImageProc::Resize(ImageData& dest, ImageData& src,
                                      uint32_t width, uint32_t height)
{
    ResizeHelper resizeOp(stream_, dvppChannelDesc_, width, height);
    return resizeOp.Process(dest, src);
}

AclLiteError AclLiteImageProc::JpegD(ImageData& dest, ImageData& src)
{
    JpegDHelper jpegD(stream_, dvppChannelDesc_);
    return jpegD.Process(dest, src);
}

AclLiteError AclLiteImageProc::PngD(ImageData& dest, ImageData& src)
{
    PngDHelper PngD(stream_, dvppChannelDesc_);
    return PngD.Process(dest, src);
}

// CropAndPasteHelper
AclLiteError AclLiteImageProc::Crop(ImageData& dest, ImageData& src,
                                    uint32_t ltHorz, uint32_t ltVert,
                                    uint32_t rbHorz, uint32_t rbVert)
{
    CropAndPasteHelper crop(stream_, dvppChannelDesc_,
                            ltHorz, ltVert, rbHorz, rbVert);
    return crop.Process(dest, src);
}

AclLiteError AclLiteImageProc::CropPaste(ImageData& dest, ImageData& src,
                                         uint32_t width, uint32_t height,
                                         uint32_t ltHorz, uint32_t ltVert,
                                         uint32_t rbHorz, uint32_t rbVert)
{
    CropAndPasteHelper crop(stream_, dvppChannelDesc_,
                            width, height, ltHorz,
                            ltVert, rbHorz, rbVert);
    return crop.ProcessCropPaste(dest, src);
}

AclLiteError AclLiteImageProc::ProportionPaste(ImageData& dest, ImageData& src,
                                               uint32_t ltHorz, uint32_t ltVert,
                                               uint32_t rbHorz, uint32_t rbVert)
{
    CropAndPasteHelper crop(stream_, dvppChannelDesc_,
                            ltHorz, ltVert, rbHorz, rbVert);
    return crop.ProportionProcess(dest, src);
}

AclLiteError AclLiteImageProc::ProportionPasteCenter(ImageData& dest, ImageData& src,
                                                     uint32_t width, uint32_t height)
{
    CropAndPasteHelper crop(stream_, dvppChannelDesc_,
                            0, 0, width, height);
    return crop.ProportionCenterProcess(dest, src);
}

AclLiteError AclLiteImageProc::JpegE(ImageData& dest, ImageData& src)
{
    JpegEHelper jpegE(stream_, dvppChannelDesc_);
    return jpegE.Process(dest, src);
}