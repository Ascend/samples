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
#include "dvpp_cropandpaste.h"

using namespace std;

DvppProcess::DvppProcess() : 
isInitOk_(false), stream_(nullptr),
dvppChannelDesc_(nullptr), isReleased_(false) {
}

DvppProcess::~DvppProcess() {
    DestroyResource();
}

void DvppProcess::DestroyResource() {
    if (isReleased_) {
        return;
    }

    aclError aclRet;

    if (dvppChannelDesc_ != nullptr) {
        aclRet = acldvppDestroyChannel(dvppChannelDesc_);
        if (aclRet != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("Destroy dvpp channel error: %d", aclRet);
        }

        (void)acldvppDestroyChannelDesc(dvppChannelDesc_);
        dvppChannelDesc_ = nullptr;
    }

    if (stream_ != nullptr) {
        aclRet = aclrtDestroyStream(stream_);
        if (aclRet != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("Vdec destroy stream failed, error %d", aclRet);
        }
        stream_ = nullptr;
    }

    isReleased_ = true;
}

AtlasError DvppProcess::Init() {
    aclError aclRet = aclrtCreateStream(&stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Create venc stream failed, error %d", aclRet);
        return ATLAS_ERROR_CREATE_STREAM;
    }

    dvppChannelDesc_ = acldvppCreateChannelDesc();
    if (dvppChannelDesc_ == nullptr) {
        ATLAS_LOG_ERROR("Create dvpp channel desc failed");
        return ATLAS_ERROR_CREATE_DVPP_CHANNEL_DESC;
    }

    aclRet = acldvppCreateChannel(dvppChannelDesc_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acldvppCreateChannel failed, aclRet = %d", aclRet);
        return ATLAS_ERRROR_CREATE_DVPP_CHANNEL;
    }

    isInitOk_ = true;
    ATLAS_LOG_INFO("dvpp init resource ok");
    
    return ATLAS_OK;
}

AtlasError DvppProcess::Resize(ImageData& dest, ImageData& src,
                        uint32_t width, uint32_t height) {
    DvppResize resizeOp(stream_, dvppChannelDesc_, width, height);
    return resizeOp.Process(dest, src);
}

AtlasError DvppProcess::JpegD(ImageData& dest, ImageData& src) {
    DvppJpegD jpegD(stream_, dvppChannelDesc_);
    return jpegD.Process(dest, src);
}

AtlasError DvppProcess::Crop(ImageData& dest, ImageData& src,
                             uint32_t ltHorz, uint32_t ltVert,
                             uint32_t rbHorz, uint32_t rbVert) {
    DvppCropAndPaste crop(stream_, dvppChannelDesc_, 
                          ltHorz, ltVert, rbHorz, rbVert);
    return crop.Process(dest, src);
}

AtlasError DvppProcess::JpegE(ImageData& dest, ImageData& src) {
    DvppJpegE jpegE(stream_, dvppChannelDesc_);
    return jpegE.Process(dest, src);
}

