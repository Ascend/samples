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

* File AclLiteImageProc.cpp
* Description: handle dvpp process
*/
#include <iostream>
#include "acl/acl.h"
#include "ResizeHelper.h"
#include "AclLiteImageProc.h"
#include "CropAndPasteHelper.h"

using namespace std;

AclLiteImageProc::AclLiteImageProc():
  isReleased_(false),
  stream_(nullptr),
  dvppChannelDesc_(nullptr),
  isInitOk_(false) {
}

AclLiteImageProc::~AclLiteImageProc() {
    DestroyResource();
}

void AclLiteImageProc::DestroyResource() {
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

AclLiteError AclLiteImageProc::Init() {
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
                                 uint32_t width, uint32_t height) {
    ResizeHelper resizeOp(stream_, dvppChannelDesc_, width, height);
    return resizeOp.Process(dest, src);
}

//CropAndPasteHelper
AclLiteError AclLiteImageProc::Crop(ImageData& dest, ImageData& src,
                                    uint32_t ltHorz, uint32_t ltVert,
                                    uint32_t rbHorz, uint32_t rbVert) {
    CropAndPasteHelper crop(stream_, dvppChannelDesc_, 
                            ltHorz, ltVert, rbHorz, rbVert);
    return crop.Process(dest, src);
}

AclLiteError AclLiteImageProc::CropResolution(ImageData& dest, ImageData& src,
                                              uint32_t width, uint32_t height) {
    CropAndPasteHelper crop(stream_, dvppChannelDesc_, 
                            width, height);
    return crop.ProcessResolution(dest, src);
}