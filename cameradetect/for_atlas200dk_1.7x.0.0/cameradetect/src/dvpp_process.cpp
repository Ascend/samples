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
#include "dvpp_jpege.h"
//#include "dvpp_jpegd.h"
#include "dvpp_process.h"

using namespace std;

DvppProcess::DvppProcess()
    : isInitOk_(false), dvppChannelDesc_(nullptr) {
    isGlobalContext_ = false;
    InitResource();
}

DvppProcess::DvppProcess(aclrtContext& context, aclrtStream stream)
          :isInitOk_(false), context_(context), stream_(stream), dvppChannelDesc_(nullptr){
      isGlobalContext_ = true;
      InitResource();
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
            ASC_LOG_ERROR("acldvppDestroyChannel failed, aclRet = %d", aclRet);
        }

        (void)acldvppDestroyChannelDesc(dvppChannelDesc_);
        dvppChannelDesc_ = nullptr;
    }

    if (!isGlobalContext_) {

    if (stream_ != nullptr) {
        aclRet = aclrtDestroyStream(stream_);
        if (aclRet != ACL_ERROR_NONE) {
            ASC_LOG_ERROR("destroy stream failed");
        }
        stream_ = nullptr;
    }
    ASC_LOG_INFO("end to destroy stream");

    if (context_ != nullptr) {
        aclRet = aclrtDestroyContext(context_);
        if (aclRet != ACL_ERROR_NONE) {
            ASC_LOG_ERROR("destroy context failed");
        }
        context_ = nullptr;
    }
    }
    ASC_LOG_INFO("end to destroy context");
}

int DvppProcess::InitResource()
{
     aclError aclRet;
    if (!isGlobalContext_) {
    int deviceId = 0;
    context_ = nullptr;
    aclError aclRet = aclrtCreateContext(&context_, deviceId);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("aclrtCreateContext failed, ret=%d.", aclRet);
        return STATUS_ERROR;
    }
    ASC_LOG_INFO("dvpp process create context ok");
    aclRet = aclrtCreateStream(&stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acl create stream failed");
        return STATUS_ERROR;
    }
    ASC_LOG_INFO("dvpp process create stream ok");
    }
    dvppChannelDesc_ = acldvppCreateChannelDesc();
    if (dvppChannelDesc_ == nullptr) {
        ASC_LOG_ERROR("acldvppCreateChannelDesc failed");
        return STATUS_ERROR;
    }

    aclRet = acldvppCreateChannel(dvppChannelDesc_);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acldvppCreateChannel failed, aclRet = %d", aclRet);
        return STATUS_ERROR;
    }
    isInitOk_ = true;
    ASC_LOG_INFO("dvpp init resource STATUS_OK");
    return STATUS_OK;
}


int DvppProcess::Resize(ImageData* dest, ImageData* src, Resolution& size) {
    DvppResize resizeOp(stream_, dvppChannelDesc_, size);
//    printf("dvpp process call resize, format %d\n", src->format);
    return resizeOp.Process(dest, src);
}

int DvppProcess::CvtYuv420spToJpeg(ImageData* dest, ImageData* src) {
    DvppJpegE jpegE(stream_, dvppChannelDesc_);
    return jpegE.Process(dest,src);
}


