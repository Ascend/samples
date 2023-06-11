/**
 * ============================================================================
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <malloc.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <sys/time.h>
#include <fstream>
#include <memory>
#include <thread>
#include <iostream>
#include "AclLiteUtils.h"
#include "VideoWriter.h"

using namespace std;

VideoWriter::VideoWriter(VencConfig& vencConfig, aclrtContext context)
    :isReleased_(false), status_(STATUS_VENC_INIT), context_(context),
    dvppVenc_(nullptr), vencInfo_(vencConfig)
{
}

VideoWriter::~VideoWriter()
{
    DestroyResource();
}

void VideoWriter::DestroyResource()
{
    if (isReleased_) return;
    dvppVenc_->DestroyResource();
    // release dvpp venc
    delete dvppVenc_;
    dvppVenc_ = nullptr;
    isReleased_ = true;
}

AclLiteError VideoWriter::InitResource()
{
    aclError aclRet;
    if (context_ == nullptr) {
        aclRet = aclrtGetCurrentContext(&context_);
        if ((aclRet != ACL_SUCCESS) || (context_ == nullptr)) {
            ACLLITE_LOG_ERROR("Get current acl context error:%d", aclRet);
            return ACLLITE_ERROR_GET_ACL_CONTEXT;
        }
    }
    // Get current run mode
    aclRet = aclrtGetRunMode(&runMode_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR_GET_RUM_MODE;
    }

    dvppVenc_ = new VencHelper(vencInfo_);
    AclLiteError ret = dvppVenc_->Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Video encoder init failed");
        return ACLLITE_ERROR;
    }
    return ACLLITE_OK;
}

AclLiteError VideoWriter::Open()
{
    // Init acl resource
    AclLiteError ret = InitResource();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init resource failed");
        return ACLLITE_ERROR;
    }
    // Set init ok
    string outvideo = vencInfo_.outFile;
    ACLLITE_LOG_INFO("Encoded Video %s init ok", outvideo.c_str());
    return ACLLITE_OK;
}

// check decoder status
bool VideoWriter::IsOpened()
{
    string outvideo = vencInfo_.outFile;
    status_ = dvppVenc_->GetStatus();
    ACLLITE_LOG_INFO("Video %s encode status %d", outvideo.c_str(), status_);
    return (status_ == STATUS_VENC_INIT) || (status_ == STATUS_VENC_WORK);
}

// read encoded frame
AclLiteError VideoWriter::Read(ImageData& image)
{
    AclLiteError ret = dvppVenc_->Process(image);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("fail to read encode image");
        return ret;
    }
    return ACLLITE_OK;
}

AclLiteError VideoWriter::Set(StreamProperty key, int value)
{
    AclLiteError ret = ACLLITE_OK;
    switch (key) {
        case OUTPUT_IMAGE_FORMAT:
            ret = SetImageFormat(value);
            break;
        case STREAM_FORMAT:
            ret = SetStreamFormat(value);
            break;
        default:
            ret = ACLLITE_ERROR_UNSURPPORT_PROPERTY;
            string outvideo = vencInfo_.outFile;
            ACLLITE_LOG_ERROR("Unsurpport property %d to set for video %s",
                              (int)key, outvideo.c_str());
            break;
    }

    return ret;
}

uint32_t VideoWriter::Get(StreamProperty key)
{
    uint32_t value = 0;
    switch (key) {
        case FRAME_WIDTH:
            value = vencInfo_.maxWidth;
            break;
        case FRAME_HEIGHT:
            value = vencInfo_.maxHeight;
            break;
        default:
            ACLLITE_LOG_ERROR("Unsurpport property %d to get for video", key);
            break;
    }

    return value;
}

AclLiteError VideoWriter::SetAclContext()
{
    if (context_ == nullptr) {
        ACLLITE_LOG_ERROR("Video decoder context is null");
        return ACLLITE_ERROR_SET_ACL_CONTEXT;
    }
    aclError ret = aclrtSetCurrentContext(context_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Video decoder set context failed, error: %d", ret);
        return ACLLITE_ERROR_SET_ACL_CONTEXT;
    }
    return ACLLITE_OK;
}

AclLiteError VideoWriter::Close()
{
    DestroyResource();
    return ACLLITE_OK;
}

AclLiteError VideoWriter::SetImageFormat(uint32_t format)
{
    if ((format != PIXEL_FORMAT_YUV_SEMIPLANAR_420) ||
        (format != PIXEL_FORMAT_YVU_SEMIPLANAR_420)) {
        ACLLITE_LOG_ERROR("Set video encoded image format to %d failed, "
                          "only support %d(YUV420SP NV12) and %d(YUV420SP NV21)",
                          format,
                          (int)PIXEL_FORMAT_YUV_SEMIPLANAR_420,
                          (int)PIXEL_FORMAT_YVU_SEMIPLANAR_420);
        return ACLLITE_ERROR;
    }

    vencInfo_.format = (acldvppPixelFormat)format;
    ACLLITE_LOG_INFO("Set video encoded image format to %d ok", format);

    return ACLLITE_OK;
}

AclLiteError VideoWriter::SetStreamFormat(uint32_t format)
{
    if ((format != H265_MAIN_LEVEL) ||
        (format != H264_MAIN_LEVEL) ||
        (format != H264_HIGH_LEVEL) ||
        (format != H264_BASELINE_LEVEL)) {
        ACLLITE_LOG_ERROR("Set video stream format to %d failed, "
                          "only support "
                          "%d(H265 MP), "
                          "%d(H264 BP), "
                          "%d(H264 MP) and"
                          "%d(H264 HP)",
                          format,
                          (int)H265_MAIN_LEVEL,
                          (int)H264_BASELINE_LEVEL,
                          (int)H264_MAIN_LEVEL,
                          (int)H264_HIGH_LEVEL);
        return ACLLITE_ERROR;
    }
    vencInfo_.enType = (acldvppStreamFormat)format;
    ACLLITE_LOG_INFO("Set video stream format to %d ok", format);

    return ACLLITE_OK;
}