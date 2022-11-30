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
#include <pthread.h>
#include "acl/acl.h"
#include "utils.h"
#include "dvpp_process.h"
using namespace std;
extern FILE *outFileFp;
bool g_runFlag = true;
void *ThreadFunc(void *arg)
{
    int deviceId = 0;
    int32_t timeout = 1000;
    aclrtContext context = nullptr;
    aclrtCreateContext(&context, deviceId);
    INFO_LOG("process callback thread start ");
    while (g_runFlag) {
        (void)aclrtProcessReport(timeout);
    }
    aclrtDestroyContext(context);
    return static_cast<void*>(nullptr);
}

bool WriteToFile(FILE *outFileFp_, const void *dataDev, uint32_t dataSize)
{
    bool ret = true;
    size_t writeRet = fwrite(dataDev, 1, dataSize, outFileFp_);
    if (writeRet != dataSize) {
        ret = false;
    }
    fflush(outFileFp_);

    return ret;
}

void callback(acldvppPicDesc *input, acldvppStreamDesc *g_outputStreamDesc, void *userdata)
{
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
    void *outputDev = acldvppGetStreamDescData(g_outputStreamDesc);
    uint32_t streamDescSize = acldvppGetStreamDescSize(g_outputStreamDesc);

    bool ret;
    aclrtRunMode g_runMode;
    aclrtGetRunMode(&g_runMode);
    if (g_runMode == ACL_HOST) {
        void * hostPtr = nullptr;
        aclrtMallocHost(&hostPtr, streamDescSize);
        aclrtMemcpy(hostPtr, streamDescSize, outputDev, streamDescSize, ACL_MEMCPY_DEVICE_TO_HOST);
        ret = WriteToFile(outFileFp, hostPtr, streamDescSize);
        (void)aclrtFreeHost(hostPtr);
    } else {
        ret = WriteToFile(outFileFp, outputDev, streamDescSize);
    }

    if (!ret) {
        ERROR_LOG("write file failed.");
    }
    INFO_LOG("success to callback, stream size:%u", streamDescSize);
}

DvppProcess::DvppProcess()
    : g_vencChannelDesc_(nullptr),
      g_enType_(2),
      g_vencFrameConfig_(nullptr),
      g_vpcInputDesc_(nullptr),
      g_outputStreamDesc(nullptr),
      g_codeInputBufferDev_(nullptr)
{
}

DvppProcess::~DvppProcess()
{
}

Result DvppProcess::InitResource(aclrtStream& stream, int imgWidth, int imgHeight)
{
    g_stream_ = stream;
    aclError ret = aclrtGetRunMode(&g_runMode);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed");
        }
    int createThreadErr = pthread_create(&g_threadId_, nullptr, ThreadFunc, nullptr);
    (void)aclrtSubscribeReport(static_cast<uint64_t>(g_threadId_), g_stream_);
    int width = imgWidth;
    int height = imgHeight;
    uint32_t alignWidth = ALIGN_UP128(width);
    uint32_t alignHeight = ALIGN_UP16(height);
    if (alignWidth == 0 || alignHeight == 0) {
        ERROR_LOG("InitCodeInputDesc AlignmentHelper failed. image w %d, h %d, align w%d, h%d",
                  width, height, alignWidth, alignHeight);
        return FAILED;
    }
    // Allocate a large enough memory
    g_inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);
    ret = acldvppMalloc(&g_codeInputBufferDev_, g_inputBufferSize);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppMalloc failed");
        }
    g_format_ = static_cast<acldvppPixelFormat>(PIXEL_FORMAT_YVU_SEMIPLANAR_420);
    g_vencFrameConfig_ = aclvencCreateFrameConfig();
    aclvencSetFrameConfigForceIFrame(g_vencFrameConfig_, 0);
    if (g_vencFrameConfig_ == nullptr) {
        ERROR_LOG("Dvpp init failed for create config failed");
        return FAILED;
    }
    g_vencChannelDesc_ = aclvencCreateChannelDesc();
    if (g_vencChannelDesc_ == nullptr) {
        ERROR_LOG("aclvencCreateChannelDesc failed");
        return FAILED;
    }
    aclvencSetChannelDescThreadId(g_vencChannelDesc_, g_threadId_);
    aclvencSetChannelDescCallback(g_vencChannelDesc_, callback);
    aclvencSetChannelDescEnType(g_vencChannelDesc_, static_cast<acldvppStreamFormat>(g_enType_));
    aclvencSetChannelDescPicFormat(g_vencChannelDesc_, g_format_);
    aclvencSetChannelDescKeyFrameInterval(g_vencChannelDesc_, 1);
    aclvencSetChannelDescPicWidth(g_vencChannelDesc_, width);
    aclvencSetChannelDescPicHeight(g_vencChannelDesc_, height);
    aclvencCreateChannel(g_vencChannelDesc_);
    g_vpcInputDesc_ = acldvppCreatePicDesc();
    if (g_vpcInputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc g_vpcInputDesc_ failed");
        return FAILED;
    }
    acldvppSetPicDescFormat(g_vpcInputDesc_, g_format_);
    acldvppSetPicDescWidth(g_vpcInputDesc_, width);
    acldvppSetPicDescHeight(g_vpcInputDesc_, height);
    acldvppSetPicDescWidthStride(g_vpcInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(g_vpcInputDesc_, alignHeight);
    INFO_LOG("dvpp init resource ok");
    return SUCCESS;
}

Result DvppProcess::Venc(cv::Mat& srcImage)
{
    aclError ret;
    if (g_runMode == ACL_HOST) {
        ret = aclrtMemcpy(g_codeInputBufferDev_, g_inputBufferSize, srcImage.data,
                          g_inputBufferSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("aclrtMemcpy failed");
            }
    } else {
        ret = aclrtMemcpy(g_codeInputBufferDev_, g_inputBufferSize,
                          srcImage.data, g_inputBufferSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("aclrtMemcpy failed");
            }
    }

    acldvppSetPicDescData(g_vpcInputDesc_, g_codeInputBufferDev_);
    acldvppSetPicDescSize(g_vpcInputDesc_, g_inputBufferSize);

    ret = aclvencSendFrame(g_vencChannelDesc_, g_vpcInputDesc_,
    static_cast<void *>(g_outputStreamDesc), g_vencFrameConfig_, nullptr);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclvencSendFrame failed");
        }
    return SUCCESS;
}

void DvppProcess::DestroyResource()
{
    aclvencSetFrameConfigEos(g_vencFrameConfig_, 1);
    aclvencSetFrameConfigForceIFrame(g_vencFrameConfig_, 0);
    aclvencSendFrame(g_vencChannelDesc_, nullptr, nullptr, g_vencFrameConfig_, nullptr);

    if (g_vencFrameConfig_ != nullptr) {
        (void)aclvencDestroyFrameConfig(g_vencFrameConfig_);
        g_vencFrameConfig_ = nullptr;
    }

    if (g_vpcInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(g_vpcInputDesc_);
        g_vpcInputDesc_ = nullptr;
    }

    if (g_codeInputBufferDev_ != nullptr) {
        (void)acldvppFree(g_codeInputBufferDev_);
        g_codeInputBufferDev_ = nullptr;
    }

    if (g_outputStreamDesc != nullptr) {
        (void)acldvppDestroyStreamDesc(g_outputStreamDesc);
        g_outputStreamDesc = nullptr;
    }

    aclError aclRet;
    if (g_vencChannelDesc_ != nullptr) {
        aclRet = aclvencDestroyChannel(g_vencChannelDesc_);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("aclvencDestroyChannel failed, aclRet = %d", aclRet);
        }
        (void)aclvencDestroyChannelDesc(g_vencChannelDesc_);
        g_vencChannelDesc_ = nullptr;
    }

    (void)aclrtUnSubscribeReport(static_cast<uint64_t>(g_threadId_), g_stream_);
    g_runFlag = false;
    void *res = nullptr;
    pthread_cancel(g_threadId_);
    int joinThreadErr = pthread_join(g_threadId_, &res);
    INFO_LOG("end to destroy DvppProcess");
}