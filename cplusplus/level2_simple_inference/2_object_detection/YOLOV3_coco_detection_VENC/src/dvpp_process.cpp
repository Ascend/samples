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
#include "utils.h"
#include <pthread.h>
#include "dvpp_process.h"
using namespace std;
extern FILE *outFileFp;
bool runFlag_= true;
void *ThreadFunc(void *arg)
{
    int deviceId = 0;
    aclrtContext context = nullptr;
    aclrtCreateContext(&context, deviceId);
    INFO_LOG("process callback thread start ");
    while (runFlag_) {
        (void)aclrtProcessReport(1000);
    }
    aclrtDestroyContext(context);
    return (void*)0;
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

void callback(acldvppPicDesc *input, acldvppStreamDesc *outputStreamDesc, void *userdata)
{
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
    void *outputDev = acldvppGetStreamDescData(outputStreamDesc);
    uint32_t streamDescSize = acldvppGetStreamDescSize(outputStreamDesc);

    bool ret;
    aclrtRunMode runMode;
    aclrtGetRunMode(&runMode);
    if (runMode == ACL_HOST) {
        void * hostPtr = nullptr;
        aclrtMallocHost(&hostPtr, streamDescSize);
        aclrtMemcpy(hostPtr, streamDescSize, outputDev, streamDescSize, ACL_MEMCPY_DEVICE_TO_HOST);
        ret = WriteToFile(outFileFp, hostPtr, streamDescSize);
        (void)aclrtFreeHost(hostPtr);
    }
    else{
        ret = WriteToFile(outFileFp, outputDev, streamDescSize);
    }

    if (!ret) {
        ERROR_LOG("write file failed.");
    }
    INFO_LOG("success to callback, stream size:%u", streamDescSize);
}

DvppProcess::DvppProcess()
:vencChannelDesc_(nullptr), enType_(2),vencFrameConfig_(nullptr),
vpcInputDesc_(nullptr), outputStreamDesc(nullptr), codeInputBufferDev_(nullptr){
}

DvppProcess::~DvppProcess(){
    //DestroyResource();
}

Result DvppProcess::InitResource(aclrtStream& stream, int imgWidth, int imgHeight){
    stream_ = stream;
    aclError ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
        }
    int createThreadErr = pthread_create(&threadId_, nullptr, ThreadFunc, nullptr);
    (void)aclrtSubscribeReport(static_cast<uint64_t>(threadId_), stream_);
    int width = imgWidth;
    int height = imgHeight;
    uint32_t alignWidth = ALIGN_UP128(width);
    uint32_t alignHeight = ALIGN_UP16(height);
    if (alignWidth == 0 || alignHeight == 0) {
        ERROR_LOG("InitCodeInputDesc AlignmentHelper failed. image w %d, h %d, align w%d, h%d",
        width, height, alignWidth, alignHeight);
        return FAILED;
    }
    //Allocate a large enough memory
    inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);
    ret = acldvppMalloc(&codeInputBufferDev_, inputBufferSize);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppMalloc failed");
        }

    format_ = static_cast<acldvppPixelFormat>(PIXEL_FORMAT_YVU_SEMIPLANAR_420);
    vencFrameConfig_ = aclvencCreateFrameConfig();
    aclvencSetFrameConfigForceIFrame(vencFrameConfig_, 0);
    if (vencFrameConfig_ == nullptr) {
        ERROR_LOG("Dvpp init failed for create config failed");
        return FAILED;
    }

    vencChannelDesc_ = aclvencCreateChannelDesc();
    if (vencChannelDesc_ == nullptr) {
        ERROR_LOG("aclvencCreateChannelDesc failed");
        return FAILED;
    }
    aclvencSetChannelDescThreadId(vencChannelDesc_, threadId_);
    aclvencSetChannelDescCallback(vencChannelDesc_, callback);
    aclvencSetChannelDescEnType(vencChannelDesc_, static_cast<acldvppStreamFormat>(enType_));
    aclvencSetChannelDescPicFormat(vencChannelDesc_, format_);
    aclvencSetChannelDescKeyFrameInterval(vencChannelDesc_, 1);
    aclvencSetChannelDescPicWidth(vencChannelDesc_, width);
    aclvencSetChannelDescPicHeight(vencChannelDesc_, height);
    aclvencCreateChannel(vencChannelDesc_);

    vpcInputDesc_ = acldvppCreatePicDesc();
    if (vpcInputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcInputDesc_ failed");
        return FAILED;
    }
    acldvppSetPicDescFormat(vpcInputDesc_, format_);
    acldvppSetPicDescWidth(vpcInputDesc_, width);
    acldvppSetPicDescHeight(vpcInputDesc_, height);
    acldvppSetPicDescWidthStride(vpcInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(vpcInputDesc_, alignHeight);
    INFO_LOG("dvpp init resource ok");
    return SUCCESS;
}

Result DvppProcess::Venc(cv::Mat& srcImage) {
    aclError ret;
    if(runMode == ACL_HOST) {
        ret = aclrtMemcpy(codeInputBufferDev_, inputBufferSize, srcImage.data, inputBufferSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
            //ERROR_LOG("aclrtMemcpy failed");
            }
    }
    else {
        ret = aclrtMemcpy(codeInputBufferDev_, inputBufferSize, srcImage.data, inputBufferSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
            //ERROR_LOG("aclrtMemcpy failed");
            }
    }

    acldvppSetPicDescData(vpcInputDesc_, codeInputBufferDev_);
    acldvppSetPicDescSize(vpcInputDesc_, inputBufferSize);

    ret = aclvencSendFrame(vencChannelDesc_, vpcInputDesc_,
    static_cast<void *>(outputStreamDesc), vencFrameConfig_, nullptr);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclvencSendFrame failed");
        }
    return SUCCESS;
}

void DvppProcess::DestroyResource(){

    aclvencSetFrameConfigEos(vencFrameConfig_, 1);
    aclvencSetFrameConfigForceIFrame(vencFrameConfig_, 0);
    aclvencSendFrame(vencChannelDesc_, nullptr, nullptr, vencFrameConfig_, nullptr);

    if (vencFrameConfig_ != nullptr) {
        (void)aclvencDestroyFrameConfig(vencFrameConfig_);
        vencFrameConfig_ = nullptr;
    }

    if (vpcInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(vpcInputDesc_);
        vpcInputDesc_ = nullptr;
    }

    if (codeInputBufferDev_ != nullptr) {
        (void)acldvppFree(codeInputBufferDev_);
        codeInputBufferDev_ = nullptr;
    }

    if (outputStreamDesc != nullptr) {
        (void)acldvppDestroyStreamDesc(outputStreamDesc);
        outputStreamDesc = nullptr;
    }

    aclError aclRet;
    if (vencChannelDesc_ != nullptr) {
        aclRet = aclvencDestroyChannel(vencChannelDesc_);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("aclvencDestroyChannel failed, aclRet = %d", aclRet);
        }
        (void)aclvencDestroyChannelDesc(vencChannelDesc_);
        vencChannelDesc_ = nullptr;
    }

    (void)aclrtUnSubscribeReport(static_cast<uint64_t>(threadId_), stream_);
    runFlag_ = false;
    void *res = nullptr;
    pthread_cancel(threadId_);
    int joinThreadErr = pthread_join(threadId_, &res);
    INFO_LOG("end to destroy DvppProcess");
}