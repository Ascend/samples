/**
* @file video_encode.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "video_encode.h"
#include <iostream>
#include "venc_process.h"
#include "acl/acl.h"
#include "utils.h"
using namespace std;
static bool runFlag = true;

VideoEncode::VideoEncode() : threadId_(0), outFolder_((char*)"output/"),
    format_(PIXEL_FORMAT_YUV_SEMIPLANAR_420), enType_(H264_MAIN_LEVEL)
{
}

VideoEncode::~VideoEncode()
{
    //DestroyResource();
}

void *ThreadFunc(void *arg)
{
     // Notice: create context for this thread
    int deviceId = 0;
    aclrtContext context = nullptr;
    aclError ret = aclrtCreateContext(&context, deviceId);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclrtCreateContext failed, ret=%d.", ret);
        return ((void*)(-1));
    }

    INFO_LOG("process callback thread start ");
    while (runFlag) {
        // Notice: timeout 1000ms
        (void)aclrtProcessReport(1000);
    }

    ret = aclrtDestroyContext(context);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclrtDestroyContext failed, ret=%d.", ret);
    }

    return (void*)0;
}

Result VideoEncode::InitResource(uint32_t width, uint32_t height)
{
    // get run mode
    aclrtRunMode runMode;
    aclError ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    isDivece = (runMode == ACL_DEVICE);
    INFO_LOG("get run mode success");

    // create process callback thread
    int createThreadErr = pthread_create(&threadId_, nullptr, ThreadFunc, nullptr);
    if (createThreadErr != 0) {
        ERROR_LOG("create thread failed, err = %d", createThreadErr);
        return FAILED;
    }
    INFO_LOG("create process callback thread successfully, threadId = %lu", threadId_);

    // check output folder
    ret = Utils::CheckFolder(outFolder_);
    if (ret != SUCCESS) {
        ERROR_LOG("mkdir out folder error.");
        return FAILED;
    }

    // dvpp init
    ret = processVenc_.InitResource(threadId_, format_, enType_, width, height);
    if (ret != SUCCESS) {
        ERROR_LOG("init dvpp resource failed");
        return FAILED;
    }

    return SUCCESS;
}

bool VideoEncode::ReadImageToDeviceMem(ImageData& image, void *&dataDev, uint32_t &dataSize)
{
    dataSize = image.size;
    // Malloc input device memory
    auto aclRet = acldvppMalloc(&dataDev, dataSize);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acl malloc dvpp data failed, dataSize=%u, ret=%d.\n", dataSize, aclRet);
        return false;
    }

    if (!(isDivece)) {
        // copy input to device memory
        aclRet = aclrtMemcpy(dataDev, dataSize, image.data.get(), image.size, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("acl memcpy data to dev failed, image.size=%u, ret=%d.\n", image.size, aclRet);
            (void)acldvppFree(dataDev);
            dataDev = nullptr;
            return false;
        }
    } else {
        aclRet = aclrtMemcpy(dataDev, dataSize, image.data.get(), image.size, ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("acl memcpy data to dev failed, image.size=%u, ret=%d.\n", image.size, aclRet);
            (void)acldvppFree(dataDev);
            dataDev = nullptr;
            return false;
        }
    }

    return true;
}

//传图片过来进行编码
Result VideoEncode::DoVencProcess(ImageData& image)
{
    //create input
    void *inBufferDev = nullptr;
    uint32_t inBufferSize = 0;

    if (!ReadImageToDeviceMem(image, inBufferDev, inBufferSize)) {
        ERROR_LOG("read image to device mem failed.\n");
        return FAILED;
    }
    processVenc_.SetInput(inBufferDev, inBufferSize);
    // venc process
    aclError ret = processVenc_.Process();
    if (ret != SUCCESS) {
        ERROR_LOG("dvpp processVenc_ failed");
        return FAILED;
    }

    return SUCCESS;
}

void VideoEncode::DestroyResource()
{
    processVenc_.DestroyResource();
    aclError ret;
    // destory thread
    runFlag = false;
    void *res = nullptr;
    int joinThreadErr = pthread_join(threadId_, &res);
    if (joinThreadErr != 0) {
        ERROR_LOG("join thread failed, threadId = %lu, err = %d", threadId_, joinThreadErr);
    } else {
        if ((uint64_t)res != 0) {
            ERROR_LOG("thread run failed. ret is %lu.", (uint64_t)res);
        }
    }
    INFO_LOG("destory process callback thread success");

}





