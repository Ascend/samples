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

* File main.cpp
* Description: dvpp sample main func
*/
#include <cstdint>
#include <iostream>
#include <cstdlib>
#include <dirent.h>
#include <unistd.h>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "main.h"

using namespace std;
namespace {
    int32_t deviceId;
    aclrtContext context;
    aclrtStream stream;
    aclrtRunMode runMode;
    pthread_t threadId;

    void *g_inBufferDev = nullptr;
    uint32_t inBufferSize = 0;
    aclvencChannelDesc *vencChannelDesc = nullptr;
    acldvppPicDesc *vpcInputDesc = nullptr;
    aclvencFrameConfig *vencFrameConfig = nullptr;
    acldvppStreamDesc *outputStreamDesc = nullptr;
    void *g_codeInputBufferDev = nullptr;
    acldvppPixelFormat format;
    int32_t enType = 2;
    FILE *outFileFp;
    bool g_runFlag= true;
    int g_rcMode = 2;
    int g_maxBitRate = 10000;
}

Result ReadBinFile(const string& fileName, void*& data, uint32_t& size)
{
    struct stat sBuf;
    int fileStatus = stat(fileName.data(), &sBuf);
    if (fileStatus == -1) {
        ERROR_LOG("failed to get file");
        return FAILED;
    }
    if (S_ISREG(sBuf.st_mode) == 0) {
        ERROR_LOG("%s is not a file, please enter a file",
                  fileName.c_str());
        return FAILED;
    }
    std::ifstream binFile(fileName, std::ifstream::binary);
    if (binFile.is_open() == false) {
        ERROR_LOG("open file %s failed", fileName.c_str());
        return FAILED;
    }

    binFile.seekg(0, binFile.end);
    uint32_t binFileBufferLen = binFile.tellg();
    if (binFileBufferLen == 0) {
        ERROR_LOG("binfile is empty, filename is %s", fileName.c_str());
        binFile.close();
        return FAILED;
    }

    binFile.seekg(0, binFile.beg);

    uint8_t* binFileBufferData = new(std::nothrow) uint8_t[binFileBufferLen];
    if (binFileBufferData == nullptr) {
        ERROR_LOG("malloc binFileBufferData failed");
        binFile.close();
        return FAILED;
    }
    binFile.read((char *)binFileBufferData, binFileBufferLen);
    binFile.close();

    data = binFileBufferData;
    size = binFileBufferLen;
    return SUCCESS;
}

void *ThreadFunc(aclrtContext sharedContext)
{
    if (sharedContext == nullptr) {
        ERROR_LOG("sharedContext can not be nullptr");
        return ((void*)(-1));
    }
    INFO_LOG("use shared context for this thread");
    aclError ret = aclrtSetCurrentContext(sharedContext);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtSetCurrentContext failed, errorCode = %d", static_cast<int32_t>(ret));
        return ((void*)(-1));
    }
    INFO_LOG("process callback thread start ");
    while (g_runFlag) {
        // Notice: timeout 1000ms
        (void)aclrtProcessReport(1000);
    }
    return nullptr;
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
    bool ret = true;

    if (runMode == ACL_HOST) {
        void *hostPtr = nullptr;
        aclrtMallocHost(&hostPtr, streamDescSize);
        aclrtMemcpy(hostPtr, streamDescSize, outputDev, streamDescSize, ACL_MEMCPY_DEVICE_TO_HOST);
        ret = WriteToFile(outFileFp, hostPtr, streamDescSize);
        (void)aclrtFreeHost(hostPtr);
    } else{
        ret = WriteToFile(outFileFp, outputDev, streamDescSize);
    }

    if (!ret) {
        ERROR_LOG("write file failed.");
    }
    INFO_LOG("success to callback, stream size:%u", streamDescSize);
}

Result InitResource()
{
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("Acl init failed");
        return FAILED;
    }
    INFO_LOG("Acl init success");

    ret = aclrtSetDevice(deviceId);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("Acl open device %d failed", deviceId);
        return FAILED;
    }
    INFO_LOG("Open device %d success", deviceId);

    ret = aclrtCreateContext(&context, deviceId);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl create context failed");
        return FAILED;
    }
    INFO_LOG("create context success");

    // Gets whether the current application is running on host or Device
    ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }
}

Result Init(int imgWidth, int imgHeight)
{
    InitResource();

    pthread_create(&threadId, nullptr, ThreadFunc, context);
    int width = imgWidth;
    int height = imgHeight;
    uint32_t alignWidth = ALIGN_UP16(width);
    uint32_t alignHeight = ALIGN_UP16(height);
    if (alignWidth == 0 || alignHeight == 0) {
        ERROR_LOG("InitCodeInputDesc AlignmentHelper failed. image w %d, h %d, align w%u, h%u",
                  width, height, alignWidth, alignHeight);
        return FAILED;
    }
    // Allocate a large enough memory
    uint32_t inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);
    aclError ret = acldvppMalloc(&g_codeInputBufferDev, inputBufferSize);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("Dvpp malloc InputBufferDev failed");
        return FAILED;
    }

    format = static_cast<acldvppPixelFormat>(PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    vencFrameConfig = aclvencCreateFrameConfig();
    aclvencSetFrameConfigForceIFrame(vencFrameConfig, 0);

    vencChannelDesc = aclvencCreateChannelDesc();
    if (vencChannelDesc == nullptr) {
        ERROR_LOG("aclvencCreateChannelDesc failed");
        return FAILED;
    }
    aclvencSetChannelDescThreadId(vencChannelDesc, threadId);
    aclvencSetChannelDescCallback(vencChannelDesc, callback);
    aclvencSetChannelDescEnType(vencChannelDesc, static_cast<acldvppStreamFormat>(enType));
    aclvencSetChannelDescPicFormat(vencChannelDesc, format);
    aclvencSetChannelDescKeyFrameInterval(vencChannelDesc, 1);
    aclvencSetChannelDescPicWidth(vencChannelDesc, width);
    aclvencSetChannelDescPicHeight(vencChannelDesc, height);
    aclvencSetChannelDescRcMode(vencChannelDesc, g_rcMode);
    aclvencSetChannelDescMaxBitRate(vencChannelDesc, g_maxBitRate);
    aclvencCreateChannel(vencChannelDesc);

    vpcInputDesc = acldvppCreatePicDesc();
    if (vpcInputDesc == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcInputDesc failed");
        return FAILED;
    }
    acldvppSetPicDescFormat(vpcInputDesc, format);
    acldvppSetPicDescWidth(vpcInputDesc, width);
    acldvppSetPicDescHeight(vpcInputDesc, height);
    acldvppSetPicDescWidthStride(vpcInputDesc, alignWidth);
    acldvppSetPicDescHeightStride(vpcInputDesc, alignHeight);
    INFO_LOG("dvpp init resource ok");
    return SUCCESS;
}

Result Process()
{
    aclError ret;
    if (runMode == ACL_HOST) {
        ret = aclrtMemcpy(g_codeInputBufferDev, inBufferSize,
        g_inBufferDev, inBufferSize, ACL_MEMCPY_HOST_TO_DEVICE);
    } else {
        ret = aclrtMemcpy(g_codeInputBufferDev, inBufferSize,
        g_inBufferDev, inBufferSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
    }

    acldvppSetPicDescData(vpcInputDesc, g_codeInputBufferDev);
    acldvppSetPicDescSize(vpcInputDesc, inBufferSize);

    aclvencSendFrame(vencChannelDesc, vpcInputDesc,
                     static_cast<void *>(outputStreamDesc), vencFrameConfig, nullptr);
    (void)acldvppFree(g_codeInputBufferDev);
    return SUCCESS;
}

void DestroyAclResource()
{
    aclError ret;
    if (context != nullptr) {
        ret = aclrtDestroyContext(context);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy context failed");
        }
        context = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("end to reset device is %d", deviceId);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");
}

void DestroyResource()
{
    aclvencSetFrameConfigEos(vencFrameConfig, 1);
    aclvencSetFrameConfigForceIFrame(vencFrameConfig, 0);
    aclvencSendFrame(vencChannelDesc, nullptr, nullptr, vencFrameConfig, nullptr);

    if (vencFrameConfig != nullptr) {
        (void)aclvencDestroyFrameConfig(vencFrameConfig);
        vencFrameConfig = nullptr;
    }

    if (vpcInputDesc != nullptr) {
        (void)acldvppDestroyPicDesc(vpcInputDesc);
        vpcInputDesc = nullptr;
    }

    if (g_codeInputBufferDev != nullptr) {
        (void)acldvppFree(g_codeInputBufferDev);
        g_codeInputBufferDev = nullptr;
    }

    if (outputStreamDesc != nullptr) {
        (void)acldvppDestroyStreamDesc(outputStreamDesc);
        outputStreamDesc = nullptr;
    }

    aclError aclRet;
    if (vencChannelDesc != nullptr) {
        aclRet = aclvencDestroyChannel(vencChannelDesc);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("aclvencDestroyChannel failed, aclRet = %d", aclRet);
        }
        (void)aclvencDestroyChannelDesc(vencChannelDesc);
        vencChannelDesc = nullptr;
    }

    g_runFlag = false;
    void *res = nullptr;
    pthread_cancel(threadId);
    pthread_join(threadId, &res);
    fclose(outFileFp);
    DestroyAclResource();
    INFO_LOG("end to destroy Resource");
}

int main(int argc, char *argv[])
{
    int paramNum = 2;
    int paramInputPath = 1;
    // Check the input when the application executes, which takes the path to the input video file
    if((argc < paramNum) || (argv[paramInputPath] == nullptr)){
        ERROR_LOG("Please input: ./main <image_dir>");
        return FAILED;
    }

    string fileName = string(argv[paramInputPath]);
    printf("open %s\n", fileName.c_str());

    if (ReadBinFile(fileName, g_inBufferDev, inBufferSize)) {
        ERROR_LOG("read file %s failed.\n", fileName);
        return FAILED;
    }

    int imgWidth = 1920;
    int imgHeight = 1080;
    Init(imgWidth, imgHeight);

    // open the output file
    string outputPath = "./output/out_video.h264";
    outFileFp = fopen(outputPath.c_str(), "wb+");
    if (outFileFp == nullptr) {
        ERROR_LOG("Failed to open  file %s.", outputPath.c_str());
        return FAILED;
    }
    int framNum = 100;
    for (int i = 0; i < framNum; i++) {
        Process();
    }

    DestroyResource();
    INFO_LOG("Execute video object detection success");
    return SUCCESS;
}
