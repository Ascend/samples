/**
* Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
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

#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include "main.h"
using namespace std;
namespace {
    void *g_inDevBuffer;  // decode input buffer
    void* g_encodeOutBufferDev; // encode output buffer
    int32_t g_deviceId = 0;
    aclrtContext g_context = nullptr;
    aclrtStream g_stream = nullptr;
    aclrtRunMode g_runMode;
    acldvppChannelDesc *g_dvppChannelDesc;
    acldvppJpegeConfig *g_jpegeConfig;
    acldvppPicDesc *g_encodeInputDesc; // encode input desf
    uint32_t g_inputWidth; // input pic width
    uint32_t g_inputHeight; // input pic height
    uint32_t g_inDevBufferSizeEncode; // input pic size for encode
    uint32_t g_encodeOutBufferSize;
    }

uint32_t AlignmentHelper(uint32_t origSize, uint32_t alignment)
{
    if (alignment == 0) {
        return 0;
    }
    uint32_t alignmentH = alignment - 1;
    return (origSize + alignmentH) / alignment * alignment;
}

uint32_t ComputeEncodeInputSize(int inputWidth, int inputHeight)
{
    uint32_t widthAlignment = 16;
    uint32_t heightAlignment = 2;
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;
    uint32_t encodeInWidthStride = AlignmentHelper(inputWidth, widthAlignment);
    uint32_t encodeInHeightStride = AlignmentHelper(inputHeight, heightAlignment);
    if (encodeInWidthStride == 0 || encodeInHeightStride == 0) {
        ERROR_LOG("ComputeEncodeInputSize AlignmentHelper failed");
        return FAILED;
    }
    uint32_t encodeInBufferSize =
    encodeInWidthStride * encodeInHeightStride * sizeAlignment / sizeNum;
    return encodeInBufferSize;
}

char* GetPicdevBuffer4Jpege(const PicDesc &picDesc, uint32_t &PicBufferSize)
{
    if (picDesc.picName.empty()) {
        ERROR_LOG("picture file name is empty");
        return nullptr;
    }

    FILE *fp = fopen(picDesc.picName.c_str(), "rb");
    if (fp == nullptr) {
        ERROR_LOG("open file %s failed", picDesc.picName.c_str());
        return nullptr;
    }

    fseek(fp, 0, SEEK_END);
    uint32_t fileLen = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if (fileLen < PicBufferSize) {
        ERROR_LOG("need read %u bytes but file %s only %u bytes",
            PicBufferSize, picDesc.picName.c_str(), fileLen);
        fclose(fp);
        return nullptr;
    }

    char* inputBuff = new(std::nothrow) char[PicBufferSize];

    size_t readSize = fread(inputBuff, sizeof(char), PicBufferSize, fp);
    if (readSize < PicBufferSize) {
        ERROR_LOG("need read file %s %u bytes, but only %zu readed",
            picDesc.picName.c_str(), PicBufferSize, readSize);
        delete[] inputBuff;
        fclose(fp);
        return nullptr;
    }

    void *inputDevBuff = nullptr;
    aclError aclRet = acldvppMalloc(&inputDevBuff, PicBufferSize);
    if (aclRet !=  ACL_SUCCESS) {
        delete[] inputBuff;
        ERROR_LOG("malloc device data buffer failed, aclRet is %d", aclRet);
        fclose(fp);
        return nullptr;
    }
    if (g_runMode == ACL_HOST) {
        aclRet = aclrtMemcpy(inputDevBuff, PicBufferSize, inputBuff, PicBufferSize, ACL_MEMCPY_HOST_TO_DEVICE);
    } else {
        aclRet = aclrtMemcpy(inputDevBuff, PicBufferSize, inputBuff, PicBufferSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
    }
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("memcpy from host to device failed, aclRet is %d", aclRet);
        (void)acldvppFree(inputDevBuff);
        delete[] inputBuff;
        fclose(fp);
        return nullptr;
    }

    fclose(fp);
    return reinterpret_cast<char *>(inputDevBuff);
}

void SetInput4Jpege(char &inDevBuffer, int inDevBufferSize, int inputWidth, int inputHeight)
{
    g_inDevBuffer = &inDevBuffer;
    g_inDevBufferSizeEncode = inDevBufferSize;
    g_inputWidth = inputWidth;
    g_inputHeight = inputHeight;
}

Result SaveDvppOutputData(const char *fileName, const void *devPtr, uint32_t dataSize)
{
    FILE * outFileFp = fopen(fileName, "wb+");
    if (nullptr == outFileFp) {
        ERROR_LOG("fopen out file %s failed.", fileName);
        return FAILED;
    }
    if (g_runMode == ACL_HOST) {
        void* hostPtr = nullptr;
        aclError aclRet = aclrtMallocHost(&hostPtr, dataSize);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("malloc host data buffer failed, aclRet is %d", aclRet);
            fclose(outFileFp);
            return FAILED;
        }

        aclRet = aclrtMemcpy(hostPtr, dataSize, devPtr, dataSize, ACL_MEMCPY_DEVICE_TO_HOST);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("dvpp output memcpy to host failed, aclRet is %d", aclRet);
            (void)aclrtFreeHost(hostPtr);
            fclose(outFileFp);
            return FAILED;
        }

        size_t writeSize = fwrite(hostPtr, sizeof(char), dataSize, outFileFp);
        if (writeSize != dataSize) {
            ERROR_LOG("need write %u bytes to %s, but only write %zu bytes.",
                dataSize, fileName, writeSize);
            (void)aclrtFreeHost(hostPtr);
            fclose(outFileFp);
            return FAILED;
        }
        (void)aclrtFreeHost(hostPtr);
    } else {
        size_t writeSize = fwrite(devPtr, sizeof(char), dataSize, outFileFp);
        if (writeSize != dataSize) {
            ERROR_LOG("need write %u bytes to %s, but only write %zu bytes.",
                dataSize, fileName, writeSize);

            fclose(outFileFp);
            return FAILED;
        }
    }

    fflush(outFileFp);
    fclose(outFileFp);
    return SUCCESS;
}

void DestroyResource()
{
    aclError ret;
    if (g_stream != nullptr) {
        ret = aclrtDestroyStream(g_stream);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy stream failed");
        }
        g_stream = nullptr;
    }
    INFO_LOG("End to destroy stream");

    if (g_context != nullptr) {
        ret = aclrtDestroyContext(g_context);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy context failed");
        }
        g_context = nullptr;
    }
    INFO_LOG("End to destroy context");

    ret = aclrtResetDevice(g_deviceId);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("End to reset device is %d", g_deviceId);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("End to finalize acl");
}

void DestroyEncodeResource()
{
    if (g_jpegeConfig != nullptr) {
        (void)acldvppDestroyJpegeConfig(g_jpegeConfig);
        g_jpegeConfig = nullptr;
    }
    INFO_LOG("Call acldvppDestroyJpegeConfig success");
    if (g_encodeInputDesc != nullptr) {
        (void)acldvppDestroyPicDesc(g_encodeInputDesc);
        g_encodeInputDesc = nullptr;
    }
    INFO_LOG("Call acldvppDestroyPicDesc success");
    if (g_inDevBuffer != nullptr) {
        (void)acldvppFree(g_inDevBuffer);
        g_inDevBuffer = nullptr;
    }
    INFO_LOG("Call acldvppFree success");
}

int main(int argc, char *argv[])
{
    int argcNum = 4;
    if ((argc < argcNum) || (argv[1] == nullptr)) {
        ERROR_LOG("Please input: ./main <image_path> <imageWidth> <imageHeight>");
        return FAILED;
    }
    string image_path = string(argv[1]);
    int imageWidth = stoi(argv[2]);
    int imageHeight = stoi(argv[3]);

    // ACL Init
    const char *aclConfigPath = "../src/acl.json";
    aclInit(aclConfigPath);
    INFO_LOG("Acl init success");
    // resource manage
    aclrtSetDevice(g_deviceId);
    INFO_LOG("Open device %d success", g_deviceId);
    aclrtCreateContext(&g_context, g_deviceId);
    INFO_LOG("Create context success");
    aclrtCreateStream(&g_stream);
    INFO_LOG("Create stream success");
    aclrtGetRunMode(&g_runMode);

    uint32_t encodeLevel = 100; // default optimal level (0-100)
    PicDesc testPic = {image_path, imageWidth, imageHeight};
    INFO_LOG("Start to process picture:%s", testPic.picName.c_str());
    INFO_LOG("Call JpegE");

    DIR *dir;
    if ((dir = opendir("./output")) == NULL) {
        system("mkdir ./output");
    }
        
    // create dvpp channel
    g_dvppChannelDesc = acldvppCreateChannelDesc();
    INFO_LOG("Call acldvppCreateChannelDesc success");
    acldvppCreateChannel(g_dvppChannelDesc);
    INFO_LOG("Call acldvppCreateChannel success");
    INFO_LOG("DVPP init resource success");

    uint32_t jpegInBufferSize;
    jpegInBufferSize = ComputeEncodeInputSize(testPic.width, testPic.height);
    // input mem malloc
    char* picDevBuffer = GetPicdevBuffer4Jpege(testPic, jpegInBufferSize);
    if (nullptr == picDevBuffer) {
        ERROR_LOG("get picDevBuffer failed, index is %d", 0);
        return FAILED;
    }
    SetInput4Jpege(*picDevBuffer, jpegInBufferSize, testPic.width, testPic.height);
    // create pic desc & set pic attrs
    uint32_t widthAlignment = 16;
    uint32_t heightAlignment = 2;
    uint32_t encodeInWidthStride = AlignmentHelper(g_inputWidth, widthAlignment);
    uint32_t encodeInHeightStride = AlignmentHelper(g_inputHeight, heightAlignment);
    if (encodeInWidthStride == 0 || encodeInHeightStride == 0) {
        ERROR_LOG("InitEncodeInputDesc AlignmentHelper failed");
        return FAILED;
    }
    g_encodeInputDesc = acldvppCreatePicDesc();
    INFO_LOG("Call acldvppCreatePicDesc success");
    if (g_encodeInputDesc == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc g_encodeInputDesc failed");
        return FAILED;
    }

    acldvppSetPicDescData(g_encodeInputDesc, reinterpret_cast<void *>(g_inDevBuffer));
    acldvppSetPicDescFormat(g_encodeInputDesc, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(g_encodeInputDesc, g_inputWidth);
    acldvppSetPicDescHeight(g_encodeInputDesc, g_inputHeight);
    acldvppSetPicDescWidthStride(g_encodeInputDesc, encodeInWidthStride);
    acldvppSetPicDescHeightStride(g_encodeInputDesc, encodeInHeightStride);
    acldvppSetPicDescSize(g_encodeInputDesc, g_inDevBufferSizeEncode);

    g_jpegeConfig = acldvppCreateJpegeConfig();
    INFO_LOG("Call acldvppCreateJpegeConfig success");
    acldvppSetJpegeConfigLevel(g_jpegeConfig, encodeLevel);

    // output mem malloc
    acldvppJpegPredictEncSize(g_encodeInputDesc, g_jpegeConfig, &g_encodeOutBufferSize);
    aclError aclRet = acldvppMalloc(&g_encodeOutBufferDev, g_encodeOutBufferSize);

    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("malloc encodeOutBufferDev_ failed, aclRet is %d", aclRet);
        return FAILED;
    }

    // call Asynchronous api
    aclRet = acldvppJpegEncodeAsync(g_dvppChannelDesc, g_encodeInputDesc, g_encodeOutBufferDev,
    &g_encodeOutBufferSize, g_jpegeConfig, g_stream);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acldvppJpegEncodeAsync failed, aclRet = %d", aclRet);
        return FAILED;
    }
    INFO_LOG("Call acldvppJpegEncodeAsync success");
    aclRet = aclrtSynchronizeStream(g_stream);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("encode aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return FAILED;
    }

    // malloc host mem & save pic
    int dirTailIndex = image_path.find("/data");
    std::string outfile_dir = image_path.substr(0, dirTailIndex) + "/" + "out/output/";
    std::string outfile_path = outfile_dir + image_path.substr(dirTailIndex+5+1,
        image_path.rfind(".yuv")-dirTailIndex-5-1)
           + "_jpege_" + std::to_string(g_inputWidth) + "_" + std::to_string(g_inputHeight) + ".jpg";
    INFO_LOG("outfile_path = %s", outfile_path.c_str());


    Result ret = SaveDvppOutputData(outfile_path.c_str(), g_encodeOutBufferDev, g_encodeOutBufferSize);
    if (ret != SUCCESS) {
        ERROR_LOG("save encode output data failed.");
        return FAILED;
    }
    
    DestroyEncodeResource();
    DestroyResource();
}
