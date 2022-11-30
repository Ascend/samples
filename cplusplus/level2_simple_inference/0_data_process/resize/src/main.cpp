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

#include <iostream>
#include <cstdlib>
#include <dirent.h>
#include "main.h"
using namespace std;

int32_t deviceId_;
aclrtContext context_;
aclrtStream stream_;

acldvppStreamDesc *streamInputDesc_;
acldvppPicDesc *picOutputDesc_;
acldvppChannelDesc *dvppChannelDesc_;
uint32_t inBufferSize;
uint32_t inputWidth;
uint32_t inputHeight;

PicDesc g_inPicDesc;
PicDesc g_outPicDesc;

/* Run managed resource applications, including Device, Context, and Stream*/
Result Initparam(int argc, char *argv[])
{
    DIR *dir;
    int paramNum = 7;
    int inputWNum = 2;
    int outputWNum = 5;
    int inputHNum =3;
    int outputHNum = 6;
    int inputPicDescNum = 1;
    int outputPicDescNum = 4;
    if ((dir = opendir("./output")) == NULL)
        system("mkdir ./output");
    if(argc != paramNum) {
        ERROR_LOG("./resize infile w h outfile w h");
        return FAILED;
    }
    int inw,inh,outw,outh;
    inw = atoi(argv[inputWNum]);
    inh = atoi(argv[inputHNum]);
    outw = atoi(argv[outputWNum]);
    outh = atoi(argv[outputHNum]);
    g_inPicDesc={argv[inputPicDescNum],inw,inh};
    g_outPicDesc={argv[outputPicDescNum],outw,outh};
    return SUCCESS;
}

uint32_t AlignmentHelper(uint32_t origSize, uint32_t alignment)
{
    if (alignment == 0) {
        return 0;
    }
    uint32_t alignmentH = alignment - 1;
    return (origSize + alignmentH) / alignment * alignment;
}

uint32_t SaveDvppOutputData(const char *fileName, const void *devPtr, uint32_t dataSize)
{
    FILE* outFileFp = fopen(fileName, "wb+");
    if (runMode == ACL_HOST) {
        void* hostPtr = nullptr;
        aclrtMallocHost(&hostPtr, dataSize);
        aclrtMemcpy(hostPtr, dataSize, devPtr, dataSize, ACL_MEMCPY_DEVICE_TO_HOST);
        fwrite(hostPtr, sizeof(char), dataSize, outFileFp);
        (void)aclrtFreeHost(hostPtr);
    } else {
        fwrite(devPtr, sizeof(char), dataSize, outFileFp);
    }
    fflush(outFileFp);
    fclose(outFileFp);
    return SUCCESS;
}

char* ReadBinFile(std::string fileName, uint32_t &fileSize)
{
    std::ifstream binFile(fileName, std::ifstream::binary);
    if (binFile.is_open() == false) {
        ERROR_LOG("open file %s failed", fileName.c_str());
        return nullptr;
    }

    binFile.seekg(0, binFile.end);
    uint32_t binFileBufferLen = binFile.tellg();
    if (binFileBufferLen == 0) {
        ERROR_LOG("binfile is empty, filename is %s", fileName.c_str());
        binFile.close();
        return nullptr;
    }

    binFile.seekg(0, binFile.beg);

    char* binFileBufferData = new(std::nothrow) char[binFileBufferLen];
    if (binFileBufferData == nullptr) {
        ERROR_LOG("malloc binFileBufferData failed");
        binFile.close();
        return nullptr;
    }
    binFile.read(binFileBufferData, binFileBufferLen);
    binFile.close();
    fileSize = binFileBufferLen;
    return binFileBufferData;
}

void DestroyResource()
{
    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy stream failed");
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy context failed");
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");
}


int main(int argc, char *argv[])
{
    /* 1.ACL initialization */
    const char *aclConfigPath = "../src/acl.json";
    aclInit(aclConfigPath);
    INFO_LOG("acl init success");

    /* 2.Run the management resource application, including Device, Context, Stream */
    aclrtSetDevice(deviceId_);
    aclrtCreateContext(&context_, deviceId_);
    aclrtCreateStream(&stream_);
    aclrtGetRunMode(&runMode);

    /* 3.Initialization parameters: width and height of the original image, crop width and height.
     Initialize folder: Output folder */
    Initparam(argc, argv);
    const int modelInputWidth = g_outPicDesc.width; // cur model shape is 224 * 224
    const int modelInputHeight = g_outPicDesc.height;

    /* 4. Channel description information when creating image data processing channels,
     dvppChannelDesc_ is acldvppChannelDesc type */
    dvppChannelDesc_ = acldvppCreateChannelDesc();

    /* 5. Create the image data processing channel. */
    acldvppCreateChannel(dvppChannelDesc_);

    // GetPicDevBuffer4JpegD
    uint32_t inputBuffSize = 0;
    char* inputBuff = ReadBinFile(g_inPicDesc.picName, inputBuffSize);
    void *inBufferDev = nullptr;
    inBufferSize = inputBuffSize;
    acldvppMalloc(&inBufferDev, inBufferSize);
    if (runMode == ACL_HOST) {
        aclrtMemcpy(inBufferDev, inBufferSize, inputBuff, inputBuffSize, ACL_MEMCPY_HOST_TO_DEVICE);
    } else {
        aclrtMemcpy(inBufferDev, inBufferSize, inputBuff, inputBuffSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
    }
    delete[] inputBuff;

    acldvppResizeConfig *resizeConfig_ = acldvppCreateResizeConfig();

    /* processdecode */
    inputWidth = g_inPicDesc.width;
    inputHeight = g_inPicDesc.height;
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;
    // if the input yuv is from JPEGD, 128*16 alignment on 310, 64*16 alignment on 310P
    // if the input yuv is from VDEC, it shoud be aligned to 16*2
    uint32_t inputWidthStride = AlignmentHelper(inputWidth, 16);
    uint32_t inputHeightStride = AlignmentHelper(inputHeight, 2);
    uint32_t inputBufferSize = inputWidthStride * inputHeightStride * sizeAlignment / sizeNum;
    acldvppPicDesc *vpcInputDesc_ = acldvppCreatePicDesc();
    acldvppPicDesc *vpcOutputDesc_ = acldvppCreatePicDesc();
    void *vpcOutBufferDev = nullptr;
    acldvppSetPicDescData(vpcInputDesc_, reinterpret_cast<char *>(inBufferDev));
    // the format is the input yuv's format.
    acldvppSetPicDescFormat(vpcInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcInputDesc_, inputWidth);
    acldvppSetPicDescHeight(vpcInputDesc_, inputHeight);
    acldvppSetPicDescWidthStride(vpcInputDesc_, inputWidthStride);
    acldvppSetPicDescHeightStride(vpcInputDesc_, inputHeightStride);
    acldvppSetPicDescSize(vpcInputDesc_, inputBufferSize);

    // here is the VPC constraints, should be aligned to 16*2
    int resizeOutWidthStride = AlignmentHelper(modelInputWidth, 16);
    int resizeOutHeightStride = AlignmentHelper(modelInputHeight, 2);
    uint32_t vpcOutBufferSize_ = resizeOutWidthStride * resizeOutHeightStride * sizeAlignment / sizeNum;
    acldvppMalloc(&vpcOutBufferDev, vpcOutBufferSize_);
    acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev);
    // the format should be under the VPC constraints(only support the following 2):
    // PIXEL_FORMAT_YUV_SEMIPLANAR_420 = 1
    // PIXEL_FORMAT_YVU_SEMIPLANAR_420 = 2
    acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcOutputDesc_, modelInputWidth);
    acldvppSetPicDescHeight(vpcOutputDesc_, modelInputHeight);
    acldvppSetPicDescWidthStride(vpcOutputDesc_, resizeOutWidthStride);
    acldvppSetPicDescHeightStride(vpcOutputDesc_, resizeOutHeightStride);
    acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);

    aclError ret = acldvppVpcResizeAsync(dvppChannelDesc_, vpcInputDesc_,
        vpcOutputDesc_, resizeConfig_, stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppVpcResizeAsync failed, ret = %d", ret);
        return FAILED;
    }
    
    aclrtSynchronizeStream(stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtSynchronizeStream failed");
        return FAILED;
    }

    /* DestroyResizeResource */
    (void)acldvppDestroyResizeConfig(resizeConfig_);
    resizeConfig_ = nullptr;
    (void)acldvppDestroyPicDesc(vpcInputDesc_);
    vpcInputDesc_ = nullptr;
    (void)acldvppDestroyPicDesc(vpcOutputDesc_);
    vpcOutputDesc_ = nullptr;

    (void)acldvppFree(inBufferDev);
    inBufferDev = nullptr;

    SaveDvppOutputData(g_outPicDesc.picName.c_str(), vpcOutBufferDev, vpcOutBufferSize_);
    if (vpcOutBufferDev != nullptr) {
        (void)acldvppFree(vpcOutBufferDev);
        vpcOutBufferDev = nullptr;
    }
    (void)acldvppDestroyChannel(dvppChannelDesc_);
    (void)acldvppDestroyChannelDesc(dvppChannelDesc_);
    dvppChannelDesc_ = nullptr;

    DestroyResource();
    return SUCCESS;
}
