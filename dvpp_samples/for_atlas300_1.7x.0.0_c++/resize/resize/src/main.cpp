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

* File main.cpp
* Description: dvpp sample main func
*/

#include <iostream>
#include <stdlib.h>
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

PicDesc inPicDesc;
PicDesc outPicDesc;

/* 运行管理资源申请,包括Device、Context、Stream*/
Result Initparam(int argc, char *argv[])
{
    if(argc!=7)	{
        ERROR_LOG("./resize infile w h outfile w h");
        return FAILED;
    }
    int inw,inh,outw,outh;
    inw = atoi(argv[2]);
    inh = atoi(argv[3]);
    outw = atoi(argv[5]);
    outh = atoi(argv[6]);
    inPicDesc={argv[1],inw,inh};
    outPicDesc={argv[4],outw,outh};
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
    void* hostPtr = nullptr;
    aclrtMallocHost(&hostPtr, dataSize);
    aclrtMemcpy(hostPtr, dataSize, devPtr, dataSize, ACL_MEMCPY_DEVICE_TO_HOST);
    FILE *outFileFp = fopen(fileName, "wb+");
    fwrite(hostPtr, sizeof(char), dataSize, outFileFp);
    (void)aclrtFreeHost(hostPtr);
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
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy stream failed");
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy context failed");
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");
}


int main(int argc, char *argv[])
{
    /* 1.ACL初始化 */
    const char *aclConfigPath = "./acl.json";
    aclInit(aclConfigPath);
    INFO_LOG("acl init success");

    /* 2.运行管理资源申请,包括Device、Context、Stream */
    aclrtSetDevice(deviceId_);
    INFO_LOG("open device %d success", deviceId_);
    aclrtCreateContext(&context_, deviceId_);
    INFO_LOG("create context success");
    aclrtCreateStream(&stream_);
    INFO_LOG("create stream success");

    /* 3.初始化参数：原图宽高，crop宽高。初始化文件夹：输出结果文件夹*/
    Initparam(argc, argv);
    const int modelInputWidth = outPicDesc.width; // cur model shape is 224 * 224
    const int modelInputHeight = outPicDesc.height;

	/* 4. 创建图片数据处理通道时的通道描述信息，dvppChannelDesc_是acldvppChannelDesc类型*/
    dvppChannelDesc_ = acldvppCreateChannelDesc();

    /* 5. 创建图片数据处理的通道。*/
    acldvppCreateChannel(dvppChannelDesc_);

    // GetPicDevBuffer4JpegD
    uint32_t devPicBufferSize;
    uint32_t inputHostBuffSize = 0;
    char* inputHostBuff = ReadBinFile(inPicDesc.picName, inputHostBuffSize);
    void *inBufferDev = nullptr;
    inBufferSize = inputHostBuffSize;
    acldvppMalloc(&inBufferDev, inBufferSize);
    aclrtMemcpy(inBufferDev, inBufferSize, inputHostBuff, inputHostBuffSize, ACL_MEMCPY_HOST_TO_DEVICE);
    delete[] inputHostBuff;
    devPicBufferSize = inBufferSize;
    //char *picDevBuffer = reinterpret_cast<char *>(inBufferDev);

    acldvppResizeConfig *resizeConfig_ = nullptr;
    resizeConfig_ = acldvppCreateResizeConfig();

    /* processdecode*/
    inputWidth = inPicDesc.width;
    inputHeight = inPicDesc.height;
    uint32_t widthAlignment = 16;
    uint32_t heightAlignment = 2;
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;
    uint32_t inputWidthStride = AlignmentHelper(inputWidth, widthAlignment);
    uint32_t inputHeightStride = AlignmentHelper(inputHeight, heightAlignment);
    uint32_t inputBufferSize = inputWidthStride * inputHeightStride * sizeAlignment / sizeNum;
    acldvppPicDesc *vpcInputDesc_ = nullptr;
    acldvppPicDesc *vpcOutputDesc_ = nullptr;
    void *vpcOutBufferDev_ = nullptr;
    vpcInputDesc_ = acldvppCreatePicDesc();
    acldvppSetPicDescData(vpcInputDesc_, reinterpret_cast<char *>(inBufferDev));
    acldvppSetPicDescFormat(vpcInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcInputDesc_, inputWidth);
    acldvppSetPicDescHeight(vpcInputDesc_, inputHeight);
    acldvppSetPicDescWidthStride(vpcInputDesc_, inputWidthStride);
    acldvppSetPicDescHeightStride(vpcInputDesc_, inputHeightStride);
    acldvppSetPicDescSize(vpcInputDesc_, inputBufferSize);


    int resizeOutWidthStride = AlignmentHelper(modelInputWidth, widthAlignment);
    int resizeOutHeightStride = AlignmentHelper(modelInputHeight, heightAlignment);
    uint32_t vpcOutBufferSize_ = resizeOutWidthStride * resizeOutHeightStride * sizeAlignment / sizeNum;
    acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    vpcOutputDesc_ = acldvppCreatePicDesc();
    acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcOutputDesc_, modelInputWidth);
    acldvppSetPicDescHeight(vpcOutputDesc_, modelInputHeight);
    acldvppSetPicDescWidthStride(vpcOutputDesc_, resizeOutWidthStride);
    acldvppSetPicDescHeightStride(vpcOutputDesc_, resizeOutHeightStride);
    acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);

    aclError ret = acldvppVpcResizeAsync(dvppChannelDesc_, vpcInputDesc_,
        vpcOutputDesc_, resizeConfig_, stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppVpcResizeAsync failed, ret = %d", ret);
        return FAILED;
    }
    
    aclrtSynchronizeStream(stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclrtSynchronizeStream failed");
        return FAILED;
    }

    /* DestroyResizeResource*/
    (void)acldvppDestroyResizeConfig(resizeConfig_);
    resizeConfig_ = nullptr;
    (void)acldvppDestroyPicDesc(vpcInputDesc_);
    vpcInputDesc_ = nullptr;
    (void)acldvppDestroyPicDesc(vpcOutputDesc_);
    vpcOutputDesc_ = nullptr;

    (void)acldvppFree(inBufferDev);
    inBufferDev = nullptr;

    SaveDvppOutputData(outPicDesc.picName.c_str(), vpcOutBufferDev_, vpcOutBufferSize_);
    if (vpcOutBufferDev_ != nullptr) {
        (void)acldvppFree(vpcOutBufferDev_);
        vpcOutBufferDev_ = nullptr;
    }
	(void)acldvppDestroyChannel(dvppChannelDesc_);
	(void)acldvppDestroyChannelDesc(dvppChannelDesc_);
	dvppChannelDesc_ = nullptr;

    DestroyResource();
    return SUCCESS;
}
