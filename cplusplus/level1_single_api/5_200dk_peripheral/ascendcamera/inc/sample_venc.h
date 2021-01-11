/**
* @file utils.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#pragma once
#include <iostream>
#include <unistd.h>
#include <dirent.h>
#include <fstream>
#include <cstring>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <map>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include <cstdint>


typedef struct PicDesc {
    std::string picName;
    int width;
    int height;
}PicDesc;

const uint32_t inputWidth_ = 704;
const uint32_t inputHeight_ = 576;
const uint32_t cameraFps_ = 15;

int32_t vencflag = 0;

int32_t deviceId_ = 0;
aclrtContext context_;
aclrtStream stream_;
pthread_t threadId_;
FILE *outFileFp_;

acldvppPixelFormat format_ = PIXEL_FORMAT_YUV_SEMIPLANAR_420; // 1：YUV420 semi-planner（nv12）; 2：YVU420 semi-planner（nv21）

/* 0：H265 main level
 * 1：H264 baseline level
 * 2：H264 main level
 * 3：H264 high level
 */
int32_t enType_ = 0;

aclvencChannelDesc *vencChannelDesc_;
aclvencFrameConfig *vencFrameConfig_;
acldvppPicDesc *encodeInputDesc_;
static bool runFlag = true;



uint32_t AlignmentHelper(uint32_t origSize, uint32_t alignment)
{
    if (alignment == 0) {
        return 0;
    }
    uint32_t alignmentH = alignment - 1;
    return (origSize + alignmentH) / alignment * alignment;
}

string getTime()
{
    time_t timep;
    time (& timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y%m%d%H%M%S", localtime(&timep));
    return tmp;
}

bool ReadFileToDeviceMem(const char *fileName, void *&dataDev, uint32_t &dataSize)
{
    // read data from file.
    FILE *fp = fopen(fileName, "rb+");
    if(fp == nullptr)
    {
        ERROR_LOG("Failed to open  file %s.", fileName);
        return FAILED;
    }

    fseek(fp, 0, SEEK_END);
    long fileLenLong = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    auto fileLen = static_cast<uint32_t>(fileLenLong);

    dataSize = fileLen;
    // Malloc input device memory
    auto aclRet = acldvppMalloc(&dataDev, dataSize);

    size_t readSize = fread(dataDev, 1, fileLen, fp);
    if (readSize < fileLen) {
        free(dataDev);
        return false;
    }

    free(dataDev);
    return true;
}

void *ThreadFunc(void *arg)
{
    // Notice: create context for this thread
    int deviceId = 0;
    aclrtContext context = nullptr;
    aclError ret = aclrtCreateContext(&context, deviceId);
    while (runFlag) {
        // Notice: timeout 1000ms
        aclError aclRet = aclrtProcessReport(1000);
    }

    ret = aclrtDestroyContext(context);
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

//3.创建回调函数
void callback(acldvppPicDesc *input, acldvppStreamDesc *output, void *userdata)
{
    //获取VENC编码的输出内存，调用自定义函数WriteToFile将输出内存中的数据写入文件
    void *vdecOutBufferDev = acldvppGetStreamDescData(output);
    uint32_t size = acldvppGetStreamDescSize(output);

    if (!WriteToFile(outFileFp_, vdecOutBufferDev, size)) {
        ERROR_LOG("write file failed.");
    }

    INFO_LOG("Write a picture size %d ", size);
}

bool setupVencDesc(int inputWidth, int inputHeight)
{
    aclError ret;
    //4.创建视频码流处理通道时的通道描述信息，设置视频处理通道描述信息的属性，其中callback回调函数需要用户提前创建。
    //vencChannelDesc_是aclvencChannelDesc类型
    vencChannelDesc_ = aclvencCreateChannelDesc();

    ret = aclvencSetChannelDescThreadId(vencChannelDesc_, threadId_);
    /* 设置回调函数 callback*/
    ret = aclvencSetChannelDescCallback(vencChannelDesc_, callback);

    //示例中使用的是H265_MAIN_LEVEL视频编码协议
    ret = aclvencSetChannelDescEnType(vencChannelDesc_, static_cast<acldvppStreamFormat>(enType_));
    //示例中使用的是PIXEL_FORMAT_YVU_SEMIPLANAR_420
    ret = aclvencSetChannelDescPicFormat(vencChannelDesc_, format_);
    ret = aclvencSetChannelDescPicWidth(vencChannelDesc_, inputWidth);
    ret = aclvencSetChannelDescPicHeight(vencChannelDesc_, inputHeight);
    ret = aclvencSetChannelDescKeyFrameInterval(vencChannelDesc_, 1);

    /* 5.创建视频码流处理的通道 */
    ret = aclvencCreateChannel(vencChannelDesc_);

    //6. 创建编码输入图片的描述信息，并设置各属性值
    //encodeInputDesc_是acldvppPicDesc类型
    uint32_t widthAlignment = 16;
    uint32_t heightAlignment = 2;
    uint32_t encodeInHeightStride = AlignmentHelper(inputHeight, heightAlignment);
    uint32_t encodeInWidthStride = AlignmentHelper(inputWidth, widthAlignment);
    if (encodeInWidthStride == 0 || encodeInHeightStride == 0) {
        ERROR_LOG("InitEncodeInputDesc AlignmentHelper failed");
        return FAILED;
    }

    encodeInputDesc_ = acldvppCreatePicDesc();
    if (encodeInputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc encodeInputDesc_ failed");
        return FAILED;
    }

    acldvppSetPicDescFormat(encodeInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(encodeInputDesc_, inputWidth);
    acldvppSetPicDescHeight(encodeInputDesc_, inputHeight);
    acldvppSetPicDescWidthStride(encodeInputDesc_, encodeInWidthStride);
    acldvppSetPicDescHeightStride(encodeInputDesc_, encodeInHeightStride);

    //创建aclvencFrameConfig类型的数据，创建VENC编码时的单帧编码配置参数。
    vencFrameConfig_ = aclvencCreateFrameConfig();
    aclvencSetFrameConfigForceIFrame(vencFrameConfig_, 0);

    return SUCCESS;
}