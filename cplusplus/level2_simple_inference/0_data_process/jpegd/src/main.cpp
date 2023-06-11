/*
* Copyright (C) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
*/
/**
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* File sample_process.cpp
* Description: handle acl resource
*/

#include <iostream>
#include <fstream>
#include <dirent.h>
#include <cstring>
#include <sys/stat.h>
#include "main.h"
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

using namespace std;

void* GetDeviceBufferOfPicture(PicDesc &picDesc, uint32_t &devPicBufferSize)
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

    uint32_t inputBuffSize = fileLen;

    char* inputBuff = new(std::nothrow) char[inputBuffSize];
    size_t readSize = fread(inputBuff, sizeof(char), inputBuffSize, fp);
    if (readSize < inputBuffSize) {
        ERROR_LOG("need read file %s %u bytes, but only %zu readed",
        picDesc.picName.c_str(), inputBuffSize, readSize);
        delete[] inputBuff;
		fclose(fp);
        return nullptr;
    }

    acldvppJpegFormat format;
    
    aclError aclRet = acldvppJpegGetImageInfoV2(inputBuff, inputBuffSize, &picDesc.width, &picDesc.height,
                                                nullptr, &format);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("get jpeg image info failed, errorCode is %d", static_cast<int32_t>(aclRet));
        delete[] inputBuff;
		fclose(fp);
        return nullptr;
    }

    INFO_LOG("get jpeg image info successed, width=%d, height=%d, format=%d, jpegDecodeSize=%d", picDesc.width, picDesc.height, format, picDesc.jpegDecodeSize);
    
    // when you run, from the output, we can see that the original format is ACL_JPEG_CSS_420, 
    // so it can be decoded as PIXEL_FORMAT_YUV_SEMIPLANAR_420 or PIXEL_FORMAT_YVU_SEMIPLANAR_420
    aclRet = acldvppJpegPredictDecSize(inputBuff, inputBuffSize, PIXEL_FORMAT_YUV_SEMIPLANAR_420, &picDesc.jpegDecodeSize);    
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("get jpeg decode size failed, errorCode is %d", static_cast<int32_t>(aclRet));
        delete[] inputBuff;
		fclose(fp);
        return nullptr;
    }    

    void *inBufferDev = nullptr;
    aclError ret = acldvppMalloc(&inBufferDev, inputBuffSize);
    if (ret !=  ACL_SUCCESS) {
        delete[] inputBuff;
        ERROR_LOG("malloc device data buffer failed, aclRet is %d", ret);
		fclose(fp);
        return nullptr;
    }

    if (runMode == ACL_HOST) {
        ret = aclrtMemcpy(inBufferDev, inputBuffSize, inputBuff, inputBuffSize, ACL_MEMCPY_HOST_TO_DEVICE);
    }
    else {
        ret = aclrtMemcpy(inBufferDev, inputBuffSize, inputBuff, inputBuffSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
    }
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("memcpy failed. Input host buffer size is %u",
        inputBuffSize);
        acldvppFree(inBufferDev);
        delete[] inputBuff;
		fclose(fp);
        return nullptr;
    }

    delete[] inputBuff;
    devPicBufferSize = inputBuffSize;
	fclose(fp);
    return inBufferDev;
}

void SetInput(void *inDevBuffer, int inDevBufferSize, int inputWidth, int inputHeight)
{
    inDevBuffer_ = inDevBuffer;
    inDevBufferSize_ = inDevBufferSize;
    inputWidth_ = inputWidth;
    inputHeight_ = inputHeight;
}

Result SaveDvppOutputData(const char *fileName, const void *devPtr, uint32_t dataSize)
{
    FILE *outFileFp = fopen(fileName, "wb+");
    if (nullptr == outFileFp) {
        ERROR_LOG("fopen out file %s failed.", fileName);
        return FAILED;
    }
    if (runMode == ACL_HOST) {
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
    }
    else {
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
    inDevBuffer_ = nullptr;
     if (decodeOutputDesc_ != nullptr) {
        acldvppDestroyPicDesc(decodeOutputDesc_);
        decodeOutputDesc_ = nullptr;
    }

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

int main(int argc, char *argv[]) {

    if((argc < 2) || (argv[1] == nullptr)){
        ERROR_LOG("Please input: ./main <image_path>");
        return FAILED;
    }
    string image_path = string(argv[1]);


    //1.acl init
    const char *aclConfigPath = "../src/acl.json";
    aclInit(aclConfigPath);
    INFO_LOG("acl init success");
    //2.create Device,Context,Stream
    aclrtSetDevice(deviceId_);
    INFO_LOG("open device %d success", deviceId_);
    // create context (set current)
    aclrtCreateContext(&context_, deviceId_);
    // create stream
    aclrtCreateStream(&stream_);
    aclrtGetRunMode(&runMode);
    INFO_LOG("create stream success");
        
    uint32_t image_width = 0;
    uint32_t image_height = 0;
    PicDesc testPic = {image_path, image_width, image_height};
    
    INFO_LOG("start to process picture:%s", testPic.picName.c_str());
    // dvpp process
    /*3.Read the picture into memory. InDevBuffer_ indicates the memory for storing the input picture, 
	inDevBufferSize indicates the memory size, please apply for the input memory in advance*/
    uint32_t devPicBufferSize;
    void *picDevBuffer = GetDeviceBufferOfPicture(testPic, devPicBufferSize);
    if (picDevBuffer == nullptr) {
        ERROR_LOG("get pic device buffer failed,index is 0");
        return FAILED;
    }
    //4.Create image data processing channel
    dvppChannelDesc_ = acldvppCreateChannelDesc();
    acldvppCreateChannel(dvppChannelDesc_);
    INFO_LOG("dvpp init resource success");


    SetInput(picDevBuffer, devPicBufferSize, testPic.width, testPic.height);

    // InitDecodeOutputDesc
    uint32_t alignWidth;
    uint32_t alignHeight;
    uint32_t decodeOutWidthStride;
    uint32_t decodeOutHeightStride;
    auto socVersion = aclrtGetSocName();
    if (strncmp(socVersion, "Ascend310P3", sizeof("Ascend310P3") - 1) == 0) {
        alignWidth = (inputWidth_ + 1) / 2 * 2; // 2-byte alignment
        alignHeight = (inputHeight_ + 1) / 2 * 2; // 2-byte alignment
        decodeOutWidthStride = (inputWidth_ + 63) / 64 * 64; // 64-byte alignment
        decodeOutHeightStride = (inputHeight_ + 15) / 16 * 16; // 16-byte alignment
    }else{
        alignWidth = inputWidth_;
        alignHeight = inputHeight_; 
        decodeOutWidthStride = (inputWidth_ + 127) / 128 * 128; // 128-byte alignment
        decodeOutHeightStride = (inputHeight_ + 15) / 16 * 16; // 16-byte alignment
    }
    // use acldvppJpegPredictDecSize to get output size.
    // uint32_t decodeOutBufferSize = decodeOutWidthStride * decodeOutHeightStride * 3 / 2; // yuv format size
    // uint32_t decodeOutBufferSize = testPic.jpegDecodeSize;
    aclError ret = acldvppMalloc(&decodeOutDevBuffer_, testPic.jpegDecodeSize);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppMalloc jpegOutBufferDev failed, ret = %d", ret);
        return FAILED;
    }

    decodeOutputDesc_ = acldvppCreatePicDesc();
    if (decodeOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc decodeOutputDesc failed");
        return FAILED;
    }

    acldvppSetPicDescData(decodeOutputDesc_, decodeOutDevBuffer_);
    // here the format shoud be same with the value you set when you get decodeOutBufferSize from
    acldvppSetPicDescFormat(decodeOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420); 
    acldvppSetPicDescWidth(decodeOutputDesc_, alignWidth);
    acldvppSetPicDescHeight(decodeOutputDesc_, alignHeight);
    acldvppSetPicDescWidthStride(decodeOutputDesc_, decodeOutWidthStride);
    acldvppSetPicDescHeightStride(decodeOutputDesc_, decodeOutHeightStride);
    acldvppSetPicDescSize(decodeOutputDesc_, testPic.jpegDecodeSize);

    ret = acldvppJpegDecodeAsync(dvppChannelDesc_, inDevBuffer_, inDevBufferSize_,
    decodeOutputDesc_, stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppJpegDecodeAsync failed, ret = %d", ret);
        return FAILED;
    }

    ret = aclrtSynchronizeStream(stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtSynchronizeStream failed");
        return FAILED;
    }

    decodeDataSize_ = acldvppGetPicDescSize(decodeOutputDesc_);

    (void)acldvppFree(picDevBuffer);
    picDevBuffer = nullptr;
   
    int dir_tail_index = image_path.find("/data");
    std::string outfile_dir = image_path.substr(0, dir_tail_index) + "/" + "out/output/";
    std::string outfile_path = outfile_dir + image_path.substr(dir_tail_index+5+1, image_path.rfind(".jpg")-dir_tail_index-5-1) 
        + "_jpegd_" + std::to_string(testPic.width) + "_" + std::to_string(testPic.height) + ".yuv";   
    INFO_LOG("outfile_path=%s", outfile_path.c_str());



    ret = SaveDvppOutputData(outfile_path.c_str(), decodeOutDevBuffer_, decodeDataSize_);
    if (ret != SUCCESS) {
        ERROR_LOG("save dvpp output data failed");
        // allow not return
    }

    DestroyResource();

    INFO_LOG("execute sample success");
    return SUCCESS;
}