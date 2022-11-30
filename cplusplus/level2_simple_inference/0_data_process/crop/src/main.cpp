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
#include <cstdlib>
#include <dirent.h>
#include "main.h"
using namespace std;

/* Run managed resource applications, including Device, Context, and Stream */
Result Initparam(int argc, char *argv[])
{
    DIR *dir;
    if ((dir = opendir("./output")) == NULL)
        system("mkdir ./output");
    if (argc != 9) {
        ERROR_LOG("./crop infile w h outfile outl outt outw outh");
        return FAILED;
    }
    int inw, inh, outl, outt, outw, outh;
    inw = atoi(argv[2]);
    inh = atoi(argv[3]);
    outl = atoi(argv[5]);
    outt = atoi(argv[6]);
    outw = atoi(argv[7]);
    outh = atoi(argv[8]);
    inPicDesc={argv[1],inw,inh};
    outPicDesc={argv[4], outl, outt, outw, outh};
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
    FILE * outFileFp = fopen(fileName, "wb+");
    if (runMode == ACL_HOST) {
        void * hostPtr = nullptr;
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
    /* 1.ACL initialization  */
    const char *aclConfigPath = "../src/acl.json";
    aclInit(aclConfigPath);

    /* 2.Run the management resource application, including Device, Context, Stream */
    aclrtSetDevice(deviceId_);
    aclrtCreateContext(&context_, deviceId_);
    aclrtCreateStream(&stream_);
    aclrtGetRunMode(&runMode);

    /* 3.Initialization parameters: width and height of the original image, crop width and height. 
     * Initialize folder: Output folder */
    Initparam(argc, argv);
    const int orimodelInputWidth = outPicDesc.width; // cur model shape is 224 * 224
    const int orimodelInputHeight = outPicDesc.height;
    const int modelInputLeft = outPicDesc.left; // cur model shape is 224 * 224
    const int modelInputTop = outPicDesc.top;

    /* 4. Channel description information when creating image data processing channels, 
     * dvppChannelDesc_ is acldvppChannelDesc type */
    dvppChannelDesc_ = acldvppCreateChannelDesc();

    /* 5. Create the image data processing channel. */
    acldvppCreateChannel(dvppChannelDesc_);

    // GetPicDevBuffer4JpegD
    int modelInputWidth = (orimodelInputWidth + 15) / 16 * 16;
    int modelInputHeight = (orimodelInputHeight + 1) / 2 * 2;
    uint32_t inputBuffSize = 0;
    char* inputBuff = ReadBinFile(inPicDesc.picName, inputBuffSize);
    void *inBufferDev = nullptr;
    inBufferSize = inputBuffSize;
    acldvppMalloc(&inBufferDev, inBufferSize);
    if (runMode == ACL_HOST) {
        aclrtMemcpy(inBufferDev, inBufferSize, inputBuff, inputBuffSize, ACL_MEMCPY_HOST_TO_DEVICE);
    } else {
        aclrtMemcpy(inBufferDev, inBufferSize, inputBuff, inputBuffSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
    }
    delete[] inputBuff;
    
    uint32_t oddNum = 1;
    uint32_t cropSizeWidth = modelInputWidth;
    uint32_t cropSizeHeight = modelInputHeight;
    uint32_t cropLeftOffset = modelInputLeft;  // must even
    uint32_t cropRightOffset = cropLeftOffset + cropSizeWidth - oddNum;  // must odd
    uint32_t cropTopOffset = modelInputTop;  // must even
    uint32_t cropBottomOffset = cropTopOffset + cropSizeHeight - oddNum;  // must odd
    acldvppRoiConfig  *cropArea_ = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset,
        cropTopOffset, cropBottomOffset);

    /* processdecode */
    inputWidth = inPicDesc.width;
    inputHeight = inPicDesc.height;
    uint32_t widthAlignment = 16;
    uint32_t heightAlignment = 2;
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;
    uint32_t decodeOutWidthStride = AlignmentHelper(inputWidth, widthAlignment);
    uint32_t decodeOutHeightStride = AlignmentHelper(inputHeight, heightAlignment);
    uint32_t decodeOutBufferSize = decodeOutWidthStride * decodeOutHeightStride * sizeAlignment / sizeNum;
    acldvppPicDesc *vpcInputDesc_ = acldvppCreatePicDesc();
    void *vpcOutBufferDev_ = nullptr;
    acldvppSetPicDescData(vpcInputDesc_, reinterpret_cast<char *>(inBufferDev));
    acldvppSetPicDescFormat(vpcInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcInputDesc_, inputWidth);
    acldvppSetPicDescHeight(vpcInputDesc_, inputHeight);
    acldvppSetPicDescWidthStride(vpcInputDesc_, decodeOutWidthStride);
    acldvppSetPicDescHeightStride(vpcInputDesc_, decodeOutHeightStride);
    acldvppSetPicDescSize(vpcInputDesc_, decodeOutBufferSize);

    uint32_t vpcOutBufferSize_ = modelInputWidth * modelInputHeight * sizeAlignment / sizeNum;
    acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    acldvppPicDesc *vpcOutputDesc_ = acldvppCreatePicDesc();
    acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcOutputDesc_, modelInputWidth);
    acldvppSetPicDescHeight(vpcOutputDesc_, modelInputHeight);
    acldvppSetPicDescWidthStride(vpcOutputDesc_, modelInputWidth);
    acldvppSetPicDescHeightStride(vpcOutputDesc_, modelInputHeight);
    acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);

    acldvppVpcCropAsync(dvppChannelDesc_, vpcInputDesc_,
        vpcOutputDesc_, cropArea_, stream_);

    aclrtSynchronizeStream(stream_);

    /* DestroycropResource */
    (void)acldvppDestroyRoiConfig(cropArea_);
    cropArea_ = nullptr;
    (void)acldvppDestroyPicDesc(vpcInputDesc_);
    vpcInputDesc_ = nullptr;
    (void)acldvppDestroyPicDesc(vpcOutputDesc_);
    vpcOutputDesc_ = nullptr;

    SaveDvppOutputData(outPicDesc.picName.c_str(), vpcOutBufferDev_, vpcOutBufferSize_);
    if (vpcOutBufferDev_ != nullptr) {
        (void)acldvppFree(vpcOutBufferDev_);
	vpcOutBufferDev_ = nullptr;
    }
    DestroyResource();
    return SUCCESS;
}
