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

uint32_t SaveOutputFile(const char *fileName, const void *devPtr, uint32_t dataSize)
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

char* ReadInputFile(std::string fileName, uint32_t &fileSize)
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

int main(int argc, char *argv[])
{
    /* 1.ACL初始化 */
    const char *aclConfigPath = "../acl.json";
    aclInit(aclConfigPath);

    /* 2.运行管理资源申请,包括Device、Context、Stream */
    aclrtSetDevice(deviceId_);
    aclrtCreateContext(&context_, deviceId_);
    aclrtCreateStream(&stream_);

    /* 3.初始化参数：原图宽高，crop宽高。初始化文件夹：输出结果文件夹*/
    Initparam(argc, argv);
    const int outputWidth = outPicDesc.width; // cur model shape is 224 * 224
    const int outputHeight = outPicDesc.height;

	/* 4. 创建图片数据处理通道时的通道描述信息，dvppChannelDesc_是acldvppChannelDesc类型*/
    dvppChannelDesc_ = acldvppCreateChannelDesc();

    /* 5. 创建图片数据处理的通道。*/
    acldvppCreateChannel(dvppChannelDesc_);

    // GetPicDevBuffer4JpegD
    uint32_t devPicBufferSize;
    uint32_t inputHostBuffSize = 0;
    char* inputHostBuff = ReadInputFile(inPicDesc.picName, inputHostBuffSize);
    void *inBufferDev = nullptr;
    inBufferSize = inputHostBuffSize;
    acldvppMalloc(&inBufferDev, inBufferSize);
    aclrtMemcpy(inBufferDev, inBufferSize, inputHostBuff, inputHostBuffSize, ACL_MEMCPY_HOST_TO_DEVICE);
    delete[] inputHostBuff;
    devPicBufferSize = inBufferSize;
    //char *picDevBuffer = reinterpret_cast<char *>(inBufferDev);

    acldvppRoiConfig *cropArea_ = nullptr;
    uint32_t midNum = 2;
    uint32_t oddNum = 1;
    uint32_t cropSizeWidth = 200;
    uint32_t cropSizeHeight = 200;
    uint32_t cropLeftOffset = 512;  // must even
    uint32_t cropRightOffset = cropLeftOffset + cropSizeWidth - oddNum;  // must odd
    uint32_t cropTopOffset = 512;  // must even
    uint32_t cropBottomOffset = cropTopOffset + cropSizeHeight - oddNum;  // must odd
    cropArea_ = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset,
        cropTopOffset, cropBottomOffset);
	
	acldvppRoiConfig *pasteArea_ = nullptr;
    uint32_t pasteLeftOffset = 16;  // must even
    uint32_t pasteRightOffset = pasteLeftOffset + cropSizeWidth - oddNum;  // must odd
    uint32_t pasteTopOffset = 0;  // must even
    uint32_t pasteBottomOffset = pasteTopOffset + cropSizeHeight - oddNum;  // must odd
    pasteArea_ = acldvppCreateRoiConfig(pasteLeftOffset, pasteRightOffset,
        pasteTopOffset, pasteBottomOffset);

    uint32_t widthAlignment = 16;
    uint32_t heightAlignment = 2;
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;
	uint32_t inputWidth = inPicDesc.width;
	uint32_t inputHeight = inPicDesc.height;
    uint32_t jpegOutWidthStride = AlignmentHelper(inputWidth, widthAlignment);
    uint32_t jpegOutHeightStride = AlignmentHelper(inputHeight, heightAlignment);
    uint32_t jpegOutBufferSize = jpegOutWidthStride * jpegOutHeightStride * sizeAlignment / sizeNum;
    acldvppPicDesc *vpcInputDesc_ = nullptr;
    acldvppPicDesc *vpcOutputDesc_ = nullptr;
    vpcInputDesc_ = acldvppCreatePicDesc();
    acldvppSetPicDescData(vpcInputDesc_, inBufferDev); // JpegD -> vpcCropAndPaste
    acldvppSetPicDescFormat(vpcInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcInputDesc_, inputWidth);
    acldvppSetPicDescHeight(vpcInputDesc_, inputHeight);
    acldvppSetPicDescWidthStride(vpcInputDesc_, jpegOutWidthStride);
    acldvppSetPicDescHeightStride(vpcInputDesc_, jpegOutHeightStride);
    acldvppSetPicDescSize(vpcInputDesc_, jpegOutBufferSize);
	
    void *vpcOutBufferDev_ = nullptr;
    int dvppOutWidth = outputWidth;
    int dvppOutHeight = outputHeight;
    int dvppOutWidthStride = AlignmentHelper(outputWidth, widthAlignment);;
    int dvppOutHeightStride = AlignmentHelper(outputHeight, heightAlignment);;
    uint32_t vpcOutBufferSize_ =
        dvppOutWidthStride * dvppOutHeightStride * sizeAlignment / sizeNum;
    aclError aclRet = acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    vpcOutputDesc_ = acldvppCreatePicDesc();
	INFO_LOG("acldvppCreatePicDesc w=%d/%d,h=%d/%d,vpcOutBufferSize=%d",dvppOutWidth,dvppOutWidthStride,dvppOutHeight,dvppOutHeightStride,vpcOutBufferSize_);
    acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcOutputDesc_, dvppOutWidth);
    acldvppSetPicDescHeight(vpcOutputDesc_, dvppOutHeight);
    acldvppSetPicDescWidthStride(vpcOutputDesc_, dvppOutWidthStride);
    acldvppSetPicDescHeightStride(vpcOutputDesc_, dvppOutHeightStride);
    acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);

    // crop and patse pic
    acldvppVpcCropAndPasteAsync(dvppChannelDesc_, vpcInputDesc_,
        vpcOutputDesc_, cropArea_, pasteArea_, stream_);
    aclrtSynchronizeStream(stream_);

	(void)acldvppDestroyRoiConfig(cropArea_);
	cropArea_ = nullptr;
	(void)acldvppDestroyRoiConfig(pasteArea_);
	pasteArea_ = nullptr;
	(void)acldvppDestroyPicDesc(vpcInputDesc_);
	vpcInputDesc_ = nullptr;
	(void)acldvppDestroyPicDesc(vpcOutputDesc_);
	vpcOutputDesc_ = nullptr;
	(void)acldvppDestroyChannel(dvppChannelDesc_);
	(void)acldvppDestroyChannelDesc(dvppChannelDesc_);
	dvppChannelDesc_ = nullptr;

    SaveOutputFile(outPicDesc.picName.c_str(), vpcOutBufferDev_, vpcOutBufferSize_);
    if (vpcOutBufferDev_ != nullptr) {
        (void)acldvppFree(vpcOutBufferDev_);
        vpcOutBufferDev_ = nullptr;
    }
	aclrtDestroyStream(stream_);
	stream_ = nullptr;
	aclrtDestroyContext(context_);
	context_ = nullptr;
    aclrtResetDevice(deviceId_);
    INFO_LOG("end to reset device is %d", deviceId_);

    aclFinalize();
    INFO_LOG("end to finalize acl");


    return SUCCESS;
}
