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

uint32_t alignment_helper(uint32_t origSize, uint32_t alignment)
{
    if (alignment == 0) {
        return 0;
    }
    uint32_t alignmentH = alignment - 1;
    return (origSize + alignmentH) / alignment * alignment;
}

uint32_t compute_encode_inputsize(int inputWidth, int inputHeight)
{
    uint32_t widthAlignment = 16;
    uint32_t heightAlignment = 2;
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;
    uint32_t encodeInWidthStride = alignment_helper(inputWidth, widthAlignment);
    uint32_t encodeInHeightStride = alignment_helper(inputHeight, heightAlignment);
    if (encodeInWidthStride == 0 || encodeInHeightStride == 0) {
        ERROR_LOG("ComputeEncodeInputSize AlignmentHelper failed");
        return FAILED;
    }
    uint32_t encodeInBufferSize =
    encodeInWidthStride * encodeInHeightStride * sizeAlignment / sizeNum;
    return encodeInBufferSize;
}

char* get_picdevbuffer4_jpege(const PicDesc &picDesc, uint32_t &PicBufferSize)
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
    if (aclRet !=  ACL_ERROR_NONE) {
        delete[] inputBuff;
        ERROR_LOG("malloc device data buffer failed, aclRet is %d", aclRet);
        fclose(fp);
        return nullptr;
    }
    if (runMode == ACL_HOST) {
        aclRet = aclrtMemcpy(inputDevBuff, PicBufferSize, inputBuff, PicBufferSize, ACL_MEMCPY_HOST_TO_DEVICE);
    }
    else {
        aclRet = aclrtMemcpy(inputDevBuff, PicBufferSize, inputBuff, PicBufferSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
    }
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("memcpy from host to device failed, aclRet is %d", aclRet);
        (void)acldvppFree(inputDevBuff);
        delete[] inputBuff;
        fclose(fp);
        return nullptr;
    }

    fclose(fp);
    return reinterpret_cast<char *>(inputDevBuff);
}

void set_input4_jpege(char &inDevBuffer, int inDevBufferSize, int inputWidth, int inputHeight)
{
    inDevBuffer_ = &inDevBuffer;
    in_devbuffer_size_encode_ = inDevBufferSize;
    inputWidth_ = inputWidth;
    inputHeight_ = inputHeight;
}

Result save_dvpp_outputdata(const char *fileName, const void *devPtr, uint32_t dataSize)
{
    FILE * outFileFp = fopen(fileName, "wb+");
    if (nullptr == outFileFp) {
        ERROR_LOG("fopen out file %s failed.", fileName);
        return FAILED;
    }
    if (runMode == ACL_HOST) {
        void* hostPtr = nullptr;
        aclError aclRet = aclrtMallocHost(&hostPtr, dataSize);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("malloc host data buffer failed, aclRet is %d", aclRet);
            fclose(outFileFp);
            return FAILED;
        }

        aclRet = aclrtMemcpy(hostPtr, dataSize, devPtr, dataSize, ACL_MEMCPY_DEVICE_TO_HOST);
        if (aclRet != ACL_ERROR_NONE) {
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

void destroy_resource()
{
    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy stream failed");
        }
        stream_ = nullptr;
    }
    INFO_LOG("End to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy context failed");
        }
        context_ = nullptr;
    }
    INFO_LOG("End to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("End to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("End to finalize acl");
}

void destroy_encode_resource()
{
    if (jpegeConfig_ != nullptr) {
        (void)acldvppDestroyJpegeConfig(jpegeConfig_);
        jpegeConfig_ = nullptr;
    }
    INFO_LOG("Call acldvppDestroyJpegeConfig success");
    if (encodeInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(encodeInputDesc_);
        encodeInputDesc_ = nullptr;
    }
    INFO_LOG("Call acldvppDestroyPicDesc success");
    if (inDevBuffer_ != nullptr) {
        (void)acldvppFree(inDevBuffer_);
        inDevBuffer_ = nullptr;
    }
    INFO_LOG("Call acldvppFree success");
}

int main()
{
    //1.ACL初始化
    const char *aclConfigPath = "../src/acl.json";
    aclInit(aclConfigPath);
    INFO_LOG("Acl init success");
    //2.运行管理资源申请,包括Device、Context、Stream，stream_是aclrtStream类型
    aclrtSetDevice(deviceId_);
    INFO_LOG("Open device %d success", deviceId_);
    aclrtCreateContext(&context_, deviceId_);
    INFO_LOG("Create context success");
    aclrtCreateStream(&stream_);
    INFO_LOG("Create stream success");
    aclrtGetRunMode(&runMode);

    std::string encodeOutFileName = "./output/jpege_output";
    uint32_t encodeLevel = 100; // default optimal level (0-100)
    PicDesc testPic = {"../data/dvpp_output.yuv", 1024, 688};
    INFO_LOG("Start to process picture:%s", testPic.picName.c_str());
    INFO_LOG("Call JpegE");

    DIR *dir;
    if ((dir = opendir("./output")) == NULL){
        system("mkdir ./output");
    }
        
    //4.创建图片数据处理的通道
    dvppChannelDesc_ = acldvppCreateChannelDesc();
    INFO_LOG("Call acldvppCreateChannelDesc success");
    acldvppCreateChannel(dvppChannelDesc_);
    INFO_LOG("Call acldvppCreateChannel success");
    INFO_LOG("DVPP init resource success");

    uint32_t jpegInBufferSize;
    jpegInBufferSize = compute_encode_inputsize(testPic.width, testPic.height);
    //5.申请内存
    //5.1 输入内存
    //申请Host内存inputHostBuff，存放YUV格式的图片数据
    //申请Device内存inputDevBuff
    //将通过aclrtMemcpy接口将Host的图片数据传输到Device，数据传输完成后，需及时调用aclrtFreeHost接口释放Host内存
    char* picDevBuffer = get_picdevbuffer4_jpege(testPic, jpegInBufferSize);
    if (nullptr == picDevBuffer) {
        ERROR_LOG("get picDevBuffer failed, index is %d", 0);
        return FAILED;
    }
    set_input4_jpege(*picDevBuffer, jpegInBufferSize, testPic.width, testPic.height);
    //6. 创建编码输入图片的描述信息，并设置各属性值
    //encodeInputDesc_是acldvppPicDesc类型
    uint32_t widthAlignment = 16;
    uint32_t heightAlignment = 2;
    uint32_t encodeInWidthStride = alignment_helper(inputWidth_, widthAlignment);
    uint32_t encodeInHeightStride = alignment_helper(inputHeight_, heightAlignment);
    if (encodeInWidthStride == 0 || encodeInHeightStride == 0) {
        ERROR_LOG("InitEncodeInputDesc AlignmentHelper failed");
        return FAILED;
    }
    encodeInputDesc_ = acldvppCreatePicDesc();
    INFO_LOG("Call acldvppCreatePicDesc success");
    if (encodeInputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc encodeInputDesc_ failed");
        return FAILED;
    }

    acldvppSetPicDescData(encodeInputDesc_, reinterpret_cast<void *>(inDevBuffer_));
    acldvppSetPicDescFormat(encodeInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(encodeInputDesc_, inputWidth_);
    acldvppSetPicDescHeight(encodeInputDesc_, inputHeight_);
    acldvppSetPicDescWidthStride(encodeInputDesc_, encodeInWidthStride);
    acldvppSetPicDescHeightStride(encodeInputDesc_, encodeInHeightStride);
    acldvppSetPicDescSize(encodeInputDesc_, in_devbuffer_size_encode_);

    jpegeConfig_ = acldvppCreateJpegeConfig();
    INFO_LOG("Call acldvppCreateJpegeConfig success");
    acldvppSetJpegeConfigLevel(jpegeConfig_, encodeLevel);

    //5.2 输出内存，申请Device内存encodeOutBufferDev_,存放编码后的输出数据
    //  uint32_t outBufferSize = jpegInBufferSize + jpegInBufferSize; // malloc enough size

    acldvppJpegPredictEncSize(encodeInputDesc_, jpegeConfig_, &encode_outbuffer_size_);
    aclError aclRet = acldvppMalloc(&encode_out_buffer_dev_, encode_outbuffer_size_);

    //aclError aclRet = acldvppMalloc(&encodeOutBufferDev_, outBufferSize);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("malloc encodeOutBufferDev_ failed, aclRet is %d", aclRet);
        return FAILED;
    }

    //8. 执行异步编码，再调用aclrtSynchronizeStream接口阻塞Host运行，直到指定Stream中的所有任务都完成
    aclRet = acldvppJpegEncodeAsync(dvppChannelDesc_, encodeInputDesc_, encode_out_buffer_dev_,
    &encode_outbuffer_size_, jpegeConfig_, stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppJpegEncodeAsync failed, aclRet = %d", aclRet);
        return FAILED;
    }
    INFO_LOG("Call acldvppJpegEncodeAsync success");
    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("encode aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return FAILED;
    }

    //9.申请Host内存hostPtr，将编码后的输出图片回传到Host，再将Host内存中的数据写入文件,写完文件后，需及时调用aclrtFreeHost接口释放Host内存
    encodeOutFileName = encodeOutFileName + ".jpg";
    Result ret = save_dvpp_outputdata(encodeOutFileName.c_str(), encode_out_buffer_dev_, encode_outbuffer_size_);
    if (ret != SUCCESS) {
        ERROR_LOG("save encode output data failed.");
        return FAILED;
    }
    destroy_encode_resource();

    destroy_resource();

}
